// Part of the ODKI VBT Firmware
// Copyright (C) 2026 Lodovico Cortelazzo
//
// Licensed under the GNU General Public License, Version 3 (this
// specific version only, not "or any later version"), modified by the
// Commons Clause License Condition v1.0 -- see LICENSE-FIRMWARE in the
// repository root for the full text of both. In short: you may use,
// study, modify, and share this file (including a modified version) for
// non-commercial purposes; you may not sell it, or a product/service
// substantially derived from it, without a separate agreement with the
// copyright holder.
//
// This program is distributed WITHOUT ANY WARRANTY, without even the
// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
// PURPOSE -- see LICENSE-FIRMWARE for details.

#include "BatteryMonitor.h"
#include <InternalFileSystem.h>

using namespace Adafruit_LittleFS_Namespace;

namespace {
  const float VBAT_MV_PER_LSB   = 0.73242188f; // 3.0V full scale (AR_INTERNAL_3_0), 12-bit ADC -> 3000/4096
  // VBAT divider compensation. Retuned after fixing analogSampleTime()
  // (the previous calibration, 2.6953, was partly compensating for the
  // ADC's systematic bias due to insufficient TACQ, not just the
  // divider's true ratio - no longer valid once TACQ was corrected). New
  // calibration: firmware was reading 3.78V with the battery reported at
  // 100% (~4.20V, the full-charge point of the curve in
  // voltageToPercent) -> 2.6953 * (4.20/3.78) = 2.9948. This remains the
  // factory/baseline value shared by ALL devices: the AUTO-CALIBRATION
  // below then corrects the residual, per-board-SPECIFIC offset (divider
  // component tolerance), so it shouldn't need manual retuning from here
  // on - see below.
  const float VBAT_DIVIDER_COMP = 3.055918f;  //2.9948f;

  const int OVERSAMPLE_COUNT = 8; // average multiple ADC readings to reduce converter noise

  // Exponential moving average between one reading and the next (calls
  // ~30s apart, see Config::BATTERY_CHECK_INTERVAL_MS): smooths out
  // momentary voltage dips under load (e.g. a BLE radio burst) without
  // making the value too slow to follow a real charge-state change.
  // Lower alpha = more stable but slower to track the true value.
  const float EMA_ALPHA = 0.3f;
  float smoothedVoltage = -1.0f; // -1 = no reading taken yet

  // ============================================================================
  // Per-device AUTO-CALIBRATION, using the charge LED as a known
  // reference.
  //
  // The XIAO nRF52840 Sense's charge chip (BQ25100) exposes its status on
  // D23 (P0.17, named "~CHG" in the board package's variant.cpp) - the
  // SAME signal that lights the onboard charge LED: active LOW (LOW =
  // charging in progress, LED on; released/HIGH = not charging - either
  // because charging is COMPLETE, or because USB/battery power is
  // missing). Confirmed by the official Seeed pin map
  // (wiki.seeedstudio.com/XIAO_BLE, "CHARGE_LED -> P0.17") and by the
  // board package's variant.cpp ("D23 is P0.17 (~CHG)").
  //
  // ~CHG alone isn't enough to distinguish "charging completed" from "USB
  // disconnected" (in BOTH cases the pin goes HIGH) - so we also need
  // NRF_POWER->USBREGSTATUS (VBUSDETECT), an nRF52840 hardware register
  // that reflects the presence of USB power independently of the charge
  // chip: if VBUS is still present when ~CHG transitions from active to
  // released, charging really did finish (nothing was unplugged).
  // Already used successfully in this exact same way in Bluefruit52Lib
  // (bluefruit.cpp).
  //
  // At the moment of that transition, the battery is by definition at the
  // BQ25100's end-of-charge voltage (4.20V standard for a single
  // LiPo/Li-ion cell, consistent with the curve in voltageToPercent
  // below) - a known and reliable reference, better than any one-off
  // manual calibration: a MULTIPLICATIVE correction factor (calFactor,
  // applied on top of VBAT_DIVIDER_COMP) is then recomputed to bring the
  // current reading to exactly 4.20V, and it's persisted to flash
  // (InternalFS/LittleFS) so it survives the full reset that every deep
  // sleep causes (see PowerManager::goToSleep()) - without persistence
  // the calibration learned during charging would always be lost before
  // the next training session, defeating its purpose.
  //
  // Guardrails against a spurious event (pin noise, a request during a
  // brief USB reconnect blip, etc.):
  //   - CHARGING_CONFIRM_CHECKS: charging must have been detected active
  //     for AT LEAST this many consecutive readings (spaced ~30s apart)
  //     before its conclusion is considered trustworthy.
  //   - CAL_FACTOR_MIN/MAX: the resulting factor must stay within +-15% -
  //     beyond that it's almost certainly a hardware issue/anomalous
  //     reading, not a genuine divider tolerance offset; in that case the
  //     calibration is discarded (not saved) instead of corrupting the
  //     good value already persisted.
  // ============================================================================
  const uint8_t PIN_CHG_STATUS = 23; // D23 = P0.17 = BQ25100's ~CHG, see above

  const char* CAL_FILE_PATH = "/battcal.dat";
  const uint32_t CAL_FILE_MAGIC = 0x42435631UL; // "BCV1", verifies the file isn't empty/a different format
  struct PersistedCal {
    uint32_t magic;
    float calFactor;
  };

  const float FULL_CHARGE_VOLTAGE = 4.20f;
  const float CAL_FACTOR_MIN = 0.85f;
  const float CAL_FACTOR_MAX = 1.15f;
  const uint8_t CHARGING_CONFIRM_CHECKS = 3; // ~90s+ of confirmed charging before trusting its conclusion

  float calFactor = 1.0f; // 1.0 = no correction learned yet, starting from VBAT_DIVIDER_COMP alone
  bool wasChgActive = false;
  uint8_t chargingConfirmedChecks = 0;

  void loadCalFactor() {
    InternalFS.begin();
    File f(InternalFS);
    if (f.open(CAL_FILE_PATH, FILE_O_READ)) {
      PersistedCal pc;
      bool ok = (f.read(&pc, sizeof(pc)) == (int)sizeof(pc)) &&
                (pc.magic == CAL_FILE_MAGIC) &&
                (pc.calFactor >= CAL_FACTOR_MIN) && (pc.calFactor <= CAL_FACTOR_MAX);
      f.close();
      if (ok) {
        calFactor = pc.calFactor;
        Serial.print("BatteryMonitor: calibration factor loaded from flash: ");
        Serial.println(calFactor, 4);
      }
    }
  }

  void saveCalFactor(float newFactor) {
    File f(InternalFS);
    if (f.open(CAL_FILE_PATH, FILE_O_WRITE)) {
      PersistedCal pc;
      pc.magic = CAL_FILE_MAGIC;
      pc.calFactor = newFactor;
      f.write((const uint8_t*)&pc, sizeof(pc));
      f.close();
    } else {
      Serial.println("BatteryMonitor: ERROR, unable to save calibration to flash.");
    }
  }

  // Updates calFactor if this sample marks the end of a real charging
  // cycle (see the AUTO-CALIBRATION comment above) - called by
  // readVoltage() on every reading (~30s, see
  // Config::BATTERY_CHECK_INTERVAL_MS), with smoothedVoltage already
  // updated with the current reading.
  void updateAutoCalibration() {
    bool chgActive = BatteryMonitor::isCharging();
    bool vbusPresent = (NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;

    if (chgActive && chargingConfirmedChecks < 255) {
      chargingConfirmedChecks++;
    }

    if (wasChgActive && !chgActive && vbusPresent &&
        chargingConfirmedChecks >= CHARGING_CONFIRM_CHECKS && smoothedVoltage > 0.5f) {
      float candidateFactor = calFactor * (FULL_CHARGE_VOLTAGE / smoothedVoltage);
      if (candidateFactor >= CAL_FACTOR_MIN && candidateFactor <= CAL_FACTOR_MAX) {
        calFactor = candidateFactor;
        saveCalFactor(calFactor);
        // Reflect the new calibration immediately, without waiting for
        // the EMA (tuned to smooth out noise, not a deliberate jump) to
        // reach it on its own over some future readings - we've just
        // been told by the hardware itself that THIS is the full-charge
        // voltage.
        smoothedVoltage = FULL_CHARGE_VOLTAGE;
        Serial.print("BatteryMonitor: charging complete (~CHG released, USB still present), "
                      "auto-calibration applied. New factor: ");
        Serial.println(calFactor, 4);
      } else {
        Serial.print("BatteryMonitor: automatic calibration discarded (factor ");
        Serial.print(candidateFactor, 4);
        Serial.println(" outside the plausible +-15% range).");
      }
    }

    if (!chgActive) {
      chargingConfirmedChecks = 0;
    }
    wasChgActive = chgActive;
  }
}

void BatteryMonitor::begin() {
  pinMode(VBAT_ENABLE, OUTPUT);
  digitalWrite(VBAT_ENABLE, HIGH); // divider disconnected until a reading is needed
  analogReference(AR_INTERNAL_3_0);
  analogReadResolution(12);

  // The VBAT divider has very high impedance (~350k ohms effective, see
  // VBAT_DIVIDER_COMP). The SAADC's default acquisition time (3us) is
  // only enough for low-impedance sources: with a source this "slow" the
  // internal sampling capacitor doesn't have time to charge to the
  // node's true voltage, producing a systematic bias (not random noise -
  // which is why oversampling/EMA alone weren't enough). 40us (the
  // maximum allowed by this core) gives ample margin for a source of
  // this impedance.
  analogSampleTime(40);

  // No known external pull-up on ~CHG (see the AUTO-CALIBRATION comment
  // above): INPUT_PULLUP still guarantees a defined HIGH when the
  // BQ25100 releases it (open-drain), instead of leaving it floating.
  pinMode(PIN_CHG_STATUS, INPUT_PULLUP);

  loadCalFactor();
}

bool BatteryMonitor::isCharging() {
  return digitalRead(PIN_CHG_STATUS) == LOW; // ~CHG active low, see the AUTO-CALIBRATION comment above
}

// Re-enables the divider only for the duration of the reading, then
// disconnects it again (protects the ADC pin and reduces the divider's
// standby power draw). The returned voltage is oversampled (average of
// several closely-spaced ADC samples), filtered over time (EMA against
// the previous reading) and corrected by the per-device auto-calibration
// factor (calFactor, see the AUTO-CALIBRATION comment above).
float BatteryMonitor::readVoltage() {
  digitalWrite(VBAT_ENABLE, LOW);
  delay(1); // let the divider settle after enabling

  uint32_t rawSum = 0;
  for (int i = 0; i < OVERSAMPLE_COUNT; i++) {
    rawSum += analogRead(PIN_VBAT);
  }
  float rawAvg = (float)rawSum / OVERSAMPLE_COUNT;

  digitalWrite(VBAT_ENABLE, HIGH);

  float instantVoltage = (rawAvg * VBAT_MV_PER_LSB * VBAT_DIVIDER_COMP * calFactor) / 1000.0f;

  if (smoothedVoltage < 0.0f) {
    smoothedVoltage = instantVoltage; // very first reading ever: nothing to smooth yet
  } else {
    smoothedVoltage = EMA_ALPHA * instantVoltage + (1.0f - EMA_ALPHA) * smoothedVoltage;
  }

  updateAutoCalibration();

  return smoothedVoltage;
}

// Simplified two-segment LiPo curve (0% at 3.30V, 10% at 3.60V, then
// linear up to about 100% at 4.20V).
float BatteryMonitor::voltageToPercent(float volts) {
  float mvolts = volts * 1000.0f;
  if (mvolts < 3300.0f) return 0.0f;
  if (mvolts < 3600.0f) {
    return (mvolts - 3300.0f) / 30.0f;
  }
  float pct = 10.0f + (mvolts - 3600.0f) * 0.15f;
  return pct > 100.0f ? 100.0f : pct;
}
