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

/*
  VBT Rep Counter & Velocity Estimator - v3 (BLE)
  Target: Seeed XIAO nRF52840 Sense (integrated LSM6DS3TR-C IMU)

  The code is organized into modules (one .h/.cpp file per responsibility);
  this .ino contains only the high-level orchestration:

    Config.h          - pins and shared constants
    StatusLED         - RGB LED, recomputed on every loop() from the
                         current state (see StatusLED.h for the color table)
    BatteryMonitor     - reading/percentage of the 3.7V 360mAh LiPo
    PowerManager       - power switch + deep sleep (System OFF)
    MadgwickAHRS       - orientation filter (quaternion)
    MotionTracker      - IMU reading, world-frame rotation, velocity/
                         position integration, phase/rep identification (see MotionTracker.h)
    BleServer          - custom BLE service (streaming + commands), PERIPHERAL role

  Required library: "Seeed Arduino LSM6DS3" by Seeed Studio (the BLE
  library, Bluefruit52Lib, is already included in the Seeeduino:nrf52
  board package).

  Power switch: connect a switch between WAKE_PIN (D6, see Config.h) and
  GND. Switch closed = on; switch open = deep sleep, wakes with a full
  reset when closed again. BLE (Bluefruit.begin()) must be started BEFORE
  any possible call to goToSleep(): deep sleep uses
  sd_power_system_off(), which requires the SoftDevice to already be
  active.

  BLE flow (see BleServer.h for the exact packet layout):
  - SystemStatus can be read at any time with a standard GATT Read (in
    addition to the automatic push via Notify on every connection and on
    every state change).
  - Orientation calibration must be explicitly requested with the
    CALIBRATE (0x02) command when SystemStatus reports calibrated=false.
    START (0x01) is ignored if the system isn't calibrated yet; CALIBRATE
    is ignored if tracking is active (it must be stopped first with STOP,
    0x00).
  - While tracking is active: fast decimated streaming (body-frame and
    world-frame acceleration, RAW velocity and world-frame position on
    all 3 axes, quaternion - v3.11.0: Z velocity is never corrected in
    real time, see CorrectedCurve below) + a RepSummary packet (Notify,
    without a GATT ack - see BleServer::sendRepSummary()) on every valid
    completed rep (concentric plus any preceding eccentric phase, see
    RuntimeConfig::repDirection and MotionTracker.h/.cpp for the
    phase/bracket algorithm). v3.11.0: the SAME rep can be re-sent with
    an improved RepSummaryPacket::correctionStatus when the bracket
    containing it closes - the client overwrites by repNumber. In
    parallel, on every bracket close one or more CorrectedCurve (Notify)
    blocks arrive with the CORRECTED velocity for the time interval
    already transmitted via Stream as raw - see BleServer.h.
    Phases that are too short (likely bounce/noise) are discarded
    internally and never reported.
    Tracking continues even through a BLE disconnect/reconnect: it is
    never automatically stopped on the firmware side.
  - Config (Read/Write): algorithm parameters adjustable at runtime (rep
    direction Up/Down, anti-bounce, phase thresholds - see RuntimeConfig
    in MotionTracker.h) - consumed below with
    BleServer::takeConfigUpdate()/MotionTracker::setConfig(). NOT
    persisted to flash: every reboot starts again from the compiled
    defaults.
  - Every 30 seconds, independent of tracking: battery status (standard
    Battery Service, see BleServer.h).
*/

#include <Adafruit_TinyUSB.h>
#include "Config.h"
#include "StatusLED.h"
#include "BatteryMonitor.h"
#include "PowerManager.h"
#include "MotionTracker.h"
#include "BleServer.h"

unsigned long lastBatteryCheckTime = 0;
uint8_t streamDecimationCounter = 0;

void setup() {
  PowerManager::begin();
  StatusLED::begin();
  BatteryMonitor::begin();

  Serial.begin(115200);
  while (!Serial && millis() < 3000) { }

  // TEMPORARY DIAGNOSTICS to isolate the wake-from-sleep issue: remove
  // once resolved. The DOG case would confirm (or, if it never appears
  // again, rule out) the hypothesis of a firmware hang saved by the
  // watchdog that was just added (see PowerManager.h).
  Serial.print("RESETREAS = 0x");
  Serial.print(PowerManager::lastResetReason(), HEX);
  if (PowerManager::lastResetReason() & POWER_RESETREAS_OFF_Msk) {
    Serial.println(" -> wake from System OFF via GPIO (SENSE/DETECT) OK");
  } else if (PowerManager::lastResetReason() & POWER_RESETREAS_DOG_Msk) {
    Serial.println(" -> WATCHDOG RESET: the firmware was stuck (hang/crash) and self-recovered");
  } else if (PowerManager::lastResetReason() & POWER_RESETREAS_LOCKUP_Msk) {
    Serial.println(" -> CPU LOCKUP RESET: unhandled hard fault");
  } else if (PowerManager::lastResetReason() == 0) {
    Serial.println(" -> true POWER-ON RESET (power reconnected from scratch, not a wake from sleep)");
  } else {
    Serial.println(" -> other reason (see the comment in PowerManager.cpp for the bit meanings)");
  }

  // BLE must be started BEFORE any possible goToSleep():
  // sd_power_system_off() requires the SoftDevice to already be active
  // to work reliably.
  BleServer::begin();
  BleServer::updateSystemStatus(false, false);

  // v3.11.6: take the first battery reading IMMEDIATELY, without waiting
  // for the first periodic cycle (Config::BATTERY_CHECK_INTERVAL_MS,
  // 30s) - without this, the Battery Level characteristic (BLEBas) stays
  // at whatever default value the SoftDevice assigns it when it's
  // registered (BLEBas::begin() never writes an initial value, see
  // Bluefruit52Lib/src/services/BLEBas.cpp) until the first real
  // sendBattery() - if a client connects and reads it within that 30s
  // window (typical right after power-on), it reads nonsensical
  // percentages (any value 0-255, so both "0%" and "over 200%" are
  // possible) instead of an explicit error. Reported by the user: "as
  // soon as I connect the device the battery shows absurd values".
  BleServer::sendBattery(BatteryMonitor::voltageToPercent(BatteryMonitor::readVoltage()));

  // If the switch is NOT closed right at startup, go back to sleep immediately
  if (!PowerManager::wakeSwitchClosed()) {
    PowerManager::goToSleep();
  }

  if (!MotionTracker::begin()) {
    Serial.println("ERROR: IMU not initialized.");
    while (1) {
      // The LED keeps reflecting connection/state even in error
      // (blinks yellow if not connected, solid yellow if connected).
      StatusLED::update(BleServer::connected(), false, false);
      delay(100);
    }
  }

  lastBatteryCheckTime = millis();
}

void loop() {
  // Feed the watchdog (see PowerManager.h): must be called on every
  // iteration, first thing, so that ANY block further down in the loop
  // (including one inside goToSleep() itself, if the switch is open)
  // remains uncovered and leads to the automatic reset within the
  // timeout.
  PowerManager::feedWatchdog();

  // Switch open -> go back to sleep
  if (!PowerManager::wakeSwitchClosed()) {
    PowerManager::goToSleep();
  }

  // --- LED: recomputed every cycle from the current state (see StatusLED.h) ---
  StatusLED::update(BleServer::connected(), MotionTracker::isCalibrated(), MotionTracker::isTrackingActive());

  // --- Commands received via BLE ---
  if (BleServer::takeCalibrateRequested()) {
    if (MotionTracker::isTrackingActive()) {
      Serial.println("CALIBRATE ignored: stop tracking (STOP) before recalibrating.");
    } else {
      MotionTracker::calibrateOrientation();
      BleServer::updateSystemStatus(true, false);
      Serial.println("Calibration complete.");
    }
  }
  if (BleServer::takeStartRequested()) {
    if (!MotionTracker::isCalibrated()) {
      Serial.println("START ignored: calibrate the device first (CALIBRATE command).");
    } else {
      MotionTracker::setTrackingActive(true);
      BleServer::updateSystemStatus(true, true);
    }
  }
  if (BleServer::takeStopRequested()) {
    MotionTracker::setTrackingActive(false);
    BleServer::updateSystemStatus(MotionTracker::isCalibrated(), false);
  }
  RuntimeConfig newConfig;
  if (BleServer::takeConfigUpdate(newConfig)) {
    MotionTracker::setConfig(newConfig);
    Serial.println("MotionTracker: new configuration applied via BLE.");
  }

  // --- Battery status, every 30 seconds (independent of tracking) ---
  unsigned long now = millis();
  if (now - lastBatteryCheckTime >= Config::BATTERY_CHECK_INTERVAL_MS) {
    lastBatteryCheckTime = now;
    float vbat = BatteryMonitor::readVoltage();
    float pct = BatteryMonitor::voltageToPercent(vbat);
    Serial.print("Battery: ");
    Serial.print(vbat, 2);
    Serial.print(" V (");
    Serial.print(pct, 0);
    Serial.print("%)");
    Serial.println(BatteryMonitor::isCharging() ? " [charging]" : "");
    BleServer::sendBattery(pct);
  }

  // --- Orientation/motion: always updated (100Hz internal) ---
  // update() self-limits to 100Hz and returns true ONLY when it has
  // computed a real sample: anything that must happen "once per sample"
  // (BLE streaming decimation) must be gated on this, otherwise loop()
  // (which runs much faster than 100Hz) would repeat the same stale
  // sample multiple times between one real update and the next.
  bool newSample = MotionTracker::update();

  if (newSample && MotionTracker::isTrackingActive()) {
    // End-of-rep data BEFORE streaming. v3.10.3: peekCompletedRep() does
    // NOT remove the rep from the queue - it's removed with
    // popCompletedRep() ONLY if sendRepSummary() returns true (notify
    // sent successfully). If it fails (BLE not yet ready to notify, or
    // momentarily disconnected), the rep stays in the queue and is
    // retried on the next sample - NEVER lost. Before this version,
    // indicate() was used (blocking GATT confirmation, WITHOUT a timeout
    // - see the RepSummary comment in BleServer.h), which in the worst
    // case blocked the entire loop() (and therefore also IMU sampling)
    // waiting for the client, the diagnosed cause of missing/delayed
    // reps reported by the user.
    RepResult rep;
    if (MotionTracker::peekCompletedRep(rep)) {
      if (BleServer::sendRepSummary(rep)) {
        MotionTracker::popCompletedRep();
      }
    }

    // Corrected curve blocks (v3.11.0, see CorrectedCurve in
    // BleServer.h) - same peek/pop scheme as RepSummary above: a block
    // stays in the queue until notify() is sent successfully, so no
    // piece of corrected curve is ever lost.
    CorrectedCurveChunk chunk;
    if (MotionTracker::peekCorrectedCurveChunk(chunk)) {
      if (BleServer::sendCorrectedCurveChunk(chunk)) {
        MotionTracker::popCorrectedCurveChunk();
      }
    }

    // Fast streaming, decimated relative to the internal 100Hz -
    // disabled while raw data logging over Serial is active
    // (RuntimeConfig::debugLogEnabled): that log is already at 100Hz (5x
    // denser than this 20Hz stream), so it would be redundant, and its
    // notify() would compete for the same loop() cycle right when we're
    // trying to get the cleanest possible capture (see the comment on
    // the flag's declaration, MotionTracker.h).
    streamDecimationCounter++;
    if (streamDecimationCounter >= Config::STREAM_DECIMATION_FACTOR) {
      streamDecimationCounter = 0;
      if (!MotionTracker::currentConfig().debugLogEnabled) {
        BleServer::sendStream(MotionTracker::currentSample());
      }
    }
  }
}
