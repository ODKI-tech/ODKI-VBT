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

#include "PowerManager.h"
#include "StatusLED.h"
#include "Config.h"
#include "nrf_gpio.h"
#include <bluefruit.h> // for sd_power_system_off()

namespace {
  uint32_t g_lastResetReas = 0;

  // Timeout chosen with a wide margin above the longest block the
  // firmware normally performs without ever calling feedWatchdog():
  //   - waiting on Serial in setup(): up to 3000ms
  //   - MotionTracker::calibrateOrientation(), called from loop() in
  //     response to the BLE CALIBRATE command: CALIBRATION_SAMPLES (200) *
  //     CALIBRATION_SAMPLE_DELAY_MS (10ms) = 2000ms blocking
  // 8000ms leaves more than 2x margin over the worst case measured
  // (full setup, typically 3-4s) without being so long that it defeats
  // the purpose of the safety net.
  const uint32_t WATCHDOG_TIMEOUT_MS = 8000;
}

uint32_t PowerManager::lastResetReason() {
  return g_lastResetReas;
}

void PowerManager::begin() {
  pinMode(WAKE_PIN, INPUT_PULLUP);

  // TEMPORARY DIAGNOSTICS: reason for the last reset (read and cleared as
  // early as possible, before any other module can alter it).
  // Relevant bits (NRF_POWER->RESETREAS):
  //   bit 0  RESETPIN - reset from the physical RESET pin
  //   bit 1  DOG      - watchdog (see below)
  //   bit 2  SREQ     - software reset requested
  //   bit 3  LOCKUP   - CPU lockup (unhandled hard fault crash)
  //   bit 16 OFF      - wake from System OFF via GPIO DETECT/SENSE
  //   (all bits 0 = a "true" Power-On Reset, e.g. from plugging in USB)
  g_lastResetReas = NRF_POWER->RESETREAS;
  NRF_POWER->RESETREAS = NRF_POWER->RESETREAS; // clear-on-write-1

  // --- Hardware watchdog (WDT), see the comment in PowerManager.h ---
  //
  // CONFIG.SLEEP = Run: the WDT keeps counting even while the CPU is in
  // WFI/idle (used internally by delay() and by the BLE stack between
  // events) - a true hang needs to be able to trigger the reset even if
  // it happens while the CPU is "asleep" in this sense (this is NOT the
  // System OFF of goToSleep(): that turns off the WDT itself, see below).
  //
  // CONFIG.HALT = Pause: while a debugger is holding the CPU stopped at a
  // breakpoint, the WDT does NOT keep counting. Convenient during
  // development (a prolonged breakpoint doesn't generate a spurious
  // reset); if in the future the watchdog needs to be "real" even under a
  // debugger, change this to Run.
  NRF_WDT->CONFIG = (WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos) |
                     (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos);

  // CRV is in ticks of the low-frequency clock (LFCLK, 32768 Hz).
  NRF_WDT->CRV = (WATCHDOG_TIMEOUT_MS * 32768UL) / 1000UL;

  // A single "reload register" (RR0): it's the only one that feedWatchdog() feeds.
  NRF_WDT->RREN = WDT_RREN_RR0_Msk;

  // Once the WDT is started it can no longer be stopped via software
  // (only a reset clears it) - this is intentional: if it could be
  // disabled, a firmware that gets stuck could remain stuck even with a
  // bug that mistakenly disables the very safety net meant to save it.
  //
  // IMPORTANT NOTE on the interaction with goToSleep(): the nRF52840's
  // WDT is NOT among the peripherals retained in System OFF - it powers
  // down along with everything else. There's therefore no need to
  // explicitly stop it before sd_power_system_off(): its count is
  // necessarily reset there. On wake (full reset, setup() re-run from
  // scratch) begin() automatically re-arms it from zero.
  NRF_WDT->TASKS_START = 1;
}

// Call on every loop() iteration (never inside wait/error loops, e.g. the
// "IMU not initialized" one in setup()): if the firmware gets stuck in
// there, or in a hang inside goToSleep() before sd_power_system_off()
// completes, the WDT does NOT get fed and expires on its own within
// WATCHDOG_TIMEOUT_MS, forcing a reset that gets the device going again
// without manual intervention. This is intentional: an error loop that
// "self-heals" via a periodic reset is an acceptable side effect (and in
// this case a desirable one: e.g. a transiently stuck IMU I2C bus can
// resolve itself on retry).
void PowerManager::feedWatchdog() {
  NRF_WDT->RR[0] = WDT_RR_RR_Reload;
}

bool PowerManager::wakeSwitchClosed() {
  return digitalRead(WAKE_PIN) == LOW;
}

void PowerManager::goToSleep() {
  StatusLED::off();

  Serial.println("Switch open: entering deep sleep...");
  Serial.flush();
  delay(50);

  // Configure WAKE_PIN as a sense input: wakes when it goes LOW
  // (i.e. when the switch connects it to GND)
  uint32_t pin = g_ADigitalPinMap[WAKE_PIN];
  nrf_gpio_cfg_sense_input(pin, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);

  // TEMPORARY DIAGNOSTICS: verify that the WAKE_PIN's PIN_CNF register
  // really has SENSE=LOW (expected: bits[17:16] = 0b11) and INPUT enabled
  // (bit1 = 0). If this prints a value different from what's expected,
  // the problem is in the pin's software configuration, not in the switch.
  Serial.print("WAKE_PIN mapped to nRF pin #");
  Serial.print(pin);
  Serial.print(", PIN_CNF = 0x");
  Serial.println(NRF_GPIO->PIN_CNF[pin], HEX);
  Serial.flush();

  // System OFF: lowest possible power consumption on the nRF52840.
  // Waking up restarts from a full reset (setup() re-run).
  //
  // NOTE: with the BLE SoftDevice active, writing directly to
  // NRF_POWER->SYSTEMOFF is blocked/ignored non-deterministically (the
  // SoftDevice protects that register) - this was the cause of the bug
  // where the device sometimes failed to go to sleep. The correct and
  // safe call with the SoftDevice active is sd_power_system_off(). It
  // requires that Bluefruit.begin() has already been called (see the
  // ordering in setup()).
  sd_power_system_off();
  while (1); // never reached
}
