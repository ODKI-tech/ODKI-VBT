// Part of the ODKI VBT firmware
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Lodovico Cortelazzo (ODKI)
//
// Licensed under the Apache License, Version 2.0 (the "License"); you may
// not use this file except in compliance with the License. You may obtain a
// copy of the License in LICENSE-APACHE in the repository root or at
// http://www.apache.org/licenses/LICENSE-2.0. Unless required by applicable
// law or agreed to in writing, software distributed under the License is
// distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
// KIND, either express or implied. See the License for the specific
// language governing permissions and limitations under the License.

#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

#include <Arduino.h>

// Handles the power switch (WAKE_PIN, see Config.h), deep sleep and the
// hardware watchdog.
//
// WATCHDOG: safety net against firmware lockups (crash, hard fault, hang
// on I2C/BLE/USB) that would otherwise leave the device "dead" - LED off,
// no response to the switch - until the power is physically disconnected.
// This is the case diagnosed previously: a lockup before goToSleep()
// completes leaves the CPU stuck in a state that is NOT true System OFF,
// so the SENSE/DETECT mechanism on WAKE_PIN (which only works to wake
// from a true System OFF) has no way to intervene.
//
// The nRF52840's watchdog, once started, cannot be stopped via software
// (only a reset clears it) - begin() starts it exactly once for the
// entire lifetime of the program, and it must be "fed" periodically with
// feedWatchdog() from then on, otherwise it expires and forces a full
// reset (equivalent to power-cycling the device, but automatic). After a
// watchdog-triggered reset, lastResetReason() will report the DOG bit
// set: that's the evidence needed to confirm (or rule out) that such a
// lockup actually occurred between one boot and the next.
namespace PowerManager {
  void begin();
  bool wakeSwitchClosed();
  void goToSleep(); // System OFF: never returns, waking up restarts from a full reset
  uint32_t lastResetReason(); // NRF_POWER->RESETREAS read in begin(), for diagnosing the wake cause
  void feedWatchdog(); // call on every loop() iteration - see comment above
}

#endif
