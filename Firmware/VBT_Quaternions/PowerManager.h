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
