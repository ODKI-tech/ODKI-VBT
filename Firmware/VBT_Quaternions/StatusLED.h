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

#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <Arduino.h>

// "USR" RGB status LED. StatusLED::update(...) must be called on every
// loop() cycle with the current state: the color is recomputed from
// scratch each time (there are no "one-shot" transitions that need to be
// kept manually in sync), so it can never end up out of alignment after
// events like a BLE disconnect mid-streaming.
//
//   Not connected                        -> blinking RED
//   Connected, not calibrated             -> solid RED
//   Connected, calibrated, tracking idle  -> GREEN
//   Connected, calibrated, tracking active -> BLUE
//
// Does not reflect battery charge status: that uses the board's
// dedicated hardware "CH" LED, driven directly by the BQ25100.
namespace StatusLED {
  void begin();
  void update(bool connected, bool calibrated, bool trackingActive);
  void off(); // used only by PowerManager for deep sleep
}

#endif
