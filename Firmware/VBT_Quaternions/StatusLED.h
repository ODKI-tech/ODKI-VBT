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
// v3.11.23: "connected" (the first argument) means BLE OR USB-serial, see
// deviceConnected() in VBT_Quaternions.ino - either is enough on its own
// to leave "not connected"; the two transports run independently and
// neither is disabled by the other being active.
//
// Does not reflect battery charge status: that uses the board's
// dedicated hardware "CH" LED, driven directly by the BQ25100.
namespace StatusLED {
  void begin();
  void update(bool connected, bool calibrated, bool trackingActive);
  void off(); // used only by PowerManager for deep sleep
}

#endif
