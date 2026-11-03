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

#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <Arduino.h>

// Reads the 3.7V/360mAh LiPo through the XIAO nRF52840 Sense's onboard
// divider (VBAT_ENABLE / PIN_VBAT). NOTE: the divider's compensation
// factor (VBAT_DIVIDER_COMP, in BatteryMonitor.cpp) is not officially
// published by Seeed; it's taken from a third-party implementation, and
// the divider's real ratio can vary slightly from board to board
// (component tolerance). For this reason readVoltage() ALSO applies a
// per-device correction factor, automatically learned and persisted to
// flash - see the AUTO-CALIBRATION comment in BatteryMonitor.cpp.
namespace BatteryMonitor {
  void begin();
  float readVoltage();                    // Volts - already calibrated (see above)
  float voltageToPercent(float volts);    // 0-100

  // true while the BQ25100's ~CHG (D23/P0.17) signals charging in
  // progress - see the AUTO-CALIBRATION comment in BatteryMonitor.cpp.
  // Diagnostic only: no phase/state logic depends on this.
  bool isCharging();
}

#endif
