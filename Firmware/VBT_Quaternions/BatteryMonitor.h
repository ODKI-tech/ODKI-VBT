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
