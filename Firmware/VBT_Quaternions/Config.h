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

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Power switch: closed (to GND) = device on.
#define WAKE_PIN D6

// Firmware version, exposed over BLE as a "MAJOR.MINOR.PATCH" string on the
// standard Device Information Service (see deviceInfoService in
// BleServer.cpp) so the app can compare it against the latest available and
// offer the OTA update (BLE DFU, see Adafruit_BLEDFU in BleServer.cpp)
// without the tester having to do anything manually.
// Increment on EVERY release sent to testers.
namespace FirmwareVersion {
  const uint8_t MAJOR = 3;
  const uint8_t MINOR = 11;
  const uint8_t PATCH = 24;
}

namespace Config {
  // How often to print/notify battery status.
  const unsigned long BATTERY_CHECK_INTERVAL_MS = 30000UL;

  // Internal IMU sampling runs at 100Hz (see MotionTracker), but BLE
  // streaming sends 1 sample every N to avoid saturating the connection:
  // 100Hz / 5 = 20Hz over BLE.
  const uint8_t STREAM_DECIMATION_FACTOR = 5;
}

#endif
