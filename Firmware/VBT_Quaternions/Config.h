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

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Power switch: closed (to GND) = device on.
#define WAKE_PIN D6

// Firmware version, exposed over BLE as a "MAJOR.MINOR.PATCH" string on the
// standard Device Information Service (see deviceInfoService in
// BleServer.cpp) so the app can compare it against the latest available and
// offer the OTA update (BLE DFU, see BLEDfu in BleServer.cpp)
// without the tester having to do anything manually.
// Increment on EVERY release sent to testers. The companion app parses the
// string as exactly three integers (anything else reads as "unknown
// version" and also turns off its chunked Config write) and offers an OTA
// update only when the manifest's version is strictly greater.
namespace FirmwareVersion {
  const uint8_t MAJOR = 3;
  const uint8_t MINOR = 12;
  const uint8_t PATCH = 9;
}

// v3.12.0: the x-io Fusion AHRS (src/Fusion/, MIT, wrapped by
// FusionOrientation.cpp) is the only orientation filter. The build-time
// filter switch, the run-time FILTER switch and the build tag of the
// 3.11.33 development build are gone; the firmware identifies itself by
// FirmwareVersion alone (boot line, VERSION serial command, "INFO," log
// line, BLE Device Information).

namespace Config {
  // How often to print/notify battery status.
  const unsigned long BATTERY_CHECK_INTERVAL_MS = 30000UL;

  // Internal IMU sampling runs at 100Hz (see MotionTracker), but BLE
  // streaming sends 1 sample every N to avoid saturating the connection:
  // 100Hz / 5 = 20Hz over BLE.
  const uint8_t STREAM_DECIMATION_FACTOR = 5;
}

#endif
