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
// offer the OTA update (BLE DFU, see BLEDfu in BleServer.cpp)
// without the tester having to do anything manually.
// Increment on EVERY release sent to testers.
namespace FirmwareVersion {
  const uint8_t MAJOR = 3;
  const uint8_t MINOR = 11;
  const uint8_t PATCH = 33;
}

// Build identity printed at boot, by the VERSION serial command, in the
// "INFO," line at every REC_START and as the BLE Device Information
// Software Revision String (0x2A28). The Firmware Revision String (0x2A26)
// stays a plain "MAJOR.MINOR.PATCH": the companion app parses it as exactly
// three integers (anything else reads as "unknown version" and also turns
// off its chunked Config write) and offers an OTA update only when the
// manifest's version is strictly newer.
// "release" for a build published through firmware-release/; anything else
// marks a development build that must never be put in the OTA manifest.
// v3.11.33 "dev-session": the recording-session build (branch session-build)
// - NOT a release.
#define ODKI_BUILD_TAG "dev-session"

// Orientation filter, selected at BUILD time (feasibility study, see
// "Orientation filter selection" in DOCUMENTATION.md):
//   ODKI_ORIENTATION_FILTER_MADGWICK - MadgwickAHRS.cpp with the adaptive beta
//                                      (default; the released behaviour)
//   ODKI_ORIENTATION_FILTER_FUSION   - x-io Fusion AHRS v1.3.3 (src/Fusion/,
//                                      MIT), wrapped by FusionOrientation.cpp
// Change the default below, or pass -DODKI_ORIENTATION_FILTER=1 to the
// compiler (arduino-cli: --build-property "compiler.cpp.extra_flags=..."
// and "compiler.c.extra_flags=..."). Experimental: no Fusion build has
// been released or validated on hardware.
#define ODKI_ORIENTATION_FILTER_MADGWICK 0
#define ODKI_ORIENTATION_FILTER_FUSION   1
#ifndef ODKI_ORIENTATION_FILTER
#define ODKI_ORIENTATION_FILTER ODKI_ORIENTATION_FILTER_MADGWICK
#endif

// Runtime filter selection (dev/session builds). 1: BOTH filters are
// compiled in, ODKI_ORIENTATION_FILTER above only picks the one active at
// boot, and the plain-text serial command "FILTER MADGWICK" / "FILTER
// FUSION" switches between them while tracking is stopped (the attitude is
// handed over, so no re-CALIBRATE is needed - see
// MotionTracker::setOrientationFilter()). 0: only the filter selected above
// is compiled (the released configuration). Set to 1 on session-build so
// one flash can record both filters.
#ifndef ODKI_FILTER_RUNTIME_SELECT
#define ODKI_FILTER_RUNTIME_SELECT 1
#endif
// Whether the Fusion code is compiled at all.
#if ODKI_FILTER_RUNTIME_SELECT || ODKI_ORIENTATION_FILTER == ODKI_ORIENTATION_FILTER_FUSION
#define ODKI_FUSION_COMPILED 1
#else
#define ODKI_FUSION_COMPILED 0
#endif

namespace Config {
  // How often to print/notify battery status.
  const unsigned long BATTERY_CHECK_INTERVAL_MS = 30000UL;

  // Internal IMU sampling runs at 100Hz (see MotionTracker), but BLE
  // streaming sends 1 sample every N to avoid saturating the connection:
  // 100Hz / 5 = 20Hz over BLE.
  const uint8_t STREAM_DECIMATION_FACTOR = 5;
}

#endif
