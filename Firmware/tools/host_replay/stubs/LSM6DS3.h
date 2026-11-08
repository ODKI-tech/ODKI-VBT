// Part of the ODKI VBT tools
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

// Host-side stand-in for the Seeed LSM6DS3 driver - never used at runtime in the replay.
#pragma once
#include <cstdint>
#define I2C_MODE 0
#define IMU_SUCCESS 0
#define LSM6DS3_ACC_GYRO_STATUS_REG 0x1E
#define LSM6DS3_ACC_GYRO_OUTX_L_XL 0x28
#define LSM6DS3_ACC_GYRO_OUTX_L_G 0x22
struct LSM6DS3 {
  struct { int accelRange = 8; int gyroRange = 500; } settings;
  LSM6DS3(int, int) {}
  int begin() { return 0; }
  int readRegister(uint8_t*, uint8_t) { return 0; }
  int readRegisterRegion(uint8_t*, uint8_t, uint8_t) { return 0; }
  int writeRegister(uint8_t, uint8_t) { return 0; }  // v3.12.10: FifoLogger (stays off on the host: CTRL5 reads back 0)
  float calcAccel(int16_t) { return 0; }
  float calcGyro(int16_t) { return 0; }
  float readFloatAccelX() { return 0; } float readFloatAccelY() { return 0; } float readFloatAccelZ() { return 0; }
  float readFloatGyroX() { return 0; } float readFloatGyroY() { return 0; } float readFloatGyroZ() { return 0; }
};
