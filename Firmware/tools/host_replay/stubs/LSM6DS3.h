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
  float calcAccel(int16_t) { return 0; }
  float calcGyro(int16_t) { return 0; }
  float readFloatAccelX() { return 0; } float readFloatAccelY() { return 0; } float readFloatAccelZ() { return 0; }
  float readFloatGyroX() { return 0; } float readFloatGyroY() { return 0; } float readFloatGyroZ() { return 0; }
};
