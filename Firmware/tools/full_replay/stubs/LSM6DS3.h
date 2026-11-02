// Host-side stand-in for the Seeed LSM6DS3 driver used by the full_replay
// harness (harness.cpp): the status register always reads "no new sample", so
// FastSampleAccumulator::harvest() falls back to readFloat*(), which return the
// values the harness injects for the current 10ms tick (accelerometer in g,
// gyroscope in deg/s - the 10ms means the real driver path would produce).
#pragma once
#include <cstdint>
#define I2C_MODE 0
#define IMU_SUCCESS 0
#define LSM6DS3_ACC_GYRO_STATUS_REG 0x1E
#define LSM6DS3_ACC_GYRO_OUTX_L_XL 0x28
#define LSM6DS3_ACC_GYRO_OUTX_L_G 0x22
extern float g_imuAccG[3];
extern float g_imuGyroDegS[3];
struct LSM6DS3 {
  struct { int accelRange = 8; int gyroRange = 500; } settings;
  LSM6DS3(int, int) {}
  int begin() { return 0; }
  int readRegister(uint8_t* out, uint8_t) { *out = 0; return 0; }
  int readRegisterRegion(uint8_t*, uint8_t, uint8_t) { return 0; }
  float calcAccel(int16_t) { return 0; }
  float calcGyro(int16_t) { return 0; }
  float readFloatAccelX() { return g_imuAccG[0]; }
  float readFloatAccelY() { return g_imuAccG[1]; }
  float readFloatAccelZ() { return g_imuAccG[2]; }
  float readFloatGyroX() { return g_imuGyroDegS[0]; }
  float readFloatGyroY() { return g_imuGyroDegS[1]; }
  float readFloatGyroZ() { return g_imuGyroDegS[2]; }
};
