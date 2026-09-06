// Part of the ODKI VBT Firmware
//
// The 6DOF Madgwick AHRS algorithm implemented in this file was
// released into the public domain by its original author, Sebastian
// O.H. Madgwick (x-io Technologies) -- no rights are claimed over the
// algorithm itself. This specific C++ implementation/adaptation of it
// is Copyright (C) 2026 Lodovico Cortelazzo, licensed under the GNU
// General Public License, Version 3 (this specific version only, not
// "or any later version"), modified by the Commons Clause License
// Condition v1.0 -- see LICENSE-FIRMWARE in the repository root for the
// full text of both. In short: you may use, study, modify, and share
// this file (including a modified version) for non-commercial
// purposes; you may not sell it, or a product/service substantially
// derived from it, without a separate agreement with the copyright
// holder.
//
// This program is distributed WITHOUT ANY WARRANTY, without even the
// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
// PURPOSE -- see LICENSE-FIRMWARE for details.

#ifndef MADGWICK_AHRS_H
#define MADGWICK_AHRS_H

#include <Arduino.h>

// 6DOF Madgwick filter (accelerometer + gyroscope fusion, no
// magnetometer). Standard algorithm, public domain
// (Sebastian O.H. Madgwick, x-io Technologies).
class MadgwickAHRS {
public:
  void reset(); // identity quaternion

  // Aligns the body frame's Z axis to the gravity measured at rest.
  // Call once during calibration (device stationary).
  void alignToGravity(float ax, float ay, float az);

  void update(float gx, float gy, float gz,
              float ax, float ay, float az,
              float dt, float beta);

  float q0() const { return _q0; }
  float q1() const { return _q1; }
  float q2() const { return _q2; }
  float q3() const { return _q3; }

private:
  float _q0 = 1.0f, _q1 = 0.0f, _q2 = 0.0f, _q3 = 0.0f;
};

#endif
