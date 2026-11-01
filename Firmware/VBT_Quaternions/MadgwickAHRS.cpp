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

#include "MadgwickAHRS.h"

void MadgwickAHRS::reset() {
  _q0 = 1.0f; _q1 = 0.0f; _q2 = 0.0f; _q3 = 0.0f;
}

void MadgwickAHRS::alignToGravity(float ax, float ay, float az) {
  // v3.11.33 fix. The attitude seeded here must reproduce the measured
  // gravity direction in the convention update() (and MotionTracker's
  // gravBody) reads the quaternion with: the estimated "up" in the BODY
  // frame is the third row of the body->world rotation matrix,
  //   gb = (2(q1q3 - q0q2), 2(q0q1 + q2q3), q0^2 - q1^2 - q2^2 + q3^2),
  // i.e. the world Z axis rotated INTO the body frame. Up to v3.11.32 this
  // function built the opposite rotation (body Z rotated onto a), whose gb is
  // (-ax, -ay, az): the tilt mirrored, off by twice the tilt, which the
  // filter then had to recover at Madgwick's bounded rate (~17 s at 72 deg,
  // never at 90 deg - see "alignToGravity()" in DOCUMENTATION.md).
  //
  // Built directly as the minimal rotation with no yaw component (q3 = 0):
  // gb = (-2 q0 q2, 2 q0 q1, 1 - 2 s^2) with s^2 = q1^2 + q2^2. Requiring
  // gb = a gives q0 = sqrt((1 + az) / 2), s = sqrt((1 - az) / 2), and the
  // horizontal direction of (q1, q2) from (ay, -ax). 2 q0 s = h, the
  // horizontal norm of a, so whichever of q0 / s is well conditioned is
  // taken from az and the other from h - accurate from level to upside down
  // (no acos(), no division by a vanishing number).
  float norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm < 1e-6f) { reset(); return; }
  ax /= norm; ay /= norm; az /= norm;

  float h = sqrt(ax * ax + ay * ay); // horizontal part of a = 2 q0 s
  float q0, s;
  if (az >= 0.0f) {
    q0 = sqrt(0.5f * (1.0f + az));   // >= 0.707
    s = 0.5f * h / q0;
  } else {
    s = sqrt(0.5f * (1.0f - az));    // >= 0.707
    q0 = 0.5f * h / s;
  }
  if (h < 1e-7f) {
    // Exactly level (identity) or exactly upside down: any horizontal
    // rotation axis is correct for the latter, X is used.
    _q0 = q0; _q1 = (az >= 0.0f) ? 0.0f : s; _q2 = 0.0f; _q3 = 0.0f;
  } else {
    _q0 = q0; _q1 = s * ay / h; _q2 = -s * ax / h; _q3 = 0.0f;
  }
  float recipNorm = 1.0f / sqrt(_q0 * _q0 + _q1 * _q1 + _q2 * _q2);
  _q0 *= recipNorm; _q1 *= recipNorm; _q2 *= recipNorm;
}

void MadgwickAHRS::setQuaternion(float q0, float q1, float q2, float q3) {
  float n = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  if (n < 1e-6f) { reset(); return; }
  _q0 = q0 / n; _q1 = q1 / n; _q2 = q2 / n; _q3 = q3 / n;
}

void MadgwickAHRS::update(float gx, float gy, float gz,
                           float ax, float ay, float az,
                           float dt, float beta) {
  float q0 = _q0, q1 = _q1, q2 = _q2, q3 = _q3;
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  // Rate of change of the quaternion from the gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  if (!(ax == 0.0f && ay == 0.0f && az == 0.0f)) {
    recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

    _2q0 = 2.0f * q0; _2q1 = 2.0f * q1; _2q2 = 2.0f * q2; _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0; _4q1 = 4.0f * q1; _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1; _8q2 = 8.0f * q2;
    q0q0 = q0 * q0; q1q1 = q1 * q1; q2q2 = q2 * q2; q3q3 = q3 * q3;

    // Gradient of the error function (difference between estimated and measured gravity)
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = 1.0f / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

    // Apply the correction (weighted by the adaptive gain beta)
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate to get the new quaternion
  q0 += qDot1 * dt; q1 += qDot2 * dt; q2 += qDot3 * dt; q3 += qDot4 * dt;

  // Normalize (the quaternion must remain unit length)
  recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  _q0 = q0 * recipNorm; _q1 = q1 * recipNorm; _q2 = q2 * recipNorm; _q3 = q3 * recipNorm;
}
