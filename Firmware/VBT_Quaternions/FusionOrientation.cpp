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
//
// This file wraps the x-io Technologies Fusion library (src/Fusion/,
// MIT license, Copyright (c) 2021 x-io Technologies - see
// src/Fusion/LICENSE.md). The Fusion files themselves are NOT covered by
// LICENSE-FIRMWARE; only this wrapper is.

#include "FusionOrientation.h"

namespace {
  const float FUSION_SAMPLE_RATE_HZ = 100.0f; // MotionTracker's SAMPLE_INTERVAL_MS
}

FusionOrientation::FusionOrientation() {
  FusionAhrsInitialise(&_ahrs);
  FusionBiasInitialise(&_bias);
  applySettings();
}

void FusionOrientation::configure(const FusionOrientationSettings& settings) {
  _settings = settings;
  applySettings();
}

void FusionOrientation::applySettings() {
  FusionAhrsSettings s;
  s.sampleRate = FUSION_SAMPLE_RATE_HZ;
  s.convention = FusionConventionNwu;
  s.gain = _settings.gain;
  s.gyroscopeRange = _settings.gyroscopeRangeDegS;
  s.accelerationRejection = _settings.accelerationRejectionDeg;
  s.magneticRejection = 0.0f; // no magnetometer
  s.rejectionTimeout = _settings.rejectionTimeoutS;
  FusionAhrsSetSettings(&_ahrs, &s);

  FusionBiasSettings b;
  b.sampleRate = FUSION_SAMPLE_RATE_HZ;
  b.stationaryThreshold = _settings.biasStationaryThresholdDegS;
  b.stationaryPeriod = _settings.biasStationaryPeriodS;
  FusionBiasSetSettings(&_bias, &b);
}

void FusionOrientation::reset() {
  FusionAhrsRestart(&_ahrs);
  FusionBiasSetOffset(&_bias, FUSION_VECTOR_ZERO);
  refreshOutput();
}

void FusionOrientation::refreshOutput() {
  const FusionQuaternion& q = _ahrs.quaternion;
  float n = sqrtf(q.element.w * q.element.w + q.element.x * q.element.x +
                  q.element.y * q.element.y + q.element.z * q.element.z);
  if (n < 1e-6f) { _q[0] = 1.0f; _q[1] = _q[2] = _q[3] = 0.0f; return; }
  float r = 1.0f / n;
  _q[0] = q.element.w * r; _q[1] = q.element.x * r; _q[2] = q.element.y * r; _q[3] = q.element.z * r;
}

void FusionOrientation::gravitySeed(float ax, float ay, float az, float q[4]) {
  // v3.11.33 fix (moved here from the former Madgwick filter in v3.12.0,
  // unchanged). The attitude seeded here must reproduce the measured gravity
  // direction in the convention MotionTracker's gravBody reads the
  // quaternion with: the estimated "up" in the BODY frame is the third row
  // of the body->world rotation matrix,
  //   gb = (2(q1q3 - q0q2), 2(q0q1 + q2q3), q0^2 - q1^2 - q2^2 + q3^2),
  // i.e. the world Z axis rotated INTO the body frame. Up to v3.11.32 the
  // seed was the opposite rotation (body Z rotated onto a), whose gb is
  // (-ax, -ay, az): the tilt mirrored, off by twice the tilt, which the
  // filter then had to recover on its own (see "Calibration" in
  // DOCUMENTATION.md).
  //
  // Built directly as the minimal rotation with no yaw component (q3 = 0):
  // gb = (-2 q0 q2, 2 q0 q1, 1 - 2 s^2) with s^2 = q1^2 + q2^2. Requiring
  // gb = a gives q0 = sqrt((1 + az) / 2), s = sqrt((1 - az) / 2), and the
  // horizontal direction of (q1, q2) from (ay, -ax). 2 q0 s = h, the
  // horizontal norm of a, so whichever of q0 / s is well conditioned is
  // taken from az and the other from h - accurate from level to upside down
  // (no acos(), no division by a vanishing number).
  float norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm < 1e-6f) { q[0] = 1.0f; q[1] = q[2] = q[3] = 0.0f; return; }
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
    q[0] = q0; q[1] = (az >= 0.0f) ? 0.0f : s; q[2] = 0.0f; q[3] = 0.0f;
  } else {
    q[0] = q0; q[1] = s * ay / h; q[2] = -s * ax / h; q[3] = 0.0f;
  }
  float recipNorm = 1.0f / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2]);
  q[0] *= recipNorm; q[1] *= recipNorm; q[2] *= recipNorm;
}

void FusionOrientation::alignToGravity(float ax, float ay, float az) {
  float q[4];
  gravitySeed(ax, ay, az, q);
  setQuaternion(q[0], q[1], q[2], q[3]);
}

void FusionOrientation::setQuaternion(float q0, float q1, float q2, float q3) {
  FusionAhrsRestart(&_ahrs);
  FusionQuaternion q;
  q.element.w = q0; q.element.x = q1; q.element.y = q2; q.element.z = q3;
  FusionAhrsSetQuaternion(&_ahrs, q);
  refreshOutput();
  // The attitude is known (seeded from the measured gravity since the
  // v3.11.33 fix): the 3 s startup ramp - a high gain meant to
  // pull an UNKNOWN initial attitude in - is skipped unless configured.
  if (_settings.skipStartup) FusionAhrsSkipStartup(&_ahrs);
  // the firmware re-seeds its own gyroscope bias at CALIBRATE: the residual
  // estimate starts again from zero too
  FusionBiasSetOffset(&_bias, FUSION_VECTOR_ZERO);
}

void FusionOrientation::update(float gx, float gy, float gz,
                               float ax, float ay, float az, float dt) {
  FusionVector gyro;
  gyro.axis.x = gx * (float)RAD_TO_DEG;
  gyro.axis.y = gy * (float)RAD_TO_DEG;
  gyro.axis.z = gz * (float)RAD_TO_DEG;
  if (_settings.gyroBiasEstimator) gyro = FusionBiasUpdate(&_bias, gyro);
  FusionVector acc;
  acc.axis.x = ax; acc.axis.y = ay; acc.axis.z = az;
  // the loop's real period (10 or 11 ms), as Fusion recommends for clock errors
  FusionAhrsSetSamplePeriod(&_ahrs, dt);
  FusionAhrsUpdateNoMagnetometer(&_ahrs, gyro, acc);
  refreshOutput();
}

float FusionOrientation::accelerationErrorDeg() const {
  return FusionAhrsGetInternalStates(&_ahrs).accelerationError;
}

bool FusionOrientation::accelerationRecovery() const {
  return FusionAhrsGetFlags(&_ahrs).accelerationRecovery;
}

void FusionOrientation::residualGyroBiasDegS(float& x, float& y, float& z) const {
  FusionVector o = FusionBiasGetOffset(&_bias);
  x = o.axis.x; y = o.axis.y; z = o.axis.z;
}
