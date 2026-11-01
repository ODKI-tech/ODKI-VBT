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

#if ODKI_ORIENTATION_FILTER == ODKI_ORIENTATION_FILTER_FUSION

#include "MadgwickAHRS.h"

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
}

void FusionOrientation::alignToGravity(float ax, float ay, float az) {
  MadgwickAHRS seed; // exactly the Madgwick build's initial attitude
  seed.alignToGravity(ax, ay, az);
  FusionAhrsRestart(&_ahrs);
  FusionQuaternion q;
  q.element.w = seed.q0(); q.element.x = seed.q1(); q.element.y = seed.q2(); q.element.z = seed.q3();
  FusionAhrsSetQuaternion(&_ahrs, q);
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

#endif // ODKI_ORIENTATION_FILTER == ODKI_ORIENTATION_FILTER_FUSION
