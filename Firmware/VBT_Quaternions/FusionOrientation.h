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

#ifndef FUSION_ORIENTATION_H
#define FUSION_ORIENTATION_H

#include "Config.h"

#if ODKI_FUSION_COMPILED

#include <Arduino.h>
#include "src/Fusion/Fusion.h"

// Settings of the Fusion orientation filter. The defaults are Fusion's own
// recommended values (README of v1.3.3), at the firmware's 100Hz rate and
// with the LSM6DS3 gyroscope range MotionTracker::begin() configures
// (500 deg/s) - see "Orientation filter selection" in DOCUMENTATION.md for
// how they relate to the Madgwick adaptive beta / TRUST_FLOOR. Plain
// struct so the offline harness (tools/fusion_feasibility/) can sweep it.
struct FusionOrientationSettings {
  float gain = 0.5f;                     // Fusion "gain" (dimensionless, ~1/time constant in s)
  float gyroscopeRangeDegS = 500.0f;     // overrange restart above 98% of this; 0 = off
  float accelerationRejectionDeg = 10.0f;// ignore the accelerometer above this tilt error; 0 = off
  float rejectionTimeoutS = 5.0f;        // forced acceleration recovery after this; 0 = rejection off
  bool skipStartup = true;               // CALIBRATE already aligns to gravity: no 3s gain ramp
  // FusionBias (run-time gyroscope offset), applied on top of the
  // firmware's own continuous gyroscope bias (gyroBiasX/Y/Z, ~8s EMA), to the
  // residual it leaves. Off by default: the firmware estimator already does
  // the same job (stillness-gated, ~8s time constant).
  bool gyroBiasEstimator = false;
  float biasStationaryThresholdDegS = 3.0f;
  float biasStationaryPeriodS = 3.0f;
};

// Same interface as MadgwickAHRS (reset / alignToGravity / update / q0..q3)
// so MotionTracker's quaternion -> gravity split -> world-frame acceleration
// path is unchanged. Quaternion convention: body -> world, world Z up -
// Fusion's NWU convention, the same as MadgwickAHRS.
class FusionOrientation {
public:
  FusionOrientation();
  void configure(const FusionOrientationSettings& settings);
  const FusionOrientationSettings& settings() const { return _settings; }

  void reset(); // identity quaternion, Fusion restarted (startup ramp)

  // Same initial attitude as MadgwickAHRS::alignToGravity() (it is computed
  // by it, so it gets the v3.11.33 seed fix too), then Fusion skips its
  // startup ramp if settings.skipStartup.
  void alignToGravity(float ax, float ay, float az);

  // Restarts Fusion at the given attitude (runtime filter hand-over), with
  // the same startup / bias handling as alignToGravity().
  void setQuaternion(float q0, float q1, float q2, float q3);

  // gx/gy/gz in rad/s (bias already subtracted by MotionTracker), ax/ay/az in
  // any consistent unit (only the direction is used), dt in seconds.
  void update(float gx, float gy, float gz,
              float ax, float ay, float az, float dt);

  // v3.11.33: unit-norm copy of Fusion's quaternion. Fusion normalises with
  // its fast inverse square root (~1e-4 relative error), and MotionTracker's
  // gravBody/rotation expressions assume an exact unit quaternion (a norm of
  // 1 + 1e-4 scales gravBody by 1 + 2e-4, ~0.002 m/s^2 of spurious vertical
  // acceleration). Fusion's internal state is left untouched.
  float q0() const { return _q[0]; }
  float q1() const { return _q[1]; }
  float q2() const { return _q[2]; }
  float q3() const { return _q[3]; }

  // Diagnostics (offline harness only).
  bool accelerometerIgnored() const { return _ahrs.accelerometerIgnored; }
  float accelerationErrorDeg() const;
  bool accelerationRecovery() const;
  bool startupActive() const { return _ahrs.startup; }
  bool overrangeRecovery() const { return _ahrs.overrangeRecovery; }
  void residualGyroBiasDegS(float& x, float& y, float& z) const;

private:
  FusionOrientationSettings _settings;
  FusionAhrs _ahrs;
  FusionBias _bias;
  float _q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  void applySettings();
  void refreshOutput();
};

#endif // ODKI_FUSION_COMPILED
#endif
