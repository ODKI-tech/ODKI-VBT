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

// Full-chain offline replay for the orientation-filter feasibility study.
// Unlike host_replay/replay.cpp (which injects the world-Z acceleration), this
// drives MotionTracker::update() itself - FastSampleAccumulator, the 5-sample
// medians, the orientation filter (Madgwick or Fusion, chosen at compile time
// with -DODKI_ORIENTATION_FILTER, exactly like the firmware), the gravity split,
// both bias estimators and the phase/bracket engine - all compiled unchanged.
//
// stdin, first line (initial state, as CALIBRATE would leave it):
//   I,accelScaleG,gravX,gravY,gravZ,gyroBiasX,gyroBiasY,gyroBiasZ,accBiasX,accBiasY,accBiasZ
// then one line per sample:
//   t_ms,accRawX,accRawY,accRawZ,gyroRawX,gyroRawY,gyroRawZ,ovAcc,medAx,medAy,medAz,ovGyro,medGx,medGy,medGz
// accRaw* in m/s^2 (the firmware's axRaw = 10ms mean * accelScaleG), gyroRaw* in deg/s
// (10ms mean, before median and bias). ovAcc/ovGyro = 1 replace the median's
// output with med* (used when the log did not record the median's input).
// argv: key=value Fusion settings (gain, gyroRange, accRej, timeout, skipStartup, bias,
// biasThr, biasPeriod) - ignored by the Madgwick build - and, for both builds, seedMode=legacy
// (the pre-v3.11.33 mirrored CALIBRATE seed; seedMode=firmware is accepted and is now the same as
// the default) and config=pre3.11.30 (the RuntimeConfig defaults of v3.11.28/3.11.29, to check a re-run against
// the R rows of a log recorded with them).
// stdout: the firmware's own S/R/B/P log, plus one "F," row per sample:
//   F,t_ms,q0,q1,q2,q3,accIgnored,accErrDeg,accRecovery,startup,overrange,resBiasX,resBiasY,resBiasZ
#include <cstdlib>
#include <cstring>
#include <string>
float g_imuAccG[3] = {0, 0, 1};
float g_imuGyroDegS[3] = {0, 0, 0};
static bool g_ovAcc = false, g_ovGyro = false;
static float g_med[6] = {0};
static bool g_seedLegacy = false;
#define ODKI_HOST_MEDIAN_OVERRIDE(ax, ay, az, gx, gy, gz) do { \
    if (g_ovAcc) { ax = g_med[0]; ay = g_med[1]; az = g_med[2]; } \
    if (g_ovGyro) { gx = g_med[3]; gy = g_med[4]; gz = g_med[5]; } } while (0)

#include "MotionTracker.cpp"   // same TU on purpose: access to the engine's internals

// The firmware is built either with one filter (`ahrs`) or, with
// ODKI_FILTER_RUNTIME_SELECT=1 (the v3.11.33 session build), with both
// (`madgwickAhrs`/`fusionAhrs`, argv filter=FUSION selects Fusion through
// MotionTracker::setOrientationFilter(), as the FILTER serial command does).
#if ODKI_FILTER_RUNTIME_SELECT
static bool useFusion() { return MotionTracker::orientationFilter() == OrientationFilter::Fusion; }
#define AHRS_Q(i) (useFusion() ? fusionAhrs.q##i() : madgwickAhrs.q##i())
#define FUSION_OBJ fusionAhrs
#define FUSION_ACTIVE useFusion()
static void seedAll(float x, float y, float z) { madgwickAhrs.alignToGravity(x, y, z); fusionAhrs.alignToGravity(x, y, z); }
#else
#define AHRS_Q(i) ahrs.q##i()
#define FUSION_OBJ ahrs
#define FUSION_ACTIVE (ODKI_ORIENTATION_FILTER == ODKI_ORIENTATION_FILTER_FUSION)
static void seedAll(float x, float y, float z) { ahrs.alignToGravity(x, y, z); }
#endif

SerialStub Serial;
WireStub Wire;
static unsigned long g_now = 0;
unsigned long millis() { return g_now; }

int main(int argc, char** argv) {
#if ODKI_FUSION_COMPILED
  FusionOrientationSettings fs;
  for (int i = 1; i < argc; i++) {
    const char* eq = std::strchr(argv[i], '=');
    if (!eq) continue;
    std::string k(argv[i], eq - argv[i]);
    if (k == "seedMode" || k == "config" || k == "filter") continue;
    float v = (float)std::atof(eq + 1);
    if (k == "gain") fs.gain = v;
    else if (k == "gyroRange") fs.gyroscopeRangeDegS = v;
    else if (k == "accRej") fs.accelerationRejectionDeg = v;
    else if (k == "timeout") fs.rejectionTimeoutS = v;
    else if (k == "skipStartup") fs.skipStartup = v != 0;
    else if (k == "bias") fs.gyroBiasEstimator = v != 0;
    else if (k == "biasThr") fs.biasStationaryThresholdDegS = v;
    else if (k == "biasPeriod") fs.biasStationaryPeriodS = v;
    else { std::fprintf(stderr, "unknown setting %s\n", k.c_str()); return 2; }
  }
  FUSION_OBJ.configure(fs);
#endif
  bool legacyConfig = false;
  for (int i = 1; i < argc; i++) {
    if (std::strcmp(argv[i], "seedMode=legacy") == 0) g_seedLegacy = true;
    if (std::strcmp(argv[i], "config=pre3.11.30") == 0) legacyConfig = true;
    if (std::strcmp(argv[i], "filter=FUSION") == 0 && !MotionTracker::setOrientationFilter(OrientationFilter::Fusion)) {
      std::fprintf(stderr, "filter=FUSION: not available in this build\n"); return 2;
    }
  }
  char line[512];
  if (!std::fgets(line, sizeof line, stdin)) return 1;
  double sc, grx, gry, grz, gbx, gby, gbz, abx, aby, abz;
  if (std::sscanf(line, "I,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &sc, &grx, &gry, &grz,
                  &gbx, &gby, &gbz, &abx, &aby, &abz) != 10) return 1;
  // what calibrateOrientation() leaves behind
  accelScaleG = (float)sc;
  // Since v3.11.33 alignToGravity(a) returns an attitude whose gravBody is a
  // (what CALIBRATE does, and the logged gravity direction back exactly).
  // seedMode=legacy reproduces the pre-v3.11.33 seed, whose gravBody was
  // (-ax, -ay, az) - the tilt mirrored - by passing the mirror of it.
  if (g_seedLegacy) seedAll((float)-grx, (float)-gry, (float)grz);
  else seedAll((float)grx, (float)gry, (float)grz);
  gyroBiasX = (float)gbx; gyroBiasY = (float)gby; gyroBiasZ = (float)gbz;
  calibrated = true;

  bool started = false;
  while (std::fgets(line, sizeof line, stdin)) {
    double v[15];
    int n = std::sscanf(line, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                        &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7], &v[8], &v[9],
                        &v[10], &v[11], &v[12], &v[13], &v[14]);
    if (n != 15) continue;
    unsigned long t = (unsigned long)v[0];
    if (!started) {
      g_now = t - 10; lastSampleTime = g_now;
      if (legacyConfig) {  // RuntimeConfig defaults before v3.11.30 (logs recorded with 3.11.28/3.11.29)
        RuntimeConfig c = MotionTracker::defaultConfig();
        c.maxVelocityFlatWindowSamples = 30; c.maxAccelerationFlatWindowSamples = 30;
        c.velocityOverrideFlatWindowSamples = 45;
        MotionTracker::setConfig(c);
      }
      MotionTracker::setTrackingActive(true);   // resets the engine (not the biases), prints REC_START + headers
      // accBiasVec is NOT reset by START: the harness seeds it after the reset
      accBiasVec[0] = (float)abx; accBiasVec[1] = (float)aby; accBiasVec[2] = (float)abz;
      started = true;
    }
    for (int k = 0; k < 3; k++) {
      g_imuAccG[k] = (float)(v[1 + k] / sc);
      g_imuGyroDegS[k] = (float)v[4 + k];
    }
    g_ovAcc = v[7] != 0; g_ovGyro = v[11] != 0;
    for (int k = 0; k < 3; k++) { g_med[k] = (float)v[8 + k]; g_med[3 + k] = (float)v[12 + k]; }
    g_now = t;
    if (!MotionTracker::update()) continue;
    int ign = 0, rec = 0, st = 0, ovr = 0; float err = 0, bx = 0, by = 0, bz = 0;
#if ODKI_FUSION_COMPILED
    if (FUSION_ACTIVE) {
      ign = FUSION_OBJ.accelerometerIgnored(); err = FUSION_OBJ.accelerationErrorDeg(); rec = FUSION_OBJ.accelerationRecovery();
      st = FUSION_OBJ.startupActive(); ovr = FUSION_OBJ.overrangeRecovery(); FUSION_OBJ.residualGyroBiasDegS(bx, by, bz);
    }
#endif
    std::printf("F,%lu,%.7f,%.7f,%.7f,%.7f,%d,%.3f,%d,%d,%d,%.4f,%.4f,%.4f\n", t, AHRS_Q(0), AHRS_Q(1),
                AHRS_Q(2), AHRS_Q(3), ign, err, rec, st, ovr, bx, by, bz);
    RepResult r;
    while (MotionTracker::peekCompletedRep(r)) MotionTracker::popCompletedRep();
    CorrectedCurveChunk c;
    while (MotionTracker::peekCorrectedCurveChunk(c)) MotionTracker::popCorrectedCurveChunk();
  }
  MotionTracker::setTrackingActive(false);
  return 0;
}
