// Part of the ODKI VBT tracking engine
// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Required Notice: Copyright (c) 2026 Lodovico Cortelazzo (ODKI)
//
// Source-available, not open source: licensed under the PolyForm
// Noncommercial License 1.0.0 with the additional permissions in
// LICENSE-ENGINE in the repository root (full text there). Free for
// personal, hobby and study use, for research for the benefit of public
// knowledge, and for use by universities, public research organisations,
// charities and the other organisations listed in the license. Any other
// use, including commercial use or selling devices or kits that contain
// this code, needs a commercial licence from ODKI: contact@odki.tech.
//
// Provided as is, without any warranty -- see LICENSE-ENGINE.
//
// This tool #includes MotionTracker.cpp (the engine) and is therefore
// under the engine's license, like the engine itself.

// Full-chain offline replay (tools/full_replay/). Unlike host_replay/replay.cpp
// (which injects the world-Z acceleration), this drives MotionTracker::update()
// itself - FastSampleAccumulator, the 5-sample medians, the orientation filter
// (FusionOrientation), the gravity split, both bias estimators and the
// phase/bracket engine - all compiled unchanged from Firmware/VBT_Quaternions/.
//
// stdin, first line (initial state, as CALIBRATE would leave it):
//   I,accelScaleG,gravX,gravY,gravZ,gyroBiasX,gyroBiasY,gyroBiasZ,accBiasX,accBiasY,accBiasZ
// then one line per sample:
//   t_ms,accRawX,accRawY,accRawZ,gyroRawX,gyroRawY,gyroRawZ,ovAcc,medAx,medAy,medAz,ovGyro,medGx,medGy,medGz
// accRaw* in m/s^2 (the firmware's axRaw = 10ms mean * accelScaleG), gyroRaw* in deg/s
// (10ms mean, before median and bias). ovAcc/ovGyro = 1 replace the median's
// output with med* (used when the log did not record the median's input).
// argv: key=value Fusion settings (gain, gyroRange, accRej, timeout, skipStartup, bias,
// biasThr, biasPeriod; defaults = FusionOrientationSettings, i.e. the firmware's) and
// config=pre3.11.30 (the RuntimeConfig defaults of v3.11.28/3.11.29, to check a re-run
// against the R rows of a log recorded with them).
// stdout: the firmware's own S/R/B/P log, plus one "F," row per sample:
//   F,t_ms,q0,q1,q2,q3,accIgnored,accErrDeg,accRecovery,startup,overrange,resBiasX,resBiasY,resBiasZ
#include <cstdlib>
#include <cstring>
#include <string>
float g_imuAccG[3] = {0, 0, 1};
float g_imuGyroDegS[3] = {0, 0, 0};
static bool g_ovAcc = false, g_ovGyro = false;
static float g_med[6] = {0};
#define ODKI_HOST_MEDIAN_OVERRIDE(ax, ay, az, gx, gy, gz) do { \
    if (g_ovAcc) { ax = g_med[0]; ay = g_med[1]; az = g_med[2]; } \
    if (g_ovGyro) { gx = g_med[3]; gy = g_med[4]; gz = g_med[5]; } } while (0)

#include "MotionTracker.cpp"   // same TU on purpose: access to the engine's internals

SerialStub Serial;
WireStub Wire;
static unsigned long g_now = 0;
unsigned long millis() { return g_now; }

int main(int argc, char** argv) {
  FusionOrientationSettings fs;
  bool legacyConfig = false;
  for (int i = 1; i < argc; i++) {
    const char* eq = std::strchr(argv[i], '=');
    if (!eq) continue;
    std::string k(argv[i], eq - argv[i]);
    if (k == "config") { legacyConfig = std::strcmp(eq + 1, "pre3.11.30") == 0; continue; }
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
  ahrs.configure(fs);
  char line[512];
  if (!std::fgets(line, sizeof line, stdin)) return 1;
  double sc, grx, gry, grz, gbx, gby, gbz, abx, aby, abz;
  if (std::sscanf(line, "I,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &sc, &grx, &gry, &grz,
                  &gbx, &gby, &gbz, &abx, &aby, &abz) != 10) return 1;
  // what calibrateOrientation() leaves behind: alignToGravity(a) returns an
  // attitude whose gravBody is a (since the v3.11.33 seed fix)
  accelScaleG = (float)sc;
  ahrs.alignToGravity((float)grx, (float)gry, (float)grz);
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
    float bx = 0, by = 0, bz = 0;
    ahrs.residualGyroBiasDegS(bx, by, bz);
    std::printf("F,%lu,%.7f,%.7f,%.7f,%.7f,%d,%.3f,%d,%d,%d,%.4f,%.4f,%.4f\n", t, ahrs.q0(), ahrs.q1(),
                ahrs.q2(), ahrs.q3(), (int)ahrs.accelerometerIgnored(), ahrs.accelerationErrorDeg(),
                (int)ahrs.accelerationRecovery(), (int)ahrs.startupActive(), (int)ahrs.overrangeRecovery(),
                bx, by, bz);
    RepResult r;
    while (MotionTracker::peekCompletedRep(r)) MotionTracker::popCompletedRep();
    CorrectedCurveChunk c;
    while (MotionTracker::peekCorrectedCurveChunk(c)) MotionTracker::popCorrectedCurveChunk();
  }
  MotionTracker::setTrackingActive(false);
  return 0;
}
