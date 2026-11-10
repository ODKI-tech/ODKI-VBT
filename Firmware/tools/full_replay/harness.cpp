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
//   I,accelScaleG,gravX,gravY,gravZ,gyroBiasX,gyroBiasY,gyroBiasZ,accBiasX,accBiasY,accBiasZ[,qw,qx,qy,qz]
// (with the optional quaternion the filter starts from it instead of the
// zero-yaw gravity seed), then optionally "T,<ms>" (the period of the first
// sample, default 10) and, for a log that joins a running
// session (no START in it, see joined_state() in reconstruct.py):
//   J,sessionT,nextBracketId,nextPhaseId,repNumber,emaInitialized,emaRate,velZFrameOffset,
//     repCalibIntercept,velZLive,worldAccZDirected,gyroMag,accMag,integrationAccZDirected,shockWz
// then one line per sample:
//   t_ms,accRawX,accRawY,accRawZ,gyroRawX,gyroRawY,gyroRawZ,ovAcc,medAx,medAy,medAz,ovGyro,medGx,medGy,medGz
// accRaw* in m/s^2 (the firmware's axRaw = 10ms mean * accelScaleG), gyroRaw* in deg/s
// (10ms mean, before median and bias). ovAcc/ovGyro = 1 replace the median's
// output with med* (used when the log did not record the median's input).
// argv: key=value Fusion settings (gain, gyroRange, accRej, timeout, skipStartup, bias,
// biasThr, biasPeriod; defaults = FusionOrientationSettings, i.e. the firmware's) and
// cfg.<field>=value (a Config-packet field the app may have written, e.g.
// cfg.maxVelocityFlatWindowSamples=30; applied before START) and
// config=pre3.11.30 (the RuntimeConfig defaults of v3.11.28/3.11.29, to check a re-run
// against the R rows of a log recorded with them).
// stdout: the firmware's own S/R/B/P log, plus one "F," row per sample:
//   F,t_ms,q0,q1,q2,q3,accIgnored,accErrDeg,accRecovery,startup,overrange,resBiasX,resBiasY,resBiasZ
// With the environment variable ODKI_DUMP_CHUNKS set, also one "C," row per
// CorrectedCurve chunk (what the app receives over BLE), off by default so the
// regression output is unchanged:
//   C,bracketId,chunkIndex,totalChunks,startTimestampMs,sampleCount,v0,v1,...
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

// cfg.<field>=value: a RuntimeConfig field the app can write over BLE (the
// Config packet), set before START - to replay a log recorded after the app
// changed it. Returns false for an unknown field.
static bool setConfigField(RuntimeConfig& c, const std::string& k, double v) {
#define ODKI_CFG(f, T) if (k == #f) { c.f = (T)v; return true; }
  if (k == "repDirection") { c.repDirection = v != 0 ? RepDirection::Down : RepDirection::Up; return true; }
  ODKI_CFG(maxPlausibleVelocityMps, float) ODKI_CFG(flatGuardMaxVelocityMps, float)
  ODKI_CFG(velocityOverrideFlatWindowSamples, uint8_t) ODKI_CFG(velocityFlatBandMps, float)
  ODKI_CFG(maxVelocityFlatWindowSamples, uint8_t) ODKI_CFG(minVelocityFlatWindowSamples, uint8_t)
  ODKI_CFG(accelerationFlatBandMps2, float) ODKI_CFG(maxAccelerationFlatWindowSamples, uint8_t)
  ODKI_CFG(minAccelerationFlatWindowSamples, uint8_t) ODKI_CFG(windowSaturationPeakVelocityMps, float)
  ODKI_CFG(phaseStartVelocityMps, float) ODKI_CFG(minPhaseDurationS, float) ODKI_CFG(maxPhaseDurationS, float)
  ODKI_CFG(phaseLookbackSamples, uint8_t) ODKI_CFG(reversalConfirmSamples, uint8_t)
#undef ODKI_CFG
  return false;
}

SerialStub Serial;
WireStub Wire;
static unsigned long g_now = 0;
unsigned long millis() { return g_now; }

int main(int argc, char** argv) {
  FusionOrientationSettings fs;
  bool legacyConfig = false;
  RuntimeConfig appConfig = MotionTracker::defaultConfig();
  bool appConfigSet = false;
  const bool dumpChunks = std::getenv("ODKI_DUMP_CHUNKS") != nullptr;
  for (int i = 1; i < argc; i++) {
    const char* eq = std::strchr(argv[i], '=');
    if (!eq) continue;
    std::string k(argv[i], eq - argv[i]);
    if (k == "config") { legacyConfig = std::strcmp(eq + 1, "pre3.11.30") == 0; continue; }
    if (k.rfind("cfg.", 0) == 0) {
      if (!setConfigField(appConfig, k.substr(4), std::atof(eq + 1))) {
        std::fprintf(stderr, "unknown config field %s\n", k.c_str()); return 2;
      }
      appConfigSet = true;
      continue;
    }
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
  double sc, grx, gry, grz, gbx, gby, gbz, abx, aby, abz, qw, qx, qy, qz;
  int nInit = std::sscanf(line, "I,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &sc, &grx, &gry,
                          &grz, &gbx, &gby, &gbz, &abx, &aby, &abz, &qw, &qx, &qy, &qz);
  if (nInit != 10 && nInit != 14) return 1;
  accelScaleG = (float)sc;
  if (nInit == 14) {
    // the attitude the filter had before the first logged sample (see
    // seed_quaternion() in reconstruct.py), yaw included
    ahrs.setQuaternion((float)qw, (float)qx, (float)qy, (float)qz);
  } else {
    // what calibrateOrientation() leaves behind: alignToGravity(a) returns an
    // attitude whose gravBody is a (since the v3.11.33 seed fix)
    ahrs.alignToGravity((float)grx, (float)gry, (float)grz);
  }
  gyroBiasX = (float)gbx; gyroBiasY = (float)gby; gyroBiasZ = (float)gbz;
  calibrated = true;

  bool started = false;
  // optional "J," line: the log joins a session already running (no START
  // in it) - see joined_state() in reconstruct.py
  bool joined = false;
  double J[15] = {0};
  unsigned long firstDtMs = 10;
  while (std::fgets(line, sizeof line, stdin)) {
    if (line[0] == 'T' && line[1] == ',') { firstDtMs = std::strtoul(line + 2, nullptr, 10); continue; }
    if (line[0] == 'J' && line[1] == ',') {
      joined = std::sscanf(line + 2, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &J[0], &J[1], &J[2],
                           &J[3], &J[4], &J[5], &J[6], &J[7], &J[8], &J[9], &J[10], &J[11], &J[12], &J[13]) == 14;
      continue;
    }
    double v[15];
    int n = std::sscanf(line, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                        &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7], &v[8], &v[9],
                        &v[10], &v[11], &v[12], &v[13], &v[14]);
    if (n != 15) continue;
    unsigned long t = (unsigned long)v[0];
    if (!started) {
      g_now = t - firstDtMs; lastSampleTime = g_now;
      if (legacyConfig) {  // RuntimeConfig defaults before v3.11.30 (logs recorded with 3.11.28/3.11.29)
        RuntimeConfig c = MotionTracker::defaultConfig();
        c.maxVelocityFlatWindowSamples = 30; c.maxAccelerationFlatWindowSamples = 30;
        c.velocityOverrideFlatWindowSamples = 45;
        MotionTracker::setConfig(c);
      }
      if (appConfigSet) MotionTracker::setConfig(appConfig);
      MotionTracker::setTrackingActive(true);   // resets the engine (not the biases), prints REC_START + headers
      // accBiasVec is NOT reset by START: the harness seeds it after the reset
      accBiasVec[0] = (float)abx; accBiasVec[1] = (float)aby; accBiasVec[2] = (float)abz;
      if (joined) {
        // the state START would not have left: the session had been running
        // (still) for a while before the first logged row
        sessionT = (float)J[0];
        nextBracketId = (uint16_t)J[1]; nextPhaseId = (uint16_t)J[2];
        currentRepNumberEngine = (uint8_t)J[3];
        emaInitialized = J[4] != 0; emaRate = (float)J[5];
        velZFrameOffset = (float)J[6]; repCalibIntercept = (float)J[7];
        bracketStartT = sessionT; wasFlatCombined = true;
        for (uint8_t i = 0; i < FLAT_RING_CAPACITY; i++) { velLiveRing[i] = (float)J[8]; accZRing[i] = (float)J[9]; }
        ringFill = FLAT_RING_CAPACITY; ringHead = 0;
        for (uint16_t i = 0; i < BIAS_QUIET_RING_CAPACITY; i++) {
          gyroMagQuietRing[i] = (float)J[10]; accMagQuietRing[i] = (float)J[11];
        }
        quietRingFill = BIAS_QUIET_RING_CAPACITY; quietRingHead = 0;
        prevAccZDirected = (float)J[12]; trapezoidPrimedZ = true;
        shockPrevWz = (float)J[13]; shockPrevValid = true;
      }
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
    while (MotionTracker::peekCorrectedCurveChunk(c)) {
      if (dumpChunks) {
        std::printf("C,%u,%u,%u,%lu,%u", (unsigned)c.bracketId, (unsigned)c.chunkIndex, (unsigned)c.totalChunks,
                    (unsigned long)c.startTimestampMs, (unsigned)c.sampleCount);
        for (unsigned k = 0; k < c.sampleCount; k++) std::printf(",%.4f", c.correctedVelZ[k]);
        std::printf("\n");
      }
      MotionTracker::popCorrectedCurveChunk();
    }
  }
  MotionTracker::setTrackingActive(false);
  return 0;
}
