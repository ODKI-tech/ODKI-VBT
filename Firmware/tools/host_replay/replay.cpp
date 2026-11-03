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

// Replays a stream of vertical (world-Z) accelerations through the REAL tracking
// engine (MotionTracker.cpp, compiled unchanged on the PC) and prints the same
// S/R/B/P debug log the firmware prints over serial - so the output can be opened
// with tools/vbt_log_viewer.py. stdin: one "dt_ms,t_ms,filteredAccZ,integrationAccZ" line per
// sample: the median-filtered world-Z acceleration (flat-guard, reported acceleration) and the
// one integrated into velZ (the pre-median one since firmware v3.11.29). Two optional extra columns
// drive the experimental shock layer used by shock_experiments.py (nothing of this exists in the
// firmware): cmd = 1 closes the current bracket BEFORE the sample is processed, cmd = 2 re-anchors
// the velocity (baselineRawVel/bracketStartT, like the confirmed-still block) AFTER it; flag = 1
// marks a sample that belongs to a shock, so the estimators that last beyond a bracket (emaRate,
// the rep-cycle regression) are rolled back when they would learn from a bracket/cycle containing it.
// The orientation path (updateOrientationAndAcceleration) is not run, so the accRawX/Y/Z and
// gyroRawX/Y/Z columns of the printed S rows read 0; the P rows' t_start_ms/t_end_ms are
// rebuilt from the replayed t_ms/dt_ms exactly as on the device.
#include "MotionTracker.cpp"   // same TU on purpose: gives access to the engine's internals

SerialStub Serial;
WireStub Wire;
static unsigned long g_now = 0;
unsigned long millis() { return g_now; }

int main() {
  MotionTracker::setTrackingActive(true);   // resets the engine, prints REC_START + headers
  char line[256];
  bool bracketHadShock = false, cycleHadShock = false;
  while (std::fgets(line, sizeof line, stdin)) {
    double dtMs, tMs, a, aInt, cmd = 0, flag = 0;
    int n = std::sscanf(line, "%lf,%lf,%lf,%lf,%lf,%lf", &dtMs, &tMs, &a, &aInt, &cmd, &flag);
    if (n < 4) continue;
    g_now = (unsigned long)tMs;
    lastSampleTime = g_now;
    sample.worldAccZ = (float)a;
    lastWorldAccZUnfiltered = (float)aInt;

    if (cmd == 1 && bracketCount > 2) closeBracketFn();

    // state the estimators had before this sample, to roll back if it was a shock sample
    float saveEmaRate = emaRate; bool saveEmaInit = emaInitialized;
    uint16_t saveCnt = repCalibCount;
    float sSumT = repCalibSumT, sSumY = repCalibSumY, sSumTT = repCalibSumTT, sSumTY = repCalibSumTY;
    float sRate = repCalibRate, sIntercept = repCalibIntercept;
    float saveLastCrossT = lastCrossingT;
    if (flag == 1) { bracketHadShock = true; cycleHadShock = true; }

    integrateMotion((float)(dtMs / 1000.0));

    if (debugStateVar.bracketClosed) {
      if (bracketHadShock) { emaRate = saveEmaRate; emaInitialized = saveEmaInit; }
      bracketHadShock = false;
    }
    if (repCalibCount != saveCnt || lastCrossingT != saveLastCrossT) {
      // a rep cycle just ended: don't let it teach the regression if it contained a shock
      if (cycleHadShock && repCalibCount != saveCnt) {
        repCalibCount = saveCnt; repCalibSumT = sSumT; repCalibSumY = sSumY; repCalibSumTT = sSumTT;
        repCalibSumTY = sSumTY; repCalibRate = sRate; repCalibIntercept = sIntercept;
      }
      cycleHadShock = false;
    }
    if (cmd == 2) {
      baselineRawVel = velZ; bracketStartT = sessionT;
      if (repCalibCount >= 2) repCalibIntercept = velZ - repCalibRate * sessionT;
    }
    RepResult r;
    while (MotionTracker::peekCompletedRep(r)) MotionTracker::popCompletedRep();
    CorrectedCurveChunk c;
    while (MotionTracker::peekCorrectedCurveChunk(c)) MotionTracker::popCorrectedCurveChunk();
  }
  MotionTracker::setTrackingActive(false);
  return 0;
}
