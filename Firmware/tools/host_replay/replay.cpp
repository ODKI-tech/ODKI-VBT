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
