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

#ifndef MOTION_TRACKER_H
#define MOTION_TRACKER_H

#include <Arduino.h>
#include "TrackingTypes.h" // the data types the rest of the firmware exchanges with the engine

// v3.11.0: tracking engine replaced in its entirety (see the version note
// at the top of MotionTracker.cpp for the full diagnosis that led to this
// change). The old segment/ZUPT engine (boundary = confirmed stillness,
// then retroactive search for zero-crossings) is replaced by a "phase"
// engine (opened/closed for flatness, reversal, or timeout - IDENTICAL to
// how it was in firmware V2.0) + a "bracket" drift-correction layer: the
// integrated velocity always stays RAW (never leaked; never zeroed up to
// v3.12.1 - since v3.12.2 its frame is shifted to 0 during confirmed
// stillness, which changes no difference the engine uses), phases are
// closed when a bracket (= interval between two moments of confirmed
// stillness) closes, the drift measured at that moment is applied
// RETROACTIVELY and linearly over time to all samples in the bracket.
// Validated against 8 real hardware logs (STACCHI_1/2, SQUAT_1-6) with a
// Python simulator (sim_v2_drift.py) before being ported here - see the
// comments in the .cpp for the detail of each mechanism.

// Velocity tracking state, exposed for diagnostics.
struct MotionDebugState {
  float gyroMag = 0;  // gyroscope magnitude, degrees/s
  float accZBias = 0; // current estimate of worldAccZ bias (m/s^2)
  // v3.11.7: this sample's velZ_live (velZ minus the current correction
  // estimate, see livePredictedOffset() in MotionTracker.cpp) - this is
  // the TRUE value the algorithm uses to decide phases/brackets,
  // captured at the moment of computation (BEFORE any continuous
  // re-alignment of the anchor on the same sample, see the end of
  // stepPhaseEngine()) - can diverge from refVelZ (always
  // anchor-relative) once calibration from rep cycles takes over from
  // the EMA estimate.
  float velZLive = 0;

  // Report ONLY the current sample's event (reset on every update()).
  // v3.11.19: |velZLive| exceeded the plausible maximum and was clamped -
  // was |velZ| (the raw, NEVER-touched integrator) through v3.11.18, see
  // the version note in MotionTracker.cpp for why that was wrong.
  bool velocityClampedToMax = false;
  bool bracketClosed = false; // a bracket closed on this sample (drift measured and applied retroactively)
  bool confirmedStillNow = false; // v3.11.2: the engine confirms stillness on this sample (== flatCombined) - drives the continuous re-alignment of the measurement anchor, see the end of stepPhaseEngine() in MotionTracker.cpp
  bool flatGuardOverrideFired = false; // v3.11.27: the Flat-guard triggered on this sample via the wide, fixed-window path (RuntimeConfig::velocityOverrideFlatWindowSamples) because fabs(velZLive) was above flatGuardMaxVelocityMps at the time - see stepPhaseEngine() in MotionTracker.cpp
};

namespace MotionTracker {
  bool begin();

  // v3.12.0: "FUSION", the only orientation filter (INFO/TIMING lines, the
  // read-only FILTER serial command).
  const char* orientationFilterName();

  // v3.11.33: serial log diagnostics. "INFO,..." (firmware version,
  // orientation filter - printed at every REC_START) and "TIMING,..." (micros() cost of
  // the filter update and of a whole sample since the last reset - printed
  // just before REC_STOP).
  void printInfoLine();
  // v3.12.16: "CONFIG," line - the BLE-writable RuntimeConfig fields (see the .cpp).
  void printConfigLine();
  void printTimingLine();
  void resetLoopTiming();

  // ~2s while still: aligns orientation to gravity. Blocking.
  void calibrateOrientation();
  bool isCalibrated();

  // Also resets velocity/position integration and phase/rep/bracket
  // state (clean new session).
  void setTrackingActive(bool active);
  bool isTrackingActive();

  // Applies a new runtime configuration (see RuntimeConfig above) - can
  // be called at any time, even while tracking is active (the new values
  // apply from the next sample). Does NOT persist to flash.
  void setConfig(const RuntimeConfig& config);
  const RuntimeConfig& currentConfig();
  // Factory-compiled values - used by the firmware at boot and available
  // to the app for a possible "reset to defaults" in the UI.
  const RuntimeConfig& defaultConfig();

  // Call on every loop(); self-limits to 100Hz INTERNALLY (if called more
  // often, the excess calls are no-ops and return false). The caller must
  // check the return value to know whether a REAL sample was computed,
  // and use it to gate anything that must happen "once per sample"
  // (decimated BLE streaming, etc.).
  bool update();

  const MotionSample& currentSample();

  // true if a rep is ready (new OR updated with a better correction
  // status, see CorrectionStatus) - copies the result WITHOUT removing it
  // from the internal queue (see popCompletedRep() below). More than one
  // rep can become ready on the same sample (e.g. a bracket closing
  // updates all the phases it covered to Corrected) - they stay in the
  // queue until consumed one at a time with popCompletedRep().
  bool peekCompletedRep(RepResult& outResult);
  // Removes the oldest rep from the queue (the one just returned by
  // peekCompletedRep()) - call ONLY after having delivered it
  // successfully. No-op if the queue is empty.
  void popCompletedRep();

  // Same peek/pop scheme as peekCompletedRep above, for corrected curve
  // blocks ready to be sent (see CorrectedCurveChunk above and the
  // CorrectedCurve comment in BleServer.h) - populated when a bracket
  // closes, drained one block at a time in the main loop.
  bool peekCorrectedCurveChunk(CorrectedCurveChunk& outChunk);
  void popCorrectedCurveChunk();

  const MotionDebugState& debugState();
}

#endif
