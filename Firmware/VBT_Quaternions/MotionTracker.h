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

#ifndef MOTION_TRACKER_H
#define MOTION_TRACKER_H

#include <Arduino.h>

// v3.11.0: tracking engine replaced in its entirety (see the version note
// at the top of MotionTracker.cpp for the full diagnosis that led to this
// change). The old segment/ZUPT engine (boundary = confirmed stillness,
// then retroactive search for zero-crossings) is replaced by a "phase"
// engine (opened/closed for flatness, reversal, or timeout - IDENTICAL to
// how it was in firmware V2.0) + a "bracket" drift-correction layer: the
// integrated velocity always stays RAW (never zeroed/leaked), phases are
// closed when a bracket (= interval between two moments of confirmed
// stillness) closes, the drift measured at that moment is applied
// RETROACTIVELY and linearly over time to all samples in the bracket.
// Validated against 8 real hardware logs (STACCHI_1/2, SQUAT_1-6) with a
// Python simulator (sim_v2_drift.py) before being ported here - see the
// comments in the .cpp for the detail of each mechanism.

// Instantaneous phase state of ONE sample - see MotionSample::phaseState.
// v3.11.3: computed from the SIGN of refVelZ (with a small dead band),
// NOT from the open/closed state of the phase/bracket engine (that is
// deliberately debounced - it requires reversalConfirmSamples consecutive
// samples before confirming a reversal, see
// RuntimeConfig::reversalConfirmSamples - and by construction it lags
// behind a zero-crossing visible on the graph: reported by the user, who
// saw the eccentric phase still colored blue for a stretch after the
// curve had already dropped below zero). It can therefore oscillate for
// a couple of samples right at the true reversal point - it is meant
// ONLY for the live graph (coloring eccentric/concentric while moving),
// it is NOT the source of truth for the peak/mean reported at the end of
// a rep (that remains the phase/bracket engine).
enum class PhaseState : uint8_t {
  Idle = 0,       // sensor confirmed still (see confirmedStillNow)
  Eccentric = 1,  // negative refVelZ - descending
  Concentric = 2, // positive refVelZ - ascending
};

// Motion sample updated on every cycle (100Hz). linAcc* = acceleration in
// the sensor frame (gravity removed). world* = the same quantities in the
// world frame (fixed axes). refVelZ is the LIVE directional velocity
// (v3.11.0: no longer continuously zeroed/leaked by a ZUPT) - see the
// CorrectedCurve comment in BleServer.h for how/when the RETROACTIVELY
// corrected version for the same time interval arrives.
// v3.11.2: refVelZ is velZ (the raw integrator, never touched) RELATIVE
// to the measurement anchor (baselineRawVel) - this anchor is kept
// continuously aligned to velZ for the entire time the engine confirms
// the sensor is still (the same robust condition - two dynamic windows +
// an absolute ceiling on the already-corrected estimate - that already
// decides when to close phases/brackets, see the end of
// stepPhaseEngine() in MotionTracker.cpp), so it reads ~0 continuously
// during a real pause (not just for an instant) instead of staying stuck
// on a not-yet-converged bias residual. During motion it shows the raw
// delta from the last confirmed still reference - not yet corrected for
// THIS rep's drift (that's only known once the bracket closes, see
// CorrectedCurve). This is not a return to v3.10.x's ZUPT: velZ itself is
// NEVER zeroed, only the anchor used to reference this value - the
// bracket correction math (drift = velZ - baseline) remains identical.
// worldPosZ is a live integration of the same quantity (so with the same
// behavior), for the graph only - the authoritative data for each
// completed rep comes from peekCompletedRep().
struct MotionSample {
  float linAccX = 0, linAccY = 0, linAccZ = 0;
  float worldAccX = 0, worldAccY = 0, worldAccZ = 0;
  float worldVelX = 0, worldVelY = 0;
  float refVelZ = 0; // RAW live directional velocity, signed (see above)
  float worldPosX = 0, worldPosY = 0, worldPosZ = 0;
  float quatW = 1, quatX = 0, quatY = 0, quatZ = 0;
  PhaseState phaseState = PhaseState::Idle;
};

// Drift-correction status of a phase (eccentric or concentric) - see the
// version note at the top of MotionTracker.cpp. Increasing order of
// reliability: a phase reported as Provisional/RepCalibrated can be
// RE-reported (same repNumber, RepSummary again) as Corrected as soon as
// the bracket containing it actually closes.
enum class CorrectionStatus : uint8_t {
  Provisional = 0,   // corrected with the still-open bracket's EMA estimate
  RepCalibrated = 1, // corrected with the global calibration from rep cycles (no bracket available)
  Corrected = 2,      // corrected with a closed bracket's MEASURED drift - final
};

// Summary of ONE rep = a concentric phase + any eccentric phase preceding
// it in the same cycle (see RepDirection below for which direction is
// "concentric"). v3.11.0: a rep can be reported MULTIPLE TIMES with the
// same repNumber, as its phases move from Provisional/RepCalibrated to
// Corrected (see correctionStatus) - the receiver must overwrite by
// repNumber, not append.
// quality1: 0-100, kinematic coefficient (displacementM vs
// meanVelocity*duration, see the _score_phase equivalent in
// MotionTracker.cpp).
// correctionStatus: the worst (least reliable) of the eccentric phase's
// status and the concentric phase's status - see CorrectionStatus above.
struct RepResult {
  uint8_t repNumber = 0;
  float peakVelocity = 0;
  float meanVelocity = 0;
  float peakAcceleration = 0;
  float meanAcceleration = 0;
  float displacementM = 0;
  float eccPeakVelocity = 0;
  float eccMeanVelocity = 0;
  uint8_t quality1 = 0;
  CorrectionStatus correctionStatus = CorrectionStatus::Provisional;
};

// A block of CORRECTED velocity samples, ready to be sent over BLE (see
// CorrectedCurveChunkPacket in BleServer.h) - populated when a bracket
// closes (drift measured) by draining bracket_samples on the engine.
// startTimestampMs/sampleIntervalMs let the client recompute the
// timestamp of every sample without having to transmit it individually
// (see the chunking comment in BleServer.h).
struct CorrectedCurveChunk {
  uint16_t bracketId = 0;
  uint16_t chunkIndex = 0;
  uint16_t totalChunks = 0;
  uint8_t sampleCount = 0;
  uint32_t startTimestampMs = 0;
  uint16_t sampleIntervalMs = 10;
  static const uint8_t CAPACITY = 20;
  float correctedVelZ[CAPACITY] = {0};
};

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

// Direction of the phase tracked as a rep - see repDirectionSign in
// MotionTracker.cpp for how it's applied internally (multiplies the
// integrated velocity IMMEDIATELY after integration, before any decision
// by the algorithm - a single point of application, the rest of the
// engine always works on the already-oriented velocity).
enum class RepDirection : uint8_t {
  Up = 0,   // positive concentric phase (e.g. squat, bench press) - default
  Down = 1, // negative concentric phase (e.g. controlled descent/pure eccentric)
};

// Algorithm parameters adjustable at runtime via BLE (see the Config
// characteristic in BleServer.h/.cpp) - NOT persisted to flash: on every
// reboot the firmware starts again from the compiled DEFAULTS (see
// DEFAULT_CONFIG in MotionTracker.cpp); the app is responsible for
// re-sending the last configuration used on every new connection, if
// different from the defaults.
//
// v3.11.0: ALL the parameters of the new phase/bracket engine are here,
// validated against 8 real hardware logs with sim_v2_drift.py before
// porting (see the version note in MotionTracker.cpp). The old
// segment/ZUPT architecture's fields (velocityMagnitudeGateMps,
// slowRepPeakThresholdMps, minSegmentDurationS, minConcentricExcursionMps)
// have been REMOVED - no direct equivalent (minPhaseDurationS below takes
// over its role). shockJerkThresholdMps2 has also been removed: the
// anti-shock filter is not part of the algorithm validated with
// sim_v2_drift.py (the logs used for validation are pre-filter) - the
// bracket mechanism absorbs an isolated shock anyway in the next drift
// measurement.
// accZBiasIdleStillTimeS/gyroBiasIdleStillTimeS/maxPlausibleVelocityMps/
// repDirection are unchanged from v3.10.x.
struct RuntimeConfig {
  RepDirection repDirection = RepDirection::Up;
  float maxPlausibleVelocityMps = 4.0f;
  float accZBiasIdleStillTimeS = 0.05f;
  float gyroBiasIdleStillTimeS = 0.3f;

  // --- bias (v3.11.0: now adjustable, previously compile-time constants) ---
  // Gyroscope magnitude threshold (degrees/s) below which a sample is
  // considered a candidate for the accZBias/gyroBias update.
  float accZBiasGyroMaxDegS = 12.0f;
  // Tolerance (m/s^2) on the deviation between |total acceleration| and g
  // for a sample to be considered a candidate for the bias update.
  // v3.11.0: tuned from 0.6 to 0.12 (full sweep on SQUAT_5/6, -14%/-43%
  // residual drift velocity) - see the version note.
  float accZBiasAccMagToleranceMps2 = 0.12f;

  // --- Flat-guard (phase/bracket close on flatness) ---
  // Absolute ceiling (m/s) above which the Flat-guard can NEVER trigger
  // while a phase is open, regardless of the windows below - without
  // this, a long/heavy set can drift the raw velocity beyond any
  // sensible threshold and the mechanism locks up for the rest of the
  // session (diagnosed on SQUAT_6: only 1 flatness close in 24s). v3.11.0:
  // the thresholds below are evaluated on the velocity CORRECTED in real
  // time (see _live_predicted_offset in MotionTracker.cpp), no longer on
  // the raw value - this is the structural fix that makes the ceiling
  // usable without having to raise it.
  float flatGuardMaxVelocityMps = 0.20f;
  // v3.11.27: replaces flatGuardOverrideStillTimeS (removed - see the
  // version note in MotionTracker.cpp). That field was a safety net
  // against the SAME lockup this one addresses (the correction estimate
  // stuck outside flatGuardMaxVelocityMps even while the sensor is truly
  // still - e.g. after ground impact resets velZ but not the
  // already-accumulated offset), but it tested RAW gyroscope/acceleration
  // stillness over a full 1.0s window that had to be ENTIRELY clean - one
  // noisy sample from post-impact ringdown anywhere in that second forced
  // a full new second of waiting, so in practice it almost never fired.
  // This field takes the opposite approach: once phaseOpen and
  // fabs(velZLive) exceeds flatGuardMaxVelocityMps, BOTH flatness tests
  // below (velocityFlatBandMps/accelerationFlatBandMps2 - same bands,
  // unchanged) switch from the normal dynamic window (which shrinks to
  // just minVelocityFlatWindowSamples/minAccelerationFlatWindowSamples,
  // as low as 2 samples, once phasePeakVelocity passes
  // windowSaturationPeakVelocityMps) to this fixed, wider one instead -
  // still on velZLive/worldAccZ (the SAME smoothed, already-integrated
  // signal the normal path uses, not raw IMU noise), just over enough
  // samples that a genuine pause (which holds flat for hundreds of ms)
  // can't be confused with the momentary, few-millisecond
  // acceleration-crosses-zero instant that occurs at the PEAK of any
  // sufficiently fast rep by simple calculus (dv/dt=0 there too) - which
  // is exactly why the ceiling above exists for the normal dynamic-window
  // path and can't simply be dropped there. Configurable 30-60 samples
  // (300-600ms) from the app.
  uint8_t velocityOverrideFlatWindowSamples = 45;
  // Excursion band (m/s) on the recent velocity window for it to be
  // considered "flat".
  float velocityFlatBandMps = 0.04f;
  uint8_t maxVelocityFlatWindowSamples = 30;
  uint8_t minVelocityFlatWindowSamples = 2;
  // Absolute band (m/s^2) on the recent acceleration window for it to be
  // considered "flat" (ALL samples in the window below the band).
  float accelerationFlatBandMps2 = 0.3f;
  uint8_t maxAccelerationFlatWindowSamples = 30;
  uint8_t minAccelerationFlatWindowSamples = 2;
  // Velocity peak (m/s) beyond which the windows above saturate to the
  // minimum (fast reps require quicker flatness confirmation).
  float windowSaturationPeakVelocityMps = 0.8f;

  // --- Phase open/close ---
  float phaseStartVelocityMps = 0.2f;
  float minPhaseDurationS = 0.35f;
  float maxPhaseDurationS = 5.0f;
  uint8_t phaseLookbackSamples = 5;
  uint8_t reversalConfirmSamples = 3;

  // --- Bracket drift correction ---
  // Weight of a just-closed bracket's measurement on the EMA estimate of
  // the drift rate (used to provisionally correct phases that close
  // before their bracket closes).
  float emaAlpha = 0.3f;

  // --- Rep-cycle calibration (independent of the bracket) ---
  // Minimum excursion (m/s, negative) required since a cycle started for
  // a negative->positive crossing to be considered a true cycle boundary
  // (complete rep), not noise.
  float minCrossingExcursionMps = -0.03f;
  // Minimum duration (s) of a cycle for it to contribute to calibration.
  float minCrossingDurationS = 0.3f;
};

namespace MotionTracker {
  bool begin();

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
