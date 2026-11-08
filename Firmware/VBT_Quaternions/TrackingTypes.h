// Part of the ODKI VBT firmware
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Lodovico Cortelazzo (ODKI)
//
// Licensed under the Apache License, Version 2.0 (the "License"); you may
// not use this file except in compliance with the License. You may obtain a
// copy of the License in LICENSE-APACHE in the repository root or at
// http://www.apache.org/licenses/LICENSE-2.0. Unless required by applicable
// law or agreed to in writing, software distributed under the License is
// distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
// KIND, either express or implied. See the License for the specific
// language governing permissions and limitations under the License.

#ifndef TRACKING_TYPES_H
#define TRACKING_TYPES_H

#include <Arduino.h>

// v3.12.0: the plain data types the tracking engine (MotionTracker,
// PolyForm Noncommercial - see LICENSE-ENGINE) exchanges with the rest of
// the firmware (BleServer, the .ino), moved here unchanged from
// MotionTracker.h so that the BLE layer, which only packs and unpacks them,
// does not depend on engine code. They describe the interface - what a
// sample, a rep, a corrected-curve block and the runtime configuration
// contain, as also specified byte for byte in BLE_PROTOCOL.md (CC0) - not
// how the engine computes them.

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
// CorrectedCurve). This is not a return to v3.10.x's ZUPT: this value is
// held at ~0 by moving the anchor, not by zeroing velZ; since v3.12.2 velZ
// is also shifted to 0 while confirmed still, but the anchor and the
// buffered bracket samples move with it, so the bracket correction math
// (drift = velZ - baseline) and this value are unchanged.
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
// v3.12.4: seven of these fields are FIRMWARE-ONLY - accZBiasIdleStillTimeS,
// gyroBiasIdleStillTimeS, accZBiasGyroMaxDegS, accZBiasAccMagToleranceMps2,
// emaAlpha, minCrossingExcursionMps, minCrossingDurationS. They keep their
// place in the 38-byte Config packet (same layout for every app version),
// but a value written over BLE is ignored: the compiled default always
// applies, and the Config Read echo shows it (lockFirmwareOnlyFields() in
// BleServer.cpp). Apps up to 1.0.5 re-sent stored values of these fields,
// which their UI does not expose. Each is marked FIRMWARE-ONLY below.
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
// repDirection are unchanged from v3.10.x (except the
// accZBiasIdleStillTimeS default, 0.05 -> 0.3 s in v3.12.3).
struct RuntimeConfig {
  RepDirection repDirection = RepDirection::Up;
  float maxPlausibleVelocityMps = 4.0f;
  // v3.12.3: 0.05 -> 0.3 s. With 0.05 s (5 samples) the accelerometer
  // bias (accBiasVec, MotionTracker.cpp) also learned DURING motion -
  // whenever the bar moved with near-constant acceleration and little
  // rotation - absorbing real acceleration: on a deadlift log worldAccZ
  // read -0.2..-0.4 m/s^2 in every pause, drift reached -1.5 m/s in one
  // bracket, a rep was lost and stillness was confirmed late. 0.3 s
  // (same as gyroBiasIdleStillTimeS): 10/10 reps, drift per bracket
  // +-0.07 m/s, lowest drift on the recorded squat sets too.
  // FIRMWARE-ONLY since v3.12.4: ignored when written over BLE, the
  // compiled default always applies (BleServer.cpp, lockFirmwareOnlyFields()).
  float accZBiasIdleStillTimeS = 0.3f;
  // FIRMWARE-ONLY since v3.12.4: ignored when written over BLE, the
  // compiled default always applies (BleServer.cpp, lockFirmwareOnlyFields()).
  float gyroBiasIdleStillTimeS = 0.3f;

  // --- bias (v3.11.0: now adjustable, previously compile-time constants) ---
  // Gyroscope magnitude threshold (degrees/s) below which a sample is
  // considered a candidate for the accZBias/gyroBias update.
  // FIRMWARE-ONLY since v3.12.4: ignored when written over BLE, the
  // compiled default always applies (BleServer.cpp, lockFirmwareOnlyFields()).
  float accZBiasGyroMaxDegS = 12.0f;
  // Half-band (m/s^2) of the excursion |total acceleration| must stay
  // within over a short sliding window for a sample to be considered a
  // candidate for the bias update (v3.11.11: a flatness test, no longer
  // an absolute distance from g - see the version note).
  // v3.11.0: tuned from 0.6 to 0.12 (full sweep on SQUAT_5/6, -14%/-43%
  // residual drift velocity) - see the version note.
  // FIRMWARE-ONLY since v3.12.4: ignored when written over BLE, the
  // compiled default always applies (BleServer.cpp, lockFirmwareOnlyFields()).
  float accZBiasAccMagToleranceMps2 = 0.12f;

  // --- Flat-guard (phase/bracket close on flatness) ---
  // Ceiling (m/s) on |velZLive| while a phase is open: below it the
  // flatness tests use the dynamic windows below, above it they switch to
  // the fixed velocityOverrideFlatWindowSamples window (v3.11.27 - before
  // that, the Flat-guard could NEVER trigger above it). Without
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
  // path and can't simply be dropped there. v3.12.8: accepted range
  // 10-40 samples (100-400ms, clamped in MotionTracker::setConfig(); was
  // 30-64); default unchanged at 30 - see the v3.12.8 version note.
  uint8_t velocityOverrideFlatWindowSamples = 30;
  // Excursion band (m/s) on the recent velocity window for it to be
  // considered "flat".
  float velocityFlatBandMps = 0.04f;
  uint8_t maxVelocityFlatWindowSamples = 20;
  uint8_t minVelocityFlatWindowSamples = 2;
  // Absolute band (m/s^2) on the recent acceleration window for it to be
  // considered "flat" (ALL samples in the window below the band).
  float accelerationFlatBandMps2 = 0.3f;
  uint8_t maxAccelerationFlatWindowSamples = 20;
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
  // FIRMWARE-ONLY since v3.12.4: ignored when written over BLE, the
  // compiled default always applies (BleServer.cpp, lockFirmwareOnlyFields()).
  float emaAlpha = 0.3f;

  // --- Rep-cycle calibration (independent of the bracket) ---
  // Minimum excursion (m/s, negative) required since a cycle started for
  // a negative->positive crossing to be considered a true cycle boundary
  // (complete rep), not noise.
  // FIRMWARE-ONLY since v3.12.4: ignored when written over BLE, the
  // compiled default always applies (BleServer.cpp, lockFirmwareOnlyFields()).
  float minCrossingExcursionMps = -0.03f;
  // Minimum duration (s) of a cycle for it to contribute to calibration.
  // FIRMWARE-ONLY since v3.12.4: ignored when written over BLE, the
  // compiled default always applies (BleServer.cpp, lockFirmwareOnlyFields()).
  float minCrossingDurationS = 0.3f;

  // --- Shock handling: landing impulse + orientation hold (v3.12.6) ---
  // FIRMWARE-ONLY, and NOT carried by the BLE Config packet at all (its
  // 38-byte layout is unchanged): packetToConfig() in BleServer.cpp starts
  // from RuntimeConfig{}, so these always keep the compiled defaults.
  // Tuned offline on recorded deadlift/squat sets (tools/full_replay/) -
  // see the v3.12.6 version note in MotionTracker.cpp and the "Mechanical
  // shocks" section of DOCUMENTATION.md.
  // false = no shock window, no orientation hold, no landing impulse
  // (the engine behaves exactly as without this layer).
  bool shockHandlingEnabled = true;
  // A shock window opens when the jerk of the unfiltered world-Z
  // acceleration (accelerometer projected on the PREVIOUS sample's gravity)
  // exceeds this (m/s^3)...
  float shockJerkOpenMps3 = 800.0f;
  // ...and closes after shockQuietSamples consecutive samples with jerk
  // below shockJerkQuietMps3 (once it is longer than shockQualifySamples),
  // or after shockMaxWindowSamples at most. While it is open the
  // orientation filter is not updated (the quaternion is held).
  float shockJerkQuietMps3 = 300.0f;
  uint8_t shockQuietSamples = 3;
  uint8_t shockMaxWindowSamples = 40;
  // Samples a landing candidate is held for before it is either confirmed
  // or released (its acceleration re-integrated unchanged).
  uint8_t shockQualifySamples = 6;
  // Landing = within the candidate: one sample with |acceleration| above
  // shockLandingPeakAccMps2, AND (shockLandingSustainSamples samples above
  // shockLandingSustainAccMps2 OR a raw gyroscope magnitude of at least
  // shockLandingGyroDegS), AND the bar was not rising at the window's
  // opening (live velocity below shockLandingMaxVPreMps, in the configured
  // concentric direction's frame). v3.12.7: +0.10 (was -0.05, then named
  // shockLandingMinDescentMps): a strong, sustained shock with the bar
  // still - a second knock right after a landing, the bar dropped onto the
  // floor at the end - is the bar on the floor too, and integrating its
  // vibration lost a rep (see the v3.12.7 version note in MotionTracker.cpp).
  float shockLandingPeakAccMps2 = 25.0f;
  float shockLandingSustainAccMps2 = 20.0f;
  uint8_t shockLandingSustainSamples = 2;
  float shockLandingGyroDegS = 100.0f;
  float shockLandingMaxVPreMps = 0.10f;

  // --- Re-level after a landing (v3.12.7) ---
  // Same status as the shock fields above: firmware-only, not in the BLE
  // Config packet. The bar really rotates when it lands, and the orientation
  // hold of the shock window throws that rotation away: after a landing the
  // attitude is re-seeded from the measured gravity at the first stillness.
  // A landing (shock column 3) arms it; on every later sample with the
  // orientation not held, a sample is "still" if the bias-corrected
  // gyroscope is below relevelGyroDegS AND the median-filtered |a| is
  // within relevelAccTolMps2 of accelScaleG. After relevelStillSamples
  // consecutive still samples, if the angle between the estimated gravity
  // and their mean accelerometer exceeds relevelMinDeg, the orientation
  // filter is re-seeded on that mean (FusionOrientation::alignToGravity());
  // either way the re-level is then disarmed. It is also disarmed, without
  // acting, relevelTimeoutS after the landing. Only a landing arms it.
  // false = no re-level; together with shockLandingMaxVPreMps = -0.05 the
  // shock layer behaves exactly as in v3.12.6.
  bool relevelEnabled = true;
  float relevelGyroDegS = 20.0f;
  float relevelAccTolMps2 = 0.4f;
  uint8_t relevelStillSamples = 8;
  float relevelTimeoutS = 3.0f;
  float relevelMinDeg = 0.5f;
};

#endif
