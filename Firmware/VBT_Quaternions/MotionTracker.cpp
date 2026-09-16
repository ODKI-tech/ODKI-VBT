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

#include "MotionTracker.h"
#include "MadgwickAHRS.h"
#include "LSM6DS3.h"
#include "Wire.h"

namespace {
  // ============================================================================
  // CONFIGURATION - all the module's tunable time parameters and
  // thresholds, grouped here so they can be modified in a single place
  // without having to hunt through the implementation below.
  //
  // History v3.0.0-v3.10.6: concentric-phase tracking went from a
  // velocity-threshold state machine (v3.0.0-v3.9.3) to an architecture
  // of segments closed by ZUPT/confirmed-stillness with a retroactive
  // split (v3.10.0-v3.10.6, see git log for the full diagnosis of each
  // intermediate fix: v3.10.2 multi-rep-per-segment, v3.10.3 non-blocking
  // notify, v3.10.4 dead-stop segments, v3.10.5 dual post-peak ZUPT
  // threshold, v3.10.6 gyroscope bias separated from accZBias). That
  // architecture zeroed the integrated velocity (trueVelZ) on every
  // confirmed-stillness (ZUPT) event - effective against drift, but with
  // a structural limitation that was never solved: a long stretch
  // without ever a real pause (continuous set/touch-and-go) relied on
  // safety solely on an emergency leak, never validated to actually
  // correct the reported values, only to contain the damage.
  //
  // v3.11.0: ENGINE REPLACED IN ITS ENTIRETY with a "phase + bracket
  // drift correction" architecture, designed and validated offline
  // (Python, sim_v2_drift.py) against 8 real hardware logs (STACCHI_1/2,
  // SQUAT_1-6) before being ported here. Fundamental difference: the
  // integrated velocity (velZ below) is NEVER zeroed anymore - it always
  // remains the true raw integration of worldAccZ. In its place:
  //
  //   1) PHASE OPEN/CLOSE (like the old v3.0.0-v3.9.3 state machine, BUT
  //      evaluated on the velocity CORRECTED in real time (velZ_live
  //      below), not on the raw value - see point 3): a phase opens when
  //      |velZ_live| exceeds phaseStartVelocityMps, closes for 'flat'
  //      (combined Flat-guard: excursion window on velZ_live + absolute
  //      window on worldAccZ, both dynamically sized - they narrow as
  //      the phase's velocity peak grows, see computeDynamicWindowSamples()),
  //      for 'reversal' (direction reversal confirmed for
  //      reversalConfirmSamples consecutive samples) or for 'timeout'
  //      (maxPhaseDurationS).
  //
  //   2) "BRACKET" DRIFT CORRECTION: a bracket runs from a rising edge of
  //      the combined Flat-guard (not every sample where it's true - see
  //      wasFlatCombined/risingEdge in stepPhaseEngine, otherwise a
  //      prolonged real stillness would close a bracket on every sample)
  //      to the next one. On close, drift = velZ - baselineRawVel (the
  //      anchor left by the previous bracket, NOT absolute zero -
  //      guarantees continuity: every bracket starts exactly where the
  //      previous one left off, no "steps" in the corrected curve). The
  //      drift is distributed RETROACTIVELY and LINEARLY OVER TIME across
  //      all samples in the bracket (corrected(t) = raw(t) - baseline -
  //      drift*(t-t0)/(t1-t0)), and every phase fully contained in the
  //      bracket is re-scored on this corrected curve and reported with
  //      correctionStatus=Corrected (see RepResult in MotionTracker.h -
  //      this can be the SECOND time that rep is reported: the first
  //      time, at its own close, was provisional).
  //
  //   3) THE STRUCTURAL FIX (the explicit requirement that drove this
  //      port): the thresholds in point 1 are evaluated on velZ_live =
  //      velZ - livePredictedOffsetFn(), the BEST CURRENT ESTIMATE of
  //      correction (rep-cycle calibration if available, otherwise
  //      bracket-relative EMA, otherwise none), NEVER on the raw
  //      velocity. Without this, on a long/heavy set the raw velocity can
  //      drift past flatGuardMaxVelocityMps and the Flat-guard NEVER
  //      triggers again for the rest of the session (diagnosed on a real
  //      log: only 1 close in 24s) - bracket correction alone isn't
  //      enough if the mechanism that triggers it stays locked up. This
  //      estimate does NOT feed the drift MEASUREMENT (always raw velZ at
  //      bracket close, see closeBracketFn) nor the retroactive correction
  //      itself (always raw + measured drift/rate) - only the DECISION of
  //      when to trust a moment as stillness: a self-reinforcing loop (a
  //      wrong estimate that would validate itself) is avoided because a
  //      bracket only ever closes on a TRUE return close to the raw
  //      baseline.
  //
  //   4) REP-CYCLE CALIBRATION (checkZeroCrossingFn/refitRepCalibrationFn),
  //      independent of the bracket mechanism: every negative->positive
  //      crossing of the raw velocity (a descent followed by an ascent =
  //      a rep cycle) should have ~0 net displacement (you return to the
  //      same position), whatever that specific rep's velocity/duration
  //      is - unlike the bracket, it NEVER requires a real stillness.
  //      Every valid cycle contributes a point (mean time, local offset)
  //      to an incrementally maintained linear regression (rate+intercept),
  //      used when available (>=2 cycles) IN PREFERENCE to the bracket
  //      EMA to correct provisional phases - solves the tail case of
  //      sessions where brackets are rare/absent but reps keep completing.
  //
  // Concepts REMOVED in this change (no direct equivalent): trueVelZ/ZUPT
  // (velZ is no longer zeroed), segBuf/processSegment()/processRepWindow()
  // (replaced by bracketBuf/closePhaseFn/closeBracketFn), the
  // DRIFT_FALLBACK_* emergency leak (the bracket mechanism makes it
  // unnecessary), velocityMagnitudeGateMps/slowRepPeakThresholdMps/
  // minSegmentDurationS/minConcentricExcursionMps (minPhaseDurationS
  // takes over its role), the anti-shock filter
  // (SHOCK_MAX_CONSECUTIVE_REJECT_SAMPLES - not part of the validated
  // algorithm: the logs used for validation were pre-filter; an isolated
  // shock is reabsorbed anyway in the next drift measurement).
  //
  // v3.11.1: v3.11.0, by construction (velZ is never zeroed anymore), did
  // not bring the value shown on the live graph back to zero when the
  // sensor was truly still - a not-yet-converged bias residual stayed
  // visible indefinitely (it no longer GREW while still, since
  // worldAccZ~0, but it didn't return to zero on its own either).
  // Reported by the user. Fix, purely cosmetic: liveDisplayHold (see
  // stepPhaseEngine()) holds refVelZ/worldPosZ at zero while flatCombined
  // is true - the SAME robust condition (two dynamic windows + an
  // absolute ceiling, evaluated on the already-corrected estimate) that
  // already decides when to close phases/brackets, not the single
  // instantaneous-threshold comparison of the v3.10.x ZUPT that caused
  // the "true stillness vs. local velocity extremum" ambiguity diagnosed
  // in v3.10.0. velZ (measurement/bracket/rep-calibration) is NEVER
  // touched by this: the bracket correction math stays identical, only
  // the value transmitted in real time changes.
  //
  // v3.11.2: v3.11.1 only held the DISPLAYED value still (a freeze-frame
  // on the display, velZ/baselineRawVel untouched) - the user pointed out
  // that it's not just cosmetic: the NEXT rep must compute its own
  // correction starting from that same zero, not from a stale reference.
  // Fix: baselineRawVel/bracketStartT (the measurement anchor, not just
  // the display) now stay aligned to velZ for the ENTIRE duration that
  // flatCombined is true, not just at the rising edge handled by
  // closeBracketFn() - a long pause with a not-yet-converged bias residual
  // no longer leaves the anchor behind. Dual effect from the same
  // mechanism: the live value reads ~0 continuously (not just a
  // freeze-frame), AND the reference used to correct/evaluate the next
  // provisional phase stays always fresh. Does not touch velZ itself nor
  // the raw values buffered in bracketBuf (from which CorrectedCurve is
  // computed) - only the relative anchor they're referenced to. MUST run
  // AFTER closeBracketFn() in the same sample, never before (see the
  // comment in stepPhaseEngine()): that function measures drift by
  // comparing velZ against the OLD anchor.
  //
  // v3.11.3: bug reported by the user on the live graph - the eccentric
  // stretch of a rep stayed colored blue (concentric) for a while after
  // the curve had already visibly dropped below zero. Cause: the porting
  // of livePhaseState (live color, see PhaseState in MotionTracker.h) had
  // been hooked to the phase engine's open/closed state (phaseOpen/
  // phaseType) instead of the sign of the velocity as in the original
  // design - the engine's phase state is DELIBERATELY debounced (requires
  // reversalConfirmSamples consecutive samples past threshold before
  // confirming a reversal), so it lags behind the zero-crossing visible
  // on the graph. Fix: livePhaseState goes back to the sign of dirVel
  // (with a small dead band), Idle only when confirmedStillNow - no
  // impact on the phase/bracket engine itself (never the source of truth
  // for reps/corrections), only on the live graph's color.
  //
  // v3.11.5: bug reported by the user during a real recording, hard to
  // reproduce - sometimes the Flat-guard stops triggering even with the
  // sensor truly still, and correction stalls for the rest of the
  // session. Cause: the v3.11.0 fix evaluates the Flat-guard on velZ_live
  // (velZ minus the current correction estimate), no longer on the raw
  // velocity - correct, but the estimate itself (bracket EMA extrapolated
  // over time, or rep-cycle calibration) can accumulate enough error over
  // a long stretch without ever a closed bracket (e.g. a series of
  // touch-and-go reps with no real pauses) that velZ_live stays outside
  // the ceiling EVEN while the sensor is still - not because the sensor
  // is drifting, but because the estimate used to judge it is the wrong
  // one. Without a bracket closing, the estimate can never correct itself
  // on its own: a genuine stall, the same family of bug diagnosed in
  // v3.10.0/v3.11.0 but shifted one level (from the raw velocity to the
  // estimate that's supposed to correct it). Fix:
  // RuntimeConfig::flatGuardOverrideStillTimeS, a safety net on the SAME
  // RAW consecutive-stillness counter already used for
  // accZBiasIdleStillTimeS/gyroBiasIdleStillTimeS (gyroscope + total
  // acceleration, independent of any velocity estimate) - if it exceeds
  // this threshold (longer than the other two, it's a rare override, not
  // the primary mechanism), the Flat-guard triggers anyway, bypassing the
  // ceiling/windows on velZ_live. See stepPhaseEngine().
  //
  // v3.11.7: requested by the user - the raw data log (debugLogEnabled)
  // didn't allow reconstructing the CORRECTED curve (the one
  // CorrectedCurve sends over BLE): the "S," line (100Hz) only showed raw
  // velZ and refVelZ (anchor-relative, NEVER the retroactive correction
  // proportional to the drift - impossible in real time, a sample's
  // correction is only known once ITS bracket closes, in the future
  // relative to when that sample gets written), and there was no line per
  // bracket with drift/baseline/basisTotal. Added: a new "B," line on
  // every bracket close (bracketId, t0, t1, baseline, drift, rate,
  // updated emaRate, basisTotal) - with this plus the "S," lines in the
  // interval [t0,t1], an offline script can reconstruct exactly
  // corrected(t) = velZ(t) - baseline - drift*(t-t0)/basisTotal, the SAME
  // formula as enqueueCorrectedCurveChunks(). Also added
  // velZLive/repCalibRate/repCalibIntercept/repCalibCount to the "S,"
  // line: velZLive is the TRUE value the algorithm decides on (can
  // diverge from refVelZ once rep-cycle calibration takes over from the
  // EMA estimate - previously not visible in the log).
  //
  // v3.11.8: user-reported bug on a real capture - from a certain rep
  // onward (rep 14, in the diagnosed case) the reported meanVelocity/
  // meanAcceleration/quality1 collapsed to 0, while peakVelocity/
  // displacementM/eccPeak/eccMean stayed correct (they don't depend on
  // the issue below). Cause: in scorePhase(), the "does this sample count
  // toward the mean?" test (isOppositeDirectionForType) was evaluated on
  // the sign of the RAW s.velZ (never reset, by design - see v3.11.0
  // above) instead of the sign of cv (the already-corrected value,
  // computed just above in the same loop and already used for the peak).
  // With small accumulated raw drift relative to the phase's amplitude
  // the two signs almost always agree and the bug stays invisible; past a
  // threshold (in the diagnosed log, raw |baseline| beyond ~1.6 m/s,
  // reached naturally after a dozen reps in a set with higher-than-usual
  // drift) the raw sign stops matching the true direction of motion for
  // the entire concentric phase: every sample tests as "opposite
  // direction", meanCnt/accCnt stay at 0, and quality1 (which compares
  // displacement against mean*duration) collapses to 0 for lack of a
  // valid comparison term. Fix: the test now reads cv, not s.velZ - the
  // same convention already used everywhere else in the engine (flat-
  // guard, phase open/close: decisions are ALWAYS made on the corrected
  // estimate, never the raw one). Validated offline against the real log
  // that exposed the bug (reps 14-21 return to sane values, consistent
  // with peakVelocity) and against a clean regression session (no
  // practical difference on the already-healthy reps - the fix is a
  // no-op wherever the two signs already agreed).
  //
  // v3.11.8 (same fix, second log): a user reported an even more severe
  // failure on another real capture - past a certain rep (rep 16, in the
  // diagnosed case) rep counting stops ENTIRELY (no further "R," lines)
  // despite another ~20s of real motion (concentric/eccentric visible in
  // the log, brackets still closing normally). Cause: same pattern as the
  // fix above, but inside accumulateIntoOpenPhase() instead of
  // scorePhase() - the direction test feeding phaseLastSameDirIdx (used
  // as `cutoff` for REVERSAL-type closes in closePhaseFn, see there) also
  // read the raw s.velZ. With enough accumulated raw drift (here, a set
  // with no real pauses between reps - closes almost all reversal, not
  // flat), NO sample in the phase ever tested as "same direction":
  // phaseLastSameDirIdx stayed at -1 (its initial value), so cutoff=-1,
  // so the computed durationS came out to 0 (see closePhaseFn: tEnd=
  // tStart when cutoff<0), so durationS > minPhaseDurationS was always
  // false - the phase was discarded IN SILENCE (no reportPhaseScore, no
  // currentRepNumberEngine increment, no lastPhaseType update) for every
  // subsequent phase, indefinitely: a permanent stall for the rest of the
  // session, not a single lost sample. Fix: accumulateIntoOpenPhase() now
  // takes an explicit parameter (directionRef) carrying the ALREADY
  // corrected estimate to use for the direction test - velZLive when
  // called live from stepPhaseEngine() (the same estimate already used
  // there to detect the reversal itself, see oppositeAndSignificant -
  // previously two mechanisms decided direction two different ways on the
  // same phase), an approximation for the backfill samples in openPhase()
  // (the current sample's raw-minus-corrected offset, held constant over
  // the short backfill window - phaseLookbackSamples defaults to 5, ~50ms,
  // the correction estimate moves slowly on that scale). The only
  // quantity still read from the raw sample in accumulateIntoOpenPhase()
  // is magnitude (absV/phasePeakVelocity/phasePeakAcceleration, used only
  // to size the flat-guard windows, not any reported value) - deliberately
  // left untouched, no evidence it's broken. Validated offline against
  // the log that exposed THIS bug (counting resumes past rep 16 instead
  // of staying stuck) and re-validated against both earlier logs as a
  // regression check.
  //
  // v3.11.9: user-requested - reintroduced an anti-shock (jerk) filter on
  // worldAccZDirected in stepPhaseEngine(), identical in shape (hold-last-
  // value, cap at 4 consecutive samples) to the one present in the V1.0.0
  // engine through v3.10.6 and dropped in the v3.11.0 rewrite (assuming,
  // incorrectly for an isolated shock, that the bracket mechanism alone
  // would be enough - see v3.11.0 above for why it isn't: it corrects
  // drift LINEARLY over time, a shock is the exact opposite, a step
  // concentrated in a few samples). Tuned (RuntimeConfig::
  // shockJerkThresholdMps2 = 2.0) against a real capture of 8 squats with
  // progressively stronger isolated shocks on each rep.
  //
  // v3.11.10: REMOVED again - a second real capture (continuous shaking of
  // the sensor, not an isolated touch like the tuning session) showed the
  // filter making drift WORSE instead of better, to the point of driving
  // velZ into the safety clamp where, without the filter, it never got
  // close (confirmed offline: a from-scratch reproduction of the filter,
  // matching the real device's velZ byte-for-byte once the
  // RuntimeConfig::maxPlausibleVelocityMps clamp was also accounted for,
  // proved the filter was executing exactly as written - the problem was
  // in the design, not the code). Cause: every "shake" in that capture was
  // a pair of OPPOSITE worldAccZ spikes close together (up-down or down-
  // up) which, integrated with no filter at all, largely cancel each other
  // out on their own - the filter, by holding the previous value through
  // the FIRST spike of the pair (hold-last-value is asymmetric by
  // construction: it substitutes, it doesn't average), often swallows only
  // half of that natural cancellation, introducing a net drift that
  // accumulates with every shake instead of cancelling out. A filter
  // designed for one isolated shock (a single clean step, as in the tuning
  // session) can therefore make things worse on a pattern of closely-spaced
  // alternating shakes, which violates the filter's core assumption (2-4
  // anomalous samples, then back to normal). No immediate replacement: a
  // moving-average low-pass on velZ was evaluated offline as an
  // alternative (safe - doesn't introduce the same asymmetry, confirmed
  // not to worsen drift on the continuous-shake capture) but too weak at
  // short windows (2-10 samples) to meaningfully correct the isolated-
  // shock peak from the tuning session, and starts clipping the TRUE peaks
  // of un-shocked reps too once the window is widened enough to have a
  // real effect (~20+ samples, 200ms+).
  //
  // v3.11.11: fixed a bug in quietForBias (the gate that lets accZBias/
  // gyroBias reconverge - see the comment on accZBias below) that could
  // stay stuck almost indefinitely after a change of ORIENTATION (even
  // with no real motion at all), causing heavy velocity drift. Cause: the
  // second half of the condition compared accMag against the ABSOLUTE
  // constant G (fabs(accMag-G) < accZBiasAccMagToleranceMps2) - a test
  // that's only valid if accelScaleG (the accelerometer's isotropic
  // magnitude scale, fit ONCE in calibrateOrientation() and never touched
  // again, just like accZBias) perfectly corrects the real sensor at EVERY
  // orientation. A real MEMS accelerometer almost always has small per-
  // axis sensitivity mismatches that an isotropic scale can't correct: at
  // a resting orientation different from the one used at calibration time,
  // accMag can read stably >tolerance away from G while the sensor is
  // genuinely still (gyro confirmed quiet), blocking the gate indefinitely
  // and leaving accZBias frozen at a value inherited from a previous
  // session/orientation. Diagnosed on a real log (rotation only, no
  // translation at all - the case that isolates exactly this effect):
  // gyroMag confirmed stillness for the entire session (max 0.21 deg/s,
  // threshold 12 deg/s) while accZBias stayed bit-exact frozen for 8.8s,
  // driving velZ all the way to the safety clamp
  // (maxPlausibleVelocityMps). Fix: quietForBias no longer compares accMag
  // against G in absolute terms, it instead checks that accMag is FLAT
  // over a small sliding window (accMagBiasRing, 8 samples - same
  // principle as the flat-guard's absoluteFlat()/excursionFlat()), reusing
  // the existing config field (accZBiasAccMagToleranceMps2) as the
  // excursion half-band - no BLE protocol change. Robust by construction
  // to any resting orientation. Validated offline (Python reconstruction
  // of the gate, real per-sample dt from the log) against the log that
  // exposed the bug: with the fix, accZBias reconverges from the frozen
  // value (0.40) to the session's natural residual (~0.06) within a few
  // seconds (time constant ACCZ_BIAS_TAU_S=2s) instead of staying stuck,
  // taking velZ at the end of the still window from -3.03 (real, heading
  // toward the -4.0 clamp) to -0.71 (simulated with the fix).
  //
  // v3.11.12: the v3.11.11 fix above did NOT work on real hardware - a
  // second log (again pure rotation, no translation at all) showed
  // accZBias bit-exact at 0.0000 for the ENTIRE 14.7s session, with
  // gyroMag and worldAccZ (hence accMag) even more stable than in the
  // first log. Cause: excursionFlat()/ringAt() (see the ringAt() comment
  // below) hard-code FLAT_RING_CAPACITY (64) as the circular-index modulus
  // - they are NOT generic over the size of the ring passed in, an
  // implicit assumption that was never documented because until now they
  // had only ever been called on the 64-element rings owned by
  // stepPhaseEngine(). Calling them on the 8-element accMagBiasRing
  // introduced in v3.11.11 made ringAt() routinely read out of the array's
  // bounds (indices up to 63 into an 8-slot buffer) - unrelated adjacent
  // memory, excursion almost always out of band, gate effectively always
  // stuck. Missed by v3.11.11's offline validation because that Python
  // simulation computed the excursion with a direct min/max - correct by
  // construction, it never reproduced the real C++ code's out-of-bounds
  // indexing. Fix: quietForBias now computes the min/max directly over the
  // array (circular order doesn't matter for a plain range check), without
  // going through excursionFlat()/ringAt(). No other change: the flatness
  // logic itself stays exactly as described above in v3.11.11, only the
  // implementation was broken.
  //
  // v3.11.14: user-reported - velocity sometimes wouldn't reset to zero
  // even with velocity visibly flat and the sensor still, and when it did
  // arrive it came after a long, unpredictable delay (live graph stuck
  // "opaque", occasionally slightly below zero, well past
  // flatGuardOverrideStillTimeS=1.0s). Two contributing causes: 1) the
  // flat-guard's normal path checks an absolute ceiling on
  // velZLive = velZ - livePredictedOffset() (RuntimeConfig::
  // flatGuardMaxVelocityMps, see stepPhaseEngine()) - livePredictedOffset()
  // extrapolates emaRate LINEARLY over elapsed time since the last bracket
  // close, so even a slightly wrong emaRate (inherited from an atypical
  // bracket) diverges further the longer that stretch runs, and can push
  // velZLive outside the ceiling EVEN WHILE raw velZ (what the user
  // actually sees) is genuinely flat - blocking the primary path. 2) the
  // only independent way out, the override on accZBiasStillDuration
  // (flatGuardOverrideStillTimeS), was fragile: quietForBias reset the
  // counter COMPLETELY on the first violation, even an isolated one
  // (rack/floor vibration, a hand not perfectly still), forcing a full
  // wait from scratch - a much higher per-noisy-sample cost than the
  // sliding-window tests (excursionFlat/absoluteFlat) already used by the
  // primary path, which lose at most 'window' samples to a single
  // outlier. Together these explain both the intermittency (depends on how
  // "dirty" the last bracket was and on ambient noise at that moment) and
  // the "long delay" (the counter often had to restart from zero more than
  // once before reaching 1.0s). Fix (this version): cause 2, the easier one
  // to make robust without risking a regression in the already-validated
  // flatness logic. accZBiasStillDuration/accMagBiasRing (a cumulative
  // counter that reset COMPLETELY on the first violation, with only the
  // accMag half windowed and the gyroscope check instantaneous) are
  // REMOVED, replaced by the exact same sliding-window logic already used
  // by the primary path (excursionFlat/absoluteFlat, generalized to take
  // the ring's capacity as a parameter instead of assuming
  // FLAT_RING_CAPACITY - see ringAt() below) applied to raw gyroscope +
  // accelerometer magnitude too (gyroMagQuietRing/accMagQuietRing, see
  // below): an isolated outlier now costs at most 'window' samples, never
  // a full reset, for ALL THREE gates that depend on it
  // (accZBiasIdleStillTimeS, gyroBiasIdleStillTimeS,
  // flatGuardOverrideStillTimeS - each just a different window length over
  // the SAME two rings, instead of three thresholds on one shared counter).
  // Cause 1 above (why velZLive can diverge in the first place) remains a
  // deeper open problem - mitigated but not eliminated by the 6-position
  // accelerometer calibration (reduces emaRate's underlying error).
  //
  // v3.11.15: cause 1 from v3.11.14 above, tracked down from a real
  // capture (a bracket left open for 10.77s, reps 14-16 only getting
  // their corrected re-report once it finally closed, unstuck only by a
  // deliberate manual jolt from the user) - the cause in THIS log wasn't
  // the EMA as first suspected, but the rep-cycle calibration
  // (repCalibCount>=2, preferred over the EMA by livePredictedOffset() as
  // soon as it's available - so in practice the branch actually active for
  // nearly all of any session with more than 2 reps). Structural cause:
  // repCalibRate*sessionT+repCalibIntercept is a fixed regression on
  // ABSOLUTE session time, never re-anchored - unlike the EMA, which
  // already gets continuously re-anchored while the engine confirms
  // stillness (see v3.11.2 above, baselineRawVel/bracketStartT), so it
  // stays correct throughout a real pause; the rep-cycle calibration
  // doesn't, so it keeps "drifting" on its own (same intercept, but
  // sessionT keeps advancing) even while the sensor is genuinely
  // motionless, until a new rep cycle comes along to refit it. Fix: the
  // same re-anchoring already applied to the EMA, applied to
  // repCalibIntercept too - see the end of stepPhaseEngine(). Doesn't
  // touch the regression's accumulated sums (repCalibSumT/SumY/SumTT/
  // SumTY), so the next real rep cycle still refits it from scratch - this
  // only corrects the live reading during the current pause, it doesn't
  // affect the regression's long-term accuracy.
  //
  // v3.11.16: found during a code-quality review, not from a hardware
  // report - accumulateIntoOpenPhase() tracked phasePeakVelocity (which
  // sizes the flat-guard's dynamic windows via computeDynamicWindowSamples,
  // see the Flat-guard comment above) from fabs(s.velZ), the RAW sample -
  // this is the exact same failure shape already fixed twice in v3.11.8
  // (scorePhase's direction test, then accumulateIntoOpenPhase's OWN
  // direction test) for the SAME reason: s.velZ never resets, so once
  // accumulated raw drift is large relative to a phase's true velocity
  // amplitude, its magnitude stops meaning anything relative to the actual
  // motion. Concretely: on a long session with several m/s of accumulated
  // raw baseline drift (the same real-log scenario documented in v3.11.8),
  // a genuinely SLOW rep could see phasePeakVelocity inflated by the
  // drifted baseline alone, making computeDynamicWindowSamples() pick the
  // SHORT (min) windows meant for a fast rep - the flat-guard would then
  // need less real stillness than it should to confirm "flat", the
  // opposite of the intent (a slow rep can afford to wait longer to be
  // sure). The v3.11.8 fix note for the direction test explicitly left
  // this magnitude usage alone ("no evidence it's broken") - it is the
  // same root cause, just not yet observed to bite in practice. Fix:
  // accumulateIntoOpenPhase() now tracks phasePeakVelocity from
  // fabs(directionRef) - the same already-corrected estimate (velZLive
  // live, or its backfill approximation from openPhase()) already used for
  // the direction test right above it - instead of the raw sample.
  // phasePeakAcceleration is unaffected (still fabs(s.accZ)): raw
  // acceleration is not an integrated/drifting quantity, it has no
  // "corrected" counterpart to switch to.
  //
  // v3.11.17: user-reported - at a real stop the live velocity could sit
  // away from zero (e.g. ~0.20 m/s) for up to about a second before
  // resetting, well past what an athlete needs to move on to the next
  // phase. Cause: closePhaseFn() reset phaseVelWindowSamples/
  // phaseAccWindowSamples to the MAX window (maxVelocityFlatWindowSamples/
  // maxAccelerationFlatWindowSamples, 30 samples/300ms by default) on every
  // phase close - including a close caused BY flatness, i.e. the exact
  // moment the engine had just proven stillness using a much smaller
  // dynamic window (down to 2 samples for a fast rep, see
  // computeDynamicWindowSamples()). While idle (phaseOpen==false) the
  // ceiling test in stepPhaseEngine() is bypassed by construction (see
  // velIsFlatNow there), so flatCombined depended solely on
  // excursionFlat() over this now-oversized window - which still contained
  // the tail of the deceleration into the very stillness just confirmed,
  // so it kept evaluating false (blocking the baselineRawVel re-anchor,
  // see the end of stepPhaseEngine()) until those samples aged out of the
  // 30-sample lookback, up to ~300ms later. Worse, if raw drift crossed
  // phaseStartVelocityMps in that gap (both it and flatGuardMaxVelocityMps
  // default to the same 0.2 m/s), a spurious phase could reopen and reset
  // the window to max again, compounding the delay until the session fell
  // back on flatGuardOverrideStillTimeS (1.0s, independent of this
  // estimate) to unstick it - the ~1s the user observed. Fix:
  // closePhaseFn() (and resetTracking(), same pattern at session start)
  // now reset the windows to the MIN size (minVelocityFlatWindowSamples/
  // minAccelerationFlatWindowSamples) instead of the max - there is no
  // "peak velocity" context to size against while idle, and the goal at
  // that point is only to reconfirm stillness as fast as possible. A
  // genuine stop now re-anchors within about one sample interval (10ms)
  // instead of up to ~300ms, and no longer relies on
  // flatGuardOverrideStillTimeS for the normal case.
  //
  // v3.11.18: user-reported, traced to a real capture (debugLogEnabled) -
  // from a certain rep onward, every eccentric phase's data
  // (eccPeakVelocity/eccMeanVelocity) turned out to belong to the WRONG
  // repNumber, off by one, for the rest of the session (never
  // resynchronizing on its own). Root cause: closePhaseFn()'s rep-number
  // bookkeeping (currentRepNumberEngine/lastPhaseType) assumed phase closes
  // always strictly ALTERNATE direction (eccentric, concentric, eccentric,
  // concentric, ...) and only incremented currentRepNumberEngine on that
  // alternation. In the diagnosed log, ONE concentric phase closed and was
  // immediately followed by ANOTHER concentric phase in the same direction
  // (visible in the raw log as a single, smooth, uninterrupted ascent -
  // velocity climbing continuously with no reversal, no pause - silently
  // split into two consecutive "rep" numbers by the engine) - almost
  // certainly a spurious flat/reversal false-trigger from noise or a
  // borderline threshold, not a real direction change; closeReason isn't
  // logged today so the exact trigger of THIS specific split couldn't be
  // pinned down further. Whatever the trigger, the alternation assumption
  // has zero tolerance for it: once two same-direction phases close back to
  // back, lastPhaseType ends up misaligned with the physical eccentric/
  // concentric cycle, and every SUBSEQUENT eccentric close inherits the
  // wrong (previous) repNumber, permanently, for the rest of the session -
  // confirmed in the log from that point through its end (reps 4-20 in the
  // diagnosed capture). Fix: currentRepNumberEngine now increments
  // unconditionally whenever a CONCENTRIC phase closes (phaseType>0),
  // regardless of what phase type closed before it - a rep is complete once
  // its concentric phase closes, by definition, independent of whatever
  // else happened. lastPhaseType is removed entirely (was used nowhere
  // else). A future spurious same-direction split still produces one
  // "orphan" rep with no matching eccentric side (recognizable by
  // eccPeakVelocity/eccMeanVelocity staying 0) instead of corrupting every
  // rep reported afterward - fails safe instead of failing permanently.
  //
  // v3.11.19: user-reported, spotted from the same capture used to diagnose
  // v3.11.17/v3.11.18 - RuntimeConfig::maxPlausibleVelocityMps ("plausible
  // velocity" ceiling) was clamping velZ, the raw integrator, in place,
  // right after its trapezoidal integration step - not velZLive, the
  // corrected estimate. velZ is NEVER supposed to be touched (the entire
  // v3.11.0 design rests on it staying the true raw integral, drift
  // measured and corrected retroactively against it - see the version note
  // at the top of this file) - on a long/heavy or touch-and-go set it is
  // EXPECTED to run past any "plausible" single-rep bound as drift
  // accumulates, exactly as diagnosed in the capture (velZ pinned at
  // exactly 4.0000 for 15 consecutive samples - the clamp firing on
  // accumulated drift, not a real 4 m/s bar speed). Clamping it in place
  // silently corrupted every downstream raw-velZ consumer for as long as
  // the clamp was active that session: the sample stored in bracketBuf
  // (which scorePhase() reads directly to build the corrected curve),
  // closeBracketFn()'s drift measurement (drift = velZ - baseline, now
  // measuring a clipped value instead of the true accumulated drift), and
  // rawPosCumulative/checkZeroCrossing()'s rep-cycle calibration input -
  // none of which should ever see a value that isn't the faithful raw
  // integral. Fix: the clamp now applies to velZLive, computed a few lines
  // below its old location, immediately after velZLive itself is computed
  // and before anything reads it - velZLive is already documented (see
  // MotionDebugState::velZLive in MotionTracker.h) as "the TRUE value the
  // algorithm uses to decide phases/brackets", which is exactly the
  // quantity a plausibility ceiling is meant to guard: every phase-engine
  // decision downstream (flat-guard, phase open/close, reversal,
  // backfill) already reads velZLive, not velZ, so nothing else changes -
  // only which variable gets clipped, and velZ is no longer touched at
  // all. Same recurring failure shape already fixed twice before in this
  // file (v3.11.8's direction test, v3.11.16's peak-velocity tracking): a
  // raw, never-reset quantity used somewhere the corrected one belongs.
  //
  // v3.11.20: user-reported, traced to a real capture where the rep counter
  // stalled for over 13s (rep 17-19) despite continuous movement - several
  // concentric phases in that window closed with durationS just under
  // minPhaseDurationS, over and over, and got silently discarded. Root
  // cause, confirmed sample-by-sample against the log: every time
  // checkZeroCrossing() registered a new valid rep cycle and called
  // refitRepCalibration() (which happens roughly once per rep, whenever
  // repCalibCount>=2 is already the active estimate - the common case past
  // a session's first couple of reps), livePredictedOffset() could jump by
  // several tenths of a m/s in a SINGLE sample - not because the sensor did
  // anything, but because repCalibRate*sessionT+repCalibIntercept is
  // re-evaluated with the NEW fit at the CURRENT (large, accumulated)
  // sessionT: even a tiny change in repCalibRate from the refit
  // (thousandths of a unit) gets multiplied by sessionT and can dominate
  // the result - a well-known effect of evaluating a freshly-refit
  // regression far from the time window it was actually fit against.
  // Confirmed on 5 consecutive refits in the diagnosed capture: velZLive
  // dropped by 0.17-0.25 m/s in one 10-12ms sample EVERY time, always right
  // as the concentric phase was ramping up (raw velZ rising smoothly by
  // +0.10 to +0.16 in that same sample) - an artificial dip exactly where
  // the phase engine is most sensitive to direction/magnitude (reversal
  // detection, phaseLastSameDirIdx, the flat-guard ceiling). Fix: the same
  // continuity principle already used in stepPhaseEngine() for the
  // confirmed-still re-anchor (v3.11.15) - which re-anchors
  // repCalibIntercept so the live reading doesn't jump when the estimate
  // itself changes - is now ALSO applied at every refit, in
  // checkZeroCrossing(): repCalibIntercept is nudged, right after
  // refitRepCalibration() runs, so livePredictedOffset() evaluates to
  // EXACTLY what it was an instant before the refit, at the same
  // sessionT. Neither repCalibRate nor the accumulated regression sums are
  // touched - the fit itself, and its accuracy for the NEXT refit, are
  // unaffected; only the artificial step at the instant of THIS refit is
  // removed from the live reading. Validated by hand against the diagnosed
  // capture's 5 refits: with the patch, velZLive moves by the same amount
  // as raw velZ in that sample instead of jumping against it.
  //
  // v3.11.24: user-requested - RuntimeConfig::debugLogEnabled (a manually
  // toggled, BLE-only, RAM-only setting - see the app's Settings screen
  // before this version) is REMOVED. In practice it was a recurring source
  // of confusion: it lived only in RAM, so it silently reset to off on
  // every power cycle/USB reconnect (same as calibration), and there was
  // no way to drive it from tools/vbt_live_monitor.py at all, since it's a
  // Config-characteristic field and that tool has no BLE access - the only
  // way to see the raw 100Hz log was to first open the phone app over BLE
  // just to flip one switch, defeating the point of a serial-only
  // workflow. Replaced by a live check of the USB-serial connection
  // itself (serialLogActive() below, == `(bool)Serial`) - the SAME BLE
  // stream vs. raw-serial-log mutual exclusion as before (see point 3
  // below and the "Fast streaming..." comment in VBT_Quaternions.ino),
  // just automatic: plug in a serial reader and the raw log starts:
  // unplug it (or don't have one open) and the normal 20Hz BLE stream the
  // app's live graph uses takes over - matching the same StatusLED fix
  // from v3.11.23 (deviceConnected() there uses the identical `(bool)
  // Serial` check). No behavior on the wire changes for either mode
  // itself, only what selects between them. The RuntimeConfigPacket byte
  // that carried this flag is removed too (see BleServer.h) - the Config
  // characteristic shrinks from 40 to 39 bytes; firmware and app must be
  // updated together.
  //
  // v3.11.26: user-reported from a real capture (a heavy/slow deadlift rep
  // - STACCO_2.log) - velZLive started diverging from velZ about 5s into
  // the session and kept getting worse, reaching nearly DOUBLE the raw
  // value before snapping back once the rep finally reversed. Root cause,
  // confirmed by replaying the capture against both the current and a
  // patched livePredictedOffset()/closeBracketFn(): a 150ms bracket
  // (noise between two close reps) measured a rate of 0.54 m/s/s and, with
  // the SAME activeConfig.emaAlpha=0.3 used for a bracket of any length,
  // moved emaRate to 0.206 in one update; the very next bracket then
  // stayed open for 8.95s (a slow eccentric with no still pause long
  // enough to re-anchor it), during which livePredictedOffset() kept
  // extrapolating that 0.206 rate LINEARLY and WITHOUT LIMIT over the
  // growing elapsed time - by t=8.87s the predicted offset had reached
  // 0.90 m/s, all from a rate seeded by 150ms of noise. Two-part fix,
  // simulated against the real capture before implementing (drift-window
  // max error 1.61 -> 0.30 m/s, RMS 0.87 -> 0.27 m/s; the 145 samples
  // where velZLive exceeded 1.0 m/s while velZ itself stayed under 0.35
  // m/s - a purely algorithmic artifact, not real motion - dropped to
  // zero): (A) livePredictedOffset() caps the extrapolated elapsed time at
  // EMA_EXTRAPOLATION_CAP_S=2.5s - past that the offset freezes at its
  // best estimate instead of continuing to run away, and normal
  // re-anchoring still corrects it the moment a real reference point
  // shows up; (B) closeBracketFn()'s EMA update now scales
  // activeConfig.emaAlpha down for brackets shorter than
  // EMA_RATE_REFERENCE_DURATION_S=1.0s, so a 150ms bracket can no longer
  // move emaRate nearly as much as a full-length one - addresses the
  // error at its source (a bad rate being seeded) rather than only
  // bounding its consequence. Neither change touches the repCalibCount>=2
  // branch (already re-anchored per v3.11.15) or bracket open/close
  // timing itself (unaffected - risingEdge/checkZeroCrossing() are
  // unchanged).
  //
  // v3.11.27: v3.11.26 bounded the drift, but user-reported (a second
  // real deadlift capture, STACCO_3) it was still visibly wrong for
  // seconds after every rep - traced to the ground contact that's
  // unavoidable in a deadlift. Sequence found in the capture: the bar's
  // eccentric descent already has an offset (from the EMA/rep-calib
  // estimate accumulated earlier - v3.11.26 bounds how large this gets,
  // it doesn't prevent it) that's grown past flatGuardMaxVelocityMps
  // (0.20 m/s) by the time the bar makes contact; the impact itself
  // (worldAccZ up to +3.2 m/s^2 in the capture) is real and expected, and
  // velZ settles genuinely flat within under a second - but velZLive,
  // still carrying the pre-impact offset, never comes back under 0.20,
  // so the PRIMARY flat-guard path (velIsFlatNow's ceiling check) can
  // never recognize this genuine rest. The existing safety net for
  // exactly this case, flatGuardOverrideStillTimeS (v3.11.5), also never
  // fired even once in the whole capture: it requires a full, unbroken
  // 1.0s (100 samples) of raw gyroscope+accelerometer quiet, and
  // post-impact ringdown realistically scatters small noise samples
  // through that whole window - each one forces the FULL 1.0s to restart
  // from scratch (absoluteFlat() needs every one of the last N samples
  // under band), so in a bouncy/noisy real session it essentially never
  // completes. Considered (and rejected, confirmed by simulating it
  // against the SAME capture before implementing anything) simply
  // dropping the flatGuardMaxVelocityMps ceiling from the primary path
  // and relying only on the dynamic-window flatness tests: at high
  // phasePeakVelocity the dynamic window shrinks to
  // minVelocityFlatWindowSamples/minAccelerationFlatWindowSamples (as low
  // as 2 samples/20ms), and a smooth velocity curve's acceleration
  // crosses zero at every local peak by simple calculus - with only a
  // 2-sample window, that momentary, few-millisecond crossing at the TOP
  // of a fast rep is indistinguishable from a genuine stop. Confirmed
  // directly in the capture's own first rep: without the ceiling,
  // stepPhaseEngine() would have declared "flat" at t=1.423s with
  // velZLive=1.0065 m/s - the exact peak of a clean 1.27 m/s pull, not a
  // stop. So the ceiling stays on the fast/dynamic-window path (it's
  // load-bearing there, not redundant), but flatGuardOverrideStillTimeS
  // is REMOVED and replaced: once phaseOpen and fabs(velZLive) exceeds
  // flatGuardMaxVelocityMps, both flatness tests switch from the dynamic
  // window to RuntimeConfig::velocityOverrideFlatWindowSamples (fixed,
  // 30-60 samples/300-600ms, configurable from the app) - still on
  // velZLive/worldAccZ (the same smoothed signal the fast path already
  // uses, not raw IMU noise, so it isn't fooled by isolated post-impact
  // ringdown samples the way the raw-quiet override was), just over
  // enough samples that a real pause (which holds flat for hundreds of
  // ms) can't be confused with a fast rep's momentary peak (tens of ms).
  // See stepPhaseEngine() and RuntimeConfig::velocityOverrideFlatWindowSamples.
  // ============================================================================

  // --- Sampling / orientation (Madgwick) ---
  const unsigned long SAMPLE_INTERVAL_MS = 1000UL / 100; // 100Hz
  const float BETA_BASE = 0.1f;
  const float TRUST_FLOOR = 0.15f;
  const float TRUST_FALLOFF = 5.0f;

  // --- Anti-shock median filter on the raw accelerometer/gyroscope ---
  const uint8_t MEDIAN_FILTER_SIZE = 5;

  // --- worldAccZ residual bias, continuous estimate (see comment on accZBias) ---
  // v3.11.0: GYRO_MAX/ACCMAG_TOLERANCE were compile-time constants up to
  // v3.10.6, now adjustable (RuntimeConfig::accZBiasGyroMaxDegS/
  // accZBiasAccMagToleranceMps2) - validated with a full sweep on real
  // logs (default tuned 0.6->0.12 on the tolerance, -14%/-43% residual
  // drift velocity on two squat logs).
  const float ACCZ_BIAS_TAU_S = 2.0f;
  const float ACCZ_BIAS_MAX_STEP = 1.5f;
  // v3.11.14: shared capacity of gyroMagQuietRing/accMagQuietRing (see
  // declaration below) - the gates that read them (accZBiasIdleStillTimeS,
  // gyroBiasIdleStillTimeS) each use a different window length over the
  // SAME rings, so the capacity has to cover the longer of the two - 200
  // (2s) is generous headroom for a value set over BLE (clamped in
  // secondsToQuietSamples() below, otherwise it would read past the
  // array's bounds - the same bug as v3.11.12, impossible here by
  // construction). A third gate used to read these same rings too
  // (flatGuardOverrideStillTimeS, defaulting to 1.0s/100 samples) - see
  // the v3.11.27 version note near the top of the file for why it was
  // replaced with a fixed window over velLiveRing/accZRing instead.
  const uint16_t BIAS_QUIET_RING_CAPACITY = 200;

  // --- Gyroscope bias, CONTINUOUS estimate (see RuntimeConfig::gyroBiasIdleStillTimeS) ---
  const float GYRO_BIAS_TAU_S = 8.0f;

  // --- "Cosmetic" leak on worldVelX/Y ONLY (no role in phases/reps) ---
  const float VELOCITY_LEAK_XY_TAU_S = 1.8f;

  // --- Current bracket's buffer (see the v3.11.0 version note) ---
  // 3000 samples at 100Hz = 30s, well beyond the worst case observed
  // BEFORE the point-3 fix (24s) - a safety margin, not a limit meant to
  // ever actually be reached once the thresholds are evaluated on the
  // corrected velocity. Compacted on every bracket close (see
  // closeBracketFn) - RAM doesn't grow with session duration.
  const uint16_t BRACKET_BUFFER_CAPACITY = 3000;
  // Closed phases waiting for the bracket to re-score them (see
  // ClosedPhaseInfo) - 64 is comfortably above any realistic cadence even
  // in the worst case of a 30s bracket (30s/64 = 470ms per phase, faster
  // than any real rep).
  const uint8_t MAX_CLOSED_PHASES_PER_BRACKET = 64;
  // Queue of corrected-curve blocks ready to be sent over BLE (see
  // CorrectedCurve in BleServer.h) - sized for the worst case of ONE
  // bracket at maximum capacity (3000/CorrectedCurveChunk::CAPACITY = 150
  // blocks), with margin for a second burst before the queue has been
  // drained by the main loop.
  const uint16_t CHUNK_QUEUE_CAPACITY = 220;
  // Queue of reps ready to be reported over BLE (see RepResult) - a
  // closing bracket can update multiple reps at once.
  const uint8_t PENDING_REP_QUEUE_CAPACITY = 32;
  // Last scored state of every rep, indexed as a "sliding window" by
  // repNumber (see reportPhaseScore) - 24 reps is well beyond how many
  // can remain open/awaiting correction inside a single bracket even in
  // the worst case.
  const uint8_t REP_STORE_CAPACITY = 24;
  // Capacity of the Flat-guard sliding windows (velZHist/accZHist in
  // sim_v2_drift.py) - well beyond the default configurable maximum (30),
  // clamped in MotionTracker::setConfig().
  const uint8_t FLAT_RING_CAPACITY = 64;

  // --- Orientation calibration (CALIBRATE command, see calibrateOrientation()) ---
  const uint16_t CALIBRATION_SAMPLES = 200;
  const uint8_t CALIBRATION_SAMPLE_DELAY_MS = 10;
  const float CALIBRATION_GYRO_RANGE_WARN_DEG_S = 3.0f;
  const float CALIBRATION_OUTLIER_REJECT_DEG_S = 1.0f;
  const uint8_t CALIBRATION_OUTLIER_MAX_RETRIES = 5;

  const float G = 9.80665f; // standard gravitational acceleration, m/s^2 - not tunable

  LSM6DS3 imu(I2C_MODE, 0x6A);
  MadgwickAHRS ahrs;

  struct MedianFilter3 {
    float bufX[MEDIAN_FILTER_SIZE] = {0};
    float bufY[MEDIAN_FILTER_SIZE] = {0};
    float bufZ[MEDIAN_FILTER_SIZE] = {0};
    uint8_t nextIdx = 0;
    bool primed = false;

    static float median(const float buf[MEDIAN_FILTER_SIZE]) {
      float tmp[MEDIAN_FILTER_SIZE];
      for (uint8_t i = 0; i < MEDIAN_FILTER_SIZE; i++) tmp[i] = buf[i];
      for (uint8_t i = 1; i < MEDIAN_FILTER_SIZE; i++) {
        float key = tmp[i];
        int8_t j = (int8_t)i - 1;
        while (j >= 0 && tmp[j] > key) { tmp[j + 1] = tmp[j]; j--; }
        tmp[j + 1] = key;
      }
      return tmp[MEDIAN_FILTER_SIZE / 2];
    }

    void filter(float x, float y, float z, float& outX, float& outY, float& outZ) {
      if (!primed) {
        for (uint8_t i = 0; i < MEDIAN_FILTER_SIZE; i++) { bufX[i] = x; bufY[i] = y; bufZ[i] = z; }
        primed = true;
      }
      bufX[nextIdx] = x; bufY[nextIdx] = y; bufZ[nextIdx] = z;
      nextIdx = (nextIdx + 1) % MEDIAN_FILTER_SIZE;
      outX = median(bufX); outY = median(bufY); outZ = median(bufZ);
    }
  };
  MedianFilter3 accMedianFilter;
  MedianFilter3 gyroMedianFilter;

  struct FastSampleAccumulator {
    float sumAx = 0, sumAy = 0, sumAz = 0;
    uint32_t countA = 0;
    float sumGx = 0, sumGy = 0, sumGz = 0;
    uint32_t countG = 0;

    void poll() {
      uint8_t status = 0;
      if (imu.readRegister(&status, LSM6DS3_ACC_GYRO_STATUS_REG) != IMU_SUCCESS) return;
      if (status & 0x01) {
        uint8_t buf[6];
        if (imu.readRegisterRegion(buf, LSM6DS3_ACC_GYRO_OUTX_L_XL, 6) == IMU_SUCCESS) {
          sumAx += imu.calcAccel((int16_t)(buf[0] | (buf[1] << 8)));
          sumAy += imu.calcAccel((int16_t)(buf[2] | (buf[3] << 8)));
          sumAz += imu.calcAccel((int16_t)(buf[4] | (buf[5] << 8)));
          countA++;
        }
      }
      if (status & 0x02) {
        uint8_t buf[6];
        if (imu.readRegisterRegion(buf, LSM6DS3_ACC_GYRO_OUTX_L_G, 6) == IMU_SUCCESS) {
          sumGx += imu.calcGyro((int16_t)(buf[0] | (buf[1] << 8)));
          sumGy += imu.calcGyro((int16_t)(buf[2] | (buf[3] << 8)));
          sumGz += imu.calcGyro((int16_t)(buf[4] | (buf[5] << 8)));
          countG++;
        }
      }
    }

    void harvest(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) {
      if (countA > 0) {
        ax = sumAx / countA; ay = sumAy / countA; az = sumAz / countA;
      } else {
        ax = imu.readFloatAccelX(); ay = imu.readFloatAccelY(); az = imu.readFloatAccelZ();
      }
      if (countG > 0) {
        gx = sumGx / countG; gy = sumGy / countG; gz = sumGz / countG;
      } else {
        gx = imu.readFloatGyroX(); gy = imu.readFloatGyroY(); gz = imu.readFloatGyroZ();
      }
      sumAx = sumAy = sumAz = 0; countA = 0;
      sumGx = sumGy = sumGz = 0; countG = 0;
    }
  };
  FastSampleAccumulator fastSamples;

  // v3.11.13: accZBias used to be a SCALAR - the real residual isn't
  // constant, it depends on the current orientation (measured 0.33 near
  // the calibration orientation, 0.15 at a moderate tilt, on the SAME
  // log). Replaced with a fixed vector in the BODY frame (accBiasVec),
  // projected onto the current gravity direction (gravBodyX/Y/Z) to get
  // the correction to apply RIGHT NOW - so it adapts during motion too,
  // not just at rest. Single-observation LMS update (a direct
  // generalization of the previous scalar EMA: that's the special case
  // where the gravity direction never changes). Fixes the SYMPTOM in the
  // velocity domain using only the existing single-position calibration -
  // it does NOT correct the Madgwick orientation estimate itself (which
  // would use the same anisotropic raw accelerometer as its reference) -
  // that would need a multi-position, per-axis calibration.
  float accBiasVec[3] = {0, 0, 0};
  // v3.11.14: raw gyroscope/accelerometer magnitude, sliding-window
  // history - replaces accZBiasStillDuration/accMagBiasRing (a cumulative
  // counter that reset COMPLETELY on the first violation - see the
  // version note at the top of the file for the full diagnosis: it caused
  // a long, unpredictable delay in resetting velocity to zero whenever
  // there was isolated noise/vibration). Always updated in
  // updateOrientationAndAcceleration() (even before START) - CANNOT reuse
  // velLiveRing/accZRing from stepPhaseEngine(), which only run while
  // trackingActive. The gates that depend on it (accZBiasIdleStillTimeS,
  // gyroBiasIdleStillTimeS) each read a different window length over the
  // SAME two rings via excursionFlat()/absoluteFlat() (see
  // secondsToQuietSamples() and where they're used in
  // updateOrientationAndAcceleration()). A third gate used to read these
  // too (flatGuardOverrideStillTimeS, in stepPhaseEngine()) - removed in
  // v3.11.27, see the version note near the top of the file.
  float gyroMagQuietRing[BIAS_QUIET_RING_CAPACITY] = {0};
  float accMagQuietRing[BIAS_QUIET_RING_CAPACITY] = {0};
  uint8_t quietRingFill = 0;
  uint8_t quietRingHead = 0;

  unsigned long lastSampleTime = 0;
  bool trackingActive = false;
  bool calibrated = false;
  MotionSample sample;

  float lastGyroMag = 0;
  float lastGyroFilteredX = 0, lastGyroFilteredY = 0, lastGyroFilteredZ = 0;
  float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
  float accelScaleG = G;
  // v3.11.13: accBiasVec correction applied to the last sample (accBiasVec
  // projected onto that sample's gravBody) - integrateMotion() reads this
  // for the debug log, since it can't recompute it (gravBodyX/Y/Z are
  // local to updateOrientationAndAcceleration()).
  float lastAccZBiasApplied = 0;

  const RuntimeConfig DEFAULT_CONFIG;
  RuntimeConfig activeConfig;

  // +1 = Up (default), -1 = Down - see the comment on the RepDirection
  // declaration in MotionTracker.h. Applied ONCE ONLY, in
  // stepPhaseEngine(), right after the trapezoidal integration: the rest
  // of the engine always works on the already-oriented velocity (velZ).
  int8_t repDirectionSign = 1;

  // ============================================================================
  // PHASE/BRACKET ENGINE (v3.11.0) - see the version note above and
  // sim_v2_drift.py (validated offline before porting). The names
  // deliberately mirror those in the Python simulator so they can be
  // compared line by line.
  // ============================================================================

  // velZ: RAW vertical velocity, already sign-oriented (repDirectionSign
  // applied) - ALWAYS integrated, NEVER zeroed/leaked. Corresponds to
  // self.velZ in sim_v2_drift.py.
  float velZ = 0;
  float prevAccZDirected = 0; // for velZ's trapezoidal integration
  bool trapezoidPrimedZ = false;
  float prevVelZ = 0; // for checkZeroCrossingFn
  float rawPosCumulative = 0; // position integral, NEVER zeroed - see checkZeroCrossingFn

  float sessionT = 0; // seconds since tracking started (self.t in Python)
  unsigned long trackingStartMs = 0; // millis() at start - to reconstruct absolute timestamps for CorrectedCurve blocks

  // Sliding windows for the Flat-guard (velZHist/accZHist in Python) -
  // always grow in pairs (one push per sample), so they share the same
  // fill/head index.
  float velLiveRing[FLAT_RING_CAPACITY] = {0};
  float accZRing[FLAT_RING_CAPACITY] = {0};
  uint8_t ringFill = 0;
  uint8_t ringHead = 0;
  bool wasFlatCombined = false;

  // Current phase (open or none) - unlike sim_v2_drift.py (which keeps
  // an explicit list of sample_idxs/included per phase), here a phase's
  // indices are ALWAYS a contiguous range of bracketBuf (every sample
  // while the phase is open is accumulated exactly once, in order - see
  // accumulateIntoOpenPhase): only start/end need to be stored,
  // "included" is recomputed on the fly from bracketBuf[i].velZ.
  bool phaseOpen = false;
  int8_t phaseType = 0; // +1 concentric, -1 eccentric
  uint16_t phaseId = 0;
  uint8_t phaseRepNumber = 1;
  uint16_t phaseStartIdxInBracket = 0;
  int16_t phaseLastSameDirIdx = -1; // 0-based offset from phase start
  uint16_t phaseSampleCount = 0;
  float phasePeakVelocity = 0;
  float phasePeakAcceleration = 0;
  uint16_t phaseVelWindowSamples = 30;
  uint16_t phaseAccWindowSamples = 30;
  uint16_t oppositeDirStreakSamples = 0; // for the reversal - independent of which phase is open

  uint8_t currentRepNumberEngine = 1;
  uint16_t nextPhaseId = 0;

  struct BracketSample { float t; float dt; float velZ; float accZ; };
  BracketSample bracketBuf[BRACKET_BUFFER_CAPACITY];
  uint16_t bracketCount = 0;
  float bracketStartT = 0;
  uint16_t nextBracketId = 1;

  struct ClosedPhaseInfo {
    uint16_t id;
    uint8_t repNumber;
    int8_t phaseType;
    uint16_t startIdx, endIdx; // inclusive, indices into bracketBuf
    int16_t cutoff; // 0-based offset from startIdx
    uint16_t discarded;
    float durationS;
    uint8_t closeReason; // 0=flat, 1=reversal, 2=timeout
  };
  ClosedPhaseInfo closedPhases[MAX_CLOSED_PHASES_PER_BRACKET];
  uint8_t closedPhaseCount = 0;

  float emaRate = 0;
  bool emaInitialized = false;
  float baselineRawVel = 0; // raw anchor: the previous bracket has already reported THIS value to 0

  // Rep-cycle calibration - linear regression sums maintained
  // INCREMENTALLY (mathematically equivalent to the from-scratch
  // recompute over all points done by
  // sim_v2_drift.py._refit_rep_calibration, without having to keep the
  // full list).
  float lastCrossingT = -1; // <0 = none yet (None in Python)
  float lastCrossingRawPos = 0;
  float sinceCrossingMin = 0;
  uint16_t repCalibCount = 0;
  float repCalibSumT = 0, repCalibSumY = 0, repCalibSumTT = 0, repCalibSumTY = 0;
  float repCalibRate = 0, repCalibIntercept = 0;

  CorrectedCurveChunk chunkQueue[CHUNK_QUEUE_CAPACITY];
  uint16_t chunkHead = 0, chunkCount = 0;

  // Last scored state of every rep, to combine eccentric+concentric into
  // a single RepResult (see reportPhaseScore) - indexed as a "sliding
  // window" by repNumber (repNumber-1) % REP_STORE_CAPACITY: repNumber
  // keeps growing for the whole session, this mapping stays correct as
  // long as no more than REP_STORE_CAPACITY reps are "pending" (open or
  // awaiting correction) at the same time - see the constant above.
  struct RepSlot {
    bool used = false;
    uint8_t repNumber = 0;
    bool hasEcc = false, hasCon = false;
    float eccPeakVel = 0, eccMeanVel = 0;
    float conPeakVel = 0, conMeanVel = 0, conPeakAcc = 0, conMeanAcc = 0, conDisp = 0;
    uint8_t quality1 = 0;
    CorrectionStatus eccStatus = CorrectionStatus::Provisional;
    CorrectionStatus conStatus = CorrectionStatus::Provisional;
  };
  RepSlot repStore[REP_STORE_CAPACITY];

  RepResult pendingReps[PENDING_REP_QUEUE_CAPACITY];
  uint8_t pendingCount = 0;
  uint8_t pendingHead = 0;

  MotionDebugState debugStateVar;

  // --- Live streaming/display (no role in phases/reps) ---
  float refVelZ = 0;
  PhaseState livePhaseState = PhaseState::Idle;
  float livePosZ = 0;
  float prevDirVelForLivePos = 0;

  // --- X/Y: streaming/debug only, no role in phases/reps ---
  bool trapezoidPrimed = false;
  float prevWorldAccX = 0, prevWorldAccY = 0;
  float prevWorldVelX = 0, prevWorldVelY = 0;
  float worldVelX = 0, worldVelY = 0;
  float worldPosX = 0, worldPosY = 0;

  // ---- helper: sliding windows (see _excursion_flat/_absolute_flat in sim_v2_drift.py) ----
  // v3.11.14: 'capacity' is now an explicit parameter, no longer
  // implicitly assumed equal to FLAT_RING_CAPACITY - see the v3.11.12
  // version note at the top of the file for the bug that implicit
  // assumption caused (ringAt() reading past the bounds of a
  // differently-sized ring). Generalized specifically so it can be shared
  // with the accZBias/gyroBias/flatGuardOverrideStillTimeS gates (see
  // gyroMagQuietRing/accMagQuietRing below), which need windows much
  // longer than FLAT_RING_CAPACITY=64.
  float ringAt(const float* ring, uint8_t head, uint16_t capacity, uint16_t iFromEnd) {
    uint16_t idx = (uint16_t)((head + capacity - 1 - iFromEnd) % capacity);
    return ring[idx];
  }

  bool excursionFlat(const float* ring, uint8_t fill, uint8_t head, uint16_t capacity, uint16_t n, float band) {
    if (n == 0 || fill < n) return false;
    float mn = 1e9f, mx = -1e9f;
    for (uint16_t i = 0; i < n; i++) {
      float v = ringAt(ring, head, capacity, i);
      if (v < mn) mn = v;
      if (v > mx) mx = v;
    }
    return (mx - mn) <= 2.0f * band;
  }

  bool absoluteFlat(const float* ring, uint8_t fill, uint8_t head, uint16_t capacity, uint16_t n, float band) {
    if (n == 0 || fill < n) return false;
    for (uint16_t i = 0; i < n; i++) {
      if (fabs(ringAt(ring, head, capacity, i)) > band) return false;
    }
    return true;
  }

  uint16_t computeDynamicWindowSamples(float peakVelocity, uint16_t minSamples, uint16_t maxSamples, float saturation) {
    if (peakVelocity <= 0.0f) return maxSamples;
    if (peakVelocity >= saturation) return minSamples;
    float fraction = peakVelocity / saturation;
    float value = (float)maxSamples - fraction * (float)((int)maxSamples - (int)minSamples);
    int result = (int)(value + 0.5f);
    if (result < (int)minSamples) result = minSamples;
    if (result > (int)maxSamples) result = maxSamples;
    return (uint16_t)result;
  }

  // v3.11.14: converts a duration (RuntimeConfig::accZBiasIdleStillTimeS/
  // gyroBiasIdleStillTimeS/flatGuardOverrideStillTimeS, seconds) into the
  // matching sample count at 100Hz, clamped to BIAS_QUIET_RING_CAPACITY -
  // without this clamp, a high value set over BLE would make
  // excursionFlat()/absoluteFlat() read past the bounds of
  // gyroMagQuietRing/accMagQuietRing (the same bug as v3.11.12, impossible
  // here by construction).
  uint16_t secondsToQuietSamples(float seconds) {
    int32_t n = (int32_t)(seconds * (1000.0f / (float)SAMPLE_INTERVAL_MS) + 0.5f);
    if (n < 1) n = 1;
    if (n > (int32_t)BIAS_QUIET_RING_CAPACITY) n = BIAS_QUIET_RING_CAPACITY;
    return (uint16_t)n;
  }

  // v3.11.15: factors out the reset of gyroMagQuietRing/accMagQuietRing,
  // otherwise duplicated identically in resetTracking() and
  // calibrateOrientation().
  void resetQuietRings() {
    quietRingFill = 0; quietRingHead = 0;
    for (uint16_t i = 0; i < BIAS_QUIET_RING_CAPACITY; i++) { gyroMagQuietRing[i] = 0; accMagQuietRing[i] = 0; }
  }

  // v3.11.24: replaces RuntimeConfig::debugLogEnabled (removed - see the
  // version note at the top of this file) - whether the 100Hz raw log
  // prints is now read live from the USB connection itself instead of a
  // manually-set, BLE-only flag. `Serial` (Adafruit_USBD_CDC's native-USB
  // connection) is truthy exactly when a host has the port open (DTR
  // asserted) - the same check `deviceConnected()` in VBT_Quaternions.ino
  // uses for the status LED, and the same one already used at the top of
  // setup() to wait for a terminal. Checked fresh at every call site
  // (cheap - a single flag read), so the raw log starts/stops exactly
  // when a serial connection appears/disappears, with no separate on/off
  // step needed from either side.
  bool serialLogActive() {
    return (bool)Serial;
  }

  // v3.11.15: factors out the pattern repeated 3 times
  // (accZBiasIdleStillTimeS/gyroBiasIdleStillTimeS in
  // updateOrientationAndAcceleration(), flatGuardOverrideStillTimeS in
  // stepPhaseEngine()) - the same absoluteFlat/excursionFlat pair over the
  // same two rings, only the window duration changes.
  bool quietForDuration(float seconds) {
    uint16_t n = secondsToQuietSamples(seconds);
    return absoluteFlat(gyroMagQuietRing, quietRingFill, quietRingHead, BIAS_QUIET_RING_CAPACITY,
                        n, activeConfig.accZBiasGyroMaxDegS) &&
           excursionFlat(accMagQuietRing, quietRingFill, quietRingHead, BIAS_QUIET_RING_CAPACITY,
                         n, activeConfig.accZBiasAccMagToleranceMps2);
  }

  bool isOppositeDirectionForType(int8_t pType, float v) {
    if (pType > 0) return v < 0.0f;
    if (pType < 0) return v > 0.0f;
    return false;
  }

  struct PhaseScore { float peak = 0, mean = 0, peakAcc = 0, meanAcc = 0, disp = 0; uint8_t quality1 = 0; };

  // Recomputes peak/mean/displacement/quality1 for a closed phase
  // [startIdx..endIdx] (inclusive, in bracketBuf) - see _score_phase in
  // sim_v2_drift.py. corrected=true uses the MEASURED drift
  // (baseline/drift/posT0/basisTotal of the just-closed bracket);
  // corrected=false uses the provisional EMA estimate (bracket-relative)
  // UNLESS useRepCalib is true, in which case it uses the global
  // rep-cycle calibration.
  PhaseScore scorePhase(uint16_t startIdx, uint16_t endIdx, int16_t cutoff, float durationS,
                        uint8_t closeReason, int8_t pType, bool corrected, bool useRepCalib,
                        float baseline, float drift, float posT0, float basisTotal) {
    PhaseScore r;
    float meanNum = 0, accNum = 0, dispAcc = 0, prevCv = 0;
    uint32_t meanCnt = 0, accCnt = 0;
    for (uint16_t i = startIdx; i <= endIdx; i++) {
      const BracketSample& s = bracketBuf[i];
      float cv;
      if (useRepCalib) {
        cv = s.velZ - (repCalibRate * s.t + repCalibIntercept);
      } else if (corrected) {
        float pos = s.t - posT0;
        cv = (basisTotal > 1e-9f) ? (s.velZ - baseline - drift * pos / basisTotal) : (s.velZ - baseline);
      } else {
        float pos = s.t - bracketStartT;
        cv = s.velZ - baseline - emaRate * pos;
      }
      // v3.11.8: direction-inclusion test moved from the raw sample (s.velZ)
      // to the corrected value (cv) computed just above - see the version
      // note near the top of this file for the full diagnosis. s.velZ never
      // resets, so once accumulated raw drift exceeds a phase's own real
      // velocity amplitude, its sign stops matching the true direction of
      // motion (cv's sign, which is what the rest of the algorithm treats as
      // ground truth everywhere else) - every sample in the phase would then
      // wrongly test as "opposite direction", meanCnt/accCnt would stay at
      // 0, and mean/meanAcc/quality1 would all silently report 0 despite
      // peak/displacement (which don't depend on `included`) staying
      // correct. Diagnosed from a real capture where this happened from
      // rep 14 onward once cumulative raw drift passed ~1.6 m/s.
      bool included = !isOppositeDirectionForType(pType, cv);
      float av = fabs(cv);
      if (av > r.peak) r.peak = av;
      float aa = fabs(s.accZ);
      if (aa > r.peakAcc) r.peakAcc = aa;

      int32_t iOff = (int32_t)i - (int32_t)startIdx;
      if (iOff <= (int32_t)cutoff) {
        if (included) { meanNum += av; meanCnt++; accNum += aa; accCnt++; }
        dispAcc += (cv + prevCv) * 0.5f * s.dt;
        prevCv = cv;
      }
    }
    r.mean = meanCnt ? (meanNum / (float)meanCnt) : 0.0f;
    r.meanAcc = accCnt ? (accNum / (float)accCnt) : 0.0f;
    r.disp = fabs(dispAcc);

    float expectedDisp = r.mean * durationS;
    float denom = fmax(fabs(r.disp), fmax(fabs(expectedDisp), 0.001f));
    float relErr = fabs(r.disp - expectedDisp) / denom;
    if (closeReason == 2) {
      r.quality1 = 0;
    } else {
      float q = 100.0f - fmin(1.0f, relErr) * 100.0f;
      if (q < 0.0f) q = 0.0f;
      r.quality1 = (uint8_t)(q + 0.5f);
    }
    return r;
  }

  void pushOrUpdatePendingRep(const RepResult& r) {
    for (uint8_t i = 0; i < pendingCount; i++) {
      uint8_t idx = (pendingHead + i) % PENDING_REP_QUEUE_CAPACITY;
      if (pendingReps[idx].repNumber == r.repNumber) {
        pendingReps[idx] = r;
        return;
      }
    }
    if (pendingCount >= PENDING_REP_QUEUE_CAPACITY) return; // safety net, should never happen
    uint8_t tail = (pendingHead + pendingCount) % PENDING_REP_QUEUE_CAPACITY;
    pendingReps[tail] = r;
    pendingCount++;
  }

  // Combines ONE phase's score (eccentric or concentric) with the other,
  // already-known side of the same rep (see RepSlot) and, if the
  // concentric side is present, (re-)enqueues a complete RepResult for
  // BLE sending - see the RepSummary comment in BleServer.h for how the
  // client must handle a repNumber sent multiple times.
  void reportPhaseScore(uint8_t repNumber, int8_t pType, CorrectionStatus status,
                        float peak, float mean, float peakAcc, float meanAcc, float disp, uint8_t quality1) {
    RepSlot& slot = repStore[(repNumber - 1) % REP_STORE_CAPACITY];
    if (!(slot.used && slot.repNumber == repNumber)) {
      slot = RepSlot();
      slot.used = true;
      slot.repNumber = repNumber;
    }
    if (pType > 0) {
      slot.hasCon = true;
      slot.conPeakVel = peak; slot.conMeanVel = mean;
      slot.conPeakAcc = peakAcc; slot.conMeanAcc = meanAcc; slot.conDisp = disp;
      slot.quality1 = quality1;
      slot.conStatus = status;
    } else {
      slot.hasEcc = true;
      slot.eccPeakVel = peak; slot.eccMeanVel = mean;
      slot.eccStatus = status;
    }
    if (slot.hasCon) {
      RepResult r;
      r.repNumber = repNumber;
      r.peakVelocity = slot.conPeakVel; r.meanVelocity = slot.conMeanVel;
      r.peakAcceleration = slot.conPeakAcc; r.meanAcceleration = slot.conMeanAcc;
      r.displacementM = slot.conDisp;
      r.eccPeakVelocity = slot.hasEcc ? slot.eccPeakVel : 0.0f;
      r.eccMeanVelocity = slot.hasEcc ? slot.eccMeanVel : 0.0f;
      r.quality1 = slot.quality1;
      CorrectionStatus worse = slot.conStatus;
      if (slot.hasEcc && (uint8_t)slot.eccStatus < (uint8_t)worse) worse = slot.eccStatus;
      r.correctionStatus = worse;

      if (serialLogActive()) {
        Serial.print("R,"); Serial.print(r.repNumber); Serial.print(',');
        Serial.print(r.peakVelocity, 4); Serial.print(',');
        Serial.print(r.meanVelocity, 4); Serial.print(',');
        Serial.print(r.peakAcceleration, 4); Serial.print(',');
        Serial.print(r.meanAcceleration, 4); Serial.print(',');
        Serial.print(r.displacementM, 4); Serial.print(',');
        Serial.print(r.eccPeakVelocity, 4); Serial.print(',');
        Serial.print(r.eccMeanVelocity, 4); Serial.print(',');
        Serial.print(r.quality1); Serial.print(',');
        Serial.println((uint8_t)r.correctionStatus);
      }

      pushOrUpdatePendingRep(r);
    }
  }

  // v3.11.8: directionRef is the CORRECTED velocity estimate for this
  // sample (velZLive when called live from stepPhaseEngine(), an
  // approximation of it for backfilled samples - see openPhase()) - used
  // for the direction test below (phaseLastSameDirIdx) AND (v3.11.16, see
  // the version note near the top of this file) for magnitude tracking
  // (absV/phasePeakVelocity/phasePeakAcceleration, which size the
  // flat-guard windows via computeDynamicWindowSamples()). s.accZ (raw
  // acceleration, not a drift-integrated quantity - has no equivalent
  // "corrected" version) is still read directly for phasePeakAcceleration.
  void accumulateIntoOpenPhase(uint16_t idxInBracket, float directionRef) {
    const BracketSample& s = bracketBuf[idxInBracket];
    float absV = fabs(directionRef);
    bool included = !isOppositeDirectionForType(phaseType, directionRef);
    uint16_t offset = phaseSampleCount;
    if (included) {
      phaseLastSameDirIdx = (int16_t)offset;
    }
    phaseSampleCount++;

    if (absV > phasePeakVelocity) {
      phasePeakVelocity = absV;
      phaseVelWindowSamples = computeDynamicWindowSamples(phasePeakVelocity, activeConfig.minVelocityFlatWindowSamples,
                                                            activeConfig.maxVelocityFlatWindowSamples,
                                                            activeConfig.windowSaturationPeakVelocityMps);
      phaseAccWindowSamples = computeDynamicWindowSamples(phasePeakVelocity, activeConfig.minAccelerationFlatWindowSamples,
                                                            activeConfig.maxAccelerationFlatWindowSamples,
                                                            activeConfig.windowSaturationPeakVelocityMps);
    }
    if (fabs(s.accZ) > phasePeakAcceleration) phasePeakAcceleration = fabs(s.accZ);
  }

  void openPhase(uint16_t idxInBracket, float directionHint) {
    phaseOpen = true;
    phaseType = (directionHint > 0.0f) ? 1 : -1;
    phasePeakVelocity = 0;
    phasePeakAcceleration = 0;
    phaseVelWindowSamples = activeConfig.maxVelocityFlatWindowSamples;
    phaseAccWindowSamples = activeConfig.maxAccelerationFlatWindowSamples;
    phaseSampleCount = 0;
    phaseLastSameDirIdx = -1;

    uint16_t avail = idxInBracket;
    uint16_t maxBackfill = activeConfig.phaseLookbackSamples;
    uint16_t nBackfill = 0;
    while (nBackfill < avail && nBackfill < maxBackfill) {
      const BracketSample& h = bracketBuf[idxInBracket - 1 - nBackfill];
      int8_t hDir = (h.velZ > 0.0f) ? 1 : (h.velZ < 0.0f ? -1 : 0);
      if (hDir != phaseType) break;
      nBackfill++;
    }

    phaseId = nextPhaseId++;
    phaseRepNumber = currentRepNumberEngine;
    phaseStartIdxInBracket = idxInBracket - nBackfill;

    // v3.11.8: directionHint IS velZLive at idxInBracket (see the call in
    // stepPhaseEngine()) - the raw-vs-corrected gap at this sample, held
    // ~constant over the short backfill window (phaseLookbackSamples
    // default 5, ~50ms - the correction estimate moves slowly), lets each
    // backfilled sample get an approximated corrected value too instead of
    // falling back to its own raw velZ - see accumulateIntoOpenPhase().
    float driftOffset = bracketBuf[idxInBracket].velZ - directionHint;

    for (uint16_t k = nBackfill; k > 0; k--) {
      uint16_t bi = idxInBracket - k;
      accumulateIntoOpenPhase(bi, bracketBuf[bi].velZ - driftOffset);
    }
    accumulateIntoOpenPhase(idxInBracket, directionHint);
  }

  void refitRepCalibration() {
    uint16_t n = repCalibCount;
    if (n == 1) {
      repCalibRate = 0.0f;
      repCalibIntercept = repCalibSumY;
      return;
    }
    if (n < 2) return;
    float denom = (float)n * repCalibSumTT - repCalibSumT * repCalibSumT;
    if (fabs(denom) < 1e-9f) return;
    repCalibRate = ((float)n * repCalibSumTY - repCalibSumT * repCalibSumY) / denom;
    repCalibIntercept = (repCalibSumY - repCalibRate * repCalibSumT) / (float)n;
  }

  void checkZeroCrossing() {
    if (velZ < sinceCrossingMin) sinceCrossingMin = velZ;
    if (prevVelZ < 0.0f && velZ >= 0.0f && sinceCrossingMin < activeConfig.minCrossingExcursionMps) {
      if (lastCrossingT >= 0.0f) {
        float duration = sessionT - lastCrossingT;
        if (duration >= activeConfig.minCrossingDurationS) {
          float disp = rawPosCumulative - lastCrossingRawPos;
          float localOffset = disp / duration;
          float midT = (lastCrossingT + sessionT) / 2.0f;

          // v3.11.20: capture what THIS refit is about to change, before it
          // changes it - see the version note at the top of this file.
          // repCalibRate*sessionT+repCalibIntercept is only the live
          // estimate livePredictedOffset() actually uses once
          // repCalibCount>=2 (checked on the OLD count, before the
          // increment below) - if it wasn't active yet, there is no live
          // reading to preserve continuity against.
          bool hadRepCalibBefore = (repCalibCount >= 2);
          float offsetBeforeRefit = hadRepCalibBefore ? (repCalibRate * sessionT + repCalibIntercept) : 0.0f;

          repCalibCount++;
          repCalibSumT += midT; repCalibSumY += localOffset;
          repCalibSumTT += midT * midT; repCalibSumTY += midT * localOffset;
          refitRepCalibration();

          // v3.11.20: patch repCalibIntercept so the formula evaluates to
          // EXACTLY offsetBeforeRefit at THIS sessionT - the same
          // continuity principle already applied in stepPhaseEngine() for
          // the confirmed-still re-anchor (v3.11.15), just triggered here
          // by a refit instead of by stillness. Does NOT touch
          // repCalibRate or the accumulated sums, so the regression's own
          // long-term fit (and the next refit's starting point) is
          // unaffected - this only removes the artificial step at the
          // instant of THIS refit from the live reading.
          if (hadRepCalibBefore) {
            float offsetAfterRefit = repCalibRate * sessionT + repCalibIntercept;
            repCalibIntercept += offsetBeforeRefit - offsetAfterRefit;
          }
        }
      }
      lastCrossingT = sessionT;
      lastCrossingRawPos = rawPosCumulative;
      sinceCrossingMin = 0.0f;
    }
  }

  void closePhaseFn(uint8_t reason, uint16_t idxInBracket) {
    uint16_t n = phaseSampleCount;
    int16_t discardedI, cutoff;
    if (reason == 0) { // flat
      uint16_t win = phaseVelWindowSamples;
      discardedI = (int16_t)((win < n) ? win : n);
      cutoff = (int16_t)n - 1 - discardedI;
    } else if (reason == 1) { // reversal
      discardedI = (int16_t)n - 1 - phaseLastSameDirIdx;
      cutoff = phaseLastSameDirIdx;
    } else { // timeout
      discardedI = 0;
      cutoff = (int16_t)n - 1;
    }

    uint16_t startIdx = phaseStartIdxInBracket;
    uint16_t endIdx = idxInBracket;
    float tStart = bracketBuf[startIdx].t;
    float tEnd = (cutoff >= 0) ? bracketBuf[startIdx + cutoff].t : tStart;
    float durationS = fmax(0.0f, tEnd - tStart);
    bool reported = durationS > activeConfig.minPhaseDurationS;

    // v3.11.18: one line per phase close attempt (reported OR silently
    // discarded for being too short) - added specifically to diagnose
    // spurious same-direction splits (see the v3.11.18 version note above)
    // and long flat-guard stalls: closeReason/durationS were never visible
    // in the log before this, only inferable indirectly from the "S," rows'
    // state column, which can't distinguish "closed and reopened the same
    // direction" from "genuinely still open". Two consecutive "P," lines
    // with the same sign phaseType and no intervening rep-completing close
    // is exactly the failure signature fixed in v3.11.18 - now directly
    // visible instead of requiring the kind of reconstruction that took to
    // diagnose the original report.
    if (serialLogActive()) {
      Serial.print("P,"); Serial.print(phaseId); Serial.print(',');
      Serial.print(phaseRepNumber); Serial.print(',');
      Serial.print(phaseType); Serial.print(',');
      Serial.print(reason); Serial.print(',');
      Serial.print(durationS, 4); Serial.print(',');
      Serial.print(phaseSampleCount); Serial.print(',');
      Serial.print(discardedI); Serial.print(',');
      Serial.println(reported ? 1 : 0);
    }

    if (reported) {
      if (closedPhaseCount < MAX_CLOSED_PHASES_PER_BRACKET) {
        ClosedPhaseInfo& cp = closedPhases[closedPhaseCount++];
        cp.id = phaseId; cp.repNumber = phaseRepNumber; cp.phaseType = phaseType;
        cp.startIdx = startIdx; cp.endIdx = endIdx; cp.cutoff = cutoff;
        cp.discarded = (uint16_t)discardedI; cp.durationS = durationS; cp.closeReason = reason;
      }
      bool useRepCalib = (repCalibCount >= 2);
      PhaseScore ps = scorePhase(startIdx, endIdx, cutoff, durationS, reason, phaseType,
                                  false, useRepCalib, baselineRawVel, 0, 0, 0);
      CorrectionStatus status = useRepCalib ? CorrectionStatus::RepCalibrated : CorrectionStatus::Provisional;
      reportPhaseScore(phaseRepNumber, phaseType, status, ps.peak, ps.mean, ps.peakAcc, ps.meanAcc, ps.disp, ps.quality1);

      // v3.11.18: unconditional on phaseType>0, not on alternation with the
      // last closed phase's type - see the version note at the top of this
      // file. A rep is, by definition, complete once its concentric phase
      // closes; incrementing only depends on THAT, never on what closed
      // before it.
      if (phaseType > 0) {
        currentRepNumberEngine++;
      }
    }

    phaseOpen = false;
    phaseType = 0;
    phasePeakVelocity = 0;
    phasePeakAcceleration = 0;
    // v3.11.17: MIN, not max - see the version note at the top of this
    // file. While idle (no phase open) there is no "peak velocity" context
    // to size the window against; the previous max-window reset kept
    // excursionFlat() looking at up to maxVelocityFlatWindowSamples/
    // maxAccelerationFlatWindowSamples (300ms default) of history that
    // still included the deceleration into this very stillness, so
    // flatCombined stayed false - and the live reading stuck away from
    // zero - for up to that long after a real stop, forcing recovery onto
    // flatGuardOverrideStillTimeS (1.0s) instead of the fast primary path.
    phaseVelWindowSamples = activeConfig.minVelocityFlatWindowSamples;
    phaseAccWindowSamples = activeConfig.minAccelerationFlatWindowSamples;
  }

  void enqueueCorrectedCurveChunks(float t0, float baseline, float drift, float basisTotal) {
    uint16_t total = bracketCount;
    if (total == 0) return;
    uint16_t totalChunks = (total + CorrectedCurveChunk::CAPACITY - 1) / CorrectedCurveChunk::CAPACITY;
    uint16_t bracketId = nextBracketId++;
    for (uint16_t c = 0; c < totalChunks; c++) {
      if (chunkCount >= CHUNK_QUEUE_CAPACITY) {
        // Safety net (see CHUNK_QUEUE_CAPACITY above): queue full, the
        // oldest not-yet-sent block is discarded - data loss accepted
        // only in this extreme scenario (never observed during
        // validation).
        chunkHead = (chunkHead + 1) % CHUNK_QUEUE_CAPACITY;
        chunkCount--;
      }
      uint16_t tail = (chunkHead + chunkCount) % CHUNK_QUEUE_CAPACITY;
      CorrectedCurveChunk& chunk = chunkQueue[tail];
      chunk.bracketId = bracketId;
      chunk.chunkIndex = c;
      chunk.totalChunks = totalChunks;
      uint16_t base = c * CorrectedCurveChunk::CAPACITY;
      uint16_t count = (uint16_t)CorrectedCurveChunk::CAPACITY;
      if (total - base < count) count = total - base;
      chunk.sampleCount = (uint8_t)count;
      chunk.startTimestampMs = trackingStartMs + (uint32_t)(bracketBuf[base].t * 1000.0f + 0.5f);
      chunk.sampleIntervalMs = (uint16_t)SAMPLE_INTERVAL_MS;
      for (uint8_t k = 0; k < CorrectedCurveChunk::CAPACITY; k++) {
        if (k < count) {
          const BracketSample& s = bracketBuf[base + k];
          float pos = s.t - t0;
          float cv = (basisTotal > 1e-9f) ? (s.velZ - baseline - drift * pos / basisTotal) : (s.velZ - baseline);
          chunk.correctedVelZ[k] = cv;
        } else {
          chunk.correctedVelZ[k] = 0.0f;
        }
      }
      chunkCount++;
    }
  }

  void closeBracketFn() {
    float t0 = bracketStartT;
    float t1 = sessionT;
    float baseline = baselineRawVel;
    // Drift measured RELATIVE to the anchor already reported to 0 by the
    // previous bracket, NEVER to absolute zero - guarantees continuity
    // (no steps at the close of each bracket).
    float drift = velZ - baseline;
    float basisTotal = t1 - t0;
    float rate = (basisTotal > 1e-6f) ? (drift / basisTotal) : 0.0f;
    if (!emaInitialized) {
      emaRate = rate;
      emaInitialized = true;
    } else {
      // v3.11.26 (fix B, see the version note below): a bracket's rate is
      // only as trustworthy as the time it was measured over - a 150ms
      // bracket (noise between two close reps) and a 1.5s bracket (a real
      // rep) used to move emaRate by the SAME activeConfig.emaAlpha
      // regardless, letting a single noisy short bracket seed a rate later
      // extrapolated, unchanged, over a much longer stretch (see fix A in
      // livePredictedOffset()). effectiveAlpha scales emaAlpha down for
      // short brackets (basisTotal below EMA_RATE_REFERENCE_DURATION_S),
      // capped at emaAlpha itself for anything at or above it - a full-
      // length bracket updates exactly as before.
      const float EMA_RATE_REFERENCE_DURATION_S = 1.0f;
      float durationWeight = basisTotal / EMA_RATE_REFERENCE_DURATION_S;
      if (durationWeight > 1.0f) durationWeight = 1.0f;
      float effectiveAlpha = activeConfig.emaAlpha * durationWeight;
      emaRate = effectiveAlpha * rate + (1.0f - effectiveAlpha) * emaRate;
    }

    for (uint8_t i = 0; i < closedPhaseCount; i++) {
      ClosedPhaseInfo& cp = closedPhases[i];
      PhaseScore ps = scorePhase(cp.startIdx, cp.endIdx, cp.cutoff, cp.durationS, cp.closeReason, cp.phaseType,
                                  true, false, baseline, drift, t0, basisTotal);
      reportPhaseScore(cp.repNumber, cp.phaseType, CorrectionStatus::Corrected,
                        ps.peak, ps.mean, ps.peakAcc, ps.meanAcc, ps.disp, ps.quality1);
    }

    // v3.11.7: log line for THIS bracket - without this, the corrected
    // curve (the one CorrectedCurve sends over BLE) could NEVER be
    // reconstructed from the serial log alone: a sample's correction
    // depends on the drift measured when ITS bracket closes, not yet
    // known at the moment that sample is written to the "S," line (100Hz,
    // always raw) - architecturally it can't be otherwise, the same
    // reason the app receives the raw value first and then CorrectedCurve
    // when the bracket closes. With bracketId/t0/t1/baseline/drift/
    // basisTotal here, an offline script can reconstruct exactly
    // corrected(t) = velZ(t) - baseline - drift*(t-t0)/basisTotal for
    // every "S," sample with t0<=t<=t1 - the same formula as
    // enqueueCorrectedCurveChunks() above.
    if (serialLogActive()) {
      uint16_t loggedBracketId = nextBracketId;
      Serial.print("B,"); Serial.print(loggedBracketId); Serial.print(',');
      Serial.print(t0, 4); Serial.print(',');
      Serial.print(t1, 4); Serial.print(',');
      Serial.print(baseline, 4); Serial.print(',');
      Serial.print(drift, 4); Serial.print(',');
      Serial.print(rate, 5); Serial.print(',');
      Serial.print(emaRate, 5); Serial.print(',');
      Serial.println(basisTotal, 4);
    }

    enqueueCorrectedCurveChunks(t0, baseline, drift, basisTotal);

    // The new anchor: this bracket has just reported velZ's raw value,
    // HERE, to 0 - future brackets measure their drift starting from this
    // point, not from absolute zero.
    baselineRawVel = velZ;

    // Carries the still-open phase's samples forward into the new
    // bracket (possible only if flatGuardMaxVelocityMps >
    // phaseStartVelocityMps, see the version note) instead of losing
    // them/going out of bounds.
    uint16_t carryCount = 0;
    if (phaseOpen) {
      uint16_t carryOffset = phaseStartIdxInBracket;
      carryCount = bracketCount - carryOffset;
      for (uint16_t i = 0; i < carryCount; i++) {
        bracketBuf[i] = bracketBuf[carryOffset + i];
      }
    }

    bracketStartT = t1;
    bracketCount = carryCount;
    closedPhaseCount = 0;

    if (phaseOpen) {
      phaseStartIdxInBracket = 0;
    }
  }

  float livePredictedOffset() {
    if (repCalibCount >= 2) {
      return repCalibRate * sessionT + repCalibIntercept;
    }
    if (emaInitialized) {
      // v3.11.26 (fix A - see the version note below): emaRate is
      // extrapolated LINEARLY and, until this fix, WITHOUT LIMIT over the
      // time elapsed since the last anchor - fine for a normal rep (under
      // a second), but a long open bracket with no still pause to
      // re-anchor it (a slow/heavy rep, or several touch-and-go reps back
      // to back) lets even a small rate error compound into seconds of
      // drift. Past EMA_EXTRAPOLATION_CAP_S the offset simply stops
      // growing instead of running away - it's frozen at its best estimate
      // rather than left to diverge further, and normal re-anchoring
      // (checkZeroCrossing()/the confirmedStillNow block in
      // stepPhaseEngine()) still corrects it the moment a real reference
      // point becomes available.
      const float EMA_EXTRAPOLATION_CAP_S = 2.5f;
      float elapsed = sessionT - bracketStartT;
      if (elapsed > EMA_EXTRAPOLATION_CAP_S) elapsed = EMA_EXTRAPOLATION_CAP_S;
      return baselineRawVel + emaRate * elapsed;
    }
    return 0.0f;
  }

  // One update() cycle of the phase/bracket engine on the Z axis - see
  // DriftCorrectedSim.step() in sim_v2_drift.py for the line-by-line
  // reference. worldAccZDirected is already bias-corrected (see
  // updateOrientationAndAcceleration()) and oriented (repDirectionSign).
  void stepPhaseEngine(float dt, float worldAccZDirected) {
    sessionT += dt;

    float incrZ = trapezoidPrimedZ ? (worldAccZDirected + prevAccZDirected) * 0.5f * dt : worldAccZDirected * dt;
    velZ += incrZ;
    prevAccZDirected = worldAccZDirected;
    trapezoidPrimedZ = true;

    rawPosCumulative += (velZ + prevVelZ) * 0.5f * dt;
    checkZeroCrossing();
    prevVelZ = velZ;

    // Best current correction estimate, used ONLY for the phase/bracket
    // DECISIONS below (see the version note, point 3) - never for the
    // retroactive measurement/correction, which always uses
    // raw+measured drift there.
    float velZLive = velZ - livePredictedOffset();
    // v3.11.19: the plausibility clamp applies HERE, to velZLive - not to
    // velZ above - see the version note at the top of this file. velZ is
    // the raw integral and is, by design (v3.11.0), NEVER reset or
    // touched - on a long session it is EXPECTED to run past any
    // "plausible" bound as drift accumulates, exactly like it did in the
    // diagnosed capture (pinned at exactly 4.0000 for 15 consecutive
    // samples, confirming the clamp was firing on drift, not on a real
    // 4 m/s bar speed). Clamping it in place corrupted every raw-velZ
    // consumer downstream of this line for as long as the clamp was
    // active: bracketBuf's stored sample (scorePhase's corrected-curve
    // math reads it directly), closeBracketFn()'s drift measurement
    // (drift = velZ - baseline), rawPosCumulative/checkZeroCrossing's
    // rep-cycle calibration input - none of which should ever see a
    // clipped value. velZLive is the right target: it is, by its own
    // doc comment in MotionDebugState, "the TRUE value the algorithm
    // uses to decide phases/brackets" - exactly the quantity a
    // "plausible velocity" ceiling is meant to guard.
    bool clamped = false;
    if (velZLive > activeConfig.maxPlausibleVelocityMps) { velZLive = activeConfig.maxPlausibleVelocityMps; clamped = true; }
    else if (velZLive < -activeConfig.maxPlausibleVelocityMps) { velZLive = -activeConfig.maxPlausibleVelocityMps; clamped = true; }
    debugStateVar.velocityClampedToMax = clamped;
    debugStateVar.velZLive = velZLive;

    velLiveRing[ringHead] = velZLive;
    accZRing[ringHead] = worldAccZDirected;
    ringHead = (ringHead + 1) % FLAT_RING_CAPACITY;
    if (ringFill < FLAT_RING_CAPACITY) ringFill++;

    // v3.11.27: two-tier flat-guard, replacing the old dynamic-window +
    // raw-quiet-override pair (see the version note below for the full
    // reasoning and the real capture that drove this). Below
    // flatGuardMaxVelocityMps (or while idle, phaseOpen==false), nothing
    // changes here: the normal dynamic window (phaseVelWindowSamples/
    // phaseAccWindowSamples, shrunk by accumulateIntoOpenPhase() as the
    // phase's peak velocity grows) is fast and, under the ceiling, safe.
    // Above it - the exact situation the ceiling exists to catch, a
    // correction estimate that's drifted past the point where a small
    // window could be trusted - both tests switch to a wider, FIXED
    // window instead: still velZLive/worldAccZ (not raw IMU noise), just
    // over enough samples (velocityOverrideFlatWindowSamples, 30-60) that
    // a genuine pause (holds flat for hundreds of ms) can't be mistaken
    // for the few-millisecond acceleration-crosses-zero instant at the
    // PEAK of a fast rep - see stepPhaseEngine()'s call sites below and
    // the version note for why that ambiguity rules out simply dropping
    // the ceiling on the dynamic-window path itself.
    bool aboveCeiling = phaseOpen && (fabs(velZLive) > activeConfig.flatGuardMaxVelocityMps);
    uint16_t velWindow = aboveCeiling ? activeConfig.velocityOverrideFlatWindowSamples : phaseVelWindowSamples;
    uint16_t accWindow = aboveCeiling ? activeConfig.velocityOverrideFlatWindowSamples : phaseAccWindowSamples;
    bool velIsFlatNow = excursionFlat(velLiveRing, ringFill, ringHead, FLAT_RING_CAPACITY, velWindow, activeConfig.velocityFlatBandMps);
    bool accelFlatNow = absoluteFlat(accZRing, ringFill, ringHead, FLAT_RING_CAPACITY, accWindow, activeConfig.accelerationFlatBandMps2);
    bool flatCombined = velIsFlatNow && accelFlatNow;
    debugStateVar.flatGuardOverrideFired = aboveCeiling && flatCombined;
    bool risingEdge = flatCombined && !wasFlatCombined;
    wasFlatCombined = flatCombined;

    if (bracketCount >= BRACKET_BUFFER_CAPACITY) {
      // Safety net (see BRACKET_BUFFER_CAPACITY above): should never
      // happen post-fix - forces a close to free up space instead of
      // losing samples.
      closeBracketFn();
    }
    uint16_t idx = bracketCount;
    BracketSample& bs = bracketBuf[idx];
    bs.t = sessionT; bs.dt = dt; bs.velZ = velZ; bs.accZ = worldAccZDirected;
    bracketCount++;

    bool oppositeAndSignificant = isOppositeDirectionForType(phaseType, velZLive) &&
                                   (fabs(velZLive) >= activeConfig.phaseStartVelocityMps);
    oppositeDirStreakSamples = oppositeAndSignificant ? (oppositeDirStreakSamples + 1) : 0;
    bool reversalConfirmed = phaseOpen && (oppositeDirStreakSamples > 0) &&
                              (oppositeDirStreakSamples >= activeConfig.reversalConfirmSamples);

    if (phaseOpen) {
      accumulateIntoOpenPhase(idx, velZLive);
      float phaseStartT = bracketBuf[phaseStartIdxInBracket].t;
      bool phaseTimedOut = (sessionT - phaseStartT) >= activeConfig.maxPhaseDurationS;
      if (phaseTimedOut) closePhaseFn(2, idx);
      else if (flatCombined) closePhaseFn(0, idx);
      else if (reversalConfirmed) closePhaseFn(1, idx);
    }

    if (!phaseOpen && fabs(velZLive) >= activeConfig.phaseStartVelocityMps) {
      openPhase(idx, velZLive);
    }

    debugStateVar.bracketClosed = false;
    if (risingEdge) {
      closeBracketFn();
      debugStateVar.bracketClosed = true;
    }

    // v3.11.2: while the engine CONFIRMS stillness (flatCombined, the
    // same robust condition as above - two dynamic windows + an absolute
    // ceiling evaluated on the already-corrected estimate, not the single
    // instantaneous-threshold comparison of the v3.10.x ZUPT), the anchor
    // stays aligned to velZ for the ENTIRE duration of the stillness, not
    // just at the rising edge handled by closeBracketFn() above -
    // otherwise a not-yet-converged bias residual during a long pause
    // would leave the anchor behind where the sensor TRULY is now. Dual
    // effect, from the SAME mechanism (no longer just a display
    // freeze-frame - see the comment on refVelZ in MotionTracker.h): 1)
    // the value transmitted live (velZ - baselineRawVel, see
    // integrateMotion()) goes back to reading ~0 continuously while
    // still, instead of staying stuck on the residual from when the
    // motion stopped; 2) the NEXT rep computes its own correction
    // starting from this same fresh reference (bracketStartT follows
    // along). MUST run AFTER closeBracketFn() above, NEVER before: that
    // function measures drift by comparing velZ against the OLD anchor -
    // if this block ran first in the same call, the anchor would already
    // have been overwritten and the measured drift would always be 0.
    debugStateVar.confirmedStillNow = flatCombined;
    if (flatCombined) {
      baselineRawVel = velZ;
      bracketStartT = sessionT;
      // v3.11.15: the same re-anchoring, applied to the rep-cycle
      // calibration too, when it's the active estimate (repCalibCount>=2,
      // see livePredictedOffset()) - unlike the EMA above, this branch had
      // NO self-correction mechanism during a pause: it's a fixed linear
      // regression on ABSOLUTE sessionT (repCalibRate*sessionT+
      // repCalibIntercept), never re-anchored until a new valid rep cycle
      // comes in - so during a real pause it keeps "drifting" on its own
      // (same intercept, but sessionT keeps advancing) even while raw
      // velZ isn't moving at all. Diagnosed from a real capture: a
      // bracket stayed open 10.77s (reps 14-16 only corrected once it
      // finally closed) because velZLive stayed stuck just outside
      // flatGuardMaxVelocityMps despite worldAccZ being visibly flat near
      // zero - only a deliberate manual jolt (moving raw velZ enough)
      // unstuck it. Fix: recompute repCalibIntercept so the formula
      // returns EXACTLY velZ at this instant - the same idea as the EMA
      // re-anchoring above, just on the intercept instead of
      // baselineRawVel/bracketStartT. Does NOT touch repCalibSumT/SumY/
      // SumTT/SumTY (the regression's accumulated sums) - the next valid
      // rep cycle still recomputes repCalibIntercept from scratch via
      // refitRepCalibration(), overwriting this "live" adjustment: it's
      // harmless to the regression's long-term accuracy, it only corrects
      // the reading during the current pause.
      if (repCalibCount >= 2) {
        repCalibIntercept = velZ - repCalibRate * sessionT;
      }
    }
  }

  // Resets integration + phase/bracket/rep engine state: called on
  // tracking start/stop (clean new session).
  void resetTracking() {
    worldVelX = worldVelY = 0;
    worldPosX = worldPosY = 0;
    prevWorldVelX = prevWorldVelY = 0;
    prevWorldAccX = prevWorldAccY = 0;
    trapezoidPrimed = false;

    velZ = 0; prevAccZDirected = 0; trapezoidPrimedZ = false;
    prevVelZ = 0; rawPosCumulative = 0;
    sessionT = 0;
    trackingStartMs = millis();

    ringFill = 0; ringHead = 0;
    for (uint8_t i = 0; i < FLAT_RING_CAPACITY; i++) { velLiveRing[i] = 0; accZRing[i] = 0; }
    wasFlatCombined = false;

    phaseOpen = false; phaseType = 0; phaseId = 0; phaseRepNumber = 1;
    phaseStartIdxInBracket = 0; phaseLastSameDirIdx = -1;
    phaseSampleCount = 0; phasePeakVelocity = 0; phasePeakAcceleration = 0;
    // v3.11.17: MIN, not max - same reasoning as closePhaseFn() (see the
    // version note at the top of this file): idle at session start has no
    // "peak velocity" context either, so the fast window applies here too.
    phaseVelWindowSamples = activeConfig.minVelocityFlatWindowSamples;
    phaseAccWindowSamples = activeConfig.minAccelerationFlatWindowSamples;
    oppositeDirStreakSamples = 0;
    currentRepNumberEngine = 1;
    nextPhaseId = 0;

    bracketCount = 0; bracketStartT = 0; nextBracketId = 1;
    closedPhaseCount = 0;

    emaRate = 0; emaInitialized = false; baselineRawVel = 0;

    lastCrossingT = -1; lastCrossingRawPos = 0; sinceCrossingMin = 0;
    repCalibCount = 0; repCalibSumT = repCalibSumY = repCalibSumTT = repCalibSumTY = 0;
    repCalibRate = 0; repCalibIntercept = 0;

    chunkHead = 0; chunkCount = 0;
    for (uint8_t i = 0; i < REP_STORE_CAPACITY; i++) repStore[i] = RepSlot();

    refVelZ = 0;
    livePhaseState = PhaseState::Idle;
    livePosZ = 0;
    prevDirVelForLivePos = 0;

    resetQuietRings();
    pendingCount = 0;
    pendingHead = 0;
  }

  // Reads the IMU, updates orientation and world-frame acceleration.
  // Always runs (even before START) so orientation stays fresh.
  void updateOrientationAndAcceleration(float dt) {
    float axAvg, ayAvg, azAvg, gxAvg, gyAvg, gzAvg;
    fastSamples.harvest(axAvg, ayAvg, azAvg, gxAvg, gyAvg, gzAvg);
    float axRaw = axAvg * accelScaleG;
    float ayRaw = ayAvg * accelScaleG;
    float azRaw = azAvg * accelScaleG;
    float ax, ay, az;
    accMedianFilter.filter(axRaw, ayRaw, azRaw, ax, ay, az);
    float gxFiltered, gyFiltered, gzFiltered;
    gyroMedianFilter.filter(gxAvg, gyAvg, gzAvg, gxFiltered, gyFiltered, gzFiltered);
    lastGyroFilteredX = gxFiltered; lastGyroFilteredY = gyFiltered; lastGyroFilteredZ = gzFiltered;
    float gx = (gxFiltered - gyroBiasX) * DEG_TO_RAD;
    float gy = (gyFiltered - gyroBiasY) * DEG_TO_RAD;
    float gz = (gzFiltered - gyroBiasZ) * DEG_TO_RAD;

    float accMag = sqrt(ax * ax + ay * ay + az * az);
    float trust = constrain(1.0f - fabs(accMag - G) / G * TRUST_FALLOFF, TRUST_FLOOR, 1.0f);
    ahrs.update(gx, gy, gz, ax, ay, az, dt, BETA_BASE * trust);

    float q0 = ahrs.q0(), q1 = ahrs.q1(), q2 = ahrs.q2(), q3 = ahrs.q3();

    float gravBodyX = 2.0f * (q1 * q3 - q0 * q2);
    float gravBodyY = 2.0f * (q0 * q1 + q2 * q3);
    float gravBodyZ = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    float linAccX = ax - gravBodyX * accelScaleG;
    float linAccY = ay - gravBodyY * accelScaleG;
    float linAccZ = az - gravBodyZ * accelScaleG;

    float R00 = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    float R01 = 2.0f * (q1 * q2 - q0 * q3);
    float R02 = 2.0f * (q1 * q3 + q0 * q2);
    float R10 = 2.0f * (q1 * q2 + q0 * q3);
    float R11 = 1.0f - 2.0f * (q1 * q1 + q3 * q3);
    float R12 = 2.0f * (q2 * q3 - q0 * q1);

    float worldAccZraw = linAccX * gravBodyX + linAccY * gravBodyY + linAccZ * gravBodyZ;

    // v3.11.11: the quiet gate for accZBias/gyroBias no longer compares
    // accMag against the ABSOLUTE constant G (see the version note at the
    // top of the file for the full diagnosis) - a windowed FLATNESS test
    // on accMag instead, same principle as the flat-guard's
    // absoluteFlat()/excursionFlat(): "accMag isn't changing" instead of
    // "accMag matches G", robust to whatever resting orientation was used
    // at calibration time. v3.11.14: raw gyroscope + accMag now
    // history-tracked on shared sliding-window rings (gyroMagQuietRing/
    // accMagQuietRing, see declaration above) instead of a cumulative
    // counter - see the version note at the top of the file for the
    // diagnosis of the problem this fixes.
    gyroMagQuietRing[quietRingHead] = lastGyroMag;
    accMagQuietRing[quietRingHead] = accMag;
    quietRingHead = (quietRingHead + 1) % BIAS_QUIET_RING_CAPACITY;
    if (quietRingFill < BIAS_QUIET_RING_CAPACITY) quietRingFill++;
    bool quietForAccZBias = quietForDuration(activeConfig.accZBiasIdleStillTimeS);
    bool quietForGyroBias = quietForDuration(activeConfig.gyroBiasIdleStillTimeS);
    // v3.11.13: current correction = the fixed body-frame vector projected
    // onto the CURRENT gravity direction (not a constant scalar anymore -
    // see the comment on accBiasVec above). The single-observation LMS
    // update pushes accBiasVec along (gravBodyX,Y,Z) in proportion to the
    // current residual error - the same alpha/time-constant as before, and
    // it reduces to exactly the old scalar EMA when the gravity direction
    // never changes between updates.
    float accZBiasNow = accBiasVec[0] * gravBodyX + accBiasVec[1] * gravBodyY + accBiasVec[2] * gravBodyZ;
    if (quietForAccZBias &&
        fabs(worldAccZraw - accZBiasNow) < ACCZ_BIAS_MAX_STEP) {
      float alpha = dt / (ACCZ_BIAS_TAU_S + dt);
      float err = worldAccZraw - accZBiasNow;
      accBiasVec[0] += alpha * err * gravBodyX;
      accBiasVec[1] += alpha * err * gravBodyY;
      accBiasVec[2] += alpha * err * gravBodyZ;
      accZBiasNow = accBiasVec[0] * gravBodyX + accBiasVec[1] * gravBodyY + accBiasVec[2] * gravBodyZ;
    }

    if (quietForGyroBias) {
      float gyroAlpha = dt / (GYRO_BIAS_TAU_S + dt);
      gyroBiasX += gyroAlpha * (gxFiltered - gyroBiasX);
      gyroBiasY += gyroAlpha * (gyFiltered - gyroBiasY);
      gyroBiasZ += gyroAlpha * (gzFiltered - gyroBiasZ);
    }

    float worldAccZ = constrain(worldAccZraw - accZBiasNow, -100.0f, 100.0f);
    lastAccZBiasApplied = accZBiasNow;

    sample.linAccX = linAccX; sample.linAccY = linAccY; sample.linAccZ = linAccZ;
    sample.worldAccX = R00 * linAccX + R01 * linAccY + R02 * linAccZ;
    sample.worldAccY = R10 * linAccX + R11 * linAccY + R12 * linAccZ;
    sample.worldAccZ = worldAccZ;
    sample.quatW = q0; sample.quatX = q1; sample.quatY = q2; sample.quatZ = q3;

    lastGyroMag = sqrt(gx * gx + gy * gy + gz * gz) * RAD_TO_DEG;
  }

  // Raw data log for lab analysis (v3.11.24: active whenever a USB-serial
  // connection is open, see serialLogActive() above - was a manually-set
  // RuntimeConfig::debugLogEnabled flag before this version) - ONE CSV
  // line per sample at 100Hz, type "S". v3.11.0: the
  // velZ column (was trueVelZ) is now the new engine's RAW velocity
  // (never zeroed); the shockRejected column removed (anti-shock filter no
  // longer exists, see v3.11.10 near the top of the file); zuptHold ->
  // bracketClosed (a bracket closed on this sample).
  void logSampleCsv(float dt) {
    const char* state = (livePhaseState == PhaseState::Concentric) ? "concentric" :
                         (livePhaseState == PhaseState::Eccentric) ? "eccentric" : "idle";
    Serial.print("S,");
    Serial.print(lastSampleTime); Serial.print(',');
    Serial.print(dt * 1000.0f, 3); Serial.print(',');
    Serial.print(state); Serial.print(',');
    Serial.print(currentRepNumberEngine); Serial.print(',');
    Serial.print(sample.linAccX, 4); Serial.print(',');
    Serial.print(sample.linAccY, 4); Serial.print(',');
    Serial.print(sample.linAccZ, 4); Serial.print(',');
    Serial.print(sample.worldAccX, 4); Serial.print(',');
    Serial.print(sample.worldAccY, 4); Serial.print(',');
    Serial.print(sample.worldAccZ, 4); Serial.print(',');
    Serial.print(velZ, 4); Serial.print(',');
    Serial.print(refVelZ, 4); Serial.print(',');
    Serial.print(debugStateVar.velZLive, 4); Serial.print(',');
    Serial.print(repCalibRate, 5); Serial.print(',');
    Serial.print(repCalibIntercept, 4); Serial.print(',');
    Serial.print(repCalibCount); Serial.print(',');
    Serial.print(worldVelX, 4); Serial.print(',');
    Serial.print(worldVelY, 4); Serial.print(',');
    Serial.print(livePosZ, 4); Serial.print(',');
    Serial.print(worldPosX, 4); Serial.print(',');
    Serial.print(worldPosY, 4); Serial.print(',');
    Serial.print(sample.quatW, 4); Serial.print(',');
    Serial.print(sample.quatX, 4); Serial.print(',');
    Serial.print(sample.quatY, 4); Serial.print(',');
    Serial.print(sample.quatZ, 4); Serial.print(',');
    Serial.print(lastGyroMag, 2); Serial.print(',');
    Serial.print(lastAccZBiasApplied, 4); Serial.print(',');
    Serial.print(gyroBiasX, 3); Serial.print(',');
    Serial.print(gyroBiasY, 3); Serial.print(',');
    Serial.print(gyroBiasZ, 3); Serial.print(',');
    Serial.print(lastGyroFilteredX, 3); Serial.print(',');
    Serial.print(lastGyroFilteredY, 3); Serial.print(',');
    Serial.print(lastGyroFilteredZ, 3); Serial.print(',');
    Serial.print(debugStateVar.velocityClampedToMax ? 1 : 0); Serial.print(',');
    Serial.print(debugStateVar.bracketClosed ? 1 : 0); Serial.print(',');
    Serial.print(debugStateVar.confirmedStillNow ? 1 : 0); Serial.print(',');
    Serial.println(debugStateVar.flatGuardOverrideFired ? 1 : 0);
  }

  // Integrates worldAccX/Y (cosmetic leak, streaming/debug) and advances
  // the phase/bracket engine on the Z axis (see stepPhaseEngine) - then
  // updates the quantities ONLY for live streaming/display (refVelZ/
  // livePhaseState/livePosZ), always raw (see the version note, point 2:
  // the corrected version arrives separately via CorrectedCurve).
  void integrateMotion(float dt) {
    debugStateVar.gyroMag = lastGyroMag;
    debugStateVar.accZBias = lastAccZBiasApplied;

    float incrX = trapezoidPrimed ? (sample.worldAccX + prevWorldAccX) * 0.5f * dt : sample.worldAccX * dt;
    float incrY = trapezoidPrimed ? (sample.worldAccY + prevWorldAccY) * 0.5f * dt : sample.worldAccY * dt;
    worldVelX += incrX;
    worldVelY += incrY;
    float leakAlphaXY = dt / (VELOCITY_LEAK_XY_TAU_S + dt);
    worldVelX -= leakAlphaXY * worldVelX;
    worldVelY -= leakAlphaXY * worldVelY;
    prevWorldAccX = sample.worldAccX; prevWorldAccY = sample.worldAccY;
    worldPosX += (worldVelX + prevWorldVelX) * 0.5f * dt;
    worldPosY += (worldVelY + prevWorldVelY) * 0.5f * dt;
    prevWorldVelX = worldVelX; prevWorldVelY = worldVelY;
    trapezoidPrimed = true;

    float worldAccZDirected = sample.worldAccZ * (float)repDirectionSign;
    stepPhaseEngine(dt, worldAccZDirected);

    // v3.11.2: dirVel for live streaming/display is velZ RELATIVE to the
    // anchor (baselineRawVel, kept continuously aligned while confirmed
    // still - see the end of stepPhaseEngine()): it reads ~0 continuously
    // during a real pause, and during motion shows the raw delta from the
    // last confirmed still reference (not yet corrected for THIS rep's
    // drift - that arrives via CorrectedCurve when its bracket closes).
    // velZ itself (measurement/bracket/rep-calibration, never zeroed)
    // always stays unchanged.
    float dirVel = velZ - baselineRawVel; // already oriented
    refVelZ = dirVel;

    // v3.11.3: from the SIGN of dirVel, NOT from the phase engine's
    // open/closed state (phaseOpen/phaseType) - see PhaseState in
    // MotionTracker.h. That is deliberately debounced
    // (reversalConfirmSamples) and lags behind a zero-crossing already
    // visible on the graph - reported by the user (the eccentric stretch
    // stayed colored blue/concentric for a while after the zero-crossing).
    // No final `else`: a negligible variation around zero keeps the
    // previous state, it doesn't force Idle - only confirmedStillNow does.
    if (debugStateVar.confirmedStillNow) {
      livePhaseState = PhaseState::Idle;
    } else if (dirVel > 0.001f) {
      livePhaseState = PhaseState::Concentric;
    } else if (dirVel < -0.001f) {
      livePhaseState = PhaseState::Eccentric;
    }

    if (dirVel <= 0.0f) {
      livePosZ = 0.0f;
    } else {
      livePosZ += (dirVel + prevDirVelForLivePos) * 0.5f * dt;
      if (livePosZ < 0.0f) livePosZ = 0.0f;
    }
    prevDirVelForLivePos = dirVel;

    if (serialLogActive()) {
      logSampleCsv(dt);
    }
  }
}

bool MotionTracker::begin() {
  pinMode(PIN_LSM6DS3TR_C_POWER, OUTPUT);
  digitalWrite(PIN_LSM6DS3TR_C_POWER, HIGH);
  delay(10);

  imu.settings.accelRange = 8;
  imu.settings.gyroRange = 500;

  bool ok = (imu.begin() == 0);
  Wire.setClock(400000);
  return ok;
}

void MotionTracker::calibrateOrientation() {
  float sumAx = 0, sumAy = 0, sumAz = 0;
  float sumGx = 0, sumGy = 0, sumGz = 0;
  float minGx = 1e9f, maxGx = -1e9f, minGy = 1e9f, maxGy = -1e9f, minGz = 1e9f, maxGz = -1e9f;
  for (uint16_t i = 0; i < CALIBRATION_SAMPLES; i++) {
    float ax, ay, az, gx, gy, gz;
    for (uint8_t retry = 0; ; retry++) {
      unsigned long windowStart = millis();
      do {
        fastSamples.poll();
      } while (millis() - windowStart < CALIBRATION_SAMPLE_DELAY_MS);
      fastSamples.harvest(ax, ay, az, gx, gy, gz);

      if (i == 0 || retry >= CALIBRATION_OUTLIER_MAX_RETRIES) break;
      float meanGx = sumGx / i, meanGy = sumGy / i, meanGz = sumGz / i;
      bool isOutlier = fabs(gx - meanGx) > CALIBRATION_OUTLIER_REJECT_DEG_S ||
                        fabs(gy - meanGy) > CALIBRATION_OUTLIER_REJECT_DEG_S ||
                        fabs(gz - meanGz) > CALIBRATION_OUTLIER_REJECT_DEG_S;
      if (!isOutlier) break;
      Serial.print("CAL i="); Serial.print(i);
      Serial.print(" DISCARDED gx="); Serial.print(gx, 3);
      Serial.print(" gy="); Serial.print(gy, 3);
      Serial.print(" gz="); Serial.print(gz, 3);
      Serial.print(" (mean so far: gx="); Serial.print(meanGx, 3);
      Serial.print(" gy="); Serial.print(meanGy, 3);
      Serial.print(" gz="); Serial.print(meanGz, 3);
      Serial.println(")");
    }
    sumAx += ax; sumAy += ay; sumAz += az;
    sumGx += gx; sumGy += gy; sumGz += gz;
    minGx = min(minGx, gx); maxGx = max(maxGx, gx);
    minGy = min(minGy, gy); maxGy = max(maxGy, gy);
    minGz = min(minGz, gz); maxGz = max(maxGz, gz);
    Serial.print("CAL i="); Serial.print(i);
    Serial.print(" ax="); Serial.print(ax, 4);
    Serial.print(" ay="); Serial.print(ay, 4);
    Serial.print(" az="); Serial.print(az, 4);
    Serial.print(" gx="); Serial.print(gx, 3);
    Serial.print(" gy="); Serial.print(gy, 3);
    Serial.print(" gz="); Serial.println(gz, 3);
  }
  float meanAx = sumAx / CALIBRATION_SAMPLES;
  float meanAy = sumAy / CALIBRATION_SAMPLES;
  float meanAz = sumAz / CALIBRATION_SAMPLES;
  ahrs.alignToGravity(meanAx, meanAy, meanAz);
  gyroBiasX = sumGx / CALIBRATION_SAMPLES;
  gyroBiasY = sumGy / CALIBRATION_SAMPLES;
  gyroBiasZ = sumGz / CALIBRATION_SAMPLES;
  float measuredMagG = sqrt(meanAx * meanAx + meanAy * meanAy + meanAz * meanAz);
  if (measuredMagG > 0.85f && measuredMagG < 1.15f) {
    accelScaleG = G / measuredMagG;
    Serial.print("Accelerometer scale: measured magnitude="); Serial.print(measuredMagG, 4);
    Serial.print("g (ideal 1.0000g), accelScaleG="); Serial.print(accelScaleG, 4);
    Serial.println(" m/s^2/g (was ideal G=9.8066).");
  } else {
    accelScaleG = G;
    Serial.print("WARNING: implausible at-rest accelerometer magnitude (");
    Serial.print(measuredMagG, 4);
    Serial.println("g, expected close to 1.0) - scale NOT corrected, using ideal G.");
  }
  accBiasVec[0] = accBiasVec[1] = accBiasVec[2] = 0;
  lastAccZBiasApplied = 0;
  resetQuietRings();
  lastSampleTime = millis();
  calibrated = true;

  float rangeX = maxGx - minGx, rangeY = maxGy - minGy, rangeZ = maxGz - minGz;
  float maxRange = max(rangeX, max(rangeY, rangeZ));
  if (maxRange > CALIBRATION_GYRO_RANGE_WARN_DEG_S) {
    Serial.print("WARNING: the device did not appear to be still during CALIBRATE "
                 "(gyroscope range ");
    Serial.print(maxRange, 1);
    Serial.println(" deg/s, expected <3). The estimated bias COULD be inaccurate: "
                    "repeat CALIBRATE while actually holding the device/bar still.");
  }
}

bool MotionTracker::isCalibrated() { return calibrated; }

void MotionTracker::setTrackingActive(bool active) {
  bool wasActive = trackingActive;
  trackingActive = active;
  resetTracking();
  if (serialLogActive()) {
    if (active) {
      Serial.println("REC_START");
      Serial.println("S,t_ms,dt_ms,state,rep,linAccX,linAccY,linAccZ,worldAccX,worldAccY,worldAccZ,"
                      "velZ,refVelZ,velZLive,repCalibRate,repCalibIntercept,repCalibCount,"
                      "worldVelX,worldVelY,worldPosZ,worldPosX,worldPosY,"
                      "quatW,quatX,quatY,quatZ,gyroMag,accZBias,gyroBiasX,gyroBiasY,gyroBiasZ,"
                      "gyroFilteredX,gyroFilteredY,gyroFilteredZ,"
                      "velClamped,bracketClosed,confirmedStillNow,flatGuardOverrideFired");
      Serial.println("R,rep,peakVelocity,meanVelocity,peakAcceleration,meanAcceleration,"
                      "displacementM,eccPeakVelocity,eccMeanVelocity,quality1,correctionStatus");
      // v3.11.7, NEW: one line per closed bracket - see the comment on
      // closeBracketFn() for the formula to reconstruct the corrected
      // curve from these fields + the "S," lines with t0<=t<=t1.
      Serial.println("B,bracketId,t0,t1,baseline,drift,rate,emaRateAfter,basisTotal");
      // v3.11.18, NEW: one line per phase close ATTEMPT (reason: 0=flat,
      // 1=reversal, 2=timeout), including ones discarded for being too
      // short (reported=0) - see the comment in closePhaseFn().
      Serial.println("P,phaseId,repNumber,phaseType,closeReason,durationS,sampleCount,discarded,reported");
    } else if (wasActive) {
      Serial.println("REC_STOP");
    }
  }
}

bool MotionTracker::isTrackingActive() { return trackingActive; }

void MotionTracker::setConfig(const RuntimeConfig& config) {
  activeConfig = config;
  // Defensive clamp: the fields below index the fixed-capacity sliding
  // windows (FLAT_RING_CAPACITY) - an out-of-range value arriving over
  // BLE from a client must not be able to corrupt memory.
  if (activeConfig.maxVelocityFlatWindowSamples > FLAT_RING_CAPACITY) activeConfig.maxVelocityFlatWindowSamples = FLAT_RING_CAPACITY;
  if (activeConfig.maxAccelerationFlatWindowSamples > FLAT_RING_CAPACITY) activeConfig.maxAccelerationFlatWindowSamples = FLAT_RING_CAPACITY;
  // v3.11.27: same protection for the above-ceiling fixed window - also
  // indexes into FLAT_RING_CAPACITY (see stepPhaseEngine()). 30 is the
  // documented minimum of its configurable 30-60 range; clamping the low
  // end too keeps an out-of-range BLE value from shrinking it back down
  // to something the peak-velocity ambiguity it exists to prevent could
  // exploit.
  if (activeConfig.velocityOverrideFlatWindowSamples > FLAT_RING_CAPACITY) activeConfig.velocityOverrideFlatWindowSamples = FLAT_RING_CAPACITY;
  if (activeConfig.velocityOverrideFlatWindowSamples < 30) activeConfig.velocityOverrideFlatWindowSamples = 30;
  if (activeConfig.minVelocityFlatWindowSamples < 1) activeConfig.minVelocityFlatWindowSamples = 1;
  if (activeConfig.minAccelerationFlatWindowSamples < 1) activeConfig.minAccelerationFlatWindowSamples = 1;
  if (activeConfig.minVelocityFlatWindowSamples > activeConfig.maxVelocityFlatWindowSamples)
    activeConfig.minVelocityFlatWindowSamples = (uint8_t)activeConfig.maxVelocityFlatWindowSamples;
  if (activeConfig.minAccelerationFlatWindowSamples > activeConfig.maxAccelerationFlatWindowSamples)
    activeConfig.minAccelerationFlatWindowSamples = (uint8_t)activeConfig.maxAccelerationFlatWindowSamples;
  repDirectionSign = (config.repDirection == RepDirection::Down) ? -1 : 1;
}

const RuntimeConfig& MotionTracker::currentConfig() { return activeConfig; }

const RuntimeConfig& MotionTracker::defaultConfig() { return DEFAULT_CONFIG; }

bool MotionTracker::update() {
  fastSamples.poll();

  unsigned long now = millis();
  if (now - lastSampleTime < SAMPLE_INTERVAL_MS) return false;
  float dt = (now - lastSampleTime) / 1000.0f;
  lastSampleTime = now;

  updateOrientationAndAcceleration(dt);

  if (!trackingActive) {
    sample.worldVelX = sample.worldVelY = 0;
    sample.refVelZ = 0;
    sample.worldPosX = sample.worldPosY = sample.worldPosZ = 0;
    sample.phaseState = PhaseState::Idle;
    return true;
  }

  integrateMotion(dt);

  sample.worldVelX = worldVelX; sample.worldVelY = worldVelY;
  sample.refVelZ = refVelZ;
  sample.worldPosX = worldPosX; sample.worldPosY = worldPosY; sample.worldPosZ = livePosZ;
  sample.phaseState = livePhaseState;
  return true;
}

const MotionSample& MotionTracker::currentSample() { return sample; }

bool MotionTracker::peekCompletedRep(RepResult& outResult) {
  if (pendingCount == 0) return false;
  outResult = pendingReps[pendingHead];
  return true;
}

void MotionTracker::popCompletedRep() {
  if (pendingCount == 0) return;
  pendingHead = (pendingHead + 1) % PENDING_REP_QUEUE_CAPACITY;
  pendingCount--;
}

bool MotionTracker::peekCorrectedCurveChunk(CorrectedCurveChunk& outChunk) {
  if (chunkCount == 0) return false;
  outChunk = chunkQueue[chunkHead];
  return true;
}

void MotionTracker::popCorrectedCurveChunk() {
  if (chunkCount == 0) return;
  chunkHead = (chunkHead + 1) % CHUNK_QUEUE_CAPACITY;
  chunkCount--;
}

const MotionDebugState& MotionTracker::debugState() { return debugStateVar; }
