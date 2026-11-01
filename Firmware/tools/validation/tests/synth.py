#!/usr/bin/env python3
# Part of the ODKI VBT Firmware
# Copyright (C) 2026 Lodovico Cortelazzo
#
# Licensed under the GNU General Public License, Version 3 (this
# specific version only, not "or any later version"), modified by the
# Commons Clause License Condition v1.0 -- see LICENSE-FIRMWARE in the
# repository root for the full text of both. In short: you may use,
# study, modify, and share this file (including a modified version) for
# non-commercial purposes; you may not sell it, or a product/service
# substantially derived from it, without a separate agreement with the
# copyright holder.
#
# This program is distributed WITHOUT ANY WARRANTY, without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
# PURPOSE -- see LICENSE-FIRMWARE for details.

"""
SYNTHETIC dataset generator for the pipeline self-test. Nothing produced
here is a measurement: every file is computed from a made-up bar motion.

It writes a complete dataset folder exactly as capture.py and the session
sheets would (PROTOCOL.md section 12): ODKI serial logs in the firmware's
S/R/B/P format (half of them with the v3.11.32 P-row timestamps, half
without), the capture sidecars, the reference recordings as import_ref.py
writes them (t_s,position_m at REF_HZ, on the encoder's own clock: started
a few seconds before the motion, running 40 ppm fast, reading upward motion
as negative and REF_SCALE_ERROR too long, with 0.1 mm resolution), and the
metadata CSVs, including the static verification that corrects the scale.
Known truths:

  final ODKI MV  = reference-definition MV + BIAS + N(0, SD) per unit
  first ODKI MV  = ... + BIAS + N(0, SD_FIRST)
  ODKI signal delayed by ODKI_DELAY_S, host arrival jitter 1-4 ms
  some reps silently missed by ODKI (FN), some ODKI-only reps (FP)

Usage (normally called by test_pipeline.py):
    python synth.py OUT_DIR
"""

import os
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import common as C  # noqa: E402

SEED = 2026
BIAS = 0.030              # m/s, final MV
SD = 0.040                # m/s, final MV
SD_FIRST = 0.060          # m/s, first-reported MV
PV_BIAS = 0.050
DRIFT_PPM = 40.0          # reference clock vs ODKI clock
ODKI_DELAY_S = 0.015      # ODKI signal lag vs the true motion
REF_SCALE_ERROR = 1.015   # the reference reads 1.5% long; the static verification below measures it
REF_HZ = 500.0            # reference sampling rate
REF_RES_M = 0.0001        # reference position resolution
FS_TRUE = 1000.0
LIFTERS = ["L-07", "L-23", "L-41"]
EXERCISES = {"back_squat": (1.45, 0.0125, 0.50), "bench_press": (1.60, 0.0150, 0.35)}  # MV = a - b*pct ; ROM m
LOADS = [(20, 3), (50, 3), (70, 2), (90, 1)]   # (% 1RM, reps)
UNITS = ["U1", "U2"]
LONG_SET_LIFTER, LONG_SET_REPS = "L-07", 8
MISSED = {("L-07", 1, "back_squat", 50, "U1"): 2, ("L-23", 2, "bench_press", 20, "U2"): 3,
          ("L-41", 1, "bench_press", 70, "U1"): 2}            # (set key) -> physical rep ODKI misses
EXTRA = {("L-23", 1, "back_squat", 20, "U2"), ("L-41", 2, "back_squat", 50, "U1")}  # ODKI-only rep at the end


def true_motion(rng, mv_target, rom, n_reps):
    """1 kHz true bar velocity; returns t, v, concentric (start, end) times."""
    segs, conc = [], []
    t = 0.0

    def add(dur, fn):
        nonlocal t
        n = int(round(dur * FS_TRUE))
        tt = np.arange(n) / FS_TRUE
        segs.append(fn(tt, dur))
        t += n / FS_TRUE

    add(1.5, lambda tt, d: np.zeros_like(tt))
    # sync event: lift ~3 cm, let it drop back onto the hooks (abrupt stop)
    add(0.25, lambda tt, d: 0.20 * np.sin(np.pi * tt / d))
    add(0.12, lambda tt, d: -0.45 * np.sin(0.5 * np.pi * tt / d))
    add(2.0, lambda tt, d: np.zeros_like(tt))
    for _ in range(n_reps):
        mv = mv_target * (1 + rng.normal(0, 0.03))
        dc = rom / mv
        vp = mv * np.pi / 2
        de = 1.0
        add(de, lambda tt, d: -(rom * np.pi / (2 * d)) * np.sin(np.pi * tt / d))
        add(0.15, lambda tt, d: np.zeros_like(tt))
        t0 = t
        add(dc, lambda tt, d, vp=vp: vp * np.sin(np.pi * tt / d))
        conc.append((t0, t))
        add(1.6, lambda tt, d: np.zeros_like(tt))
    add(2.5, lambda tt, d: np.zeros_like(tt))
    v = np.concatenate(segs)
    return np.arange(len(v)) / FS_TRUE, v, conc


def write_set(out, set_id, phys_id, rng, t, v, conc, missed_rep, extra, p_v2, enc_written):
    # ---------- ODKI samples (device clock = true time + T0, signal delayed) ----------
    T0 = 812.345 + rng.uniform(0, 5)
    dt = 0.010 + rng.normal(0, 0.00002, int((t[-1] - 0.2) / 0.010))
    tt = np.cumsum(dt) + 0.05                      # true time of each sample
    tt = tt[tt < t[-1] - 0.1]
    dt = np.r_[tt[0] - 0.04, np.diff(tt)]
    # what ODKI "sees": the bar velocity as a centred 10 ms mean (the same
    # smoothing the reference velocity gets), delayed by ODKI_DELAY_S
    x_true = np.cumsum(v) / FS_TRUE
    v_o = (np.interp(tt - ODKI_DELAY_S + 0.005, t, x_true) - np.interp(tt - ODKI_DELAY_S - 0.005, t, x_true)) / 0.010
    if extra:  # ODKI-only bump during the final rest
        tb = t[-1] - 1.6
        m = (tt > tb) & (tt < tb + 0.5)
        v_o[m] += 0.6 * np.sin(np.pi * (tt[m] - tb) / 0.5)
    noise = rng.normal(0, 0.004, len(tt))
    k_drift, d0 = rng.uniform(-0.02, 0.02), rng.uniform(-0.1, 0.1)
    session_t = np.cumsum(dt)
    velz = v_o + noise + d0 + k_drift * session_t
    acc = np.r_[0.0, np.diff(v_o) / dt[1:]] + rng.normal(0, 0.05, len(tt))
    t_dev = T0 + tt
    t_ms = np.floor(t_dev * 1000).astype(int)
    state = np.where(v_o > 0.01, "concentric", np.where(v_o < -0.01, "eccentric", "idle"))

    # rep numbering and R/P/B rows, keyed by the sample index they're printed after
    events = {}
    rep_col = np.ones(len(tt), int)
    rep_no = 1
    conc_true = [(a + ODKI_DELAY_S, b + ODKI_DELAY_S) for a, b in conc]
    if extra:
        conc_true.append((t[-1] - 1.6, t[-1] - 1.1))
    reported = []
    for j, (a, b) in enumerate(conc_true):
        if missed_rep == j + 1:
            continue
        # the concentric = the run of v_o > 0 around the true one (same rule as the reference)
        mid = np.searchsorted(tt, 0.5 * (a + b))
        i0, i1 = mid, mid
        while i0 > 0 and v_o[i0 - 1] > 0:
            i0 -= 1
        while i1 < len(tt) - 1 and v_o[i1 + 1] > 0:
            i1 += 1
        idx_close = min(i1 + 12, len(tt) - 1)
        rep_col[idx_close:] = rep_no + 1
        mv_def = C.mv_odki_definition(v_o, i0, i1)
        pv = float(v_o[i0:i1 + 1].max())
        rom = float(np.sum(np.clip(v_o[i0:i1 + 1], 0, None) * dt[i0:i1 + 1]))
        first = [rep_no, pv + PV_BIAS + rng.normal(0, 0.06), mv_def + BIAS + rng.normal(0, SD_FIRST),
                 9.1, 3.2, rom + rng.normal(0, 0.02), 0.9, 0.5, int(rng.integers(70, 101)), 1]
        final = [rep_no, pv + PV_BIAS + rng.normal(0, 0.04), mv_def + BIAS + rng.normal(0, SD),
                 9.0, 3.1, rom + rng.normal(0, 0.01), 0.9, 0.5, int(rng.integers(70, 101)), 2]
        p_row = [2 * rep_no - 1, rep_no, 1, 1, round(tt[i1] - tt[i0], 4), i1 - i0 + 1, 0, 1]
        if p_v2:
            p_row += [int(t_ms[i0]), int(t_ms[i1])]
        events.setdefault(idx_close, []).append("P," + ",".join(str(x) for x in p_row))
        events.setdefault(idx_close, []).append("R," + ",".join(f"{x:.4f}" if isinstance(x, float) else str(x)
                                                                for x in first))
        events.setdefault(min(idx_close + 60, len(tt) - 1), []).append(
            "R," + ",".join(f"{x:.4f}" if isinstance(x, float) else str(x) for x in final))
        reported.append((rep_no, mv_def, final[2], first[2]))
        rep_no += 1
    # one bracket over the whole set closes the drift exactly (B row at the end)
    t0b, t1b = session_t[0], session_t[-1]
    events.setdefault(len(tt) - 1, []).append(
        f"B,0,{t0b:.4f},{t1b:.4f},{d0 + k_drift * t0b:.5f},{k_drift * (t1b - t0b):.5f},0,0,{t1b - t0b:.4f}")

    lines = ["Battery: 4.10 V (85%)", "START (serial): tracking active.", "REC_START",
             "S,t_ms,dt_ms,state,rep,linAccX,linAccY,linAccZ,worldAccX,worldAccY,worldAccZ,velZ,refVelZ,velZLive,"
             "repCalibRate,repCalibIntercept,repCalibCount,worldVelX,worldVelY,worldPosZ,worldPosX,worldPosY,"
             "quatW,quatX,quatY,quatZ,gyroMag,accZBias,gyroBiasX,gyroBiasY,gyroBiasZ,gyroFilteredX,gyroFilteredY,"
             "gyroFilteredZ,velClamped,bracketClosed,confirmedStillNow,flatGuardOverrideFired,worldAccZRaw,"
             "gyroRawX,gyroRawY,gyroRawZ",
             "R,rep,peakVelocity,meanVelocity,peakAcceleration,meanAcceleration,displacementM,eccPeakVelocity,"
             "eccMeanVelocity,quality1,correctionStatus",
             "B,bracketId,t0,t1,baseline,drift,rate,emaRateAfter,basisTotal",
             "P,phaseId,repNumber,phaseType,closeReason,durationS,sampleCount,discarded,reported"]
    line_t = [t_dev[0] - 0.01] * len(lines)
    z = "0"
    for i in range(len(tt)):
        a = f"{acc[i]:.4f}"
        row = [str(t_ms[i]), f"{dt[i] * 1000:.3f}", state[i], str(rep_col[i]), z, z, a, z, z, a,
               f"{velz[i]:.4f}", f"{v_o[i]:.4f}", f"{v_o[i]:.4f}", z, z, z, z, z, z, z, z,
               "1", z, z, z, "5.0", z, z, z, z, z, z, z, z, "0" if i != len(tt) - 1 else "1", z, z, a, z, z, z]
        lines.append("S," + ",".join(row))
        line_t.append(t_dev[i])
        for e in events.get(i, []):
            lines.append(e)
            line_t.append(t_dev[i])
    lines += ["REC_STOP", "STOP (serial): tracking stopped."]
    line_t += [t_dev[-1]] * 2

    host_true0 = 50.0  # host clock = true time + host_true0
    os.makedirs(os.path.join(out, "raw", "odki"), exist_ok=True)
    with open(os.path.join(out, "raw", "odki", f"{set_id}.log"), "w") as f:
        f.write("\n".join(lines) + "\n")
    lat = rng.uniform(0.001, 0.004, len(lines))
    host = (np.array(line_t) - T0 + host_true0 + lat) * 1e9
    pd.DataFrame({"line_no": np.arange(len(lines)), "host_ns": host.astype(np.int64),
                  "kind": [ln[:1] for ln in lines]}).to_csv(
        os.path.join(out, "raw", "odki", f"{set_id}.sidecar.csv"), index=False)

    # ---------- reference: one recording per physical set, on its own clock ----------
    if phys_id not in enc_written:
        lead = rng.uniform(2.0, 6.0)               # recording started this long before the motion
        te_k = np.arange(0.0, (t[-1] + lead) * (1 + DRIFT_PPM * 1e-6) - 0.02, 1.0 / REF_HZ)
        t_true_k = te_k / (1 + DRIFT_PPM * 1e-6) - lead
        x = np.cumsum(v) / FS_TRUE
        pos = -REF_SCALE_ERROR * np.interp(t_true_k, t, x, left=0.0)   # reads upward motion as negative
        pos = np.round(pos / REF_RES_M) * REF_RES_M
        os.makedirs(os.path.join(out, "raw", "ref"), exist_ok=True)
        pd.DataFrame({"t_s": te_k, "position_m": pos}).to_csv(
            os.path.join(out, "raw", "ref", f"{phys_id}.csv"), index=False, float_format="%.6f")
        enc_written.add(phys_id)
    return reported, tt, v_o


def generate(out):
    rng = np.random.default_rng(SEED)
    os.makedirs(out, exist_ok=True)
    sessions, sets, truth, rep_truth = [], [], [], []
    enc_written = set()
    for li, lifter in enumerate(LIFTERS):
        offset = rng.normal(0, 0.05)
        for day in (1, 2):
            for ex, (a, b, rom) in EXERCISES.items():
                sid = f"{lifter}-D{day}-{ex}"
                sessions.append({"session_id": sid, "lifter_id": lifter, "day_index": day, "exercise": ex,
                                 "firmware_version": "synthetic", "firmware_commit": "synthetic",
                                 "runtime_config_hex": "", "reference_type": "linear_encoder",
                                 "reference_id": "REF-SYN", "operator": "ODKI", "notes": "SYNTHETIC"})
                for pct, n_reps in LOADS:
                    if lifter == LONG_SET_LIFTER and pct == LOADS[0][0]:
                        n_reps = LONG_SET_REPS  # one long set per session, to exercise the drift estimate
                    mv = a - b * pct + offset + rng.normal(0, 0.02)
                    t, v, conc = true_motion(rng, mv, rom, n_reps)
                    phys = f"{sid}-{pct:03d}-S1"
                    for ui, unit in enumerate(UNITS):
                        key = (lifter, day, ex, pct, unit)
                        set_id = f"{phys}_{unit}"
                        reported, _, _ = write_set(out, set_id, phys, rng, t, v, conc, MISSED.get(key),
                                                   key in EXTRA, p_v2=(unit == "U2"), enc_written=enc_written)
                        sets.append({"set_id": set_id, "physical_set_id": phys, "session_id": sid, "unit_id": unit,
                                     "load_band_kg": "SYN", "pct_1rm_est": pct, "planned_reps": n_reps,
                                     "video": "no"})
                        truth.append({"set_id": set_id, "n_ref": n_reps, "missed": int(key in MISSED),
                                      "extra": int(key in EXTRA)})
                        rep_truth += [{"set_id": set_id, "odki_rep_number": r, "mv_def_true": m,
                                       "odki_mv_final": f, "odki_mv_first": fi} for r, m, f, fi in reported]
    pd.DataFrame(sessions).to_csv(os.path.join(out, "sessions.csv"), index=False)
    pd.DataFrame(sets).to_csv(os.path.join(out, "sets.csv"), index=False)
    pd.DataFrame({"lifter_id": LIFTERS, "age_band": ["30-39"] * 3, "training_years_band": ["1-3"] * 3}).to_csv(
        os.path.join(out, "lifters.csv"), index=False)
    ver = [{"reference_id": "REF-SYN", "test": "static", "trial": i + 1, "expected": 1.000,
            "measured": round(REF_SCALE_ERROR + rng.normal(0, 0.0005), 4), "unit": "m"} for i in range(10)]
    pd.DataFrame(ver).to_csv(os.path.join(out, "reference_verification.csv"), index=False)
    pd.DataFrame(truth).to_csv(os.path.join(out, "_synthetic_truth.csv"), index=False)
    pd.DataFrame(rep_truth).to_csv(os.path.join(out, "_synthetic_reps.csv"), index=False)
    return pd.DataFrame(truth)


if __name__ == "__main__":
    generate(sys.argv[1] if len(sys.argv) > 1 else "synthetic_data")
