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
Offline experiment: what would shock handling change in the velocity the engine reports?

Detection and classification are done here in Python, with look-ahead (a firmware
version would need a small delay buffer); the REAL engine (MotionTracker.cpp, see
replay.cpp) is then driven with the result. Variants, each adding one idea:

  old       median-filtered acceleration integrated (firmware <= v3.11.28)
  A raw     pre-median acceleration integrated (firmware >= v3.11.29)
  B +zupt   A + shocks followed by stillness (bar landed / racked) close the bracket at
            the shock start and re-anchor the velocity to zero once still again, so the
            error stays localized there instead of being spread over the whole bracket
  C +despike  B + shocks in the middle of motion (peak jerk > despike threshold) get the
            acceleration inside their window replaced by a linear interpolation
  D +skip   C + brackets / rep cycles that contain a shock don't teach emaRate / the
            rep-cycle regression

There is no ground-truth velocity in these logs. The metric used is the corrected
velocity at "known-zero" moments (bar certainly still: raw gyro and acceleration quiet for
250 ms) - closer to 0 is better - plus rep count and the spread of the rep displacement.

Usage: python shock_experiments.py LOG [LOG ...] [--outdir DIR] [--jerk 600] [--despike-jerk 1500]
"""

import argparse
import os
import sys

import numpy as np

import replay as R

QUIET_JERK = 250.0
STILL_GYRO = 25.0
STILL_ACC = 0.8
SETTLE = 15        # samples (150 ms) of stillness required after a burst
SETTLE_SEARCH = 60 # how long (samples) after the burst to look for that stillness
GYRO_GATE = 120.0  # a shock needs a raw gyroscope burst above this (deg/s) on top of the jerk spike
MAX_BURST = 15


def bursts_from(jerk, jerk_thr):
    n = len(jerk)
    out, i = [], 0
    while i < n:
        if jerk[i] > jerk_thr:
            b0 = max(i - 1, 1)
            j, quiet = i, 0
            while j < n - 1 and quiet < 3 and (j - b0) < MAX_BURST:
                j += 1
                quiet = quiet + 1 if jerk[j] < QUIET_JERK else 0
            b1 = j - quiet
            out.append((b0, b1))
            i = j + 1
        else:
            i += 1
    return out


def build_inputs(s, jerk_thr, despike_thr):
    ar = s["worldAccZRaw"].values.astype(float)
    af = s["worldAccZ"].values.astype(float)
    gyro = np.sqrt(s["gyroRawX"].values ** 2 + s["gyroRawY"].values ** 2 + s["gyroRawZ"].values ** 2)
    dt = np.maximum(s["dt_ms"].values / 1000.0, 1e-3)
    jerk = np.zeros(len(ar)); jerk[1:] = np.abs(np.diff(ar)) / dt[1:]
    n = len(ar)
    zupt = np.zeros(n); flag = np.zeros(n)
    despiked = ar.copy()
    events = []
    for (b0, b1) in bursts_from(jerk, jerk_thr):
        hi = min(b1 + 8, n - 1)
        if gyro[b0:hi + 1].max() < GYRO_GATE:
            continue      # a jerk spike without a gyroscope burst: normal motion (squat reversal), not a shock
        # still afterwards? look for SETTLE consecutive quiet samples within SETTLE_SEARCH samples
        e, run = None, 0
        for k in range(b1 + 1, min(b1 + SETTLE_SEARCH, n - 1) + 1):
            run = run + 1 if (gyro[k] < STILL_GYRO and abs(af[k]) < STILL_ACC) else 0
            if run >= SETTLE:
                e = k
                break
        still = e is not None
        end = e if still else min(b1 + SETTLE, n - 1)
        flag[b0:end + 1] = 1
        kind = "still" if still else "motion"
        if still:
            zupt[b0] = 1                      # cmd 1: close the bracket before the burst
            zupt[e] = 2                       # cmd 2: re-anchor once still again
        events.append((b0, b1, kind, float(jerk[b0:b1 + 1].max())))
    mot_desp = ar.copy()
    for (b0, b1, kind, pj) in events:
        if kind == "motion" and pj > despike_thr:
            lo, hi = max(b0 - 1, 0), min(b1 + 1, n - 1)
            x = np.arange(lo, hi + 1)
            mot_desp[lo:hi + 1] = ar[lo] + (ar[hi] - ar[lo]) * (x - lo) / max(hi - lo, 1)
    return ar, af, mot_desp, zupt, flag, events


def known_zero_mask(s, min_run=25, skip=10):
    gyro = np.sqrt(s["gyroRawX"].values ** 2 + s["gyroRawY"].values ** 2 + s["gyroRawZ"].values ** 2)
    quiet = (gyro < 8.0) & (np.abs(s["worldAccZ"].values) < 0.4) & (np.abs(s["worldAccZRaw"].values) < 0.6)
    mask = np.zeros(len(quiet), bool)
    i = 0
    while i < len(quiet):
        if quiet[i]:
            j = i
            while j < len(quiet) and quiet[j]:
                j += 1
            if j - i >= min_run:
                mask[i + skip:j] = True
            i = j
        else:
            i += 1
    return mask


def run(binary, v, s, af, ai, zupt=None, flag=None):
    n = len(af)
    zupt = np.zeros(n) if zupt is None else zupt
    flag = np.zeros(n) if flag is None else flag
    inp = "\n".join("%.3f,%.0f,%.5f,%.5f,%d,%d" % row for row in zip(
        s["dt_ms"].values, s["t_ms"].values, af, ai, zupt, flag))
    import subprocess
    out = subprocess.run([binary], input=inp, capture_output=True, text=True, check=True).stdout
    s2, r2, b2 = v.parse_lines(out.split("\n"))
    return v.reconstruct_corrected(s2, b2), r2, b2, out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--outdir", default=None)
    ap.add_argument("--jerk", type=float, default=600.0, help="raw jerk (m/s^3) that opens a shock")
    ap.add_argument("--despike-jerk", type=float, default=1500.0, help="peak jerk above which a mid-motion shock is despiked")
    args = ap.parse_args()
    outdir = args.outdir or R.HERE
    os.makedirs(outdir, exist_ok=True)
    v = R.load_viewer()
    binary = os.path.join(outdir, "replay_bin")
    R.build(binary)

    summary = {}
    for path in args.logs:
        name = os.path.basename(path).replace(".log", "")
        ss = v.split_sessions(path)
        s, _r, _b = v.parse_lines(max(ss, key=lambda l: sum(1 for x in l if x.startswith("S,"))))
        if s["worldAccZRaw"].isna().any():
            print(f"{name}: no worldAccZRaw column, skipped"); continue
        ar, af, mot_desp, zupt, flag, events = build_inputs(s, args.jerk, args.despike_jerk)
        t = s["sessionT"].values
        print(f"\n=========== {name}  ({t[-1]:.0f}s) - shocks (raw jerk > {args.jerk:.0f} AND raw gyro > {GYRO_GATE:.0f} deg/s):")
        for (b0, b1, kind, pj) in events:
            print(f"   t={t[b0]:6.2f}s  {b1 - b0 + 1:2d} samples  peak jerk {pj:5.0f}  -> {'bar STILL afterwards (boundary event)' if kind == 'still' else 'motion continues'}")
        zero = known_zero_mask(s)
        d_ar = np.where(flag > 0, mot_desp, ar)   # (mot_desp already only touches mid-motion shocks)
        variants = {
            "old (median)":  dict(af=af, ai=af),
            "A raw":         dict(af=af, ai=ar),
            "B +zupt":       dict(af=af, ai=ar, zupt=zupt),
            "C +despike":    dict(af=af, ai=mot_desp, zupt=zupt),
            "D +skip est.":  dict(af=af, ai=mot_desp, zupt=zupt, flag=flag),
        }
        res = {}
        for vn, kw in variants.items():
            s2, r2, b2, out = run(binary, v, s, kw["af"], kw["ai"], kw.get("zupt"), kw.get("flag"))
            open(os.path.join(outdir, f"exp_{name}_{vn.split()[0]}.log"), "w").write(out)
            res[vn] = (s2, r2, b2)
        common = zero.copy()
        for vn, (s2, r2, b2) in res.items():
            common &= np.isfinite(s2["corrected"].values)
        print(f"\n{'variant':<14}{'reps':>5}{'brackets':>9}{'rms|corr| @ zero*':>19}{'disp CV (reps>0.3m)':>21}   displacement per rep (m)")
        rows = {}
        for vn, (s2, r2, b2) in res.items():
            c = s2["corrected"].values
            rms = float(np.sqrt(np.mean(c[common] ** 2))) if common.sum() else float("nan")
            d_all = r2.sort_values("rep")["displacementM"].values
            d = d_all[d_all > 0.3]
            cv = float(np.std(d) / np.mean(d) * 100) if len(d) > 1 else float("nan")
            rows[vn] = (len(r2), len(b2), rms, 0, cv, int(common.sum()))
            print(f"{vn:<14}{len(r2):>5}{len(b2):>9}{rms:>19.3f}{cv:>20.1f}%   " + " ".join(f"{x:.2f}" for x in d_all))
        print(f"   (* known-zero samples that have a corrected value in EVERY variant: n={int(common.sum())}; the engine closes brackets at stillness, so this metric is weak)")
        summary[name] = rows

        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        fig, axs = plt.subplots(2, 1, figsize=(15, 8), sharex=True)
        col = {"old (median)": "#c9392c", "A raw": "#1f7a5a", "B +zupt": "#1f4fb4", "C +despike": "#8e44ad", "D +skip est.": "#e08a00"}
        for vn, (s2, r2, b2) in res.items():
            axs[0].plot(s2["sessionT"], s2["velZ"], color=col[vn], lw=1.1, label=vn)
            axs[1].plot(s2["sessionT"], s2["corrected"], color=col[vn], lw=1.1, label=vn)
        for (b0, b1, kind, pj) in events:
            for ax in axs:
                ax.axvline(t[b0], color="red" if kind == "still" else "#999", lw=0.8, alpha=0.6, ls="-" if kind == "still" else ":")
        axs[0].set_ylabel("velZ - raw integral (m/s)"); axs[1].set_ylabel("corrected velocity (m/s)")
        axs[1].set_xlabel("session time (s)  -  red line: shock followed by stillness, dotted: shock in motion")
        for ax in axs:
            ax.axhline(0, color="#888", lw=0.6); ax.grid(alpha=0.25); ax.legend(loc="upper right", fontsize=8, ncol=2)
        fig.suptitle(f"{name} - shock handling experiments (real engine, offline)")
        fig.tight_layout(); fig.savefig(os.path.join(outdir, f"exp_{name}.png"), dpi=120); plt.close(fig)

    print("\n\n############ SUMMARY - rms |corrected velocity| at known-zero moments (m/s, lower is better)")
    names = list(next(iter(summary.values())).keys()) if summary else []
    print(f"{'log':<22}" + "".join(f"{n:>15}" for n in names))
    for lg, rows in summary.items():
        print(f"{lg:<22}" + "".join(f"{rows[n][2]:>15.3f}" for n in names))
    print(f"\n{'reps reported':<22}" + "".join(f"{n:>15}" for n in names))
    for lg, rows in summary.items():
        print(f"{lg:<22}" + "".join(f"{rows[n][0]:>15d}" for n in names))


if __name__ == "__main__":
    main()
