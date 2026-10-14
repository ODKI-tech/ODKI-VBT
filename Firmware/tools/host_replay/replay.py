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
Offline simulation: what would the engine report if its input were a different
vertical acceleration signal?

Takes a debug log recorded with firmware >= v3.11.28 (it has both worldAccZ -
after the 5-sample median - and worldAccZRaw - before it), feeds the REAL
tracking engine (MotionTracker.cpp compiled unchanged on the PC, see
replay.cpp) with each variant of the input and compares the results:

  filtered : integrate worldAccZ (median)         - firmware <= v3.11.28
  raw      : integrate worldAccZRaw, flat-guard and reported acceleration still on the
             median-filtered worldAccZ                - firmware >= v3.11.29
  raw-all  : worldAccZRaw everywhere, flat-guard included (reference)
  hybrid   : like raw, but only inside detected shock windows (reference)

The shock detector of the hybrid variant is deliberately simple (raw jerk or
raw |acc| over a threshold) - it is what is being evaluated, not a final design.

Usage:
    python replay.py capture.log [--outdir DIR] [--jerk 1500] [--acc 25]
Requires numpy, pandas, matplotlib and a C++ compiler (clang++ / g++).
"""

import argparse
import importlib.util
import os
import shutil
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
FW = os.path.join(os.path.dirname(TOOLS), "VBT_Quaternions")


def load_viewer():
    spec = importlib.util.spec_from_file_location("vbt_log_viewer", os.path.join(TOOLS, "vbt_log_viewer.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def build(binary):
    cxx = shutil.which("clang++") or shutil.which("g++")
    if not cxx:
        sys.exit("No C++ compiler found (clang++/g++).")
    cmd = [cxx, "-std=c++17", "-O2", "-w", "-I", os.path.join(HERE, "stubs"), "-I", FW,
           os.path.join(HERE, "replay.cpp"), os.path.join(FW, "MadgwickAHRS.cpp"), "-o", binary]
    subprocess.run(cmd, check=True)


def run_engine(binary, dt_ms, t_ms, acc_filtered, acc_integration):
    inp = "\n".join("%.3f,%.0f,%.5f,%.5f" % x for x in zip(dt_ms, t_ms, acc_filtered, acc_integration))
    return subprocess.run([binary], input=inp, capture_output=True, text=True, check=True).stdout


def shock_windows(s_df, jerk_thr, acc_thr, pre=2, post=8, merge=15):
    """Boolean mask of samples inside a detected shock window (raw jerk or raw |acc| over threshold)."""
    ar = s_df["worldAccZRaw"].values
    dt = np.maximum(s_df["dt_ms"].values / 1000.0, 1e-3)
    jerk = np.zeros(len(ar)); jerk[1:] = np.abs(np.diff(ar)) / dt[1:]
    idx = np.where((jerk > jerk_thr) | (np.abs(ar) > acc_thr))[0]
    mask = np.zeros(len(ar), bool)
    start = prev = None
    for i in idx:
        if start is None:
            start = prev = i
        elif i - prev <= merge:
            prev = i
        else:
            mask[max(start - pre, 0):prev + post + 1] = True
            start = prev = i
    if start is not None:
        mask[max(start - pre, 0):prev + post + 1] = True
    return mask


def rest_mask(s_df, centers, lo=0.4, hi=2.0, gyro_max=10.0):
    t = s_df["sessionT"].values
    gm = s_df["gyroMag"].values
    m = np.zeros(len(t), bool)
    for k, c in enumerate(centers):
        nxt = centers[k + 1] - 0.3 if k + 1 < len(centers) else t[-1]
        m |= (t > c + lo) & (t < min(c + hi, nxt)) & (gm < gyro_max)
    return m


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logfile")
    ap.add_argument("--outdir", default=None, help="where to write the replayed logs and the plot (default: next to this script)")
    ap.add_argument("--jerk", type=float, default=1500.0, help="raw jerk threshold (m/s^3) of the hybrid shock detector")
    ap.add_argument("--acc", type=float, default=25.0, help="raw |acc| threshold (m/s^2) of the hybrid shock detector")
    args = ap.parse_args()
    outdir = args.outdir or HERE
    os.makedirs(outdir, exist_ok=True)

    v = load_viewer()
    sessions = v.split_sessions(args.logfile)
    s_df, _r, b_df = v.parse_lines(max(sessions, key=lambda l: sum(1 for x in l if x.startswith("S,"))))
    if s_df["worldAccZRaw"].isna().any():
        sys.exit("This log has no worldAccZRaw column: record it with firmware v3.11.28 or newer.")

    binary = os.path.join(outdir, "replay_bin")
    build(binary)

    a_f, a_r = s_df["worldAccZ"].values, s_df["worldAccZRaw"].values
    mask = shock_windows(s_df, args.jerk, args.acc)
    # name -> (acceleration for the flat-guard/metrics, acceleration integrated into velZ)
    variants = {"filtered": (a_f, a_f), "raw": (a_f, a_r), "raw-all": (a_r, a_r),
                "hybrid": (a_f, np.where(mask, a_r, a_f))}
    print(f"hybrid reference: {int(mask.sum())} samples ({mask.sum() / len(mask) * 100:.1f}%) integrated from the raw signal")

    res = {}
    for name, (acc_f, acc_i) in variants.items():
        out = run_engine(binary, s_df["dt_ms"].values, s_df["t_ms"].values, acc_f, acc_i)
        with open(os.path.join(outdir, f"replay_{name}.log"), "w") as f:
            f.write(out)
        s2, r2, b2 = v.parse_lines(out.split("\n"))
        res[name] = (v.reconstruct_corrected(s2, b2), r2, b2)

    t = s_df["sessionT"].values
    # impacts = clusters of raw jerk/acc over the thresholds
    ev_idx = np.where(mask)[0]
    centers = []
    for i in ev_idx:
        if not centers or t[i] - centers[-1] > 0.4:
            centers.append(t[i])
    centers = [c + 0.03 for c in centers]

    print(f"\nshock windows found at t = {', '.join(f'{c:.2f}s' for c in centers)}")
    print("\n=== velocity at rest after the impacts (raw integral velZ, bar standing still; ideal = anchor before the rep) ===")
    print(f"{'variant':<9}" + "".join(f"{f'#{k + 1} @{c:.1f}s':>12}" for k, c in enumerate(centers)))
    for name, (s2, r2, b2) in res.items():
        vals = []
        for k, c in enumerate(centers):
            one = rest_mask(s_df, centers, 0.4, 2.0)
            lo, hi = c + 0.4, (centers[k + 1] - 0.3 if k + 1 < len(centers) else t[-1])
            mk = one & (t > lo) & (t < hi)
            vals.append(np.median(s2["velZ"].values[mk]) if mk.sum() > 5 else np.nan)
        print(f"{name:<9}" + "".join(f"{x:>+12.2f}" for x in vals))

    print("\n=== drift the engine had to remove (|drift| measured at each bracket close, m/s; 0 = nothing to correct) ===")
    for name, (s2, r2, b2) in res.items():
        d = np.abs(b2["drift"].values)
        long_ = np.abs(b2.loc[b2["basisTotal"] > 0.5, "drift"].values)
        print(f"  {name:<9} brackets {len(b2):>2}   max {d.max():.2f}   median {np.median(d):.3f}   "
              f"brackets > 0.5 s: {len(long_)}, max |drift| {long_.max() if len(long_) else float('nan'):.2f}")

    print("\n=== rep metrics (concentric peak / mean velocity, m/s) ===")
    base = res["filtered"][1].set_index("rep")
    print(f"{'rep':>3}  " + "  ".join(f"{n:>17}" for n in res))
    for rep in base.index:
        cells = []
        for name, (s2, r2, b2) in res.items():
            rr = r2.set_index("rep")
            cells.append(f"{rr.loc[rep, 'peakVelocity']:6.3f} / {rr.loc[rep, 'meanVelocity']:6.3f}" if rep in rr.index else "      n/a")
        print(f"{rep:>3}  " + "  ".join(f"{c:>17}" for c in cells))

    print("\n=== engine behaviour ===")
    for name, (s2, r2, b2) in res.items():
        print(f"  {name:<9} reps {len(r2)}  brackets {len(b2)}  flatGuardOverride {int(s2['flatGuardOverrideFired'].sum())}  "
              f"velClamped {int(s2['velClamped'].sum())}")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    colors = {"filtered": "#c9392c", "raw": "#1f7a5a", "raw-all": "#8e44ad", "hybrid": "#1f4fb4"}
    fig, axs = plt.subplots(3, 1, figsize=(15, 10), sharex=True, gridspec_kw={"height_ratios": [2.2, 2.2, 1.4]})
    for name, (s2, r2, b2) in res.items():
        axs[0].plot(s2["sessionT"], s2["velZ"], color=colors[name], lw=1.2, label=name)
        axs[1].plot(s2["sessionT"], s2["corrected"], color=colors[name], lw=1.2, label=name)
    axs[2].plot(t, a_f, color=colors["filtered"], lw=0.9, label="worldAccZ (median)")
    axs[2].plot(t, a_r, color=colors["raw"], lw=0.7, alpha=0.8, label="worldAccZ raw")
    axs[0].set_ylabel("velZ - raw integral (m/s)"); axs[1].set_ylabel("corrected velocity (m/s)"); axs[2].set_ylabel("accel Z (m/s²)")
    for ax in axs:
        ax.axhline(0, color="#888", lw=0.6)
        for c in centers:
            ax.axvline(c, color="#999", lw=0.6, ls=":")
        ax.legend(loc="upper right", fontsize=8); ax.grid(alpha=0.25)
    axs[2].set_xlabel("session time (s)")
    fig.suptitle("Engine replay with different acceleration inputs")
    fig.tight_layout()
    png = os.path.join(outdir, "replay_comparison.png")
    fig.savefig(png, dpi=130)
    print(f"\nplot: {png}\nreplayed logs: {outdir}/replay_<variant>.log (open them with vbt_log_viewer.py)")


if __name__ == "__main__":
    main()
