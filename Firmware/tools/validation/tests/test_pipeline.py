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
Self-test of the validation pipeline on SYNTHETIC data (tests/synth.py).

  1. statistics against published worked examples (Shrout & Fleiss 1979 ICCs)
  2. import_ref.py on a velocity-only export with ';' fields and decimal
     commas, times in ms with an offset
  3. end to end: generate a dataset with a known bias, noise, clock offset
     (no shared host clock), clock drift, reference scale error, sign
     inversion, missed and false reps; run align.py,
     pair_reps.py and analyze.py through their command lines; assert that
     the alignment, the confusion matrix and the agreement statistics come
     back as built in

Usage:
    python tests/test_pipeline.py [--keep DIR]
Exit code 0 = every check passed.
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)
import analyze  # noqa: E402
import import_ref  # noqa: E402
import synth  # noqa: E402

FAILS = []


def check(name, ok, detail=""):
    print(f"  [{'ok' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))
    if not ok:
        FAILS.append(name)


def test_statistics():
    print("statistics vs worked examples")
    # Shrout & Fleiss 1979, Table 2: 6 targets x 4 judges -> ICC(2,1)=0.29, ICC(3,1)=0.71
    y = [[9, 2, 5, 8], [6, 1, 3, 2], [8, 4, 6, 8], [7, 1, 2, 6], [10, 5, 6, 9], [6, 2, 4, 7]]
    a1, c1 = analyze.icc_two_way(y)
    check("ICC(2,1) Shrout & Fleiss = 0.29", abs(a1 - 0.29) < 0.005, f"{a1:.4f}")
    check("ICC(3,1) Shrout & Fleiss = 0.71", abs(c1 - 0.71) < 0.005, f"{c1:.4f}")
    # repeated-measures BA: subject-level bias spread must widen the SD
    rng = np.random.default_rng(1)
    subj = np.repeat(np.arange(10), 30)
    d = rng.normal(0, 0.03, 300) + np.repeat(rng.normal(0, 0.04, 10), 30)
    sd_rm = analyze.ba_repeated(d, subj)
    check("repeated-measures BA SD ~ sqrt(0.03^2 + 0.04^2)", abs(sd_rm - 0.05) < 0.012, f"{sd_rm:.4f}")
    lo, hi = analyze.wilson(98, 100)
    check("Wilson CI 98/100", abs(lo - 0.9300) < 0.002 and abs(hi - 0.9945) < 0.002, f"{lo:.4f}-{hi:.4f}")


def test_import_ref(tmp):
    print("import_ref on a velocity-only export (';', decimal comma, ms)")
    t = np.arange(0, 3.0, 0.002)
    v = 0.8 * np.sin(np.pi * t / 1.5) ** 2            # m/s, closed-form position below
    x_true = 0.8 * (t / 2 - 1.5 / (4 * np.pi) * np.sin(2 * np.pi * t / 1.5))
    src = os.path.join(tmp, "export.csv")
    with open(src, "w") as f:
        f.write("Time [ms];Rep;Velocity [m/s]\n")
        for ti, vi in zip(t, v):
            f.write(f"{(ti + 12.345) * 1000:.1f};1;{vi:.5f}".replace(".", ",") + "\n")
    out = os.path.join(tmp, "ref.csv")
    run("import_ref.py", src, "--out", out, "--time-col", "Time [ms]", "--time-unit", "ms",
        "--vel-col", "2", "--delimiter", ";", "--decimal", ",", "--zero-time")
    df = pd.read_csv(out)
    check("columns are t_s,position_m", list(df.columns) == ["t_s", "position_m"])
    check("time offset reset to 0 and converted from ms",
          abs(df["t_s"].iloc[0]) < 1e-9 and abs(df["t_s"].iloc[-1] - t[-1]) < 1e-6, f"{df['t_s'].iloc[-1]:.4f} s")
    err = np.max(np.abs(df["position_m"].values - x_true))
    check("velocity integrated to position (trapezoids, < 0.5 mm)", err < 5e-4, f"max error {err * 1000:.3f} mm")
    pos = import_ref.convert(out, "t_s", pos_col="position_m", pos_unit="mm")
    check("position column with unit conversion (mm -> m)",
          np.allclose(pos["position_m"].values, df["position_m"].values / 1000.0))


def run(script, *args):
    r = subprocess.run([sys.executable, os.path.join(ROOT, script), *args], capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout, r.stderr)
        raise SystemExit(f"{script} failed")
    return r.stdout


def test_end_to_end(data):
    print(f"end to end on synthetic data in {data}")
    truth = synth.generate(data)
    run("align.py", "--data", data, "--all")
    run("pair_reps.py", "--data", data)
    run("analyze.py", "--data", data)

    al = pd.read_csv(os.path.join(data, "sets_alignment.csv"))
    check("reference sign inversion detected", (al["sign"] == -1).all())
    check(f"reference scale from the static verification (1/{synth.REF_SCALE_ERROR})",
          (np.abs(al["ref_scale"] * synth.REF_SCALE_ERROR - 1) < 0.002).all(), f"{al['ref_scale'].iloc[0]:.4f}")
    est = al[al["drift_estimated"].astype(bool)]
    check("clock drift recovered on the long sets (40 ppm +/- 25)",
          len(est) > 0 and (np.abs(est["drift_ppm"] - synth.DRIFT_PPM) < 25).all(),
          f"{len(est)} sets, {', '.join(f'{x:.0f}' for x in est['drift_ppm'])} ppm")
    check("no set flagged by the sync check", (~al["flagged"].astype(bool)).all(),
          f"max |sync| {np.nanmax(np.abs(al['sync_check_ms'])):.1f} ms")
    # alignment accuracy: aligned reference vs the noise-free signal ODKI saw
    errs = []
    for sid in al["set_id"].values[:6]:
        a = pd.read_csv(os.path.join(data, "aligned", f"{sid}.csv"))
        m = np.isfinite(a["v_ref"]) & np.isfinite(a["v_odki_corrected"])
        errs.append(np.sqrt(np.mean((a["v_ref"][m] - a["v_odki_corrected"][m]) ** 2)))
    check("aligned velocity RMS difference < 0.01 m/s", max(errs) < 0.01, f"max {max(errs):.4f}")

    reps = pd.read_csv(os.path.join(data, "reps_paired.csv"))
    n = reps["match"].value_counts()
    exp_fn, exp_fp = int(truth["missed"].sum()), int(truth["extra"].sum())
    exp_tp = int(truth["n_ref"].sum()) - exp_fn
    check(f"confusion matrix TP {exp_tp} FN {exp_fn} FP {exp_fp}",
          n.get("TP", 0) == exp_tp and n.get("FN", 0) == exp_fn and n.get("FP", 0) == exp_fp,
          f"got TP {n.get('TP', 0)} FN {n.get('FN', 0)} FP {n.get('FP', 0)}")
    tp = reps[reps["match"] == "TP"]
    check("reference MV matches the built-in ODKI-definition MV (pairing is right)",
          np.abs((tp["odki_mv_final"] - tp["ref_mv_odkidef"]) - synth.BIAS).max() < 6 * synth.SD)

    s = json.load(open(os.path.join(data, "results", "summary.json")))
    p = s["primary"]
    se = synth.SD / np.sqrt(p["n"])
    check(f"bias recovered ({synth.BIAS} m/s)", abs(p["bias"] - synth.BIAS) < 4 * se + 0.002,
          f"{p['bias']:.4f}, CI {p['ci95']['bias'][0]:.4f} to {p['ci95']['bias'][1]:.4f}")
    check(f"SD of differences recovered ({synth.SD} m/s)", abs(p["sd_diff_rm"] - synth.SD) < 0.006,
          f"{p['sd_diff_rm']:.4f}")
    check("LoA = bias +/- 1.96 SD",
          abs(p["loa_high_rm"] - (synth.BIAS + 1.96 * synth.SD)) < 0.015 and
          abs(p["loa_low_rm"] - (synth.BIAS - 1.96 * synth.SD)) < 0.015,
          f"{p['loa_low_rm']:.3f} to {p['loa_high_rm']:.3f}")
    truth = pd.read_csv(os.path.join(data, "_synthetic_reps.csv"))
    mm = tp.merge(truth, on=["set_id", "odki_rep_number"], suffixes=("", "_truth"))
    err = mm["ref_mv_odkidef"] - mm["mv_def_true"]
    # a 10 ms sample more or less at the 0.20 m/s opening moves a mean by ~1%:
    # inherent to the definition, so a few reps per hundred may differ by > 0.01
    check("reference MV per rep = built-in MV (|error| < 0.01 m/s on >= 97% of reps)",
          np.mean(np.abs(err) < 0.01) >= 0.97, f"{np.mean(np.abs(err) < 0.01) * 100:.1f}% of reps, median |error| {np.median(np.abs(err)):.4f}")
    built = (mm["odki_mv_final"] - mm["mv_def_true"]).mean()  # bias actually drawn, noise included
    check("bias equals the bias actually built into the reps (+/- 0.003)", abs(p["bias"] - built) < 0.003,
          f"built {built:.4f}, recovered {p['bias']:.4f}")
    check("no proportional bias built in", abs(p["prop_bias_slope"]) < 0.05, f"slope {p['prop_bias_slope']:.3f}")
    first = s["validity"]["MV_first"]
    check(f"first-reported SD recovered ({synth.SD_FIRST})", abs(first["sd_diff_rm"] - synth.SD_FIRST) < 0.01,
          f"{first['sd_diff_rm']:.4f}")
    pv = s["validity"]["PV_final"]
    check(f"PV bias recovered ({synth.PV_BIAS})", abs(pv["bias"] - synth.PV_BIAS) < 0.012, f"{pv['bias']:.4f}")
    bu = s["between_units"]
    check("between-unit SEM ~ SD (two independent noises)", bu["available"] and abs(bu["sem"] - synth.SD) < 0.008,
          f"{bu.get('sem', float('nan')):.4f}")
    check("test-retest computed", s["test_retest_odki"]["available"])
    det = s["detection"]
    check("recall / false-rep rate consistent", abs(det["recall"] - exp_tp / (exp_tp + exp_fn)) < 1e-5)
    check("velocity bands populated", all(len(v) >= 2 for v in s["by_velocity_band"].values()))
    check("RESULTS.md and plots written",
          os.path.exists(os.path.join(data, "results", "RESULTS.md")) and
          os.path.exists(os.path.join(data, "results", "mv_back_squat.png")))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--keep", default=None, help="write the synthetic dataset here and keep it")
    args = ap.parse_args()
    test_statistics()
    data = args.keep or tempfile.mkdtemp(prefix="odki_validation_selftest_")
    try:
        imp = os.path.join(data, "_import_ref")
        os.makedirs(imp, exist_ok=True)
        test_import_ref(imp)
        test_end_to_end(data)
    finally:
        if not args.keep:
            shutil.rmtree(data, ignore_errors=True)
    print(f"\n{'ALL CHECKS PASSED' if not FAILS else str(len(FAILS)) + ' CHECK(S) FAILED: ' + ', '.join(FAILS)}")
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    main()
