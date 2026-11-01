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
Feasibility study: x-io Fusion vs the Madgwick filter, through the full
tracking chain, on real logs.

For every log:
  1. reconstructs the orientation filter's per-sample input (reconstruct.py)
     and the state CALIBRATE left (accelScaleG, accBiasVec - fitted when the
     log does not record them);
  2. re-runs the Madgwick build on it and reports how closely it reproduces
     the logged attitude / world-Z acceleration / velocity (the reconstruction
     residual - the floor below which a Fusion-vs-Madgwick difference means
     nothing);
  3. runs the Fusion build (default settings, then a parameter sweep) and
     compares reps, per-rep values, traces and reference-free plausibility
     metrics with the Madgwick re-run;
  4. with --encoder LOG=CSV (t_s,position_m as written by
     ../validation/import_ref.py), also aligns a linear encoder recording to
     each variant (../validation/align.py, pair_reps.py) and reports bias, LoA
     and MAE per filter.

Usage:
    python feasibility.py LOG [LOG ...] --out DIR [--label LOG=NAME] [--encoder LOG=CSV]
                          [--no-sweep] [--selftest-encoder]
Requires numpy, pandas, matplotlib and clang/clang++ (or gcc/g++).
"""

import argparse
import itertools
import json
import os
import shutil
import subprocess
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import reconstruct as R  # noqa: E402

TOOLS = os.path.dirname(HERE)
FW = os.path.join(os.path.dirname(TOOLS), "VBT_Quaternions")
VALIDATION = os.path.join(TOOLS, "validation")

# Fusion defaults (FusionOrientationSettings in FusionOrientation.h)
FUSION_DEFAULT = {"gain": 0.5, "gyroRange": 500, "accRej": 10, "timeout": 5, "skipStartup": 1, "bias": 0}
SWEEP = {"gain": [0.1, 0.25, 0.5, 1.0, 2.0], "accRej": [0, 5, 10, 20], "timeout": [1, 5, 10]}
EXTRA_VARIANTS = {  # one-at-a-time departures from the default
    "fusion+bias": {"bias": 1},
    "fusion-noOverrange": {"gyroRange": 0},
    "fusion-startup": {"skipStartup": 0},
}

COL_M, COL_F, COL_D = "#2a78d6", "#eb6834", "#52514e"
SURFACE, INK, INK2 = "#fcfcfb", "#0b0b0b", "#52514e"


# ----------------------------------------------------------------------
# Build / run
# ----------------------------------------------------------------------

def build(bindir):
    os.makedirs(bindir, exist_ok=True)
    cc = shutil.which("clang") or shutil.which("gcc")
    cxx = shutil.which("clang++") or shutil.which("g++")
    if not cc or not cxx:
        sys.exit("No C/C++ compiler found (clang/clang++ or gcc/g++).")
    objs = []
    for f in ("FusionAhrs", "FusionBias", "FusionCompass", "FusionConvention", "FusionRemap"):
        o = os.path.join(bindir, f + ".o")
        subprocess.run([cc, "-std=c99", "-O2", "-c", os.path.join(FW, "src", "Fusion", f + ".c"), "-o", o], check=True)
        objs.append(o)
    inc = ["-I", os.path.join(HERE, "stubs"), "-I", os.path.join(TOOLS, "host_replay", "stubs"), "-I", FW]
    out = {}
    for name, flag, extra in (("madgwick", 0, []), ("fusion", 1, [os.path.join(FW, "FusionOrientation.cpp")] + objs)):
        b = os.path.join(bindir, "harness_" + name)
        # single-filter builds (ODKI_FILTER_RUNTIME_SELECT=0): the harness talks to `ahrs` directly
        subprocess.run([cxx, "-std=c++17", "-O2", "-w", *inc, f"-DODKI_ORIENTATION_FILTER={flag}",
                        "-DODKI_FILTER_RUNTIME_SELECT=0",
                        os.path.join(HERE, "harness.cpp"), os.path.join(FW, "MadgwickAHRS.cpp"), *extra, "-o", b],
                       check=True)
        out[name] = b
    return out


def run(binary, init, lines, settings=None, seed_legacy=False):
    args = [binary] + [f"{k}={v}" for k, v in (settings or {}).items()]
    if seed_legacy:
        args.append("seedMode=legacy")
    res = subprocess.run(args, input=init + "\n" + "\n".join(lines) + "\n", capture_output=True, text=True, check=True)
    out = res.stdout.split("\n")
    o = R.parse_session(out)
    f = [l.split(",")[1:] for l in out if l.startswith("F,")]
    fcols = ["t_ms", "q0", "q1", "q2", "q3", "accIgnored", "accErrDeg", "accRecovery", "startup", "overrange",
             "resBiasX", "resBiasY", "resBiasZ"]
    o["f"] = pd.DataFrame(f, columns=fcols).apply(pd.to_numeric)
    o["lines"] = out
    o["r_all"] = R.r_all_rows(out)
    return o


def quiet_parse():
    """vbt_log_viewer prints a note when it drops the header row - silence it."""
    import contextlib
    import io
    return contextlib.redirect_stdout(io.StringIO())


# ----------------------------------------------------------------------
# Reconstruction: fit the unlogged CALIBRATE state, measure the residual
# ----------------------------------------------------------------------

def tilt_residual(o, d):
    q = d["s"][["quatW", "quatX", "quatY", "quatZ"]].values.astype(float)
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    return R.angle_deg(R.grav_body(q), R.grav_body(o["f"][["q0", "q1", "q2", "q3"]].values))


def fit_state(d, bins):
    """accelScaleG (logged, from accRaw, or a grid search on the Madgwick
    attitude residual) and the initial accBiasVec (only its projection on
    gravity is logged: the other two components are fitted, the bias
    estimator being affine in its initial value, on the logged accZBias)."""
    s = d["s"]
    info = {}
    if s["accelScaleG"].notna().all():
        scale, info["scale_source"] = float(s["accelScaleG"].iloc[0]), "logged (S row)"
    elif d["logged_scale"]:
        scale, info["scale_source"] = d["logged_scale"], "logged (CALIBRATE line)"
    elif s["accRawX"].notna().all():
        scale, info["scale_source"] = R.scale_from_accraw(d), "derived from accRaw vs linAcc"
    else:
        best = None
        for grid in (np.arange(9.40, 10.21, 0.05), None):
            if grid is None:
                grid = np.arange(best[1] - 0.05, best[1] + 0.051, 0.01)
            for sc in grid:
                init, lines, _ = R.build_inputs(d, sc)
                with quiet_parse():
                    o = run(bins["madgwick"], init, lines)
                r = float(np.median(tilt_residual(o, d)))
                if best is None or r < best[0]:
                    best = (r, float(sc))
        scale, info["scale_source"] = best[1], "fitted (min. median attitude residual of the Madgwick re-run)"
    init, lines, rinfo = R.build_inputs(d, scale)
    info.update(rinfo)
    if s["accBiasX"].notna().all():
        info["accbias_source"] = "logged (S row)"
        return init, lines, info
    head, tail = init.rsplit(",", 3)[0], [float(x) for x in init.rsplit(",", 3)[1:]]
    b0 = np.array(tail)
    z = s["accZBias"].values

    def series(b):
        ini = head + ",%.6f,%.6f,%.6f" % tuple(b)
        with quiet_parse():
            return run(bins["madgwick"], ini, lines)["s"]["accZBias"].values

    base = series(b0)
    b, fitted = b0, base
    for _ in range(3):  # Gauss-Newton: affine except where the max-step gate switches
        cols = [series(b + 0.05 * e) - fitted for e in np.eye(3)]
        db, *_ = np.linalg.lstsq(np.column_stack(cols) / 0.05, z - fitted, rcond=None)
        nb = b + db
        nf = series(nb)
        if np.sqrt(np.mean((nf - z) ** 2)) >= np.sqrt(np.mean((fitted - z) ** 2)) - 1e-6:
            break
        b, fitted = nb, nf
    if np.sqrt(np.mean((fitted - z) ** 2)) < np.sqrt(np.mean((base - z) ** 2)):
        info["accbias_source"] = "fitted (affine least squares on the logged accZBias)"
    else:
        b = b0
        info["accbias_source"] = "projection only (fit did not improve)"
    info["accbias0"] = [float(x) for x in b]
    return head + ",%.6f,%.6f,%.6f" % tuple(b), lines, info


def reconstruction_residual(o, d):
    s, so = d["s"], o["s"]
    ang = tilt_residual(o, d)
    out = {"tilt_deg_median": float(np.median(ang)), "tilt_deg_p95": float(np.percentile(ang, 95)),
           "tilt_deg_max": float(ang.max())}
    for c in ("worldAccZ", "worldAccZRaw", "velZ"):
        if s[c].notna().all():
            dd = so[c].values - s[c].values
            out[c + "_rms"] = float(np.sqrt(np.mean(dd ** 2)))
            out[c + "_maxabs"] = float(np.abs(dd).max())
    out["reps_logged"] = int(len(d["r"]))
    out["reps_rerun"] = int(len(o["r"]))
    if len(d["r"]) and len(o["r"]):
        m = d["r"].merge(o["r"], on="rep", suffixes=("_log", "_rerun"))
        out["mv_absdiff_vs_logged_mean"] = float(np.mean(np.abs(m["meanVelocity_log"] - m["meanVelocity_rerun"])))
        out["mv_absdiff_vs_logged_max"] = float(np.max(np.abs(m["meanVelocity_log"] - m["meanVelocity_rerun"])))
    return out, ang


# ----------------------------------------------------------------------
# Metrics
# ----------------------------------------------------------------------

def rep_table(o):
    C = R.validation_common()
    s, r = o["s"], o["r"]
    if not len(r):
        return pd.DataFrame(columns=["rep", "t0", "t1", "mv", "pv", "disp", "status", "duration", "mv_first"])
    iv = C.odki_rep_intervals(s, list(r["rep"].values), o["p"]) if C else {}
    first = o["r_all"].groupby("rep").first() if len(o["r_all"]) else None
    p = o["p"]
    rows = []
    for _, x in r.iterrows():
        k = int(x["rep"])
        t0, t1 = (iv[k][0], iv[k][1]) if k in iv else (np.nan, np.nan)
        pc = p[(p["phaseType"] > 0) & (p["reported"] == 1) & (p["repNumber"] == k)] if len(p) else p
        rows.append({"rep": k, "t0": t0, "t1": t1, "mv": x["meanVelocity"], "pv": x["peakVelocity"],
                     "disp": x["displacementM"], "status": int(x["correctionStatus"]),
                     "duration": float(pc["durationS"].iloc[-1]) if len(pc) else np.nan,
                     "mv_first": float(first.loc[k, "meanVelocity"]) if first is not None and k in first.index else np.nan})
    return pd.DataFrame(rows)


def match_reps(ra, rb, min_iou=0.5):
    """Greedy one-to-one match of concentric intervals by IoU (same rule as
    ../validation/pair_reps.py)."""
    cand = []
    for i, a in ra.iterrows():
        for j, b in rb.iterrows():
            inter = max(0.0, min(a.t1, b.t1) - max(a.t0, b.t0))
            union = max(a.t1, b.t1) - min(a.t0, b.t0)
            v = inter / union if union > 0 else 0
            if v >= min_iou:
                cand.append((v, i, j))
    used_a, used_b, pairs = set(), set(), []
    for v, i, j in sorted(cand, reverse=True):
        if i in used_a or j in used_b:
            continue
        used_a.add(i); used_b.add(j); pairs.append((i, j))
    return pairs, [i for i in ra.index if i not in used_a], [j for j in rb.index if j not in used_b]


def plausibility(o, reps):
    """Reference-free plausibility metrics (lower is better for all)."""
    s, b, f = o["s"], o["b"], o["f"]
    t = s["sessionT"].values
    out = {}
    if len(b):
        out["bracket_absdrift_median"] = float(np.median(np.abs(b["drift"])))
        out["bracket_absdrift_max"] = float(np.max(np.abs(b["drift"])))
        long_ = b[(b["t1"] - b["t0"]) > 0.5]
        out["bracket_absrate_median"] = float(np.median(np.abs(long_["drift"] / (long_["t1"] - long_["t0"])))) if len(long_) else np.nan
        # net displacement over each bracket that contains a rep (still to still:
        # the bar should be back at the same height)
        net = []
        for _, br in b.iterrows():
            if br["t1"] - br["t0"] < 0.8:
                continue
            if not len(reps) or not ((reps["t0"] >= br["t0"]) & (reps["t1"] <= br["t1"])).any():
                continue
            m = (t >= br["t0"]) & (t <= br["t1"])
            v = s["corrected"].values[m]
            if np.isfinite(v).all() and m.sum() > 2:
                net.append(abs(float(np.trapezoid(v, t[m]))))
        out["bracket_net_disp_median_m"] = float(np.median(net)) if net else np.nan
        out["bracket_net_disp_n"] = len(net)
    still = s["confirmedStillNow"].values == 1
    out["still_absacc_mean"] = float(np.mean(np.abs(s["worldAccZ"].values[still]))) if still.any() else np.nan
    edge = np.where(np.diff(still.astype(int)) == 1)[0]  # sample before stillness is confirmed
    out["velLive_at_still_onset_median"] = float(np.median(np.abs(s["velZLive"].values[edge]))) if len(edge) else np.nan
    if len(reps) >= 3:
        rr = reps[reps["status"] == 2] if (reps["status"] == 2).sum() >= 3 else reps
        out["disp_cv_pct"] = float(np.std(rr["disp"], ddof=1) / np.mean(rr["disp"]) * 100)
        out["mv_cv_pct"] = float(np.std(rr["mv"], ddof=1) / np.mean(rr["mv"]) * 100)
    out["acc_ignored_pct"] = float(f["accIgnored"].mean() * 100)
    out["acc_recovery_pct"] = float(f["accRecovery"].mean() * 100)
    out["overrange_samples"] = int(f["overrange"].sum())
    return out


def impact_events(d):
    """Impact-like events on the LOGGED signal: jerk of the pre-median world-Z
    acceleration above 600 m/s^3 AND a raw gyroscope burst above 120 deg/s
    (the separation found in DOCUMENTATION.md, Mechanical shocks)."""
    s = d["s"]
    if not s["worldAccZRaw"].notna().all() or not s["gyroRawX"].notna().all():
        return []
    a = s["worldAccZRaw"].values
    dt = np.maximum(s["dt_ms"].values / 1000, 1e-3)
    jerk = np.r_[0, np.abs(np.diff(a)) / dt[1:]]
    gr = np.linalg.norm(s[["gyroRawX", "gyroRawY", "gyroRawZ"]].values - s[["gyroBiasX", "gyroBiasY", "gyroBiasZ"]].values, axis=1)
    t = s["sessionT"].values
    ev = []
    for i in np.where(jerk > 600)[0]:
        lo, hi = max(i - 5, 0), min(i + 6, len(t))
        if gr[lo:hi].max() > 120 and (not ev or t[i] - ev[-1] > 0.4):
            ev.append(float(t[i]))
    return ev


def compare(om, of, d, ev):
    rm, rf = rep_table(om), rep_table(of)
    pairs, miss, extra = match_reps(rm, rf)
    pm = [rm.loc[i] for i, j in pairs]
    pf = [rf.loc[j] for i, j in pairs]
    out = {"reps_madgwick": int(len(rm)), "reps_fusion": int(len(rf)), "matched": len(pairs),
           "only_madgwick": len(miss), "only_fusion": len(extra)}
    for k in ("mv", "pv", "disp", "duration", "mv_first"):
        dd = np.array([b[k] - a[k] for a, b in zip(pm, pf)], float)
        dd = dd[np.isfinite(dd)]
        if len(dd):
            out[f"{k}_bias"] = float(dd.mean())
            out[f"{k}_sd"] = float(dd.std(ddof=1)) if len(dd) > 1 else 0.0
            out[f"{k}_maxabs"] = float(np.abs(dd).max())
            out[f"{k}_mae"] = float(np.abs(dd).mean())
    out["status_changed"] = int(sum(a["status"] != b["status"] for a, b in zip(pm, pf)))
    qm = om["f"][["q0", "q1", "q2", "q3"]].values
    qf = of["f"][["q0", "q1", "q2", "q3"]].values
    tilt = R.angle_deg(R.grav_body(qm), R.grav_body(qf))
    out["tilt_diff_deg_median"] = float(np.median(tilt))
    out["tilt_diff_deg_p95"] = float(np.percentile(tilt, 95))
    out["tilt_diff_deg_max"] = float(tilt.max())
    for c in ("worldAccZ", "worldAccZRaw", "corrected"):
        a, b = om["s"][c].values, of["s"][c].values
        m = np.isfinite(a) & np.isfinite(b)
        out[f"{c}_diff_rms"] = float(np.sqrt(np.mean((a[m] - b[m]) ** 2)))
        out[f"{c}_diff_maxabs"] = float(np.abs(a[m] - b[m]).max())
    t = om["s"]["sessionT"].values
    if ev:
        near = np.zeros(len(t), bool)
        for e in ev:
            near |= (t > e - 0.1) & (t < e + 0.4)
        out["impacts"] = len(ev)
        out["impact_tilt_diff_deg_max"] = float(tilt[near].max())
        a, b = om["s"]["worldAccZRaw"].values, of["s"]["worldAccZRaw"].values
        out["impact_accz_diff_maxabs"] = float(np.abs(a - b)[near].max())
        # velocity step across each event (raw integral, 0.1 s before -> 0.4 s after)
        steps = []
        for e in ev:
            i0, i1 = np.searchsorted(t, e - 0.1), min(np.searchsorted(t, e + 0.4), len(t) - 1)
            steps.append(float((of["s"]["velZ"].values[i1] - of["s"]["velZ"].values[i0]) -
                               (om["s"]["velZ"].values[i1] - om["s"]["velZ"].values[i0])))
        out["impact_velstep_diff_maxabs"] = float(np.max(np.abs(steps)))
    else:
        out["impacts"] = 0
    return out, rm, rf, pairs, tilt


# ----------------------------------------------------------------------
# Encoder (optional)
# ----------------------------------------------------------------------

def encoder_agreement(variant_logs, ref_csv, outdir, tag):
    """Aligns the encoder to each variant's replayed log and pairs the reps,
    with ../validation/align.py and pair_reps.py, then bias / LoA / MAE per
    filter on MV (firmware definition), PV and ROM."""
    if VALIDATION not in sys.path:
        sys.path.insert(0, VALIDATION)
    import align as AL
    import pair_reps as PR
    res = {}
    for name, path in variant_logs.items():
        aligned, info = AL.align_set(path, ref_csv)
        apath = os.path.join(outdir, f"{tag}_{name}_aligned.csv")
        aligned.to_csv(apath, index=False)
        rows = pd.DataFrame(PR.pair_set(path, apath, {"set_id": f"{tag}_{name}", "physical_set_id": tag,
                                                      "unit_id": name}, info))
        tp = rows[rows["match"] == "TP"]
        r = {"align": {k: info[k] for k in ("lag_ms", "xcorr_r", "sync_check_ms", "flagged")},
             "TP": int(len(tp)), "FN": int((rows["match"] == "FN").sum()), "FP": int((rows["match"] == "FP").sum())}
        for m, (o, ref) in {"MV": ("odki_mv_final", "ref_mv_odkidef"), "PV": ("odki_pv_final", "ref_pv"),
                            "ROM": ("odki_rom_final", "ref_rom_m")}.items():
            dd = (tp[o] - tp[ref]).dropna().values
            if len(dd) >= 2:
                sd = float(np.std(dd, ddof=1))
                r[m] = {"n": int(len(dd)), "bias": float(dd.mean()), "loa_low": float(dd.mean() - 1.96 * sd),
                        "loa_high": float(dd.mean() + 1.96 * sd), "mae": float(np.mean(np.abs(dd)))}
        rows.to_csv(os.path.join(outdir, f"{tag}_{name}_reps_vs_encoder.csv"), index=False)
        res[name] = r
    return res


def selftest_encoder_csv(om, path, lag_s=2.0, hz=25.0):
    """Plumbing self-test ONLY: a fake 'encoder' made from the Madgwick
    re-run's own corrected velocity (integrated, resampled at 25 Hz, clock
    shifted). Says nothing about accuracy."""
    s = om["s"]
    t = R.validation_common().odki_device_time(s)
    v = np.nan_to_num(s["corrected"].values)
    x = np.r_[0, np.cumsum(0.5 * (v[1:] + v[:-1]) * np.diff(t))]
    te = np.arange(t[0], t[-1], 1 / hz)
    pd.DataFrame({"t_s": te - t[0] + lag_s, "position_m": np.interp(te, t, x)}).to_csv(path, index=False)


# ----------------------------------------------------------------------
# Charts (deterministic PNGs)
# ----------------------------------------------------------------------

def _plt():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
                         "axes.edgecolor": "#b8b7b0", "axes.labelcolor": INK2, "xtick.color": INK2,
                         "ytick.color": INK2, "text.color": INK, "axes.grid": True, "grid.color": "#e6e5e0",
                         "grid.linewidth": 0.6, "axes.spines.top": False, "axes.spines.right": False,
                         "font.size": 9, "legend.frameon": False, "lines.linewidth": 1.4,
                         "svg.hashsalt": "odki", "path.simplify": False})
    return plt


def _save(fig, path):
    fig.savefig(path, dpi=130, metadata={"Software": None})


def chart_traces(om, of, rm, tilt, ev, tag, path):
    plt = _plt()
    t = om["s"]["sessionT"].values
    fig, axs = plt.subplots(4, 1, figsize=(11, 8.6), sharex=True,
                            gridspec_kw={"height_ratios": [2.2, 1.2, 1.2, 1.2]})
    vm, vf = om["s"]["corrected"].values, of["s"]["corrected"].values
    axs[0].plot(t, vm, color=COL_M, label="Madgwick")
    axs[0].plot(t, vf, color=COL_F, lw=1.1, label="Fusion")
    axs[0].set_ylabel("velocity, corrected (m/s)")
    axs[0].legend(loc="upper right", ncol=2)
    axs[1].plot(t, vf - vm, color=COL_D, lw=1.0)
    axs[1].set_ylabel("Δ velocity\nFusion − Madgwick (m/s)")
    am, af = om["s"]["worldAccZRaw"].values, of["s"]["worldAccZRaw"].values
    axs[2].plot(t, af - am, color=COL_D, lw=1.0)
    axs[2].set_ylabel("Δ vertical acc.\n(m/s²)")
    axs[3].plot(t, tilt, color=COL_D, lw=1.0)
    ign = of["f"]["accIgnored"].values.astype(bool)
    if ign.any():
        axs[3].fill_between(t, 0, max(tilt.max(), 0.1), where=ign, color=COL_F, alpha=0.15, lw=0,
                            label="Fusion ignoring the accelerometer")
        axs[3].legend(loc="upper right")
    axs[3].set_ylabel("tilt difference\n(deg)")
    axs[3].set_xlabel("time in set (s)")
    for ax in axs:
        for _, r in rm.iterrows():
            if np.isfinite(r.t0):
                ax.axvspan(r.t0, r.t1, color="#dfe9f7", lw=0, zorder=0)
        for e in ev:
            ax.axvline(e, color="#e34948", lw=0.8, ls=":")
        ax.axhline(0, color="#9a9990", lw=0.6)
    fig.suptitle(f"{tag}: Fusion (default settings) vs Madgwick, same input - shaded: Madgwick's concentric phases"
                 + (", dotted: impacts" if ev else ""), fontsize=10, color=INK)
    fig.tight_layout()
    _save(fig, path)
    plt.close(fig)


def chart_reconstruction(o, d, ang, tag, path):
    plt = _plt()
    s, so = d["s"], o["s"]
    t = s["sessionT"].values
    fig, axs = plt.subplots(3, 1, figsize=(11, 6.4), sharex=True)
    axs[0].plot(t, ang, color=COL_D, lw=1.0)
    axs[0].set_ylabel("attitude residual\n(deg)")
    axs[1].plot(t, so["worldAccZ"].values - s["worldAccZ"].values, color=COL_D, lw=1.0)
    axs[1].set_ylabel("world-Z acc. residual\n(m/s²)")
    axs[2].plot(t, s["velZ"].values, color="#b8b7b0", lw=2.2, label="logged")
    axs[2].plot(t, so["velZ"].values, color=COL_M, lw=1.0, label="Madgwick re-run")
    axs[2].set_ylabel("velZ, raw integral (m/s)")
    axs[2].legend(loc="upper right", ncol=2)
    axs[2].set_xlabel("time in set (s)")
    fig.suptitle(f"{tag}: Madgwick re-run on the reconstructed input vs what the firmware logged", fontsize=10)
    fig.tight_layout()
    _save(fig, path)
    plt.close(fig)


def chart_reps(pooled, path, ctrl=None):
    plt = _plt()
    fig, axs = plt.subplots(2, 3, figsize=(11, 6.8))
    for k, (col, name, unit) in enumerate((("mv", "mean velocity", "m/s"), ("pv", "peak velocity", "m/s"),
                                           ("disp", "displacement", "m"))):
        a, b = pooled[f"{col}_m"].values, pooled[f"{col}_f"].values
        lo, hi = min(a.min(), b.min()), max(a.max(), b.max())
        pad = 0.05 * (hi - lo)
        ax = axs[0, k]
        ax.plot([lo - pad, hi + pad], [lo - pad, hi + pad], color="#9a9990", lw=0.8)
        ax.scatter(a, b, s=22, color=COL_F, edgecolor=SURFACE, linewidth=1.0, zorder=3)
        ax.set_xlabel(f"Madgwick {name} ({unit})")
        ax.set_ylabel(f"Fusion {name} ({unit})")
        ax.set_aspect("equal", adjustable="datalim")
        ax = axs[1, k]
        mean, diff = (a + b) / 2, b - a
        bias, sd = diff.mean(), diff.std(ddof=1)
        if ctrl is not None and len(ctrl):
            ca, cb = ctrl[f"{col}_m"].values, ctrl[f"{col}_c"].values
            ax.scatter((ca + cb) / 2, cb - ca, s=22, facecolor="none", edgecolor=COL_D, linewidth=1.0, zorder=2,
                       label="control: Madgwick, accelScaleG +0.01")
        ax.scatter(mean, diff, s=22, color=COL_F, edgecolor=SURFACE, linewidth=1.0, zorder=3, label="Fusion")
        if k == 0:
            ax.legend(loc="lower left", fontsize=7)
        for y, ls in ((bias, "-"), (bias + 1.96 * sd, "--"), (bias - 1.96 * sd, "--")):
            ax.axhline(y, color=COL_D, lw=0.9, ls=ls)
        ax.axhline(0, color="#9a9990", lw=0.6)
        ax.text(0.02, 0.95, f"bias {bias:+.3f}  LoA [{bias - 1.96 * sd:+.3f}, {bias + 1.96 * sd:+.3f}] {unit}",
                transform=ax.transAxes, va="top", fontsize=8, color=INK2)
        ax.set_xlabel(f"mean of the two ({unit})")
        ax.set_ylabel(f"Fusion − Madgwick ({unit})")
    fig.suptitle(f"Per-rep values, Fusion (default) vs Madgwick - {len(pooled)} matched reps, all logs", fontsize=10)
    fig.tight_layout()
    _save(fig, path)
    plt.close(fig)


def chart_plausibility(rows, path):
    plt = _plt()
    metrics = [("bracket_absdrift_median", "drift removed at a\nbracket close, median (m/s)"),
               ("bracket_net_disp_median_m", "net displacement still→still,\nmedian |m| (should be 0)"),
               ("disp_cv_pct", "rep displacement CV\nwithin the set (%)"),
               ("still_absacc_mean", "|vertical acc.| while\nconfirmed still (m/s²)")]
    tags = [r["tag"] for r in rows]
    x = np.arange(len(tags))
    fig, axs = plt.subplots(1, 4, figsize=(12, 3.6))
    for ax, (k, label) in zip(axs, metrics):
        vm = [r["plaus_madgwick"].get(k, np.nan) for r in rows]
        vf = [r["plaus_fusion"].get(k, np.nan) for r in rows]
        ax.bar(x - 0.2, vm, 0.38, color=COL_M, label="Madgwick")
        ax.bar(x + 0.2, vf, 0.38, color=COL_F, label="Fusion")
        ax.set_xticks(x, tags, rotation=0)
        ax.set_title(label, fontsize=9, color=INK)
        ax.grid(axis="x", visible=False)
    axs[0].legend(loc="upper left")
    fig.suptitle("Reference-free plausibility (lower is better) - per log", fontsize=10)
    fig.tight_layout()
    _save(fig, path)
    plt.close(fig)


def chart_sweep(sweep_df, path):
    plt = _plt()
    fig, axs = plt.subplots(1, 3, figsize=(12, 3.8))
    cols = ["#2a78d6", "#eb6834", "#1baf7a", "#6250d6"]
    d5 = sweep_df[sweep_df["timeout"] == 5]
    for ax, (k, label) in zip(axs, (("mv_mae", "mean |Δ mean velocity| vs Madgwick,\nmedian of logs (m/s)"),
                                    ("rep_count_diff", "reps detected by only one filter,\nsum over logs (count)"),
                                    ("disp_cv_delta", "rep displacement CV, Fusion − Madgwick,\nmedian of logs (points; < 0 = more consistent)"))):
        for c, rej in zip(cols, sorted(d5["accRej"].unique())):
            g = d5[d5["accRej"] == rej].groupby("gain")[k].mean()
            ax.plot(g.index, g.values, color=c, marker="o", ms=4, label=f"rejection {rej:g}°" if rej else "rejection off")
        ax.set_xscale("log")
        ax.set_xticks(SWEEP["gain"], [f"{g:g}" for g in SWEEP["gain"]])
        ax.set_xlabel("Fusion gain")
        ax.set_title(label, fontsize=9)
    axs[2].axhline(0, color=COL_D, ls="--", lw=0.9)
    axs[0].legend(loc="upper right", fontsize=8)
    fig.suptitle("Fusion parameter sweep (rejection timeout 5 s) - every point is a full re-run of all logs", fontsize=10)
    fig.tight_layout()
    _save(fig, path)
    plt.close(fig)


CONV_STYLES = {"Madgwick": (COL_M, "-"), "Fusion (startup skipped)": (COL_F, "-"),
               "Fusion (startup ramp)": (COL_F, "--")}
CONV_TILTS = (5, 15, 30, 45, 60, 75, 90, 120, 150, 175)


def chart_convergence_vs_tilt(table, path):
    """table[seed][tilt][variant] = seconds to < 1 deg (None: not within the run)
    and table["max_err_fixed"][tilt][variant] = worst error after the fixed seed."""
    plt = _plt()
    fig, (ax, bx) = plt.subplots(1, 2, figsize=(10, 3.9))
    tilts = sorted(table["legacy"])
    for name, (c, ls) in CONV_STYLES.items():
        ys = [table["legacy"][t][name] if table["legacy"][t][name] is not None else np.nan for t in tilts]
        ax.plot(tilts, ys, color=c, ls=ls, marker="o", ms=4, label=name)
        for t, y in zip(tilts, ys):
            if not np.isfinite(y):
                ax.plot([t], [19.5], marker="x", color=c, ms=7, mew=2)
        ym = [table["max_err_fixed"][t][name] for t in tilts]
        bx.plot(tilts, ym, color=c, ls=ls, marker="o", ms=4, label=name)
    ax.text(0.98, 0.97, "x = not below 1° within the run", transform=ax.transAxes, fontsize=7, color=INK2,
            ha="right", va="top")
    ax.set_xlabel("sensor tilt from vertical at CALIBRATE (deg)")
    ax.set_ylabel("seconds until attitude error < 1°")
    ax.set_title("Before the fix (mirrored seed, ≤ v3.11.32)", fontsize=9)
    ax.legend(loc="upper left", fontsize=7)
    bx.set_xlabel("sensor tilt from vertical at CALIBRATE (deg)")
    bx.set_ylabel("max attitude error after CALIBRATE (deg)")
    bx.set_title("After the fix (v3.11.33): worst error over the whole run", fontsize=9)
    bx.axhline(1, color="#9a9990", lw=0.6)
    top = max(1.2, np.nanmax([table["max_err_fixed"][t][n] for t in tilts for n in CONV_STYLES]) * 1.2)
    bx.set_ylim(0, top)
    fig.suptitle("After CALIBRATE, sensor still (synthetic, real code)", fontsize=10)
    fig.tight_layout()
    _save(fig, path)
    plt.close(fig)


def chart_convergence(conv, path):
    plt = _plt()
    fig, ax = plt.subplots(figsize=(8, 3.6))
    for (name, seed), (t, err) in conv.items():
        c, ls = CONV_STYLES[name]
        ax.plot(t, np.maximum(err, 1e-4), color=c, ls=ls, alpha=1.0 if seed == "legacy" else 0.55,
                lw=1.6 if seed == "legacy" else 1.0, label=f"{name}, {seed} seed")
    ax.axhline(1, color="#9a9990", lw=0.6)
    ax.set_yscale("log")
    ax.set_xlabel("time after CALIBRATE (s), sensor still")
    ax.set_ylabel("attitude error (deg, floor 1e-4)")
    ax.legend(loc="upper right", fontsize=7)
    ax.set_title("Convergence after CALIBRATE at the first log's tilt: legacy (mirrored) vs fixed seed", fontsize=9)
    fig.tight_layout()
    _save(fig, path)
    plt.close(fig)


# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------

def convergence_test(bins, grav, duration_s=30.0):
    """Synthetic: sensor still at the given gravity direction, noise-free,
    seeded the way CALIBRATE does (alignToGravity of the mean accelerometer)
    - with the pre-v3.11.33 mirrored seed ("legacy") and with the fixed one
    ("fixed"). Attitude error vs time, keyed by (variant, seed)."""
    g = np.asarray(grav, float) / np.linalg.norm(grav)
    init = "I,%.6f,%.7f,%.7f,%.7f,0,0,0,0,0,0" % (R.G, *g)
    n = int(duration_s * 100)
    lines = ["%d,%.5f,%.5f,%.5f,0,0,0,0,0,0,0,0,0,0,0" % (1000 + 10 * i, *(g * R.G)) for i in range(n)]
    out = {}
    for name, b, st in (("Madgwick", "madgwick", {}), ("Fusion (startup skipped)", "fusion", FUSION_DEFAULT),
                        ("Fusion (startup ramp)", "fusion", {**FUSION_DEFAULT, "skipStartup": 0})):
        for seed in ("legacy", "fixed"):
            with quiet_parse():
                o = run(bins[b], init, lines, st, seed_legacy=(seed == "legacy"))
            q = o["f"][["q0", "q1", "q2", "q3"]].values
            err = R.angle_deg(R.grav_body(q), np.tile(g, (len(q), 1)))
            out[(name, seed)] = (np.arange(len(err)) / 100.0, err)
    return out


def tilt_gravity(deg):
    a = np.radians(deg)
    return [0.3 * np.sin(a), 0.954 * np.sin(a), np.cos(a)]


def start_attitude_vs_acc(d, window_s=0.5):
    """Median angle (deg) between the logged gravBody and the median-filtered
    accelerometer (linAcc + gravBody * accelScaleG) over the first window_s -
    near 0 when the filter was settled at REC_START and the bar still."""
    s = d["s"]
    m = (s["sessionT"] <= window_s).values
    q = s.loc[m, ["quatW", "quatX", "quatY", "quatZ"]].values.astype(float)
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    g = R.grav_body(q)
    acc = s.loc[m, ["linAccX", "linAccY", "linAccZ"]].values.astype(float) + g * R.G
    return float(np.median(R.angle_deg(g, acc)))


def write_variant_log(o, path):
    with open(path, "w") as f:
        f.write("REC_START\n" + "\n".join(l for l in o["lines"] if l[:2] in ("S,", "R,", "B,", "P,")) + "\nREC_STOP\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--out", required=True, help="output folder (keep it outside git: logs are private data)")
    ap.add_argument("--label", action="append", default=[], help="LOG=NAME, the name used in charts and tables")
    ap.add_argument("--encoder", action="append", default=[], help="LOG=CSV, encoder export for that log (t_s,position_m)")
    ap.add_argument("--no-sweep", action="store_true")
    ap.add_argument("--all-sessions", action=argparse.BooleanOptionalAction, default=True,
                    help="analyse every REC_START..REC_STOP block with at least --min-samples samples (default), "
                         "else only the longest")
    ap.add_argument("--min-samples", type=int, default=1000)
    ap.add_argument("--selftest-encoder", action="store_true",
                    help="exercise the encoder path with a fake encoder made from the Madgwick re-run (plumbing test only)")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    labels = dict(x.split("=", 1) for x in args.label)
    encoders = dict(x.split("=", 1) for x in args.encoder)
    bins = build(os.path.join(args.out, "bin"))

    summary, pooled, sweep_rows, control_pooled = [], [], [], []
    first_grav = None
    jobs = []
    for path in args.logs:
        base_tag = labels.get(path, os.path.splitext(os.path.basename(path))[0])
        with quiet_parse():
            n_sess = len(R.viewer().split_sessions(path))
            sizes = [sum(1 for x in l if x.startswith("S,")) for l in R.viewer().split_sessions(path)]
        keep = [i for i, n in enumerate(sizes) if n >= args.min_samples] if args.all_sessions else [int(np.argmax(sizes))]
        for i in keep:
            jobs.append((path, i, base_tag if len(keep) == 1 else f"{base_tag}{'abcdefghij'[keep.index(i)]}"))
    for path, sess, tag in jobs:
        print(f"\n=== {tag} ===")
        with quiet_parse():
            d = R.load_log(path, sess)
        print(f"  layout {d['layout']}, session {d['session_index'] + 1}/{d['n_sessions']}, {len(d['s'])} samples, "
              f"{len(d['r'])} reps logged")
        init, lines, info = fit_state(d, bins)
        if first_grav is None:
            first_grav = [float(x) for x in init.split(",")[2:5]]
        with quiet_parse():
            om = run(bins["madgwick"], init, lines)
            of = run(bins["fusion"], init, lines, FUSION_DEFAULT)
        resid, ang = reconstruction_residual(om, d)
        # the logged R rows come from the engine version that recorded the log: re-run also
        # with the pre-3.11.30 RuntimeConfig defaults to tell engine changes from input errors
        with quiet_parse():
            ol = run(bins["madgwick"], init, lines, {"config": "pre3.11.30"})
        rl, _ = reconstruction_residual(ol, d)
        resid["legacy_config"] = {k: rl[k] for k in rl if k.startswith(("reps_", "mv_"))}
        print(f"  input: acc = {info['acc_source']}; gyro = {info['gyro_source']}")
        print(f"  accelScaleG {info['scale']:.3f} ({info['scale_source']}); accBiasVec: {info['accbias_source']}")
        print("  reconstruction residual (Madgwick re-run vs log): attitude median {tilt_deg_median:.3f} / p95 "
              "{tilt_deg_p95:.3f} / max {tilt_deg_max:.3f} deg, worldAccZ rms {worldAccZ_rms:.4f} m/s², "
              "velZ max {velZ_maxabs:.3f} m/s, reps {reps_rerun} (logged {reps_logged})".format(**resid))
        lc = resid["legacy_config"]
        print(f"  per-rep MV vs logged R rows: current config max |Δ| {resid.get('mv_absdiff_vs_logged_max', np.nan):.4f}, "
              f"pre-3.11.30 config max |Δ| {lc.get('mv_absdiff_vs_logged_max', np.nan):.4f} m/s (reps {lc['reps_rerun']})")
        ev = impact_events(d)
        cmp_, rm, rf, pairs, tilt = compare(om, of, d, ev)
        # CALIBRATE seed (v3.11.33 fix). The re-runs above seed the attitude the
        # log starts with through the FIXED alignToGravity (it reproduces it). The
        # same seed through the pre-fix (mirrored) alignToGravity is what the
        # device would have done had CALIBRATE run right before START; the
        # difference is what the fix changes. Also: was the recorded attitude at
        # REC_START already settled (angle between the logged gravBody and the
        # median-filtered accelerometer over the first 0.5 s)?
        with quiet_parse():
            oleg = run(bins["madgwick"], init, lines, seed_legacy=True)
            olegf = run(bins["fusion"], init, lines, FUSION_DEFAULT, seed_legacy=True)
        seed_row = {"start_attitude_vs_acc_deg": start_attitude_vs_acc(d)}
        for fname, base, leg in (("madgwick", om, oleg), ("fusion", of, olegf)):
            cs, rb, rl, pairs_s, tilt_s = compare(base, leg, d, ev)
            t_s = base["s"]["sessionT"].values
            settle = float(t_s[np.argmax(tilt_s < 1.0)]) if (tilt_s < 1.0).any() else None
            reps_s = [{"rep": int(rb.loc[i, "rep"]), "t0": float(rb.loc[i, "t0"]),
                       "mv_fixed": float(rb.loc[i, "mv"]), "mv_legacy": float(rl.loc[j, "mv"]),
                       "dmv": float(rl.loc[j, "mv"] - rb.loc[i, "mv"])} for i, j in pairs_s]
            seed_row[fname] = {"reps_fixed": cs["reps_madgwick"], "reps_legacy": cs["reps_fusion"],
                               "matched": cs["matched"], "attitude_diff_deg_t0": float(tilt_s[0]),
                               "attitude_diff_below_1deg_after_s": settle, "reps": reps_s}
            big = [r for r in reps_s if abs(r["dmv"]) > 0.005]
            print(f"  seed fix ({fname}): legacy-seed attitude off by {tilt_s[0]:.1f}° at START, < 1° after "
                  f"{settle if settle is None else round(settle, 1)} s; reps fixed {cs['reps_madgwick']} / legacy "
                  f"{cs['reps_fusion']}, matched {cs['matched']}; |ΔMV| > 0.005 m/s in {len(big)} reps"
                  + (": " + ", ".join(f"rep {r['rep']} (t0 {r['t0']:.1f} s, {r['dmv']:+.3f})" for r in big) if big else ""))
        print(f"  recorded attitude at REC_START vs accelerometer (first 0.5 s): "
              f"{seed_row['start_attitude_vs_acc_deg']:.2f}° (≈ settled if small)")
        pm, pf = plausibility(om, rm), plausibility(of, rf)
        print(f"  reps: Madgwick {cmp_['reps_madgwick']}, Fusion {cmp_['reps_fusion']}, matched {cmp_['matched']}, "
              f"only-M {cmp_['only_madgwick']}, only-F {cmp_['only_fusion']}")
        if "mv_bias" in cmp_:
            print(f"  Fusion − Madgwick per rep: MV {cmp_['mv_bias']:+.4f} ± {cmp_['mv_sd']:.4f} (max {cmp_['mv_maxabs']:.3f}) m/s, "
                  f"PV {cmp_['pv_bias']:+.4f} ± {cmp_['pv_sd']:.4f}, disp {cmp_['disp_bias']:+.4f} ± {cmp_['disp_sd']:.4f} m")
        print(f"  tilt difference median {cmp_['tilt_diff_deg_median']:.2f}, p95 {cmp_['tilt_diff_deg_p95']:.2f}, "
              f"max {cmp_['tilt_diff_deg_max']:.2f} deg; Fusion ignored the accelerometer {pf['acc_ignored_pct']:.1f}% of samples")
        for i, j in pairs:
            pooled.append({"tag": tag, "rep_m": int(rm.loc[i, "rep"]), "rep_f": int(rf.loc[j, "rep"]),
                           **{f"{k}_m": rm.loc[i, k] for k in ("mv", "pv", "disp", "duration", "status")},
                           **{f"{k}_f": rf.loc[j, k] for k in ("mv", "pv", "disp", "duration", "status")}})
        rm.assign(filter="madgwick").to_csv(os.path.join(args.out, f"reps_{tag}_madgwick.csv"), index=False)
        rf.assign(filter="fusion").to_csv(os.path.join(args.out, f"reps_{tag}_fusion.csv"), index=False)
        pd.DataFrame({"sessionT": om["s"]["sessionT"], "vel_madgwick": om["s"]["corrected"],
                      "vel_fusion": of["s"]["corrected"], "velZ_madgwick": om["s"]["velZ"], "velZ_fusion": of["s"]["velZ"],
                      "accZRaw_madgwick": om["s"]["worldAccZRaw"], "accZRaw_fusion": of["s"]["worldAccZRaw"],
                      "accZ_madgwick": om["s"]["worldAccZ"], "accZ_fusion": of["s"]["worldAccZ"],
                      "tilt_diff_deg": tilt, "fusion_acc_ignored": of["f"]["accIgnored"],
                      "fusion_acc_error_deg": of["f"]["accErrDeg"]}).to_csv(
            os.path.join(args.out, f"traces_{tag}.csv"), index=False, float_format="%.5f")
        chart_traces(om, of, rm, tilt, ev, tag, os.path.join(args.out, f"traces_{tag}.png"))
        chart_reconstruction(om, d, ang, tag, os.path.join(args.out, f"reconstruction_{tag}.png"))
        row = {"tag": tag, "layout": d["layout"], "samples": int(len(d["s"])), "duration_s": float(d["s"]["sessionT"].iloc[-1]),
               "input": info, "reconstruction": resid, "compare": cmp_, "plaus_madgwick": pm, "plaus_fusion": pf,
               "impacts_s": ev}
        # control: the same Madgwick build with accelScaleG moved by +0.01 (inside
        # the reconstruction uncertainty) - how much reps move from a change that
        # is NOT a filter change (the engine's own sensitivity floor)
        ctrl_init = init.split(",")
        ctrl_init[1] = "%.6f" % (float(ctrl_init[1]) + 0.01)
        with quiet_parse():
            oc = run(bins["madgwick"], ",".join(ctrl_init), lines)
        cc, _, rc, _, _ = compare(om, oc, d, ev)
        row["control_madgwick_scale+0.01"] = {"compare": cc, "plaus": plausibility(oc, rc)}
        for i, j in match_reps(rm, rc)[0]:
            control_pooled.append({"tag": tag, "mv_m": rm.loc[i, "mv"], "mv_c": rc.loc[j, "mv"],
                                   "pv_m": rm.loc[i, "pv"], "pv_c": rc.loc[j, "pv"],
                                   "disp_m": rm.loc[i, "disp"], "disp_c": rc.loc[j, "disp"]})
        if "mv_bias" in cc:
            print(f"  control (Madgwick, accelScaleG +0.01): reps {cc['reps_fusion']}, MV {cc['mv_bias']:+.4f} ± "
                  f"{cc['mv_sd']:.4f} (max {cc['mv_maxabs']:.3f}) m/s")
        # one-at-a-time variants
        row["variants"] = {}
        for vname, ch in EXTRA_VARIANTS.items():
            with quiet_parse():
                ov = run(bins["fusion"], init, lines, {**FUSION_DEFAULT, **ch})
            c2, _, rv, _, _ = compare(om, ov, d, ev)
            row["variants"][vname] = {"compare": c2, "plaus": plausibility(ov, rv)}
        # encoder
        enc_csv = encoders.get(path)
        if args.selftest_encoder and not enc_csv:
            enc_csv = os.path.join(args.out, f"{tag}_SELFTEST_fake_encoder.csv")
            selftest_encoder_csv(om, enc_csv)
            row["encoder_is_selftest"] = True
        if enc_csv:
            vlogs = {}
            for name, o in (("madgwick", om), ("fusion", of)):
                vlogs[name] = os.path.join(args.out, f"{tag}_{name}_replay.log")
                write_variant_log(o, vlogs[name])
            with quiet_parse():
                row["encoder"] = encoder_agreement(vlogs, enc_csv, args.out, tag)
            for name, r in row["encoder"].items():
                mv = r.get("MV", {})
                print(f"  encoder vs {name}: TP {r['TP']} FN {r['FN']} FP {r['FP']}, MV bias {mv.get('bias', np.nan):+.3f} "
                      f"MAE {mv.get('mae', np.nan):.3f} m/s" + ("  [SELF-TEST, fake encoder]" if row.get("encoder_is_selftest") else ""))
        # sweep
        if not args.no_sweep:
            for gain, rej, to in itertools.product(SWEEP["gain"], SWEEP["accRej"], SWEEP["timeout"]):
                st = {**FUSION_DEFAULT, "gain": gain, "accRej": rej, "timeout": to}
                with quiet_parse():
                    ov = run(bins["fusion"], init, lines, st)
                c2, _, rv, _, _ = compare(om, ov, d, ev)
                p2 = plausibility(ov, rv)
                sweep_rows.append({"tag": tag, "gain": gain, "accRej": rej, "timeout": to,
                                   "reps": c2["reps_fusion"], "rep_count_diff": c2["only_madgwick"] + c2["only_fusion"],
                                   "mv_mae": c2.get("mv_mae", np.nan), "mv_bias": c2.get("mv_bias", np.nan),
                                   "pv_mae": c2.get("pv_mae", np.nan), "disp_mae": c2.get("disp_mae", np.nan),
                                   "tilt_p95": c2["tilt_diff_deg_p95"], "disp_cv_pct": p2.get("disp_cv_pct", np.nan),
                                   "disp_cv_pct_madgwick": pm.get("disp_cv_pct", np.nan),
                                   "disp_cv_delta": p2.get("disp_cv_pct", np.nan) - pm.get("disp_cv_pct", np.nan),
                                   "drift_delta": p2.get("bracket_absdrift_median", np.nan) - pm.get("bracket_absdrift_median", np.nan),
                                   "bracket_absdrift_median": p2.get("bracket_absdrift_median", np.nan),
                                   "bracket_net_disp_median_m": p2.get("bracket_net_disp_median_m", np.nan),
                                   "acc_ignored_pct": p2["acc_ignored_pct"]})
        row["seed_effect"] = seed_row
        summary.append(row)

    ctrl_df = pd.DataFrame(control_pooled)
    ctrl_df.to_csv(os.path.join(args.out, "reps_control_all.csv"), index=False)
    pooled_df = pd.DataFrame(pooled)
    pooled_df.to_csv(os.path.join(args.out, "reps_matched_all.csv"), index=False)
    if len(pooled_df) >= 3:
        chart_reps(pooled_df, os.path.join(args.out, "reps_fusion_vs_madgwick.png"), ctrl_df)
    chart_plausibility(summary, os.path.join(args.out, "plausibility.png"))
    if sweep_rows:
        sw = pd.DataFrame(sweep_rows)
        sw.to_csv(os.path.join(args.out, "sweep.csv"), index=False)
        # median over the logs: one pathological set must not drive the picture
        agg = sw.groupby(["gain", "accRej", "timeout"]).median(numeric_only=True).reset_index()
        agg["rep_count_diff"] = sw.groupby(["gain", "accRej", "timeout"])["rep_count_diff"].sum().values
        agg.to_csv(os.path.join(args.out, "sweep_mean.csv"), index=False)
        chart_sweep(agg, os.path.join(args.out, "sweep.png"))
    table = {"legacy": {}, "fixed": {}, "max_err_fixed": {}, "err_first_sample_fixed": {}}
    for deg in CONV_TILTS:
        c = convergence_test(bins, tilt_gravity(deg), 90.0)
        for seed in ("legacy", "fixed"):
            table[seed][deg] = {k: (float(t[np.argmax(e < 1.0)]) if (e < 1.0).any() else None)
                                for (k, sd), (t, e) in c.items() if sd == seed}
        table["max_err_fixed"][deg] = {k: float(e.max()) for (k, sd), (t, e) in c.items() if sd == "fixed"}
        table["err_first_sample_fixed"][deg] = {k: float(e[0]) for (k, sd), (t, e) in c.items() if sd == "fixed"}
    chart_convergence_vs_tilt(table, os.path.join(args.out, "calibrate_convergence_vs_tilt.png"))
    conv = convergence_test(bins, first_grav)
    chart_convergence(conv, os.path.join(args.out, "calibrate_convergence.png"))
    conv_s = {f"{k} ({sd})": float(t[np.argmax(e < 1.0)]) if (e < 1.0).any() else None
              for (k, sd), (t, e) in conv.items()}
    print("\nconvergence to < 1 deg after CALIBRATE (synthetic, still, first log's tilt): " +
          ", ".join(f"{k} {v if v is None else round(v, 2)} s" for k, v in conv_s.items()))
    print("after CALIBRATE vs tilt - legacy seed, s to < 1 deg / fixed seed, max error deg:")
    for deg in CONV_TILTS:
        print(f"  {deg:3d} deg: " + "; ".join(
            f"{k}: {table['legacy'][deg][k]} s / {table['max_err_fixed'][deg][k]:.4f} deg" for k in CONV_STYLES))
    if len(pooled_df):
        dd = pooled_df["mv_f"] - pooled_df["mv_m"]
        print(f"\nALL LOGS: {len(pooled_df)} matched reps, MV Fusion − Madgwick {dd.mean():+.4f} ± {dd.std(ddof=1):.4f} m/s, "
              f"max |Δ| {dd.abs().max():.3f}, |Δ| > 0.02 m/s in {int((dd.abs() > 0.02).sum())} reps")
    if len(ctrl_df):
        dc = ctrl_df["mv_c"] - ctrl_df["mv_m"]
        print(f"CONTROL : {len(ctrl_df)} matched reps, MV control − Madgwick {dc.mean():+.4f} ± {dc.std(ddof=1):.4f} m/s, "
              f"max |Δ| {dc.abs().max():.3f}, |Δ| > 0.02 m/s in {int((dc.abs() > 0.02).sum())} reps")
    with open(os.path.join(args.out, "results.json"), "w") as f:
        json.dump({"fusion_default": FUSION_DEFAULT, "sweep_grid": SWEEP, "logs": summary,
                   "calibrate_convergence_s": conv_s, "calibrate_convergence_vs_tilt_s": table}, f, indent=2, default=float)
    print(f"\nresults: {args.out}/results.json, charts: {args.out}/*.png")


if __name__ == "__main__":
    main()
