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
Statistics of the validation study (PROTOCOL.md section 7), computed
deterministically from the dataset folder:

  validity     ODKI vs reference, per metric (MV, PV, MPV, ROM) and per value
               (first reported / final): bias and 95% limits of agreement
               (Bland-Altman, plain and for repeated measures, Bland & Altman
               2007), proportional bias (OLS of the difference on the mean),
               ICC(2,1) absolute agreement and ICC(3,1) consistency, SEM
               (typical error), CV% (log typical error, Hopkins), MAE, RMSE,
               Pearson r, SEE, standardised bias
  strata       exercise, velocity band (reference MV), correction status, unit,
               phase close reason
  detection    TP / FN / FP, recall, precision, F1, false-rep rate (Wilson 95% CI)
  reliability  between units (same reps, two sensors) and test-retest (day 1
               vs day 2, per lifter x exercise x load): ICC, SEM, MDC95, SWC
  quality1     Spearman rho with |MV error|, ROC AUC for |error| > 0.10 m/s
  thresholds   pass / fail against the pre-registered criteria (common.THRESHOLDS)

95% CIs: cluster bootstrap over lifters (2000 resamples, fixed seed), so the
output is identical on every run. numpy, pandas and matplotlib only.

Output: <out>/summary.json, <out>/RESULTS.md, <out>/mv_<exercise>.png

Usage:
    python analyze.py --data DATA_DIR [--out DATA_DIR/results] [--no-plots]
"""

import argparse
import json
import os

import numpy as np
import pandas as pd

import common as C

SEED = 12345
N_BOOT = 2000
METRICS = {  # name: (odki column, reference column); {v} = first | final
    "MV": ("odki_mv_{v}", "ref_mv_odkidef"),
    "MV_std_def": ("odki_mv_{v}", "ref_mv_std"),
    "PV": ("odki_pv_{v}", "ref_pv"),
    "MPV": ("odki_mpv_offline", "ref_mpv"),
    "ROM": ("odki_rom_{v}", "ref_rom_m"),
}
PRIMARY_EXERCISES = ("back_squat", "bench_press")


# ----------------------------------------------------------------------
# Statistics
# ----------------------------------------------------------------------

def icc_two_way(y):
    """y: n targets x k raters, complete. Returns (ICC(A,1), ICC(C,1)) per
    McGraw & Wong 1996 - numerically ICC(2,1) absolute agreement and
    ICC(3,1) consistency of Shrout & Fleiss 1979."""
    y = np.asarray(y, float)
    n, k = y.shape
    if n < 2 or k < 2:
        return np.nan, np.nan
    gm = y.mean()
    ss_r = k * ((y.mean(axis=1) - gm) ** 2).sum()
    ss_c = n * ((y.mean(axis=0) - gm) ** 2).sum()
    ss_e = ((y - gm) ** 2).sum() - ss_r - ss_c
    msr, msc, mse = ss_r / (n - 1), ss_c / (k - 1), ss_e / ((n - 1) * (k - 1))
    a1 = (msr - mse) / (msr + (k - 1) * mse + k / n * (msc - mse))
    c1 = (msr - mse) / (msr + (k - 1) * mse)
    return float(a1), float(c1)


def ba_repeated(d, subj):
    """Bland & Altman 2007, true value varying: SD of the differences from
    the between- and within-subject variance components (one-way ANOVA)."""
    df = pd.DataFrame({"d": d, "s": subj})
    g = df.groupby("s")["d"]
    m = g.size().values
    k, n = len(m), len(d)
    if k < 2 or n <= k:
        return float(np.std(d, ddof=1)) if n > 1 else np.nan
    msw = ((df["d"] - g.transform("mean")) ** 2).sum() / (n - k)
    msb = (m * (g.mean().values - df["d"].mean()) ** 2).sum() / (k - 1)
    div = (n ** 2 - (m ** 2).sum()) / ((k - 1) * n)
    s2b = max(0.0, (msb - msw) / div)
    return float(np.sqrt(s2b + msw))


def agreement(o, r, subj):
    """Point estimates of every validity statistic for paired arrays."""
    o, r = np.asarray(o, float), np.asarray(r, float)
    d = o - r
    n = len(d)
    out = {"n": int(n)}
    if n < 3:
        return out
    sd = float(np.std(d, ddof=1))
    sd_rm = ba_repeated(d, subj)
    bias = float(d.mean())
    mean_pair = (o + r) / 2
    slope = float(np.polyfit(mean_pair, d, 1)[0]) if np.ptp(mean_pair) > 0 else np.nan
    rr = float(np.corrcoef(o, r)[0, 1]) if np.std(o) > 0 and np.std(r) > 0 else np.nan
    b1, b0 = np.polyfit(o, r, 1) if np.ptp(o) > 0 else (np.nan, np.nan)
    see = float(np.sqrt(np.sum((r - (b1 * o + b0)) ** 2) / (n - 2)))
    pos = (o > 0) & (r > 0)
    cv = (float(100 * (np.exp(np.std(np.log(o[pos]) - np.log(r[pos]), ddof=1) / np.sqrt(2)) - 1))
          if pos.sum() > 2 else np.nan)
    sem = sd / np.sqrt(2)
    a1, c1 = icc_two_way(np.c_[o, r])
    out.update({
        "bias": bias, "sd_diff": sd, "loa_low": bias - 1.96 * sd, "loa_high": bias + 1.96 * sd,
        "sd_diff_rm": sd_rm, "loa_low_rm": bias - 1.96 * sd_rm, "loa_high_rm": bias + 1.96 * sd_rm,
        "loa_halfwidth_rm": 1.96 * sd_rm, "prop_bias_slope": slope,
        "icc_2_1": a1, "icc_3_1": c1,
        "sem": float(sem), "mdc95": float(1.96 * np.sqrt(2) * sem), "cv_pct": cv,
        "mae": float(np.mean(np.abs(d))), "rmse": float(np.sqrt(np.mean(d ** 2))),
        "pearson_r": rr, "see": see,
        "std_bias": float(bias / np.std(r, ddof=1)) if np.std(r) > 0 else np.nan,
    })
    return out


def bootstrap_ci(df, fn, keys, cluster="lifter_id"):
    """Percentile 95% CI by resampling clusters (lifters) with replacement;
    reps are resampled instead when there are fewer than 3 lifters."""
    rng = np.random.default_rng(SEED)
    groups = [g for _, g in df.groupby(cluster)] if df[cluster].nunique() >= 3 else None
    vals = {k: [] for k in keys}
    for _ in range(N_BOOT):
        if groups:
            b = pd.concat([groups[i] for i in rng.integers(0, len(groups), len(groups))])
        else:
            b = df.iloc[rng.integers(0, len(df), len(df))]
        res = fn(b)
        for k in keys:
            vals[k].append(res.get(k, np.nan))
    return {k: [float(np.nanpercentile(v, 2.5)), float(np.nanpercentile(v, 97.5))] for k, v in vals.items()}


def wilson(k, n, z=1.96):
    if n == 0:
        return [np.nan, np.nan]
    p = k / n
    den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return [float(c - h), float(c + h)]


def rank(x):
    return pd.Series(x).rank().values


def spearman(a, b):
    return float(np.corrcoef(rank(a), rank(b))[0, 1]) if len(a) > 2 else np.nan


def auc(score, positive):
    """ROC AUC (Mann-Whitney) of score for the positive class."""
    pos, neg = score[positive], score[~positive]
    if not len(pos) or not len(neg):
        return np.nan
    r = rank(np.r_[pos, neg])
    return float((r[:len(pos)].sum() - len(pos) * (len(pos) + 1) / 2) / (len(pos) * len(neg)))


# ----------------------------------------------------------------------
# Analysis
# ----------------------------------------------------------------------

def load(data):
    reps = pd.read_csv(os.path.join(data, "reps_paired.csv"))
    sets = pd.read_csv(os.path.join(data, "sets.csv"))
    sessions = pd.read_csv(os.path.join(data, "sessions.csv"))
    reps = reps.merge(sets.drop(columns=[c for c in ("physical_set_id", "unit_id") if c in sets.columns]),
                      on="set_id", how="left")
    reps = reps.merge(sessions[["session_id", "lifter_id", "day_index", "exercise"]], on="session_id", how="left")
    reps["exclude_reason"] = reps["exclude_reason"].fillna("")
    reps["velocity_band"] = [C.velocity_band(v) if np.isfinite(v) else "n/a" for v in reps["ref_mv_std"]]
    return reps


def metric_frame(tp, metric, which):
    oc, rc = METRICS[metric]
    oc = oc.format(v=which)
    m = tp[[oc, rc]].notna().all(axis=1)
    return tp[m], tp.loc[m, oc].values, tp.loc[m, rc].values


def validity(tp, metric="MV", which="final", ci=True):
    sub, o, r = metric_frame(tp, metric, which)
    res = agreement(o, r, sub["lifter_id"].values)
    if ci and res["n"] >= 10:
        keys = ["bias", "loa_low_rm", "loa_high_rm", "icc_2_1", "sem", "cv_pct", "pearson_r", "prop_bias_slope"]

        def one(b):
            bs, bo, br = metric_frame(b, metric, which)
            return agreement(bo, br, bs["lifter_id"].values)
        res["ci95"] = bootstrap_ci(sub, one, keys)
    return res


def detection(reps):
    n = reps["match"].value_counts()
    tp, fn, fp = int(n.get("TP", 0)), int(n.get("FN", 0)), int(n.get("FP", 0))
    rec = tp / (tp + fn) if tp + fn else np.nan
    prec = tp / (tp + fp) if tp + fp else np.nan
    return {"TP": tp, "FN": fn, "FP": fp, "recall": rec, "recall_ci95": wilson(tp, tp + fn),
            "precision": prec, "precision_ci95": wilson(tp, tp + fp),
            "f1": 2 * rec * prec / (rec + prec) if rec + prec else np.nan,
            "false_rep_rate": fp / (tp + fp) if tp + fp else np.nan, "false_rep_rate_ci95": wilson(fp, tp + fp)}


def between_units(tp):
    if tp["unit_id"].nunique() < 2:
        return {"available": False, "reason": "one unit only"}
    piv = tp.pivot_table(index=["physical_set_id", "ref_rep_idx"], columns="unit_id", values="odki_mv_final")
    units = sorted(piv.columns)[:2]
    y = piv[units].dropna().values
    if len(y) < 3:
        return {"available": False, "reason": "fewer than 3 reps measured by two units"}
    d = y[:, 0] - y[:, 1]
    sem = np.std(d, ddof=1) / np.sqrt(2)
    cv = 100 * (np.exp(np.std(np.log(y[:, 0]) - np.log(y[:, 1]), ddof=1) / np.sqrt(2)) - 1)
    a1, c1 = icc_two_way(y)
    return {"available": True, "units": units, "n": int(len(y)), "icc_2_1": a1, "icc_3_1": c1,
            "sem": float(sem), "cv_pct": float(cv), "mean_diff": float(d.mean())}


def test_retest(tp, col):
    if tp["day_index"].nunique() < 2:
        return {"available": False, "reason": "one day only"}
    g = tp.groupby(["lifter_id", "exercise", "pct_1rm_est", "day_index"])[col].mean().unstack("day_index")
    days = sorted(g.columns)[:2]
    y = g[days].dropna()
    if len(y) < 3:
        return {"available": False, "reason": "fewer than 3 lifter x load units with both days"}
    d = y[days[1]] - y[days[0]]
    sem = float(np.std(d, ddof=1) / np.sqrt(2))
    a1, c1 = icc_two_way(y.values)
    lifter_means = y.mean(axis=1).groupby(level="lifter_id").mean()
    swc = float(0.2 * np.std(lifter_means, ddof=1)) if len(lifter_means) > 1 else np.nan
    return {"available": True, "n_units": int(len(y)), "icc_3_1_consistency": c1, "icc_absolute": a1,
            "sem": sem, "mdc95": 1.96 * np.sqrt(2) * sem, "swc": swc,
            "sem_below_swc": bool(sem < swc) if np.isfinite(swc) else None}


def quality1(tp):
    sub = tp[["odki_quality1_final", "odki_mv_final", "ref_mv_odkidef"]].dropna()
    if len(sub) < 10:
        return {"available": False}
    err = np.abs(sub["odki_mv_final"] - sub["ref_mv_odkidef"]).values
    q = sub["odki_quality1_final"].values
    return {"available": True, "n": int(len(sub)), "spearman_rho_quality_vs_abs_error": spearman(q, err),
            "auc_low_quality_flags_error_gt_0.10": auc(100 - q, err > 0.10)}


def strata(tp, col):
    return {str(k): validity(g, "MV", "final", ci=False) for k, g in tp.groupby(col)}


_OPS = {"pearson_r_min": ("pearson_r", lambda v, t: v >= t), "sem_max_mps": ("sem", lambda v, t: v <= t),
        "abs_bias_max_mps": ("bias", lambda v, t: abs(v) <= t), "cv_pct_max": ("cv_pct", lambda v, t: v < t),
        "loa_halfwidth_max_mps": ("loa_halfwidth_rm", lambda v, t: v <= t),
        "icc_2_1_min": ("icc_2_1", lambda v, t: v >= t), "std_bias_max": ("std_bias", lambda v, t: abs(v) < t),
        "recall_min": ("recall", lambda v, t: v >= t),
        "false_rep_rate_max": ("false_rep_rate", lambda v, t: v <= t)}


def check_thresholds(primary, det):
    res = {}
    for tier, spec in C.THRESHOLDS.items():
        src = det if tier == "rep_detection" else primary
        rows = {}
        for crit, thr in spec["criteria"].items():
            key, ok = _OPS[crit]
            v = src.get(key, np.nan)
            rows[crit] = {"value": v, "threshold": thr, "pass": bool(np.isfinite(v) and ok(v, thr))}
        res[tier] = {"label": spec["label"], "source": spec["source"],
                     "pass": all(r["pass"] for r in rows.values()), "criteria": rows}
    return res


def analyze(data):
    reps = load(data)
    excluded = reps[reps["exclude_reason"] != ""]
    use = reps[reps["exclude_reason"] == ""]
    prim = use[use["exercise"].isin(PRIMARY_EXERCISES)]
    tp = use[use["match"] == "TP"]
    tp_prim = prim[prim["match"] == "TP"]

    s = {"n_rows": int(len(reps)), "n_excluded": int(len(excluded)),
         "excluded_by_reason": excluded["exclude_reason"].value_counts().to_dict(),
         "n_lifters": int(use["lifter_id"].nunique()),
         "exercises": sorted(use["exercise"].dropna().unique().tolist())}
    s["primary"] = validity(tp_prim, "MV", "final")
    s["validity"] = {f"{m}_{w}": validity(tp_prim, m, w, ci=False)
                     for m in METRICS for w in (("first", "final") if "{v}" in METRICS[m][0] else ("offline",))}
    s["by_exercise"] = {ex: {"MV_final": validity(g[g["match"] == "TP"], "MV", "final", ci=False),
                             "detection": detection(g)} for ex, g in use.groupby("exercise")}
    s["by_velocity_band"] = {ex: strata(g, "velocity_band") for ex, g in tp.groupby("exercise")}
    s["by_status_final"] = strata(tp_prim, "odki_status_final")
    s["by_unit"] = strata(tp_prim, "unit_id")
    s["by_close_reason"] = strata(tp_prim, "close_reason")
    s["detection"] = detection(prim)
    s["between_units"] = between_units(tp_prim)
    if len(tp_prim):
        first_unit = sorted(tp_prim["unit_id"].unique())[0]
        s["test_retest_odki"] = test_retest(tp_prim[tp_prim["unit_id"] == first_unit], "odki_mv_final")
        s["test_retest_reference"] = test_retest(tp_prim[tp_prim["unit_id"] == first_unit], "ref_mv_odkidef")
    else:
        s["test_retest_odki"] = s["test_retest_reference"] = {"available": False}
    s["quality1"] = quality1(tp_prim)
    s["thresholds"] = check_thresholds(s["primary"], s["detection"])
    return s, reps


def clean(o):
    """JSON-safe, rounded, NaN -> null."""
    if isinstance(o, dict):
        return {str(k): clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [clean(v) for v in o]
    if isinstance(o, (np.bool_, bool)):
        return bool(o)
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.floating, float)):
        return None if not np.isfinite(o) else round(float(o), 6)
    return o


def fmt(v, nd=3):
    return "n/a" if v is None or (isinstance(v, float) and not np.isfinite(v)) else f"{v:.{nd}f}"


def write_markdown(s, path):
    p = s["primary"]
    ci = p.get("ci95", {})
    L = ["# Validation results", "",
         "Generated by `analyze.py` from the dataset; do not edit by hand. Protocol: `PROTOCOL.md`.", "",
         f"Lifters: {s['n_lifters']} · exercises: {', '.join(s['exercises'])} · rows excluded: {s['n_excluded']}", "",
         "## Primary endpoint: mean velocity (final value) vs reference, squat + bench", "",
         "| statistic | value | 95% CI |", "|---|---|---|"]
    for k, label in [("n", "paired reps"), ("bias", "bias (m/s)"), ("loa_low_rm", "LoA low, repeated measures (m/s)"),
                     ("loa_high_rm", "LoA high, repeated measures (m/s)"), ("prop_bias_slope", "proportional bias slope"),
                     ("icc_2_1", "ICC(2,1) absolute agreement"), ("sem", "SEM / typical error (m/s)"),
                     ("cv_pct", "CV (%)"), ("mae", "MAE (m/s)"), ("rmse", "RMSE (m/s)"),
                     ("pearson_r", "Pearson r"), ("see", "SEE (m/s)"), ("std_bias", "standardised bias")]:
        c = ci.get(k)
        val = p.get(k) if k == "n" else fmt(p.get(k))
        L.append(f"| {label} | {val} | {'' if not c else fmt(c[0]) + ' to ' + fmt(c[1])} |")
    L += ["", "## Pre-registered thresholds", "", "| tier | criterion | value | threshold | pass |",
          "|---|---|---|---|---|"]
    for tier, t in s["thresholds"].items():
        for crit, r in t["criteria"].items():
            L.append(f"| {tier} | {crit} | {fmt(r['value'])} | {r['threshold']} | {'yes' if r['pass'] else 'NO'} |")
    d = s["detection"]
    L += ["", "## Rep detection (squat + bench)", "",
          f"TP {d['TP']} · FN {d['FN']} · FP {d['FP']} · recall {fmt(d['recall'])} "
          f"({fmt(d['recall_ci95'][0])} to {fmt(d['recall_ci95'][1])}) · precision {fmt(d['precision'])} · "
          f"false-rep rate {fmt(d['false_rep_rate'])}", "",
          "## Every metric (squat + bench)", "", "| metric | n | bias | LoA (rm) | ICC(2,1) | SEM | CV% | r |",
          "|---|---|---|---|---|---|---|---|"]
    for k, v in s["validity"].items():
        if v.get("n", 0) >= 3:
            L.append(f"| {k} | {v['n']} | {fmt(v['bias'])} | {fmt(v['loa_low_rm'])} to {fmt(v['loa_high_rm'])} | "
                     f"{fmt(v['icc_2_1'])} | {fmt(v['sem'])} | {fmt(v['cv_pct'], 1)} | {fmt(v['pearson_r'])} |")
    L += ["", "## By exercise and velocity band (MV, final)", "",
          "| exercise | band | n | bias | LoA (rm) | SEM | CV% |", "|---|---|---|---|---|---|---|"]
    for ex, bands in s["by_velocity_band"].items():
        for b, v in bands.items():
            if v.get("n", 0) >= 3:
                L.append(f"| {ex} | {b} | {v['n']} | {fmt(v['bias'])} | {fmt(v['loa_low_rm'])} to "
                         f"{fmt(v['loa_high_rm'])} | {fmt(v['sem'])} | {fmt(v['cv_pct'], 1)} |")
    for key, title in [("between_units", "Between units"), ("test_retest_odki", "Test-retest, ODKI"),
                       ("test_retest_reference", "Test-retest, reference (biological + reference error)"),
                       ("quality1", "quality1 vs error")]:
        L += ["", f"## {title}", "", "```json", json.dumps(s[key], indent=2), "```"]
    L += ["", "Limits: in-house study; the reference is a linear encoder, verified as in PROTOCOL.md section 4. "
          "These numbers describe agreement with a linear encoder, not a validation against 3D motion capture.", ""]
    with open(path, "w") as f:
        f.write("\n".join(L))


def plots(reps, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    tp = reps[(reps["match"] == "TP") & (reps["exclude_reason"] == "")]
    for ex, g in tp.groupby("exercise"):
        g = g.dropna(subset=["odki_mv_final", "ref_mv_odkidef"])
        if len(g) < 3:
            continue
        o, r = g["odki_mv_final"].values, g["ref_mv_odkidef"].values
        a = agreement(o, r, g["lifter_id"].values)
        fig, ax = plt.subplots(1, 2, figsize=(11, 4.5))
        ax[0].scatter((o + r) / 2, o - r, s=10, alpha=0.6, color="#1f4fb4")
        for y, ls in [(a["bias"], "-"), (a["loa_low_rm"], "--"), (a["loa_high_rm"], "--")]:
            ax[0].axhline(y, color="#c9392c", ls=ls, lw=1)
        ax[0].set_xlabel("mean of ODKI and reference MV (m/s)")
        ax[0].set_ylabel("ODKI - reference (m/s)")
        ax[0].set_title(f"{ex}: Bland-Altman, n = {len(g)}")
        lim = [0, max(o.max(), r.max()) * 1.05]
        ax[1].scatter(r, o, s=10, alpha=0.6, color="#1f4fb4")
        ax[1].plot(lim, lim, color="#888", lw=0.8)
        ax[1].set_xlabel("reference MV (m/s)")
        ax[1].set_ylabel("ODKI MV, final (m/s)")
        ax[1].set_title(f"r = {a['pearson_r']:.3f}, SEE = {a['see']:.3f} m/s")
        for x in ax:
            x.grid(alpha=0.25)
        fig.tight_layout()
        fig.savefig(os.path.join(out, f"mv_{ex}.png"), dpi=110, metadata={"Software": None})
        plt.close(fig)


def run(data, out=None, make_plots=True):
    out = out or os.path.join(data, "results")
    os.makedirs(out, exist_ok=True)
    s, reps = analyze(data)
    s = clean(s)
    with open(os.path.join(out, "summary.json"), "w") as f:
        json.dump(s, f, indent=2, sort_keys=True)
    write_markdown(s, os.path.join(out, "RESULTS.md"))
    if make_plots:
        plots(reps, out)
    return s, out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", required=True)
    ap.add_argument("--out", default=None)
    ap.add_argument("--no-plots", action="store_true")
    args = ap.parse_args()
    s, out = run(args.data, args.out, not args.no_plots)
    p = s["primary"]
    print(f"MV final, squat + bench: n {p.get('n')}  bias {fmt(p.get('bias'))}  "
          f"LoA {fmt(p.get('loa_low_rm'))} to {fmt(p.get('loa_high_rm'))}  ICC(2,1) {fmt(p.get('icc_2_1'))}")
    print(f"wrote {out}/summary.json, RESULTS.md")


if __name__ == "__main__":
    main()
