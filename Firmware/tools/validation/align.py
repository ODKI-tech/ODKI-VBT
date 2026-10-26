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
Time alignment of one ODKI log with the reference linear encoder
(PROTOCOL.md section 5.2). The two are recorded by different programs on
different clocks (the encoder by its own software), so there is no shared
host clock and the alignment comes from the signals alone, in three steps:

  1. coarse  - full-range cross-correlation of the ODKI drift-corrected
               velocity (rebuilt with vbt_log_viewer.reconstruct_corrected)
               with the reference velocity on a common 100 Hz grid, both signs
  2. fine    - the same correlation searched +/-300 ms around the coarse
               estimate at 1 ms steps with parabolic refinement; repeated on
               the first and the second half of the set, whose difference
               gives the clock drift (ppm), corrected linearly
  3. check   - the sync drop at the start of the set: the peak of ODKI's raw
               vertical acceleration and the peak of the reference
               acceleration must fall within +/-10 ms after alignment,
               otherwise the set is flagged; so is a set whose reference
               has a gap above two sample intervals

Output, for set_id S:
  aligned/S.csv   t_s, v_ref, v_odki_corrected, v_odki_raw, acc_odki_raw at the
                  ODKI sample times (t_s = ODKI session time, 100 Hz)
  aligned/S.json  sign, offset_s, lag_ms, drift_ppm, xcorr_r, sync_check_ms, flagged

Usage:
    python align.py --data DATA_DIR --set-id L01-D1-SQ-060-S1_U1
    python align.py --data DATA_DIR --all
  The reference scale factor comes from the static verification in
  reference_verification.csv (the reference_id of the set's session; 1.0 if
  there is none); --ref-scale overrides it.
"""

import argparse
import json
import os
import sys

import numpy as np
import pandas as pd

import common as C


def pearson(a, b):
    m = np.isfinite(a) & np.isfinite(b)
    if m.sum() < 20:
        return np.nan
    a, b = a[m] - a[m].mean(), b[m] - b[m].mean()
    den = np.sqrt((a * a).sum() * (b * b).sum())
    return float((a * b).sum() / den) if den > 0 else np.nan


def parabolic(xs, ys, k):
    """Sub-step peak location around index k."""
    if 0 < k < len(ys) - 1 and np.all(np.isfinite(ys[k - 1:k + 2])):
        y0, y1, y2 = ys[k - 1], ys[k], ys[k + 1]
        den = y0 - 2 * y1 + y2
        if den != 0:
            return xs[k] + 0.5 * (y0 - y2) / den * (xs[1] - xs[0])
    return xs[k]


def coarse_from_xcorr(t_dev, v_o, t_e, x):
    """Full-range cross-correlation on a common 100 Hz grid (both signs)."""
    step = 1.0 / C.GRID_HZ
    go = np.arange(t_dev[0], t_dev[-1], step)
    vo = np.interp(go, t_dev, np.nan_to_num(v_o))
    ge = np.arange(t_e[0] + 0.01, t_e[-1] - 0.01, step)
    ve = C.ref_velocity_at(t_e, x, ge)
    vo, ve = vo - vo.mean(), np.nan_to_num(ve - np.nanmean(ve))
    n = 1 << int(np.ceil(np.log2(len(vo) + len(ve))))
    cc = np.fft.irfft(np.fft.rfft(ve, n) * np.conj(np.fft.rfft(vo, n)), n)
    k = int(np.argmax(np.abs(cc)))
    lag = k if k < n // 2 else k - n          # ve[i + lag] ~ vo[i]
    return float(ge[0] + lag * step - go[0])


def search(t_dev, v_o, t_e, x, center, half_range, step, mask):
    offs = np.arange(center - half_range, center + half_range + step / 2, step)
    rs = np.array([pearson(v_o[mask], C.ref_velocity_at(t_e, x, t_dev[mask] + c)) for c in offs])
    k = int(np.nanargmax(rs))
    return parabolic(offs, rs, k), float(rs[k])


def peak_time(t, y, t_max):
    m = (t <= t[0] + t_max) & np.isfinite(y)
    if m.sum() < 3:
        return np.nan, np.nan
    idx = np.where(m)[0]
    k = idx[int(np.argmax(y[idx]))]
    tt = t[k]
    if 0 < k < len(y) - 1 and np.all(np.isfinite(y[k - 1:k + 2])):
        y0, y1, y2 = y[k - 1], y[k], y[k + 1]
        den = y0 - 2 * y1 + y2
        if den != 0:
            tt = t[k] + 0.5 * (y0 - y2) / den * (t[k + 1] - t[k])
    return float(tt), float(y[k])


def align_set(log_path, ref_path, ref_scale=C.DEFAULT_REF_SCALE, session=None):
    d = C.read_odki(log_path, session)
    s = d["s"]
    t_s = s["sessionT"].values
    t_dev = C.odki_device_time(s)
    v_o = s["corrected"].values
    ref = C.read_ref(ref_path)
    t_e, x = C.ref_position(ref, ref_scale)
    c0, method = coarse_from_xcorr(t_dev, v_o, t_e, x), "xcorr"

    valid = np.isfinite(v_o)
    # sign: the encoder may report upward motion as negative (how it is mounted)
    r_pos = search(t_dev, v_o, t_e, x, c0, C.LAG_SEARCH_S, 0.005, valid)
    r_neg = search(t_dev, v_o, t_e, -x, c0, C.LAG_SEARCH_S, 0.005, valid)
    sign = 1.0 if r_pos[1] >= r_neg[1] else -1.0
    x = sign * x
    c_all, r_all = search(t_dev, v_o, t_e, x, c0, C.LAG_SEARCH_S, 0.001, valid)

    mid = 0.5 * (t_dev[valid][0] + t_dev[valid][-1])
    h1, h2 = valid & (t_dev < mid), valid & (t_dev >= mid)
    c1, _ = search(t_dev, v_o, t_e, x, c_all, 0.05, 0.0005, h1)
    c2, _ = search(t_dev, v_o, t_e, x, c_all, 0.05, 0.0005, h2)
    m1, m2 = np.mean(t_dev[h1]), np.mean(t_dev[h2])
    duration = t_dev[valid][-1] - t_dev[valid][0]
    drift_estimated = duration >= C.MIN_DRIFT_SET_S and m2 > m1
    if drift_estimated:
        k = (c2 - c1) / (m2 - m1)
    else:
        # short set: a quartz drift of tens of ppm is < 1 ms here, below what
        # the half-set cross-correlations can resolve - not estimated
        k, c1, m1 = 0.0, c_all, mid
    offset = c1 + k * (t_dev - m1)               # t_e = t_dev + offset(t_dev)

    v_ref = C.ref_velocity_at(t_e, x, t_dev + offset)
    acc_col = "worldAccZRaw" if s["worldAccZRaw"].notna().any() else "worldAccZ"
    aligned = pd.DataFrame({"t_s": t_s, "v_ref": v_ref, "v_odki_corrected": v_o,
                            "v_odki_raw": s["velZ"].values, "acc_odki_raw": s[acc_col].values})

    # sync check: the drop happens before the first lift, within SYNC_WINDOW_S.
    # Backward difference, like ODKI's acceleration (a mean over the samples
    # leading up to each sample), so both peaks carry the same 5 ms offset.
    acc_ref = np.r_[0.0, np.diff(np.nan_to_num(v_ref)) / np.maximum(np.diff(t_s), 1e-4)]
    seg = C.segment_ref_concentrics(t_s, v_ref)
    win = C.SYNC_WINDOW_S if not seg else min(C.SYNC_WINDOW_S, t_s[seg[0][0]] - 0.3 - t_s[0])
    t_pr, a_pr = peak_time(t_s, acc_ref, win)
    t_po, a_po = peak_time(t_s, aligned["acc_odki_raw"].values, win)
    found = np.isfinite(a_pr) and a_pr > 5.0 and np.isfinite(a_po) and a_po > 5.0
    sync_ms = (t_po - t_pr) * 1000.0 if found else np.nan
    r_final = pearson(v_o, v_ref)
    # sampling check (PROTOCOL.md section 4): no reference gap above two sample
    # intervals over the span of the ODKI set
    te_span = t_e[(t_e >= t_dev[0] + offset[0]) & (t_e <= t_dev[-1] + offset[-1])]
    dte = np.diff(te_span)
    ref_dt = float(np.median(np.diff(t_e)))
    max_gap = float(dte.max()) if len(dte) else np.inf
    gap_ok = max_gap <= 2.0 * ref_dt + 1e-9
    flagged = (not found) or abs(sync_ms) > C.SYNC_TOLERANCE_MS or not (r_final > 0.9) or not gap_ok
    info = {"coarse_method": method, "sign": sign,
            "offset_s": float(c1 + k * (0.5 * (m1 + m2) - m1)),
            "lag_ms": float((c1 + k * (0.5 * (m1 + m2) - m1) - c0) * 1000.0),
            "drift_ppm": float(k * 1e6) if drift_estimated else None, "drift_estimated": bool(drift_estimated),
            "set_duration_s": float(duration), "xcorr_r": r_final,
            "sync_check_ms": None if not np.isfinite(sync_ms) else float(sync_ms),
            "sync_event_found": bool(found), "ref_rate_hz": 1.0 / ref_dt,
            "ref_max_gap_ms": max_gap * 1000.0, "ref_sampling_ok": bool(gap_ok), "flagged": bool(flagged),
            "ref_scale": float(ref_scale), "odki_acc_column": acc_col}
    return aligned, info


def ref_scale_for(data, set_row):
    """Static verification (PROTOCOL.md section 4): mean of expected /
    measured over the 'static' trials of the session's reference_id (rule
    length over encoder reading, same unit). None if there are none."""
    try:
        sessions = pd.read_csv(os.path.join(data, "sessions.csv"))
        ver = pd.read_csv(os.path.join(data, "reference_verification.csv"))
    except FileNotFoundError:
        return None
    ref_id = sessions.set_index("session_id").loc[set_row["session_id"], "reference_id"]
    st = ver[(ver["reference_id"] == ref_id) & (ver["test"] == "static")]
    if not len(st):
        return None
    return float(np.mean(st["expected"].values / st["measured"].values))


def set_paths(data, set_row):
    sid, phys = set_row["set_id"], set_row["physical_set_id"]
    return (os.path.join(data, "raw", "odki", f"{sid}.log"),
            os.path.join(data, "raw", "ref", f"{phys}.csv"),
            os.path.join(data, "raw", "odki", f"{sid}.sidecar.csv"))


def run(data, set_ids=None, scale_override=None, quiet=False):
    sets = pd.read_csv(os.path.join(data, "sets.csv"))
    if set_ids:
        sets = sets[sets["set_id"].isin(set_ids)]
    os.makedirs(os.path.join(data, "aligned"), exist_ok=True)
    results = {}
    for _, row in sets.iterrows():
        scale = scale_override or ref_scale_for(data, row) or C.DEFAULT_REF_SCALE
        log, ref, _side = set_paths(data, row)
        aligned, info = align_set(log, ref, scale)
        aligned.to_csv(os.path.join(data, "aligned", f"{row['set_id']}.csv"), index=False, float_format="%.5f")
        with open(os.path.join(data, "aligned", f"{row['set_id']}.json"), "w") as f:
            json.dump(info, f, indent=2)
        results[row["set_id"]] = info
        if not quiet:
            sc = "n/a" if info["sync_check_ms"] is None else f"{info['sync_check_ms']:+.1f} ms"
            dr = "    n/a" if info["drift_ppm"] is None else f"{info['drift_ppm']:+7.0f}"
            print(f"{row['set_id']:<28} lag {info['lag_ms']:+8.1f} ms  drift {dr} ppm  "
                  f"r {info['xcorr_r']:.3f}  sync {sc}{'  FLAGGED' if info['flagged'] else ''}")
    return results


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", required=True)
    ap.add_argument("--set-id", action="append", help="set to align (repeatable)")
    ap.add_argument("--all", action="store_true", help="align every set in sets.csv")
    ap.add_argument("--ref-scale", type=float, default=None,
                    help="reference position scale factor (default: from the static verification, else 1.0)")
    args = ap.parse_args()
    if not args.all and not args.set_id:
        sys.exit("give --set-id or --all")
    run(args.data, None if args.all else args.set_id, args.ref_scale)


if __name__ == "__main__":
    main()
