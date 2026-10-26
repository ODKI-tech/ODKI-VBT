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
Rep segmentation and pairing (PROTOCOL.md section 5.3).

  reference reps : concentrics segmented from the aligned encoder velocity
                   (runs of v > 0 rising >= 10 cm, gaps < 50 ms merged)
  ODKI reps      : every rep with an R row; its concentric interval comes from
                   the P row's t_start_ms/t_end_ms (firmware >= 3.11.32) or,
                   on older logs, is rebuilt from the S rows' state/rep columns
  pairing        : one-to-one, greedy on temporal intersection-over-union,
                   IoU >= 0.5. Paired = TP, ODKI only = FP (false rep),
                   reference only = FN (missed rep)

Writes reps_paired.csv (one row per TP/FN/FP, PROTOCOL.md section 9) and
sets_alignment.csv (the align.py result of every set) in the dataset folder.
Run align.py first.

Usage:
    python pair_reps.py --data DATA_DIR
"""

import argparse
import json
import os

import numpy as np
import pandas as pd

import common as C
from align import set_paths

COLUMNS = ["rep_uid", "set_id", "physical_set_id", "unit_id", "match", "ref_rep_idx", "odki_rep_number",
           "t_start_ref_s", "t_end_ref_s", "t_start_odki_s", "t_end_odki_s", "iou",
           "ref_mv_std", "ref_mv_odkidef", "ref_pv", "ref_mpv", "ref_rom_m",
           "odki_mv_first", "odki_mv_final", "odki_pv_first", "odki_pv_final",
           "odki_rom_first", "odki_rom_final", "odki_mpv_offline", "odki_ecc_mv_final",
           "odki_quality1_final", "odki_status_first", "odki_status_final", "close_reason", "exclude_reason"]


def iou(a, b):
    inter = max(0.0, min(a[1], b[1]) - max(a[0], b[0]))
    union = max(a[1], b[1]) - min(a[0], b[0])
    return inter / union if union > 0 else 0.0


def match(ref_iv, odki_iv, min_iou=C.PAIR_MIN_IOU):
    """Greedy one-to-one matching on IoU. Returns list of (ref_idx, odki_key, iou)."""
    cand = [(iou(r, o), i, k) for i, r in enumerate(ref_iv) for k, o in odki_iv.items()]
    cand = sorted([c for c in cand if c[0] >= min_iou], reverse=True)
    used_r, used_o, pairs = set(), set(), []
    for v, i, k in cand:
        if i in used_r or k in used_o:
            continue
        used_r.add(i)
        used_o.add(k)
        pairs.append((i, k, v))
    return pairs


def pair_set(log_path, aligned_path, set_row, align_info=None):
    d = C.read_odki(log_path)
    s = d["s"]
    al = pd.read_csv(aligned_path)
    t, v_ref = al["t_s"].values, al["v_ref"].values
    v_odki = al["v_odki_corrected"].values

    ref_seg = C.segment_ref_concentrics(t, v_ref)
    ref_iv = [(t[a], t[b]) for a, b in ref_seg]
    reps = list(d["r_final"]["rep"].values) if len(d["r_final"]) else []
    odki = C.odki_rep_intervals(s, reps, d["p"])
    odki_iv = {k: (v[0], v[1]) for k, v in odki.items()}
    pairs = match(ref_iv, odki_iv)

    rf = d["r_first"].set_index("rep") if len(d["r_first"]) else None
    rl = d["r_final"].set_index("rep") if len(d["r_final"]) else None
    p = d["p"]
    conc_p = p[(p["phaseType"] > 0) & (p["reported"] == 1)] if len(p) else p

    def ref_fields(i):
        a, b = ref_seg[i]
        return {"ref_rep_idx": i + 1, "t_start_ref_s": t[a], "t_end_ref_s": t[b],
                "ref_mv_std": C.mv_standard(v_ref, a, b), "ref_mv_odkidef": C.mv_odki_definition(v_ref, a, b),
                "ref_pv": C.peak_velocity(v_ref, a, b), "ref_mpv": C.mpv(t, v_ref, a, b),
                "ref_rom_m": C.rom(t, v_ref, a, b)}

    def odki_fields(k):
        t0, t1, i0, i1 = odki[k]
        out = {"odki_rep_number": int(k), "t_start_odki_s": t0, "t_end_odki_s": t1,
               "odki_mv_first": rf.loc[k, "meanVelocity"], "odki_mv_final": rl.loc[k, "meanVelocity"],
               "odki_pv_first": rf.loc[k, "peakVelocity"], "odki_pv_final": rl.loc[k, "peakVelocity"],
               "odki_rom_first": rf.loc[k, "displacementM"], "odki_rom_final": rl.loc[k, "displacementM"],
               "odki_ecc_mv_final": rl.loc[k, "eccMeanVelocity"], "odki_quality1_final": int(rl.loc[k, "quality1"]),
               "odki_status_first": int(rf.loc[k, "correctionStatus"]),
               "odki_status_final": int(rl.loc[k, "correctionStatus"]),
               "odki_mpv_offline": C.mpv(t, v_odki, i0, i1) if np.isfinite(v_odki[i0:i1 + 1]).all() else np.nan}
        cp = conc_p[conc_p["repNumber"] == k] if len(conc_p) else conc_p
        out["close_reason"] = int(cp.iloc[-1]["closeReason"]) if len(cp) else np.nan
        return out

    base = {"set_id": set_row["set_id"], "physical_set_id": set_row["physical_set_id"],
            "unit_id": set_row["unit_id"]}
    flagged = bool(align_info and align_info.get("flagged"))
    rows = []
    paired_r, paired_o = set(), set()
    for i, k, v in pairs:
        rows.append({**base, "match": "TP", "iou": v, **ref_fields(i), **odki_fields(k)})
        paired_r.add(i)
        paired_o.add(k)
    for i in range(len(ref_seg)):
        if i not in paired_r:
            rows.append({**base, "match": "FN", **ref_fields(i)})
    for k in odki:
        if k not in paired_o:
            rows.append({**base, "match": "FP", **odki_fields(k)})
    for r in rows:
        r["exclude_reason"] = "alignment_flagged" if flagged else ""
        key = r.get("ref_rep_idx") if r["match"] != "FP" else f"x{r['odki_rep_number']}"
        r["rep_uid"] = f"{set_row['set_id']}-{key}"
    return rows


def run(data, quiet=False):
    sets = pd.read_csv(os.path.join(data, "sets.csv"))
    rows, al_rows = [], []
    for _, row in sets.iterrows():
        log, _ref, _side = set_paths(data, row)
        apath = os.path.join(data, "aligned", f"{row['set_id']}.csv")
        jpath = os.path.join(data, "aligned", f"{row['set_id']}.json")
        if not os.path.exists(apath):
            print(f"{row['set_id']}: not aligned yet (run align.py), skipped")
            continue
        info = json.load(open(jpath)) if os.path.exists(jpath) else {}
        rs = pair_set(log, apath, row, info)
        rows += rs
        al_rows.append({"set_id": row["set_id"], **{k: info.get(k) for k in
                        ("lag_ms", "drift_ppm", "drift_estimated", "set_duration_s", "xcorr_r", "sync_check_ms", "flagged", "sign", "ref_scale")}})
        if not quiet:
            n = pd.Series([r["match"] for r in rs]).value_counts()
            print(f"{row['set_id']:<28} TP {n.get('TP', 0):>3}  FN {n.get('FN', 0):>3}  FP {n.get('FP', 0):>3}")
    df = pd.DataFrame(rows).reindex(columns=COLUMNS)
    df.to_csv(os.path.join(data, "reps_paired.csv"), index=False, float_format="%.5f")
    pd.DataFrame(al_rows).to_csv(os.path.join(data, "sets_alignment.csv"), index=False, float_format="%.4f")
    return df


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", required=True)
    args = ap.parse_args()
    df = run(args.data)
    print(f"wrote {os.path.join(args.data, 'reps_paired.csv')} ({len(df)} rows)")


if __name__ == "__main__":
    main()
