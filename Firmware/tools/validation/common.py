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
Shared helpers for the validation pipeline (see PROTOCOL.md).

Everything that has to be computed the same way by several scripts lives
here: reading an ODKI serial log (reusing the parser of vbt_log_viewer.py),
reading a reference (linear encoder) recording, the per-rep metric definitions
(MV, MPV, PV, ROM) and the PRE-REGISTERED constants. Changing a constant in
the PRE-REGISTERED block after data collection has started is a protocol
deviation and must be reported as such (PROTOCOL.md, section 14).
"""

import importlib.util
import os

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)

# ----------------------------------------------------------------------
# PRE-REGISTERED constants (PROTOCOL.md sections 5-8). Do not edit after
# the first data-collection session without logging a deviation.
# ----------------------------------------------------------------------

G = 9.80665                       # m/s^2, standard gravity (MPV definition)
GRID_HZ = 100.0                   # analysis rate (ODKI native rate)
REF_WINDOW_S = 0.010              # reference velocity = displacement over a centred 10 ms window
ODKI_PHASE_START_MPS = 0.20       # firmware default phaseStartVelocityMps (MV "ODKI definition")
ODKI_LOOKBACK_SAMPLES = 5         # firmware default phaseLookbackSamples
LOOKBACK_FLOOR_MPS = 0.05         # back-filled samples must reach this (see mv_odki_definition)
REF_MIN_ROM_M = 0.10              # a reference concentric must rise >= 10 cm
REF_MERGE_GAP_S = 0.05            # positive-velocity runs closer than this are one concentric
PAIR_MIN_IOU = 0.50               # rep pairing: temporal intersection-over-union
LAG_SEARCH_S = 0.300              # fine alignment search range around the coarse estimate
MIN_DRIFT_SET_S = 25.0            # clock drift is estimated only on sets at least this long
SYNC_WINDOW_S = 8.0               # the sync drop is looked for in the first N s of a set, before the first lift
SYNC_TOLERANCE_MS = 10.0          # |sync check| above this flags the set
VELOCITY_BANDS = [(0.0, 0.5, "<0.5"), (0.5, 1.0, "0.5-1.0"), (1.0, 99.0, ">1.0")]  # m/s, on reference MV
DEFAULT_REF_SCALE = 1.0           # reference position scale; replaced by the static verification (section 4)

# Acceptance thresholds (PROTOCOL.md section 8). Mean velocity, final value,
# pooled squat + bench press. "source" is what the number is taken from.
THRESHOLDS = {
    "tier_A": {
        "label": "comparable to published results for commercial IMU sensors",
        "criteria": {
            "pearson_r_min": 0.95,
            "sem_max_mps": 0.06,
            "abs_bias_max_mps": 0.05,
            "cv_pct_max": 10.0,
            "loa_halfwidth_max_mps": 0.12,
            "icc_2_1_min": 0.90,
        },
        "source": "Achermann et al. 2023 (IMU vs 3D mocap, MV SEE 0.059 m/s); Weakley et al. 2021 (CV < 10%); Koo & Li 2016 (ICC > 0.90 excellent)",
    },
    "tier_B": {
        "label": "minimum acceptable in the literature",
        "criteria": {
            "pearson_r_min": 0.70,
            "cv_pct_max": 10.0,
            "std_bias_max": 0.60,
        },
        "source": "Weakley et al. 2021 (r > 0.70, CV < 10%, standardised mean bias < 0.6)",
    },
    "internal_target": {
        "label": "ODKI internal target (not derived from a published source)",
        "criteria": {
            "abs_bias_max_mps": 0.03,
            "loa_halfwidth_max_mps": 0.10,
            "cv_pct_max": 5.0,
            "icc_2_1_min": 0.90,
        },
        "source": "ODKI, declared in advance",
    },
    "rep_detection": {
        "label": "rep detection (squat and bench press)",
        "criteria": {"recall_min": 0.98, "false_rep_rate_max": 0.02},
        "source": "ODKI internal target, declared in advance",
    },
}

# ----------------------------------------------------------------------
# ODKI serial log
# ----------------------------------------------------------------------

R_COLUMNS = ["rep", "peakVelocity", "meanVelocity", "peakAcceleration", "meanAcceleration",
             "displacementM", "eccPeakVelocity", "eccMeanVelocity", "quality1", "correctionStatus"]
P_COLUMNS = ["phaseId", "repNumber", "phaseType", "closeReason", "durationS",
             "sampleCount", "discarded", "reported"]
# firmware >= 3.11.32 appends the device millis of the phase's first and last kept sample
P_COLUMNS_V2 = P_COLUMNS + ["t_start_ms", "t_end_ms"]
S_FIELDS_V2 = 41  # fields after "S," up to gyroRawZ (firmware 3.11.28-3.11.31)

_viewer = None


def load_viewer():
    """Imports ../vbt_log_viewer.py (same mechanism as host_replay/replay.py)."""
    global _viewer
    if _viewer is None:
        spec = importlib.util.spec_from_file_location("vbt_log_viewer", os.path.join(TOOLS, "vbt_log_viewer.py"))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        _viewer = mod
    return _viewer


def _numeric_rows(lines, prefix, columns, short_columns=None):
    """Rows with exactly len(columns) fields, or len(short_columns) fields
    (older firmware: the missing trailing columns become NaN)."""
    rows = []
    n_short = len(short_columns) if short_columns else None
    for line in lines:
        if line.startswith(prefix):
            parts = line.strip().split(",")[1:]
            if len(parts) == len(columns):
                rows.append(parts)
            elif n_short is not None and len(parts) == n_short:
                rows.append(parts + ["nan"] * (len(columns) - n_short))
    df = pd.DataFrame(rows, columns=columns)
    for c in columns:
        df[c] = pd.to_numeric(df[c], errors="coerce")
    return df.dropna(subset=columns[:n_short] if n_short else columns).reset_index(drop=True)  # drops the header row


def select_session(path, session=None):
    """Lines of one REC_START..REC_STOP block: the longest one by default,
    exactly like vbt_log_viewer.py."""
    v = load_viewer()
    sessions = v.split_sessions(path)
    if not sessions:
        raise ValueError(f"{path}: no S, rows found")
    if session is None:
        return max(sessions, key=lambda l: sum(1 for x in l if x.startswith("S,")))
    return sessions[session]


def read_odki(path, session=None):
    """Returns a dict with
       s     : S rows + 'sessionT' + 'corrected' (vbt_log_viewer.reconstruct_corrected)
       r_first / r_final : first and last R row of every rep (first reported vs authoritative value)
       b     : B rows
       p     : P rows (empty for firmware < 3.11.18)"""
    v = load_viewer()
    lines = select_session(path, session)
    if not hasattr(v, "S_COLUMNS_V3"):
        # viewer older than firmware 3.11.32: trim the appended accRawX/Y/Z
        # (unused here) so the 44-field rows are not dropped
        lines = [",".join(l.split(",")[:S_FIELDS_V2 + 1]) if l.startswith("S,") and l.count(",") > S_FIELDS_V2 else l
                 for l in lines]
    s_df, _r_last, b_df = v.parse_lines(lines)
    s_df = v.reconstruct_corrected(s_df, b_df)
    r_all = _numeric_rows(lines, "R,", R_COLUMNS)
    if len(r_all):
        r_all["rep"] = r_all["rep"].astype(int)
        r_all["correctionStatus"] = r_all["correctionStatus"].astype(int)
    r_first = r_all.groupby("rep", as_index=False).first() if len(r_all) else r_all
    r_final = r_all.groupby("rep", as_index=False).last() if len(r_all) else r_all
    p_df = _numeric_rows(lines, "P,", P_COLUMNS_V2, P_COLUMNS)
    return {"s": s_df, "r_first": r_first, "r_final": r_final, "b": b_df, "p": p_df}


def odki_device_time(s_df):
    """Device clock of each sample in seconds: the first t_ms (millis) plus
    the cumulative dt_ms (sessionT), which is not quantised to 1 ms."""
    st = s_df["sessionT"].values
    return s_df["t_ms"].values[0] / 1000.0 + (st - st[0])


def odki_rep_intervals(s_df, rep_numbers, p_df=None):
    """Concentric interval (sessionT start, end, start index, end index) of
    every reported rep, rebuilt from the S rows: the S 'rep' column holds the
    number the NEXT concentric close will report, and 'state' is the sign of
    the live corrected velocity. Among the concentric runs carrying rep == k,
    those rising at least half as much as the largest one are real
    concentrics, and rep k is the LAST of them: its close is what made the
    counter move on (an earlier one carrying the same number is a concentric
    the firmware missed). Short noise runs around the turnaround are ignored.
    Firmware >= 3.11.32 logs the exact interval in the P row, used instead."""
    t = s_df["sessionT"].values
    rep = s_df["rep"].values
    conc = (s_df["state"].values == "concentric")
    vel = s_df["corrected"].values
    vel = np.where(np.isfinite(vel), vel, s_df["velZLive"].values)
    dt = np.r_[0.01, np.diff(t)]
    out = {}
    # firmware >= 3.11.32: the P row of the reported concentric carries its exact start/end
    if p_df is not None and len(p_df) and p_df["t_start_ms"].notna().any():
        tdev_ms = odki_device_time(s_df) * 1000.0
        rows = p_df[(p_df["phaseType"] > 0) & (p_df["reported"] == 1)]
        for k in rep_numbers:
            rk = rows[rows["repNumber"] == k]
            if not len(rk):
                continue
            last = rk.iloc[-1]
            i0 = int(np.argmin(np.abs(tdev_ms - last["t_start_ms"])))
            i1 = int(np.argmin(np.abs(tdev_ms - last["t_end_ms"])))
            out[int(k)] = (float(t[i0]), float(t[i1]), i0, i1)
    for k in rep_numbers:
        if int(k) in out:
            continue
        idx = np.where((rep == k) & conc)[0]
        if not len(idx):
            continue
        runs = np.split(idx, np.where(np.diff(idx) > 1)[0] + 1)
        disp = [float(np.sum(vel[r] * dt[r])) for r in runs]
        big = [r for r, dd in zip(runs, disp) if dd >= 0.5 * max(disp)]
        best = big[-1]
        out[int(k)] = (float(t[best[0]]), float(t[best[-1]]), int(best[0]), int(best[-1]))
    return out


# ----------------------------------------------------------------------
# Reference (linear encoder)
# ----------------------------------------------------------------------

def read_ref(path):
    """Reference recording, raw/ref/<physical_set_id>.csv as written by
    import_ref.py: t_s (the encoder's own clock, relative to its start),
    position_m."""
    df = pd.read_csv(path)
    df = df[["t_s", "position_m"]].apply(pd.to_numeric, errors="coerce").dropna()
    return df.sort_values("t_s").drop_duplicates("t_s").reset_index(drop=True)


def ref_position(ref_df, scale=DEFAULT_REF_SCALE, sign=1.0):
    """(t_s on the encoder clock, position in m). scale is the factor from the
    static verification (rule length / encoder reading); sign flips the
    direction if the encoder reads upward motion as negative."""
    t = ref_df["t_s"].values.astype(float)
    x = sign * scale * ref_df["position_m"].values.astype(float)
    return t, x


def ref_velocity_at(t_e, x, query_t_e, window=REF_WINDOW_S):
    """Mean velocity over a centred window (displacement / window) at the
    requested reference-clock times. NaN outside the capture."""
    lo = np.interp(query_t_e - window / 2, t_e, x, left=np.nan, right=np.nan)
    hi = np.interp(query_t_e + window / 2, t_e, x, left=np.nan, right=np.nan)
    return (hi - lo) / window


# ----------------------------------------------------------------------
# Per-rep metric definitions (PROTOCOL.md section 6)
# ----------------------------------------------------------------------

def segment_ref_concentrics(t, v, min_rom=REF_MIN_ROM_M, merge_gap=REF_MERGE_GAP_S):
    """Reference concentrics: maximal runs of v > 0, runs separated by less
    than merge_gap merged, kept if the upward displacement is >= min_rom.
    Returns a list of (start index, end index), inclusive."""
    v = np.where(np.isfinite(v), v, 0.0)
    pos = np.where(v > 0)[0]
    if not len(pos):
        return []
    runs = np.split(pos, np.where(np.diff(pos) > 1)[0] + 1)
    merged = [[runs[0][0], runs[0][-1]]]
    for r in runs[1:]:
        if t[r[0]] - t[merged[-1][1]] < merge_gap:
            merged[-1][1] = r[-1]
        else:
            merged.append([r[0], r[-1]])
    out = []
    for a, b in merged:
        if rom(t, v, a, b) >= min_rom:
            out.append((int(a), int(b)))
    return out


def _dt(t, a, b):
    seg = t[a:b + 1]
    d = np.diff(seg)
    return np.r_[d[0] if len(d) else 0.01, d]


def rom(t, v, a, b):
    vv = np.clip(np.nan_to_num(v[a:b + 1]), 0, None)
    return float(np.sum(vv * _dt(t, a, b)))


def mv_standard(v, a, b):
    """Mean concentric velocity, standard LPT definition: every sample from
    the start of upward motion (v > 0) to its end."""
    seg = v[a:b + 1]
    return float(np.nanmean(seg[seg > 0])) if np.any(seg > 0) else np.nan


def mv_odki_definition(v, a, b, start=ODKI_PHASE_START_MPS, lookback=ODKI_LOOKBACK_SAMPLES,
                       floor=LOOKBACK_FLOOR_MPS):
    """Mean velocity as the firmware defines it (scorePhase() in
    MotionTracker.cpp): the phase opens at the first sample >= 0.20 m/s,
    back-filled by up to 5 earlier same-direction samples, and runs to the
    last same-direction sample. Approximation, declared in PROTOCOL.md: the
    firmware back-fills on the SIGN of the raw (drifting) velocity, which no
    reference can reproduce, and a near-zero sample more or less moves a
    mean by up to a few cm/s; here the back-filled samples and the tail into
    the turnaround must reach LOOKBACK_FLOOR_MPS, which makes the value
    independent of the sampling phase and of the reference's quantisation
    near zero velocity."""
    seg = v[a:b + 1]
    above = np.where(seg >= start)[0]
    if not len(above):
        return np.nan
    i0 = above[0]
    j = i0
    while j > 0 and i0 - j < lookback and seg[j - 1] >= floor:
        j -= 1
    k = len(seg) - 1
    while k > i0 and not seg[k] >= floor:   # same floor on the tail into the turnaround
        k -= 1
    s = seg[j:k + 1]
    return float(np.nanmean(s[s > 0]))


def peak_velocity(v, a, b):
    return float(np.nanmax(v[a:b + 1]))


def mpv(t, v, a, b):
    """Mean propulsive velocity (Sanchez-Medina et al. 2010): mean velocity
    from the start of the concentric to the first sample after peak velocity
    where the bar's acceleration falls below -g."""
    seg = v[a:b + 1]
    ts = t[a:b + 1]
    if len(seg) < 3:
        return mv_standard(v, a, b)
    acc = np.gradient(seg, ts)
    k = int(np.nanargmax(seg))
    after = np.where(acc[k:] < -G)[0]
    end = k + after[0] if len(after) else len(seg)
    s = seg[:end]
    return float(np.nanmean(s[s > 0])) if np.any(s > 0) else np.nan


def velocity_band(mv):
    for lo, hi, name in VELOCITY_BANDS:
        if lo <= mv < hi:
            return name
    return "n/a"
