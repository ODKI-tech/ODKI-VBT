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
VBT ODKI - debug log replay / drift-correction visualizer
===========================================================

Replays a debug-log capture from the firmware (the "S,../R,../B,.." lines
printed over serial between REC_START and REC_STOP) and reconstructs exactly
what the app sees:

  - the RAW curve (velZ), available sample by sample in real time
  - the DRIFT-CORRECTED curve, which only becomes available AFTER each
    bracket closes (the firmware sends it retroactively as "CorrectedCurve"
    over BLE) - here it's reconstructed offline with the same formula used
    by enqueueCorrectedCurveChunks() in MotionTracker.cpp:

        corrected(t) = velZ(t) - baseline - drift * (t - t0) / basisTotal

    using (t0, t1, baseline, drift, basisTotal) from each "B," row.

Modes:

  (default)  static overview of the whole session: both curves overlaid,
             a band for each bracket, every rep boundary marked, and
             markers on the rare flatGuardOverrideFired events.

  --acc      adds worldAccZ and jerk panels to that overview and prints a
             shock-candidate scan (jerk / magnitude of the unfiltered
             worldAccZRaw when the log has it - firmware >= v3.11.28 - else
             of the median-filtered worldAccZ).

  --gui      interactive window (also what you get with no log argument):
             tick any column of the log - raw/corrected velocity,
             accelerations, jerk, gyroscope, flags, phases, rep number - one
             stacked panel per unit, with overlays for phases, bracket
             closes, rep numbers and shock candidates.

  --live     animated playback, paced by the samples' real dt_ms (at 1x
             it's faithful to real device time): the raw curve is drawn
             as it goes, the corrected curve "snaps in" the moment the
             bracket containing it closes - exactly like on the app.

In both modes, the console prints:
  - a recap of every single rep (peak/mean velocity, peak/mean
    acceleration, displacement, eccentric peak/mean, quality,
    correctionStatus, duration)
  - an automatic anomaly scan: count/timestamps of flatGuardOverrideFired,
    and "candidate" windows where the gyroscope is still but NO bracket
    closes - a check independent of the firmware's own flags, meant to
    catch exactly the rare stall this tool was built to investigate (flat,
    but the bracket never arrives) even when the internal counters don't
    flag it.

Usage:
    pip install numpy pandas matplotlib
    python vbt_log_viewer.py                # opens the interactive GUI (pick a log inside it)
    python vbt_log_viewer.py session.log --gui   # the GUI, with this log preloaded
    python vbt_log_viewer.py session.log
    python vbt_log_viewer.py session.log --live --speed 4
    python vbt_log_viewer.py session.log --save overview.png
    python vbt_log_viewer.py session.log --acc            # + acceleration/jerk panels, shock scan

If the file contains multiple REC_START..REC_STOP blocks, by default the
one with the most samples is analyzed (presumably the longest series); use
--session N (0-based, in the order they appear in the file) to pick another.
--list-sessions lists them without plotting anything.
"""

import argparse
import os
import sys

try:
    import numpy as np
    import pandas as pd
except ImportError:
    sys.exit("Requires numpy and pandas: pip install numpy pandas matplotlib")

# ----------------------------------------------------------------------
# Column layout (firmware v3.11.7, see logSampleCsv() / closeBracketFn()
# in MotionTracker.cpp). If the header the firmware prints changes in the
# future, update it here too.
# ----------------------------------------------------------------------

S_COLUMNS = [
    "t_ms", "dt_ms", "state", "rep",
    "linAccX", "linAccY", "linAccZ",
    "worldAccX", "worldAccY", "worldAccZ",
    "velZ", "refVelZ", "velZLive",
    "repCalibRate", "repCalibIntercept", "repCalibCount",
    "worldVelX", "worldVelY", "worldPosZ", "worldPosX", "worldPosY",
    "quatW", "quatX", "quatY", "quatZ",
    "gyroMag", "accZBias",
    "gyroBiasX", "gyroBiasY", "gyroBiasZ",
    "gyroFilteredX", "gyroFilteredY", "gyroFilteredZ",
    "velClamped", "bracketClosed", "confirmedStillNow", "flatGuardOverrideFired",
]

# v3.11.28 appended four pre-median columns at the END of the S row
# (worldAccZRaw, gyroRawX/Y/Z). Rows from older firmware have only the 37
# fields above: both layouts are accepted, the missing columns become NaN.
S_COLUMNS_V2 = S_COLUMNS + ["worldAccZRaw", "gyroRawX", "gyroRawY", "gyroRawZ"]

R_COLUMNS = [
    "rep", "peakVelocity", "meanVelocity", "peakAcceleration", "meanAcceleration",
    "displacementM", "eccPeakVelocity", "eccMeanVelocity", "quality1", "correctionStatus",
]

B_COLUMNS = ["bracketId", "t0", "t1", "baseline", "drift", "rate", "emaRateAfter", "basisTotal"]

STATE_COLORS = {"idle": "#eef1f6", "eccentric": "#ffe3e3", "concentric": "#dff3e3"}


# ----------------------------------------------------------------------
# Parsing
# ----------------------------------------------------------------------

def split_sessions(path):
    """Splits the file on REC_START/REC_STOP. Tolerates unrelated status
    lines (battery, BLE config, etc.) mixed in - they're simply ignored
    since they don't start with S,/R,/B,."""
    sessions = []
    current = []
    with open(path, "r", errors="replace") as f:
        for raw in f:
            line = raw.rstrip("\n")
            stripped = line.strip()
            if stripped == "REC_START":
                if current:
                    sessions.append(current)
                current = []
                continue
            if stripped == "REC_STOP":
                sessions.append(current)
                current = []
                continue
            current.append(line)
    if current:
        sessions.append(current)
    # discard blocks with not even one sample (noise between two sessions,
    # or a file that doesn't use the REC_START/REC_STOP markers)
    sessions = [s for s in sessions if any(l.startswith("S,") for l in s)]
    return sessions


def parse_lines(lines):
    s_rows, r_rows, b_rows = [], [], []
    for line in lines:
        if line.startswith("S,"):
            parts = line.split(",")[1:]
            if len(parts) == len(S_COLUMNS):
                s_rows.append(parts + ["nan"] * (len(S_COLUMNS_V2) - len(S_COLUMNS)))
            elif len(parts) == len(S_COLUMNS_V2):
                s_rows.append(parts)
        elif line.startswith("R,"):
            parts = line.split(",")[1:]
            if len(parts) == len(R_COLUMNS):
                r_rows.append(parts)
        elif line.startswith("B,"):
            parts = line.split(",")[1:]
            if len(parts) == len(B_COLUMNS):
                b_rows.append(parts)

    if not s_rows:
        return None, None, None

    s_df = pd.DataFrame(s_rows, columns=S_COLUMNS_V2)
    numeric_cols = [c for c in S_COLUMNS_V2 if c != "state"]
    for c in numeric_cols:
        s_df[c] = pd.to_numeric(s_df[c], errors="coerce")

    before = len(s_df)
    s_df = s_df.dropna(subset=["t_ms", "dt_ms", "velZ"]).reset_index(drop=True)
    dropped = before - len(s_df)
    if dropped:
        print(f"  (dropped {dropped} malformed/non-numeric S rows)")

    s_df["rep"] = s_df["rep"].astype(int)
    for c in ("velClamped", "bracketClosed", "confirmedStillNow", "flatGuardOverrideFired"):
        s_df[c] = s_df[c].fillna(0).astype(int)

    # The "sessionT" clock used by the B rows (t0/t1) is the time since
    # tracking started, i.e. the cumulative sum of dt_ms sample by sample -
    # NOT t_ms (which is just the raw millis() and has no meaning on its own).
    s_df["sessionT"] = s_df["dt_ms"].cumsum() / 1000.0

    # Note: the header lines the firmware prints once after REC_START
    # ("R,rep,peakVelocity,...", "B,bracketId,t0,t1,...") have the same
    # field count as real data rows, so the column-count check alone
    # doesn't filter them out - they're removed below because they aren't
    # numeric (coerce -> NaN -> dropna).
    r_df = pd.DataFrame(r_rows, columns=R_COLUMNS) if r_rows else pd.DataFrame(columns=R_COLUMNS)
    for c in R_COLUMNS:
        r_df[c] = pd.to_numeric(r_df[c], errors="coerce")
    r_df = r_df.dropna().reset_index(drop=True)
    if len(r_df):
        r_df["rep"] = r_df["rep"].astype(int)
        r_df["correctionStatus"] = r_df["correctionStatus"].astype(int)
        # the firmware re-prints the R row as corrections finalize -> keep
        # only the last one (the most corrected) per rep
        r_df = r_df.sort_index().groupby("rep", as_index=False).last()

    b_df = pd.DataFrame(b_rows, columns=B_COLUMNS) if b_rows else pd.DataFrame(columns=B_COLUMNS)
    for c in B_COLUMNS:
        b_df[c] = pd.to_numeric(b_df[c], errors="coerce")
    b_df = b_df.dropna().reset_index(drop=True)
    if len(b_df):
        b_df["bracketId"] = b_df["bracketId"].astype(int)
        b_df = b_df.sort_values("t0").reset_index(drop=True)

    return s_df, r_df, b_df


def reconstruct_corrected(s_df, b_df):
    """Offline reconstruction of the curve the app receives retroactively
    after each bracket closes. NaN where the bracket hasn't closed yet
    (exactly like on the app, where that stretch stays "raw" until
    CorrectedCurve arrives for it)."""
    n = len(s_df)
    corrected = np.full(n, np.nan)
    bracket_of_sample = np.full(n, -1, dtype=int)
    t = s_df["sessionT"].values
    velz = s_df["velZ"].values

    n_brackets = len(b_df)
    for i in range(n_brackets):
        br = b_df.iloc[i]
        span = br["basisTotal"] if br["basisTotal"] > 1e-9 else (br["t1"] - br["t0"])
        if span <= 1e-9:
            continue
        is_last = (i == n_brackets - 1)
        if is_last:
            mask = (t >= br["t0"] - 1e-6) & (t <= br["t1"] + 1e-6)
        else:
            # half-open interval so the same boundary sample between one
            # bracket and the next doesn't get "claimed" twice
            mask = (t >= br["t0"] - 1e-6) & (t < br["t1"] - 1e-6)
        corrected[mask] = velz[mask] - br["baseline"] - br["drift"] * (t[mask] - br["t0"]) / span
        bracket_of_sample[mask] = int(br["bracketId"])

    s_df = s_df.copy()
    s_df["corrected"] = corrected
    s_df["bracketId"] = bracket_of_sample
    return s_df


# ----------------------------------------------------------------------
# Diagnostics
# ----------------------------------------------------------------------

def scan_anomalies(s_df, gyro_thresh=8.0, min_still_s=1.0):
    """Heuristic scan, independent of the firmware's own flags: flags
    stretches where the gyroscope stays 'quiet' longer than min_still_s
    without a bracketClosed falling inside them. This is a visual aid, not
    a certain diagnosis - verify the candidates by hand."""
    overrides = s_df[s_df["flatGuardOverrideFired"] == 1]
    closes = s_df[s_df["bracketClosed"] == 1]

    still = (s_df["gyroMag"] < gyro_thresh).values
    t = s_df["sessionT"].values
    bracket_closed = s_df["bracketClosed"].values
    stalls = []
    i, n = 0, len(s_df)
    while i < n:
        if not still[i]:
            i += 1
            continue
        j = i
        while j < n and still[j]:
            j += 1
        duration = t[j - 1] - t[i]
        if duration >= min_still_s and not bracket_closed[i:j].any():
            stalls.append((t[i], t[j - 1], duration))
        i = j
    return overrides, closes, stalls


def compute_jerk(s_df, col="worldAccZ"):
    """|d col / dt| in m/s^3, one value per sample (0 for the first)."""
    a = s_df[col].values
    dt = np.maximum(s_df["dt_ms"].values / 1000.0, 1e-3)
    jerk = np.zeros(len(a))
    jerk[1:] = np.abs(np.diff(a)) / dt[1:]
    return jerk


def shock_signal(s_df):
    """Which acceleration the shock scan reads, and its default thresholds.
    Logs from firmware >= v3.11.28 carry worldAccZRaw (before the 5-sample
    median, which erases the few-sample impact spikes - see the v3.11.29 note
    in MotionTracker.cpp): the scan uses that, with thresholds set from a real
    deadlift capture (normal motion peaks at ~880 m/s^3 / ~17 m/s^2, the floor
    impacts at 2100-2800 m/s^3 / 31-43 m/s^2). Older logs only have the
    median-filtered worldAccZ, where shocks are much weaker (thresholds 600 /
    12) and some impacts are invisible."""
    if "worldAccZRaw" in s_df.columns and s_df["worldAccZRaw"].notna().any():
        return "worldAccZRaw", 1500.0, 25.0
    return "worldAccZ", 600.0, 12.0


def scan_shocks(s_df, jerk_thresh=None, acc_thresh=None, merge_s=0.3):
    """Heuristic shock-candidate scan: a sample is a candidate when the jerk of
    the world-Z acceleration exceeds jerk_thresh (m/s^3) or its magnitude
    exceeds acc_thresh (m/s^2). The acceleration is the RAW one (worldAccZRaw)
    whenever the log has it, the median-filtered worldAccZ otherwise (see
    shock_signal()); thresholds left as None take that signal's defaults.
    Candidates closer than merge_s seconds are merged into one event. For each
    event reports the peak values (gyro from the raw gyroscope when logged) and
    how much raw velZ moved from 2 samples before to 10 samples after it (a
    rough proxy for the velocity step the shock injected). A visual aid, not a
    diagnosis. Returns (events, jerk of the signal used, info dict)."""
    col, jerk_default, acc_default = shock_signal(s_df)
    jerk_thresh = jerk_default if jerk_thresh is None else jerk_thresh
    acc_thresh = acc_default if acc_thresh is None else acc_thresh
    jerk = compute_jerk(s_df, col)
    a = s_df[col].values
    t = s_df["sessionT"].values
    velz = s_df["velZ"].values
    if s_df["gyroRawX"].notna().any():
        gyro = np.sqrt(s_df["gyroRawX"].values ** 2 + s_df["gyroRawY"].values ** 2 + s_df["gyroRawZ"].values ** 2)
    else:
        gyro = s_df["gyroMag"].values
    idx = np.where((jerk > jerk_thresh) | (np.abs(a) > acc_thresh))[0]
    events = []
    for i in idx:
        if events and t[i] - t[events[-1][-1]] < merge_s:
            events[-1].append(i)
        else:
            events.append([i])
    out = []
    n = len(s_df)
    for ev in events:
        lo, hi = max(ev[0] - 2, 0), min(ev[-1] + 10, n - 1)
        out.append({
            "t": t[ev[0]], "n": len(ev),
            "maxAcc": float(np.abs(a[lo:hi + 1]).max()),
            "maxJerk": float(jerk[lo:hi + 1].max()),
            "maxGyro": float(gyro[lo:hi + 1].max()),
            "dVel": float(velz[hi] - velz[lo]),
            "rep": int(s_df["rep"].iloc[ev[0]]),
        })
    return out, jerk, {"col": col, "jerk_thresh": jerk_thresh, "acc_thresh": acc_thresh}


def print_shock_report(shocks, info):
    print(f"\n=== Shock candidates ({info['col']}) ===")
    print(f"criteria: jerk > {info['jerk_thresh']:.0f} m/s^3 or |{info['col']}| > {info['acc_thresh']:.0f} m/s^2"
          + ("" if info["col"] == "worldAccZRaw" else "   (no worldAccZRaw in this log: median-filtered signal, weaker)"))
    if not shocks:
        print("   none")
        return
    print(f"{'t(s)':>8} {'rep':>4} {'samples':>8} {'max|acc|':>9} {'maxJerk':>9} {'maxGyro':>8} {'dVelZ':>8}")
    for e in shocks:
        print(f"{e['t']:8.2f} {e['rep']:>4} {e['n']:>8} {e['maxAcc']:>9.1f} {e['maxJerk']:>9.0f} "
              f"{e['maxGyro']:>8.0f} {e['dVel']:>+8.2f}")


def print_anomaly_report(overrides, closes, stalls):
    print("\n=== Anomaly scan ===")
    print(f"flatGuardOverrideFired : {len(overrides)} events"
          + (" (never fired in this session - brackets always closed normally)" if len(overrides) == 0 else ""))
    for _, row in overrides.iterrows():
        print(f"   t={row['sessionT']:8.2f}s  rep={int(row['rep']):>2}  state={row['state']:<10} velZ={row['velZ']:+.3f}")

    print(f"bracketClosed          : {len(closes)} events")

    print("Candidate 'still but no bracket closed' windows:")
    if not stalls:
        print("   none (with default thresholds: gyro<{:.1f} deg/s for >= {:.1f}s)".format(8.0, 1.0))
    else:
        for t0, t1, dur in stalls:
            print(f"   t={t0:8.2f}s -> {t1:8.2f}s  (duration {dur:.2f}s)  <-- verify by hand")


def print_rep_recap(r_df, s_df):
    print("\n=== Rep recap ===")
    if not len(r_df):
        print("   no 'R,' rows found in this session.")
        return
    header = (f"{'rep':>3} {'peakVel':>8} {'meanVel':>8} {'peakAcc':>8} {'meanAcc':>8} "
              f"{'dispM':>7} {'eccPk':>7} {'eccMean':>8} {'qual':>5} {'corrStat':>8} {'dur(s)':>7} {'bracket':>7}")
    print(header)
    print("-" * len(header))
    for _, row in r_df.sort_values("rep").iterrows():
        rep_samples = s_df[s_df["rep"] == row["rep"]]
        if len(rep_samples):
            dur = rep_samples["sessionT"].max() - rep_samples["sessionT"].min()
            brackets = sorted(set(b for b in rep_samples["bracketId"] if b >= 0))
            bracket_str = ",".join(str(b) for b in brackets) if brackets else "-"
        else:
            dur = float("nan")
            bracket_str = "-"
        print(f"{int(row['rep']):>3} {row['peakVelocity']:>8.3f} {row['meanVelocity']:>8.3f} "
              f"{row['peakAcceleration']:>8.3f} {row['meanAcceleration']:>8.3f} {row['displacementM']:>7.3f} "
              f"{row['eccPeakVelocity']:>7.3f} {row['eccMeanVelocity']:>8.3f} {int(row['quality1']):>5} "
              f"{int(row['correctionStatus']):>8} {dur:>7.2f} {bracket_str:>7}")


# ----------------------------------------------------------------------
# Static plot
# ----------------------------------------------------------------------

def plot_static(s_df, r_df, b_df, overrides, stalls, out_path=None, show_ref=False,
                acc=False, jerk=None, shocks=None, jerk_thresh=1500.0):
    import matplotlib.pyplot as plt

    if acc:
        fig, (ax_state, ax_main, ax_acc, ax_jerk, ax_diag) = plt.subplots(
            5, 1, figsize=(16, 12), sharex=True,
            gridspec_kw={"height_ratios": [0.4, 3, 1.5, 1.2, 1]},
        )
    else:
        fig, (ax_state, ax_main, ax_diag) = plt.subplots(
            3, 1, figsize=(16, 9), sharex=True,
            gridspec_kw={"height_ratios": [0.4, 3, 1]},
        )

    t = s_df["sessionT"].values

    # phase strip (idle/eccentric/concentric)
    prev_state, seg_start = None, 0
    for i, st in enumerate(s_df["state"]):
        if st != prev_state:
            if prev_state is not None:
                ax_state.axvspan(t[seg_start], t[i], color=STATE_COLORS.get(prev_state, "#ffffff"), lw=0)
            seg_start, prev_state = i, st
    ax_state.axvspan(t[seg_start], t[-1], color=STATE_COLORS.get(prev_state, "#ffffff"), lw=0)
    ax_state.set_yticks([])
    ax_state.set_ylabel("phase", rotation=0, ha="right", va="center", fontsize=9)

    # bracket bands
    for _, br in b_df.iterrows():
        color = "#dbe9ff" if br["bracketId"] % 2 == 0 else "#eef4ff"
        ax_main.axvspan(br["t0"], br["t1"], color=color, alpha=0.6, lw=0)
        ax_main.axvline(br["t1"], color="#7ba3d0", lw=0.7, ls="--")

    ax_main.plot(t, s_df["velZ"], color="#c9c9c9", lw=0.8, label="raw velZ")
    if show_ref:
        ax_main.plot(t, s_df["refVelZ"], color="#2ca089", lw=0.6, ls=":", label="refVelZ (live re-anchor)")
    ax_main.plot(t, s_df["corrected"], color="#1f4fb4", lw=1.5, label="corrected (offline, per bracket)")
    ax_main.axhline(0, color="#999", lw=0.6)

    if len(overrides):
        ax_main.scatter(overrides["sessionT"], overrides["velZ"], color="red", marker="v", s=70,
                         zorder=5, label="flatGuardOverrideFired")

    ylim = None
    for _, row in r_df.iterrows():
        rep_samples = s_df[s_df["rep"] == row["rep"]]
        if len(rep_samples):
            rep_end_t = rep_samples["sessionT"].max()
            ax_main.axvline(rep_end_t, color="#333333", lw=0.5, alpha=0.3)
            if ylim is None:
                ylim = ax_main.get_ylim()
            ax_main.text(rep_end_t, ylim[1] * 0.92, f"#{int(row['rep'])}", fontsize=7, ha="right", color="#555555")

    for t0, t1, _dur in stalls:
        ax_main.axvspan(t0, t1, color="orange", alpha=0.25)

    ax_main.set_ylabel("velocity (m/s)")
    ax_main.legend(loc="upper right", fontsize=8)

    if acc:
        ax_acc.plot(t, s_df["worldAccZ"], color="#d9480f", lw=0.8, label="worldAccZ (median-filtered)")
        if s_df["worldAccZRaw"].notna().any():
            ax_acc.plot(t, s_df["worldAccZRaw"], color="#1f7a5a", lw=0.7, alpha=0.8, label="worldAccZRaw (pre-median; integrated since v3.11.29)")
        ax_acc.axhline(0, color="#999", lw=0.6)
        ax_acc.set_ylabel("accel Z (m/s^2)")
        ax_acc.legend(loc="upper right", fontsize=8)
        used_col = shock_signal(s_df)[0]
        if used_col == "worldAccZRaw":
            ax_jerk.plot(t, compute_jerk(s_df, "worldAccZ"), color="#b9a8e8", lw=0.6, label="|jerk| of worldAccZ (median-filtered, reference)")
        ax_jerk.plot(t, jerk, color="#6741d9", lw=0.8, label=f"|jerk| of {used_col} (used for the shock scan)")
        ax_jerk.axhline(jerk_thresh, color="red", lw=0.8, ls="--", label=f"shock threshold {jerk_thresh:.0f}")
        ax_jerk.set_ylabel("jerk (m/s^3)")
        ax_jerk.legend(loc="upper right", fontsize=8)
        for e in (shocks or []):
            for ax in (ax_main, ax_acc, ax_jerk):
                ax.axvline(e["t"], color="red", lw=0.8, alpha=0.5)

    ax_diag.plot(t, s_df["gyroMag"], color="#8e44ad", lw=0.7, label="gyroMag (deg/s)")
    ax_diag.axhline(8.0, color="#bbbbbb", lw=0.6, ls=":", label="'still' scan threshold")
    still_rise = s_df.index[(s_df["confirmedStillNow"].diff() == 1)]
    if len(still_rise):
        ax_diag.scatter(t[still_rise], s_df["gyroMag"].values[still_rise], color="green", marker="^", s=30,
                         zorder=5, label="confirmedStillNow")
    ax_diag.set_ylabel("gyroMag")
    ax_diag.set_xlabel("session time (s)")
    ax_diag.legend(loc="upper right", fontsize=8)

    fig.suptitle("VBT session replay - raw vs. drift-corrected velocity")
    fig.tight_layout()
    if out_path:
        fig.savefig(out_path, dpi=150)
        print(f"\nPlot saved to {out_path}")
    else:
        plt.show()


# ----------------------------------------------------------------------
# Animated ("real time") playback
# ----------------------------------------------------------------------

def run_live(s_df, b_df, speed=4.0):
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation

    t = s_df["sessionT"].values
    velz = s_df["velZ"].values
    corrected = s_df["corrected"].values
    n = len(s_df)

    fig, ax = plt.subplots(figsize=(14, 7))
    ax.set_xlim(0, t[-1] * 1.02)
    finite_vals = np.concatenate([velz[np.isfinite(velz)], corrected[np.isfinite(corrected)]])
    pad = 0.3
    ax.set_ylim(finite_vals.min() - pad, finite_vals.max() + pad)
    ax.axhline(0, color="#999999", lw=0.6)
    ax.set_xlabel("session time (s)")
    ax.set_ylabel("velocity (m/s)")
    ax.set_title("Live playback - raw (grey) vs. corrected (blue, appears when the bracket closes)")

    raw_line, = ax.plot([], [], color="#c9c9c9", lw=1.0, label="raw velZ (live)")
    corr_line, = ax.plot([], [], color="#1f4fb4", lw=1.6, label="corrected (retroactive)")
    cursor = ax.axvline(0, color="red", lw=1.0, alpha=0.7)
    override_pts = ax.scatter([], [], color="red", marker="v", s=70, zorder=5, label="flatGuardOverride")
    state_txt = ax.text(0.01, 0.97, "", transform=ax.transAxes, va="top", ha="left",
                         fontsize=10, family="monospace",
                         bbox=dict(boxstyle="round", fc="white", alpha=0.85))
    ax.legend(loc="upper right", fontsize=8)

    cursor_state = {"idx": 0, "shown_brackets": set()}
    target_dt = 0.03 * speed  # seconds of session-time advanced per frame

    def update(_frame):
        idx = cursor_state["idx"]
        if idx >= n - 1:
            return raw_line, corr_line, cursor, state_txt, override_pts

        start_t = t[idx]
        while idx < n - 1 and (t[idx] - start_t) < target_dt:
            idx += 1
        cursor_state["idx"] = idx

        raw_line.set_data(t[: idx + 1], velz[: idx + 1])
        corr_line.set_data(t[: idx + 1], corrected[: idx + 1])
        cursor.set_xdata([t[idx], t[idx]])

        row = s_df.iloc[idx]
        corr_val = row["corrected"]
        corr_str = f"{corr_val:+.3f}" if np.isfinite(corr_val) else "  n/a"
        state_txt.set_text(
            f"t={row['sessionT']:7.2f}s  state={row['state']:<10} rep={int(row['rep']):>2}\n"
            f"velZ={row['velZ']:+.3f}  corrected={corr_str}\n"
            f"gyroMag={row['gyroMag']:5.1f}  stillNow={int(row['confirmedStillNow'])}  "
            f"bracketClose={int(row['bracketClosed'])}  override={int(row['flatGuardOverrideFired'])}"
        )

        newly_closed = b_df[(b_df["t1"] <= row["sessionT"]) & (~b_df["bracketId"].isin(cursor_state["shown_brackets"]))]
        for _, br in newly_closed.iterrows():
            ax.axvspan(br["t0"], br["t1"], color="#dbe9ff", alpha=0.35, lw=0)
            cursor_state["shown_brackets"].add(int(br["bracketId"]))

        visible = s_df.iloc[: idx + 1]
        visible_overrides = visible[visible["flatGuardOverrideFired"] == 1]
        if len(visible_overrides):
            override_pts.set_offsets(np.c_[visible_overrides["sessionT"], visible_overrides["velZ"]])

        return raw_line, corr_line, cursor, state_txt, override_pts

    ani = animation.FuncAnimation(fig, update, interval=30, blit=False, cache_frame_data=False)
    plt.show()
    return ani


# ----------------------------------------------------------------------
# Interactive GUI: pick any column of the log, see it on the graph
# ----------------------------------------------------------------------
# Same look as vbt_live_monitor.py (dark "arena at night" palette, phase
# colors from the phone app). The design is data-driven: SERIES_GROUPS is
# the only place that knows about columns; series with a different unit
# never share a y-axis (one stacked panel per unit group, shared time axis),
# and any numeric column the log has that isn't listed here shows up under
# "Other" automatically, so a future firmware column needs no GUI change.

BG_VOID = "#0A0A0E"
BG_SURFACE = "#16161C"
BG_SURFACE_RAISED = "#1E1E27"
BORDER_SUBTLE = "#2C2C37"
TEXT_PRIMARY = "#F5F5F7"
TEXT_SECONDARY = "#9A9AA5"
ELECTRIC_BLUE = "#2FE0FF"
EMBER_ORANGE = "#FF6A00"
VOLT_LIME = "#CCFF00"
DISCONNECTED_GREY = "#6E6E78"
ALERT_RED = "#FF3B5C"

GUI_STATE_COLOR = {"idle": DISCONNECTED_GREY, "eccentric": EMBER_ORANGE, "concentric": ELECTRIC_BLUE}
SERIES_PALETTE = [ELECTRIC_BLUE, VOLT_LIME, EMBER_ORANGE, "#B388FF", "#FF6EC7", "#3DDC84",
                  "#FFB800", "#7FDBFF", "#F5F5F7", "#9A9AA5"]

# (group title, y-axis label, relative panel height, [(column, label), ...])
SERIES_GROUPS = [
    ("Velocity", "velocity (m/s)", 3.0, [
        ("velZ", "velZ (raw)"), ("corrected", "corrected (per bracket)"), ("velZLive", "velZLive"),
        ("refVelZ", "refVelZ"), ("worldVelX", "worldVelX"), ("worldVelY", "worldVelY")]),
    ("Acceleration", "acceleration (m/s²)", 2.2, [
        ("worldAccZ", "worldAccZ (median-filtered; flat-guard)"), ("worldAccZRaw", "worldAccZ raw (pre-median; integrated since v3.11.29)"),
        ("linAccX", "linAccX"), ("linAccY", "linAccY"), ("linAccZ", "linAccZ"),
        ("worldAccX", "worldAccX"), ("worldAccY", "worldAccY"), ("accZBias", "accZBias")]),
    ("Jerk", "jerk (m/s³)", 1.6, [
        ("jerk", "jerk of worldAccZ"), ("jerkRaw", "jerk of worldAccZ raw")]),
    ("Angular rate", "angular rate (deg/s)", 1.8, [
        ("gyroMag", "gyroMag"), ("gyroRawX", "gyroRawX (pre-median)"), ("gyroRawY", "gyroRawY (pre-median)"),
        ("gyroRawZ", "gyroRawZ (pre-median)"), ("gyroFilteredX", "gyroFilteredX"),
        ("gyroFilteredY", "gyroFilteredY"), ("gyroFilteredZ", "gyroFilteredZ"),
        ("gyroBiasX", "gyroBiasX"), ("gyroBiasY", "gyroBiasY"), ("gyroBiasZ", "gyroBiasZ")]),
    ("Position", "position (m)", 1.6, [
        ("worldPosZ", "worldPosZ (live)"), ("worldPosX", "worldPosX"), ("worldPosY", "worldPosY")]),
    ("Orientation", "quaternion", 1.4, [
        ("quatW", "quatW"), ("quatX", "quatX"), ("quatY", "quatY"), ("quatZ", "quatZ")]),
    ("Rep calibration", "", 1.4, [
        ("repCalibRate", "repCalibRate"), ("repCalibIntercept", "repCalibIntercept"),
        ("repCalibCount", "repCalibCount")]),
    ("Flags", "flag", 0.9, [
        ("velClamped", "velClamped"), ("bracketClosed", "bracketClosed"),
        ("confirmedStillNow", "confirmedStillNow"), ("flatGuardOverrideFired", "flatGuardOverrideFired")]),
    ("Phase", "", 0.8, [
        ("state", "phase (idle / ecc / conc)"), ("rep", "rep number")]),
]
_NON_PLOTTABLE = {"t_ms", "dt_ms", "sessionT", "bracketId"}

PRESETS = {
    "Velocity": ["velZ", "corrected"],
    "Shock analysis": ["velZ", "corrected", "worldAccZ", "worldAccZRaw", "jerk", "gyroMag", "state"],
}


def prepare_gui_frame(s_df, b_df):
    """Adds the derived columns the GUI offers on top of the raw log."""
    s_df = reconstruct_corrected(s_df, b_df)
    s_df["jerk"] = compute_jerk(s_df)
    if s_df["worldAccZRaw"].notna().any():
        s_df["jerkRaw"] = compute_jerk(s_df.fillna({"worldAccZRaw": 0.0}), "worldAccZRaw")
    else:
        s_df["jerkRaw"] = np.nan
    return s_df


def run_gui(initial_path=None, session=None):
    import tkinter as tk
    from tkinter import ttk, filedialog, messagebox
    import matplotlib
    matplotlib.use("TkAgg")
    from matplotlib.figure import Figure
    from matplotlib.patches import Patch
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

    class App:
        def __init__(self, root):
            self.root = root
            root.title("ODKI VBT - log viewer")
            root.geometry("1500x900")
            root.configure(bg=BG_VOID)
            self.sessions, self.path = [], None
            self.s_df = self.r_df = self.b_df = None
            self.shocks = []
            self.vars = {}          # column -> BooleanVar
            self.checks = {}        # column -> Checkbutton
            self.labels = {}        # column -> label
            self.colors = {}        # column -> color
            self._init_style()
            self._build_ui()
            self._register_columns()
            if initial_path:
                self.load(initial_path, session)

        # ---------------- UI ----------------
        def _init_style(self):
            st = ttk.Style(self.root)
            st.theme_use("clam")
            st.configure(".", background=BG_VOID, foreground=TEXT_PRIMARY, fieldbackground=BG_SURFACE,
                         bordercolor=BORDER_SUBTLE, lightcolor=BG_VOID, darkcolor=BG_VOID)
            st.configure("TFrame", background=BG_VOID)
            st.configure("TLabel", background=BG_VOID, foreground=TEXT_PRIMARY)
            st.configure("Dim.TLabel", foreground=TEXT_SECONDARY)
            st.configure("TButton", background=BG_SURFACE_RAISED, foreground=TEXT_PRIMARY, padding=(10, 4))
            st.map("TButton", background=[("active", BORDER_SUBTLE)])
            st.configure("Open.TButton", background=VOLT_LIME, foreground=BG_VOID, font=("Helvetica", 10, "bold"))
            st.map("Open.TButton", background=[("active", "#b8e600")])
            st.configure("TCheckbutton", background=BG_VOID, foreground=TEXT_PRIMARY)
            st.map("TCheckbutton", background=[("active", BG_VOID)], foreground=[("disabled", DISCONNECTED_GREY)])
            st.configure("TLabelframe", background=BG_VOID, bordercolor=BORDER_SUBTLE)
            st.configure("TLabelframe.Label", background=BG_VOID, foreground=ELECTRIC_BLUE, font=("Helvetica", 10, "bold"))
            st.configure("TCombobox", fieldbackground=BG_SURFACE, background=BG_SURFACE_RAISED, foreground=TEXT_PRIMARY)
            st.configure("TSpinbox", fieldbackground=BG_SURFACE, foreground=TEXT_PRIMARY)
            st.configure("Vertical.TScrollbar", background=BG_SURFACE_RAISED, troughcolor=BG_VOID)

        def _build_ui(self):
            top = ttk.Frame(self.root, padding=(8, 8, 8, 4)); top.pack(fill="x")
            ttk.Button(top, text="Open log…", style="Open.TButton", command=self.on_open).pack(side="left")
            self.file_lbl = ttk.Label(top, text="no file loaded", style="Dim.TLabel"); self.file_lbl.pack(side="left", padx=10)
            ttk.Label(top, text="Session").pack(side="left", padx=(20, 4))
            self.session_cb = ttk.Combobox(top, state="readonly", width=34, values=[])
            self.session_cb.pack(side="left"); self.session_cb.bind("<<ComboboxSelected>>", self.on_session)
            ttk.Button(top, text="Clear all", command=lambda: self.apply_preset([])).pack(side="right")
            for name in reversed(list(PRESETS)):
                ttk.Button(top, text=name, command=lambda n=name: self.apply_preset(PRESETS[n])).pack(side="right", padx=(0, 6))
            ttk.Label(top, text="Presets").pack(side="right", padx=(0, 6))

            body = ttk.Frame(self.root); body.pack(fill="both", expand=True)
            left = ttk.Frame(body, width=290); left.pack(side="left", fill="y"); left.pack_propagate(False)
            canvas = tk.Canvas(left, bg=BG_VOID, highlightthickness=0)
            sb = ttk.Scrollbar(left, orient="vertical", command=canvas.yview)
            # the scrollbar must be packed BEFORE the expanding canvas, otherwise the canvas takes
            # all the space and the scrollbar ends up with none (invisible, unreachable)
            sb.pack(side="right", fill="y")
            canvas.pack(side="left", fill="both", expand=True)
            canvas.configure(yscrollcommand=sb.set)
            self.panel = ttk.Frame(canvas)
            win = canvas.create_window((0, 0), window=self.panel, anchor="nw")
            self.panel.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
            canvas.bind("<Configure>", lambda e: canvas.itemconfigure(win, width=e.width))
            self.panel_canvas = canvas

            def wheel(e):
                step = -1 if (getattr(e, "delta", 0) > 0 or getattr(e, "num", 0) == 4) else 1
                canvas.yview_scroll(step * 2, "units")
            # wheel scrolls the list only while the pointer is over it (not over the plot)
            for w in (left, canvas):
                w.bind("<Enter>", lambda e: (self.root.bind_all("<MouseWheel>", wheel),
                                              self.root.bind_all("<Button-4>", wheel),
                                              self.root.bind_all("<Button-5>", wheel)))
                w.bind("<Leave>", lambda e: (self.root.unbind_all("<MouseWheel>"),
                                              self.root.unbind_all("<Button-4>"),
                                              self.root.unbind_all("<Button-5>")))

            right = ttk.Frame(body); right.pack(side="left", fill="both", expand=True)
            self.fig = Figure(figsize=(10, 7), facecolor=BG_VOID)
            self.canvas = FigureCanvasTkAgg(self.fig, master=right)
            self.canvas.get_tk_widget().pack(fill="both", expand=True)
            tb = NavigationToolbar2Tk(self.canvas, right, pack_toolbar=False)
            tb.config(background=BG_SURFACE); [w.config(background=BG_SURFACE) for w in tb.winfo_children() if isinstance(w, (tk.Frame, tk.Label))]
            tb.pack(fill="x"); self.toolbar = tb
            self.readout = ttk.Label(right, text="", style="Dim.TLabel", font=("Menlo", 10)); self.readout.pack(fill="x", padx=8, pady=(2, 6))
            self.canvas.mpl_connect("motion_notify_event", self.on_hover)

        def _register_columns(self, extra=()):
            """(Re)builds the checkbox panel; `extra` = columns of the loaded log not in SERIES_GROUPS."""
            for w in self.panel.winfo_children():
                w.destroy()
            keep = {c: v.get() for c, v in self.vars.items()}
            self.vars, self.checks = {}, {}
            groups = [(g, [(c, l) for c, l in cols]) for g, _u, _h, cols in SERIES_GROUPS]
            if extra:
                groups.append(("Other", [(c, c) for c in extra]))
            color_i = 0
            for gname, cols in groups:
                box = ttk.LabelFrame(self.panel, text=gname, padding=(8, 4)); box.pack(fill="x", padx=6, pady=4)
                for col, label in cols:
                    var = tk.BooleanVar(value=keep.get(col, False))
                    var.trace_add("write", lambda *_: self.schedule_redraw())
                    cb = ttk.Checkbutton(box, text=label, variable=var); cb.pack(anchor="w")
                    self.vars[col], self.checks[col], self.labels[col] = var, cb, label
                    self.colors[col] = SERIES_PALETTE[color_i % len(SERIES_PALETTE)]; color_i += 1
            box = ttk.LabelFrame(self.panel, text="Overlays", padding=(8, 4)); box.pack(fill="x", padx=6, pady=4)
            self.ov = {}
            for key, label, default in (("phases", "Phase shading", True), ("brackets", "Bracket closes", False),
                                         ("reps", "Rep numbers", True), ("shocks", "Shock candidates", False)):
                v = tk.BooleanVar(value=self.ov_state.get(key, default) if hasattr(self, "ov_state") else default)
                v.trace_add("write", lambda *_: self.schedule_redraw())
                ttk.Checkbutton(box, text=label, variable=v).pack(anchor="w"); self.ov[key] = v
            row = ttk.Frame(box); row.pack(anchor="w", pady=(4, 0))
            self.jerk_lbl = ttk.Label(row, text="shock jerk (raw) >", style="Dim.TLabel"); self.jerk_lbl.pack(side="left")
            self.jerk_thr = tk.StringVar(value="1500")
            sp = ttk.Spinbox(row, from_=50, to=10000, increment=100, width=6, textvariable=self.jerk_thr, command=self.on_jerk_thr)
            sp.pack(side="left", padx=4); sp.bind("<Return>", lambda e: self.on_jerk_thr()); ttk.Label(row, text="m/s³", style="Dim.TLabel").pack(side="left")

        # ---------------- data ----------------
        def on_open(self):
            p = filedialog.askopenfilename(title="Open a VBT debug log",
                                           filetypes=[("Log files", "*.log *.txt *.csv"), ("All files", "*.*")])
            if p:
                self.load(p)

        def load(self, path, session=None):
            try:
                sessions = split_sessions(path)
            except OSError as e:
                messagebox.showerror("Cannot open file", str(e)); return
            if not sessions:
                messagebox.showerror("Not a VBT log", "No 'S,' rows found: this doesn't look like a debug-log capture."); return
            self.path, self.sessions = path, sessions
            counts = [sum(1 for l in s if l.startswith("S,")) for s in sessions]
            self.session_cb.configure(values=[f"[{i}] {n} samples" for i, n in enumerate(counts)])
            idx = session if session is not None and 0 <= session < len(sessions) else int(np.argmax(counts))
            self.session_cb.current(idx)
            self.file_lbl.configure(text=os.path.basename(path))
            self.show_session(idx)

        def on_session(self, _e=None):
            self.show_session(self.session_cb.current())

        def show_session(self, idx):
            s_df, r_df, b_df = parse_lines(self.sessions[idx])
            if s_df is None:
                messagebox.showerror("Empty session", "This block contains no valid samples."); return
            self.s_df, self.r_df, self.b_df = prepare_gui_frame(s_df, b_df), r_df, b_df
            known = {c for _g, _u, _h, cols in SERIES_GROUPS for c, _l in cols} | _NON_PLOTTABLE
            extra = [c for c in self.s_df.columns if c not in known and pd.api.types.is_numeric_dtype(self.s_df[c])]
            if hasattr(self, "ov"):
                self.ov_state = {k: v.get() for k, v in self.ov.items()}
            self._register_columns(extra)
            for col, cb in self.checks.items():
                has = col in self.s_df.columns and (col == "state" or self.s_df[col].notna().any())
                cb.state(["!disabled"] if has else ["disabled"])
                if not has:
                    self.vars[col].set(False)
            if not any(v.get() for v in self.vars.values()):
                for c in PRESETS["Velocity"]:
                    self.vars[c].set(True)
            # the shock scan reads the raw acceleration when the log has it: reset the threshold to
            # that signal's default (1500 raw / 600 median-filtered) each time a log is loaded
            col, jerk_default, _acc = shock_signal(self.s_df)
            self.jerk_thr.set(f"{jerk_default:.0f}")
            self.jerk_lbl.configure(text="shock jerk (raw) >" if col == "worldAccZRaw" else "shock jerk (filtered) >")
            self.on_jerk_thr(redraw=False)
            self._xlim = None
            self.redraw()

        def on_jerk_thr(self, redraw=True):
            if self.s_df is None:
                return
            try:
                thr = float(self.jerk_thr.get())
            except ValueError:
                return
            self.shocks, _, _ = scan_shocks(self.s_df, jerk_thresh=thr)
            if redraw:
                self.schedule_redraw()

        def apply_preset(self, cols):
            for c, v in self.vars.items():
                v.set(c in cols and self.checks[c].instate(["!disabled"]))

        # ---------------- plot ----------------
        def schedule_redraw(self):
            if getattr(self, "_pending", None) is None:
                self._pending = self.root.after(30, self._do_redraw)

        def _do_redraw(self):
            self._pending = None
            self.redraw()

        def selected_groups(self):
            out = []
            known = set()
            for gname, ylabel, h, cols in SERIES_GROUPS:
                sel = [(c, l) for c, l in cols if self.vars.get(c) is not None and self.vars[c].get()]
                known |= {c for c, _ in cols}
                if sel:
                    out.append((gname, ylabel, h, sel))
            other = [(c, c) for c in self.vars if c not in known and self.vars[c].get()]
            if other:
                out.append(("Other", "", 1.5, other))
            return out

        def redraw(self):
            if self.s_df is None:
                return
            prev_xlim = self.fig.axes[0].get_xlim() if (self.fig.axes and getattr(self, "_xlim", None) is not None) else None
            self.fig.clear()
            groups = self.selected_groups()
            df, t = self.s_df, self.s_df["sessionT"].values
            if not groups:
                self.fig.text(0.5, 0.5, "Tick one or more columns on the left", color=TEXT_SECONDARY, ha="center", va="center", fontsize=13)
                self.canvas.draw_idle(); return
            axes = self.fig.subplots(len(groups), 1, sharex=True, squeeze=False,
                                      gridspec_kw={"height_ratios": [g[2] for g in groups], "hspace": 0.12})[:, 0]
            self.fig.subplots_adjust(left=0.07, right=0.985, top=0.97, bottom=0.07)
            for ax, (gname, ylabel, _h, sel) in zip(axes, groups):
                ax.set_facecolor(BG_SURFACE)
                for sp in ax.spines.values():
                    sp.set_color(BORDER_SUBTLE)
                ax.tick_params(colors=TEXT_SECONDARY, labelsize=8)
                ax.grid(True, color=BORDER_SUBTLE, lw=0.5, alpha=0.7)
                ax.set_ylabel(ylabel, color=TEXT_SECONDARY, fontsize=9)
                handles = []
                for col, label in sel:
                    if col == "state":
                        self._draw_state_strip(ax, df, t); ax.set_yticks([]); ax.grid(False)
                        handles += [Patch(color=c, alpha=0.8, label=n) for n, c in GUI_STATE_COLOR.items()]
                        continue
                    y = df[col].values.astype(float)
                    if gname == "Flags" or col in ("rep", "repCalibCount"):
                        ax.step(t, y, where="post", color=self.colors[col], lw=1.1, label=label)
                    else:
                        ax.plot(t, y, color=self.colors[col], lw=1.1, label=label)
                    if gname in ("Velocity", "Acceleration", "Angular rate", "Position"):
                        ax.axhline(0, color=DISCONNECTED_GREY, lw=0.6, ls=(0, (4, 3)), alpha=0.6)
                h, l = ax.get_legend_handles_labels()
                if h or handles:
                    leg = ax.legend(handles=h + handles, loc="upper right", fontsize=8, facecolor=BG_SURFACE_RAISED,
                                    edgecolor=BORDER_SUBTLE, labelcolor=TEXT_PRIMARY, ncol=2 if len(h + handles) > 4 else 1)
                    leg.get_frame().set_alpha(0.85)
            axes[-1].set_xlabel("session time (s)", color=TEXT_SECONDARY, fontsize=9)
            self._draw_overlays(axes, df, t)
            if prev_xlim is not None:
                axes[0].set_xlim(prev_xlim)
            else:
                axes[0].set_xlim(t[0], t[-1])
            self._xlim = axes[0].get_xlim()
            for ax in axes:
                ax.callbacks.connect("xlim_changed", lambda a: setattr(self, "_xlim", a.get_xlim()))
            self.toolbar.update()
            self.canvas.draw_idle()

        def _draw_state_strip(self, ax, df, t):
            states = df["state"].values
            start = 0
            for i in range(1, len(states) + 1):
                if i == len(states) or states[i] != states[start]:
                    c = GUI_STATE_COLOR.get(states[start])
                    if c:
                        ax.axvspan(t[start], t[min(i, len(t) - 1)], color=c, alpha=0.75, lw=0)
                    start = i

        def _draw_overlays(self, axes, df, t):
            if self.ov["phases"].get():
                states = df["state"].values
                start = 0
                for i in range(1, len(states) + 1):
                    if i == len(states) or states[i] != states[start]:
                        c = GUI_STATE_COLOR.get(states[start])
                        if c and states[start] != "idle":
                            for ax in axes:
                                ax.axvspan(t[start], t[min(i, len(t) - 1)], color=c, alpha=0.10, lw=0)
                        start = i
            if self.ov["brackets"].get() and self.b_df is not None and len(self.b_df):
                for t1 in self.b_df["t1"].values:
                    for ax in axes:
                        ax.axvline(t1, color="#7ba3d0", lw=0.7, ls="--", alpha=0.45)
            if self.ov["shocks"].get():
                for e in self.shocks:
                    for ax in axes:
                        ax.axvline(e["t"], color=ALERT_RED, lw=1.0, alpha=0.75)
            if self.ov["reps"].get() and len(self.r_df):
                for rep in self.r_df["rep"].values:
                    seg = df[df["rep"] == rep]
                    if len(seg):
                        te = seg["sessionT"].max()
                        axes[0].axvline(te, color=TEXT_SECONDARY, lw=0.5, alpha=0.4)
                        axes[0].text(te, 1.0, f" #{int(rep)}", transform=axes[0].get_xaxis_transform(),
                                     color=TEXT_SECONDARY, fontsize=8, va="bottom", ha="right")

        def on_hover(self, ev):
            if self.s_df is None or ev.inaxes is None or ev.xdata is None:
                return
            t = self.s_df["sessionT"].values
            i = int(np.clip(np.searchsorted(t, ev.xdata), 0, len(t) - 1))
            row = self.s_df.iloc[i]
            parts = [f"t={t[i]:7.3f}s", f"{row['state']:<10}", f"rep {int(row['rep'])}"]
            for col, var in self.vars.items():
                if var.get() and col != "state" and col != "rep":
                    v = row[col]
                    parts.append(f"{col}={v:+.3f}" if np.isfinite(v) else f"{col}=n/a")
            self.readout.configure(text="   ".join(parts[:10]))

    root = tk.Tk()
    root.app = App(root)
    root.mainloop()


# ----------------------------------------------------------------------
# main
# ----------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logfile", nargs="?", help="log file to analyze (omit to open the GUI and pick one there)")
    ap.add_argument("--gui", action="store_true", help="interactive GUI: tick any columns, phases and overlays (default when no logfile is given)")
    ap.add_argument("--live", action="store_true", help="animated playback instead of the static overview")
    ap.add_argument("--speed", type=float, default=4.0, help="live playback speed (default 4x)")
    ap.add_argument("--gyro-thresh", type=float, default=8.0, help="deg/s below which a sample counts as 'still' for the anomaly scan")
    ap.add_argument("--min-still", type=float, default=1.0, help="seconds of gyroscope stillness before flagging a window")
    ap.add_argument("--save", help="save the static overview to a file instead of opening it (implies not --live)")
    ap.add_argument("--show-ref", action="store_true", help="also show refVelZ (live re-anchor) in the static overview")
    ap.add_argument("--acc", action="store_true", help="add worldAccZ and jerk panels to the static overview, and print a shock-candidate scan")
    ap.add_argument("--jerk-thresh", type=float, default=None, help="jerk threshold in m/s^3 for the shock scan (default: 1500 on the raw acceleration of firmware >= v3.11.28 logs, 600 on the median-filtered one of older logs)")
    ap.add_argument("--session", type=int, default=None, help="0-based index of the REC_START..REC_STOP block to analyze")
    ap.add_argument("--list-sessions", action="store_true", help="list the blocks found in the file and exit")
    args = ap.parse_args()

    if args.gui or not args.logfile:
        run_gui(args.logfile, args.session)
        return

    sessions = split_sessions(args.logfile)
    if not sessions:
        sys.exit("No 'S,' rows found in the file: doesn't look like a valid debug-log capture.")

    if args.list_sessions or len(sessions) > 1:
        print(f"Found {len(sessions)} REC_START..REC_STOP blocks:")
        for i, lines in enumerate(sessions):
            n_s = sum(1 for l in lines if l.startswith("S,"))
            n_r = sum(1 for l in lines if l.startswith("R,"))
            n_b = sum(1 for l in lines if l.startswith("B,"))
            print(f"  [{i}] {n_s} samples, {n_r} rep rows, {n_b} brackets")
        if args.list_sessions:
            return

    if args.session is not None:
        if not (0 <= args.session < len(sessions)):
            sys.exit(f"--session {args.session} out of range (0..{len(sessions) - 1})")
        chosen = sessions[args.session]
    else:
        chosen = max(sessions, key=lambda lines: sum(1 for l in lines if l.startswith("S,")))

    s_df, r_df, b_df = parse_lines(chosen)
    if s_df is None:
        sys.exit("The chosen block contains no valid samples.")
    s_df = reconstruct_corrected(s_df, b_df)

    print(f"Session analyzed: {len(s_df)} samples, {len(r_df)} reps, {len(b_df)} brackets "
          f"({s_df['sessionT'].iloc[-1]:.1f}s total).")

    overrides, closes, stalls = scan_anomalies(s_df, args.gyro_thresh, args.min_still)
    print_anomaly_report(overrides, closes, stalls)
    print_rep_recap(r_df, s_df)

    shocks, jerk, shock_info = None, None, None
    if args.acc:
        shocks, jerk, shock_info = scan_shocks(s_df, jerk_thresh=args.jerk_thresh)
        print_shock_report(shocks, shock_info)

    if args.live:
        if args.acc:
            print("(--acc only affects the static overview; ignored with --live)")
        run_live(s_df, b_df, speed=args.speed)
    else:
        plot_static(s_df, r_df, b_df, overrides, stalls, out_path=args.save, show_ref=args.show_ref,
                    acc=args.acc, jerk=jerk, shocks=shocks,
                    jerk_thresh=shock_info["jerk_thresh"] if shock_info else 1500.0)


if __name__ == "__main__":
    main()
