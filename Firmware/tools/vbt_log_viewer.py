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

Two modes:

  (default)  static overview of the whole session: both curves overlaid,
             a band for each bracket, every rep boundary marked, and
             markers on the rare flatGuardOverrideFired events.

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
    python vbt_log_viewer.py session.log
    python vbt_log_viewer.py session.log --live --speed 4
    python vbt_log_viewer.py session.log --save overview.png

If the file contains multiple REC_START..REC_STOP blocks, by default the
one with the most samples is analyzed (presumably the longest series); use
--session N (0-based, in the order they appear in the file) to pick another.
--list-sessions lists them without plotting anything.
"""

import argparse
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

    s_df = pd.DataFrame(s_rows, columns=S_COLUMNS)
    numeric_cols = [c for c in S_COLUMNS if c != "state"]
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

def plot_static(s_df, r_df, b_df, overrides, stalls, out_path=None, show_ref=False):
    import matplotlib.pyplot as plt

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
# main
# ----------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logfile")
    ap.add_argument("--live", action="store_true", help="animated playback instead of the static overview")
    ap.add_argument("--speed", type=float, default=4.0, help="live playback speed (default 4x)")
    ap.add_argument("--gyro-thresh", type=float, default=8.0, help="deg/s below which a sample counts as 'still' for the anomaly scan")
    ap.add_argument("--min-still", type=float, default=1.0, help="seconds of gyroscope stillness before flagging a window")
    ap.add_argument("--save", help="save the static overview to a file instead of opening it (implies not --live)")
    ap.add_argument("--show-ref", action="store_true", help="also show refVelZ (live re-anchor) in the static overview")
    ap.add_argument("--session", type=int, default=None, help="0-based index of the REC_START..REC_STOP block to analyze")
    ap.add_argument("--list-sessions", action="store_true", help="list the blocks found in the file and exit")
    args = ap.parse_args()

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

    if args.live:
        run_live(s_df, b_df, speed=args.speed)
    else:
        plot_static(s_df, r_df, b_df, overrides, stalls, out_path=args.save, show_ref=args.show_ref)


if __name__ == "__main__":
    main()
