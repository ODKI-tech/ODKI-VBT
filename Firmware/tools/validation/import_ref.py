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
Converts the reference linear encoder's own export into the dataset format
(PROTOCOL.md sections 5 and 9):

  raw/ref/<physical_set_id>.csv      t_s,position_m

t_s is the encoder's own clock in seconds (relative to the start of its
recording with --zero-time); position_m is the bar position in metres. If
the export only has velocity, it is integrated with the trapezoidal rule
(position 0 at the first sample). The scale factor from the static
verification is NOT applied here: align.py applies it, so the raw file stays
what the instrument recorded.

Columns are given by header name or by 0-based index.

Usage:
    python import_ref.py EXPORT.csv --data DATA_DIR --set L01-D1-SQ-060-S1 \
        --time-col time --time-unit ms --pos-col position --pos-unit mm
    python import_ref.py EXPORT.csv --out ref.csv \
        --time-col 0 --time-unit s --vel-col 2 --delimiter ";" --decimal "," --zero-time
"""

import argparse
import os
import sys

import numpy as np
import pandas as pd

TIME_UNITS = {"s": 1.0, "ms": 1e-3, "us": 1e-6}
POS_UNITS = {"m": 1.0, "cm": 1e-2, "mm": 1e-3}


def _column(df, spec):
    if spec in df.columns:
        return df[spec]
    try:
        return df.iloc[:, int(spec)]
    except (ValueError, IndexError):
        raise SystemExit(f"column {spec!r} not found; columns are {list(df.columns)}")


def convert(src, time_col, time_unit="s", pos_col=None, pos_unit="m", vel_col=None,
            delimiter=",", decimal=".", zero_time=False, skiprows=0):
    """Returns a DataFrame t_s, position_m (sorted, duplicate times dropped)."""
    if (pos_col is None) == (vel_col is None):
        raise SystemExit("give exactly one of --pos-col or --vel-col")
    df = pd.read_csv(src, sep=delimiter, decimal=decimal, skiprows=skiprows, engine="python")
    df.columns = [str(c).strip() for c in df.columns]
    t = pd.to_numeric(_column(df, time_col), errors="coerce").values * TIME_UNITS[time_unit]
    if pos_col is not None:
        y = pd.to_numeric(_column(df, pos_col), errors="coerce").values * POS_UNITS[pos_unit]
    else:
        y = pd.to_numeric(_column(df, vel_col), errors="coerce").values
    m = np.isfinite(t) & np.isfinite(y)
    out = pd.DataFrame({"t_s": t[m], "y": y[m]}).sort_values("t_s").drop_duplicates("t_s")
    t, y = out["t_s"].values, out["y"].values
    if len(t) < 2:
        raise SystemExit(f"{src}: fewer than 2 valid samples")
    if vel_col is not None:
        y = np.r_[0.0, np.cumsum(0.5 * (y[1:] + y[:-1]) * np.diff(t))]
    if zero_time:
        t = t - t[0]
    return pd.DataFrame({"t_s": t, "position_m": y})


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("export", help="CSV exported by the linear encoder's software")
    dest = ap.add_mutually_exclusive_group(required=True)
    dest.add_argument("--out", help="output CSV path")
    dest.add_argument("--data", help="dataset directory: writes raw/ref/<set>.csv (needs --set)")
    ap.add_argument("--set", help="physical set id (with --data)")
    ap.add_argument("--time-col", required=True, help="time column (name or 0-based index)")
    ap.add_argument("--time-unit", choices=sorted(TIME_UNITS), default="s")
    ap.add_argument("--pos-col", help="position column (name or index)")
    ap.add_argument("--pos-unit", choices=sorted(POS_UNITS), default="m")
    ap.add_argument("--vel-col", help="velocity column in m/s (name or index), integrated to position")
    ap.add_argument("--delimiter", default=",", help="field separator (default ',')")
    ap.add_argument("--decimal", default=".", choices=[".", ","], help="decimal separator (default '.')")
    ap.add_argument("--zero-time", action="store_true", help="shift time so the first sample is t = 0")
    ap.add_argument("--skiprows", type=int, default=0, help="lines to skip before the header row")
    args = ap.parse_args()
    if args.data and not args.set:
        sys.exit("--data needs --set")
    out = args.out or os.path.join(args.data, "raw", "ref", f"{args.set}.csv")
    if os.path.exists(out):
        sys.exit(f"{out} already exists - one file per set")
    df = convert(args.export, args.time_col, args.time_unit, args.pos_col, args.pos_unit, args.vel_col,
                 args.delimiter, args.decimal, args.zero_time, args.skiprows)
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    df.to_csv(out, index=False, float_format="%.6f")
    dt = np.median(np.diff(df["t_s"].values))
    print(f"wrote {out}: {len(df)} samples, {df['t_s'].iloc[-1] - df['t_s'].iloc[0]:.2f} s, "
          f"median rate {1.0 / dt:.0f} Hz, position range {df['position_m'].min():.3f} to "
          f"{df['position_m'].max():.3f} m")


if __name__ == "__main__":
    main()
