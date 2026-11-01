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
Reading a debug log and reconstructing, sample by sample, the IMU input of
the orientation filter (see "Orientation filter selection" in
Firmware/DOCUMENTATION.md for the derivation).

Log layouts accepted (fields after "S,"):
  37  firmware <= 3.11.27       no pre-median taps
  41  firmware 3.11.28-3.11.31  + worldAccZRaw, gyroRawX/Y/Z
  44  firmware 3.11.32          + accRawX/Y/Z (pre-median body-frame accelerometer)
  48  branch fw-rawacc-log      + accelScaleG, accBiasX/Y/Z (state the replay needs)

The first 41 fields are parsed by vbt_log_viewer.py (imported, never copied);
the trailing ones here.
"""

import importlib.util
import os
import re

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
G = 9.80665
N_V2 = 41
EXTRA_V3 = ["accRawX", "accRawY", "accRawZ"]
EXTRA_V4 = EXTRA_V3 + ["accelScaleG", "accBiasX", "accBiasY", "accBiasZ"]
P_COLUMNS = ["phaseId", "repNumber", "phaseType", "closeReason", "durationS",
             "sampleCount", "discarded", "reported"]
P_COLUMNS_V2 = P_COLUMNS + ["t_start_ms", "t_end_ms"]

_mods = {}


def load_module(name, path):
    if name not in _mods:
        spec = importlib.util.spec_from_file_location(name, path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        _mods[name] = mod
    return _mods[name]


def viewer():
    return load_module("vbt_log_viewer", os.path.join(TOOLS, "vbt_log_viewer.py"))


def validation_common():
    """tools/validation/common.py (rep intervals, metric definitions), if present."""
    p = os.path.join(TOOLS, "validation", "common.py")
    return load_module("validation_common", p) if os.path.exists(p) else None


def grav_body(q):
    """Gravity direction in the body frame for body->world quaternions (N x 4,
    w first) - the same expression as updateOrientationAndAcceleration()."""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.c_[2 * (x * z - w * y), 2 * (w * x + y * z), w * w - x * x - y * y + z * z]


def angle_deg(u, v):
    """Angle between row vectors (deg)."""
    nu = np.linalg.norm(u, axis=1)
    nv = np.linalg.norm(v, axis=1)
    c = np.clip(np.sum(u * v, axis=1) / np.maximum(nu * nv, 1e-12), -1, 1)
    return np.degrees(np.arccos(c))


def median5(x):
    """MedianFilter3 of the firmware on one axis (primed with the first sample)."""
    buf = np.full(5, x[0])
    out = np.empty_like(x)
    for i, v in enumerate(x):
        buf[i % 5] = v
        out[i] = np.median(buf)
    return out


def parse_p(lines):
    rows = []
    for line in lines:
        if line.startswith("P,"):
            parts = line.strip().split(",")[1:]
            if len(parts) in (len(P_COLUMNS), len(P_COLUMNS_V2)):
                rows.append(parts + ["nan"] * (len(P_COLUMNS_V2) - len(parts)))
    df = pd.DataFrame(rows, columns=P_COLUMNS_V2).apply(pd.to_numeric, errors="coerce")
    return df.dropna(subset=P_COLUMNS).reset_index(drop=True)


def parse_session(lines):
    """S/R/B/P frames of one REC_START..REC_STOP block, with the trailing
    columns of newer layouts added to the S frame."""
    v = viewer()
    trimmed, extras = [], []
    for line in lines:
        if line.startswith("S,"):
            parts = line.strip().split(",")
            n = len(parts) - 1
            if n > N_V2:
                extras.append(parts[N_V2 + 1:])
                line = ",".join(parts[:N_V2 + 1])
            elif n in (37, N_V2):
                extras.append([])
            else:
                continue
        trimmed.append(line)
    s_df, r_df, b_df = v.parse_lines(trimmed)
    if s_df is None:
        return None
    # parse_lines drops the header row (non-numeric) and malformed rows: keep
    # the extras aligned by re-deriving them from the numeric t_ms rows
    ex = [e for e, l in zip(extras, [l for l in trimmed if l.startswith("S,")])
          if re.match(r"^S,\d", l)]
    for c in EXTRA_V4:
        s_df[c] = np.nan
    if len(ex) == len(s_df):
        for i, c in enumerate(EXTRA_V4):
            s_df[c] = pd.to_numeric(pd.Series([e[i] if len(e) > i else "nan" for e in ex]), errors="coerce")
    s_df = v.reconstruct_corrected(s_df, b_df)
    return {"s": s_df, "r": r_df, "b": b_df, "p": parse_p(lines)}


def r_all_rows(lines):
    """Every R row in order (first and last report of a rep)."""
    cols = viewer().R_COLUMNS
    rows = [l.strip().split(",")[1:] for l in lines if l.startswith("R,")]
    rows = [r for r in rows if len(r) == len(cols)]
    df = pd.DataFrame(rows, columns=cols).apply(pd.to_numeric, errors="coerce").dropna()
    return df.reset_index(drop=True)


def load_log(path, session=None):
    v = viewer()
    sessions = v.split_sessions(path)
    if not sessions:
        raise ValueError(f"{path}: no S rows")
    if session is None:
        idx = int(np.argmax([sum(1 for x in l if x.startswith("S,")) for l in sessions]))
    else:
        idx = session
    d = parse_session(sessions[idx])
    d["lines"] = sessions[idx]
    d["session_index"] = idx
    d["n_sessions"] = len(sessions)
    s = d["s"]
    if s["accelScaleG"].notna().all():
        d["layout"] = "v4 (fw-rawacc-log: accRaw + accelScaleG/accBias)"
    elif s["accRawX"].notna().all():
        d["layout"] = "v3 (3.11.32: accRaw)"
    elif s["worldAccZRaw"].notna().all():
        d["layout"] = "v2 (3.11.28-3.11.31: worldAccZRaw + gyroRaw)"
    else:
        d["layout"] = "v1 (<= 3.11.27: no pre-median taps)"
    # accelScaleG printed by CALIBRATE, if the serial port was open then
    scale = None
    with open(path, errors="replace") as f:
        for line in f:
            m = re.search(r"accelScaleG=([0-9.]+)", line)
            if m:
                scale = float(m.group(1))
    d["logged_scale"] = scale
    return d


def build_inputs(d, scale, gyro_mode="auto"):
    """Returns (init_line, sample_lines, info) for harness.cpp.

    accelerometer, by layout:
      v3/v4: accRaw is the median's input -> fed as is, the firmware's own median runs.
      v2:    median output acc_med = linAcc + gravBody(q_log) * accelScaleG (exact up to
             log rounding); pre-median input approximated as acc_med + (worldAccZRaw -
             worldAccZ) * gravBody(q_log): the raw-minus-median difference is known only
             along the logged vertical. The median output is forced to acc_med.
      v1:    acc_med as above, used as both input and (forced) median output.
    gyroscope: gyroRaw (pre-median, pre-bias) through the firmware's median when the log
    has it ("raw"), else the logged gyroFiltered forced as the median output ("filtered")."""
    s = d["s"]
    q = s[["quatW", "quatX", "quatY", "quatZ"]].values.astype(float)
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    g = grav_body(q)
    lin = s[["linAccX", "linAccY", "linAccZ"]].values
    has_accraw = s["accRawX"].notna().all()
    has_v2 = s["worldAccZRaw"].notna().all()
    has_gyroraw = s["gyroRawX"].notna().all()
    info = {}
    if has_accraw:
        raw = s[["accRawX", "accRawY", "accRawZ"]].values
        med = np.c_[median5(raw[:, 0]), median5(raw[:, 1]), median5(raw[:, 2])]
        acc_in, ov_acc, acc_med = raw, 0, med
        info["acc_source"] = "accRaw (logged pre-median), firmware median"
    else:
        acc_med = lin + g * scale
        if has_v2:
            delta = (s["worldAccZRaw"].values - s["worldAccZ"].values)[:, None]
            acc_in = acc_med + delta * g
            info["acc_source"] = "median output rebuilt from linAcc+quat; pre-median approximated along the logged vertical"
        else:
            acc_in = acc_med
            info["acc_source"] = "median output rebuilt from linAcc+quat; no pre-median data (integration input = median)"
        ov_acc = 1
    gf = s[["gyroFilteredX", "gyroFilteredY", "gyroFilteredZ"]].values
    use_raw = has_gyroraw and gyro_mode in ("auto", "raw")
    if use_raw:
        gr = s[["gyroRawX", "gyroRawY", "gyroRawZ"]].values
        gm = np.c_[median5(gr[:, 0]), median5(gr[:, 1]), median5(gr[:, 2])]
        res = np.abs(gm - gf)[5:]
        info["gyro_median_residual_degs"] = {"max": float(res.max()), "p99": float(np.percentile(res, 99)),
                                             "mean": float(res.mean())}
        gyro_in, ov_gyro = gr, 0
        info["gyro_source"] = "gyroRaw (pre-median, pre-bias), firmware median + bias"
    else:
        gyro_in, ov_gyro = gf, 1
        info["gyro_source"] = "gyroFiltered (logged post-median), firmware bias"
    gb0 = s[["gyroBiasX", "gyroBiasY", "gyroBiasZ"]].values[0]
    if s["accBiasX"].notna().all():
        ab0 = s[["accBiasX", "accBiasY", "accBiasZ"]].values[0]
    else:
        ab0 = s["accZBias"].values[0] * g[0]   # only its projection on gravity is logged
    init = "I,%.6f,%.7f,%.7f,%.7f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f" % (scale, *g[0], *gb0, *ab0)
    t = s["t_ms"].values.astype(np.int64)
    lines = []
    for i in range(len(s)):
        lines.append("%d,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%d,%.5f,%.5f,%.5f,%d,%.4f,%.4f,%.4f" % (
            t[i], *acc_in[i], *gyro_in[i], ov_acc, *acc_med[i], ov_gyro, *gf[i]))
    info["scale"] = scale
    return init, lines, info


def scale_from_accraw(d):
    """accelScaleG from a log with accRaw: linAcc = median(accRaw) - gravBody * s."""
    s = d["s"]
    q = s[["quatW", "quatX", "quatY", "quatZ"]].values.astype(float)
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    g = grav_body(q)
    raw = s[["accRawX", "accRawY", "accRawZ"]].values
    med = np.c_[median5(raw[:, 0]), median5(raw[:, 1]), median5(raw[:, 2])]
    lin = s[["linAccX", "linAccY", "linAccZ"]].values
    return float(np.median(np.sum((med - lin) * g, axis=1)[5:]))
