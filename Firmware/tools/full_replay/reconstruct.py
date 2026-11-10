#!/usr/bin/env python3
# Part of the ODKI VBT tools
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Lodovico Cortelazzo (ODKI)
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may
# not use this file except in compliance with the License. You may obtain a
# copy of the License in LICENSE-APACHE in the repository root or at
# http://www.apache.org/licenses/LICENSE-2.0. Unless required by applicable
# law or agreed to in writing, software distributed under the License is
# distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
# KIND, either express or implied. See the License for the specific
# language governing permissions and limitations under the License.

"""
Reading a debug log and reconstructing, sample by sample, the IMU input of
the orientation filter, for the full-chain replay (harness.cpp, see
full_replay.py and "tools/full_replay/" in Firmware/DOCUMENTATION.md for the
derivation).

Log layouts accepted (fields after "S,"):
  37  firmware <= 3.11.27       no pre-median taps
  41  firmware 3.11.28-3.11.31  + worldAccZRaw, gyroRawX/Y/Z
  44  firmware 3.11.32          + accRawX/Y/Z (pre-median body-frame accelerometer)
  48  firmware >= 3.11.33       + accelScaleG, accBiasX/Y/Z (state the replay needs)
  53  firmware 3.12.5 (diag.)   + accPeakG, accClipCount, accSampleCount, gyroPeakDegS,
                                gyroClipCount (416Hz saturation taps, not used by the replay)
  54  firmware 3.12.6 (test)    + shock (shock-handling state: 0 none, 1 window/candidate
                                open, 2 landing window, 3 landing impulse applied;
                                4 orientation re-levelled, since the re-level test build)

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
# trailing columns of the 53/54-field rows, carried into the S frame (NaN on older logs)
EXTRA_V6 = EXTRA_V4 + ["accPeakG", "accClipCount", "accSampleCount", "gyroPeakDegS", "gyroClipCount", "shock"]
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
    for c in EXTRA_V6:
        s_df[c] = np.nan
    if len(ex) == len(s_df):
        for i, c in enumerate(EXTRA_V6):
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
        d["layout"] = "v4 (>= 3.11.33: accRaw + accelScaleG/accBias)"
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


MEDIAN_PRIME = 4   # samples whose 5-tap median window reaches back before the first logged row


def seed_quaternion(q0, gyro_dps, dt):
    """Attitude before the first logged sample: the logged quaternion of that
    row (body->world, w first) rotated back by its own gyroscope step, so that
    the filter's first update lands on the logged attitude (yaw included). The
    accelerometer correction of that one step is neglected (a fraction of a
    degree times gain*dt)."""
    w = np.radians(np.asarray(gyro_dps, dtype=float)) * dt
    th = np.linalg.norm(w)
    if th < 1e-12:
        return q0 / np.linalg.norm(q0)
    n = w / th
    r = np.r_[np.cos(th / 2), -np.sin(th / 2) * n]           # inverse of the body-frame step
    a, b = q0, r
    q = np.r_[a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
              a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
              a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
              a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]]
    return q / np.linalg.norm(q)


def build_inputs(d, scale, gyro_mode="auto", prime=True):
    """Returns (init_line, sample_lines, info) for harness.cpp.

    Start state (prime=True, the default): on the sensor the two 5-tap medians
    already hold the four readings before the first logged row (they run from
    boot, START does not reset them) and the orientation filter has been
    running all along. So for the first MEDIAN_PRIME rows the medians' OUTPUT
    is forced to the logged one (accelerometer: linAcc + gravBody(q_log) *
    accelScaleG; gyroscope: gyroFiltered) while their input is still fed, so
    that from row MEDIAN_PRIME on the firmware's own median runs on logged
    rows only; and the filter is seeded with the logged quaternion of the
    first row rotated back by that row's gyroscope step (seed_quaternion()),
    not with a zero-yaw gravity seed. Without it the first worldAccZ values
    differ by up to 0.4 m/s^2, stillness can be confirmed a few samples
    earlier and the first bracket's rate, which seeds the EMA, changes the
    whole set. prime=False gives the previous behaviour (empty medians,
    gravity seed).

    accelerometer, by layout:
      v3/v4: accRaw is the median's input -> fed as is, the firmware's own median runs.
      v2:    median output acc_med = linAcc + gravBody(q_log) * accelScaleG (exact up to
             log rounding); pre-median input approximated as acc_med + (worldAccZRaw -
             worldAccZ) * gravBody(q_log): the raw-minus-median difference is known only
             along the logged vertical (exact for the filter that recorded the log, an
             approximation of second order in the attitude difference for another one).
             The median output is forced to acc_med.
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
    n_prime = 0
    if prime:
        acc_med_log = lin + g * scale
        dt0 = float(s["dt_ms"].values[0]) / 1000.0
        qs = seed_quaternion(q[0], gf[0] - gb0, dt0)
        init += ",%.7f,%.7f,%.7f,%.7f" % tuple(qs)
        # the first dt after START is the real period of that sample (often 12-13 ms), not 10
        init += "\nT,%d" % int(round(float(s["dt_ms"].values[0])))
        n_prime = min(MEDIAN_PRIME, len(s))
        info["start_state"] = "medians forced to the logged output for %d rows, filter seeded from the logged quaternion" % n_prime
    else:
        info["start_state"] = "empty medians, gravity seed (zero yaw)"
    lines = []
    for i in range(len(s)):
        if i < n_prime:
            lines.append("%d,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,1,%.5f,%.5f,%.5f,1,%.4f,%.4f,%.4f" % (
                t[i], *acc_in[i], *gyro_in[i], *acc_med_log[i], *gf[i]))
            continue
        lines.append("%d,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%d,%.5f,%.5f,%.5f,%d,%.4f,%.4f,%.4f" % (
            t[i], *acc_in[i], *gyro_in[i], ov_acc, *acc_med[i], ov_gyro, *gf[i]))
    info["scale"] = scale
    return init, lines, info


def _float32_session_start(estimate, lines, t_ms, dt_ms, span=800):
    """sessionT is a float accumulated on the sensor since START (one
    (now - last) / 1000.0f per sample), so at the first row of a joined log it
    carries the rounding of the unlogged samples. Among the float32 values
    within +-span ulps of the estimate, returns the one whose accumulation
    over the logged rows (the firmware's arithmetic) prints every B row's t1
    (the sessionT of the S row after it, 4 decimals) as logged."""
    c = np.float32(estimate)
    cands = [c]
    lo = hi = c
    for _ in range(span):
        lo = np.nextafter(lo, np.float32(-1e9)); hi = np.nextafter(hi, np.float32(1e9))
        cands += [lo, hi]
    cands = np.array(cands, dtype=np.float32)
    targets = {}
    pending = []
    for l in lines:
        if l.startswith("B,") and l[2:3].isdigit():
            pending.append(l.split(",")[3])
        elif l.startswith("S,") and l[2:3].isdigit() and pending:
            targets[int(l.split(",")[1])] = pending[-1]
            pending = []
    dts = (dt_ms.astype(np.float32) / np.float32(1000.0)).astype(np.float32)
    acc = cands.copy()
    score = np.zeros(len(cands), int)
    for i in range(len(t_ms)):
        acc = (acc + dts[i]).astype(np.float32)
        tgt = targets.get(int(t_ms[i]))
        if tgt is not None:
            score += np.array(["%.4f" % x for x in acc]) == tgt
    best = int(np.argmax(score))     # first maximum = closest to the estimate (cands is ordered by distance)
    return float(cands[best])


def joined_state(d, scale):
    """The "J," line for harness.cpp when the log joins a session that was
    already running (the serial port opened after START, so there is no
    START/REC_START line before the first S row), or None. Everything comes
    from the log, under one condition the function checks: the first row is
    a confirmed-still sample before any rep (then velZ has just been set to
    0, no phase is open and the bracket starts there).
      sessionT, nextBracketId: from the first B row (its t1 is the sessionT of
        the S row that follows it; its id is the next one);
      nextPhaseId: the first P row's id; repNumber: the first row's rep;
      emaRate before that B row: inverted from its emaRateAfter and rate
        (closeBracketFn(): alpha = emaAlpha * min(1, basisTotal / 1 s));
      velZFrameOffset: -repCalibIntercept while repCalibCount is still 0 (each
        frame shift moves both by the same amount, a landing would break it);
      the flat-guard and bias-gate rings: filled with the first row's values
        (still), the trapezoid and the jerk with its acceleration. The rep direction
    is assumed Up (the Config is not in the log).
    Not in the log: the zero-crossing reference (assumed none) and the
    re-level / shock state (assumed idle)."""
    lines = d["lines"]
    if any(l.startswith("START") or l.strip() == "REC_START" for l in lines):
        return None
    s = d["s"]
    r0 = s.iloc[0]
    if not (r0["confirmedStillNow"] == 1 and r0["repCalibCount"] == 0 and r0["state"] == "idle"):
        raise ValueError("joined log: the first row is not a confirmed-still sample before any rep")
    b = d["b"].astype(float)
    if not len(b):
        raise ValueError("joined log: no B row to recover the session time from")
    b0 = b.iloc[0]
    blines = [l for l in lines if l.startswith("B,") and l[2:3].isdigit()]
    after = None
    seen = False
    for l in lines:
        if l.startswith("B,") and l.strip() == blines[0].strip():
            seen = True
        elif seen and l.startswith("S,") and l[2:3].isdigit():
            after = int(l.split(",")[1])
            break
    t_ms = s["t_ms"].values.astype(np.int64)
    dt = s["dt_ms"].values / 1000.0
    j = int(np.searchsorted(t_ms, after))
    t_first = float(b0["t1"]) - float(np.sum(dt[1:j + 1]))      # sessionT of the first row
    session_t = _float32_session_start(t_first - dt[0], lines, t_ms, s["dt_ms"].values)
    alpha = 0.3 * min(1.0, float(b0["basisTotal"]) / 1.0)        # emaAlpha (firmware-only) * duration weight
    ema_before = (float(b0["emaRateAfter"]) - alpha * float(b0["rate"])) / (1.0 - alpha)
    p = d["p"]
    next_phase = int(p["phaseId"].iloc[0]) if len(p) else 0
    q = np.array([r0["quatW"], r0["quatX"], r0["quatY"], r0["quatZ"]], float)
    g = grav_body((q / np.linalg.norm(q))[None, :])[0]
    acc_mag = float(np.linalg.norm(np.array([r0["linAccX"], r0["linAccY"], r0["linAccZ"]]) + g * scale))
    return "J,%.9g,%d,%d,%d,1,%.6f,%.4f,%.4f,%.4f,%.4f,%.2f,%.4f,%.4f,%.4f" % (
        session_t, int(b0["bracketId"]), next_phase, int(r0["rep"]), ema_before,
        -float(r0["repCalibIntercept"]), float(r0["repCalibIntercept"]), float(r0["velZLive"]),
        float(r0["worldAccZ"]), float(r0["gyroMag"]), acc_mag, float(r0["worldAccZRaw"]),
        float(r0["worldAccZRaw"]) + float(r0["accZBias"]))     # the shock detector's jerk input has no bias


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
