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
World-Z acceleration from the 416Hz FIFO stream of the v3.12.10 diagnostic
build ("H," lines), compared with the 10ms poll means the engine integrates.

  python fifo_worldz.py capture.log [--session N] [--csv OUT.csv]
  python fifo_worldz.py --self-test

1. Decodes the "H," lines into segments of consecutive FIFO sets
   (Gx,Gy,Gz,Ax,Ay,Az raw int16; see DOCUMENTATION.md, "Lab data capture").
2. Aligns every segment with the "S," rows. With the IMU bus at 400 kHz no
   reading is lost between two polls, so the accSampleCount readings of one
   poll mean are consecutive FIFO sets; the logged accRawX/Y/Z (mean x
   accelScaleG, 4 decimals) must equal the mean of such a run to the printed
   digit. Each row is matched against every run of its length; a unique
   match consistent with the order of the others fixes the index of the
   row's last reading. Rows without a match get the index from a straight
   line fitted to the matched rows of the segment (reported as "fitted").
3. For every row, the readings after the previous row's last reading up to
   its own last one are the readings of that period. Each is projected on
   the vertical with the row's quaternion, accelScaleG and accZBias:
   gravity g = quaternion's gravity direction, worldZ = (a - accelScaleG*g).g
   - accZBias - the projection the firmware uses for worldAccZRaw.
4. Per row: worldAccZRaw logged (poll mean), the same recomputed from the
   logged accRaw and the printed quaternion (shows the rounding of the
   printed quaternion), and the mean of all the readings of the period.
   Per phase of the "P," rows: the integral (sum of value x dt_ms) of the
   poll mean and of the all-readings mean, and their difference - the
   velocity error due to sampling alone, same attitude.

--csv writes one line per reading: seg, seq, row t_ms, estimated reading
time (row t_ms minus the readings after it / 416Hz), raw values, worldZ.
--self-test builds a synthetic log (known tilt, vertical motion with a
vibration, poll means of 2-4 consecutive readings, two segments) and
checks the alignment and the rebuilt acceleration. Requires numpy.
"""

import argparse
import math
import sys

import numpy as np

ACC_LSB_G = 0.061 * 4 / 1000      # LSM6DS3 library calcAccel() at +-8 g
GYRO_LSB_DPS = 4.375 * 4 / 1000   # calcGyro() at +-500 deg/s
ODR_HZ = 416.0
ACC_TOL = 6e-5                    # accRaw is printed with 4 decimals


# ----------------------------------------------------------------------
# Parsing
# ----------------------------------------------------------------------

def split_sessions(lines):
    sessions, cur = [], None
    for line in lines:
        line = line.rstrip("\r\n")
        if line == "REC_START":
            cur = []
            sessions.append(cur)
        elif line == "REC_STOP":
            cur = None
        elif cur is not None:
            cur.append(line)
    return sessions


def s16(h):
    v = int(h, 16)
    return v - 65536 if v >= 32768 else v


def parse_session(lines):
    s_hdr, p_hdr = None, None
    rows, phases, segs, flags = [], [], {}, []
    for line in lines:
        if line.startswith("S,t_ms"):
            s_hdr = line.split(",")[1:]
        elif line.startswith("P,phaseId"):
            p_hdr = line.split(",")[1:]
        elif line.startswith("S,") and s_hdr:
            parts = line.split(",")[1:]
            if len(parts) != len(s_hdr):
                continue
            try:
                rows.append({k: (v if k == "state" else float(v)) for k, v in zip(s_hdr, parts)})
            except ValueError:
                continue
        elif line.startswith("P,") and p_hdr:
            parts = line.split(",")[1:]
            if len(parts) == len(p_hdr):
                try:
                    phases.append({k: float(v) for k, v in zip(p_hdr, parts)})
                except ValueError:
                    pass
        elif line.startswith("H,") and not line.startswith("H,t_ms"):
            p = line.split(",")
            if len(p) < 8:
                continue
            seg, seq0, n, fl = int(p[2]), int(p[3]), int(p[4]), int(p[5])
            data = p[7]
            if len(data) != n * 24:
                continue
            if fl:
                flags.append((int(p[1]), seg, fl))
            lst = segs.setdefault(seg, {})
            for j in range(n):
                lst[seq0 + j] = [s16(data[j * 24 + 4 * i: j * 24 + 4 * i + 4]) for i in range(6)]
    out = {}
    for seg, d in segs.items():
        idx = np.array(sorted(d))
        out[seg] = {"seq": idx, "raw": np.array([d[i] for i in idx], dtype=float)}
    return rows, phases, out, flags


# ----------------------------------------------------------------------
# Geometry
# ----------------------------------------------------------------------

def grav_body(qw, qx, qy, qz):
    """Gravity direction in the body frame, as in updateOrientationAndAcceleration()."""
    return np.array([2 * (qx * qz - qw * qy), 2 * (qw * qx + qy * qz), qw * qw - qx * qx - qy * qy + qz * qz])


def world_z(acc_ms2, g, scale, bias):
    """(a - scale*g).g - bias; acc_ms2 is (..., 3)."""
    return (acc_ms2 - scale * g) @ g - bias


# ----------------------------------------------------------------------
# Alignment
# ----------------------------------------------------------------------

def align(rows, seg):
    """Index (into seg arrays) of the last reading of every row, or -1;
    and a per-row status: 'matched', 'fitted' or ''."""
    raw = seg["raw"]
    seq = seg["seq"]
    m = len(raw)
    n_rows = len(rows)
    end = np.full(n_rows, -1, dtype=int)
    status = [""] * n_rows
    if m == 0:
        return end, status
    # consecutive runs only: break at seq gaps
    contiguous = np.concatenate([[True], np.diff(seq) == 1])
    csum = np.vstack([np.zeros(3), np.cumsum(raw[:, 3:6], axis=0)])
    # run_ok[k][e]: sets e-k+1..e are consecutive
    breaks = np.cumsum(~contiguous)
    cands = []
    for i, r in enumerate(rows):
        k = int(r.get("accSampleCount", 0))
        if k < 1 or k > m:
            cands.append(None)
            continue
        e = np.arange(k - 1, m)
        mean = (csum[e + 1] - csum[e + 1 - k]) / k
        ok_run = breaks[e] == breaks[e - k + 1] if k > 1 else np.ones(len(e), bool)
        target = np.array([r["accRawX"], r["accRawY"], r["accRawZ"]])
        val = mean * ACC_LSB_G * r["accelScaleG"]
        hit = np.all(np.abs(val - target) <= ACC_TOL, axis=1) & ok_run
        cands.append(e[hit])
    # keep unique matches, then enforce order with a straight-line fit
    uniq = [(i, int(c[0])) for i, c in enumerate(cands) if c is not None and len(c) == 1]
    if len(uniq) >= 2:
        ii = np.array([u[0] for u in uniq], float)
        ee = np.array([u[1] for u in uniq], float)
        t = np.array([rows[int(i)]["t_ms"] for i in ii])
        # Robust line: unique matches at rest can be chance coincidences, so
        # start from the most common offset at the nominal rate (readings
        # per ms), then refit on the inliers.
        a = ODR_HZ / 1000.0
        off = ee - a * t
        hist, edges = np.histogram(off, bins=np.arange(off.min() - 1, off.max() + 3, 2.0))
        b = edges[np.argmax(hist)] + 1.0
        for _ in range(3):
            good = np.abs(ee - (a * t + b)) <= 3
            if good.sum() < 2:
                break
            a, b = np.polyfit(t[good], ee[good], 1)
        good = np.abs(ee - (a * t + b)) <= 3
        for (i, e), g in zip(uniq, good):
            if g:
                end[i] = e
                status[i] = "matched"
        # rows with several candidates: take the one closest to the line
        for i, c in enumerate(cands):
            if status[i] or c is None or len(c) < 2:
                continue
            pred = a * rows[i]["t_ms"] + b
            j = int(c[np.argmin(np.abs(c - pred))])
            if abs(j - pred) <= 3:
                end[i] = j
                status[i] = "matched"
        for i in range(n_rows):
            if status[i]:
                continue
            pred = int(round(a * rows[i]["t_ms"] + b))
            if 0 <= pred < m:
                end[i] = pred
                status[i] = "fitted"
    elif len(uniq) == 1:
        end[uniq[0][0]] = uniq[0][1]
        status[uniq[0][0]] = "matched"
    # the mapping must not go backwards
    last = -1
    for i in range(n_rows):
        if end[i] >= 0:
            if end[i] < last:
                end[i] = -1
                status[i] = ""
            else:
                last = end[i]
    return end, status


# ----------------------------------------------------------------------
# Analysis
# ----------------------------------------------------------------------

def analyse(rows, phases, segs, csv_path=None, quiet=False):
    n = len(rows)
    owner = np.full(n, -1)            # segment that covers each row
    end = np.full(n, -1)
    status = [""] * n
    for sid, seg in segs.items():
        e, st = align(rows, seg)
        for i in range(n):
            if st[i] and (owner[i] < 0 or (st[i] == "matched" and status[i] != "matched")):
                owner[i], end[i], status[i] = sid, e[i], st[i]
    wz_log = np.array([r.get("worldAccZRaw", np.nan) for r in rows])
    wz_poll = np.full(n, np.nan)
    wz_full = np.full(n, np.nan)
    n_full = np.zeros(n, int)
    dt = np.array([r["dt_ms"] for r in rows]) / 1000.0
    csv = open(csv_path, "w") if csv_path else None
    if csv:
        csv.write("seg,seq,row_t_ms,t_est_ms,gx,gy,gz,ax,ay,az,worldZ\n")
    for i, r in enumerate(rows):
        g = grav_body(r["quatW"], r["quatX"], r["quatY"], r["quatZ"])
        sc, bias = r["accelScaleG"], r["accZBias"]
        wz_poll[i] = world_z(np.array([r["accRawX"], r["accRawY"], r["accRawZ"]]), g, sc, bias)
        if i == 0 or end[i] < 0 or end[i - 1] < 0 or owner[i] != owner[i - 1]:
            continue
        seg = segs[owner[i]]
        lo, hi = end[i - 1] + 1, end[i] + 1
        if hi <= lo or np.any(np.diff(seg["seq"][lo - 1:hi]) != 1):
            continue
        acc = seg["raw"][lo:hi, 3:6] * ACC_LSB_G * sc
        wz = world_z(acc, g, sc, bias)
        wz_full[i] = wz.mean()
        n_full[i] = hi - lo
        if csv:
            for j in range(lo, hi):
                t_est = r["t_ms"] - (end[i] - j) * 1000.0 / ODR_HZ
                rw = seg["raw"][j]
                csv.write(f"{owner[i]},{seg['seq'][j]},{r['t_ms']:.0f},{t_est:.2f},"
                          + ",".join(f"{v:.0f}" for v in rw) + f",{wz[j - lo]:.5f}\n")
    if csv:
        csv.close()
    res = {"rows": n, "matched": sum(s == "matched" for s in status), "fitted": sum(s == "fitted" for s in status),
           "with_all_readings": int(np.sum(~np.isnan(wz_full))), "end": end, "status": status,
           "wz_log": wz_log, "wz_poll": wz_poll, "wz_full": wz_full, "n_full": n_full, "dt": dt}
    # per phase
    t = np.array([r["t_ms"] for r in rows])
    res["phases"] = []
    for p in phases:
        if "t_start_ms" not in p or p.get("reported", 1) == 0:
            continue
        sel = (t > p["t_start_ms"]) & (t <= p["t_end_ms"])
        ok = sel & ~np.isnan(wz_full)
        if sel.sum() == 0:
            continue
        v_poll = float(np.sum(wz_poll[ok] * dt[ok]))
        v_full = float(np.sum(wz_full[ok] * dt[ok]))
        res["phases"].append({"rep": int(p["repNumber"]), "type": int(p["phaseType"]), "rows": int(sel.sum()),
                              "rows_ok": int(ok.sum()), "v_poll": v_poll, "v_full": v_full, "diff": v_full - v_poll})
    if not quiet:
        print(f"rows {n}: matched {res['matched']}, fitted {res['fitted']}, "
              f"with every reading of the period {res['with_all_readings']}")
        ok = ~np.isnan(wz_full)
        if ok.any():
            d = wz_full[ok] - wz_poll[ok]
            print(f"all-readings mean - poll mean, worldZ: mean {d.mean():+.4f}, rms {np.sqrt((d ** 2).mean()):.4f}, "
                  f"max |.| {np.abs(d).max():.4f} m/s^2; readings per period {n_full[ok].mean():.2f}")
            q = ~np.isnan(wz_log) & ok
            if q.any():
                e = wz_poll[q] - wz_log[q]
                print(f"recomputed - logged worldAccZRaw (printed-quaternion rounding): rms {np.sqrt((e ** 2).mean()):.4f}")
        if res["phases"]:
            print("rep type rows(ok)   v_poll    v_full   full-poll  [m/s, sum of a*dt over the phase]")
            for p in res["phases"]:
                print(f"{p['rep']:3d} {p['type']:+3d} {p['rows']:4d}({p['rows_ok']:4d}) {p['v_poll']:+8.3f} "
                      f"{p['v_full']:+8.3f} {p['diff']:+9.3f}")
    return res


# ----------------------------------------------------------------------
# Synthetic self-test
# ----------------------------------------------------------------------

def quat_from_tilt(rx_deg, ry_deg):
    ax, ay = math.radians(rx_deg) / 2, math.radians(ry_deg) / 2
    qx = (math.cos(ax), math.sin(ax), 0.0, 0.0)
    qy = (math.cos(ay), 0.0, math.sin(ay), 0.0)
    w1, x1, y1, z1 = qx
    w2, x2, y2, z2 = qy
    return (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)


def synthetic_log(seed=1):
    rng = np.random.default_rng(seed)
    sc = 9.49057
    q = [round(v, 4) for v in quat_from_tilt(14.0, -9.0)]
    g = grav_body(*q)
    dur = 8.0
    n_set = int(dur * ODR_HZ)
    t_set = np.arange(n_set) / ODR_HZ * 1000.0 + 0.7                 # ms

    def a_world(tms):
        ts = tms / 1000.0
        rep = np.where((ts > 1.0) & (ts < 7.0), 2.0 * np.sin(2 * np.pi * (ts - 1.0) / 1.5), 0.0)
        vib = np.where((ts > 1.0) & (ts < 7.0), 3.0 * np.sin(2 * np.pi * 37.0 * ts), 0.0)
        return rep + vib

    aw = a_world(t_set)
    acc_g = ((sc + aw)[:, None] * g[None, :]) / sc
    raw_acc = np.round(acc_g / ACC_LSB_G + rng.normal(0, 2, (n_set, 3)))
    raw_gyr = np.round(rng.normal(0, 3, (n_set, 3)))
    raw = np.hstack([raw_gyr, raw_acc]).astype(int)
    lines = ["REC_START",
             "S,t_ms,dt_ms,state,quatW,quatX,quatY,quatZ,accZBias,worldAccZRaw,accRawX,accRawY,accRawZ,"
             "accelScaleG,accSampleCount",
             "P,phaseId,repNumber,phaseType,closeReason,durationS,sampleCount,discarded,reported,t_start_ms,t_end_ms",
             "H,t_ms,seg,seq0,n,flags,levelWords,data"]
    truth = []
    t_ms, last_end, next_line = 20, -1, 0
    seg, seg_start = 0, 0
    gap_at = 4000.0                        # ms: an overrun, a new segment
    while True:
        t_ms += int(rng.choice([10, 10, 11]))
        if t_ms > dur * 1000 - 20:
            break
        e = int(np.searchsorted(t_set, t_ms, side="right")) - 1   # last reading before the row
        k = int(rng.choice([2, 3, 3, 4]))
        run = raw[e - k + 1:e + 1, 3:6]
        mean_ms2 = run.mean(axis=0) * ACC_LSB_G * sc
        wz_poll = world_z(mean_ms2, grav_body(*q), sc, 0.0)
        period = raw[last_end + 1:e + 1, 3:6] if last_end >= 0 else None
        truth.append({"t_ms": t_ms, "e_abs": e,
                      "wz_full": world_z(period * ACC_LSB_G * sc, grav_body(*q), sc, 0.0).mean()
                      if period is not None and len(period) else np.nan,
                      "aw_true": aw[last_end + 1:e + 1].mean() if last_end >= 0 else np.nan})
        last_end = e
        lines.append(f"S,{t_ms},{rng.choice([10.0, 11.0]):.3f},move,{q[0]:.4f},{q[1]:.4f},{q[2]:.4f},{q[3]:.4f},"
                     f"0.0000,{wz_poll:.4f},{mean_ms2[0]:.4f},{mean_ms2[1]:.4f},{mean_ms2[2]:.4f},{sc:.5f},{k}")
        # H line: the FIFO sets read so far (lags the row by 0-3 readings)
        upto = max(next_line, e - int(rng.integers(0, 4)))
        if t_ms > gap_at and seg == 0:
            seg, seg_start, next_line = 1, e + 25, e + 25   # 25 readings lost
            lines.append(f"H,{t_ms},1,0,0,1,0,")
            continue
        if upto > next_line:
            n = upto - next_line
            data = "".join("".join(f"{v & 0xFFFF:04x}" for v in raw[j]) for j in range(next_line, upto))
            lines.append(f"H,{t_ms},{seg},{next_line - seg_start},{n},0,{6 * int(rng.integers(0, 4))},{data}")
            next_line = upto
    lines.append("P,0,1,1,0,1.5,150,2,1,1000,2500")
    lines.append("P,1,2,1,0,1.5,150,2,1,5500,7000")
    lines.append("REC_STOP")
    return lines, truth, seg_start


def self_test():
    lines, truth, seg1_start = synthetic_log()
    rows, phases, segs, flags = parse_session(split_sessions(lines)[0])
    res = analyse(rows, phases, segs)
    for tr in truth:   # where the row's last reading really is in the logged stream (None: not logged)
        sid, sq = (1, tr["e_abs"] - seg1_start) if tr["e_abs"] >= seg1_start else (0, tr["e_abs"])
        present = sid in segs and sq in set(segs[sid]["seq"].tolist())
        tr["seg"], tr["seq_end"] = (sid, sq) if present else (None, None)
    ok = True
    # alignment: every aligned row must point at the true last reading
    wrong = covered = 0
    for i, tr in enumerate(truth):
        e = res["end"][i]
        if e < 0:
            continue
        covered += 1
        if tr["seg"] is None or tr["seg"] not in segs or segs[tr["seg"]]["seq"][e] != tr["seq_end"]:
            wrong += 1
    lost = sum(tr["seg"] is None for tr in truth)
    print(f"self-test: rows {len(rows)} ({lost} whose last reading is not in the logged stream: gap or not yet read), aligned {covered} "
          f"(matched {res['matched']}, fitted {res['fitted']}), wrong index {wrong}, segments {sorted(segs)}")
    if wrong or res["matched"] < 0.9 * (len(rows) - lost):
        ok = False
    # rebuilt acceleration = mean of every reading of the period (truth from the generator)
    d = [abs(res["wz_full"][i] - tr["wz_full"]) for i, tr in enumerate(truth)
         if not np.isnan(res["wz_full"][i]) and not np.isnan(tr["wz_full"])]
    e = [abs(res["wz_full"][i] - tr["aw_true"]) for i, tr in enumerate(truth)
         if not np.isnan(res["wz_full"][i]) and not np.isnan(tr["aw_true"])]
    print(f"self-test: periods compared {len(d)}, max |rebuilt - generator| {max(d):.2e} m/s^2, "
          f"max |rebuilt - true vertical acceleration| {max(e):.4f} m/s^2 (quantisation + noise)")
    if not d or max(d) > 1e-6 or max(e) > 0.05:
        ok = False
    print("PASS" if ok else "FAIL")
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logfile", nargs="?")
    ap.add_argument("--session", type=int, help="REC_START block (0-based; default: the longest)")
    ap.add_argument("--csv", help="write one line per 416Hz reading")
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args()
    if a.self_test:
        sys.exit(0 if self_test() else 1)
    if not a.logfile:
        ap.error("logfile required (or --self-test)")
    with open(a.logfile, errors="replace") as f:
        sessions = split_sessions(f.readlines())
    if not sessions:
        sys.exit("no REC_START block")
    if a.session is None:
        idx = int(np.argmax([sum(1 for l in s if l.startswith("S,")) for s in sessions]))
    else:
        idx = a.session
    rows, phases, segs, flags = parse_session(sessions[idx])
    if not segs:
        sys.exit("no H, lines in this block (not a v3.12.10 diagnostic log, or the FIFO did not start)")
    print(f"block {idx}: {len(rows)} S rows, {len(segs)} segment(s), "
          f"{sum(len(s['seq']) for s in segs.values())} readings, {len(flags)} flagged H lines")
    analyse(rows, phases, segs, a.csv)


if __name__ == "__main__":
    main()
