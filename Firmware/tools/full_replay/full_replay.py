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
Full-chain offline replay of a debug log through the real firmware: the
orientation filter (Fusion), the gravity split, both bias estimators and the
phase/bracket engine, compiled unchanged on the PC (harness.cpp).

  python full_replay.py capture.log [--session N] [--scale G] [--out DIR]
                        [--config pre3.11.30] [--set gain=0.5 ...]

reconstruct.py rebuilds the filter's per-sample input from the log. A log
recorded with firmware >= 3.11.33 (48-field S row) carries everything the
replay needs (accRaw, accelScaleG, accBiasVec). On older logs:
  - accelScaleG is taken from the CALIBRATE line if the log has one, else
    derived from accRaw (3.11.32), else it must be given with --scale;
  - of accBiasVec only the projection on gravity (accZBias) is logged: the
    replay starts from that projection.
Those logs were recorded with an earlier orientation filter, so the replay
shows what this firmware would have reported on the same motion, not a
bit-exact reproduction of the log.

Prints the per-rep table (final R row of each rep) of the replay next to the
logged one and, with --out, writes the replayed log (open it with
vbt_log_viewer.py). Keep DIR outside the repository: logs are personal data.
Requires numpy, pandas and clang/clang++ (or gcc/g++).
"""

import argparse
import contextlib
import io
import os
import shutil
import subprocess
import sys

import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import reconstruct as R  # noqa: E402

TOOLS = os.path.dirname(HERE)
FW = os.path.join(os.path.dirname(TOOLS), "VBT_Quaternions")
FUSION_SOURCES = ("FusionAhrs", "FusionBias", "FusionCompass", "FusionConvention", "FusionRemap")
F_COLUMNS = ["t_ms", "q0", "q1", "q2", "q3", "accIgnored", "accErrDeg", "accRecovery", "startup", "overrange",
             "resBiasX", "resBiasY", "resBiasZ"]


def build(bindir):
    """Compiles the vendored Fusion sources and harness.cpp (which #includes
    MotionTracker.cpp) into bindir/harness; returns its path."""
    os.makedirs(bindir, exist_ok=True)
    cc = shutil.which("clang") or shutil.which("gcc")
    cxx = shutil.which("clang++") or shutil.which("g++")
    if not cc or not cxx:
        sys.exit("No C/C++ compiler found (clang/clang++ or gcc/g++).")
    objs = []
    for f in FUSION_SOURCES:
        o = os.path.join(bindir, f + ".o")
        subprocess.run([cc, "-std=c99", "-O2", "-c", os.path.join(FW, "src", "Fusion", f + ".c"), "-o", o], check=True)
        objs.append(o)
    b = os.path.join(bindir, "harness")
    inc = ["-I", os.path.join(HERE, "stubs"), "-I", os.path.join(TOOLS, "host_replay", "stubs"), "-I", FW]
    subprocess.run([cxx, "-std=c++17", "-O2", "-w", *inc, os.path.join(HERE, "harness.cpp"),
                    os.path.join(FW, "FusionOrientation.cpp"), *objs, "-o", b], check=True)
    return b


def run_raw(binary, init, lines, settings=None):
    """Runs the harness; returns its stdout (the firmware's log + F rows)."""
    args = [binary] + [f"{k}={v}" for k, v in (settings or {}).items()]
    res = subprocess.run(args, input=init + "\n" + "\n".join(lines) + "\n", capture_output=True, text=True,
                         check=True)
    return res.stdout


def quiet():
    """vbt_log_viewer prints a note when it drops the header row - silence it."""
    return contextlib.redirect_stdout(io.StringIO())


def parse_output(out):
    lines = out.split("\n")
    with quiet():
        o = R.parse_session(lines)
    f = [l.split(",")[1:] for l in lines if l.startswith("F,")]
    o["f"] = pd.DataFrame(f, columns=F_COLUMNS).apply(pd.to_numeric)
    o["lines"] = lines
    return o


def final_reps(out):
    """{rep: [peakVelocity, meanVelocity, ..., correctionStatus]} from the LAST
    R row of each rep (the authoritative one), as the printed strings."""
    reps = {}
    for line in out.split("\n"):
        if line.startswith("R,") and line[2:3].isdigit():
            parts = line.strip().split(",")
            reps[int(parts[1])] = parts[2:]
    return reps


def state_from_log(d, scale=None):
    """(init line, sample lines, info) for harness.cpp from a parsed log."""
    s = d["s"]
    info = {}
    if s["accelScaleG"].notna().all():
        scale, info["scale_source"] = float(s["accelScaleG"].iloc[0]), "logged (S row)"
    elif scale is not None:
        info["scale_source"] = "--scale"
    elif d["logged_scale"]:
        scale, info["scale_source"] = d["logged_scale"], "logged (CALIBRATE line)"
    elif s["accRawX"].notna().all():
        scale, info["scale_source"] = R.scale_from_accraw(d), "derived from accRaw vs linAcc"
    else:
        raise SystemExit("This log records no accelerometer scale: pass it with --scale (e.g. --scale 9.80665).")
    init, lines, rinfo = R.build_inputs(d, scale)
    info.update(rinfo)
    joined = R.joined_state(d, scale)
    if joined:
        init += "\n" + joined
        info["start_state"] += "; joins a running session (engine state recovered from the log)"
    info["accbias_source"] = "logged (S row)" if s["accBiasX"].notna().all() else "projection of accZBias only"
    return init, lines, info


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logfile")
    ap.add_argument("--session", type=int, default=None, help="REC_START block (0-based; default: the longest)")
    ap.add_argument("--all-sessions", action="store_true",
                    help="replay every REC_START block in order; with --out, one replayed log for the whole file")
    ap.add_argument("--scale", type=float, default=None, help="accelScaleG, for logs that do not record it")
    ap.add_argument("--out", default=None, help="folder for the replayed log (outside git)")
    ap.add_argument("--config", choices=["pre3.11.30"], default=None,
                    help="replay with the RuntimeConfig defaults of v3.11.28/3.11.29")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                    help="Fusion setting override (gain, gyroRange, accRej, timeout, skipStartup, bias, biasThr, "
                         "biasPeriod) - for experiments; the firmware uses the defaults - or cfg.<field>=value, a "
                         "Config-packet field the app may have written (e.g. cfg.maxVelocityFlatWindowSamples=30)")
    args = ap.parse_args()
    out_dir = args.out
    bindir = os.path.join(out_dir or HERE, "bin")
    binary = build(bindir)
    settings = dict(x.split("=", 1) for x in args.set)
    if args.config:
        settings["config"] = args.config
    if args.all_sessions:
        with quiet():
            n = len(R.viewer().split_sessions(args.logfile))
        outs = []
        for k in range(n):
            with quiet():
                dk = R.load_log(args.logfile, k)
            init, lines, info = state_from_log(dk, args.scale)
            outs.append(run_raw(binary, init, lines, settings))
            reps = final_reps(outs[-1])
            print(f"session {k + 1}/{n}: {len(dk['s'])} samples, {len(reps)} reps; {info['start_state']}")
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
            p = os.path.join(out_dir, os.path.splitext(os.path.basename(args.logfile))[0] + "_replay.log")
            with open(p, "w") as f:
                f.write("\n".join(l for o in outs for l in o.split("\n") if not l.startswith("F,")))
            print(f"replayed log: {p}")
        return
    with quiet():
        d = R.load_log(args.logfile, args.session)
    init, lines, info = state_from_log(d, args.scale)
    out = run_raw(binary, init, lines, settings)
    print(f"layout {d['layout']}, session {d['session_index'] + 1}/{d['n_sessions']}, {len(d['s'])} samples")
    print(f"accelerometer: {info['acc_source']}; gyroscope: {info['gyro_source']}")
    print(f"accelScaleG {info['scale']:.4f} ({info['scale_source']}); accBiasVec: {info['accbias_source']}")
    print(f"start state: {info['start_state']}")
    logged = {int(r["rep"]): r for _, r in d["r"].iterrows()} if len(d["r"]) else {}
    replay = final_reps(out)
    print(f"\n{'rep':>3}  {'logged PV / MV / disp':>26}  {'replay PV / MV / disp':>26}")
    for k in sorted(set(logged) | set(replay)):
        a = logged.get(k)
        b = replay.get(k)
        sa = f"{a['peakVelocity']:.3f} / {a['meanVelocity']:.3f} / {a['displacementM']:.3f}" if a is not None else "-"
        sb = f"{float(b[0]):.3f} / {float(b[1]):.3f} / {float(b[4]):.3f}" if b is not None else "-"
        print(f"{k:>3}  {sa:>26}  {sb:>26}")
    print(f"\nreps: logged {len(logged)}, replay {len(replay)}")
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
        p = os.path.join(out_dir, os.path.splitext(os.path.basename(args.logfile))[0] + "_replay.log")
        with open(p, "w") as f:
            f.write("\n".join(l for l in out.split("\n") if not l.startswith("F,")))
        print(f"replayed log: {p}")


if __name__ == "__main__":
    main()
