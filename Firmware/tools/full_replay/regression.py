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
Regression test of the whole tracking chain (Fusion orientation filter +
engine), pinned to the behaviour of firmware 3.12.4.

  python regression.py                  synthetic data, golden file committed
  python regression.py --update         rewrite the synthetic golden file
  python regression.py --real DIR       recorded logs, golden data kept OUTSIDE
                                        the repository (logs are personal data)

Synthetic mode generates a deterministic IMU recording (a tilted sensor, six
squat-like reps with small rotations, sensor noise and a gyroscope bias -
made-up motion, not a measurement), replays it through harness.cpp and
compares every rep's final R row with golden/synthetic_3.12.4.json.

Real mode reads DIR/golden.json: for each entry the log (read in place), the
REC_START block, the pinned initial state (the "I," line: accelScaleG and
accBiasVec, which logs older than 3.11.33 do not record) and the expected
output. It checks that the input rebuilt from the log is unchanged (SHA-256),
then that the replay's output is unchanged (SHA-256 of every line except the
INFO line, which carries the version) and every rep's final R row.

PASS requires the same reps and every R value within its printed precision
(the last digit); "byte-identical" is reported separately - another compiler
or CPU may round the last float digit differently.
Requires numpy, pandas and clang/clang++ (or gcc/g++).
"""

import argparse
import hashlib
import json
import math
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import full_replay as FR  # noqa: E402

GOLDEN = os.path.join(HERE, "golden", "synthetic_3.12.4.json")
G = 9.80665


# ----------------------------------------------------------------------
# Synthetic recording
# ----------------------------------------------------------------------

class Lcg:
    """Deterministic, platform-independent noise (no numpy RNG versions involved)."""

    def __init__(self, seed):
        self.x = seed

    def uniform(self):
        self.x = (self.x * 1103515245 + 12345) % 2147483648
        return self.x / 2147483648.0

    def noise(self, sd):
        # sum of 4 uniforms: zero mean, close enough to normal for sensor noise
        return (sum(self.uniform() for _ in range(4)) - 2.0) * sd * math.sqrt(3.0)


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return [[1, 0, 0], [0, c, -s], [0, s, c]]


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, 0, s], [0, 1, 0], [-s, 0, c]]


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mat_t_vec(m, v):
    """m^T v"""
    return [sum(m[k][i] * v[k] for k in range(3)) for i in range(3)]


def motion():
    """[(kind, duration_s, dz_m)] segments of the made-up set: rest, six reps
    (eccentric 0.45 m down in 1.4 s, 0.25 s at the bottom, concentric up in 0.6-1.0 s, a
    pause at the top), rest. Cosine profiles; the bar tilts by up to 4 deg about X with each rep."""
    segs = [("rest", 4.0, 0.0)]
    for k, up_t in enumerate((0.70, 0.75, 0.80, 0.85, 0.95, 1.05)):
        segs += [("move", 1.4, -0.45), ("rest", 0.25, 0.0), ("move", up_t, 0.45), ("rest", 1.2 + 0.1 * k, 0.0)]
    segs.append(("rest", 4.0, 0.0))
    return segs


def synthetic_recording():
    """(init line, sample lines) for harness.cpp."""
    rng = Lcg(20261102)
    tilt_x0, tilt_y = math.radians(25.0), math.radians(-12.0)
    gyro_bias = [0.35, -0.22, 0.12]   # deg/s, what CALIBRATE measured
    sample_lines = []
    t_ms = 1000
    seg_list = motion()
    total = sum(d for _, d, _ in seg_list)
    # sample times: 10 ms, every 13th period 11 ms (as the loop on the device sometimes does)
    times = []
    t = 0.0
    n = 0
    while t < total:
        times.append(t)
        n += 1
        t += 0.011 if n % 13 == 0 else 0.010
    bounds = []
    acc_t = 0.0
    for kind, dur, dz in seg_list:
        bounds.append((acc_t, acc_t + dur, kind, dz))
        acc_t += dur
    seg_i = 0
    first_grav = None
    for ts in times:
        while seg_i < len(bounds) - 1 and ts >= bounds[seg_i][1]:
            seg_i += 1
        t0, t1, kind, dz = bounds[seg_i]
        u = (ts - t0) / (t1 - t0)
        if kind == "move":
            T = t1 - t0
            az = dz * (math.pi / T) ** 2 / 2.0 * math.cos(math.pi * u)
            # tilt follows the motion: +4 deg at mid-movement, back to 0 at the ends
            tilt = math.radians(4.0) * math.sin(math.pi * u) * (1 if dz > 0 else -1)
            tilt_rate = math.radians(4.0) * math.pi / T * math.cos(math.pi * u) * (1 if dz > 0 else -1)
        else:
            az, tilt, tilt_rate = 0.0, 0.0, 0.0
        R = matmul(rot_x(tilt_x0 + tilt), rot_y(tilt_y))   # body -> world
        f_body = mat_t_vec(R, [0.0, 0.0, G + az])          # specific force in the body frame
        w_body = mat_t_vec(rot_y(tilt_y), [tilt_rate, 0.0, 0.0])  # rad/s: R^T dR/dt = [Ry^T (rate e_x)]x
        if first_grav is None:
            first_grav = mat_t_vec(R, [0.0, 0.0, 1.0])
        acc = [f + rng.noise(0.03) for f in f_body]
        gyr = [math.degrees(w) + b + rng.noise(0.06) for w, b in zip(w_body, gyro_bias)]
        sample_lines.append("%d,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,0,0,0,0,0,0,0,0" % (
            t_ms + round(ts * 1000.0), *acc, *gyr))
    init = "I,%.6f,%.7f,%.7f,%.7f,%.4f,%.4f,%.4f,0,0,0" % (G, *first_grav, *gyro_bias)
    return init, sample_lines


# ----------------------------------------------------------------------
# Comparison
# ----------------------------------------------------------------------

def output_digest(out):
    """SHA-256 of the replay output without the INFO line (it names the version)."""
    keep = [l for l in out.split("\n") if not l.startswith("INFO,")]
    return hashlib.sha256("\n".join(keep).encode()).hexdigest()


def compare_reps(got, want):
    """Same rep numbers; every value equal within one unit of its last printed digit."""
    problems = []
    if sorted(got) != sorted(int(k) for k in want):
        problems.append(f"reps {sorted(got)} != expected {sorted(int(k) for k in want)}")
        return problems
    for k, row in want.items():
        g = got[int(k)]
        for i, (a, b) in enumerate(zip(g, row)):
            dec = len(b.split(".")[1]) if "." in b else 0
            if abs(float(a) - float(b)) > 1.0001 * 10 ** -dec:
                problems.append(f"rep {k} column {i}: {a} != {b}")
    return problems


def check_synthetic(binary, update):
    init, lines = synthetic_recording()
    out = FR.run_raw(binary, init, lines)
    reps = FR.final_reps(out)
    in_sha = hashlib.sha256((init + "\n" + "\n".join(lines) + "\n").encode()).hexdigest()
    if update:
        os.makedirs(os.path.dirname(GOLDEN), exist_ok=True)
        with open(GOLDEN, "w") as f:
            json.dump({"firmware": "3.12.4", "note": "synthetic made-up motion, see regression.py",
                       "input_sha256": in_sha, "output_sha256": output_digest(out),
                       "R_columns": ["peakVelocity", "meanVelocity", "peakAcceleration", "meanAcceleration",
                                     "displacementM", "eccPeakVelocity", "eccMeanVelocity", "quality1",
                                     "correctionStatus"],
                       "reps": {str(k): v for k, v in sorted(reps.items())}}, f, indent=1)
            f.write("\n")
        print(f"wrote {GOLDEN}: {len(reps)} reps")
        return True
    with open(GOLDEN) as f:
        gold = json.load(f)
    ok = True
    if in_sha != gold["input_sha256"]:
        print("  NOTE: the synthetic input differs from the one the golden file was made with "
              "(Python float formatting?) - comparing reps anyway")
    problems = compare_reps(reps, gold["reps"])
    ident = output_digest(out) == gold["output_sha256"]
    print(f"synthetic: {len(reps)} reps (golden {len(gold['reps'])}), output byte-identical: {'yes' if ident else 'no'}")
    for p in problems:
        print("  " + p)
    ok &= not problems
    return ok


def check_real(binary, gdir):
    with open(os.path.join(gdir, "golden.json")) as f:
        gold = json.load(f)
    ok = True
    for e in gold["entries"]:
        path = e["log"] if os.path.isabs(e["log"]) else os.path.join(gdir, e["log"])
        with FR.quiet():
            d = FR.R.load_log(path, e["session"])
        _init, lines = FR.R.build_inputs(d, e["scale"])[:2]
        init = e["init"]
        in_sha = hashlib.sha256((init + "\n" + "\n".join(lines) + "\n").encode()).hexdigest()
        out = FR.run_raw(binary, init, lines)
        reps = FR.final_reps(out)
        problems = []
        if in_sha != e["input_sha256"]:
            problems.append("rebuilt input differs from the golden one (reconstruct.py changed?)")
        problems += compare_reps(reps, e["reps"])
        ident = output_digest(out) == e["output_sha256"]
        print(f"{e['name']}: {len(reps)} reps (golden {len(e['reps'])}), output byte-identical: "
              f"{'yes' if ident else 'no'}" + ("" if not problems else " - " + "; ".join(problems[:5])))
        ok &= not problems
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--update", action="store_true", help="rewrite the synthetic golden file")
    ap.add_argument("--real", metavar="DIR", help="folder with golden.json for recorded logs (outside git)")
    args = ap.parse_args()
    with tempfile.TemporaryDirectory() as tmp:
        binary = FR.build(tmp)
        ok = check_real(binary, args.real) if args.real else check_synthetic(binary, args.update)
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
