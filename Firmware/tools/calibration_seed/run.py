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
Host tests for the CALIBRATE attitude seed fix (v3.11.33).

  python run.py [--ref COMMIT]

1. seed_test.cpp: for 0..180 deg of tilt (incl. 90 deg and near-180 deg),
   24 azimuths and 3 magnitudes, the gravity direction implied by the
   quaternion alignToGravity() seeds equals the measured accelerometer
   direction within 1e-5 (Madgwick and Fusion), and stays < 1 deg while still.
2. update_regression.cpp: MadgwickAHRS::update() is bit-for-bit unchanged -
   the same 500 000-step pseudo-random run hashed against the current
   MadgwickAHRS.cpp and against the one at --ref (default b3ea20e, the
   v3.11.31 tree on main, before the fix).
Requires clang/clang++ (or gcc/g++) and git.
"""

import argparse
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
FW = os.path.join(os.path.dirname(TOOLS), "VBT_Quaternions")
STUBS = os.path.join(TOOLS, "host_replay", "stubs")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ref", default="b3ea20e", help="git commit with the pre-fix MadgwickAHRS.cpp")
    args = ap.parse_args()
    cc = shutil.which("clang") or shutil.which("gcc")
    cxx = shutil.which("clang++") or shutil.which("g++")
    if not cc or not cxx:
        sys.exit("No C/C++ compiler found.")
    ok = True
    with tempfile.TemporaryDirectory() as tmp:
        objs = []
        for f in ("FusionAhrs", "FusionBias", "FusionCompass", "FusionConvention", "FusionRemap"):
            o = os.path.join(tmp, f + ".o")
            subprocess.run([cc, "-std=c99", "-O2", "-c", os.path.join(FW, "src", "Fusion", f + ".c"), "-o", o], check=True)
            objs.append(o)
        exe = os.path.join(tmp, "seed_test")
        subprocess.run([cxx, "-std=c++17", "-O2", "-w", "-I", STUBS, "-I", FW, "-DODKI_FILTER_RUNTIME_SELECT=1",
                        os.path.join(HERE, "seed_test.cpp"), os.path.join(FW, "MadgwickAHRS.cpp"),
                        os.path.join(FW, "FusionOrientation.cpp"), *objs, "-o", exe], check=True)
        r = subprocess.run([exe], capture_output=True, text=True)
        print("== seed test\n" + r.stdout.rstrip())
        ok &= r.returncode == 0

        refdir = os.path.join(tmp, "ref")
        os.makedirs(refdir)
        repo = subprocess.run(["git", "-C", HERE, "rev-parse", "--show-toplevel"], capture_output=True, text=True,
                              check=True).stdout.strip()
        for f in ("MadgwickAHRS.cpp", "MadgwickAHRS.h"):
            src = subprocess.run(["git", "-C", repo, "show", f"{args.ref}:Firmware/VBT_Quaternions/{f}"],
                                 capture_output=True, text=True, check=True).stdout
            with open(os.path.join(refdir, f), "w") as fh:
                fh.write(src)
        hashes = {}
        for name, d in (("current", FW), (f"ref {args.ref}", refdir)):
            e = os.path.join(tmp, "upd_" + name.split()[0])
            subprocess.run([cxx, "-std=c++17", "-O2", "-w", "-I", d, "-I", STUBS,
                            os.path.join(HERE, "update_regression.cpp"), os.path.join(d, "MadgwickAHRS.cpp"), "-o", e],
                           check=True)
            hashes[name] = subprocess.run([e], capture_output=True, text=True, check=True).stdout.strip()
        print("== MadgwickAHRS::update() regression (500 000 steps, hash of every quaternion)")
        for k, v in hashes.items():
            print(f"  {k:>14}: {v}")
        same = len(set(hashes.values())) == 1
        print("  " + ("IDENTICAL" if same else "DIFFERENT"))
        ok &= same
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
