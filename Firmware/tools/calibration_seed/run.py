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
Host test of the CALIBRATE attitude seed (v3.11.33 fix; Fusion only since
v3.12.0).

  python run.py

seed_test.cpp: for 0..180 deg of tilt (incl. 90 deg and near-180 deg), 24
azimuths and 3 magnitudes, the gravity direction implied by the quaternion
FusionOrientation::alignToGravity() seeds equals the measured accelerometer
direction within 1e-5, and stays < 1 deg while still for 3 s. Also prints the
error the pre-v3.11.33 (mirrored) seed had, for reference.
Requires clang/clang++ (or gcc/g++).
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
    argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter).parse_args()
    cc = shutil.which("clang") or shutil.which("gcc")
    cxx = shutil.which("clang++") or shutil.which("g++")
    if not cc or not cxx:
        sys.exit("No C/C++ compiler found.")
    with tempfile.TemporaryDirectory() as tmp:
        objs = []
        for f in ("FusionAhrs", "FusionBias", "FusionCompass", "FusionConvention", "FusionRemap"):
            o = os.path.join(tmp, f + ".o")
            subprocess.run([cc, "-std=c99", "-O2", "-c", os.path.join(FW, "src", "Fusion", f + ".c"), "-o", o], check=True)
            objs.append(o)
        exe = os.path.join(tmp, "seed_test")
        subprocess.run([cxx, "-std=c++17", "-O2", "-w", "-I", STUBS, "-I", FW,
                        os.path.join(HERE, "seed_test.cpp"), os.path.join(FW, "FusionOrientation.cpp"), *objs,
                        "-o", exe], check=True)
        r = subprocess.run([exe], capture_output=True, text=True)
    print("== seed test\n" + r.stdout.rstrip())
    return r.returncode


if __name__ == "__main__":
    sys.exit(main())
