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
Full analysis of a validation dataset (PROTOCOL.md sections 5-8), in one
command:

  1. align   every set: ODKI log vs reference, from the signals alone
             (pipeline/align.py -> aligned/, sets_alignment.csv)
  2. pair    reference reps with ODKI reps (pipeline/pair_reps.py ->
             reps_paired.csv)
  3. stats   agreement, rep detection, test-retest and the pre-registered
             thresholds (pipeline/stats.py -> results/summary.json,
             RESULTS.md, plots)

Usage:
    python analyze.py --data DATA_DIR [--ref-scale K] [--no-plots]

Deterministic: the same dataset always gives the same numbers.
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "pipeline"))
import align  # noqa: E402
import pair_reps  # noqa: E402
import stats  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", required=True, help="dataset folder (PROTOCOL.md section 9)")
    ap.add_argument("--ref-scale", type=float, default=None,
                    help="reference scale factor (default: from the static verification, else 1.0)")
    ap.add_argument("--no-plots", action="store_true")
    args = ap.parse_args()
    print("1/3 align")
    align.run(args.data, None, args.ref_scale)
    print("2/3 pair")
    df = pair_reps.run(args.data)
    print(f"    {len(df)} rows -> reps_paired.csv")
    print("3/3 stats")
    s, out = stats.run(args.data, None, not args.no_plots)
    p = s["primary"]
    passed = [t for t, r in s["thresholds"].items() if r["pass"] and t.startswith("tier_")]
    print(f"MV final, squat + bench: n {p.get('n')}  bias {stats.fmt(p.get('bias'))}  "
          f"LoA {stats.fmt(p.get('loa_low_rm'))} to {stats.fmt(p.get('loa_high_rm'))}  "
          f"ICC(2,1) {stats.fmt(p.get('icc_2_1'))}  tiers passed: {', '.join(passed) or 'none'}")
    print(f"results in {out}/ (RESULTS.md, summary.json)")


if __name__ == "__main__":
    main()
