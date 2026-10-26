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
Serial capture of the ODKI sensor(s) for one validation set (PROTOCOL.md
section 5).

Reads the USB-serial log of one or more ODKI sensors on the bar. The
reference linear encoder is recorded separately by its own software and
converted afterwards with import_ref.py. For a physical set <set> and sensor
unit <unit> this writes:

  raw/odki/<set>_<unit>.log          the ODKI log, BYTE-FOR-BYTE as received
                                     (vbt_log_viewer.py and host_replay read it as-is)
  raw/odki/<set>_<unit>.sidecar.csv  line_no,host_ns,kind - arrival time of every
                                     ODKI line on the PC's monotonic clock

<set>_<unit> is the set_id used everywhere downstream (sets.csv).

host_ns is the PC's monotonic clock when the line arrives, RELATIVE to the
start of the capture (never an epoch time: the raw files carry no date or
time of day, PROTOCOL.md section 10). It is kept for diagnostics (USB gaps);
the alignment with the reference uses the signals only (align.py).

Usage:
    python capture.py --odki U1=/dev/cu.usbmodem1101 --odki U2=/dev/cu.usbmodem1201 \
        --data DATA_DIR --set L01-D1-SQ-060-S1 [--start] [--duration 60]

  --start      sends START to every sensor when capture begins and STOP when it
               ends (Ctrl+C or --duration), same plain-text commands as
               vbt_live_monitor.py. Calibrate beforehand (CALIBRATE, bar still).

Requires pyserial (pip install pyserial) - the only extra dependency of the
pipeline, needed here only.
"""

import argparse
import os
import sys
import threading
import time


def open_port(port, baud):
    try:
        import serial
    except ImportError:
        sys.exit("capture.py requires pyserial: pip install pyserial")
    return serial.Serial(port, baud, timeout=0.2)


def odki_reader(ser, log_path, sidecar_path, stop, stats, t0_ns):
    with open(log_path, "wb") as log, open(sidecar_path, "w") as side:
        side.write("line_no,host_ns,kind\n")
        n = 0
        while not stop.is_set():
            raw = ser.readline()
            if not raw:
                continue
            host_ns = time.monotonic_ns() - t0_ns
            log.write(raw)
            kind = raw[:2].decode("ascii", "replace").rstrip(",\r\n") or "?"
            side.write(f"{n},{host_ns},{kind}\n")
            n += 1
            if raw.startswith(b"S,"):
                stats["odki_samples"] += 1
        stats["odki_lines"] = n


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--odki", required=True, action="append", metavar="UNIT=PORT",
                    help="unit id and serial port of an ODKI sensor (repeat for several units)")
    ap.add_argument("--data", required=True, help="dataset directory (raw/ is created inside)")
    ap.add_argument("--set", required=True, help="physical set id, e.g. L01-D1-SQ-060-S1")
    ap.add_argument("--odki-baud", type=int, default=115200)
    ap.add_argument("--start", action="store_true", help="send START/STOP to the sensor")
    ap.add_argument("--duration", type=float, default=None, help="stop after N seconds (default: Ctrl+C)")
    args = ap.parse_args()

    odki_dir = os.path.join(args.data, "raw", "odki")
    os.makedirs(odki_dir, exist_ok=True)
    units = []
    for spec in args.odki:
        if "=" not in spec:
            sys.exit(f"--odki expects UNIT=PORT, got {spec!r}")
        unit, port = spec.split("=", 1)
        set_id = f"{args.set}_{unit}"
        units.append((set_id, port, os.path.join(odki_dir, f"{set_id}.log"),
                      os.path.join(odki_dir, f"{set_id}.sidecar.csv")))
    for u in units:
        if os.path.exists(u[2]):
            sys.exit(f"{u[2]} already exists - one file per set, pick a new --set")

    odki_ports = [open_port(u[1], args.odki_baud) for u in units]
    stop = threading.Event()
    t0_ns = time.monotonic_ns()
    unit_stats = [{"odki_samples": 0, "odki_lines": 0} for _ in units]
    threads = [threading.Thread(target=odki_reader, args=(ser, u[2], u[3], stop, st, t0_ns), daemon=True)
               for ser, u, st in zip(odki_ports, units, unit_stats)]
    for th in threads:
        th.start()
    time.sleep(0.5)
    if args.start:
        for ser in odki_ports:
            ser.write(b"START\n")
    print(f"capturing set {args.set} - Ctrl+C to stop")
    t0 = time.monotonic()
    try:
        while args.duration is None or time.monotonic() - t0 < args.duration:
            time.sleep(1.0)
            odki_n = " ".join(f"{u[0]}:{st['odki_samples']}" for u, st in zip(units, unit_stats))
            print(f"\r  {time.monotonic() - t0:6.1f} s  ODKI samples {odki_n}", end="", flush=True)
    except KeyboardInterrupt:
        pass
    if args.start:
        for ser in odki_ports:
            ser.write(b"STOP\n")
        time.sleep(0.5)  # let REC_STOP and the last R/B rows arrive
    stop.set()
    for th in threads:
        th.join(timeout=2)
    for ser in odki_ports:
        ser.close()
    print("\nwrote " + "\n      ".join([u[2] for u in units] + [u[3] for u in units]))
    dur = time.monotonic() - t0
    for u, st in zip(units, unit_stats):
        if st["odki_samples"] < 0.95 * 100 * dur and args.start:
            print(f"WARNING: {u[0]}: {st['odki_samples']} ODKI samples in {dur:.1f} s (< 95% of 100 Hz)")


if __name__ == "__main__":
    main()
