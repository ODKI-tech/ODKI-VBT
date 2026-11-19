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
VBT ODKI - minimal BLE example client
=====================================

The smallest complete client of the sensor's Bluetooth protocol
(Firmware/BLE_PROTOCOL.md), written as a starting point for third-party
apps (see DEVELOPERS.md in the repository root). It scans for the sensor
by its service UUID, connects, reads the protocol and firmware versions,
subscribes to SystemStatus, RepSummary and CorrectedCurve (and, with
--stream, Stream), optionally sends CALIBRATE and START, and prints every
rep as it is reported, overwriting earlier versions of the same rep.

It never writes Config and never starts a firmware update: both are
documented in the spec, and recommended to leave to the ODKI app.

The decoders below are plain functions over bytes, independent of the BLE
library, and --selftest checks them against the test vectors of the spec
without any hardware.

Usage:
    pip install bleak
    python vbt_ble_client.py --selftest
    python vbt_ble_client.py                       # listen only
    python vbt_ble_client.py --calibrate --start   # hold the sensor still first
    python vbt_ble_client.py --start --stream

Ctrl-C ends the session; if this script sent START, it sends STOP first.
"""

from __future__ import annotations

import argparse
import asyncio
import struct
import sys

SERVICE_UUID = "6c7325bb-1784-4d77-8b99-89ce838ac2ab"
STREAM_UUID = "6c7325bb-1784-4d77-8b99-89ce838ac2ac"
REP_SUMMARY_UUID = "6c7325bb-1784-4d77-8b99-89ce838ac2ad"
CORRECTED_CURVE_UUID = "6c7325bb-1784-4d77-8b99-89ce838ac2ae"
SYSTEM_STATUS_UUID = "6c7325bb-1784-4d77-8b99-89ce838ac2af"
COMMAND_UUID = "6c7325bb-1784-4d77-8b99-89ce838ac2b0"
PROTOCOL_VERSION_UUID = "6c7325bb-1784-4d77-8b99-89ce838ac2b3"
FIRMWARE_REVISION_UUID = "00002a26-0000-1000-8000-00805f9b34fb"
BATTERY_LEVEL_UUID = "00002a19-0000-1000-8000-00805f9b34fb"

SUPPORTED_PROTOCOL_MAJOR = 1
MIN_MTU = 56  # largest notification (CorrectedCurve, 53 B) + 3 B ATT header

CMD_STOP, CMD_START, CMD_CALIBRATE = 0x00, 0x01, 0x02
PHASES = {0: "idle", 1: "eccentric", 2: "concentric"}
STATUSES = {0: "provisional", 1: "rep-calibrated", 2: "corrected"}

# ---------------------------------------------------------------------------
# Decoders. Each reads the documented prefix and ignores trailing bytes (a
# later minor protocol version may append fields); a shorter packet raises
# ValueError.
# ---------------------------------------------------------------------------


def _unpack(fmt: str, data: bytes):
    size = struct.calcsize(fmt)
    if len(data) < size:
        raise ValueError(f"packet too short: {len(data)} < {size} bytes")
    return struct.unpack(fmt, bytes(data[:size]))


def decode_protocol_version(data: bytes) -> tuple[int, int]:
    return _unpack("<BB", data)


def decode_system_status(data: bytes) -> dict:
    calibrated, tracking = _unpack("<BB", data)
    return {"calibrated": bool(calibrated), "trackingActive": bool(tracking)}


def decode_stream(data: bytes) -> dict:
    v = _unpack("<I16hB", data)
    return {
        "timestampMs": v[0],
        "linAcc": tuple(x / 100 for x in v[1:4]),
        "worldAcc": tuple(x / 100 for x in v[4:7]),
        "worldVelX": v[7] / 1000, "worldVelY": v[8] / 1000,
        "refVelZ": v[9] / 1000,
        "worldPos": tuple(x / 1000 for x in v[10:13]),
        "quat": tuple(x / 10000 for x in v[13:17]),
        "phaseState": v[17],
    }


def decode_rep_summary(data: bytes) -> dict:
    v = _unpack("<B4hH2hBB", data)
    return {
        "repNumber": v[0],
        "peakVelocity": v[1] / 1000, "meanVelocity": v[2] / 1000,
        "peakAcceleration": v[3] / 100, "meanAcceleration": v[4] / 100,
        "displacementM": v[5] / 1000,
        "eccPeakVelocity": v[6] / 1000, "eccMeanVelocity": v[7] / 1000,
        "quality1": v[8], "correctionStatus": v[9],
    }


def decode_corrected_curve(data: bytes) -> dict:
    v = _unpack("<HHHBIH20h", data)
    count = min(v[3], 20)
    return {
        "bracketId": v[0], "chunkIndex": v[1], "totalChunks": v[2],
        "startTimestampMs": v[4], "sampleIntervalMs": v[5],
        "correctedVelZ": [x / 1000 for x in v[6:6 + count]],
    }


CONFIG_FIELDS = (
    "repDirection", "maxPlausibleVelocityMmps", "accZBiasIdleStillTimeMs",
    "gyroBiasIdleStillTimeMs", "accZBiasGyroMaxDegSx10",
    "accZBiasAccMagToleranceX1000", "flatGuardMaxVelocityMmps",
    "velocityOverrideFlatWindowSamples", "velocityFlatBandMmps",
    "maxVelocityFlatWindowSamples", "minVelocityFlatWindowSamples",
    "accelerationFlatBandX1000Mps2", "maxAccelerationFlatWindowSamples",
    "minAccelerationFlatWindowSamples", "windowSaturationPeakVelocityMmps",
    "phaseStartVelocityMmps", "minPhaseDurationMs", "maxPhaseDurationMs",
    "phaseLookbackSamples", "reversalConfirmSamples", "emaAlphaX1000",
    "minCrossingExcursionMmps", "minCrossingDurationMs",
)


def decode_config_raw(data: bytes) -> dict:
    """Raw wire integers of the Config characteristic (read-only use here)."""
    return dict(zip(CONFIG_FIELDS, _unpack("<BHHHHHHBHBBHBBHHHHBBHhH", data)))


# ---------------------------------------------------------------------------
# Self-test against the test vectors in BLE_PROTOCOL.md.
# ---------------------------------------------------------------------------

VECTOR_CONFIG = bytes.fromhex(
    "00a00f2c012c0178007800c8001e280014022c01"
    "14022003c8005e0188130c032c01e2ff2c01")
VECTOR_STREAM = bytes.fromhex(
    "c0d401000a00e7ff96000500f6ffa000ecff0f00"
    "c201f6ff050006ffde26f401d4fe200302")
VECTOR_REP_PROVISIONAL = bytes.fromhex("03b6030802a401d2002602200390015f00")
VECTOR_REP_CORRECTED = bytes.fromhex("03ac031202a401d7001c022a039a016402")


def selftest() -> int:
    def close(a, b):
        return abs(a - b) < 1e-9

    checks = []
    cfg = decode_config_raw(VECTOR_CONFIG)
    checks.append(("config size", len(VECTOR_CONFIG) == 38))
    checks.append(("config values", (
        cfg["repDirection"], cfg["maxPlausibleVelocityMmps"],
        cfg["accZBiasIdleStillTimeMs"], cfg["velocityOverrideFlatWindowSamples"],
        cfg["maxPhaseDurationMs"], cfg["minCrossingExcursionMmps"],
        cfg["minCrossingDurationMs"]) == (0, 4000, 300, 30, 5000, -30, 300)))

    s = decode_stream(VECTOR_STREAM)
    checks.append(("stream size", len(VECTOR_STREAM) == 37))
    checks.append(("stream values",
                   s["timestampMs"] == 120000 and s["phaseState"] == 2
                   and all(close(a, b) for a, b in zip(s["linAcc"], (0.10, -0.25, 1.50)))
                   and all(close(a, b) for a, b in zip(s["worldAcc"], (0.05, -0.10, 1.60)))
                   and close(s["worldVelX"], -0.020) and close(s["worldVelY"], 0.015)
                   and close(s["refVelZ"], 0.450)
                   and all(close(a, b) for a, b in zip(s["worldPos"], (-0.010, 0.005, -0.250)))
                   and all(close(a, b) for a, b in zip(s["quat"], (0.9950, 0.0500, -0.0300, 0.0800)))))

    r1 = decode_rep_summary(VECTOR_REP_PROVISIONAL)
    r2 = decode_rep_summary(VECTOR_REP_CORRECTED)
    checks.append(("rep provisional", (
        r1["repNumber"], r1["quality1"], r1["correctionStatus"]) == (3, 95, 0)
        and close(r1["peakVelocity"], 0.950) and close(r1["meanVelocity"], 0.520)
        and close(r1["peakAcceleration"], 4.20) and close(r1["meanAcceleration"], 2.10)
        and close(r1["displacementM"], 0.550) and close(r1["eccPeakVelocity"], 0.800)
        and close(r1["eccMeanVelocity"], 0.400)))
    checks.append(("rep corrected", (
        r2["repNumber"], r2["quality1"], r2["correctionStatus"]) == (3, 100, 2)
        and close(r2["peakVelocity"], 0.940) and close(r2["meanVelocity"], 0.530)
        and close(r2["meanAcceleration"], 2.15) and close(r2["displacementM"], 0.540)
        and close(r2["eccPeakVelocity"], 0.810) and close(r2["eccMeanVelocity"], 0.410)))

    checks.append(("trailing bytes ignored",
                   decode_rep_summary(VECTOR_REP_CORRECTED + b"\xff\xff") == r2))
    try:
        decode_rep_summary(VECTOR_REP_CORRECTED[:16])
        checks.append(("short packet rejected", False))
    except ValueError:
        checks.append(("short packet rejected", True))
    checks.append(("protocol version", decode_protocol_version(b"\x01\x00\x07") == (1, 0)))

    failed = [name for name, ok in checks if not ok]
    for name, ok in checks:
        print(f"{'ok  ' if ok else 'FAIL'} {name}")
    return 1 if failed else 0


# ---------------------------------------------------------------------------
# BLE session (needs `pip install bleak`).
# ---------------------------------------------------------------------------


def print_rep(rep: dict) -> None:
    status = STATUSES.get(rep["correctionStatus"], f"unknown({rep['correctionStatus']})")
    print(f"rep {rep['repNumber']:3d}  peak {rep['peakVelocity']:.3f} m/s  "
          f"mean {rep['meanVelocity']:.3f} m/s  disp {rep['displacementM']:.3f} m  "
          f"q {rep['quality1']:3d}  [{status}]")


async def run(args) -> int:
    try:
        from bleak import BleakClient, BleakScanner
    except ImportError:
        print("This needs the bleak library: pip install bleak", file=sys.stderr)
        return 2

    print("Scanning for a sensor (by service UUID)...")
    device = await BleakScanner.find_device_by_filter(
        lambda d, ad: SERVICE_UUID in [u.lower() for u in ad.service_uuids],
        timeout=args.timeout)
    if device is None:
        print("No sensor found. Is it switched on and not connected to another app?")
        return 1

    async with BleakClient(device) as client:
        print(f"Connected to {device.address} ({device.name})")
        mtu = getattr(client, "mtu_size", None)
        if mtu is not None and mtu < MIN_MTU:
            print(f"Warning: MTU {mtu} < {MIN_MTU}, long packets will arrive split.")

        if client.services.get_characteristic(PROTOCOL_VERSION_UUID) is None:
            print("Protocol version: not exposed (firmware from before protocol versioning)")
        else:
            major, minor = decode_protocol_version(
                await client.read_gatt_char(PROTOCOL_VERSION_UUID))
            print(f"Protocol version: {major}.{minor}")
            if major > SUPPORTED_PROTOCOL_MAJOR:
                print(f"This client understands protocol {SUPPORTED_PROTOCOL_MAJOR}.x only.")
                return 1
        fw = await client.read_gatt_char(FIRMWARE_REVISION_UUID)
        print(f"Firmware: {bytes(fw).decode('utf-8', 'replace')}")
        battery = await client.read_gatt_char(BATTERY_LEVEL_UUID)
        print(f"Battery: {battery[0]}%")

        reps: dict[int, dict] = {}

        def on_status(_, data):
            print(f"Status: {decode_system_status(data)}")

        def on_rep(_, data):
            rep = decode_rep_summary(data)
            reps[rep["repNumber"]] = rep  # overwrite by repNumber, never append
            print_rep(rep)

        def on_curve(_, data):
            c = decode_corrected_curve(data)
            print(f"corrected curve: bracket {c['bracketId']} chunk "
                  f"{c['chunkIndex'] + 1}/{c['totalChunks']}, "
                  f"{len(c['correctedVelZ'])} samples from t={c['startTimestampMs']} ms")

        def on_stream(_, data):
            s = decode_stream(data)
            print(f"t={s['timestampMs']} ms  v={s['refVelZ']:+.3f} m/s (raw)  "
                  f"{PHASES.get(s['phaseState'], '?')}")

        await client.start_notify(SYSTEM_STATUS_UUID, on_status)
        await client.start_notify(REP_SUMMARY_UUID, on_rep)
        await client.start_notify(CORRECTED_CURVE_UUID, on_curve)
        if args.stream:
            await client.start_notify(STREAM_UUID, on_stream)
        # The push at connect time happens before anyone subscribed: read it.
        status = decode_system_status(await client.read_gatt_char(SYSTEM_STATUS_UUID))
        print(f"Status: {status}")

        if args.calibrate:
            if status["trackingActive"]:
                print("Tracking is active: CALIBRATE would be ignored, skipping it.")
            else:
                print("CALIBRATE: keep the sensor still...")
                await client.write_gatt_char(COMMAND_UUID, bytes([CMD_CALIBRATE]), response=True)
                await asyncio.sleep(3)
        started = False
        if args.start:
            await client.write_gatt_char(COMMAND_UUID, bytes([CMD_START]), response=True)
            started = True
            print("START sent. Lift; Ctrl-C to stop.")
        try:
            while client.is_connected:
                await asyncio.sleep(0.5)
        except (KeyboardInterrupt, asyncio.CancelledError):
            pass
        finally:
            if started and client.is_connected:
                await client.write_gatt_char(COMMAND_UUID, bytes([CMD_STOP]), response=True)
                print("STOP sent.")
        print(f"{len(reps)} reps received.")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Minimal ODKI VBT BLE client.")
    parser.add_argument("--selftest", action="store_true",
                        help="check the decoders against the spec's test vectors and exit")
    parser.add_argument("--calibrate", action="store_true", help="send CALIBRATE first")
    parser.add_argument("--start", action="store_true", help="send START (and STOP on exit)")
    parser.add_argument("--stream", action="store_true", help="also print the live Stream")
    parser.add_argument("--timeout", type=float, default=10.0, help="scan timeout, s")
    args = parser.parse_args()
    if args.selftest:
        return selftest()
    try:
        return asyncio.run(run(args))
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
