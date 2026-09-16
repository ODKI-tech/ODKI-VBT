# ODKI VBT — BLE Protocol Reference

**License note:** unlike the rest of this repository, *this document* is released under **CC0 1.0** (public domain dedication, see the [License](#license) section at the end) — deliberately, so any third-party app or integration can implement a client against it with zero legal uncertainty, commercial or not.

This is the byte-exact specification of the sensor's Bluetooth Low Energy interface: every characteristic UUID, every packet's layout down to the byte offset, and every scale factor needed to turn a raw value back into a physical one. It's written so a **third-party client** — a script, a different mobile app, a home-automation integration, anything — can talk to the device without needing to read the firmware source at all.

For *why* the protocol is shaped this way (why `Stream` is always raw, why `CorrectedCurve` exists and arrives late, why `RepSummary` can repeat), see the [BleServer chapter](DOCUMENTATION.md#bleserver) of the main firmware documentation — this document only covers *what's on the wire*, not the reasoning behind it. Reference implementation: `BleServer.h`/`.cpp` in this repository.

## Conventions

- **Byte order:** little-endian throughout — the nRF52840's native order, and standard for BLE.
- **No padding:** every packet struct in the firmware is declared `#pragma pack(push, 1)`, so byte offsets below are a plain cumulative sum of field sizes, with nothing inserted between fields.
- **Fixed-point physical values:** most quantities travel as a scaled integer, not a float: `physical_value = raw_value / scale`. This is called out per field below. A `scale` of `1` means the integer *is* the value directly (no conversion needed) — this is used for millisecond durations and millimetre-per-second velocities, so the wire value is already in a convenient integer unit.
- **Security:** every custom characteristic is open — `SECMODE_OPEN`, no pairing or bonding required for either read/notify or write access. Any BLE-capable device within range can connect, read live data, and issue `Command`/`Config` writes (including stopping tracking or changing algorithm parameters) without authentication. Keep this in mind if deploying in a setting where that matters.

## Advertising & connecting

| | |
|---|---|
| Device name | `VBT-Sensor` |
| Advertised service | the custom service UUID below |
| Role | Peripheral only (never Central) — exactly one simultaneous connection |
| Pairing | None required |

**MTU is mandatory to negotiate.** The default BLE MTU (23 bytes, 20 usable) is smaller than every custom packet except `Command`, `SystemStatus`, and `Battery`. Request the largest MTU the stack allows (the firmware itself requests up to 247 bytes on connect) before expecting `Stream`, `RepSummary`, `CorrectedCurve`, or `Config` to work — a client that skips this will simply never receive a successful notify for those characteristics.

## All UUIDs at a glance

| Name | UUID | Properties | Size |
|---|---|---|---|
| ODKI VBT Service | `6c7325bb-1784-4d77-8b99-89ce838ac2ab` | — | — |
| Stream | `6c7325bb-1784-4d77-8b99-89ce838ac2ac` | Notify | 37 B |
| RepSummary | `6c7325bb-1784-4d77-8b99-89ce838ac2ad` | Notify | 17 B |
| CorrectedCurve | `6c7325bb-1784-4d77-8b99-89ce838ac2ae` | Notify | 53 B |
| SystemStatus | `6c7325bb-1784-4d77-8b99-89ce838ac2af` | Read, Notify | 2 B |
| Command | `6c7325bb-1784-4d77-8b99-89ce838ac2b0` | Write | 1 B |
| Config | `6c7325bb-1784-4d77-8b99-89ce838ac2b2` | Read, Write | 39 B |
| Battery Service | `0x180F` (standard) | — | — |
| Battery Level | `0x2A19` (standard) | Read, Notify | 1 B |
| Device Information Service | `0x180A` (standard) | — | — |
| Firmware Revision String | `0x2A26` (standard) | Read | variable (UTF-8) |
| Buttonless DFU Service | `00001530-1212-efde-1523-785feabcd123` | — | — |
| DFU Control | `00001531-1212-efde-1523-785feabcd123` | Write, Notify | 1 B (command) |

(The custom UUIDs all share the base `6c7325bb-1784-4d77-8b99-89ce838ac2ab`, incrementing only the last byte — `0xb1` is intentionally absent from the sequence, freed when the original custom Battery and Firmware-Version characteristics were replaced by the two standard SIG services above.)

---

## Stream — `…c2ac`, Notify, 37 bytes

Live telemetry, sent only while tracking is active, decimated to 20Hz (`Config::STREAM_DECIMATION_FACTOR`). Automatically disabled while a raw serial debug capture is running.

| Offset | Size | Field | Type | Scale | Unit |
|---|---|---|---|---|---|
| 0 | 4 | `timestampMs` | uint32 | 1 | ms |
| 4 | 2 | `linAccX` | int16 | ÷100 | m/s² (sensor frame) |
| 6 | 2 | `linAccY` | int16 | ÷100 | m/s² |
| 8 | 2 | `linAccZ` | int16 | ÷100 | m/s² |
| 10 | 2 | `worldAccX` | int16 | ÷100 | m/s² (world frame) |
| 12 | 2 | `worldAccY` | int16 | ÷100 | m/s² |
| 14 | 2 | `worldAccZ` | int16 | ÷100 | m/s² |
| 16 | 2 | `worldVelX` | int16 | ÷1000 | m/s (debug/streaming only) |
| 18 | 2 | `worldVelY` | int16 | ÷1000 | m/s |
| 20 | 2 | `refVelZ` | int16 | ÷1000 | m/s — **raw**, live-re-anchored vertical velocity; never the retroactively-corrected value (see `CorrectedCurve` below) |
| 22 | 2 | `worldPosX` | int16 | ÷1000 | m |
| 24 | 2 | `worldPosY` | int16 | ÷1000 | m |
| 26 | 2 | `worldPosZ` | int16 | ÷1000 | m |
| 28 | 2 | `quatW` | int16 | ÷10000 | dimensionless, [-1, 1] |
| 30 | 2 | `quatX` | int16 | ÷10000 | dimensionless, [-1, 1] |
| 32 | 2 | `quatY` | int16 | ÷10000 | dimensionless, [-1, 1] |
| 34 | 2 | `quatZ` | int16 | ÷10000 | dimensionless, [-1, 1] |
| 36 | 1 | `phaseState` | uint8 | direct | `0`=Idle, `1`=Eccentric, `2`=Concentric |

Python decode: `struct.unpack("<I16hB", data)` — field order matches the table above.

---

## RepSummary — `…c2ad`, Notify, 17 bytes

One packet per completed rep. **The same `repNumber` can arrive more than once** as `correctionStatus` improves — a client must overwrite its stored copy of that rep by `repNumber`, never append a duplicate.

| Offset | Size | Field | Type | Scale | Unit |
|---|---|---|---|---|---|
| 0 | 1 | `repNumber` | uint8 | direct | 1-based |
| 1 | 2 | `peakVelocity` | int16 | ÷1000 | m/s, concentric |
| 3 | 2 | `meanVelocity` | int16 | ÷1000 | m/s, concentric |
| 5 | 2 | `peakAcceleration` | int16 | ÷100 | m/s², concentric |
| 7 | 2 | `meanAcceleration` | int16 | ÷100 | m/s², concentric |
| 9 | 2 | `displacementM` | uint16 | ÷1000 | m, concentric-only, always ≥0 |
| 11 | 2 | `eccPeakVelocity` | int16 | ÷1000 | m/s, `0` if this rep had no eccentric phase |
| 13 | 2 | `eccMeanVelocity` | int16 | ÷1000 | m/s, `0` if this rep had no eccentric phase |
| 15 | 1 | `quality1` | uint8 | direct | 0–100, kinematic confidence score |
| 16 | 1 | `correctionStatus` | uint8 | direct | `0`=Provisional, `1`=RepCalibrated, `2`=Corrected |

Python decode: `struct.unpack("<B4hH2hBB", data)`.

**`correctionStatus` in detail:**

| Value | Name | Meaning |
|---|---|---|
| `0` | Provisional | Scored against an extrapolated drift-rate estimate; the bracket containing this rep hasn't closed yet. Treat the numbers as a best-effort live estimate, not final. |
| `1` | RepCalibrated | Scored against a rep-cycle-based calibration (no bracket needed) — used when no bracket has closed recently, typically during a continuous, gapless set. |
| `2` | Corrected | Scored against a bracket's *measured* drift. Final — this `repNumber` will not be re-reported again. |

---

## CorrectedCurve — `…c2ae`, Notify, 53 bytes

Retroactively drift-corrected velocity for a time range that was already streamed as raw via `Stream`. Sent in chunks when the bracket covering that range closes; a bracket can span thousands of samples, far more than fit in one BLE packet. A client should replace the portion of its live graph already drawn from `Stream` in `[startTimestampMs, startTimestampMs + sampleCount × sampleIntervalMs)` with these values.

| Offset | Size | Field | Type | Scale | Unit |
|---|---|---|---|---|---|
| 0 | 2 | `bracketId` | uint16 | direct | increments on every bracket close |
| 2 | 2 | `chunkIndex` | uint16 | direct | 0-based, within this bracket |
| 4 | 2 | `totalChunks` | uint16 | direct | how many chunks this bracket was split into |
| 6 | 1 | `sampleCount` | uint8 | direct | how many of the 20 slots below are valid |
| 7 | 4 | `startTimestampMs` | uint32 | direct | ms — timestamp of `correctedVelZ[0]`, same clock/scale as `Stream::timestampMs` |
| 11 | 2 | `sampleIntervalMs` | uint16 | direct | ms between consecutive samples (nominally 10ms at 100Hz) |
| 13 | 40 | `correctedVelZ[0..19]` | int16 × 20 | ÷1000 | m/s each |

Only the first `sampleCount` entries of `correctedVelZ` are meaningful — a bracket's final chunk is usually partial, and the remaining slots are zero-padding to be ignored.

Python decode: `struct.unpack("<HHHBIH20h", data)`.

To reconstruct which of the already-received `Stream` samples a chunk corrects, match on timestamp: sample `i` in the chunk corresponds to `t = startTimestampMs + i × sampleIntervalMs`.

---

## SystemStatus — `…c2af`, Read/Notify, 2 bytes

Pushed automatically on every new connection and on every state change; also independently readable at any time via a standard GATT Read.

| Offset | Size | Field | Type | Meaning |
|---|---|---|---|---|
| 0 | 1 | `calibrated` | uint8 | `0`/`1` |
| 1 | 1 | `trackingActive` | uint8 | `0`/`1` |

`Start` (see `Command` below) is rejected by the firmware while `calibrated = 0`; `Calibrate` is rejected while `trackingActive = 1`.

---

## Command — `…c2b0`, Write, 1 byte

A single control byte:

| Value | Command | Effect |
|---|---|---|
| `0x00` | STOP | Stops tracking (if active). No effect otherwise. |
| `0x01` | START | Starts tracking. Ignored if the device isn't calibrated yet. |
| `0x02` | CALIBRATE | Blocking ~2s orientation calibration — the device must be held still. Ignored while tracking is active (stop first). |

Any other value, or a write shorter than 1 byte, is silently ignored.

---

## Config — `…c2b2`, Read/Write, 39 bytes

Runtime-adjustable algorithm parameters, mirroring the firmware's `RuntimeConfig` struct field-for-field. **Not persisted to flash** — the firmware always boots from its compiled defaults, so a client that wants non-default settings must re-send this characteristic on every new connection. Reading it returns whatever configuration is currently active (defaults, until a client writes otherwise). A write is echoed back into the Read value immediately, before the firmware has necessarily applied it internally.

See [`Firmware/DOCUMENTATION.md`](DOCUMENTATION.md#runtimeconfig-field-reference) for what each field actually controls in the algorithm — this table is the wire layout only.

| Offset | Size | Field | Type | Scale | Unit |
|---|---|---|---|---|---|
| 0 | 1 | `repDirection` | uint8 | direct | `0`=Up, `1`=Down |
| 1 | 2 | `maxPlausibleVelocityMmps` | uint16 | direct | mm/s |
| 3 | 2 | `accZBiasIdleStillTimeMs` | uint16 | direct | ms |
| 5 | 2 | `gyroBiasIdleStillTimeMs` | uint16 | direct | ms |
| 7 | 2 | `accZBiasGyroMaxDegSx10` | uint16 | ÷10 | degrees/s |
| 9 | 2 | `accZBiasAccMagToleranceX1000` | uint16 | ÷1000 | m/s² |
| 11 | 2 | `flatGuardMaxVelocityMmps` | uint16 | direct | mm/s |
| 13 | 2 | `flatGuardOverrideStillTimeMs` | uint16 | direct | ms |
| 15 | 2 | `velocityFlatBandMmps` | uint16 | direct | mm/s |
| 17 | 1 | `maxVelocityFlatWindowSamples` | uint8 | direct | samples |
| 18 | 1 | `minVelocityFlatWindowSamples` | uint8 | direct | samples |
| 19 | 2 | `accelerationFlatBandX1000Mps2` | uint16 | ÷1000 | m/s² |
| 21 | 1 | `maxAccelerationFlatWindowSamples` | uint8 | direct | samples |
| 22 | 1 | `minAccelerationFlatWindowSamples` | uint8 | direct | samples |
| 23 | 2 | `windowSaturationPeakVelocityMmps` | uint16 | direct | mm/s |
| 25 | 2 | `phaseStartVelocityMmps` | uint16 | direct | mm/s |
| 27 | 2 | `minPhaseDurationMs` | uint16 | direct | ms |
| 29 | 2 | `maxPhaseDurationMs` | uint16 | direct | ms |
| 31 | 1 | `phaseLookbackSamples` | uint8 | direct | samples |
| 32 | 1 | `reversalConfirmSamples` | uint8 | direct | samples |
| 33 | 2 | `emaAlphaX1000` | uint16 | ÷1000 | dimensionless, [0, 1] |
| 35 | 2 | `minCrossingExcursionMmps` | **int16** | direct | mm/s, always ≤ 0 |
| 37 | 2 | `minCrossingDurationMs` | uint16 | direct | ms |

Every field is unsigned except `minCrossingExcursionMmps` at offset 35 — the one signed field in the packet.

Python decode: `struct.unpack("<B8HBBHBBHHHHBBHhH", data)` — 23 fields, matching the table row order exactly (`H`×8, then `BB`, `H`, `BB`, `H`×4, `BB`, `H`, `h`, `H`).

A write shorter than 39 bytes is ignored entirely (the whole packet, not just the missing tail).

**v3.11.24:** `debugLogEnabled` (offset 39) was removed — the packet shrank from 40 to 39 bytes. Whether the sensor prints its raw serial log or streams over BLE is now decided automatically from whether a USB-serial connection is open (see [`Firmware/DOCUMENTATION.md`](DOCUMENTATION.md#lab-data-capture-serial-log)), not a field a client sends. A client built against the pre-v3.11.24 40-byte layout will have its Config writes silently ignored by v3.11.24+ firmware (`len < sizeof(RuntimeConfigPacket)` now means `len < 39`, not `< 40`) — update the packet size before talking to updated firmware.

---

## Battery Service (standard, `0x180F`)

**Battery Level** (`0x2A19`, Read/Notify, 1 byte): `percent`, uint8, direct, 0–100. Updated by the firmware every `Config::BATTERY_CHECK_INTERVAL_MS` (30s) and once immediately on boot.

## Device Information Service (standard, `0x180A`)

**Firmware Revision String** (`0x2A26`, Read, variable-length UTF-8): the firmware version as `"MAJOR.MINOR.PATCH"` (e.g. `"3.11.7"`). Set once at boot, static for the whole connection — read it to check whether an OTA update is available before offering one to the user.

## Firmware update (DFU)

Updating the firmware is a two-stage handoff, and only the first stage is custom to this project:

1. **Enter bootloader mode.** Write `0x01` to the DFU Control characteristic (`00001531-1212-efde-1523-785feabcd123`, under the Buttonless DFU service `00001530-1212-efde-1523-785feabcd123`). The device disconnects and reboots into the factory Adafruit bootloader.
2. **Transfer the firmware.** The bootloader now exposes Nordic's standard **Secure DFU** service — a public, well-documented Nordic protocol, unrelated to anything custom in this repository. Use an existing DFU-capable client library rather than implementing this stage from scratch (the companion app uses `nordic_dfu` on Flutter; equivalents exist for most platforms — e.g. Nordic's own `nRF Connect Device Manager` / `iOS-DFU-Library` / `Android-DFU-Library`).

There is no `.h`/`.cpp` in this firmware implementing the second stage — it's entirely the stock bootloader's responsibility, which is also why a device stuck mid-update can generally be recovered by simply reflashing over USB rather than needing another BLE transfer.

## License

This document (`BLE_PROTOCOL.md` only — not the firmware source it describes, see [`LICENSE-FIRMWARE`](LICENSE-FIRMWARE) for that) is released under **CC0 1.0 Universal**: to the extent possible under law, the author(s) have waived all copyright and related or neighboring rights to this document. You may copy, modify, distribute, and use it — including commercially, including without attribution — with no conditions at all.

Full legal text: <https://creativecommons.org/publicdomain/zero/1.0/legalcode>

This applies only to the protocol *specification* in this file — implementing a compatible client (an app, a script, an integration) based on it is unrestricted. It does not relicense the firmware itself, which remains under [`LICENSE-FIRMWARE`](LICENSE-FIRMWARE).
