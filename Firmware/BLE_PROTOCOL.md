# ODKI VBT — BLE Protocol Reference

**License note:** unlike the other documents in this repository, *this document* is released under **CC0 1.0** (public domain dedication, see the [License](#license) section at the end) — deliberately, so any third-party app or integration can implement a client against it with zero legal uncertainty, commercial or not.

This is the byte-exact specification of the sensor's Bluetooth Low Energy interface: every characteristic UUID, every packet's layout down to the byte offset, and every scale factor needed to turn a raw value back into a physical one. It's written so a **third-party client** — a script, a different mobile app, a home-automation integration, anything — can talk to the device without needing to read the firmware source at all. It describes **protocol version 1.0** (see [Protocol version and compatibility](#protocol-version-and-compatibility)).

New to the protocol? [`DEVELOPERS.md`](../DEVELOPERS.md) is a short guide for app developers: quick start, a minimal example client ([`tools/vbt_ble_client.py`](tools/vbt_ble_client.py)) and licensing. Packets with their expected decoded values are in [Test vectors](#test-vectors) at the end of this document.

For *why* the protocol is shaped this way (why `Stream` is always raw, why `CorrectedCurve` exists and arrives late, why `RepSummary` can repeat), see the [BleServer chapter](DOCUMENTATION.md#bleserver) of the main firmware documentation — this document only covers *what's on the wire*, not the reasoning behind it. Reference implementation: `BleServer.h`/`.cpp` in this repository.

## Conventions

- **Byte order:** little-endian throughout — the nRF52840's native order, and standard for BLE.
- **No padding:** every packet struct in the firmware is declared `#pragma pack(push, 1)`, so byte offsets below are a plain cumulative sum of field sizes, with nothing inserted between fields.
- **Fixed-point physical values:** most quantities travel as a scaled integer, not a float: `physical_value = raw_value / scale`. This is called out per field below. A `scale` of `1` means the integer *is* the value directly (no conversion needed) — this is used for millisecond durations and millimetre-per-second velocities, so the wire value is already in a convenient integer unit. The firmware rounds to the nearest integer and saturates at the type's range (±32767 for int16, 0–65535 for uint16) instead of wrapping.
- **Writes are Write Requests** (with response). No characteristic of the custom service accepts Write Without Response.
- **Security:** every custom characteristic is open — `SECMODE_OPEN`, no pairing or bonding required for either read/notify or write access. Any BLE-capable device within range can connect, read live data, and issue `Command`/`Config` writes (including stopping tracking or changing algorithm parameters) without authentication. Keep this in mind if deploying in a setting where that matters.

## Protocol version and compatibility

### Reading the protocol version

The protocol has its own version, `MAJOR.MINOR`, independent of the firmware version: a firmware release that leaves the wire format alone does not change it. Read it from the **ProtocolVersion** characteristic (`6c7325bb-1784-4d77-8b99-89ce838ac2b3`, Read, 2 bytes: `[MAJOR, MINOR]`, see [ProtocolVersion](#protocolversion--c2b3-read-2-bytes)). This document describes **1.0**.

- **MAJOR equal to the one you implemented:** talk to the sensor. A higher MINOR only adds things you can ignore.
- **MAJOR higher than the one you implemented:** something your client relies on may have changed. Do not decode packets blindly; tell the user an app update is needed.
- **Characteristic absent:** firmware from before protocol versioning. Everything in this document applies except ProtocolVersion itself; where a field or behaviour changed between firmware releases, the per-characteristic notes below say so by Firmware Revision String.

The **Firmware Revision String** stays what it was — the firmware's own `MAJOR.MINOR.PATCH` — and is the right thing to read for firmware updates, but not for protocol compatibility.

### Versioning policy

ODKI keeps the protocol open in later versions, and versions it like this:

- **MINOR (additive, backwards compatible):** a new characteristic or service; a new `Command` value; bytes **appended** at the end of a Notify or Read value; a new value of an enumerated field (`phaseState`, `correctionStatus`); a clarification of behaviour already shipped. A client written for `1.0` keeps working against any `1.x`.
- **MAJOR (breaking):** removing a characteristic, field or command; moving, resizing, re-scaling or changing the sign or meaning of an existing field; giving an existing UUID a new meaning; rejecting a write that was accepted before; making `Config` longer or shorter in a way that a `1.0` client's write is no longer accepted. A major change is announced in advance in this document and in the firmware release notes, and the old behaviour is kept alongside the new one, marked deprecated, wherever that is technically feasible.
- **Never:** a UUID is never reused with a different meaning. Retired UUIDs stay retired (`…c2b1`, see [All UUIDs](#all-uuids-at-a-glance)), and a field that is no longer used keeps its place in the packet (as the firmware-only `Config` fields do).
- **Not part of the protocol:** the tracking algorithm and the numbers it produces (how reps are detected and scored, the default and accepted values of `Config` fields, which `Config` fields the firmware honours), timing (how often `Stream` is sent, how late `CorrectedCurve` and a `Corrected` `RepSummary` arrive), the Device Information strings other than the Firmware Revision String, and the second stage of a firmware update (Nordic's Legacy DFU protocol, see [Firmware update](#firmware-update-dfu)). These can change in any firmware release without a protocol version change.

### What a client must do to stay compatible

- **Accept longer values.** Parse the bytes you know and **ignore any trailing bytes** of a Notify or Read value. A shorter value than documented is an error: drop it.
- **Accept unknown enumerated values.** Treat an unknown `phaseState` as unknown and an unknown `correctionStatus` as not final (like `Provisional`), rather than rejecting the packet.
- **Ignore what you don't know.** Unknown services, characteristics and properties are not an error.
- **Find characteristics by UUID, never by handle,** and rediscover services after a firmware update: adding a characteristic (as ProtocolVersion did) moves the handles of the services declared after it.
- **Identify the sensor by its advertised service UUID, not by name** (see [Advertising & connecting](#advertising--connecting)).

### What is supported for third-party apps

| Use | Status |
|---|---|
| Scanning, connecting, reading ProtocolVersion, Device Information, Battery and `SystemStatus` | **Supported** |
| Subscribing to `Stream`, `RepSummary` and `CorrectedCurve` and decoding them | **Supported** |
| `Command`: `CALIBRATE`, `START`, `STOP` | **Supported** — a sensor boots uncalibrated, so a client that starts tracking needs `CALIBRATE` too |
| Firmware update over the air ([DFU](#firmware-update-dfu)) | Technically possible and documented; **recommended to leave to the ODKI app.** An interrupted or wrongly packaged transfer leaves the sensor in its bootloader until a transfer completes, and an update the user did not expect changes the firmware their data is measured with. |
| Writing [`Config`](#config--c2b2-readwrite-38-bytes) | Technically possible and documented; **recommended to leave at the defaults** (or to the ODKI app's settings). The parameters steer rep detection and drift correction: changing them changes the readings, and is not kept across a power cycle. If your app does write `Config`, tell your users their numbers are no longer comparable with default settings. |

These are recommendations, not restrictions: the protocol is open and every characteristic is documented below. ODKI's support covers the supported uses.

## Advertising & connecting

| | |
|---|---|
| GAP device name | `VBT-Sensor` (Device Name characteristic, readable after connecting) |
| Advertised name | `VBT-S` — the 128-bit service UUID leaves room only for a *shortened* local name in the 31-byte advertising packet, and no scan response is sent. Operating systems may show either name. |
| Advertised service | the custom service UUID below — **filter scans on this**, not on the name |
| Role | Peripheral only (never Central) — exactly one simultaneous connection |
| Pairing | None required |

**MTU is mandatory to negotiate.** The default ATT MTU (23 bytes, 20 usable) is smaller than every custom packet except `Command`, `SystemStatus` and `ProtocolVersion`. With a 23-byte MTU the firmware's BLE library sends a longer notification as several consecutive 20-byte pieces, with nothing marking where a packet starts, so a client cannot reliably reassemble them. Negotiate an ATT MTU of **at least 56 bytes** (the largest notification, `CorrectedCurve`, is 53 bytes, plus 3 bytes of ATT header) before subscribing; the firmware itself requests up to 247 bytes on connect. iOS and macOS negotiate it automatically; on Android call `requestMtu()`; on other platforms check what your BLE library does, and read back the MTU it obtained.

**Writes longer than 20 bytes may not get through.** Some Android BLE controllers (observed: Samsung Galaxy Tab A8, Unisoc T618) can receive long packets but drop the link on any client write longer than 20 bytes — the longest ATT write that fits a single default-size link-layer packet — whatever MTU was negotiated: the write never arrives and the connection dies to a supervision timeout ~5 s later. `Config` is the only characteristic a client writes with more than 20 bytes; from v3.11.31 it can be sent in chunks instead (see [Config](#config--c2b2-readwrite-38-bytes)), and clients should prefer that form.

## All UUIDs at a glance

| Name | UUID | Properties | Size |
|---|---|---|---|
| ODKI VBT Service | `6c7325bb-1784-4d77-8b99-89ce838ac2ab` | — | — |
| Stream | `6c7325bb-1784-4d77-8b99-89ce838ac2ac` | Notify | 37 B |
| RepSummary | `6c7325bb-1784-4d77-8b99-89ce838ac2ad` | Notify | 17 B |
| CorrectedCurve | `6c7325bb-1784-4d77-8b99-89ce838ac2ae` | Notify | 53 B |
| SystemStatus | `6c7325bb-1784-4d77-8b99-89ce838ac2af` | Read, Notify | 2 B |
| Command | `6c7325bb-1784-4d77-8b99-89ce838ac2b0` | Write | 1 B |
| *(retired)* | `6c7325bb-1784-4d77-8b99-89ce838ac2b1` | — | never reused |
| Config | `6c7325bb-1784-4d77-8b99-89ce838ac2b2` | Read, Write | 38 B (writable in 20 B chunks from v3.11.31) |
| ProtocolVersion | `6c7325bb-1784-4d77-8b99-89ce838ac2b3` | Read | 2 B (protocol 1.0+) |
| Battery Service | `0x180F` (standard) | — | — |
| Battery Level | `0x2A19` (standard) | Read, Notify | 1 B |
| Device Information Service | `0x180A` (standard) | — | — |
| Firmware Revision String | `0x2A26` (standard) | Read | variable (UTF-8) |
| Software Revision String | `0x2A28` (standard) | Read | variable (UTF-8) |
| Buttonless DFU Service | `00001530-1212-efde-1523-785feabcd123` | — | — |
| DFU Control | `00001531-1212-efde-1523-785feabcd123` | Write, Notify | up to 23 B (command `0x01`) |
| DFU Packet | `00001532-1212-efde-1523-785feabcd123` | Write Without Response | up to 20 B (not used in application mode) |
| DFU Revision | `00001534-1212-efde-1523-785feabcd123` | Read | 2 B (`0x0001`, application mode) |

(The custom UUIDs all share the base `6c7325bb-1784-4d77-8b99-89ce838ac2ab`, incrementing only the last byte. `0xb1` is retired: it carried the firmware version in early firmware, before the standard Device Information Service above replaced it, and is not reused so that no client can mistake one for the other. `0xae`, the early custom Battery characteristic, was reused for `CorrectedCurve` before this policy existed.)

---

## Stream — `…c2ac`, Notify, 37 bytes

Live telemetry, sent only while tracking is active, decimated: one packet every `Config::STREAM_DECIMATION_FACTOR` (5) engine samples. Not sent at all while a USB-serial connection is open on the device (the raw serial log takes over in that case, see [`Firmware/DOCUMENTATION.md`](DOCUMENTATION.md#lab-data-capture-serial-log)) — a client connected over BLE to a sensor that is also plugged into a computer with its serial port open will receive `RepSummary`/`CorrectedCurve` but no `Stream`. Packets are not queued: a notification that cannot be sent at that moment (link busy, client not subscribed) is skipped.

| Offset | Size | Field | Type | Scale | Unit |
|---|---|---|---|---|---|
| 0 | 4 | `timestampMs` | uint32 | 1 | ms — the device's `millis()` (time since boot) when the packet is sent |
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
| 26 | 2 | `worldPosZ` | int16 | ÷1000 | m — live vertical position, **signed**, relative to the start of the current rep cycle; see below |
| 28 | 2 | `quatW` | int16 | ÷10000 | dimensionless, [-1, 1] |
| 30 | 2 | `quatX` | int16 | ÷10000 | dimensionless, [-1, 1] |
| 32 | 2 | `quatY` | int16 | ÷10000 | dimensionless, [-1, 1] |
| 34 | 2 | `quatZ` | int16 | ÷10000 | dimensionless, [-1, 1] |
| 36 | 1 | `phaseState` | uint8 | direct | `0`=Idle, `1`=Eccentric, `2`=Concentric |

Python decode: `struct.unpack("<I16hB", data[:37])` — field order matches the table above.

`timestampMs` and `CorrectedCurve.startTimestampMs` run on the same device clock, so a corrected chunk can be matched to the `Stream` packets it replaces; `timestampMs` is taken when the packet is sent, which can be a few milliseconds after the sample was measured.

`worldPosZ` is integrated from the firmware's live-corrected vertical velocity (`velZLive`, the one the engine decides phases with — not `refVelZ` and not the retroactive `CorrectedCurve`). It is signed in the rep-direction frame and keeps its value through pauses; it is reset to 0 only when a rep cycle is complete (eccentric + concentric for a squat, back at the top; concentric + eccentric for a deadlift from the floor, back on the floor — see `Firmware/DOCUMENTATION.md`, Live position), so within a cycle it reads the bar's height relative to where the cycle started (a squat negative during the descent, a deadlift positive while up, including a pause at the top). The value it had just before a reset is the cycle's integration error. In firmware 3.12.16 the field was reset at every confirmed stillness (or landing). Being live, it is not corrected afterwards: its error is the live velocity's; the residual left at the end of each rep cycle is written to the USB serial log (`Y,` row, see DOCUMENTATION.md). Up to firmware 3.12.15 the same field was the integral of `refVelZ` while it was positive, reset to 0 at every reversal (never negative) — same layout, same scale; the range of int16 ÷1000 (±32.767 m) is far beyond a rep. For the displacement of a rep use `RepSummary.displacementM` (concentric only, ≥ 0, from the corrected curve), which does not change.

---

## RepSummary — `…c2ad`, Notify, 17 bytes

One packet per completed rep. **The same `repNumber` can arrive more than once** as `correctionStatus` improves — a client must overwrite its stored copy of that rep by `repNumber`, never append a duplicate. Updates of a rep not yet sent replace each other in the firmware's queue, so a client may receive fewer versions of a rep than were computed, but always the latest one.

| Offset | Size | Field | Type | Scale | Unit |
|---|---|---|---|---|---|
| 0 | 1 | `repNumber` | uint8 | direct | 1-based, restarts at 1 at every `START`; wraps to 0 after 255 |
| 1 | 2 | `peakVelocity` | int16 | ÷1000 | m/s, concentric |
| 3 | 2 | `meanVelocity` | int16 | ÷1000 | m/s, concentric |
| 5 | 2 | `peakAcceleration` | int16 | ÷100 | m/s², concentric |
| 7 | 2 | `meanAcceleration` | int16 | ÷100 | m/s², concentric |
| 9 | 2 | `displacementM` | uint16 | ÷1000 | m, concentric-only, always ≥0 |
| 11 | 2 | `eccPeakVelocity` | int16 | ÷1000 | m/s, `0` if this rep had no eccentric phase |
| 13 | 2 | `eccMeanVelocity` | int16 | ÷1000 | m/s, `0` if this rep had no eccentric phase |
| 15 | 1 | `quality1` | uint8 | direct | 0–100, kinematic confidence score |
| 16 | 1 | `correctionStatus` | uint8 | direct | `0`=Provisional, `1`=RepCalibrated, `2`=Corrected |

Python decode: `struct.unpack("<B4hH2hBB", data[:17])`.

Velocities and accelerations are sent as magnitudes (the firmware takes absolute values, so they are ≥ 0 even though the fields are signed); the direction is given by which field they are in. The eccentric fields belong to the eccentric phase *before* the concentric in the same cycle: on a lift that starts with the concentric (a deadlift from the floor), the lowering after rep *n* is reported with rep *n + 1*, and the first rep has none.

**`correctionStatus` in detail:**

| Value | Name | Meaning |
|---|---|---|
| `0` | Provisional | Scored against an extrapolated drift-rate estimate; the bracket containing this rep hasn't closed yet. Treat the numbers as a best-effort live estimate, not final. |
| `1` | RepCalibrated | Scored against a rep-cycle-based calibration (no bracket needed) — used when no bracket has closed recently, typically during a continuous, gapless set. |
| `2` | Corrected | Scored against a bracket's *measured* drift. Final — this `repNumber` will not be re-reported again. |

A rep is not guaranteed to reach `Corrected`: if tracking is stopped before the bracket containing it closes, it keeps the last status it was sent with (see [Sessions](#sessions-start-stop-and-disconnections)).

---

## CorrectedCurve — `…c2ae`, Notify, 53 bytes

Retroactively drift-corrected velocity for a time range that was already streamed as raw via `Stream`. Sent in chunks when the bracket covering that range closes; a bracket can span thousands of samples, far more than fit in one BLE packet. A client should replace the portion of its live graph already drawn from `Stream` in `[startTimestampMs, startTimestampMs + sampleCount × sampleIntervalMs)` with these values.

A bracket starts at the last still sample before a movement (its corrected value is 0) and ends at the sample where the next stillness is confirmed; the samples of a pause in between two brackets are not sent here, so a client keeps their live `Stream` values (0 while still). Since 3.12.22 a bracket with reps to rescore waits for the stillness to settle before it closes: its chunks (and the `Corrected` `RepSummary`) arrive up to 20 samples after the stillness is confirmed, and a stillness shorter than 5 samples does not end it (see [Bracket drift correction](DOCUMENTATION.md#bracket-drift-correction)); the chunks still end at the first still sample, and the packets are unchanged. Up to and including 3.12.0 the chunks also carried the pause before the movement, with a non-zero, wrongly extrapolated correction (fixed in 3.12.1, see [Bracket drift correction](DOCUMENTATION.md#bracket-drift-correction)); a client needs no change.

| Offset | Size | Field | Type | Scale | Unit |
|---|---|---|---|---|---|
| 0 | 2 | `bracketId` | uint16 | direct | 1 for the first bracket after `START`, then +1 on every bracket close |
| 2 | 2 | `chunkIndex` | uint16 | direct | 0-based, within this bracket |
| 4 | 2 | `totalChunks` | uint16 | direct | how many chunks this bracket was split into |
| 6 | 1 | `sampleCount` | uint8 | direct | how many of the 20 slots below are valid |
| 7 | 4 | `startTimestampMs` | uint32 | direct | ms — timestamp of `correctedVelZ[0]`, same clock/scale as `Stream::timestampMs` |
| 11 | 2 | `sampleIntervalMs` | uint16 | direct | ms between consecutive samples, nominal; the real spacing can be slightly longer, see the note below |
| 13 | 40 | `correctedVelZ[0..19]` | int16 × 20 | ÷1000 | m/s each |

Only the first `sampleCount` entries of `correctedVelZ` are meaningful — a bracket's final chunk is usually partial, and the remaining slots are zero-padding to be ignored.

**Note:** `sampleIntervalMs` is the nominal sample spacing. Real sample periods can be slightly longer, so a curve redrawn with the nominal spacing is a close approximation of the true timing, not an exact one. Use the value in the packet rather than a constant.

Python decode: `struct.unpack("<HHHBIH20h", data[:53])`.

To reconstruct which of the already-received `Stream` samples a chunk corrects, match on timestamp: sample `i` in the chunk corresponds to `t = startTimestampMs + i × sampleIntervalMs`. Because `bracketId` restarts at every `START`, key brackets by session as well as by `bracketId`.

---

## SystemStatus — `…c2af`, Read/Notify, 2 bytes

Notified on every state change; readable at any time via a standard GATT Read. **Read it once after subscribing**: the firmware also pushes it when a connection opens, but at that moment no client has enabled notifications yet, so in practice that first push is not received.

| Offset | Size | Field | Type | Meaning |
|---|---|---|---|---|
| 0 | 1 | `calibrated` | uint8 | `0`/`1` |
| 1 | 1 | `trackingActive` | uint8 | `0`/`1` |

A sensor boots with `calibrated = 0`, and calibration is not kept across a power cycle. `Start` (see `Command` below) is rejected by the firmware while `calibrated = 0`; `Calibrate` is rejected while `trackingActive = 1`. Tracking is not stopped by a disconnection.

---

## Command — `…c2b0`, Write, 1 byte

A single control byte, written with a Write Request:

| Value | Command | Effect |
|---|---|---|
| `0x00` | STOP | Stops tracking (if active). No effect otherwise. |
| `0x01` | START | Starts a new tracking session. Ignored if the device isn't calibrated yet. Sent while tracking is already active, it restarts the session (see [Sessions](#sessions-start-stop-and-disconnections)). |
| `0x02` | CALIBRATE | Blocking ~2s orientation calibration — the device must be held still. Ignored while tracking is active (stop first). |

Any other value, or an empty write, is silently ignored. The characteristic has a fixed length of 1 byte, so a longer write is refused by the BLE stack with an ATT error. The outcome of a command is visible in `SystemStatus`, which is notified when the state changes.

---

## Sessions: start, stop and disconnections

- Each `START` begins a clean session: `repNumber` and `bracketId` restart at 1 and the live velocity and position restart from 0. `STOP` ends it.
- `RepSummary` and `CorrectedCurve` packets are **only sent while tracking is active**. At `STOP` (and at a new `START`), packets not yet sent are discarded, and a rep whose bracket has not closed keeps the status it was last sent with. To make sure the last reps of a set are final, wait for their `Corrected` `RepSummary` before sending `STOP`.
- During a disconnection with tracking active, `RepSummary` and `CorrectedCurve` packets are queued and delivered after the client reconnects and re-subscribes; `Stream` packets are not. The queues are finite (the oldest corrected-curve chunks are dropped first if a very long disconnection fills them).
- `Config` and the calibration are kept across disconnections, until the device is switched off.

---

## Config — `…c2b2`, Read/Write, 38 bytes

Runtime-adjustable algorithm parameters, mirroring the firmware's `RuntimeConfig` struct field-for-field, except for a few firmware-only fields that are not carried at all (they always keep their compiled defaults; see [`DOCUMENTATION.md`](DOCUMENTATION.md#runtimeconfig-field-reference)). Third-party clients are recommended to leave it at the defaults (see [What is supported](#what-is-supported-for-third-party-apps)). **Not persisted to flash** — the firmware always boots from its compiled defaults and keeps a written configuration only until it is switched off, so a client that wants non-default settings must re-send this characteristic on every new connection. A write takes effect at once, also during a tracking session; write it while tracking is stopped. Reading it returns the defaults until a client writes otherwise, and from then on the last packet accepted. An accepted write is echoed back into the Read value immediately — except for the firmware-only fields (below), which the echo shows at their compiled defaults, and `repDirection`, where any value other than `1` is taken (and echoed) as `0` — before the firmware has necessarily applied it internally, and **without** the defensive clamps the firmware applies when it does (see below), so a Read can show a value the algorithm is not actually using.

**Firmware-only fields (v3.12.4+).** Seven fields — `accZBiasIdleStillTimeMs`, `gyroBiasIdleStillTimeMs`, `accZBiasGyroMaxDegSx10`, `accZBiasAccMagToleranceX1000`, `emaAlphaX1000`, `minCrossingExcursionMmps`, `minCrossingDurationMs`, marked in the table below — keep their place in the packet, so the layout and size are unchanged for every client, but the values a client writes there are **ignored**: the firmware always uses its compiled defaults for them, and a Read returns those defaults. A client should still send a full, well-formed 38-byte packet; the simplest correct behaviour is to send back, in these seven fields, what it last read. On firmware before v3.12.4 they are applied like every other field.

**Chunked writes (v3.11.31+).** Besides the single 38-byte write, the firmware accepts the same packet as consecutive writes of `[offset][data…]`, each at most 20 bytes long: byte 0 is the offset of `data` within the 38-byte packet, the rest is that slice of the packet. Chunks must arrive in order starting at offset 0, each continuing exactly where the previous one ended; the packet is applied only once all 38 bytes are in. An out-of-order or out-of-range chunk discards what was assembled so far; a chunk with offset 0 always starts over. The companion app sends two chunks, `[0x00][bytes 0–18]` and `[0x13][bytes 19–37]`. Between chunks a Read still returns the last complete packet. Firmware before v3.11.31 rejects any write that isn't exactly 38 bytes (the characteristic had a fixed length), so a client should fall back to the single write on older firmware: a sensor that has the ProtocolVersion characteristic always accepts chunks; otherwise check the `Firmware Revision`.

**Write lengths, in short:** exactly 38 bytes — a full packet; 2–20 bytes — a chunk; 1 or 21–37 bytes — ignored (the Read value keeps the last complete packet; up to the firmware that introduced ProtocolVersion, the ignored bytes themselves could show up in the Read value until the next accepted write); more than 38 bytes — refused by the BLE stack.

See [`Firmware/DOCUMENTATION.md`](DOCUMENTATION.md#runtimeconfig-field-reference) for what each field actually controls in the algorithm — this table is the wire layout only. The default packet, as a sensor returns it after boot, is in [Test vectors](#test-vectors).

| Offset | Size | Field | Type | Scale | Unit |
|---|---|---|---|---|---|
| 0 | 1 | `repDirection` | uint8 | direct | `0`=Up, `1`=Down |
| 1 | 2 | `maxPlausibleVelocityMmps` | uint16 | direct | mm/s |
| 3 | 2 | `accZBiasIdleStillTimeMs` | uint16 | direct | ms — **firmware-only** since v3.12.4, ignored on write |
| 5 | 2 | `gyroBiasIdleStillTimeMs` | uint16 | direct | ms — **firmware-only** since v3.12.4, ignored on write |
| 7 | 2 | `accZBiasGyroMaxDegSx10` | uint16 | ÷10 | degrees/s — **firmware-only** since v3.12.4, ignored on write |
| 9 | 2 | `accZBiasAccMagToleranceX1000` | uint16 | ÷1000 | m/s² — **firmware-only** since v3.12.4, ignored on write |
| 11 | 2 | `flatGuardMaxVelocityMmps` | uint16 | direct | mm/s |
| 13 | 1 | `velocityOverrideFlatWindowSamples` | uint8 | direct | samples |
| 14 | 2 | `velocityFlatBandMmps` | uint16 | direct | mm/s |
| 16 | 1 | `maxVelocityFlatWindowSamples` | uint8 | direct | samples |
| 17 | 1 | `minVelocityFlatWindowSamples` | uint8 | direct | samples |
| 18 | 2 | `accelerationFlatBandX1000Mps2` | uint16 | ÷1000 | m/s² |
| 20 | 1 | `maxAccelerationFlatWindowSamples` | uint8 | direct | samples |
| 21 | 1 | `minAccelerationFlatWindowSamples` | uint8 | direct | samples |
| 22 | 2 | `windowSaturationPeakVelocityMmps` | uint16 | direct | mm/s |
| 24 | 2 | `phaseStartVelocityMmps` | uint16 | direct | mm/s |
| 26 | 2 | `minPhaseDurationMs` | uint16 | direct | ms |
| 28 | 2 | `maxPhaseDurationMs` | uint16 | direct | ms |
| 30 | 1 | `phaseLookbackSamples` | uint8 | direct | samples |
| 31 | 1 | `reversalConfirmSamples` | uint8 | direct | samples |
| 32 | 2 | `emaAlphaX1000` | uint16 | ÷1000 | dimensionless, [0, 1] — **firmware-only** since v3.12.4, ignored on write |
| 34 | 2 | `minCrossingExcursionMmps` | **int16** | direct | mm/s, always ≤ 0 — **firmware-only** since v3.12.4, ignored on write |
| 36 | 2 | `minCrossingDurationMs` | uint16 | direct | ms — **firmware-only** since v3.12.4, ignored on write |

Every field is unsigned except `minCrossingExcursionMmps` at offset 34 — the one signed field in the packet.

Python decode: `struct.unpack("<BHHHHHHBHBBHBBHHHHBBHhH", data[:38])` — 23 fields, matching the table row order exactly (`B`, `H`×6, `B`, `H`, `BB`, `H`, `BB`, `H`×4, `BB`, `H`, `h`, `H`).

**Firmware-side clamps** (applied in `MotionTracker::setConfig()`, because these fields index fixed 64-sample ring buffers): `maxVelocityFlatWindowSamples` and `maxAccelerationFlatWindowSamples` are capped at 64; `velocityOverrideFlatWindowSamples` is clamped to 10–40 (since v3.12.8; 30–64 before, default 30); `minVelocityFlatWindowSamples`/`minAccelerationFlatWindowSamples` are raised to at least 1 and lowered to their matching `max…` value if larger. No other field is range-checked.

**v3.12.4:** the seven bias and drift-correction fields became firmware-only (see above). Layout and size unchanged. Reason: a client that stores and re-sends fields its user cannot see (the companion app up to 1.0.5 did) would otherwise override a default changed in a firmware update — v3.12.3 raised `accZBiasIdleStillTimeMs` from 50 to 300.

**v3.11.27:** `flatGuardOverrideStillTimeMs` (a uint16 duration, was offset 13) was removed and replaced by `velocityOverrideFlatWindowSamples` (a uint8 sample count) at the same offset — the packet shrank from 39 to 38 bytes. The old field gated a raw gyroscope/accelerometer quiet-time safety net that in practice almost never fired (it required a full, unbroken second with zero noise anywhere in it); the new field instead widens the SAME velocity/acceleration flatness tests the sensor already runs (to a fixed window of 30–64 samples at the time, 10–40 since v3.12.8; the companion app offered 30–60) once the corrected velocity estimate is outside `flatGuardMaxVelocityMps` — see [`Firmware/DOCUMENTATION.md`](DOCUMENTATION.md#the-flat-guards-two-tier-design) for the full reasoning. A client built against the pre-v3.11.27 39-byte layout will have its Config writes silently ignored by v3.11.27+ firmware — update the packet size and field before talking to updated firmware.

**v3.11.24:** `debugLogEnabled` (offset 39) was removed — the packet shrank from 40 to 39 bytes. Whether the sensor prints its raw serial log or streams over BLE is now decided automatically from whether a USB-serial connection is open (see [`Firmware/DOCUMENTATION.md`](DOCUMENTATION.md#lab-data-capture-serial-log)), not a field a client sends.

---

## ProtocolVersion — `…c2b3`, Read, 2 bytes

The version of this protocol, independent of the firmware version (see [Protocol version and compatibility](#protocol-version-and-compatibility)). Static: set once at boot.

| Offset | Size | Field | Type | Meaning |
|---|---|---|---|---|
| 0 | 1 | `major` | uint8 | protocol MAJOR — `1` |
| 1 | 1 | `minor` | uint8 | protocol MINOR — `0` |

A later version may append bytes: read the first two and ignore the rest. Firmware from before protocol versioning does not have this characteristic.

---

## Battery Service (standard, `0x180F`)

**Battery Level** (`0x2A19`, Read/Notify, 1 byte): `percent`, uint8, direct, 0–100. Updated by the firmware every `Config::BATTERY_CHECK_INTERVAL_MS` (30s) and once immediately on boot.

## Device Information Service (standard, `0x180A`)

**Firmware Revision String** (`0x2A26`, Read, variable-length UTF-8): the firmware version as `"MAJOR.MINOR.PATCH"` (e.g. `"3.12.0"`). Set once at boot, static for the whole connection — read it to check whether an OTA update is available before offering one to the user.

**Software Revision String** (`0x2A28`, Read, variable-length UTF-8): informative only — a client must use the Firmware Revision String above for version comparisons, and may ignore this one. Since 3.12.0 it carries the same `"MAJOR.MINOR.PATCH"`. Earlier firmware did not set it, so it holds the Arduino board-package version there (e.g. `"1.1.13"`). It is not the protocol version: read ProtocolVersion for that.

The service also carries Manufacturer Name, Model Number and Serial Number strings filled in by the BLE library (the Serial Number is the microcontroller's unique ID). They are not part of the protocol and may change.

## Firmware update (DFU)

Third-party apps are recommended to leave firmware updates to the ODKI app (see [What is supported](#what-is-supported-for-third-party-apps)). For completeness, updating the firmware is a two-stage handoff, and only the first stage is custom to this project:

1. **Enter bootloader mode.** Write `0x01` to the DFU Control characteristic (`00001531-1212-efde-1523-785feabcd123`, under the Buttonless DFU service `00001530-1212-efde-1523-785feabcd123`). The device disconnects and reboots into its bootloader (the Adafruit nRF52 bootloader; ODKI sensors use its community OTAFIX variant, which advertises as `XIAO_DFU` — the stock one advertises as `AdaDFU` — see [`Hardware/Instructions/BOOTLOADER.md`](../Hardware/Instructions/BOOTLOADER.md)). The name and, usually, the BLE address change: a client has to scan for the bootloader again.
2. **Transfer the firmware.** The bootloader now exposes Nordic's **Legacy DFU** service (nRF5 SDK 11 protocol, service `00001530-1212-efde-1523-785feabcd123`) — a public, well-documented Nordic protocol, unrelated to anything custom in this repository. Use an existing DFU-capable client library rather than implementing this stage from scratch (the companion app uses `nordic_dfu` on Flutter; equivalents exist for most platforms — e.g. Nordic's own `nRF Connect Device Manager` / `iOS-DFU-Library` / `Android-DFU-Library`). The packages ODKI publishes are listed in [`firmware-release/manifest.json`](firmware-release/manifest.json).

There is no `.h`/`.cpp` in this firmware implementing the second stage — it's entirely the bootloader's responsibility, which is also why a device stuck mid-update stays in its bootloader, still advertising for DFU, and is recovered by running the transfer again (the companion app's **FINISH UPDATE**, see [`Hardware/Instructions/BOOTLOADER.md`](../Hardware/Instructions/BOOTLOADER.md#if-something-goes-wrong)).

## Test vectors

Packets as the firmware encodes them, with the values a correct decoder must produce. They are produced by the encoding code of `BleServer.cpp` itself (`configToPacket()`, `sendStream()`, `sendRepSummary()`, with `toFixed16()`/`toFixedU16()`), run on a computer; the `Stream` and `RepSummary` inputs are synthetic values chosen to exercise every field, including negative ones (not a recording), and the `Config` input is the compiled defaults, so the `Config` vector is exactly what a sensor returns on Read after boot. Decoded values are rounded to the field's resolution.

**Config, defaults** (38 bytes, Read after boot):

```
00 a0 0f 2c 01 2c 01 78 00 78 00 c8 00 1e 28 00 14 02 2c 01
14 02 20 03 c8 00 5e 01 88 13 0c 03 2c 01 e2 ff 2c 01
```

| Field | Raw | Value | Field | Raw | Value |
|---|---|---|---|---|---|
| `repDirection` | 0 | Up | `accelerationFlatBandX1000Mps2` | 300 | 0.300 m/s² |
| `maxPlausibleVelocityMmps` | 4000 | 4.000 m/s | `maxAccelerationFlatWindowSamples` | 20 | 20 |
| `accZBiasIdleStillTimeMs` | 300 | 0.300 s | `minAccelerationFlatWindowSamples` | 2 | 2 |
| `gyroBiasIdleStillTimeMs` | 300 | 0.300 s | `windowSaturationPeakVelocityMmps` | 800 | 0.800 m/s |
| `accZBiasGyroMaxDegSx10` | 120 | 12.0 °/s | `phaseStartVelocityMmps` | 200 | 0.200 m/s |
| `accZBiasAccMagToleranceX1000` | 120 | 0.120 m/s² | `minPhaseDurationMs` | 350 | 0.350 s |
| `flatGuardMaxVelocityMmps` | 200 | 0.200 m/s | `maxPhaseDurationMs` | 5000 | 5.000 s |
| `velocityOverrideFlatWindowSamples` | 30 | 30 | `phaseLookbackSamples` | 12 | 12 |
| `velocityFlatBandMmps` | 40 | 0.040 m/s | `reversalConfirmSamples` | 3 | 3 |
| `maxVelocityFlatWindowSamples` | 20 | 20 | `emaAlphaX1000` | 300 | 0.300 |
| `minVelocityFlatWindowSamples` | 2 | 2 | `minCrossingExcursionMmps` | −30 | −0.030 m/s |
| | | | `minCrossingDurationMs` | 300 | 0.300 s |

The defaults are algorithm tuning, not protocol: they can change in a firmware release (see the [versioning policy](#versioning-policy)). Use this vector to test a decoder, not as a constant.

**Stream** (37 bytes, one sample during a concentric phase):

```
c0 d4 01 00 0a 00 e7 ff 96 00 05 00 f6 ff a0 00 ec ff 0f 00
c2 01 f6 ff 05 00 06 ff de 26 f4 01 d4 fe 20 03 02
```

| Field | Value | Field | Value |
|---|---|---|---|
| `timestampMs` | 120000 | `refVelZ` | 0.450 m/s |
| `linAccX`, `linAccY`, `linAccZ` | 0.10, −0.25, 1.50 m/s² | `worldPosX`, `worldPosY`, `worldPosZ` | −0.010, 0.005, −0.250 m |
| `worldAccX`, `worldAccY`, `worldAccZ` | 0.05, −0.10, 1.60 m/s² | `quatW`, `quatX`, `quatY`, `quatZ` | 0.9950, 0.0500, −0.0300, 0.0800 |
| `worldVelX`, `worldVelY` | −0.020, 0.015 m/s | `phaseState` | 2 (Concentric) |

**RepSummary**, the same rep sent twice (17 bytes each) — first `Provisional`, then `Corrected`; a client keeps only the second:

```
03 b6 03 08 02 a4 01 d2 00 26 02 20 03 90 01 5f 00
03 ac 03 12 02 a4 01 d7 00 1c 02 2a 03 9a 01 64 02
```

| Field | First packet | Second packet |
|---|---|---|
| `repNumber` | 3 | 3 |
| `peakVelocity` | 0.950 m/s | 0.940 m/s |
| `meanVelocity` | 0.520 m/s | 0.530 m/s |
| `peakAcceleration` | 4.20 m/s² | 4.20 m/s² |
| `meanAcceleration` | 2.10 m/s² | 2.15 m/s² |
| `displacementM` | 0.550 m | 0.540 m |
| `eccPeakVelocity` | 0.800 m/s | 0.810 m/s |
| `eccMeanVelocity` | 0.400 m/s | 0.410 m/s |
| `quality1` | 95 | 100 |
| `correctionStatus` | 0 (Provisional) | 2 (Corrected) |

These numbers are made up for the test and say nothing about the sensor or its accuracy.

**Fixed values:**

| Packet | Bytes | Meaning |
|---|---|---|
| ProtocolVersion | `01 00` | protocol 1.0 |
| SystemStatus | `00 00` | not calibrated, not tracking (after boot) |
| SystemStatus | `01 00` | calibrated, not tracking |
| SystemStatus | `01 01` | calibrated, tracking |
| Command | `02` / `01` / `00` | CALIBRATE / START / STOP |

There is no `CorrectedCurve` vector yet. Its layout is fully specified above.

## License

This document (`BLE_PROTOCOL.md` only — not the firmware source it describes, see [`LICENSE`](../LICENSE) for that) is released under **CC0 1.0 Universal**: to the extent possible under law, the author(s) have waived all copyright and related or neighboring rights to this document. You may copy, modify, distribute, and use it — including commercially, including without attribution — with no conditions at all.

Full legal text: <https://creativecommons.org/publicdomain/zero/1.0/legalcode>

This applies only to the protocol *specification* in this file — implementing a compatible client (an app, a script, an integration) based on it is unrestricted. It does not relicense the firmware itself (Apache-2.0, with the tracking engine under PolyForm Noncommercial — see [`LICENSE`](../LICENSE)).
