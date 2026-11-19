# Building apps for ODKI VBT

**License note:** this guide is under the Apache License 2.0, like the other repository documents ([`LICENSE`](LICENSE)). The protocol specification it points to, [`Firmware/BLE_PROTOCOL.md`](Firmware/BLE_PROTOCOL.md), is CC0 1.0.

This guide is for developers who want their own app, script or integration to work with an ODKI VBT sensor over Bluetooth Low Energy — free or paid, open or closed source. You don't need to know the firmware or the hardware: the sensor does the tracking, and your app reads the results.

## What you can rely on

- **An open, documented protocol.** [`BLE_PROTOCOL.md`](Firmware/BLE_PROTOCOL.md) specifies every characteristic byte by byte. It describes protocol version **1.0**, and ODKI keeps the protocol open in later versions.
- **A protocol version the sensor reports.** Read it from the ProtocolVersion characteristic; it changes only when the wire format changes, not with every firmware release.
- **A compatibility policy.** Additions are minor versions and don't break a client that follows the [client rules](Firmware/BLE_PROTOCOL.md#what-a-client-must-do-to-stay-compatible) (ignore trailing bytes, unknown values and unknown characteristics). Breaking changes are major versions, announced in advance. Details: [Versioning policy](Firmware/BLE_PROTOCOL.md#versioning-policy).
- **A clear list of supported uses.** Reading data, `CALIBRATE`, `START` and `STOP` are supported. Firmware updates over the air and writing the algorithm `Config` are documented and possible, but recommended to leave to the ODKI app: an interrupted update leaves the sensor in its bootloader, and changed parameters change the readings. See [What is supported](Firmware/BLE_PROTOCOL.md#what-is-supported-for-third-party-apps).

## Quick start

All UUIDs share the base `6c7325bb-1784-4d77-8b99-89ce838ac2xx`; the full table is in [All UUIDs](Firmware/BLE_PROTOCOL.md#all-uuids-at-a-glance).

1. **Scan** for devices advertising the service `6c7325bb-1784-4d77-8b99-89ce838ac2ab`. Filter on the service UUID, not on the name: the advertised name is shortened to `VBT-S`, while the device name read after connecting is `VBT-Sensor`.
2. **Connect** and make sure the ATT MTU is at least 56 bytes (iOS and macOS negotiate it on their own; on Android call `requestMtu()`). With a smaller MTU, packets arrive split.
3. **Read the protocol version** from `…c2b3`: two bytes, `01 00` for 1.0. If the characteristic is missing, the firmware is older than protocol versioning; the spec notes what differs. If the major version is higher than the one you implemented, ask the user to update your app instead of decoding.
4. **Subscribe** to `RepSummary` (`…c2ad`) and `SystemStatus` (`…c2af`), and to `CorrectedCurve` (`…c2ae`) and `Stream` (`…c2ac`) if you draw a live graph. Then **read** `SystemStatus` once: the push the sensor sends at connect time arrives before you have subscribed.
5. **Calibrate and start.** A sensor boots uncalibrated. Ask the user to hold the bar still, write `02` to `Command` (`…c2b0`), and wait for `SystemStatus` to report `calibrated = 1`. Then write `01` to start a session and `00` to stop it. Use Write Requests (with response).
6. **Decode the reps.** Each `RepSummary` is 17 bytes. The same `repNumber` can arrive several times as the drift correction improves (`correctionStatus` 0 → 2): overwrite by `repNumber`, never append. Before sending `STOP`, wait until the last reps are `Corrected` (2), or they stay provisional.

Decoding one rep in Python, with a test vector from the spec:

```python
import struct

packet = bytes.fromhex("03ac031202a401d7001c022a039a016402")  # a RepSummary notification
(rep, peak, mean, peak_acc, mean_acc, disp,
 ecc_peak, ecc_mean, quality, status) = struct.unpack("<B4hH2hBB", packet[:17])

print(rep, peak / 1000, mean / 1000, disp / 1000, status)
# 3 0.94 0.53 0.54 2   -> rep 3, m/s, m/s, m, Corrected
```

`packet[:17]` reads the documented prefix and ignores any bytes a later minor version may append.

## Example client and test vectors

- **[`Firmware/tools/vbt_ble_client.py`](Firmware/tools/vbt_ble_client.py)** is a minimal complete client in Python ([`bleak`](https://github.com/hbldh/bleak)): it scans, connects, reads the versions, subscribes, can send `CALIBRATE`/`START`/`STOP` and prints each rep. It never writes `Config` and never starts an update. Its decoders are plain functions you can port to any language. `python vbt_ble_client.py --selftest` checks them without hardware.
- **[Test vectors](Firmware/BLE_PROTOCOL.md#test-vectors)** in the spec give packets in hex with their decoded values: the default `Config` exactly as a sensor returns it, a `Stream` sample and two versions of one rep. Make your decoder pass them before testing with a sensor.

The other tools in `Firmware/tools/` read the sensor's USB serial log, not Bluetooth.

## Getting a sensor

Assembled units are available to order at [odki.tech](https://odki.tech). You can also build one from this repository — read the licensing section below first, because what a self-built unit may be used for is narrower.

## Licensing

This is a summary. The license files govern: [`LICENSE`](LICENSE) (the map), [`LICENSE-ENGINE`](LICENSE-ENGINE), [`LICENSE-HARDWARE`](LICENSE-HARDWARE), [`TRADEMARKS.md`](TRADEMARKS.md). If you are unsure, ask at contact@odki.tech.

- **The protocol is free to implement.** `BLE_PROTOCOL.md` is CC0 1.0: you may build any client from it, including a paid or closed-source app, with no royalty, no approval and no attribution required.
- **Code from this repository** keeps its license. The example client and the other tools are Apache-2.0: you may use them in a commercial app under its terms (keep the license and notice). The ODKI companion app is proprietary and not covered by any license here.
- **The firmware on the sensor is a separate matter.** Its tracking engine is under the PolyForm Noncommercial License 1.0.0 with ODKI's additional permissions ([`LICENSE-ENGINE`](LICENSE-ENGINE)), and what a sensor may be used for depends on where it came from:
  - **Units from ODKI.** Anyone who lawfully owns or uses a unit sold or otherwise supplied by ODKI — including second-hand, loaned, replacement units and units sold by a reseller or licensee authorised by ODKI in writing — may use its firmware and ODKI's updates "for any purpose, including commercial purposes such as professional coaching or use in a gym". The permission follows the unit.
  - **Self-built units running the ODKI firmware** may be used for the noncommercial purposes the license allows: personal, hobby and study use, research for the benefit of public knowledge, and use by universities, public research organisations, charities and the other organisations the license lists. "Any commercial use" needs a commercial licence from ODKI (contact@odki.tech) — for example a gym, a coach or a commercial service running self-built sensors. Research carried out by a company is commercial use too.
  - **Selling or distributing devices with the firmware.** Making, selling or otherwise distributing copies of the firmware, "on its own or installed on a board, kit or device", for a fee or as part of a commercial product or service, is not covered by the noncommercial license and needs a commercial licence from ODKI.
- **Your own hardware and firmware.** A compatible sensor that runs entirely your own firmware is not restricted by the protocol specification (CC0). If the hardware is built from ODKI's design files in [`Hardware/`](Hardware/), those files are under CC BY-NC-SA 4.0: selling them, prints made from them, or kits or devices that include such prints needs ODKI's permission.
- **The ODKI name.** No license here grants the name or logo. Saying truthfully that your app is "compatible with ODKI VBT sensors" is fine, in plain text and without suggesting it comes from or is endorsed by ODKI; using "ODKI" as the name or branding of your app is not ([`TRADEMARKS.md`](TRADEMARKS.md)).

## Questions and feedback

Protocol questions and suggestions are welcome in [GitHub Discussions](https://github.com/ODKI-tech/ODKI-VBT/discussions); bugs in the spec or the firmware's protocol behaviour go to [Issues](https://github.com/ODKI-tech/ODKI-VBT/issues). Commercial licences: contact@odki.tech.
