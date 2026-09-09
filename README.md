# ODKI VBT

**[odki.tech](https://odki.tech)** — project site, pre-orders, and the companion app.

ODKI VBT is a source-available **Velocity-Based Training (VBT)** sensor — a small wireless device that clips onto a barbell (or is otherwise attached to the load being lifted) and measures bar velocity, displacement, and rep count in real time, streaming the results to a companion app over Bluetooth Low Energy. Build your own from the files here for personal or research use, or buy a ready-made, ODKI-branded unit at [odki.tech](https://odki.tech) — see the license note below for exactly what "source-available" means here.

Velocity-based training uses how *fast* a lift moves — not just the weight on the bar — to gauge effort, fatigue, and training intensity in real time, which is the whole reason this project exists: an accessible, understandable, hackable device to do that.

Both the hardware and the firmware are source-available: anyone can read exactly how it works, build one for themselves, modify it, and share their own non-commercial improvements. Selling an assembled unit — under the ODKI name or otherwise, built from these files or a modified version of them — is reserved to ODKI; that's the one thing this isn't open to. See [LICENSE-FIRMWARE](LICENSE-FIRMWARE) and [LICENSE-HARDWARE](LICENSE-HARDWARE) for the exact terms of each, and [CONTRIBUTING.md](CONTRIBUTING.md) for how contributing to the official repository works.

## How it works, briefly

The sensor fuses its onboard accelerometer and gyroscope into a stable orientation estimate, uses that to isolate real acceleration from gravity, integrates it into velocity along the vertical axis, and detects individual reps from that velocity signal — continuously correcting for drift using brief natural pauses in motion, without requiring the athlete to explicitly mark the start/end of each rep. Every rep's peak/mean velocity, acceleration, displacement, and a data-quality score are reported live. The full algorithm is documented in detail in [`Firmware/DOCUMENTATION.md`](Firmware/DOCUMENTATION.md).

## Repository layout

- **[Hardware/](Hardware/)** — bill of materials, CAD/mechanical design files, and assembly instructions for building a unit.
- **[Firmware/](Firmware/)** — the Arduino C++ firmware source (`VBT_Quaternions/`), [`DOCUMENTATION.md`](Firmware/DOCUMENTATION.md) (a chapter-by-chapter reference covering what every source file does and the rep-detection algorithm in detail), [`BLE_PROTOCOL.md`](Firmware/BLE_PROTOCOL.md) (the byte-exact Bluetooth protocol spec — everything a third-party app needs to talk to the sensor), and `tools/vbt_log_viewer.py`, a Python script for replaying and visualizing a raw debug-log capture from the device.
- **[Apps/](Apps/)** — download links and usage instructions for the companion app (Android, iOS, Windows).
- **[LICENSE-FIRMWARE](LICENSE-FIRMWARE)** (GPL-3.0 + Commons Clause) / **[LICENSE-HARDWARE](LICENSE-HARDWARE)** (CC BY-NC-SA 4.0) — non-commercial use, modification, and redistribution allowed; selling a unit built from these files is reserved to ODKI. The BLE protocol specification itself ([`Firmware/BLE_PROTOCOL.md`](Firmware/BLE_PROTOCOL.md)) is separately released under CC0 (public domain), so third-party apps can be built against it without restriction.

## Getting started

1. **Build the hardware.** The [bill of materials](Hardware/BOM/), [CAD/mechanical design](Hardware/CAD/), and [step-by-step assembly instructions](Hardware/Instructions/) are all published — start there.
2. **Flash the firmware.** Open `Firmware/VBT_Quaternions/VBT_Quaternions.ino` in the Arduino IDE with the `Seeeduino:nrf52` board package installed (board: `xiaonRF52840Sense`) and the `Seeed Arduino LSM6DS3` library, then upload. See [`Firmware/DOCUMENTATION.md`](Firmware/DOCUMENTATION.md) for what each file does and how the algorithm works.
3. **Install the app.** See [Apps/](Apps/) for your platform — currently pending first public release.
4. **Calibrate and lift.** Power on, connect over Bluetooth, calibrate the sensor while it's still, then start tracking a set.

## Contributing

Both the hardware and firmware are meant to be read and modified. `Firmware/DOCUMENTATION.md` is written specifically to get a new contributor from "never seen this codebase" to "can confidently modify the rep-detection algorithm" — start there. See [`CONTRIBUTING.md`](CONTRIBUTING.md) for how to report a bug, propose a change, and the code style expected in a pull request.
