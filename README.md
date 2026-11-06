# ODKI VBT

**[odki.tech](https://odki.tech)** — project site, pre-orders, and the companion app.

ODKI VBT is an open-core **Velocity-Based Training (VBT)** sensor — a small wireless device that clips onto a barbell (or is otherwise attached to the load being lifted) and measures bar velocity, displacement, and rep count in real time, streaming the results to a companion app over Bluetooth Low Energy. Build your own from the files here for personal or research use, or buy a ready-made, ODKI-branded unit at [odki.tech](https://odki.tech) — see the license note below for exactly what "open core" means here.

Velocity-based training uses how *fast* a lift moves — not just the weight on the bar — to gauge effort, fatigue, and training intensity in real time, which is the whole reason this project exists: an accessible, understandable, hackable device to do that.

**Licenses, in short** (the map of which file is under which license is in [LICENSE](LICENSE)):

- **Tracking engine** (`Firmware/VBT_Quaternions/MotionTracker.cpp`/`.h`, the two offline tools that compile it in, and `Firmware/DOCUMENTATION.md`) — source-available, not open source: [PolyForm Noncommercial 1.0.0](LICENSE-ENGINE) with additional permissions. Free for personal, hobby and study use, for research for the benefit of public knowledge, and for universities, public research organisations, charities and the other organisations the license lists. Research by or for a company is commercial use. Commercial use — including selling devices or kits that run the engine — needs a commercial licence from ODKI (contact@odki.tech). Units from ODKI may be used for any purpose, including in a gym or for professional coaching.
- **Rest of the firmware and all the tools** (`TrackingTypes.h`, `FusionOrientation`, BLE, power, battery, LED, the `.ino`, the Python and C++ tools) — open source under the [Apache License 2.0](LICENSE-APACHE) (see also [NOTICE](NOTICE)).
- **Hardware design** (`Hardware/`) — [CC BY-NC-SA 4.0](LICENSE-HARDWARE): build, modify and share for noncommercial purposes; selling the files, prints made from them, or kits or devices that include such prints needs ODKI's permission.
- **BLE protocol specification** ([`Firmware/BLE_PROTOCOL.md`](Firmware/BLE_PROTOCOL.md)) — CC0 1.0, so anyone can build a client or a compatible device.
- **Third-party code** keeps its own license: the vendored x-io Fusion library is MIT; the libraries linked into the firmware binaries are listed in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
- **Companion app** — proprietary, see [Apps/README.md](Apps/README.md). The ODKI name is not licensed by any of the above ([TRADEMARKS.md](TRADEMARKS.md)).

Firmware and tools up to and including v3.11.31 were released under the GNU GPL v3.0 with the Commons Clause; copies received under those terms stay under them. See [CONTRIBUTING.md](CONTRIBUTING.md) for how contributing to the official repository works.

## Status

| | |
|---|---|
| Firmware | v3.12.4; updates over the air from the app (the phone must be online; needs the OTAFIX bootloader, see [`Hardware/Instructions/BOOTLOADER.md`](Hardware/Instructions/BOOTLOADER.md)) |
| Android app | 1.0.6, APK in [Apps/Android/](Apps/Android/); Google Play test coming |
| iOS app | not published yet |
| Accuracy | **not yet validated** (see [Validation](#validation)) |

## How it works, briefly

The sensor fuses its onboard accelerometer and gyroscope into a stable orientation estimate, uses that to isolate real acceleration from gravity, integrates it into velocity along the vertical axis, and detects individual reps from that velocity signal — continuously correcting for drift using brief natural pauses in motion, without requiring the athlete to explicitly mark the start/end of each rep. Every rep's peak/mean velocity, acceleration, displacement, and a data-quality score are reported live. The full algorithm is documented in detail in [`Firmware/DOCUMENTATION.md`](Firmware/DOCUMENTATION.md).

## Repository layout

- **[Hardware/](Hardware/)** — bill of materials, CAD/mechanical design files, and assembly instructions for building a unit.
- **[Firmware/](Firmware/)** — the Arduino C++ firmware source (`VBT_Quaternions/`), [`DOCUMENTATION.md`](Firmware/DOCUMENTATION.md) (a chapter-by-chapter reference covering what every source file does and the rep-detection algorithm in detail), [`BLE_PROTOCOL.md`](Firmware/BLE_PROTOCOL.md) (the byte-exact Bluetooth protocol spec — everything a third-party app needs to talk to the sensor), and the tools in `tools/`: `vbt_log_viewer.py`, for replaying and visualizing a raw debug-log capture from the device (with an interactive GUI to pick which columns to plot); `vbt_live_monitor.py`, a desktop GUI that shows that same data live over a USB-serial connection and can drive calibration/start/stop directly, no phone app needed; `host_replay/` and `full_replay/`, which run a captured log through the real tracking engine on a PC to evaluate algorithm changes offline (their C++ programs, `replay.cpp` and `harness.cpp`, compile the engine in and are under its license; the Python scripts are Apache-2.0); plus the regression, calibration and validation tools.
- **[Apps/](Apps/)** — download links and usage instructions for the companion app (Android, iOS, Windows).
- **[LICENSE](LICENSE)** — which file is under which license: **[LICENSE-ENGINE](LICENSE-ENGINE)** (tracking engine, PolyForm Noncommercial 1.0.0 with additional permissions), **[LICENSE-APACHE](LICENSE-APACHE)** + **[NOTICE](NOTICE)** (rest of the firmware and the tools, Apache-2.0), **[LICENSE-HARDWARE](LICENSE-HARDWARE)** (CC BY-NC-SA 4.0), **[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)** (libraries in the firmware binaries), **[TRADEMARKS.md](TRADEMARKS.md)** (the ODKI name). The BLE protocol specification ([`Firmware/BLE_PROTOCOL.md`](Firmware/BLE_PROTOCOL.md)) is released under CC0 (public domain), so third-party apps can be built against it without restriction.

## Getting started

1. **Build the hardware.** The [bill of materials](Hardware/BOM/), [CAD/mechanical design](Hardware/CAD/), and [step-by-step assembly instructions](Hardware/Instructions/) are all published — start there.
2. **Flash the firmware.** Open `Firmware/VBT_Quaternions/VBT_Quaternions.ino` in the Arduino IDE with the `Seeeduino:nrf52` board package installed (board: `xiaonRF52840Sense`) and the `Seeed Arduino LSM6DS3` library, then upload. See [`Firmware/DOCUMENTATION.md`](Firmware/DOCUMENTATION.md) for what each file does and how the algorithm works.
3. **Install the OTAFIX bootloader, before the first OTA update.** Over-the-air updates from the app need the community OTAFIX variant of the XIAO's bootloader — see [`Hardware/Instructions/BOOTLOADER.md`](Hardware/Instructions/BOOTLOADER.md) for why, how to check which one a board has, and how to install it, also on a closed unit without a reachable reset button.
4. **Install the app.** See [Apps/](Apps/) for your platform — currently an Android APK is available to download; iOS and Windows are not yet published.
5. **Calibrate and lift.** Power on, connect over Bluetooth, calibrate the sensor while it's still, then start tracking a set.

## Known limitations

- **Sample rate about 95 Hz instead of 100 Hz.** The sampler restarts its 10 ms interval from the current time at each sample, so real periods are 10–12 ms. Each sample integrates its true `dt`, so velocities are not biased by it, but windows counted in samples last about 5 % longer than nominal. A fix is planned for a later release — details in [`Firmware/DOCUMENTATION.md`](Firmware/DOCUMENTATION.md#known-limitations).

## Validation

**Not yet validated: the protocol is published before the data.** The study compares the sensor with a verified linear encoder. Its protocol, acceptance thresholds and analysis code are fixed in advance in [`Firmware/tools/validation/PROTOCOL.md`](Firmware/tools/validation/PROTOCOL.md), and the raw paired data will be published whatever it shows. Until then, no accuracy figures are claimed.

## Community

Questions, builds and ideas live in [GitHub Discussions](https://github.com/ODKI-tech/ODKI-VBT/discussions):

- **Show your build**: post a photo of your ODKI-VBT. The first 20 builders are credited as *Founding builders*.
- **Share a log**: recorded a set over USB? Shared logs help test and improve the tracking engine and, with your consent, join the open community dataset (CC BY 4.0).
- **Ideas** and **Q&A**: features you'd like in the free app, or anything you're stuck on.

Bugs go to [Issues](https://github.com/ODKI-tech/ODKI-VBT/issues). Everyone taking part follows the [Code of Conduct](CODE_OF_CONDUCT.md).

## Contributing

Both the hardware and firmware are meant to be read and modified. `Firmware/DOCUMENTATION.md` is written specifically to get a new contributor from "never seen this codebase" to "can confidently modify the rep-detection algorithm" — start there. See [`CONTRIBUTING.md`](CONTRIBUTING.md) for how to report a bug, propose a change, and the code style expected in a pull request.
