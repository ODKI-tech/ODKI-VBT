# Apps

The ODKI VBT companion app connects to the sensor over Bluetooth Low Energy to calibrate it, start/stop tracking, view live velocity/rep data, and install firmware updates over the air.

**License note:** unlike the firmware and hardware design files in this repository, the app itself is closed-source and is not covered by [`LICENSE-FIRMWARE`](../LICENSE-FIRMWARE) or [`LICENSE-HARDWARE`](../LICENSE-HARDWARE) — only builds/installers are distributed here, no app source. Some of its features (training-data management, AI-based analysis) are paid; this has no effect on using the sensor itself, which works over the fully open [BLE protocol](../Firmware/BLE_PROTOCOL.md) regardless of which app — this one or a third-party alternative — you use with it.

Download and setup instructions for each platform:

- **[Android/](Android/)**
- **[iOS/](iOS/)**
- **[Windows/](Windows/)**

None of these are published yet — see each platform's README for current status.
