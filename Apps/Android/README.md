# Android

**Status: not yet published.** Link coming soon.

## Download & install

Distributed as a downloadable APK directly from [`app-release/`](app-release/) in this repository (sideload — no Google Play listing planned initially). Unlike the firmware (which self-checks for updates in-app), there's no auto-update mechanism for the app itself yet — check back here for a newer `.apk`.

1. Download the latest `.apk` from [`app-release/`](app-release/) (the filename includes the version, e.g. `odki-vbt-v1.0.0.apk`).
2. Android will likely warn that installing from outside the Play Store is blocked — go to **Settings → Apps → Special access → Install unknown apps**, select the app you downloaded with (e.g. your browser or file manager), and allow it. The exact wording/path varies a bit by Android version and manufacturer.
3. Open the downloaded `.apk` and confirm the install.
4. To update later, just download and install the newer `.apk` the same way — it installs over the existing app as long as it's signed with the same key, no need to uninstall first.

## Usage

1. Power on the sensor (see Hardware/Instructions).
2. Open the app, grant Bluetooth and location permissions when prompted (required by Android for BLE scanning).
3. Connect to the sensor from the device list.
4. Calibrate (hold the sensor/bar still and tap Calibrate).
5. Start a set to see live velocity and rep data; each completed rep appears automatically.
6. Firmware updates, when available, are offered in-app.
