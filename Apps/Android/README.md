# Android

**Status: not yet published.** Link coming soon.

## Download & install

Distributed as a downloadable APK attached to this repository's GitHub Releases (sideload — no Google Play listing planned initially). <!-- TODO: once this repo is pushed to GitHub, link this to https://github.com/<org>/<repo>/releases -->

1. Go to this repository's **Releases** page (the "Releases" link in the sidebar on GitHub) and download the `.apk` from the latest release tagged `app-vX.Y.Z`.
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
