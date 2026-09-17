# Android

**Status: published.** [Download the latest APK](https://github.com/ODKI-tech/ODKI-VBT/raw/main/Apps/Android/app-release/odki-vbt-latest.apk).

## Download & install

Distributed as a downloadable APK directly from [`app-release/`](app-release/) in this repository (sideload — no Google Play listing planned initially). Unlike the firmware (which self-checks for updates in-app), there's no auto-update mechanism for the app itself yet — check back here for a newer `.apk`.

1. Download [`odki-vbt-latest.apk`](https://github.com/ODKI-tech/ODKI-VBT/raw/main/Apps/Android/app-release/odki-vbt-latest.apk) — always the current release, regardless of version (see the note for maintainers below). Use the `github.com/.../raw/...` link specifically, not a plain `raw.githubusercontent.com` one — the APK is stored with Git LFS (see `.gitattributes`), and only the former correctly redirects to the actual binary; the latter serves the LFS pointer text instead (133 bytes, not the app).
2. Android will likely warn that installing from outside the Play Store is blocked — go to **Settings → Apps → Special access → Install unknown apps**, select the app you downloaded with (e.g. your browser or file manager), and allow it. The exact wording/path varies a bit by Android version and manufacturer.
3. Open the downloaded `.apk` and confirm the install.
4. To update later, just download and install the newer `.apk` the same way — it installs over the existing app as long as it's signed with the same key, no need to uninstall first.

**Note for maintainers, cutting a new release:** build the APK (`flutter build apk --release` in the app repo), overwrite `app-release/odki-vbt-latest.apk` with it, delete any other `.apk` left over in this folder, and bump `version` in the app repo's `pubspec.yaml` — same "keep only the latest, git history is the archive" convention as [`Firmware/firmware-release/`](../../Firmware/firmware-release/README.md). Keeping the filename itself stable (not versioned) is deliberate: it's what lets the link above, and everywhere else in this repo's docs, never need updating.

## Usage

1. Power on the sensor (see Hardware/Instructions).
2. Open the app, grant Bluetooth and location permissions when prompted (required by Android for BLE scanning).
3. Connect to the sensor from the device list.
4. Calibrate (hold the sensor/bar still and tap Calibrate).
5. Start a set to see live velocity and rep data; each completed rep appears automatically.
6. Firmware updates, when available, are offered in-app.
