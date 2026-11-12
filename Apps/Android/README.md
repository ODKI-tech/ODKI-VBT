# Android

**Status: published.**

## [⬇ Download the Android app (APK)](https://github.com/ODKI-tech/ODKI-VBT/releases/latest/download/odki-vbt.apk)

This link always gives the latest version. All versions and their changes: [Releases](https://github.com/ODKI-tech/ODKI-VBT/releases).

## Download & install

Distributed as a downloadable APK attached to this repository's [GitHub Releases](https://github.com/ODKI-tech/ODKI-VBT/releases) (sideload; a Google Play listing is being prepared but is not public yet). Unlike the firmware (which self-checks for updates in-app), there's no auto-update mechanism for the app itself yet — check back here for a newer version.

1. On your Android phone or tablet, download [`odki-vbt.apk`](https://github.com/ODKI-tech/ODKI-VBT/releases/latest/download/odki-vbt.apk).
2. Android will likely warn that installing from outside the Play Store is blocked — go to **Settings → Apps → Special access → Install unknown apps**, select the app you downloaded with (e.g. your browser or file manager), and allow it. The exact wording/path varies a bit by Android version and manufacturer.
3. Open the downloaded `.apk` and confirm the install.
4. To update later, download and install the newer `.apk` the same way — it installs over the existing app as long as it's signed with the same key, no need to uninstall first.

**Note for maintainers, cutting a new release:** bump `version` in the app repo's `pubspec.yaml`, build the APK (`flutter build apk --release` in the app repo), rename it `odki-vbt.apk` and publish it as a new release tagged `app-vX.Y.Z`:

```
gh release create app-vX.Y.Z odki-vbt.apk --title "Android app X.Y.Z" --notes-file notes.md --latest
```

The asset name must stay `odki-vbt.apk` and the app release must be the one marked **Latest** (firmware releases are created with `--latest=false`, see [`Firmware/firmware-release/README.md`](../../Firmware/firmware-release/README.md)): that is what keeps the `releases/latest/download/odki-vbt.apk` link above, and everywhere else, working without updates. The APK is no longer stored in the repository itself.

## Usage

1. Power on the sensor (see Hardware/Instructions).
2. Open the app, grant the Bluetooth ("Nearby devices") permission when prompted — on Android 12 and newer the app does not ask for location.
3. Connect to the sensor from the device list.
4. Calibrate (hold the sensor/bar still and tap Calibrate).
5. Start a set to see live velocity and rep data; each completed rep appears automatically.
6. Firmware updates, when available, are offered in-app. On a self-built sensor they need the OTAFIX bootloader — see [`Hardware/Instructions/BOOTLOADER.md`](../../Hardware/Instructions/BOOTLOADER.md). The phone or tablet must be online: the app looks for updates (and downloads them) from this repository each time it connects to the sensor, so with no internet connection no update is offered. Keep the sensor close and the app open on screen until the update finishes (the app keeps the screen on; locking the phone or switching apps can stop the transfer). If the transfer stops, the app offers **Retry**; a sensor left waiting in update mode no longer connects normally but is listed in the Bluetooth menu as **Sensor in update mode** with a **FINISH UPDATE** button, and Sensor settings has **FINISH INTERRUPTED UPDATE** as a fallback.
