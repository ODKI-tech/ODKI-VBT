# Firmware release

The live OTA update the companion app checks for and downloads (see `FirmwareUpdateService` in the app's source, and the BLEDfu/buttonless-DFU chapter in [`../DOCUMENTATION.md`](../DOCUMENTATION.md)) — this folder, not a GitHub Release, is the actual source of truth the app reads at runtime.

- **`manifest.json`** — a single static JSON file the app fetches over HTTPS (via `raw.githubusercontent.com`) on every launch to check for an update: `{"version": "MAJOR.MINOR.PATCH", "url": "<direct link to the matching .zip>", "notes": "<changelog shown to the tester>"}`. **Keep `notes` a plain string with no unescaped quotes** — an earlier release once broke the update check entirely this way (invalid JSON silently fails `jsonDecode`, which the app treats as "no update available", not an error).
- **`vbt_firmware_X.Y.Z_OTA.zip`** — the actual Nordic Secure DFU package for that version. Only the *latest* one needs to stay here (older ones can be deleted once superseded — the app only ever looks at `manifest.json`'s current `url`, there's no version history to preserve in this folder specifically; the git history of this folder already preserves every past `.zip` if one is ever needed).

## Cutting a release

1. Bump `FirmwareVersion::PATCH` (or `MINOR`/`MAJOR`) in [`../VBT_Quaternions/Config.h`](../VBT_Quaternions/Config.h), matching the version note already added at the top of the changed file(s) (see `MotionTracker.cpp`'s convention).
2. Compile for `Seeeduino:nrf52:xiaonRF52840Sense` — the Arduino/Adafruit nRF52 core produces a ready-to-use DFU package directly as part of the build (`<sketch>.ino.zip`, containing `.bin` + `.dat` + an internal `manifest.json` describing the device type/softdevice requirement — a different, unrelated file from this folder's own `manifest.json` above). No separate packaging step (`adafruit-nrfutil`) is needed on top of that.
3. Rename it to `vbt_firmware_X.Y.Z_OTA.zip`, replace the previous one here, and delete the old `.zip`.
4. Update this folder's `manifest.json`: new `version`, `url` pointing at the new filename, and `notes` describing the change for testers (validate it's well-formed JSON before committing — `python3 -m json.tool manifest.json`).
5. Commit and push to `main` — the app reads directly from the raw file on the default branch, no separate publish step needed.
