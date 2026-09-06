# App release

The signed release `.apk` lives here once published — see [`../README.md`](../README.md) for download/install steps.

## Cutting a release (maintainer notes)

The app's source lives in a separate, private repository (not this one — the app is closed-source, see the license note in [`../../README.md`](../../README.md)).

1. Bump the version in `pubspec.yaml` (`version: X.Y.Z+buildNumber`).
2. Make sure `android/key.properties` exists locally (see `android/key.properties.example` in the app repo) — without it, a release build silently falls back to debug signing instead of failing loudly, so double-check the build actually got signed with the release key (`apksigner verify --print-certs` on the output `.apk`, or check `flutter build apk --release` output for "Signing... release").
3. `flutter build apk --release`
4. Rename the output (`build/app/outputs/flutter-apk/app-release.apk`) to `odki-vbt-vX.Y.Z.apk`, drop it here (replacing the previous one — git history keeps the old ones if ever needed), and update the version/filename mentioned in `../README.md`.
5. Commit and push to `main`.
