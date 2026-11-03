## What this changes and why

<!-- The *why* matters more than the *what* here, especially for anything touching MotionTracker — see CONTRIBUTING.md. -->

## Contributor License Agreement

<!-- Required. Keep the sentence below exactly as it is (it is the record of your acceptance). If you are under legal age, a parent or guardian must agree too: say so here. -->

I have read the ODKI VBT CLA version 1.0 and I agree to it.

- [ ] I agree to the [Contributor License Agreement](../CONTRIBUTING.md#contributor-license-agreement-cla-version-10-of-3-november-2026) in CONTRIBUTING.md: ODKI may use, modify, distribute, sublicense and relicense this contribution — including under commercial licences and in ODKI's own products — I grant the patent licence described there, and I'll be credited as a contributor. This contribution is my own original work, or I otherwise have the right to submit it under these terms (including my employer's permission where needed).

## Checklist

- [ ] If this changes rep-detection, drift-correction or orientation behaviour, I validated it against real hardware captures with `vbt_log_viewer.py` / `tools/full_replay/` and ran `tools/full_replay/regression.py` (describe below).
- [ ] If this changes behaviour described in `Firmware/DOCUMENTATION.md` or `Firmware/BLE_PROTOCOL.md`, I updated the relevant section in the same PR.
- [ ] New source files carry the license header of their area (see CONTRIBUTING.md's Code style section); third-party code, if any, is identified with its license.

## Validation (if applicable)

<!-- What you checked, and against which capture(s). -->
