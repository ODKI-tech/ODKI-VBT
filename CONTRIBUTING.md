# Contributing to ODKI VBT

Thanks for considering it. This project is small and largely built by one person so far, so the process below is intentionally lightweight — the goal is just to keep changes easy to review and the codebase easy for the next person to pick up.

Before anything else: **read the license.** [`LICENSE-FIRMWARE`](LICENSE-FIRMWARE) (GPL-3.0 + Commons Clause) and [`LICENSE-HARDWARE`](LICENSE-HARDWARE) (CC BY-NC-SA 4.0) both allow personal, non-commercial use, modification, and redistribution of your own modified version — but not selling a product built from them, or from a modified version of them; only ODKI does that, under the ODKI trademark. If that's not the deal you want, this isn't the right project to contribute to.

## Contributor License Agreement (CLA)

This project exists so that anyone can make the sensor better for everyone, purely for the satisfaction of having contributed — there's no monetary payment for a merged contribution, and there isn't meant to be. In exchange for public credit as a contributor, submitting a change for inclusion in this repository (a pull request, a patch, a design file) means you agree to the following, before it gets reviewed:

- You grant ODKI a perpetual, worldwide, royalty-free license to use, modify, and redistribute your contribution as part of this project — **including in ODKI's own commercial product** (the sensor and accessories it sells under its trademark), not just in the community-licensed firmware/hardware files.
- You confirm the contribution is your own original work, or that you otherwise have the right to submit it under these terms.
- Your name (or handle, your choice) is credited in the repository's contributor history and, for a non-trivial contribution, in the relevant version note or changelog entry — this is the only form of compensation for a contribution, and by contributing you're confirming that's what you expect.
- Everyone downstream of you — anyone who gets the project *without* going through this CLA — still only gets it under [`LICENSE-FIRMWARE`](LICENSE-FIRMWARE)/[`LICENSE-HARDWARE`](LICENSE-HARDWARE): the CLA only affects the relationship between a contributor and ODKI, it does not change what the wider community non-commercial license grants everyone else.

This only applies to contributions accepted into *this* repository. Your own personal fork, kept outside the official directory, is governed only by the license file, with no CLA obligation — you don't need to agree to any of the above just to build on the project for yourself or share your own non-commercial fork.

**How this is confirmed in practice:** every pull request against this repository uses a template ([`.github/PULL_REQUEST_TEMPLATE.md`](.github/PULL_REQUEST_TEMPLATE.md)) with an explicit checkbox confirming CLA agreement — opening a PR without checking it doesn't grant the rights above, so it won't be merged.

## Reporting a bug

The most useful bug report includes:

- **Firmware version** — the `MAJOR.MINOR.PATCH` from `Config.h`, or read live from the app / any BLE tool via the standard Device Information Service.
- **What you did and what you expected vs. what happened** — as concrete as possible ("rep 4 in a 6-rep set was never reported" beats "rep detection is buggy sometimes").
- **A raw debug-log capture, if the issue is about rep detection or drift correction.** Enable `RuntimeConfig::debugLogEnabled` (see [`Firmware/DOCUMENTATION.md`](Firmware/DOCUMENTATION.md#lab-data-capture-debuglogenabled)), reproduce the issue with a laptop reading the serial port, and attach the log. Run it through [`Firmware/tools/vbt_log_viewer.py`](Firmware/tools/vbt_log_viewer.py) yourself first if you can — its console output (rep recap, anomaly scan) and overview plot often make the actual problem obvious immediately, and are far easier to discuss than a raw description.
- **Hardware/board revision**, if relevant (e.g. an issue that looks specific to a particular battery or IMU batch).

## Proposing a feature or change

Open an issue first for anything non-trivial (a new `RuntimeConfig` field, a change to the BLE wire format, a different rep-detection heuristic) before writing code — the algorithm in particular has a real design history (see the version notes at the top of `MotionTracker.cpp`), and it's better to align on the approach before investing time in an implementation. Small, self-contained fixes (an off-by-one, a stale comment, a doc correction) can just go straight to a PR.

## Submitting a change

- Keep pull requests focused: one logical change per PR, not a bundle of unrelated fixes. Small and reviewable beats large and comprehensive.
- Explain the *why* in the PR description, not just the *what* — especially for anything touching `MotionTracker`, where the reasoning behind a threshold or a gate is usually more important than the line that changes.
- If your change affects rep detection or drift correction, validate it against a real hardware capture (or several) with `vbt_log_viewer.py`, and mention what you checked. There's no automated test suite for the firmware — the algorithm was designed and validated offline against real logs before being ported into `MotionTracker.cpp` (see the v3.11.0 version note there), and that remains the practical way to sanity-check a change to it.
- Update [`Firmware/DOCUMENTATION.md`](Firmware/DOCUMENTATION.md) in the same PR if the change affects behavior it describes (a `RuntimeConfig` default, a wire-format field, a module's responsibility). Undocumented behavior changes are the main way this repo would drift out of date again.

## Code style

**Language:** English, throughout — code comments, commit messages, PR descriptions, Serial/log strings meant for a human to read. This is an international, source-available project (see [License](#contributor-license-agreement-cla) above for what that means in practice).

**License header:** every new source file (firmware `.h`/`.cpp`/`.ino`, tools `.py`) needs the same license header the existing files carry at the top — copy it verbatim from any existing file (e.g. `Firmware/VBT_Quaternions/Config.h`) rather than retyping it, so the wording stays identical everywhere. If the new file bundles or adapts third-party code under a different license (as `MadgwickAHRS.h`/`.cpp` does), say so explicitly instead of using the standard header unmodified — see those two files for the pattern to follow.

**Firmware (`Firmware/VBT_Quaternions/`, C++/Arduino):**
- One `.h`/`.cpp` pair per module, exposing a `namespace` with free functions as its public interface (`MotionTracker::`, `BleServer::`, `PowerManager::`, …) — not a class, except where wrapping a self-contained algorithm that's naturally object-shaped (`MadgwickAHRS`). Match the existing modules' shape for a new one.
- `camelCase` for functions and variables, `PascalCase` for types/enums/structs, `ALL_CAPS` for compile-time constants.
- No dynamic allocation (`new`, `std::vector`, …) in code that runs on the device — fixed-capacity buffers and ring buffers only, sized with an explicit, commented rationale for the capacity chosen (see `MotionTracker.cpp` for examples: `BRACKET_BUFFER_CAPACITY`, `CHUNK_QUEUE_CAPACITY`, etc.). This is an embedded target with a fixed, small RAM budget — allocation failures aren't something to discover at runtime.
- Comment the *why*, not the *what* — a well-named function rarely needs a comment explaining what it does; it's worth one when a threshold, ordering constraint, or edge case would otherwise be non-obvious to the next reader. For a change to `MotionTracker.cpp` specifically, follow the existing pattern of a dated, numbered version note at the top of the file explaining what changed and why (see the v3.11.0–v3.11.7 notes there) — this file's history has mattered more than once when tracking down a regression.

**Tools (`Firmware/tools/`, Python):** match the style already in `vbt_log_viewer.py` — plain functions over classes, `pandas`/`numpy` for data handling, no external dependencies beyond `numpy`/`pandas`/`matplotlib` unless there's a good reason.

**Hardware (`Hardware/`):** CAD source files in an open or widely-supported format (STEP for mechanical parts, KiCad for any PCB work — avoid formats that require a specific paid tool to even open), plus a plain-text BOM with part numbers and a real supplier/link per line, not just a description.

## Documentation

[`Firmware/DOCUMENTATION.md`](Firmware/DOCUMENTATION.md) is written to take a new contributor from "never seen this codebase" to "can confidently modify the rep-detection algorithm." If you change something it describes, update it in the same PR — a documentation fix that lags behind the code it describes is worse than no documentation at all, since it actively misleads the next reader instead of just being silent.
