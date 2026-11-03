# Contributing to ODKI VBT

Thanks for considering it. The process below is intentionally lightweight — the goal is to keep changes easy to review and the codebase easy for the next person to pick up.

Before anything else: **read the licenses** — the map is in [`LICENSE`](LICENSE). In short:

| Area | License | Commercial use |
|---|---|---|
| Tracking engine (`Firmware/VBT_Quaternions/MotionTracker.*`, the tools that compile it in, `Firmware/DOCUMENTATION.md`) | [PolyForm Noncommercial 1.0.0](LICENSE-ENGINE) + ODKI's additional permissions — source-available, not open source | needs a commercial licence from ODKI |
| Rest of the firmware and the tools (`Firmware/`, other files) | [Apache-2.0](LICENSE-APACHE) — open source | allowed under Apache-2.0 |
| BLE protocol spec (`Firmware/BLE_PROTOCOL.md`) | CC0 1.0 | allowed |
| Hardware design (`Hardware/`) | [CC BY-NC-SA 4.0](LICENSE-HARDWARE) | needs ODKI's permission |
| Vendored x-io Fusion (`Firmware/VBT_Quaternions/src/Fusion/`) | MIT | allowed |

A contribution to a file is published under that file's license, and — through the CLA below — ODKI may also license it commercially. If that is not the deal you want, this isn't the right project to contribute to.

## Contributor License Agreement (CLA), version 1.0 of 3 November 2026

This project exists so that anyone can make the sensor better for everyone. There is no payment for a merged contribution. In exchange for public credit as a contributor, submitting a change for inclusion in this repository (a pull request, a patch, a design file, documentation) means you agree to the following. "ODKI" means Lodovico Cortelazzo, trading as ODKI, and his successors and assigns, including a company formed to carry on the project to which these rights are assigned.

1. **Copyright licence.** You grant ODKI a perpetual, worldwide, non-exclusive, royalty-free, irrevocable licence to use, copy, modify, adapt, combine with other works, distribute and publish your contribution, **and to sublicense and relicense it under any terms, including commercial licences** — in ODKI's own products, in the publicly licensed files, and in licences ODKI grants to others (for example a company integrating the firmware into its product). You keep the copyright in your contribution and can use it in any other way you like.
2. **Patent licence.** You grant ODKI, and everyone who receives your contribution from ODKI under any of those licences, a perpetual, worldwide, non-exclusive, royalty-free, irrevocable (except as stated here) patent licence to make, use, sell, offer for sale, import and otherwise transfer your contribution, for the patent claims you can license that your contribution, alone or combined with the project, would infringe. If you start patent litigation claiming that the project infringes a patent, the patent licences you received for the project from ODKI end.
3. **Your right to submit.** You confirm the contribution is your own original work, or that you otherwise have the right to submit it under these terms, and you identify any third-party code or material in it, with its license. If your employer or a client has rights in it (for example because you wrote it as part of your job), you confirm that they have given you permission to submit it under these terms or have waived those rights.
4. **Moral rights.** Where the law allows (in Italy, articles 20–22 of law 633/1941: moral rights cannot be transferred), you consent to ODKI and its licensees modifying, adapting, translating, shortening and combining your contribution with other works, and to it being attributed through the repository's history and contributor credits rather than in every copy; to the extent your law allows a waiver, you waive any right to object to such changes.
5. **Age.** You are of legal age where you live, or a parent or guardian agrees to this CLA on your behalf (say so in the pull request).
6. **No warranty.** You provide your contribution "as is", without warranties, except for point 3.
7. **Credit.** Your name (or handle, your choice) is credited in the repository's history and, for a non-trivial contribution, in the relevant version note or changelog entry — this is the only form of compensation, and by contributing you confirm that is what you expect.
8. **Everyone else.** Anyone who gets the project *without* going through this CLA gets it only under the licenses in [`LICENSE`](LICENSE); the CLA affects only the relationship between you and ODKI.
9. **Governing law.** This CLA is governed by Italian law; the courts of Italy have jurisdiction, without prejudice to mandatory consumer-protection rules where you live.

This applies only to contributions submitted to *this* repository. Your own fork, kept outside it, is governed only by the license files.

**How acceptance is recorded:** the pull-request template ([`.github/PULL_REQUEST_TEMPLATE.md`](.github/PULL_REQUEST_TEMPLATE.md)) asks you to tick a box **and** to leave the statement "I have read the ODKI VBT CLA version 1.0 and I agree to it." in the pull-request description or in a comment, with your GitHub account. That comment, the PR's history and the version of this file at the PR's base commit are the record. A pull request without it is not reviewed or merged. (An automated tool such as CLA Assistant may replace this later; it would record the same three things.)

## Reporting a bug

The most useful bug report includes:

- **Firmware version** — the `MAJOR.MINOR.PATCH` from `Config.h`, or read live from the app / any BLE tool via the standard Device Information Service.
- **What you did and what you expected vs. what happened** — as concrete as possible ("rep 4 in a 6-rep set was never reported" beats "rep detection is buggy sometimes").
- **A raw debug-log capture, if the issue is about rep detection or drift correction.** Since v3.11.24 the sensor prints this log automatically to any open USB-serial connection (see [`Firmware/DOCUMENTATION.md`](Firmware/DOCUMENTATION.md#lab-data-capture-serial-log)) — open one with a terminal, `Firmware/tools/vbt_live_monitor.py`, or any serial monitor, reproduce the issue, and attach the log. Run it through [`Firmware/tools/vbt_log_viewer.py`](Firmware/tools/vbt_log_viewer.py) yourself first if you can — its console output (rep recap, anomaly scan) and overview plot often make the actual problem obvious immediately, and are far easier to discuss than a raw description.
- **Hardware/board revision**, if relevant (e.g. an issue that looks specific to a particular battery or IMU batch).

## Proposing a feature or change

Open an issue first for anything non-trivial (a new `RuntimeConfig` field, a change to the BLE wire format, a different rep-detection heuristic) before writing code — the algorithm in particular has a real design history (see the version notes at the top of `MotionTracker.cpp`), and it's better to align on the approach before investing time in an implementation. Small, self-contained fixes (an off-by-one, a stale comment, a doc correction) can just go straight to a PR.

## Submitting a change

- Keep pull requests focused: one logical change per PR, not a bundle of unrelated fixes. Small and reviewable beats large and comprehensive.
- Explain the *why* in the PR description, not just the *what* — especially for anything touching `MotionTracker`, where the reasoning behind a threshold or a gate is usually more important than the line that changes.
- If your change affects rep detection or drift correction, validate it against a real hardware capture (or several) with `vbt_log_viewer.py` — and, to compare an engine change on a capture without re-flashing, replay it through the real engine with `Firmware/tools/host_replay/` — and mention what you checked. The automated checks are `Firmware/tools/full_replay/regression.py` (the whole chain — orientation filter and engine — on committed synthetic data; a change that is meant to alter the output must say so and update the golden file with `--update`), `Firmware/tools/calibration_seed/run.py` and `Firmware/tools/validation/tests/test_pipeline.py`; they do not replace a check on real captures.
- Update [`Firmware/DOCUMENTATION.md`](Firmware/DOCUMENTATION.md) in the same PR if the change affects behavior it describes (a `RuntimeConfig` default, a wire-format field, a module's responsibility). Undocumented behavior changes are the main way this repo would drift out of date again.

## Code style

**Language:** English, throughout — code comments, commit messages, PR descriptions, Serial/log strings meant for a human to read. This is an international project (see the licenses above for what is open source and what is source-available).

**License header:** every new source file (firmware `.h`/`.cpp`/`.ino`, tools `.py`/`.cpp`/`.h`) starts with the header of its area, copied verbatim from an existing file rather than retyped: the Apache-2.0 header (`SPDX-License-Identifier: Apache-2.0`, e.g. from `Firmware/VBT_Quaternions/Config.h`) for everything except the engine, the engine header (`SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0` plus the `Required Notice:` line, from `MotionTracker.h`) for the engine and for any file that `#include`s `MotionTracker.cpp`. Don't copy third-party code into the repository without opening an issue first; if it is accepted it keeps its own license and notice, as the vendored Fusion library in `Firmware/VBT_Quaternions/src/Fusion/` does (and is listed in `THIRD_PARTY_NOTICES.md`).

**Firmware (`Firmware/VBT_Quaternions/`, C++/Arduino):**
- One `.h`/`.cpp` pair per module, exposing a `namespace` with free functions as its public interface (`MotionTracker::`, `BleServer::`, `PowerManager::`, …) — not a class, except where wrapping a self-contained algorithm that's naturally object-shaped (`FusionOrientation`). Match the existing modules' shape for a new one.
- `camelCase` for functions and variables, `PascalCase` for types/enums/structs, `ALL_CAPS` for compile-time constants.
- No dynamic allocation (`new`, `std::vector`, …) in code that runs on the device — fixed-capacity buffers and ring buffers only, sized with an explicit, commented rationale for the capacity chosen (see `MotionTracker.cpp` for examples: `BRACKET_BUFFER_CAPACITY`, `CHUNK_QUEUE_CAPACITY`, etc.). This is an embedded target with a fixed, small RAM budget — allocation failures aren't something to discover at runtime.
- Comment the *why*, not the *what* — a well-named function rarely needs a comment explaining what it does; it's worth one when a threshold, ordering constraint, or edge case would otherwise be non-obvious to the next reader. For a change to `MotionTracker.cpp` specifically, follow the existing pattern of a dated, numbered version note at the top of the file explaining what changed and why (see the v3.11.0–v3.11.7 notes there) — this file's history has mattered more than once when tracking down a regression.

**Tools (`Firmware/tools/`, Python):** match the style already in `vbt_log_viewer.py` — plain functions over classes (Tkinter windows are the exception, see `vbt_live_monitor.py` and the GUI in `vbt_log_viewer.py`), `pandas`/`numpy` for data handling, no external dependencies beyond `numpy`/`pandas`/`matplotlib` unless there's a good reason.

**Hardware (`Hardware/`):** CAD source files in an open or widely-supported format (STEP for mechanical parts, KiCad for any PCB work — avoid formats that require a specific paid tool to even open), plus a plain-text BOM (`Hardware/BOM/bom.csv`) with quantities, part numbers where there is one, and the key specs to match — deliberately no supplier links, see [`Hardware/BOM/README.md`](Hardware/BOM/README.md).

## Documentation

[`Firmware/DOCUMENTATION.md`](Firmware/DOCUMENTATION.md) is written to take a new contributor from "never seen this codebase" to "can confidently modify the rep-detection algorithm." If you change something it describes, update it in the same PR — a documentation fix that lags behind the code it describes is worse than no documentation at all, since it actively misleads the next reader instead of just being silent.
