# ODKI-VBT validation protocol (pre-registered)

**Status: not yet validated.** This protocol, the analysis code and the acceptance thresholds are published **before any data is collected**. The git commit that adds this file is its timestamp. Any later change is listed in section 14, *Deviations*, with its date and the reason for it. The results will be published whatever they show.

| | |
|---|---|
| Version | 1.0 (draft for sign-off, see section 15). Revised on 26/10/2026, before any data was collected: the reference instrument changed to a linear encoder |
| Device | ODKI-VBT sensor (Seeed XIAO nRF52840 Sense, LSM6DS3TR-C IMU) |
| Firmware under test | fixed for the whole study: **3.11.32** if released before the pilot, otherwise 3.11.31 (**TBD**, section 15) |
| Reference | a linear encoder (linear position transducer), verified as in section 4 |
| Study type | in-house engineering study by ODKI. Not independent, and not a laboratory study |
| Code | this folder: `capture.py`, `import_ref.py`, `align.py`, `pair_reps.py`, `analyze.py` |
| Data licence | **CC BY 4.0** (dataset); the code keeps the repository's licence (`LICENSE-FIRMWARE`) |

## 1. Question

How closely do the per-rep values that the ODKI-VBT sensor reports agree with a verified linear encoder attached to the same bar? This covers mean concentric velocity (primary), peak velocity, mean propulsive velocity, range of motion and rep detection, for the back squat and the bench press over 20–90% of an estimated 1RM. A second question is how repeatable the sensor's readings are between two units and between two days.

The study measures **agreement with a linear encoder**. A validation against 3D motion capture (the gold standard in Wannouch et al. 2025) is a later, separate phase run with a university laboratory (section 13).

## 2. Design: reduced Phase 3 (about 7 h of operator time)

The reduced design was approved on 26/10/2026 in place of the full design (10 lifters, three exercises, 12–15 h).

| Item | Plan |
|---|---|
| Phases | **P (pilot):** the operator only, squat and bench, 2 units, encoder + video. It checks the reference and the pipeline; its numbers are **not** published. **Phase 3:** the study below |
| Lifters | **6** adults (≥ 18), training with a free barbell for ≥ 1 year, volunteers, unpaid. Each completes the non-medical self-screening and signs the written consent before the first session. Minors are excluded |
| Exercises | back squat (free barbell) and flat bench press. Deadlift, Smith machine, jumps and throws are **out of scope** (jumps and throws may be shorter than the firmware's 0.35 s minimum phase duration) |
| Loads | 20, 40, 60, 80 and 90% of an **estimated** 1RM (from the lifter's training history or a load–velocity profile; **no maximal test**). **Option, TBD:** if no insurance cover is confirmed for third-party participants, loads are capped at **80% 1RM**. The > 90% band and part of the < 0.5 m/s band are then not tested, and this is declared in the results |
| Reps per set | 20–60%: 3 reps; 80%: 2; 90%: 1–2. 2 sets per load, 2–3 min rest, maximal intended concentric speed. Loads only go up |
| Days | 2 sessions per lifter, 48–72 h apart, at the same time of day with the same warm-up and load order (test–retest) |
| Scheduling | 2 lifters per session on alternating sets, so 6 sessions of about 70 min each, about 7 h in total |
| Units | **2 sensors** side by side on the same sleeve side; positions swapped on day 2 |
| Placement | on the shaft, inside the collar. The encoder is attached on the same side, within 5 cm of the sensors |
| Expected size | 6 lifters × 2 exercises × about 25 reps × 2 days ≈ **600 reps**, so about 300 per exercise and about 1200 sensor–reference pairs with 2 units |
| Safety | rack with safety bars, a spotter always present and free of other tasks above 80% 1RM, standardised warm-up, immediate-stop criteria (pain, dizziness, failing technique, the participant's request) |

**Sample size.** The 95% CI half-width of a Bland–Altman limit of agreement is about 1.96·√(3s²/n) ≈ 3.39·s/√n, where s is the SD of the differences. n ≥ 128 pairs per exercise keeps it within 0.3·s, and about 300 per exercise gives about 0.2·s. For the ICC (Bonett 2002; expected ICC 0.90, two methods, CI width 0.10) about 61 subjects are needed. The test–retest units here are lifter × load = 30 per exercise, so the test–retest CIs will be wider than that target, and this is declared. For rep detection, about 600 reps estimate a 98% detection rate within about ±1.1%. The velocity bands at the extremes will hold fewer reps; a band with fewer than 30 pairs is reported descriptively only.

## 3. What is measured

| Source | Content | Rate |
|---|---|---|
| ODKI USB serial log (`S,` `R,` `B,` `P,` rows, see `Firmware/DOCUMENTATION.md`, "Lab data capture") | per sample: raw velocity, drift-correction brackets, raw accelerations; per rep: every reported value, first and final; per phase: close reason, plus start and end time from firmware 3.11.32 | 100 Hz |
| Linear encoder, recorded by its own software | the raw position-vs-time (or velocity-vs-time) export, on the encoder's own clock, converted with `import_ref.py` to `t_s, position_m`. Per-rep values computed by the encoder's software are not used | the encoder's own rate, recorded in the dataset (**TBD**, section 15) |
| Capture sidecar | PC arrival time of every ODKI line, relative to the start of the capture. Diagnostics only, not used for alignment | per line |
| Video (240 fps, **TBD**) | pilot, plus a subset of Phase 3, used only to verify the reference (section 4, step 4). **Never part of the dataset** | — |

The BLE output reproduces the same values as the `R,` rows (same engine), but it carries no timestamps. One or two cable-free control sets are recorded over BLE to confirm that the serial cable does not change the reps.

## 4. Reference instrument

The reference is a linear encoder (linear position transducer) owned by ODKI, recorded with its own software. Only its raw position-vs-time (or velocity-vs-time) export is used, so every reference value in section 6 is computed by the same code from the same kind of signal. The encoder is used only after it passes this verification, with the limits fixed in advance:

1. **Static scale**: 10 displacements of 1000 mm against a rigid rule or tape measure, CV < 0.3%. The mean of rule length / encoder reading is the scale factor (`ref_scale`) the analysis applies to the reference position. If it differs from 1 by more than 1%, this is reported.
2. **Free fall**, if the encoder's attachment allows it: the acceleration fitted to the position trace of a freely falling 5–10 kg mass (10 trials) is within 1% of the expected value (g, less any retraction force of the encoder divided by the mass). Skipped and declared otherwise.
3. **Sampling**: the export's sampling rate and position resolution are recorded. Within a set, no gap may exceed two sample intervals, otherwise the set is flagged.
4. **240 fps video**: 20 reps compared encoder vs Kinovea marker tracking (4th-order zero-phase Butterworth, 8 Hz), reported as bias and LoA. Skipped and declared if the phone cannot record at 240 fps (**TBD**).

Error budget at about 1 m/s: the resolution and rate of the export, stated in the results (a position resolution of 0.1 mm at ≥ 100 Hz contributes < 0.01 m/s per sample, negligible on a mean); scale ≤ 0.3% after the static check; cosine error < 0.1% if the measured line is within 2.5° of vertical; clock difference handled by alignment (section 5). Bar tilt between the two ends is avoided by putting the sensor and the encoder's attachment on the same side, within 5 cm of each other.

## 5. Synchronisation, capture and pairing

**5.1 Capture.** One laptop reads every ODKI unit over USB serial (`capture.py`). The ODKI log is saved byte for byte, so it stays readable by `vbt_log_viewer.py` and `host_replay`. Each line's arrival time is saved separately, relative to the start of the capture: the files contain no date and no time of day. The linear encoder is recorded at the same time by its own software, on its own clock; its export is converted with `import_ref.py` to `raw/ref/<physical_set_id>.csv` (section 9). Per set: `CALIBRATE` with the bar still (at the start of each session and after any position change), start the encoder recording, `START`, **sync event**, set, 2 s still, `STOP`, stop the encoder recording. One file per set and device. The `RuntimeConfig` is the default and the same for the whole study.

**Sync event.** Before the first rep, lift the bar a few cm off the hooks and let it drop back onto them.

**5.2 Alignment** (`align.py`).
The two recordings share no clock (they come from different programs), so the alignment uses the signals only.
1. *Coarse:* full-range cross-correlation of the ODKI drift-corrected velocity with the reference velocity on a common 100 Hz grid.
2. *Fine:* the same correlation, with the reference velocity as the displacement over a centred 10 ms window (interpolated from the export), searched ±300 ms around the coarse estimate in 1 ms steps, with parabolic refinement. The encoder's sign convention is detected automatically.
3. *Clock drift:* the same search on each half of the set gives the drift in ppm, corrected linearly. It is estimated only on sets ≥ 25 s. On shorter sets a quartz drift is below 1 ms and is not estimated.
4. *Check:* the peak of the ODKI raw vertical acceleration at the sync drop and the peak of the encoder acceleration must coincide within **±10 ms**, and the velocity correlation must exceed 0.90. A set that fails either check is **flagged and excluded** from the agreement statistics, and the number of such sets is reported.

**5.3 Rep segmentation and pairing** (`pair_reps.py`).
- *Reference rep:* a maximal run of encoder velocity > 0, with runs closer than 50 ms merged, kept if it rises ≥ 10 cm.
- *ODKI rep:* every rep with an `R,` row. Its concentric interval comes from the `P,` row's start and end time (firmware ≥ 3.11.32) or, on older logs, is rebuilt from the `S,` rows.
- *Pairing:* one-to-one, greedy on temporal intersection-over-union ≥ 0.5. Paired = true positive (TP); ODKI only = false rep (FP); reference only = missed rep (FN).

## 6. Endpoints

| Endpoint | ODKI | Reference |
|---|---|---|
| **Primary: mean concentric velocity (MV)**, final value | `meanVelocity` of the last `R,` row of the rep | MV with the **ODKI definition**: from the first sample ≥ 0.20 m/s, back-filled by up to 5 samples ≥ 0.05 m/s, to the last sample ≥ 0.05 m/s. This isolates measurement error from definition error |
| MV, standard definition | same | mean of all samples with v > 0 (usual LPT definition), so definition and measurement error together |
| MV, first reported value | first `R,` row (what the athlete sees first) | as above |
| Peak velocity (PV) | `peakVelocity`, first and final | max v |
| Mean propulsive velocity (MPV) | computed offline from the drift-corrected curve, labelled "ODKI offline" (the firmware does not report it) | from the start to where acceleration first falls below −g after peak velocity (Sánchez-Medina et al. 2010) |
| Range of motion | `displacementM` | integral of v over the concentric |
| Rep detection | TP / FN / FP | — |

The firmware back-fills on the sign of a drifting raw velocity, which no reference can reproduce. The 0.05 m/s floor makes the reference value independent of sampling phase and encoder quantisation. In the synthetic self-test it reproduces the definition within 0.01 m/s on ≥ 97% of reps.

## 7. Statistics (`analyze.py`, deterministic)

- **Validity**, per endpoint and value (first or final):
  - bias and 95% limits of agreement, from a plain Bland–Altman and from a Bland–Altman for repeated measures (Bland & Altman 2007; reps nested in lifters);
  - proportional bias: slope of the difference on the mean;
  - **ICC(2,1)**, absolute agreement;
  - **SEM** (typical error, SD of the differences / √2) and MDC95;
  - **CV%**, as the log-transformed typical error (Hopkins);
  - **MAE**, RMSE, **Pearson r**, SEE and standardised bias (bias / SD of the reference).
- **Rep detection:** confusion matrix, recall, precision, F1 and false-rep rate, with Wilson 95% CIs.
- **Between units:** ICC(2,1), SEM and CV% of the two units on the same reps.
- **Test–retest** (per lifter × exercise × load, mean of the day): **ICC(3,1)** (consistency, and the absolute-agreement form), SEM, MDC95 and SWC = 0.2 × the between-lifter SD. The same statistics are computed on the reference, which separates biological variation from device error.
- **Strata:**
  - exercise;
  - velocity band of the reference MV: < 0.5, 0.5–1.0 and > 1.0 m/s;
  - final correction status;
  - unit;
  - phase close reason.
- **`quality1`:** Spearman ρ with |MV error|, and ROC AUC for detecting |error| > 0.10 m/s. Only if this shows a useful relation may `quality1` be described as more than a "signal-consistency score".
- **CIs:** percentile bootstrap resampling lifters (2000 resamples, fixed seed).
- **Exclusions:** only the pre-registered ones: flagged alignment (section 5.2) or sampling (section 4), and sets with a documented equipment failure, logged *before* the analysis is run. Every exclusion is listed in the results. No outlier removal.

The pipeline is checked on synthetic data with a known bias, noise, clock offset and drift, a reference scale error, a reversed encoder, and missed and false reps (`tests/test_pipeline.py`). It also checks the ICC against the worked example in Shrout & Fleiss 1979.

## 8. Acceptance thresholds (fixed before data)

Evaluated on the **primary endpoint** (MV, final value, squat and bench pooled), and also reported per exercise and per velocity band. Single thresholds are arbitrary (Wannouch et al. 2025), so each tier uses **at least three statistics**, as that review recommends. The same values are coded in `common.THRESHOLDS`.

| Tier | Criteria (all must hold) | Source |
|---|---|---|
| **A: comparable to published results for commercial IMU sensors** | r ≥ 0.95; SEM ≤ 0.06 m/s; \|bias\| ≤ 0.05 m/s; CV < 10%; LoA half-width ≤ 0.12 m/s; ICC(2,1) ≥ 0.90 | Achermann et al. 2023 (best commercial IMU vs 3D motion capture: MV r 0.966, SEE 0.059 m/s); Weakley et al. 2021 (CV < 10%); Koo & Li 2016 (ICC > 0.90 "excellent") |
| **B: minimum acceptable in the literature** | r > 0.70; CV < 10%; standardised bias < 0.6 | Weakley et al. 2021 |
| **Internal target** (not from a published source) | \|bias\| ≤ 0.03 m/s; LoA half-width ≤ 0.10 m/s; CV ≤ 5%; ICC(2,1) ≥ 0.90 | ODKI, declared in advance |
| **Rep detection** (internal target) | recall ≥ 98%; false-rep rate ≤ 2% | ODKI, declared in advance |

The literature already identifies peak velocity, loads above 90% 1RM and reps slower than about 0.3 m/s as weak points for all accelerometer-based devices (Weakley et al. 2021). These are reported as such and do not count towards the tier.

**Consequence.** Whichever tier is reached, or none, is what gets published, with the full data. Below tier B, no accuracy claim is made at all (section 11).

## 9. Dataset (CSV, UTF-8, SI units, `.` as the decimal separator)

The dataset is published on Zenodo, with a DOI, under **CC BY 4.0** (publication needs the CEO's OK). The code stays in this repository.

| File | Columns |
|---|---|
| `lifters.csv` | `lifter_id` (random, `L-xx`), `age_band` (10-year bands), `training_years_band`, `sex` (only if the analysis uses it). **No height, no body mass** |
| `sessions.csv` | `session_id, lifter_id, day_index (1/2), exercise, firmware_version, firmware_commit, runtime_config_hex, reference_type, reference_id, reference_rate_hz, operator ("ODKI"), notes` (empty or cleaned by hand). **No dates or times** |
| `units.csv` | `session_id, unit_id, placement, side, distance_to_ref_mm, battery_pct_start` |
| `reference_verification.csv` | `reference_id, test (static/free_fall/sampling/video), trial, expected, measured, unit` (static: rule length and encoder reading, same unit) |
| `sets.csv` | `set_id (<physical_set_id>_<unit_id>), physical_set_id, session_id, unit_id, load_band_kg (10 kg bands, or omitted), pct_1rm_est, planned_reps, video (yes/no)` |
| `sets_alignment.csv` | generated: `set_id, lag_ms, drift_ppm, drift_estimated, set_duration_s, xcorr_r, sync_check_ms, flagged, sign, ref_scale` |
| `reps_paired.csv` | generated, one row per TP/FN/FP: see `pair_reps.COLUMNS` (reference MV in both definitions, PV, MPV and ROM; ODKI first and final MV, PV and ROM; offline MPV; eccentric MV; quality1; correction status first and final; close reason; exclusion reason) |
| `raw/odki/<set_id>.log` | the original ODKI serial log, unmodified (device uptime only, no wall-clock time) |
| `raw/odki/<set_id>.sidecar.csv` | `line_no, host_ns, kind` (`host_ns` relative to the start of the capture) |
| `raw/ref/<physical_set_id>.csv` | `t_s, position_m`: the encoder's clock in s, relative to the start of its recording, and the position in m before the scale factor (written by `import_ref.py`). The encoder software's original export files are kept offline and not published |
| `aligned/<set_id>.csv` / `.json` | generated: `t_s, v_ref, v_odki_corrected, v_odki_raw, acc_odki_raw`, plus alignment metadata |
| `results/` | generated: `summary.json`, `RESULTS.md`, plots |

## 10. Consent, anonymisation and data handling

- **Who takes part:** adults only (≥ 18). Before the first session each participant receives the information sheet and the privacy notice, completes a **non-medical self-screening** and signs a written consent. **No health information is recorded**; the only inclusion criterion is "no current injury that prevents lifting" (yes/no, not stored). Withdrawal is possible at any time until publication. Published anonymous data cannot be withdrawn afterwards, and the consent form says so.
- **Random IDs:** `L-xx` codes are assigned at random, never initials or birth dates. The ID-to-name table and the signed forms stay offline, outside any repository, and the table is destroyed at the end of the study.
- **Indirect identifiers:** age in 10-year bands; **no height and no body mass**; sex only if the analysis uses it. Before publication every combination of indirect attributes must cover at least 5 participants, otherwise the columns are aggregated or dropped.
- **Loads:** `% 1RM` estimate plus, at most, **10 kg bands**. Exact kg are never published, because kg plus %1RM gives away a lifter's maximum.
- **Time:** **no dates or times of day** anywhere. Sessions become `day_index`; capture times are relative to the start of each set; file names contain no names or dates.
- **Video:** never in the dataset. Videos frame the bar and body (no faces where possible, no bystanders), are used only for reference verification, stay off cloud backups, and are deleted at the end of the study.
- **Before publication:** the raw logs are checked for device names, host names and absolute timestamps. The Zenodo metadata gives the collection period as a month only.

## 11. What we will and won't claim

**Before results** (the only accuracy statements allowed):
- "Not yet validated against a linear position transducer. Validation is the current roadmap stage (05). We'll publish the protocol and the raw data, whatever the result."
- "Validation in progress: protocol published [link]."

**After results**, and only with the context in the same block:
- "Mean velocity vs a linear encoder (verified as in the protocol): mean bias X m/s, 95% limits of agreement L1 to L2 m/s, ICC Y. N lifters, N reps, back squat and bench press, 20–90% 1RM (or 20–80%). Firmware x.y.z. In-house study by ODKI. Full protocol and raw data: [DOI]."
- The wording is **"agreement with a linear encoder"**. It is never "validated", "accurate", "precise", "lab-grade", "research-grade" or "gold standard", and never "independent".
- Results apply to the exercises, load range and firmware tested. Untested exercises and conditions are listed as "not tested".
- No comparison with any named commercial device, because none was measured in the same study.
- `quality1` is a "signal-consistency score, not an accuracy estimate" unless section 7 shows otherwise.
- "Validated", with no qualifier, is kept for the laboratory phase against 3D motion capture (section 13).

## 12. Reproducing the analysis

```
python align.py --data DATA --all
python pair_reps.py --data DATA
python analyze.py --data DATA          # -> DATA/results/summary.json, RESULTS.md, PNGs
```

Anyone with the dataset gets the same numbers, because the bootstrap seed is fixed. See [`README.md`](README.md).

## 13. Later phases

- **Laboratory phase (2027):** the same protocol with a university laboratory using 3D motion capture or a laboratory linear position transducer, with the laboratory as authors and under its ethics approval.
- **Regression at every firmware release:** the dataset logs are replayed through `host_replay` against the new engine.

## 14. Deviations

None so far. Every change after the first Phase 3 session is listed here with its date and reason.

## 15. Open items to settle before the pilot (TBD, CEO)

| Item | Default if not decided |
|---|---|
| Sign-off of the thresholds in section 8 | not frozen until signed |
| Firmware for the whole study: 3.11.32 (log-only additions) or 3.11.31 | 3.11.31 |
| Export format of the linear encoder: raw position or velocity time series and its sampling rate (if it only exports per-rep values, sections 5 and 9 are revised before the pilot) | — |
| Phone records at 240 fps | video step skipped and declared |
| Insurance cover for third-party participants | loads capped at 80% 1RM |
| Session dates and place; number of lifters (6 planned) | — |
| Optional OSF pre-registration in addition to this commit | not done |

## References

- Achermann B. et al. (2023). Velocity-based strength training: the validity and personal monitoring of barbell velocity with the Apple Watch. *Sports* 11(7):125. https://pmc.ncbi.nlm.nih.gov/articles/PMC10383699/
- Bland J.M., Altman D.G. (2007). Agreement between methods of measurement with multiple observations per individual. *J Biopharm Stat* 17(4):571–582. doi:10.1080/10543400701329422
- Bonett D.G. (2002). Sample size requirements for estimating intraclass correlations with desired precision. *Stat Med* 21:1331–1335. doi:10.1002/sim.1108
- Koo T.K., Li M.Y. (2016). A guideline of selecting and reporting intraclass correlation coefficients for reliability research. *J Chiropr Med* 15(2):155–163. doi:10.1016/j.jcm.2016.02.012
- McGraw K.O., Wong S.P. (1996). Forming inferences about some intraclass correlation coefficients. *Psychol Methods* 1(1):30–46.
- Sánchez-Medina L., Pérez C.E., González-Badillo J.J. (2010). Importance of the propulsive phase in strength assessment. *Int J Sports Med* 31(2):123–129. doi:10.1055/s-0029-1242815
- Shrout P.E., Fleiss J.L. (1979). Intraclass correlations: uses in assessing rater reliability. *Psychol Bull* 86(2):420–428.
- Wannouch Y.J. et al. (2025). A systematic review using a multi-layered criteria framework for assessing the validity and reliability of velocity monitoring devices in resistance training. *PLoS One* 20(9):e0324606. doi:10.1371/journal.pone.0324606
- Weakley J. et al. (2021). The validity and reliability of commercially available resistance training monitoring devices: a systematic review. *Sports Med* 51(3):443–502. doi:10.1007/s40279-020-01382-w
