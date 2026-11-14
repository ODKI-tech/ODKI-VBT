# ODKI-VBT validation protocol (pre-registered)

**Status: not yet validated.** This protocol, the analysis code and the acceptance thresholds are published **before any study data is collected**. The git commit that adds this version is its timestamp. Every later change is listed in section 14 with its date and reason. The results will be published whatever they show.

| | |
|---|---|
| Version | 2.0, 14/11/2026. Replaces version 1.0 before any study data was collected (section 14) |
| Device | ODKI-VBT sensor (Seeed XIAO nRF52840 Sense, LSM6DS3TR-C IMU) |
| Firmware under test | **3.12.19**, default settings. The exact commit is recorded for every session |
| Reference | the built-in cable position encoder of a commercial cable resistance machine (brand and model not disclosed), its cables attached to the ends of a free barbell, verified as in section 4. It is **not** a laboratory instrument |
| Study type | in-house engineering study by ODKI. Not independent, and not a laboratory study |
| Code | this folder: `capture.py`, `import_ref.py`, `analyze.py` (and `pipeline/`) |
| Licences | dataset **CC BY 4.0**; code Apache License 2.0 (`LICENSE-APACHE`) |

## 1. Questions

1. **Agreement (primary).** How closely do the per-rep values reported by the sensor agree with a cable encoder on the same bar? Mean concentric velocity (primary), peak velocity, range of motion, phase timing and rep detection, for the back squat and the bench press, from 0.5 to above 1.0 m/s.
2. **Repeatability.** How repeatable are the readings between two days?
3. **Lifter experience.** Does agreement hold for a beginner as well as for trained lifters?

## 2. Design

| Item | Plan |
|---|---|
| Phases | **Pilot:** the operator only; reference verification (section 4) and a dry run of the analysis. Not published. **Familiarisation:** the beginner learns both lifts with a coach; nothing recorded. **Study:** below |
| Lifters | **4** adult volunteers, unpaid: **3 trained** (free barbell ≥ 1 year) and **1 beginner** (< 6 months). One of them is the operator (2.1). Each completes a non-medical self-screening and signs a written consent first |
| Exercises | **back squat** and **flat bench press**, free barbell and plates, in a rack with safety bars. Both bar ends are attached to the machine's cables; the machine only keeps them taut with a constant 3 kg per side (6 kg, counted in the load). **Deadlift** comes later as an amendment |
| Loads | by **velocity band**, so no maximal test is needed. The band is chosen from the reference MV of a warm-up rep: **V1** ≥ 1.0 m/s, **V2** 0.75–1.0, **V3** 0.5–0.75, each band including its lower bound (`VELOCITY_BANDS` in `pipeline/common.py`). Below 0.5 m/s (roughly above 80% 1RM) is **not tested**. Loads only go up within a session |
| Primary sets | per lifter, exercise and day: V1 2 × 4 reps, V2 2 × 4, V3 3 × 3. Maximal intended concentric speed, controlled eccentric, the usual short pause at lockout, 2–3 min rest, no set to failure |
| Exploratory sets | day 2 only, at V2, reported apart and **never** counted in the tiers: **E1** touch-and-go, 6 reps; **E2** 2 s pause at the bottom, 4 reps; **E3** 3 s eccentric, 4 reps; **E4** one deliberate half rep and one bar adjustment without a rep; **E5** cable-free set over Bluetooth with the app (rep count and final values only) |
| Days | 2 sessions per lifter, 48–72 h apart, same time of day, warm-up and order |
| Unit and placement | 1 sensor for the whole study, on the bar end next to the hook of the reference cable (≤ 5 cm). The difference between the two cables is recorded as bar tilt |
| Expected size | 4 lifters × 25 reps × 2 days ≈ **200 reps per exercise**, about **400** in total; exploratory about 80 per exercise |
| Safety | safety bars set for every lift; a spotter for every bench set and for squat at V3; the machine's stop within reach; immediate stop on pain, dizziness, failing technique or the lifter's request |

**2.1 Operator also lifting.** When the operator lifts, the capture starts after a fixed delay and another participant spots. During primary sets no lifter sees velocity feedback, except the warm-up reading used to choose the band. Declared in the results.

**2.2 Sample size.** The 95% CI half-width of a Bland–Altman limit of agreement is about 3.39·s/√n (s = SD of the differences): about 0.24·s with 200 pairs per exercise, 0.17·s with 400 pooled. With four lifters the between-lifter uncertainty is poorly estimated, so results are also given per lifter and nothing is generalised to a population; one beginner shows whether a beginner's technique breaks agreement, not how beginners behave in general. Test–retest has 12 lifter × band units per exercise and is **descriptive only**.

## 3. What is measured

| Source | Content | Rate |
|---|---|---|
| ODKI USB serial log (`S,` `R,` `B,` `P,` rows, `Firmware/DOCUMENTATION.md`, "Lab data capture") | per sample: velocities, drift-correction brackets, accelerations; per rep: every reported value, first and final; per phase: start, end and close reason | 100 Hz |
| Reference | position of the reference cable (and of the second cable, for tilt), converted to `t_s, position_m` | 50 Hz (measured in the pilot) |

No video is recorded.

## 4. Reference instrument

The reference is a position transducer on the cable of a commercial machine, not a laboratory instrument, and its internal processing is not documented. Its **raw position** is the reference signal; the machine's own velocity output is used only as a check, because it is filtered. The reference velocity is position change over time times the scale (metres per count), so a scale error of x% gives x% on every velocity. The reference is used only after these checks:

1. **Static scale:** 10 cable pulls of 1000 mm against a rigid rule, held still at both ends. CV < 0.3%. The mean rule length / reading is the scale factor applied to the reference; a difference from nominal above 1% is reported.
2. **Machine velocity:** derivative of the scaled position vs the machine's own velocity over the pilot sets; a slope more than 2% from 1 is reported.
3. **Sampling:** rate and resolution recorded; a set with a gap above two sample intervals is flagged.
4. **Two cables:** the two cables against each other over the pilot sets (bias and LoA of MV), to size the tilt error.

Free-fall and video checks are not possible here (the machine resists the cable; no 240 fps camera) and are declared as not done. At 50 Hz and 1 mm resolution a single velocity sample is quantised to 0.05 m/s, but a concentric mean over 0.4–1.5 s is limited to about 1 mm / duration, below 0.003 m/s. Peak velocity is the most affected by the 50 Hz rate.

## 5. Capture, alignment and pairing

**5.1 Capture.** For each set: `CALIBRATE` with the bar still (at the start of each session), start both recordings, **sync event** (with the bar still for 2 s, lift it sharply by a few cm and set it back; wait 2 s), the set, 2 s still, stop. `capture.py` saves the ODKI log byte for byte. The reference is recorded by its own reader and saved as `raw/ref/<set>.csv` (`t_s, position_m`); `import_ref.py` converts other encoders' exports to the same file. The machine reader itself is not published, because it would identify the machine.

**5.2 Alignment** (`pipeline/align.py`), from the signals alone: coarse full-range cross-correlation of the ODKI drift-corrected velocity with the reference velocity on a 100 Hz grid; fine search ±300 ms in 1 ms steps with parabolic refinement; clock drift on sets ≥ 25 s. A set is **flagged and excluded** if the sync event is not found, if it differs between the ODKI acceleration and the reference by more than **±20 ms** (one reference sample), if the velocity correlation is below 0.90, or if the reference has a gap (section 4, check 3). The number of flagged sets is reported.

**5.3 Pairing** (`pipeline/pair_reps.py`). Reference rep: a run of reference velocity > 0 rising ≥ 10 cm (runs < 50 ms apart merged). ODKI rep: every rep with an `R,` row, its interval from the `P,` row. One-to-one pairing on time overlap (IoU ≥ 0.5): paired = TP, ODKI only = false rep (FP), reference only = missed rep (FN).

## 6. Endpoints

| Endpoint | ODKI | Reference |
|---|---|---|
| **Primary: mean concentric velocity (MV)**, final value | `meanVelocity` of the last `R,` row | mean over the **same samples** the firmware averages: those of the ODKI phase window (`P,` row start and end) that its signal puts in the phase's direction (`mv_same_window`). Isolates measurement error from definition error |
| MV, standard definition | same | mean of all samples with v > 0 over the reference's own rep: definition and measurement error together |
| MV, first reported value | first `R,` row (what the athlete sees first) | as the primary endpoint |
| Peak velocity | `peakVelocity`, first and final | max v (50 Hz caveat) |
| Range of motion | `displacementM` | integral of v over the concentric |
| Phase timing | `P,` row start and end | start and end of the reference rep |
| Mean propulsive velocity (descriptive) | offline, from the drift-corrected curve | Sánchez-Medina et al. 2010; descriptive only (acceleration from a 50 Hz position is noisy) |
| Rep detection | TP / FN / FP | — |

**Known definition difference at lockout.** The firmware ends a concentric when its stillness test fires. A very slow finish of the lockout (a few mm at about 0.03–0.05 m/s) can fall inside the stillness band: the firmware leaves it out, the standard definition keeps it and gets a lower mean. A second small push at lockout is the opposite case. Both affect only the standard-definition endpoint, not the primary one, and are reported per rep with the phase timing.

## 7. Statistics (`pipeline/stats.py`, deterministic)

- **Agreement**, per endpoint and value (first, final): bias and 95% limits of agreement (plain and for repeated measures, Bland & Altman 2007); proportional bias; ICC(2,1) absolute agreement; SEM and MDC95; CV% (Hopkins, log-transformed); MAE, RMSE, Pearson r, SEE (standard error of estimate of the reference from ODKI) and standardised bias.
- **Rep detection:** recall, precision, F1 and false-rep rate with Wilson 95% CIs.
- **Test–retest** (lifter × exercise × planned band, mean of the day): ICC(3,1), SEM, MDC95, on ODKI and on the reference. Descriptive.
- **Strata:** exercise, measured velocity band, lifter, experience, final correction status, phase close reason; exploratory sets apart.
- **`quality1`:** Spearman ρ with |MV error| and ROC AUC for |error| > 0.10 m/s. Only a useful relation allows calling it more than a "signal-consistency score".
- **CIs:** cluster bootstrap over **sets** (2000 resamples, fixed seed); with four lifters a bootstrap over lifters is not meaningful.
- **Exclusions:** only flagged sets (5.2, 4) and sets with a documented equipment failure, logged before the analysis is run. No outlier removal.

The code is checked on synthetic data with known errors (`tests/test_pipeline.py`).

## 8. Acceptance thresholds (fixed before data)

Evaluated on the **primary endpoint** (MV, final, squat and bench pooled), also reported per exercise, band and lifter. Tiers E and A use the **same six statistics**, so the result can be placed against both classes of device. All criteria of a tier must hold; the highest tier reached is reported. The values are coded in `pipeline/common.py` (`THRESHOLDS`).

| Tier | r | \|bias\| | LoA half-width | SEE | CV | ICC(2,1) |
|---|---|---|---|---|---|---|
| **E: comparable to high-end linear encoders** | ≥ 0.99 | ≤ 0.03 m/s | ≤ 0.06 m/s | ≤ 0.03 m/s | ≤ 5% | ≥ 0.98 |
| **A: comparable to high-end IMU sensors** | ≥ 0.95 | ≤ 0.05 m/s | ≤ 0.12 m/s | ≤ 0.06 m/s | < 10% | ≥ 0.90 |
| **B: minimum acceptable in the literature** | > 0.70 | — | — | — | < 10% | — (effect size < 0.60) |
| Internal target (not from a published source) | — | ≤ 0.03 m/s | ≤ 0.10 m/s | — | ≤ 5% | ≥ 0.90 |
| Rep detection (internal target) | recall ≥ 98%, false-rep rate ≤ 2% | | | | | |

Sources, checked on the full texts on 14/11/2026:
- **E:** GymAware vs 3D motion capture, back squat: R² 0.99, slope 0.991, no fixed or proportional bias (Thompson et al. 2020, Table 3). As tabulated in Weakley et al. 2021: GymAware vs Vicon SEE 0.024–0.030 m/s; GymAware vs laboratory transducers CV 2.5–3.6%; T-Force vs 3D motion capture bias −0.01 ± 0.03 m/s (LoA about ±0.06); Tendo vs T-Force ICC 0.985–0.989. Here the reference is itself an encoder with its own error, which makes tier E harder to reach, not easier.
- **A:** Enode Pro on the barbell vs 3D motion capture, back squat: r 0.966, SEE 0.059 m/s (Achermann et al. 2023, Table 2). Vmaxpro vs a linear transducer, squat: bias 0.01 ± 0.04 m/s, LoA 0.10 m/s, r 0.96, ICC 0.94, CV 7.6% (Held et al. 2021, Table 1). ICC > 0.90 "excellent" (Koo & Li 2016).
- **B:** Weakley et al. 2021 (r > 0.70, CV < 10%, effect size < 0.60).

Peak velocity, the first reported value and the exploratory sets are reported but do not count towards the tiers. **Whatever tier is reached, or none, is published with the full data. Below tier B no accuracy claim is made.**

## 9. Dataset (CSV, UTF-8, SI units)

Published on Zenodo with a DOI under CC BY 4.0.

| File | Columns |
|---|---|
| `lifters.csv` | `lifter_id` (random `L-xx`), `experience` (beginner / trained). Nothing else: with four lifters, age, sex, height or body mass would identify them |
| `sessions.csv` | `session_id, lifter_id, day_index, exercise, firmware_version, firmware_commit, runtime_config_hex, reference_type, reference_id, reference_rate_hz, operator, notes`. No dates or times |
| `units.csv` | `session_id, unit_id, placement, distance_to_ref_mm, battery_pct_start` |
| `reference_verification.csv` | `reference_id, test (static/machine_velocity/sampling/two_cables), trial, expected, measured, unit` |
| `sets.csv` | `set_id (<physical set>_<unit>), physical_set_id, session_id, unit_id, velocity_band (planned V1–V3), set_type (primary / E1–E5), planned_reps, tilt_p95_mm`. No loads |
| `raw/odki/<set_id>.log` | the ODKI serial log, unmodified (device uptime only) |
| `raw/ref/<physical_set_id>.csv` | `t_s, position_m`, before the scale factor |
| generated by `analyze.py` | `aligned/`, `sets_alignment.csv`, `reps_paired.csv` (columns in `pipeline/pair_reps.py`), `results/` |

## 10. Consent and data protection

- Adults only. Each participant gets the information sheet and privacy notice, completes a non-medical self-screening and signs a written consent. No health information is recorded.
- Participation can be refused or withdrawn without explanation until publication; published anonymous data cannot be withdrawn afterwards, and the consent form says so. Participants who know the operator are told this explicitly.
- With four lifters even anonymous data can be linked to a person by people who know them: each participant is told before consenting and sees `lifters.csv` and `sets.csv` before publication.
- Random `L-xx` IDs; the ID-to-name table and the signed forms stay offline and the table is destroyed at the end. No dates or times of day anywhere; logs are checked for device names, host names and absolute timestamps before publication.

## 11. Claims

**Before results:** "Not yet validated. Validation is in progress: protocol published [link]."

**After results**, only with tier B or better and only with the context in the same block:
- "Validated against a cable encoder: mean velocity bias X m/s, 95% limits of agreement L1 to L2 m/s, ICC Y [tier name, e.g. comparable to high-end linear encoders]. 4 lifters (1 beginner), N reps, back squat and bench press with a free barbell, 0.5–1.2 m/s. Firmware x.y.z. In-house study by ODKI; the reference is not a laboratory instrument. Protocol and raw data: [DOI]."
- Never "validated" alone, "lab-grade", "research-grade", "gold standard" or "independent".
- Results apply only to the exercises, velocity range and firmware tested; below 0.5 m/s, the deadlift and other exercises are "not tested".
- No comparison with a named commercial device, because none was measured in this study.
- Anyone can repeat the study with their own reference using this protocol and code, and publish their results.

## 12. Reproducing the analysis

```
python analyze.py --data DATA      # -> DATA/results/RESULTS.md, summary.json, plots
```

The bootstrap seed is fixed, so the published dataset always gives the published numbers.

## 13. Later phases

- **Deadlift:** an amendment, published before its data is collected.
- **Phase end at lockout:** after the study, its slow-lockout reps are used to evaluate ending the concentric at the highest bar position; any change is a new firmware version, outside this study.
- **Laboratory phase:** a university laboratory with a laboratory transducer or 3D motion capture, under its ethics approval.
- **Every firmware release:** the dataset logs are replayed through `host_replay` against the new engine.

## 14. Deviations and history

- **Version 2.0, 14/11/2026, before any study data.** Replaces version 1.0 (stand-alone linear encoder, six trained lifters, loads in % 1RM, firmware 3.12.4). Reasons: the reference available is a machine's cable encoder; four lifters are available, one a beginner on purpose. Loads by velocity band up to 0.5 m/s; exploratory sets added; tier E added next to the IMU tier; one sensor; no video; firmware 3.12.19.
- **Primary endpoint reference.** Version 1.0 approximated the firmware's phase definition on the reference (first sample ≥ 0.20 m/s plus a short back-fill). On real lifts that cut the slow start of the push that the firmware keeps, so it measured a definition artefact; the reference is now averaged over the same samples as the firmware (section 6).
- **Data seen before this version.** On 13/11/2026 the operator recorded two exploratory squat sets (13 reps, firmware 3.12.18) on the same reference to test the set-up. Their analysis led to firmware 3.12.19 (phase look-back 5 → 12 samples) and to the endpoint change above. They are not part of the study.

Every change after the first study session is listed here with its date and reason.

## 15. Open items before the pilot

| Item | If not settled |
|---|---|
| The beginner (fourth lifter) | the study runs with 3 trained lifters; question 3 is dropped and this is declared |
| Session dates | — |

## References

- Achermann B., Oberhofer K., Ferguson S.J., Lorenzetti S.R. (2023). Velocity-based strength training: the validity and personal monitoring of barbell velocity with the Apple Watch. *Sports* 11(7):125. doi:10.3390/sports11070125
- Bland J.M., Altman D.G. (2007). Agreement between methods of measurement with multiple observations per individual. *J Biopharm Stat* 17(4):571–582. doi:10.1080/10543400701329422
- Held S., Rappelt L., Deutsch J.-P., Donath L. (2021). Valid and reliable barbell velocity estimation using an inertial measurement unit. *Int J Environ Res Public Health* 18(17):9170. doi:10.3390/ijerph18179170
- Koo T.K., Li M.Y. (2016). A guideline of selecting and reporting intraclass correlation coefficients for reliability research. *J Chiropr Med* 15(2):155–163. doi:10.1016/j.jcm.2016.02.012
- Sánchez-Medina L., Pérez C.E., González-Badillo J.J. (2010). Importance of the propulsive phase in strength assessment. *Int J Sports Med* 31(2):123–129. doi:10.1055/s-0029-1242815
- Shrout P.E., Fleiss J.L. (1979). Intraclass correlations: uses in assessing rater reliability. *Psychol Bull* 86(2):420–428.
- Thompson S.W., Rogerson D., Dorrell H.F., Ruddock A., Barnes A. (2020). The reliability and validity of current technologies for measuring barbell velocity in the free-weight back squat and power clean. *Sports* 8(7):94. doi:10.3390/sports8070094
- Weakley J. et al. (2021). The validity and reliability of commercially available resistance training monitoring devices: a systematic review. *Sports Med* 51(3):443–502. doi:10.1007/s40279-020-01382-w
