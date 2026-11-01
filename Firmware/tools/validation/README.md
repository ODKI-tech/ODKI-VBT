# Validation pipeline

These are the tools for the pre-registered validation study described in [`PROTOCOL.md`](PROTOCOL.md). The study compares the ODKI-VBT sensor's per-rep values with a verified linear encoder, recorded by its own software. **No validation data exists yet**: the protocol and this code are published first.

Requirements: Python 3, `numpy`, `pandas` and `matplotlib` (`pip install numpy pandas matplotlib`). `capture.py` also needs `pyserial`. The ODKI log parser is reused from `../vbt_log_viewer.py` and imported, never copied.

| Script | Does |
|---|---|
| `capture.py` | Records one set: every ODKI unit's serial log (byte for byte), with relative arrival times |
| `import_ref.py` | Converts the linear encoder's export (position or velocity vs time, any delimiter and decimal separator) to `raw/ref/<set>.csv` (`t_s,position_m`) |
| `align.py` | Cross-correlation alignment (no shared clock), clock drift, sync-drop check → `aligned/` |
| `pair_reps.py` | Segments reference and ODKI reps, pairs them by IoU → `reps_paired.csv`, `sets_alignment.csv` |
| `analyze.py` | Bland–Altman (plain and repeated measures), ICC(2,1)/(3,1), SEM, CV%, MAE, r, rep-detection confusion, strata by exercise and velocity band, thresholds → `results/` |
| `common.py` | Shared parsing, metric definitions and the **pre-registered constants and thresholds** |
| `tests/` | Synthetic self-test (`synth.py` generates data, `test_pipeline.py` checks recovery) |

## Usage

### 1. Record a set

Set up the dataset folder with `sessions.csv`, `sets.csv` and `reference_verification.csv` (columns in PROTOCOL.md section 9). Then, for every set:

```
python capture.py --odki U1=/dev/cu.usbmodem1101 --odki U2=/dev/cu.usbmodem1201 \
    --data DATA --set L-07-D1-SQ-060-S1 --start
#   calibrate first (CALIBRATE with the bar still, e.g. from vbt_live_monitor.py);
#   start the encoder recording in its own software, then this command;
#   after START: sync drop, the set, 2 s still, then Ctrl+C (sends STOP); stop the encoder recording
```

This writes `DATA/raw/odki/L-07-D1-SQ-060-S1_U1.log` (+ `.sidecar.csv`) and `..._U2.log`. The set ids in `sets.csv` are `<physical set>_<unit>`. Then convert the encoder's export for the same physical set:

```
python import_ref.py export.csv --data DATA --set L-07-D1-SQ-060-S1 \
    --time-col 0 --time-unit ms --pos-col 1 --pos-unit mm --zero-time
#   velocity-only export: --vel-col <col> (m/s, integrated with trapezoids)
#   e.g. semicolon-separated with decimal commas: --delimiter ";" --decimal ","
```

This writes `DATA/raw/ref/L-07-D1-SQ-060-S1.csv` (`t_s,position_m`, on the encoder's own clock). The scale factor from the static verification is applied later by `align.py`, not here.

### 2. Align, pair and analyse

```
python align.py --data DATA --all          # or --set-id <id>; --ref-scale to override the static verification
python pair_reps.py --data DATA
python analyze.py --data DATA              # -> DATA/results/summary.json, RESULTS.md, mv_<exercise>.png
```

`align.py` finds the offset between the two clocks by full-range cross-correlation, then refines it and the drift; it prints lag, drift, correlation and the sync check for each set. Sets marked `FLAGGED` are excluded from the agreement statistics, as pre-registered.

### 3. Self-test (synthetic data)

```
python tests/test_pipeline.py              # about 30 s; --keep DIR keeps the generated dataset
```

The test generates a full synthetic dataset: 3 lifters, 2 days, squat and bench at 4 loads, 2 units, 128 reference reps per unit. It contains a known MV bias (0.030 m/s) and noise (SD 0.040 m/s), a 15 ms sensor delay, a reference recording started a few seconds early on its own clock with a 40 ppm drift, a 1.5% reference scale error (corrected by the synthetic static verification), a reversed sign, 3 missed reps and 2 false reps. It also converts a velocity-only export with `;` fields and decimal commas through `import_ref.py`. It runs the three scripts through their command lines and checks that all of this comes back out. It also checks the ICC code against the Shrout & Fleiss (1979) worked example. **Synthetic data only tests the code. It says nothing about the sensor's accuracy.**

## Notes

- Logs from every firmware since 3.11.18 work. Firmware 3.11.32 adds per-phase start and end times to the `P,` rows; when they are present, `pair_reps.py` uses them instead of rebuilding the rep interval from the `S,` rows.
- Every number in `results/` is deterministic (the bootstrap seed is fixed), so re-running the scripts on the published dataset must reproduce the published results exactly.
