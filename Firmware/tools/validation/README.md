# Validation

How we test the accuracy of ODKI-VBT against a reference encoder, and the tools to repeat the test yourself.

- **[PROTOCOL.md](PROTOCOL.md)**: the study (design, endpoints, statistics, pass thresholds), published before any data.
- **No results yet.** They will be published here with the raw data, whatever they show.

## What you need

- An ODKI-VBT sensor connected to a computer by USB.
- A reference that records bar **position over time** (a linear encoder or a cable encoder) on the same end of the bar.
- Python 3 with `pip install numpy pandas matplotlib pyserial`.

## Three steps

| Step | Command | Output |
|---|---|---|
| 1. Record a set with the sensor | `python capture.py --odki U1=/dev/cu.usbmodem1101 --data DATA --set L-01-D1-SQ-V2-S1 --start` | `DATA/raw/odki/<set>_U1.log` |
| 2. Convert the reference's export | `python import_ref.py export.csv --data DATA --set L-01-D1-SQ-V2-S1 --time-col 0 --time-unit ms --pos-col 1 --pos-unit mm` | `DATA/raw/ref/<set>.csv` |
| 3. Analyse everything | `python analyze.py --data DATA` | `DATA/results/RESULTS.md` |

Before step 1, calibrate the sensor with the bar still and start the reference recording. During the set: 2 s still, a short sharp lift as sync, the reps, 2 s still, then Ctrl+C. The dataset also needs `sessions.csv`, `sets.csv` and `reference_verification.csv`, described in PROTOCOL.md section 9. Run each script with `--help` for all options.

`analyze.py` aligns each set from the signals (no shared clock needed), pairs the reps and computes agreement, rep detection and the thresholds of PROTOCOL.md section 8. The same data always gives the same numbers.

## Files

| | |
|---|---|
| `capture.py`, `import_ref.py`, `analyze.py` | the three commands above |
| `pipeline/` | the analysis code called by `analyze.py`: alignment, pairing, statistics, and the pre-registered constants and thresholds (`common.py`) |
| `tests/` | self-test on synthetic data with known errors: `python tests/test_pipeline.py`. It checks the code, not the sensor |

Logs from firmware 3.11.32 or later are needed for the primary endpoint (it uses the phase start and end times in the `P,` rows).
