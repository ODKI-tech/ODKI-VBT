# Bill of Materials

Full parts list for one ODKI VBT unit: see [`bom.csv`](bom.csv) for the structured table (description, quantity per unit, key specs, approximate unit cost). This page adds the technical detail that doesn't fit comfortably in a spreadsheet cell.

Where to buy each part is up to you — specs below are what to match, not a specific listing to use. Magnets and the power switch are typically sold in bulk packs (e.g. 50+ magnets, 100+ switches); only a few units per pack are needed per sensor, so the realistic per-unit cost is a small fraction of the pack price. Prices in `bom.csv` are rough, observed ranges, not a quote — shop around, especially for the microcontroller, where equivalent listings for the same board have been seen varying quite a bit in price between sellers.

## LiPo battery (1 per unit)

3.7V nominal, 400 mAh (1.48 Wh) capacity, Li-Po size code **602035** (6mm thick x 20mm wide x 35mm long). CE certified. **Make sure whatever cell you use includes a built-in protection chip** (over-charge, over-discharge, and short-circuit protection) — the reference cell does, and the two leads are soldered directly to the XIAO board's `B+`/`B-` pads (see [`Hardware/Instructions`](../Instructions/)) with no separate protection circuit in between, so an unprotected bare cell would have nothing else guarding it. This is the cell referenced throughout `Firmware/DOCUMENTATION.md` (BatteryMonitor chapter) for voltage reading, charge percentage, and auto-calibration — any similarly-sized single-cell 3.7V protected LiPo should work, but the discharge curve assumed by `BatteryMonitor::voltageToPercent()` (3.30V = 0%, 3.60V = 10%, 4.20V = 100%) is tuned for a typical LiPo chemistry and may need adjusting for a very different cell.

## Microcontroller (1 per unit)

**Seeed Studio XIAO nRF52840 Sense** — the reference design's MCU + IMU + BLE module, all-in-one. Official specifications (Seeed Studio wiki, more reliable than any individual listing's page):

- Nordic nRF52840, ARM Cortex-M4 32-bit processor with FPU, 64 MHz
- Bluetooth 5.0/5.4, BLE Mesh, NFC
- 256 KB RAM, 1 MB internal flash + 2 MB onboard flash
- Onboard 6-axis IMU: **LSM6DS3TR-C** (accelerometer + gyroscope) — this exact part is what `MotionTracker.cpp` talks to over I2C; a substitute board would need firmware changes, not just a BOM swap
- Onboard PDM digital microphone (unused by this project's firmware, but present on the "Sense" variant)
- Integrated LiPo charge/discharge management (the BQ25100 chip `BatteryMonitor.cpp` reads charge status from)
- USB-C connector, single-sided surface-mount components
- Standby power consumption < 5µA
- Dimensions: **21mm x 17.8mm** (the classic XIAO form factor) — note that some listings display shipping/package dimensions instead of the board's own; 21 x 17.8mm is the board itself, confirmed against Seeed's own wiki.

## Neodymium magnets (3 per unit)

Round disc, **10mm diameter x 2.2mm thick** (±0.1mm tolerance), neodymium, ~2kg pull force per magnet. Used for the barbell-mount attachment — press-fit (plus a drop of glue) into 3 blind pockets molded into [`case-bottom`](../CAD/) — rather than a mechanical clip.

## Power switch (1 per unit)

**SPDT slide switch** (1-pole 2-throw, 3-pin, 2-position), panel-mount, vertical slide actuation, DC 50V / 0.5A rated, model SS12D00-G3. Body without pins/knob: 8 x 4 x 3.8mm; overall with knob: 8 x 4 x 10mm; pin spacing 2.54mm (breadboard-standard). This is the physical power switch `Firmware/DOCUMENTATION.md`'s PowerManager chapter describes as wired to `WAKE_PIN` (`Config.h`, `D6`) and GND, active-low — any SPDT switch of a similar size fits the same wiring, this is just the specific one used in the reference build.

## Hookup wire (2 per unit)

Generic stranded hookup wire, ~26-30AWG, roughly 5-10cm per length — connects the power switch's two relevant pins to `WAKE_PIN` and GND. No specific product; any reel of thin stranded wire works, these are typically leftover offcuts rather than a dedicated purchase.
