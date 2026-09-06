# Hardware

This folder holds everything needed to build the ODKI VBT sensor's physical hardware. The current reference design is built around a **Seeed XIAO nRF52840 Sense** module (which already integrates the nRF52840 MCU, an LSM6DS3TR-C accelerometer/gyroscope, BLE radio, and USB-C/battery charging), plus a LiPo battery and a physical power switch — see `Firmware/DOCUMENTATION.md` for how the firmware expects that switch to be wired (`WAKE_PIN`, `Config.h`).

- **[BOM/](BOM/)** — bill of materials: every part needed to build one unit, with quantities and key specs (no sourcing links — where to buy each part is up to you, see the note in [BOM/README.md](BOM/README.md)).
- **[CAD/](CAD/)** — the two-body enclosure design (source + print-ready files) and, if a custom PCB is ever designed instead of using the XIAO module directly, its schematic/layout sources.
- **[Instructions/](Instructions/)** — step-by-step assembly instructions, from raw parts to a working, closed unit ready to flash.

See [LICENSE-HARDWARE](../LICENSE-HARDWARE) at the project root for the license status of everything in this folder.
