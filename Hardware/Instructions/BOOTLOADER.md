# Bootloader: install OTAFIX for over-the-air updates

The companion app installs firmware updates over Bluetooth (OTA). That transfer is run by the **bootloader**, the small program that lives at the top of the XIAO's flash, below the SoftDevice and the firmware, and that the firmware hands over to when an update starts. ODKI sensors use a community variant of the Adafruit nRF52 bootloader, **OTAFIX**, instead of the one the XIAO nRF52840 Sense ships with. This page explains why, how to check which one a board has, and how to install it — on a board you can still reach (method A) and on a finished, glued unit whose reset button is inside the case (method B).

Everything here applies to the **Seeed XIAO nRF52840 Sense**. A step marked **(to be verified)** has not yet been tested on hardware by ODKI with exactly these files and commands; follow it only if you accept that risk, and report what you see.

> **You are flashing the code that recovers every other mistake.** A bootloader update interrupted half way (cable pulled, battery flat) can leave the board needing a hardware programmer (SWD/J-Link) to recover. Use a charged battery *and* a USB cable you trust, keep the board still and plugged in until the end, and do not run anything else on that serial port while it works.

## Why it is needed

- The XIAO nRF52840 Sense ships with Seeed's build of the Adafruit nRF52 bootloader (version 0.6.2, with Nordic's SoftDevice S140 7.3.0 — the version Seeed's Arduino board package 1.1.13 carries, and the one users report in `INFO_UF2.TXT`, see below; **to be verified** on each board). Over Bluetooth it speaks Nordic's *Legacy* DFU protocol and advertises as **`AdaDFU`**.
- On ODKI's sensors with that bootloader, the app's OTA transfer stalled: the bootloader never acknowledged the data with *Packet Receipt Notifications* (the flow control a DFU client waits for), with 12 or with 1 packet per notification, and the same happened with Nordic's own nRF Connect app. (Adafruit's bootloader README asks for a PRN value of 8 or less for OTA; the stall happened below that too.)
- With **OTAFIX** the same transfer completes, from the app and from nRF Connect, and the sensor reboots into the new firmware. In update mode it advertises as **`XIAO_DFU`**. The companion app is set up for it (PRN on, 10 packets).

OTAFIX keeps the same SoftDevice (**S140 7.3.0**), so the firmware itself does not change: the same `vbt_firmware_X.Y.Z_OTA.zip` (built for SoftDevice ID `0x0123`) works on both bootloaders. Only the OTA transfer differs.

## Where to get it

| | |
|---|---|
| Project | [oltaco/Adafruit_nRF52_Bootloader_OTAFIX](https://github.com/oltaco/Adafruit_nRF52_Bootloader_OTAFIX) — a fork of [adafruit/Adafruit_nRF52_Bootloader](https://github.com/adafruit/Adafruit_nRF52_Bootloader) |
| License | MIT (Copyright (c) 2016 Adafruit Industries, see the project's `LICENSE`); the bundled SoftDevice is under Nordic Semiconductor's own license (`lib/softdevice/s140_nrf52_7.3.0/s140_nrf52_7.3.0_license-agreement.txt` there: redistribution in binary form must keep Nordic's notice, use only with Nordic chips, no reverse engineering) |
| Release used by ODKI | [`0.9.2-OTAFIX2.3-BP1.4`](https://github.com/oltaco/Adafruit_nRF52_Bootloader_OTAFIX/releases/tag/0.9.2-OTAFIX2.3-BP1.4) (19 August 2026) — `INFO_UF2.TXT` reads `UF2 Bootloader 0.9.2-OTAFIX2.3-BP1.4`, `Board-ID: nRF52840-SeeedXiaoSense-v1`, `SoftDevice: S140 7.3.0` |

Download the two files for the **Sense** board (`..._ble_sense_...`, not `..._ble_...`) from that release and check them before use (`shasum -a 256 FILE` on macOS/Linux, `certutil -hashfile FILE SHA256` on Windows). The checksums below are the ones GitHub publishes for the release assets; ODKI downloaded the files on 2 November 2026 and got the same values.

| File | Used by | SHA-256 |
|---|---|---|
| `update-xiao_nrf52840_ble_sense_bootloader-0.9.2-OTAFIX2.3-BP1.4_nosd.uf2` | method A — bootloader only | `5aa99201f3adcc128e1ebdc7ea4b4ba8e370ededda13ad94942059da5a1617c4` |
| `xiao_nrf52840_ble_sense_bootloader-0.9.2-OTAFIX2.3-BP1.4_s140_7.3.0.zip` | method B and recovery — SoftDevice + bootloader | `b64e055ac84247d95aadf977478cc4d7ac8f9730aa12e895deaf4ee17ef1e3ff` |

ODKI links to the upstream release instead of copying the binaries into this repository. Both licenses would allow a copy with the notices kept, but a link credits the maintainer, always points at files whose provenance anyone can check, and keeps third-party binaries (and their license conditions) out of this repository.

## 1. Check which bootloader a board has

**`INFO_UF2.TXT` (the reliable way).** Put the board in UF2 mode (below): a USB drive named **`XIAO-SENSE`** appears. Open `INFO_UF2.TXT` on it.

- OTAFIX: first line `UF2 Bootloader 0.9.2-OTAFIX2.3-BP1.4 …`, `Board-ID: nRF52840-SeeedXiaoSense-v1`.
- As shipped (reported by users, **to be verified** on your board): `UF2 Bootloader 0.6.2-12-g459adc9-dirty …`, `Model: Seeed XIAO nRF52840`, `Board-ID: Seeed_XIAO_nRF52840_Sense`, `SoftDevice: S140 7.3.0`.

Leave UF2 mode by pressing reset once, or by copying a firmware `.uf2` onto the drive (the board installs it and restarts).

How to get to UF2 mode:
- **Reset button reachable:** press the tiny reset button twice within half a second.
- **Closed unit:** UF2 mode cannot be reached without the button. A firmware command to open the drive without the button was tried and does not work with OTAFIX 2.3 (the board restarts straight back into the firmware), so on a closed unit check the bootloader by Bluetooth name (below) and install OTAFIX with method B.

**By Bluetooth name (no cable).** Start a firmware update from the app and watch a BLE scanner (e.g. nRF Connect): the sensor reappears as `XIAO_DFU` with OTAFIX, `AdaDFU` with the stock bootloader. Only do this when you mean to finish an update: a bootloader in OTA update mode keeps waiting for one (it advertises with no timeout) until an update completes or the board is reset.

## 2. Method A — board accessible (reset button reachable)

The method OTAFIX's own README recommends. Only the bootloader is replaced: SoftDevice and firmware stay.

1. Charge the battery, plug the board into the computer.
2. Double-press reset: the `XIAO-SENSE` drive appears. Open `INFO_UF2.TXT` and note the first and `Board-ID` lines.
3. Copy `update-xiao_nrf52840_ble_sense_bootloader-0.9.2-OTAFIX2.3-BP1.4_nosd.uf2` onto the drive. The drive disappears after a few seconds while the bootloader replaces itself; do not unplug for at least 30 seconds.
4. Double-press reset again and check `INFO_UF2.TXT`: it must now read `0.9.2-OTAFIX2.3-BP1.4` and `Board-ID: nRF52840-SeeedXiaoSense-v1`. Press reset once (or copy the firmware `.uf2`) to go back to the firmware.

If the drive refuses the file (it reappears unchanged and `INFO_UF2.TXT` still shows the old version), the running bootloader rejected the update — a UF2 bootloader update is only accepted when its USB identity matches the installed bootloader's. Do not try other boards' files; use method B instead. (**To be verified:** whether the stock Seeed bootloader accepts this file.)

## 3. Method B — closed unit, no reset button

The finished unit has its USB-C port at the end of the case, so everything below goes through the cable.

**SoftDevice + bootloader over USB serial (erases the firmware) — to be verified.** The `XIAO-SENSE` drive is not reachable without the button (a firmware command to open it was tried and does not work with OTAFIX 2.3), so a closed unit goes through the bootloader's serial update mode, which the running firmware enters on the 1200-baud touch below.

The command Adafruit's own bootloader build uses to upgrade a board over serial (`flash-dfu` target of the bootloader's `Makefile`; Seeed's Arduino package uses the same command for *Burn Bootloader* with the "Bootloader DFU for Bluefruit nRF52" programmer):

```sh
"$HOME/Library/Arduino15/packages/Seeeduino/hardware/nrf52/1.1.13/tools/adafruit-nrfutil/macos/adafruit-nrfutil" \
  --verbose dfu serial --package xiao_nrf52840_ble_sense_bootloader-0.9.2-OTAFIX2.3-BP1.4_s140_7.3.0.zip \
  -p /dev/cu.usbmodemXXXX -b 115200 --singlebank --touch 1200
```

List the ports first (`ls /dev/cu.usbmodem*` on macOS) and close any other program using the sensor's port. `--touch 1200` opens the port at 1200 baud for a moment: the running firmware (the Adafruit nRF52 core's USB stack) takes that as the signal to restart into the bootloader's serial update mode, so no button is needed. On Linux/Windows install the tool with `pip3 install --user adafruit-nrfutil` and use the port name your system shows (`/dev/ttyACM0`, `COM5`, …).

What to expect, from the bootloader's source:
- It works from the firmware currently running (any ODKI firmware, which has the core's 1200-baud touch), whatever Adafruit-based bootloader is installed: the *current* bootloader receives the package and replaces SoftDevice and bootloader. Stock and OTAFIX both use S140 7.3.0, so the SoftDevice is rewritten with the same version.
- **The firmware is erased:** in single-bank mode the package is received into the application area. When the new bootloader starts it finds no valid firmware and, since OTAFIX 2.1, waits in **Bluetooth update mode** (`XIAO_DFU`) — no USB drive, no serial port.
- Reinstall the firmware **over Bluetooth**: the companion app cannot see a sensor in this state, so use Nordic's *nRF Device Firmware Update* app (Android/iOS) with the `vbt_firmware_X.Y.Z_OTA.zip` currently in [`Firmware/firmware-release/`](../../Firmware/firmware-release/) copied to the phone, target `XIAO_DFU`, settings as recommended in OTAFIX's README (PRN on). On a board whose reset button is reachable, a double reset gives the USB drive instead.

## 4. After the bootloader update

1. Install or confirm the firmware (the app shows the firmware version in its Bluetooth menu; over serial, `VERSION`).
2. Test an OTA update from the app once: the update should run to 100 % and the sensor should come back as `VBT-Sensor` with the new version.

## 5. If something goes wrong

| Symptom | What it means | What to do |
|---|---|---|
| No USB drive and no serial port, sensor advertises `XIAO_DFU` | OTAFIX in Bluetooth update mode: no valid firmware (after method B, or an interrupted OTA) | Install the firmware OTA with nRF Device Firmware Update (method B, last step), or double reset if reachable |
| The `.uf2` copy fails or the drive reappears unchanged | The bootloader refused the file (wrong board file, or identity mismatch) | Check you used the `ble_sense` file; otherwise use method B |
| `adafruit-nrfutil` stops with a timeout or "No ACK" | Wrong port, another program holding it, or the port name changed after the 1200-baud restart | Close other serial programs, list the ports again, re-run with the new name (and without `--touch 1200` if the board is already in serial update mode) |
| Board shows no sign of life after a bootloader update | Bootloader damaged (interrupted while it was being replaced) | Needs SWD: an nRF52 programmer (J-Link, or a CMSIS-DAP probe) on the XIAO's SWD pads, then flash `xiao_nrf52840_ble_sense_bootloader-0.9.2-OTAFIX2.3-BP1.4_s140_7.3.0.hex` from the same release; on a glued unit this means opening the case |
| The app never offers an update | The phone or tablet is offline (the app checks GitHub each time it connects), or the sensor already runs the latest version | Connect the phone to the internet, then disconnect and reconnect the sensor in the app |
| An OTA update from the app stalls on a board you converted | Still the old bootloader, or a phone BLE quirk | Check `INFO_UF2.TXT`; retry the update (OTAFIX's README lists DFU settings that help on some Android phones) |

## 6. Risks, in short

- Do not unplug the cable or let the battery run flat while a bootloader is being written. Method A replaces only the bootloader (a few seconds); method B rewrites SoftDevice and bootloader (about 20–30 seconds).
- Use only the **Sense** files of the release named above, checked against the SHA-256 values.
- A board stuck in Bluetooth update mode keeps advertising until it receives an update; on a closed unit the battery cannot be disconnected, so finish the update rather than leaving it.
- These are third-party binaries, provided by their authors "as is"; ODKI does not modify or support them beyond this guide.
