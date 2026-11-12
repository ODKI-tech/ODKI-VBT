# Install the bootloader and the firmware

This guide puts the software on a Seeed XIAO nRF52840 Sense. It works the same way on a bare board and on a finished, closed sensor, and it doesn't need the reset button. It takes about 10 minutes.

You will:
1. replace the XIAO's bootloader with **OTAFIX**, over the USB cable;
2. install the ODKI firmware from the companion app, over Bluetooth.

ODKI ran these steps on 12 November 2026 on a factory-new XIAO nRF52840 Sense, on macOS, with the files and commands below. Windows and Linux use the same tools, but ODKI has not tested them there.

## Why OTAFIX

The bootloader is the small program that installs firmware on the board. The app's Bluetooth updates did not work with the bootloader the XIAO ships with: the transfer stalled, and it stalled in Nordic's own nRF Connect app too. They do work with **OTAFIX**, a community fix of the same Adafruit bootloader, which keeps the same Bluetooth stack (SoftDevice S140 7.3.0). After this guide, every firmware update comes from the app, with no cable needed.

## Before you start: safety

- **Write the bootloader without the battery, or with a charged one.** Don't do it while the battery is running flat.
- **Don't unplug the cable while step 3 runs (about 20 seconds).** A bootloader write that stops half-way can leave a board that only a hardware programmer can recover (see [If something goes wrong](#if-something-goes-wrong)).
- **Use only the file named in step 1, and check its fingerprint.** A file built for another board can damage this one.
- **On a closed sensor:** charge it first, and set it down where it won't move or get unplugged.

## What you need

- A computer with a USB port, and a USB-C **data** cable (some cables only charge).
- Python 3 (built into macOS and Linux; on Windows, from [python.org](https://www.python.org/downloads/)).
- The ODKI companion app on a phone or tablet that is online (see [`../../Apps/`](../../Apps/)).

## 1. Download the bootloader file

From the OTAFIX release [`0.9.2-OTAFIX2.3-BP1.4`](https://github.com/oltaco/Adafruit_nRF52_Bootloader_OTAFIX/releases/tag/0.9.2-OTAFIX2.3-BP1.4), download this one file. It is the one for the **Sense** board (`ble_sense`, not `ble`):

`xiao_nrf52840_ble_sense_bootloader-0.9.2-OTAFIX2.3-BP1.4_s140_7.3.0.zip`

Check its fingerprint in a terminal, from the folder you saved it to:

```sh
shasum -a 256 xiao_nrf52840_ble_sense_bootloader-0.9.2-OTAFIX2.3-BP1.4_s140_7.3.0.zip
```

(On Windows: `certutil -hashfile <file> SHA256`.) It must print exactly:

`b64e055ac84247d95aadf977478cc4d7ac8f9730aa12e895deaf4ee17ef1e3ff`

If it prints anything else, delete the file and download it again. Don't continue with it.

## 2. Install the update tool and find the port

1. Install Adafruit's update tool (it brings `pyserial` with it):

   ```sh
   pip3 install --user adafruit-nrfutil
   ```

   If you already have the Arduino IDE with Seeed's nRF52 board package, the same tool is already there. On macOS it is at `~/Library/Arduino15/packages/Seeeduino/hardware/nrf52/<version>/tools/adafruit-nrfutil/macos/adafruit-nrfutil`. ODKI used that copy.

2. Plug the board into the computer and list the serial ports:
   - macOS: `ls /dev/cu.usbmodem*`
   - Linux: `ls /dev/ttyACM*`
   - Windows: Device Manager → Ports (COM & LPT)

   Close any other program that might use the port (Arduino IDE, serial monitor, `vbt_live_monitor.py`). Below, `PORT` stands for the name you found, e.g. `/dev/cu.usbmodem101`.

## 3. Write the bootloader

1. **Put the board in update mode.** This one-line command opens the port at 1200 baud and closes it, which is the standard signal for the board to restart into its bootloader:

   ```sh
   python3 -c "import serial,time; s=serial.Serial(); s.port='PORT'; s.baudrate=1200; s.dtr=True; s.open(); time.sleep(0.5); s.dtr=False; time.sleep(0.5); s.close()"
   ```

   Wait 5 seconds, then list the ports again. The port normally comes back with the same name; the board is now in its bootloader. With no port at all, see [If something goes wrong](#if-something-goes-wrong).

2. **Write OTAFIX** (about 20 seconds; don't touch the cable):

   ```sh
   adafruit-nrfutil --verbose dfu serial \
     --package xiao_nrf52840_ble_sense_bootloader-0.9.2-OTAFIX2.3-BP1.4_s140_7.3.0.zip \
     -p PORT -b 115200 --singlebank
   ```

   It ends with `Device programmed.`

3. **The board disappears from USB. This is normal.** This step erases the firmware, and OTAFIX with no firmware waits for one over Bluetooth, as **`XIAO_DFU`**. Leave the board plugged in: the cable now only powers it.

## 4. Install the ODKI firmware from the app

1. Keep the phone next to the board, online, with the app open.
2. Bluetooth icon → **SCAN FOR DEVICES**. At the top of the list, an orange card says **Sensor in update mode**.
3. Tap **FINISH UPDATE** and keep the screen open until it reaches 100 %. The app downloads the latest firmware from this repository by itself.

If the card doesn't appear, use Sensor settings (gear icon) → **FINISH INTERRUPTED UPDATE**. As a last resort, use Nordic's *nRF Device Firmware Update* app with the `.zip` from [`Firmware/firmware-release/`](../../Firmware/firmware-release/), on target `XIAO_DFU`.

## 5. Check it works

**The firmware only stays awake when the power switch is closed.** It reads the switch on pin D6, and with no switch wired, D6 reads as "open": the board goes straight into deep sleep and seems dead. That is expected. On a bare board, wire the switch first ([assembly step 1](README.md#1-wire-the-power-switch)), or briefly join **D6** and **GND** with a wire.

With the switch closed:
- the status LED blinks red;
- the app finds the sensor as `VBT-Sensor` and shows its firmware version in the Bluetooth menu.

From now on, firmware updates come from the app.

## If something goes wrong

| What you see | What to do |
|---|---|
| `shasum` prints a different value | Delete the file and download it again from the link in step 1. Never use it. |
| No port in step 2 | Try another cable (charge-only cables have no data) and another USB port, directly on the computer, not through a hub. |
| `adafruit-nrfutil` stops with "No ACK" or a timeout | Close any other program using the port, list the ports again and repeat step 3 from the beginning, with the new port name if it changed. |
| In step 3.1 the port vanishes and doesn't come back | The board already runs OTAFIX with no firmware: go to step 4. |
| A firmware update from the app stopped half-way | The sensor waits in update mode and doesn't connect normally. In the app: Bluetooth → **SCAN FOR DEVICES** → **FINISH UPDATE** on "Sensor in update mode". |
| The board shows no sign of life, even with D6 joined to GND, and no "Sensor in update mode" appears | The bootloader write was interrupted. Recovery needs an SWD programmer (J-Link or a CMSIS-DAP probe) on the XIAO's SWD pads, with `xiao_nrf52840_ble_sense_bootloader-0.9.2-OTAFIX2.3-BP1.4_s140_7.3.0.hex` from the same release. On a glued sensor this means opening the case. |

## Other way: the UF2 drive (reset button reachable)

On a board whose reset button you can reach, OTAFIX's own README offers a way that replaces only the bootloader and keeps the firmware. ODKI has not tested it on a factory-new board.
1. Double-press reset: a USB drive named **`XIAO-SENSE`** appears.
2. Copy `update-xiao_nrf52840_ble_sense_bootloader-0.9.2-OTAFIX2.3-BP1.4_nosd.uf2` onto it. Its SHA-256 is `5aa99201f3adcc128e1ebdc7ea4b4ba8e370ededda13ad94942059da5a1617c4`. Wait 30 seconds without unplugging.
3. Double-press reset again. `INFO_UF2.TXT` on the drive must read `UF2 Bootloader 0.9.2-OTAFIX2.3-BP1.4` and `Board-ID: nRF52840-SeeedXiaoSense-v1`.

The same drive is how to check which bootloader a board has. The bootloader the XIAO ships with reads `0.6.2-…`, according to user reports.

## About the files

| | |
|---|---|
| Project | [oltaco/Adafruit_nRF52_Bootloader_OTAFIX](https://github.com/oltaco/Adafruit_nRF52_Bootloader_OTAFIX), a fork of [adafruit/Adafruit_nRF52_Bootloader](https://github.com/adafruit/Adafruit_nRF52_Bootloader) |
| License | MIT (Copyright (c) 2016 Adafruit Industries). The bundled SoftDevice is under Nordic Semiconductor's own license (`lib/softdevice/s140_nrf52_7.3.0/s140_nrf52_7.3.0_license-agreement.txt` in that project): redistribution in binary form must keep Nordic's notice, use only with Nordic chips, no reverse engineering. |
| Checksums | The ones GitHub publishes for the release assets. ODKI downloaded the files on 2 and 12 November 2026 and got the same values. |

ODKI links to the upstream release instead of copying the binaries into this repository. A link credits the maintainer, points at files whose origin anyone can check, and keeps third-party binaries and their license conditions out of this repository. They are provided by their authors "as is"; ODKI does not modify or support them beyond this guide.
