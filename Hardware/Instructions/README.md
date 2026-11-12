# Assembly Instructions

Step-by-step guide to build one ODKI VBT unit from the parts in [`../BOM`](../BOM/) and the enclosure files in [`../CAD`](../CAD/), ending with a working, glued-shut unit ready for normal use.

## ⚠️ Before you start

This guide assumes you're already comfortable **soldering** and **handling a LiPo battery safely** (short-circuiting, puncturing, or reverse-wiring one can cause a fire). Installing the software needs no programming: a few commands copied into a terminal, then the app ([BOOTLOADER.md](BOOTLOADER.md)). If soldering or LiPo batteries are unfamiliar, get help from someone who has done it before, or practice on scrap hardware first — this is not a beginner-friendly first electronics project.

This design and these instructions are provided as-is, under the terms of [LICENSE-HARDWARE](../../LICENSE-HARDWARE) (the firmware: see [LICENSE](../../LICENSE)), with no warranty of any kind. Building and assembling a unit from them is entirely at your own risk — ODKI accepts no responsibility for any damage to your components, injury, fire, or other loss resulting from it.

## What you need

- All parts from the [BOM](../BOM/): battery, Seeed XIAO nRF52840 Sense, 3 neodymium magnets, power switch, ~4cm of thin hookup wire (x2).
- The two printed case bodies from [CAD](../CAD/): `case-top` and `case-bottom`.
- A soldering iron + solder.
- Cyanoacrylate (superglue) — a small amount for the magnets, a bit more for the final case closure.
- A computer with a USB port, a USB-C data cable and Python 3, to install the software (step 2, [BOOTLOADER.md](BOOTLOADER.md)). No Arduino IDE needed.
- The ODKI VBT companion app (see [`../../Apps`](../../Apps/)), on a phone or tablet that is online: it installs the firmware (step 2) and runs the functional test (step 7).

**Order matters:** the switch first (the firmware only stays awake with it closed), then the software over USB with **no battery attached**, and the battery last.

## 1. Wire the power switch

The switch is used as a plain 2-position on/off contact: only 2 of its 3 pins are used — the **center (common) pin** and **one outer pin**. The third pin stays unused.

1. Cut two lengths of hookup wire, roughly 4cm each.
2. Solder one wire between the XIAO's **GND** pad and the switch's **outer** pin.
3. Solder the other wire between the XIAO's **D6** pad and the switch's **center (common)** pin.

This is the exact pin the firmware calls `WAKE_PIN` (see `Config.h` / `Firmware/DOCUMENTATION.md`, PowerManager chapter) — sliding the switch closed pulls it to GND, which is how the firmware later knows to stay awake instead of going to sleep.

![Switch and battery wired to the XIAO board](img/electronics-wiring.jpg)

## 2. Install the bootloader and the firmware (USB only, no battery yet)

Follow [BOOTLOADER.md](BOOTLOADER.md), steps 1–5: it writes the OTAFIX bootloader over the USB cable, then the app installs the ODKI firmware over Bluetooth. Do it now, with the battery not yet connected: the board runs from the USB cable, and a battery soldered to it can't get shorted while you work.

Then check the switch: closed, the status LED blinks red; open, the board goes quiet (deep sleep, see the PowerManager chapter in `Firmware/DOCUMENTATION.md`). Leave it open for the next step.

## 3. Connect the battery

**Unplug the USB cable and slide the switch open (off) first.**

Solder the battery's two leads directly to the **B+ / B-** pads on the underside of the XIAO board.

**Double-check polarity before soldering:** match the leads against the pads' silkscreen markings (or the official [Seeed XIAO nRF52840 Sense pinout diagram](https://wiki.seeedstudio.com/XIAO_BLE/) if the marking isn't clear). Reversed polarity on a LiPo connection can damage the board or the battery. Keep the two bare leads apart until each is soldered, so they never touch each other.

With the switch open the board stays asleep; closing it powers the sensor from the battery.

## 4. Prepare `case-bottom`: magnets + battery

`case-bottom` has 3 round pockets sized for the magnets, in the floor of the same compartment that holds the battery. A small integrated post inside this compartment keeps the battery pinned in place once everything is seated — it also has a second job in step 6 below.

1. Press the 3 magnets into their pockets. Add a small drop of superglue to each before/as you seat them — the fit is snug, but glue keeps them from ever working loose in use.
2. Let the glue set, then place the battery into the same compartment, on top of the magnets, against the post.

## 5. Mount the electronics in `case-top`

Slide the XIAO board (now wired to the switch and the battery) into `case-top` along its internal rail, in the direction that brings the board's USB-C connector up to the cutout in the short end wall — that's what lets the finished, glued unit be charged normally over USB without ever opening it again (the bootloader can also be rewritten through the same port, see [BOOTLOADER.md](BOOTLOADER.md); normal firmware updates go over BLE from the app). The **opposite** short end has its own dedicated recess for the switch, so it stays operable from outside once the case is closed.

Don't force the board all the way in yet if you're about to plug in a USB cable (e.g. to charge it or to check the software from step 2) — pushing a cable in can push the board back out of the rail. That's exactly what `case-bottom`'s post (step 4) is there to prevent, but only once the two halves are actually joined — see the next step.

| Battery lifted, board + switch seated in the case | Battery seated in place |
|---|---|
| ![XIAO board and switch seated in the case, battery lifted aside](img/electronics-in-case-open.jpg) | ![Battery seated in place over the board](img/electronics-in-case-closed.jpg) |

## 6. Close the case (provisionally)

The two halves aren't snap-fit or screwed — the only thing that ever holds them together is the superglue in the final step. For now, don't glue anything yet: just hold the two halves closed (by hand, a rubber band, or a strip of tape) — enough that `case-bottom`'s post backs up the XIAO board against the rail, and everything sits in roughly its final position for the test below.

## 7. Functional test

With the case closed (provisionally) and the switch on, install the companion app and check everything below **before gluing anything**:

- [ ] **LED status** — right after power-on, the status LED blinks red (not connected to the app yet), visible through the small oval window on top of `case-top`. See the StatusLED chapter in `Firmware/DOCUMENTATION.md` for the full state table.
- [ ] **BLE connection** — the app finds the device when scanning, and connects to it successfully. LED should switch from blinking red to solid red (connected, not yet calibrated).
- [ ] **Calibration** — run the app's calibration step (device held still). LED should switch to solid green (calibrated, idle).
- [ ] **Live streaming** — start tracking in the app (LED goes solid blue) and move the device: live velocity data should appear and update in real time, not freeze or stay at zero.
- [ ] **Data sanity** — velocity reads ~0 while the device is genuinely still, rises and falls smoothly and in the expected direction as you move it, and a rep gets detected and reported in the app when you perform one. No wild spikes, NaNs, or values stuck at a clamp.

If **any** of these fail, open the case back up and check the solder joints from steps 1 and 3 before assuming it's a firmware issue.

## 8. Permanent assembly

Once every check above passes, open the case one last time, apply cyanoacrylate along the mating edges of `case-top` and `case-bottom`, and close them together for good. From this point on the unit is sealed — see the note in [`../CAD/README.md`](../CAD/README.md) about what that means for future battery replacement.
