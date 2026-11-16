# Battery calibration

The sensor learns to read its own battery correctly the first time it finishes a full charge while it is switched on.

## When to do it

- **Once, after you build the sensor** (battery connected, firmware installed).
- **If you replace the XIAO board.** The correction belongs to the board, not to the battery.
- **If the battery percentage in the app looks wrong**, for example it never gets close to 100% after a full charge.

Until the first calibration, the sensor uses a value shared by all boards, so the percentage it shows can be a few points off.

## Steps

1. **Make sure the battery is not full.** Use the sensor for a while first. A battery that is already full when you plug it in does not calibrate.
2. **Switch the sensor on.** The status LED blinks red. It does not need to be connected to the app.
3. **Plug in the USB-C cable**, to a computer or a phone charger. The board's charge LED lights up.
4. **Leave it alone until the charge LED goes off, then at least one more minute.** Keep the switch on and the cable in the whole time. Don't unplug it the moment the LED goes off: the sensor checks the battery every 30 seconds and needs one more check to notice that charging has finished. If you can't see the charge LED, leave it plugged in overnight.
5. **Unplug the cable.** That's it. The sensor keeps the result when it is switched off and when it sleeps, so you don't need to repeat it.

The sensor must stay **switched on** during the charge. With the switch off, the battery still charges, but the sensor is asleep and doesn't calibrate.

## Check that it worked

- **In the app:** right after the charge, the battery reads close to 100%. It can drop by a point or two in the following minutes, which is normal.
- **Optional, with a serial monitor** (USB, 115200 baud) open while it charges: when charging finishes, the sensor prints a line that starts with

  `BatteryMonitor: charging complete (~CHG released, USB still present), auto-calibration applied. New factor:`

  followed by a number. At every start-up after that, it prints `BatteryMonitor: calibration factor loaded from flash:` followed by the same number.

## If it didn't work

| What happened | What to do |
|---|---|
| The battery was already full, or almost full, when you plugged it in | Use the sensor until the battery is lower, then repeat the steps. |
| The cable was unplugged, or the sensor switched off, before the check after the LED went off | Repeat the steps. Nothing is lost: the sensor keeps its previous value. |
| The sensor was switched off during the charge | Repeat the steps with the switch on. |
| The serial monitor shows `BatteryMonitor: automatic calibration discarded (factor ... outside the plausible +-15% range).` | The reading was too far off to be a normal difference between boards, so the sensor ignored it. Check the battery wires and solder joints on the B+ / B- pads, then repeat the steps. If it happens again, ask in [Discussions](https://github.com/ODKI-tech/ODKI-VBT/discussions). |

## How it works

The XIAO's charge chip (BQ25100) drives the charge LED through its `~CHG` signal, which the firmware also reads. When `~CHG` goes from "charging" to "done" while USB power is still present, the battery is at its full-charge voltage, 4.20 V. The firmware compares its own reading with 4.20 V, computes a correction factor and saves it to the board's flash memory. Two checks guard against a false result: charging must have been seen on at least 3 consecutive checks about 30 seconds apart, and a factor more than 15% away from 1.0 is discarded. The checks run only while the firmware is awake, which is why the switch must stay on. Details: [BatteryMonitor in `Firmware/DOCUMENTATION.md`](../Firmware/DOCUMENTATION.md#batterymonitor).
