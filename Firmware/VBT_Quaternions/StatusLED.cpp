// Part of the ODKI VBT Firmware
// Copyright (C) 2026 Lodovico Cortelazzo
//
// Licensed under the GNU General Public License, Version 3 (this
// specific version only, not "or any later version"), modified by the
// Commons Clause License Condition v1.0 -- see LICENSE-FIRMWARE in the
// repository root for the full text of both. In short: you may use,
// study, modify, and share this file (including a modified version) for
// non-commercial purposes; you may not sell it, or a product/service
// substantially derived from it, without a separate agreement with the
// copyright holder.
//
// This program is distributed WITHOUT ANY WARRANTY, without even the
// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
// PURPOSE -- see LICENSE-FIRMWARE for details.

#include "StatusLED.h"

namespace {
  // This board's onboard RGB LED is wired active-low:
  // LOW = on, HIGH = off (verified empirically: the board package's
  // LED_STATE_ON macro is misleading for this variant).
  //
  // The channels don't have the same perceived brightness at equal
  // current (red visually "dominates" over green) - that's why yellow
  // is driven via PWM (analogWrite) with red attenuated, instead of
  // turning both channels on at full digital intensity.
  // RED_MIX_BRIGHTNESS is the only value to tweak if yellow isn't
  // well balanced yet: lower = less red/more greenish, higher = more
  // orange. Valid range 0-255.
  const uint8_t RED_MIX_BRIGHTNESS = 30;
  const uint8_t FULL_BRIGHTNESS = 255;

  // brightness: 0 = off, 255 = maximum brightness. Inverts the
  // standard PWM convention because the pin is active-low.
  void setChannel(uint32_t pin, uint8_t brightness) {
    analogWrite(pin, 255 - brightness);
  }

  void setRGB(uint8_t red, uint8_t green, uint8_t blue) {
    setChannel(LED_RED, red);
    setChannel(LED_GREEN, green);
    setChannel(LED_BLUE, blue);
  }

  void red() { setRGB(FULL_BRIGHTNESS, 0, 0); }
  void green()  { setRGB(0, FULL_BRIGHTNESS, 0); }
  void blue()   { setRGB(0, 0, FULL_BRIGHTNESS); }
  void off_()   { setRGB(0, 0, 0); }

  const unsigned long BLINK_INTERVAL_MS = 500;
  unsigned long lastBlinkTime = 0;
  bool blinkOn = false;
}

void StatusLED::begin() {
  analogWriteResolution(8);
  off_();
}

void StatusLED::update(bool connected, bool calibrated, bool trackingActive) {
  if (!connected) {
    unsigned long now = millis();
    if (now - lastBlinkTime >= BLINK_INTERVAL_MS) {
      lastBlinkTime = now;
      blinkOn = !blinkOn;
    }
    if (blinkOn) red(); else off_();
    return;
  }

  if (!calibrated) {
    red();
  } else if (trackingActive) {
    blue();
  } else {
    green();
  }
}

void StatusLED::off() { off_(); }
