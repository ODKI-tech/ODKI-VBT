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


// MadgwickAHRS::update() regression (see run.py): drives update() from the
// identity attitude through a long deterministic pseudo-random sequence
// (gyro up to +-8 rad/s, accelerometer of any size including exactly zero,
// dt 10/11 ms, beta 0.015..0.1) and prints an FNV-1a hash of the bit
// patterns of every quaternion it produces. Built once against the current
// MadgwickAHRS.cpp and once against a reference commit's: equal hashes mean
// update() is bit-for-bit unchanged.
#include <cstdint>
#include <cstdio>
#include <cstring>
#include "Arduino.h"
#include "MadgwickAHRS.h"

static uint32_t lcg = 12345u;
static float rnd() { lcg = lcg * 1664525u + 1013904223u; return (lcg >> 8) * (1.0f / 16777216.0f); } // [0,1)

int main() {
  MadgwickAHRS m;
  m.reset();
  uint64_t h = 1469598103934665603ull;
  auto mix = [&](float v) { uint32_t b; std::memcpy(&b, &v, 4); for (int k = 0; k < 4; k++) { h ^= (b >> (8 * k)) & 0xff; h *= 1099511628211ull; } };
  for (int i = 0; i < 500000; i++) {
    float gx = (rnd() - 0.5f) * 16, gy = (rnd() - 0.5f) * 16, gz = (rnd() - 0.5f) * 16;
    float ax = (rnd() - 0.5f) * 40, ay = (rnd() - 0.5f) * 40, az = (rnd() - 0.5f) * 40;
    if (i % 97 == 0) ax = ay = az = 0.0f;   // the "no accelerometer" branch
    float dt = (i % 7 == 0) ? 0.011f : 0.010f;
    float beta = 0.015f + 0.085f * rnd();
    m.update(gx, gy, gz, ax, ay, az, dt, beta);
    mix(m.q0()); mix(m.q1()); mix(m.q2()); mix(m.q3());
  }
  std::printf("%016llx %.9g %.9g %.9g %.9g\n", (unsigned long long)h, m.q0(), m.q1(), m.q2(), m.q3());
  return 0;
}
