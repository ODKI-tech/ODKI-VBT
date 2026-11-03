// Part of the ODKI VBT tools
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Lodovico Cortelazzo (ODKI)
//
// Licensed under the Apache License, Version 2.0 (the "License"); you may
// not use this file except in compliance with the License. You may obtain a
// copy of the License in LICENSE-APACHE in the repository root or at
// http://www.apache.org/licenses/LICENSE-2.0. Unless required by applicable
// law or agreed to in writing, software distributed under the License is
// distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
// KIND, either express or implied. See the License for the specific
// language governing permissions and limitations under the License.

// Host-side stand-in for <Arduino.h>: just enough for MotionTracker.cpp /
// FusionOrientation.cpp to compile and run on a PC (see replay.cpp). Serial writes to stdout.
#pragma once
#include <cstdint>
#include <cstdio>
#include <cmath>
#include <algorithm>

#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define HIGH 1
#define OUTPUT 1
#define PIN_LSM6DS3TR_C_POWER 0

using std::min;
using std::max;

template <class T, class L, class H>
inline T constrain(T x, L lo, H hi) { return x < (T)lo ? (T)lo : (x > (T)hi ? (T)hi : x); }

inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
inline void delay(unsigned long) {}
unsigned long millis();
// v3.11.33: MotionTracker's TIMING diagnostics read micros(); on the host it
// only has to compile (the values are meaningless off-device).
inline unsigned long micros() { return millis() * 1000UL; }

struct SerialStub {
  bool operator!() const { return false; }
  explicit operator bool() const { return true; }
  void print(const char* s) { std::fputs(s, stdout); }
  void print(char c) { std::fputc(c, stdout); }
  void print(unsigned char v) { std::printf("%u", v); }
  void print(int v) { std::printf("%d", v); }
  void print(unsigned int v) { std::printf("%u", v); }
  void print(long v) { std::printf("%ld", v); }
  void print(unsigned long v) { std::printf("%lu", v); }
  void print(float v, int d = 2) { std::printf("%.*f", d, (double)v); }
  void print(double v, int d = 2) { std::printf("%.*f", d, v); }
  template <class T> void println(T v) { print(v); std::fputc('\n', stdout); }
  template <class T> void println(T v, int d) { print(v, d); std::fputc('\n', stdout); }
  void println() { std::fputc('\n', stdout); }
};
extern SerialStub Serial;
