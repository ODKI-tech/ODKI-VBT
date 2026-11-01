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


// Host test of the CALIBRATE attitude seed (v3.11.33 fix), see run.py.
//  1. For tilts 0..180 deg (incl. 90 deg and the near-180 deg degenerate
//     cases) x 24 azimuths x 3 magnitudes, the gravity direction implied by
//     the quaternion MadgwickAHRS::alignToGravity() seeds - read with the
//     expression update() and MotionTracker's gravBody use - must equal the
//     normalised accelerometer within 1e-5 (per component). Same for
//     FusionOrientation::alignToGravity().
//  2. Right after the seed, 3 s of still updates (zero gyro, the same
//     accelerometer) through each filter must keep the attitude error < 1
//     deg at every step (it is a fixed point: a convention mismatch between
//     the seed and update() would show here).
//  3. Prints the error the pre-v3.11.33 seed had (2 x tilt) for reference.
// Exit code 0 = pass.
#include <cmath>
#include <cstdio>
#include "Arduino.h"
#include "MadgwickAHRS.h"
#include "FusionOrientation.h"

static void gravBody(float q0, float q1, float q2, float q3, double g[3]) {
  g[0] = 2.0 * (q1 * q3 - q0 * q2);
  g[1] = 2.0 * (q0 * q1 + q2 * q3);
  g[2] = (double)q0 * q0 - (double)q1 * q1 - (double)q2 * q2 + (double)q3 * q3;
}

static double angleDeg(const double a[3], const double b[3]) {
  double na = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  double nb = std::sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
  double c = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (na * nb);
  if (c > 1) c = 1;
  if (c < -1) c = -1;
  return std::acos(c) * 180.0 / M_PI;
}

// The pre-v3.11.33 alignToGravity(), verbatim, for the reference column.
static void legacySeed(float ax, float ay, float az, float q[4]) {
  float norm = sqrt(ax * ax + ay * ay + az * az);
  ax /= norm; ay /= norm; az /= norm;
  float vx = -ay, vy = ax, vz = 0;
  float vNorm = sqrt(vx * vx + vy * vy + vz * vz);
  if (vNorm < 1e-6f) { q[0] = 1; q[1] = q[2] = q[3] = 0; return; }
  vx /= vNorm; vy /= vNorm; vz /= vNorm;
  float angle = acos(constrain(az, -1.0f, 1.0f));
  float s = sin(angle / 2.0f);
  q[0] = cos(angle / 2.0f); q[1] = vx * s; q[2] = vy * s; q[3] = vz * s;
}

int main() {
  const double tiltsExtra[] = {0.0, 1e-4, 0.01, 0.1, 89.9, 90.0, 90.1, 179.0, 179.9, 179.99, 179.999, 179.9999, 180.0};
  double tilts[200]; int nt = 0;
  for (int d = 0; d <= 180; d += 1) tilts[nt++] = d;
  for (double x : tiltsExtra) tilts[nt++] = x;
  const double mags[] = {1.0, 9.80665, 0.3};
  double maxCompM = 0, maxCompF = 0, maxAngM = 0, maxAngF = 0, maxStillM = 0, maxStillF = 0;
  double worstTiltM = 0, worstTiltF = 0;
  long cases = 0;
  for (int it = 0; it < nt; it++) {
    for (int az = 0; az < 360; az += 15) {
      for (double mag : mags) {
        double t = tilts[it] * M_PI / 180.0, p = az * M_PI / 180.0;
        double a[3] = {std::sin(t) * std::cos(p), std::sin(t) * std::sin(p), std::cos(t)};
        float fa[3] = {(float)(a[0] * mag), (float)(a[1] * mag), (float)(a[2] * mag)};
        // the float input's own direction is the reference
        double an = std::sqrt((double)fa[0] * fa[0] + (double)fa[1] * fa[1] + (double)fa[2] * fa[2]);
        double ref[3] = {fa[0] / an, fa[1] / an, fa[2] / an};
        cases++;

        MadgwickAHRS m;
        m.alignToGravity(fa[0], fa[1], fa[2]);
        double g[3];
        gravBody(m.q0(), m.q1(), m.q2(), m.q3(), g);
        for (int k = 0; k < 3; k++) {
          double e = std::fabs(g[k] - ref[k]);
          if (e > maxCompM) { maxCompM = e; worstTiltM = tilts[it]; }
        }
        maxAngM = std::fmax(maxAngM, angleDeg(g, ref));
        for (int i = 0; i < 300; i++) {
          m.update(0, 0, 0, fa[0], fa[1], fa[2], 0.01f, 0.1f);
          gravBody(m.q0(), m.q1(), m.q2(), m.q3(), g);
          maxStillM = std::fmax(maxStillM, angleDeg(g, ref));
        }

        FusionOrientation f;
        f.alignToGravity(fa[0], fa[1], fa[2]);
        gravBody(f.q0(), f.q1(), f.q2(), f.q3(), g);
        for (int k = 0; k < 3; k++) {
          double e = std::fabs(g[k] - ref[k]);
          if (e > maxCompF) { maxCompF = e; worstTiltF = tilts[it]; }
        }
        maxAngF = std::fmax(maxAngF, angleDeg(g, ref));
        for (int i = 0; i < 300; i++) {
          f.update(0, 0, 0, fa[0], fa[1], fa[2], 0.01f);
          gravBody(f.q0(), f.q1(), f.q2(), f.q3(), g);
          maxStillF = std::fmax(maxStillF, angleDeg(g, ref));
        }
      }
    }
  }
  std::printf("cases: %ld (%d tilts x 24 azimuths x 3 magnitudes)\n", cases, nt);
  std::printf("Madgwick seed: max |gravBody - a| per component %.3g (at tilt %g deg), max angle %.3g deg\n",
              maxCompM, worstTiltM, maxAngM);
  std::printf("Fusion seed:   max |gravBody - a| per component %.3g (at tilt %g deg), max angle %.3g deg\n",
              maxCompF, worstTiltF, maxAngF);
  std::printf("still for 3 s after the seed: max attitude error Madgwick %.3g deg, Fusion %.3g deg\n",
              maxStillM, maxStillF);
  std::printf("pre-v3.11.33 seed, error vs tilt (azimuth 30 deg):");
  for (double td : {15.0, 45.0, 72.0, 90.0, 135.0, 180.0}) {
    double t = td * M_PI / 180.0, p = 30 * M_PI / 180.0;
    double ref[3] = {std::sin(t) * std::cos(p), std::sin(t) * std::sin(p), std::cos(t)};
    float q[4];
    legacySeed((float)ref[0], (float)ref[1], (float)ref[2], q);
    double g[3];
    gravBody(q[0], q[1], q[2], q[3], g);
    std::printf(" %g->%.1f", td, angleDeg(g, ref));
  }
  std::printf(" deg\n");
  bool ok = maxCompM < 1e-5 && maxCompF < 1e-5 && maxStillM < 1.0 && maxStillF < 1.0;
  std::printf("%s\n", ok ? "PASS" : "FAIL");
  return ok ? 0 : 1;
}
