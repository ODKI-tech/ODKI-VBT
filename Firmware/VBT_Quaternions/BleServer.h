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

#ifndef BLE_SERVER_H
#define BLE_SERVER_H

#include <Arduino.h>
#include "MotionTracker.h"

// BLE server. The CUSTOM service (see UUID_SERVICE in BleServer.cpp) has 7
// characteristics:
//
//   Stream          (Notify)   - fast, decimated telemetry (see Config.h),
//                                 sent only while tracking is active.
//                                 v3.11.0: MotionSample::refVelZ is now the
//                                 RAW velocity (never corrected in real
//                                 time, see MotionTracker.h) - the live
//                                 graph should be drawn with these values
//                                 as they arrive, and then UPDATED when the
//                                 CorrectedCurve blocks for the same
//                                 interval arrive. v3.11.2: referenced to
//                                 an anchor kept continuously re-aligned
//                                 while the sensor is confirmed still (see
//                                 the end of stepPhaseEngine() in
//                                 MotionTracker.cpp), not just the
//                                 displayed value - it reads ~0
//                                 continuously during a real pause instead
//                                 of staying stuck on a not-yet-converged
//                                 bias residual, and the next rep starts
//                                 its own correction from that same fresh
//                                 reference.
//   RepSummary      (Notify)   - one packet for every completed phase
//                                 (concentric + any preceding eccentric),
//                                 keyed on repNumber. v3.11.0: the SAME rep
//                                 can be sent multiple times, with
//                                 RepSummaryPacket::correctionStatus
//                                 improving (Provisional/RepCalibrated ->
//                                 Corrected) as the bracket containing it
//                                 closes - the client must OVERWRITE by
//                                 repNumber, not append. Notify, NEVER
//                                 indicate(): see the historical comment
//                                 on indicate() further below.
//   CorrectedCurve  (Notify)   - v3.11.0, NEW: blocks of CORRECTED
//                                 velocity for a time interval already
//                                 transmitted via Stream as raw velocity.
//                                 A bracket can contain thousands of
//                                 samples (up to tens of seconds at
//                                 100Hz) - too many for a single BLE
//                                 packet, so they arrive split across
//                                 multiple notifies (see
//                                 CorrectedCurveChunkPacket below):
//                                 bracketId + chunkIndex/totalChunks let
//                                 the client group together the pieces of
//                                 ONE bracket, startTimestampMs +
//                                 sampleIntervalMs let it reconstruct the
//                                 timestamp of every sample (same
//                                 clock/scale as StreamPacket::timestampMs)
//                                 without having to transmit it for each
//                                 one. The client replaces the portion of
//                                 the graph already drawn (raw, from
//                                 Stream) in that interval with these
//                                 values.
//   SystemStatus    (Read/Notify) - calibrated/not calibrated, tracking
//                                 active/stopped. Automatic push (Notify)
//                                 on every established connection and on
//                                 every state change; the client can also
//                                 query it at any time with a standard
//                                 GATT Read (Read property).
//   Command         (Write)    - 0x01 = START, 0x00 = STOP tracking,
//                                 0x02 = CALIBRATE (recalibrate orientation)
//   Config          (Read/Write) - algorithm parameters adjustable at
//                                 runtime (see RuntimeConfig in
//                                 MotionTracker.h) - v3.11.0: ALL the
//                                 phase/bracket engine's parameters, not
//                                 just the few from v3.10.x. NOT persisted
//                                 to flash: on every reboot the firmware
//                                 starts again from the compiled defaults,
//                                 the app is responsible for re-sending
//                                 the last configuration used on every new
//                                 connection. Read always returns the
//                                 configuration ACTIVE at that moment.
//
// RepSummary is Notify, NEVER indicate(): indicate() blocks the entire
// loop() (and therefore also IMU sampling) until the client confirms,
// WITHOUT any timeout (verified in the Bluefruit52Lib library -
// waitForIndicateConfirm() uses xSemaphoreTake(..., portMAX_DELAY)) -
// diagnosed cause of missing/delayed reps reported by the user in
// v3.10.2, fixed in v3.10.3. sendRepSummary()/sendCorrectedCurveChunk()
// return false if notify() fails (not connected, or notify buffer full)
// - in that case the caller (see VBT_Quaternions.ino) does NOT consume
// the element from the corresponding queue in MotionTracker and retries
// on the next sample: no data is ever lost (not even during a BLE
// disconnect/reconnect), and the loop never blocks.
//
// Battery and FirmwareVersion instead use the STANDARD Bluetooth SIG
// services (natively recognized by any BLE tool, including nRF Connect),
// via the classes already included in Bluefruit52Lib:
//   BLEBas (Battery Service, 0x180F / Battery Level 0x2A19, Read/Notify,
//           1 byte percentage 0-100)
//   BLEDis (Device Information Service, 0x180A / Firmware Revision String
//           0x2A26, Read, "MAJOR.MINOR.PATCH" string - static for the
//           whole session, set once in BleServer::begin() from the
//           constants in Config.h)
//
// Separately, the Adafruit bootloader also exposes the "buttonless DFU"
// service (BLEDfu class from the same Bluefruit52Lib, enabled in
// BleServer::begin()): the app writes to it to make the device reboot
// into bootloader mode, which then exposes the actual Nordic Secure DFU
// service for the firmware transfer - handled entirely by the app-side
// DFU library (e.g. nordic_dfu on Flutter), NEVER by custom code here.
//
// Binary packet layout (little-endian, like the whole nRF52 / BLE
// platform): see the structs below. The client must know this layout to
// parse them.
//
// All physical quantities are transmitted as FIXED-POINT SCALED
// INTEGERS, not floats: this halves/reduces the payload and works
// identically on any client platform (no endianness/IEEE-754 format to
// handle). To recover the physical value: real_value = raw / scale (see
// the comment for each field and the summary table at the end of this
// file).
namespace BleServer {
  void begin();
  bool connected();

  // Call on every calibration/tracking state change (including from the
  // connect callback, to send the current state to the new client).
  void updateSystemStatus(bool calibrated, bool trackingActive);

  void sendStream(const MotionSample& sample);
  // true if notify() was sent successfully (connected, buffer free) -
  // see the RepSummary comment at the top of the file for how the
  // caller must use this return value.
  bool sendRepSummary(const RepResult& rep);
  // v3.11.0, NEW - see CorrectedCurve at the top of the file.
  bool sendCorrectedCurveChunk(const CorrectedCurveChunk& chunk);
  // percent: 0-100, see BatteryMonitor::voltageToPercent().
  void sendBattery(float percent);

  // Commands received from the client (written to the Command
  // characteristic), to be consumed in the main loop. Each "take" call
  // returns true at most once per command received.
  bool takeStartRequested();
  bool takeStopRequested();
  bool takeCalibrateRequested();

  // New configuration received from the client (written to the Config
  // characteristic), to be consumed in the main loop with
  // MotionTracker::setConfig(outConfig) - same "take" scheme as the
  // commands above. Returns true at most once per write received.
  bool takeConfigUpdate(RuntimeConfig& outConfig);
}

#pragma pack(push, 1)

struct StreamPacket {
  uint32_t timestampMs;                    // ms, unchanged
  int16_t linAccX, linAccY, linAccZ;       // x100  -> m/s^2 (0.01 m/s^2 resolution)
  int16_t worldAccX, worldAccY, worldAccZ; // x100  -> m/s^2
  int16_t worldVelX, worldVelY;            // x1000 -> m/s   (1 mm/s resolution) - streaming/debug only, no role in phases/reps
  int16_t refVelZ;                         // x1000 -> m/s   (v3.11.0: directional velocity, never RETROACTIVELY corrected in real time - see CorrectedCurve; v3.11.2: referenced to an anchor continuously re-aligned while still, see above)
  int16_t worldPosX, worldPosY, worldPosZ; // x1000 -> m     (1 mm resolution)
  int16_t quatW, quatX, quatY, quatZ;      // x10000 -> dimensionless, range [-1,1]
  uint8_t phaseState;                      // 0=Idle, 1=Eccentric, 2=Concentric - see PhaseState in MotionTracker.h
}; // 4 + 12*2 + 3*2 + 4*2 + 1 = 37 bytes

struct RepSummaryPacket {
  // repNumber: 1-based INTEGER rep number. v3.11.0: can be sent MULTIPLE
  // TIMES for the same repNumber (see correctionStatus below and the
  // RepSummary comment at the top of the file) - the client overwrites.
  uint8_t repNumber;
  int16_t peakVelocity;     // x1000 -> m/s (concentric)
  int16_t meanVelocity;     // x1000 -> m/s (concentric)
  int16_t peakAcceleration; // x100  -> m/s^2 (concentric)
  int16_t meanAcceleration; // x100  -> m/s^2 (concentric)
  uint16_t displacementM;   // x1000 -> m (1mm resolution), always >=0: distance covered during the concentric phase only
  int16_t eccPeakVelocity;  // x1000 -> m/s - eccentric peak of this cycle, 0 if absent (rep already purely concentric)
  int16_t eccMeanVelocity;  // x1000 -> m/s - eccentric mean of this cycle, 0 if absent
  // quality1: rep CONFIDENCE, 0-100 - kinematic coefficient.
  uint8_t quality1;
  // correctionStatus: 0=Provisional, 1=RepCalibrated, 2=Corrected - see
  // CorrectionStatus in MotionTracker.h. The WORST (least reliable) of
  // the eccentric phase's status and the concentric phase's status.
  uint8_t correctionStatus;
}; // 1 + 4*2 + 2 + 2*2 + 1 + 1 = 17 bytes

// v3.11.0, NEW - see CorrectedCurve at the top of the file. bracketId
// identifies the bracket (increments on every close); chunkIndex/
// totalChunks let a long bracket's pieces be reassembled; sampleCount is
// how many elements of correctedVelZ are valid in THIS packet (a
// bracket's last piece can be partial - the rest of the array is
// zero-padding, to be ignored). Fixed length (setFixedLen) like the
// other characteristics: every notify always has the same size.
struct CorrectedCurveChunkPacket {
  uint16_t bracketId;
  uint16_t chunkIndex;      // 0-based
  uint16_t totalChunks;
  uint8_t sampleCount;      // 1..CorrectedCurveChunk::CAPACITY valid entries in correctedVelZ
  uint32_t startTimestampMs; // ms - timestamp of correctedVelZ[0], same scale as StreamPacket::timestampMs
  uint16_t sampleIntervalMs; // ms between one sample and the next in this block (nominal: 10ms at 100Hz)
  int16_t correctedVelZ[20]; // x1000 -> m/s, RETROACTIVELY corrected velocity
}; // 2+2+2+1+4+2 + 20*2 = 53 bytes

struct SystemStatusPacket {
  uint8_t calibrated;     // 0/1
  uint8_t trackingActive; // 0/1
}; // 2 bytes (unchanged)

// Layout of the Config characteristic (Read/Write) - mirror of
// RuntimeConfig (MotionTracker.h) in fixed-point format, same scheme as
// the other characteristics. repDirection is already a direct integer
// (no scale needed); durations are in milliseconds, velocities in mm/s.
//
// v3.11.0: phase/bracket engine, ALL parameters validated with
// sim_v2_drift.py - see the version note in MotionTracker.cpp.
struct RuntimeConfigPacket {
  uint8_t repDirection;                       // 0=Up, 1=Down
  uint16_t maxPlausibleVelocityMmps;          // x1    -> mm/s
  uint16_t accZBiasIdleStillTimeMs;           // x1    -> ms
  uint16_t gyroBiasIdleStillTimeMs;           // x1    -> ms (v3.11.0: gap filled in - never exposed before v3.10.6)
  uint16_t accZBiasGyroMaxDegSx10;            // x10   -> degrees/s
  uint16_t accZBiasAccMagToleranceX1000;      // x1000 -> m/s^2
  uint16_t flatGuardMaxVelocityMmps;          // x1    -> mm/s
  uint8_t velocityOverrideFlatWindowSamples;  // direct, samples (v3.11.27, replaces flatGuardOverrideStillTimeMs - see RuntimeConfig)
  uint16_t velocityFlatBandMmps;              // x1    -> mm/s
  uint8_t maxVelocityFlatWindowSamples;       // direct, samples
  uint8_t minVelocityFlatWindowSamples;       // direct, samples
  uint16_t accelerationFlatBandX1000Mps2;     // x1000 -> m/s^2
  uint8_t maxAccelerationFlatWindowSamples;   // direct, samples
  uint8_t minAccelerationFlatWindowSamples;   // direct, samples
  uint16_t windowSaturationPeakVelocityMmps;  // x1    -> mm/s
  uint16_t phaseStartVelocityMmps;            // x1    -> mm/s
  uint16_t minPhaseDurationMs;                // x1    -> ms
  uint16_t maxPhaseDurationMs;                // x1    -> ms
  uint8_t phaseLookbackSamples;               // direct, samples
  uint8_t reversalConfirmSamples;             // direct, samples
  uint16_t emaAlphaX1000;                     // x1000 -> dimensionless [0,1]
  int16_t minCrossingExcursionMmps;           // x1    -> mm/s, negative
  uint16_t minCrossingDurationMs;             // x1    -> ms
}; // 38 bytes (v3.11.27: was 39 - flatGuardOverrideStillTimeMs (uint16_t)
   // replaced by velocityOverrideFlatWindowSamples (uint8_t), see the
   // version note in MotionTracker.cpp: the raw-quiet-time safety net was
   // replaced by a fixed sample-count window over velZLive/worldAccZ.
   // v3.11.24: was 40 before that - debugLogEnabled removed, see the
   // version note in MotionTracker.cpp: the raw-serial-log vs. BLE-stream
   // choice is now automatic, driven by whether a USB-serial connection
   // is open, not a stored Config field)

#pragma pack(pop)

// ============================================================================
// SUMMARY TABLE: characteristic / message composition / scale
// ============================================================================
//
// Stream (Notify, 37 bytes): see StreamPacket above - refVelZ now raw.
//
// RepSummary (Notify, 17 bytes): see RepSummaryPacket above - can be
//   re-sent for the same repNumber with an improved correctionStatus.
//
// CorrectedCurve (Notify, 53 bytes, v3.11.0 NEW): see
//   CorrectedCurveChunkPacket above.
//
// Battery (Read/Notify, 1 byte, standard service 0x180F/0x2A19):
//   percent          uint8   direct -> 0-100
//
// SystemStatus (Read/Notify, 2 bytes):
//   calibrated       uint8   0/1
//   trackingActive   uint8   0/1
//
// Command (Write, 1 byte):
//   0x00 STOP, 0x01 START, 0x02 CALIBRATE
//
// Config (Read/Write, 38 bytes, v3.11.0/v3.11.24/v3.11.27): see RuntimeConfigPacket above.
//
// ============================================================================
// STANDARD SERVICES (Bluetooth SIG) - see the comment at the top of the file
// ============================================================================
//
// Battery Service (0x180F) / Battery Level (0x2A19) - Read/Notify, 1 byte:
//   percent          uint8   direct -> 0-100
//
// Device Information Service (0x180A) / Firmware Revision String (0x2A26) -
// Read, UTF-8 string (NOT fixed-point) like "3.11.0" - variable length,
// see FirmwareVersion in Config.h for how it's composed.

#endif
