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

#include "BleServer.h"
#include "Config.h"
#include <bluefruit.h>
// BLEDfu (a class from the same Bluefruit52Lib, see bluefruit.h): friend
// service of the Adafruit bootloader, exposes only the "buttonless DFU"
// characteristic (writing to it reboots the device into the bootloader,
// which exposes the ACTUAL Nordic Secure DFU service for the transfer -
// this second step is handled entirely by the factory bootloader, NEVER
// by this file). App-side DFU libraries (e.g. nordic_dfu on Flutter)
// recognize this service automatically.

namespace {
  // Base UUID generated once (random v4): 6c7325bb-1784-4d77-8b99-89ce838ac2ab
  // The characteristics derive from it by incrementing the last byte. The
  // arrays are in little-endian order (least significant byte first), as
  // required by Bluefruit52Lib's BLECharacteristic/BLEService. 0xAE and
  // 0xB1 (custom Battery/FirmwareVersion) have been freed up: that data
  // now uses the standard SIG services (BLEBas/BLEDis below), they're no
  // longer part of this custom service.
  const uint8_t UUID_SERVICE[]      = {0xAB,0xC2,0x8A,0x83,0xCE,0x89,0x99,0x8B,0x77,0x4D,0x84,0x17,0xBB,0x25,0x73,0x6C};
  const uint8_t UUID_CHR_STREAM[]   = {0xAC,0xC2,0x8A,0x83,0xCE,0x89,0x99,0x8B,0x77,0x4D,0x84,0x17,0xBB,0x25,0x73,0x6C};
  const uint8_t UUID_CHR_REPSUM[]   = {0xAD,0xC2,0x8A,0x83,0xCE,0x89,0x99,0x8B,0x77,0x4D,0x84,0x17,0xBB,0x25,0x73,0x6C};
  // v3.11.0: 0xAE had been freed up (old custom Battery, now standard
  // BLEBas) - reused for the new CorrectedCurve instead of growing the
  // UUID space unnecessarily.
  const uint8_t UUID_CHR_CORRCURVE[] = {0xAE,0xC2,0x8A,0x83,0xCE,0x89,0x99,0x8B,0x77,0x4D,0x84,0x17,0xBB,0x25,0x73,0x6C};
  const uint8_t UUID_CHR_SYSSTAT[]  = {0xAF,0xC2,0x8A,0x83,0xCE,0x89,0x99,0x8B,0x77,0x4D,0x84,0x17,0xBB,0x25,0x73,0x6C};
  const uint8_t UUID_CHR_COMMAND[]  = {0xB0,0xC2,0x8A,0x83,0xCE,0x89,0x99,0x8B,0x77,0x4D,0x84,0x17,0xBB,0x25,0x73,0x6C};
  const uint8_t UUID_CHR_CONFIG[]   = {0xB2,0xC2,0x8A,0x83,0xCE,0x89,0x99,0x8B,0x77,0x4D,0x84,0x17,0xBB,0x25,0x73,0x6C};

  BLEService        vbtService(UUID_SERVICE);
  BLECharacteristic streamChr(UUID_CHR_STREAM);
  BLECharacteristic repSummaryChr(UUID_CHR_REPSUM);
  BLECharacteristic correctedCurveChr(UUID_CHR_CORRCURVE);
  BLECharacteristic systemStatusChr(UUID_CHR_SYSSTAT);
  BLECharacteristic commandChr(UUID_CHR_COMMAND);
  BLECharacteristic configChr(UUID_CHR_CONFIG);

  // Standard Bluetooth SIG services - see the layout comment in BleServer.h.
  BLEBas batteryService;
  BLEDis deviceInfoService;

  // Buttonless DFU: bootloader friend service, see comment above.
  BLEDfu bledfu;

  volatile bool startRequested = false;
  volatile bool stopRequested = false;
  volatile bool calibrateRequested = false;
  volatile bool configUpdateAvailable = false;
  RuntimeConfig pendingConfig;

  bool lastCalibrated = false;
  bool lastTrackingActive = false;

  // Converts a float to a fixed-point scaled integer, rounding to the
  // nearest value (not truncating) and clamped to the int16_t range to
  // avoid wraparound on out-of-scale values.
  int16_t toFixed16(float value, float scale) {
    float scaled = value * scale;
    scaled = constrain(scaled, -32767.0f, 32767.0f);
    return (int16_t)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
  }

  uint16_t toFixedU16(float value, float scale) {
    float scaled = value * scale;
    scaled = constrain(scaled, 0.0f, 65535.0f);
    return (uint16_t)(scaled + 0.5f);
  }

  uint8_t percentToU8(float pct) {
    pct = constrain(pct, 0.0f, 255.0f);
    return (uint8_t)(pct + 0.5f);
  }

  void notifySystemStatus() {
    SystemStatusPacket pkt;
    pkt.calibrated = lastCalibrated ? 1 : 0;
    pkt.trackingActive = lastTrackingActive ? 1 : 0;
    systemStatusChr.write((uint8_t*)&pkt, sizeof(pkt));
    if (Bluefruit.connected()) {
      systemStatusChr.notify((uint8_t*)&pkt, sizeof(pkt));
    }
  }

  // Command received from the client: 0x01 = START, 0x00 = STOP, 0x02 =
  // CALIBRATE. The callback runs in the BLE scheduler's context: it just
  // sets a flag, the actual action (state reset, LED/status update) is
  // done by the main loop.
  void command_write_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
    (void) conn_hdl; (void) chr;
    if (len < 1) return;
    if (data[0] == 0x01) startRequested = true;
    else if (data[0] == 0x00) stopRequested = true;
    else if (data[0] == 0x02) calibrateRequested = true;
  }

  // RuntimeConfig (physical units, MotionTracker.h) <-> RuntimeConfigPacket
  // (fixed-point on the wire, BleServer.h) - same toFixed16/toFixedU16
  // scheme as above, but unsigned (no Config field is negative).
  RuntimeConfigPacket configToPacket(const RuntimeConfig& cfg) {
    RuntimeConfigPacket pkt;
    pkt.repDirection = (uint8_t)cfg.repDirection;
    pkt.maxPlausibleVelocityMmps = toFixedU16(cfg.maxPlausibleVelocityMps, 1000.0f);
    pkt.accZBiasIdleStillTimeMs = toFixedU16(cfg.accZBiasIdleStillTimeS, 1000.0f);
    pkt.gyroBiasIdleStillTimeMs = toFixedU16(cfg.gyroBiasIdleStillTimeS, 1000.0f);
    pkt.accZBiasGyroMaxDegSx10 = toFixedU16(cfg.accZBiasGyroMaxDegS, 10.0f);
    pkt.accZBiasAccMagToleranceX1000 = toFixedU16(cfg.accZBiasAccMagToleranceMps2, 1000.0f);
    pkt.flatGuardMaxVelocityMmps = toFixedU16(cfg.flatGuardMaxVelocityMps, 1000.0f);
    pkt.flatGuardOverrideStillTimeMs = toFixedU16(cfg.flatGuardOverrideStillTimeS, 1000.0f);
    pkt.velocityFlatBandMmps = toFixedU16(cfg.velocityFlatBandMps, 1000.0f);
    pkt.maxVelocityFlatWindowSamples = cfg.maxVelocityFlatWindowSamples;
    pkt.minVelocityFlatWindowSamples = cfg.minVelocityFlatWindowSamples;
    pkt.accelerationFlatBandX1000Mps2 = toFixedU16(cfg.accelerationFlatBandMps2, 1000.0f);
    pkt.maxAccelerationFlatWindowSamples = cfg.maxAccelerationFlatWindowSamples;
    pkt.minAccelerationFlatWindowSamples = cfg.minAccelerationFlatWindowSamples;
    pkt.windowSaturationPeakVelocityMmps = toFixedU16(cfg.windowSaturationPeakVelocityMps, 1000.0f);
    pkt.phaseStartVelocityMmps = toFixedU16(cfg.phaseStartVelocityMps, 1000.0f);
    pkt.minPhaseDurationMs = toFixedU16(cfg.minPhaseDurationS, 1000.0f);
    pkt.maxPhaseDurationMs = toFixedU16(cfg.maxPhaseDurationS, 1000.0f);
    pkt.phaseLookbackSamples = cfg.phaseLookbackSamples;
    pkt.reversalConfirmSamples = cfg.reversalConfirmSamples;
    pkt.emaAlphaX1000 = toFixedU16(cfg.emaAlpha, 1000.0f);
    pkt.minCrossingExcursionMmps = toFixed16(cfg.minCrossingExcursionMps, 1000.0f);
    pkt.minCrossingDurationMs = toFixedU16(cfg.minCrossingDurationS, 1000.0f);
    return pkt;
  }

  RuntimeConfig packetToConfig(const RuntimeConfigPacket& pkt) {
    RuntimeConfig cfg;
    cfg.repDirection = (pkt.repDirection == 1) ? RepDirection::Down : RepDirection::Up;
    cfg.maxPlausibleVelocityMps = pkt.maxPlausibleVelocityMmps / 1000.0f;
    cfg.accZBiasIdleStillTimeS = pkt.accZBiasIdleStillTimeMs / 1000.0f;
    cfg.gyroBiasIdleStillTimeS = pkt.gyroBiasIdleStillTimeMs / 1000.0f;
    cfg.accZBiasGyroMaxDegS = pkt.accZBiasGyroMaxDegSx10 / 10.0f;
    cfg.accZBiasAccMagToleranceMps2 = pkt.accZBiasAccMagToleranceX1000 / 1000.0f;
    cfg.flatGuardMaxVelocityMps = pkt.flatGuardMaxVelocityMmps / 1000.0f;
    cfg.flatGuardOverrideStillTimeS = pkt.flatGuardOverrideStillTimeMs / 1000.0f;
    cfg.velocityFlatBandMps = pkt.velocityFlatBandMmps / 1000.0f;
    cfg.maxVelocityFlatWindowSamples = pkt.maxVelocityFlatWindowSamples;
    cfg.minVelocityFlatWindowSamples = pkt.minVelocityFlatWindowSamples;
    cfg.accelerationFlatBandMps2 = pkt.accelerationFlatBandX1000Mps2 / 1000.0f;
    cfg.maxAccelerationFlatWindowSamples = pkt.maxAccelerationFlatWindowSamples;
    cfg.minAccelerationFlatWindowSamples = pkt.minAccelerationFlatWindowSamples;
    cfg.windowSaturationPeakVelocityMps = pkt.windowSaturationPeakVelocityMmps / 1000.0f;
    cfg.phaseStartVelocityMps = pkt.phaseStartVelocityMmps / 1000.0f;
    cfg.minPhaseDurationS = pkt.minPhaseDurationMs / 1000.0f;
    cfg.maxPhaseDurationS = pkt.maxPhaseDurationMs / 1000.0f;
    cfg.phaseLookbackSamples = pkt.phaseLookbackSamples;
    cfg.reversalConfirmSamples = pkt.reversalConfirmSamples;
    cfg.emaAlpha = pkt.emaAlphaX1000 / 1000.0f;
    cfg.minCrossingExcursionMps = pkt.minCrossingExcursionMmps / 1000.0f;
    cfg.minCrossingDurationS = pkt.minCrossingDurationMs / 1000.0f;
    return cfg;
  }

  // Updates the Config characteristic's GATT Read value (read by the
  // client on every new connection or on explicit request) - does not
  // emit a Notify: unlike Battery/SystemStatus, the client itself writes
  // configuration changes, so it doesn't need to be notified of a change
  // it just caused.
  void writeConfigCharacteristic(const RuntimeConfig& cfg) {
    RuntimeConfigPacket pkt = configToPacket(cfg);
    configChr.write((uint8_t*)&pkt, sizeof(pkt));
  }

  // New configuration written by the client to the Config characteristic:
  // just sets a pending flag/value, like command_write_callback above -
  // the actual application (MotionTracker::setConfig()) is done by the
  // main loop via BleServer::takeConfigUpdate(). Still updates the Read
  // value IMMEDIATELY (see writeConfigCharacteristic above), so a Read
  // immediately following the Write already sees the echo of what was
  // just written, even before the main loop has consumed it.
  void config_write_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
    (void) conn_hdl; (void) chr;
    if (len < sizeof(RuntimeConfigPacket)) return;
    RuntimeConfigPacket pkt;
    memcpy(&pkt, data, sizeof(pkt));
    pendingConfig = packetToConfig(pkt);
    configUpdateAvailable = true;
    writeConfigCharacteristic(pendingConfig);
  }

  void connect_callback(uint16_t conn_handle) {
    // StreamPacket is 36 bytes: with the default MTU (23 bytes) it
    // wouldn't fit in a single notification anyway. The maximum
    // supported is requested from the client; if the client doesn't
    // grant it, notify() will fail until the MTU is sufficient.
    BLEConnection* connection = Bluefruit.Connection(conn_handle);
    connection->requestDataLengthUpdate();
    connection->requestMtuExchange(247);

    // On every established connection, the client needs to know
    // immediately whether the system is calibrated and whether tracking
    // is active.
    notifySystemStatus();
  }

  void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
    (void) conn_handle; (void) reason;
  }
}

void BleServer::begin() {
  // Must be called before begin(): allocates enough bandwidth/RAM for
  // large MTUs (requested below, in connect_callback) and deeper notify
  // queues, useful for streaming ~20Hz of 36-byte packets.
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  // begin(prph_count, central_count): only 1 peripheral connection to
  // the app, no central role.
  Bluefruit.begin(1, 0);
  Bluefruit.setName("VBT-Sensor");
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // Enables the OTA update (BLE DFU): writing to its characteristic
  // reboots the device into the factory Adafruit bootloader, which
  // exposes the actual Nordic Secure DFU service for transferring the
  // new firmware - see the comment on the include above.
  bledfu.begin();

  vbtService.begin();

  streamChr.setProperties(CHR_PROPS_NOTIFY);
  streamChr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  streamChr.setFixedLen(sizeof(StreamPacket));
  streamChr.begin();

  repSummaryChr.setProperties(CHR_PROPS_NOTIFY);
  repSummaryChr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  repSummaryChr.setFixedLen(sizeof(RepSummaryPacket));
  repSummaryChr.begin();

  correctedCurveChr.setProperties(CHR_PROPS_NOTIFY);
  correctedCurveChr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  correctedCurveChr.setFixedLen(sizeof(CorrectedCurveChunkPacket));
  correctedCurveChr.begin();

  systemStatusChr.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  systemStatusChr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  systemStatusChr.setFixedLen(sizeof(SystemStatusPacket));
  systemStatusChr.begin();
  notifySystemStatus(); // initial value: not calibrated, not tracking

  commandChr.setProperties(CHR_PROPS_WRITE);
  commandChr.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  commandChr.setFixedLen(1);
  commandChr.begin();
  commandChr.setWriteCallback(command_write_callback);

  // Initial value = the compiled defaults (see RuntimeConfig in
  // MotionTracker.h) - MotionTracker already starts with
  // activeConfig=default RuntimeConfig(), so this just mirrors the same
  // state over BLE, without needing to go through
  // takeConfigUpdate()/setConfig() for the initial value.
  configChr.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  configChr.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  configChr.setFixedLen(sizeof(RuntimeConfigPacket));
  configChr.begin();
  configChr.setWriteCallback(config_write_callback);
  writeConfigCharacteristic(MotionTracker::defaultConfig());

  // Standard Bluetooth SIG services (see the layout comment in
  // BleServer.h) - battery updated periodically by sendBattery(),
  // firmware version static for the whole session (never a write).
  batteryService.begin();

  // setFirmwareRev() BEFORE begin(): BLEDis::begin() is what reads the
  // set value and copies it into the actual GATT characteristic (see
  // BLEDis.cpp) - called in the wrong order, begin() would still find
  // the default (the BOOTLOADER's version, not our firmware's) and the
  // assignment below would no longer have any effect.
  char fwVerStr[16];
  snprintf(fwVerStr, sizeof(fwVerStr), "%u.%u.%u",
           FirmwareVersion::MAJOR, FirmwareVersion::MINOR, FirmwareVersion::PATCH);
  deviceInfoService.setFirmwareRev(fwVerStr);
  deviceInfoService.begin();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(vbtService);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244); // 0.625ms unit: 20ms..152.5ms
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0); // 0 = advertise with no timeout
}

bool BleServer::connected() {
  return Bluefruit.connected();
}

void BleServer::updateSystemStatus(bool calibrated, bool trackingActive) {
  lastCalibrated = calibrated;
  lastTrackingActive = trackingActive;
  notifySystemStatus();
}

void BleServer::sendStream(const MotionSample& s) {
  if (!Bluefruit.connected()) return;

  StreamPacket pkt;
  pkt.timestampMs = millis();
  pkt.linAccX = toFixed16(s.linAccX, 100.0f);
  pkt.linAccY = toFixed16(s.linAccY, 100.0f);
  pkt.linAccZ = toFixed16(s.linAccZ, 100.0f);
  pkt.worldAccX = toFixed16(s.worldAccX, 100.0f);
  pkt.worldAccY = toFixed16(s.worldAccY, 100.0f);
  pkt.worldAccZ = toFixed16(s.worldAccZ, 100.0f);
  pkt.worldVelX = toFixed16(s.worldVelX, 1000.0f);
  pkt.worldVelY = toFixed16(s.worldVelY, 1000.0f);
  pkt.refVelZ = toFixed16(s.refVelZ, 1000.0f);
  pkt.worldPosX = toFixed16(s.worldPosX, 1000.0f);
  pkt.worldPosY = toFixed16(s.worldPosY, 1000.0f);
  pkt.worldPosZ = toFixed16(s.worldPosZ, 1000.0f);
  pkt.quatW = toFixed16(s.quatW, 10000.0f);
  pkt.quatX = toFixed16(s.quatX, 10000.0f);
  pkt.quatY = toFixed16(s.quatY, 10000.0f);
  pkt.quatZ = toFixed16(s.quatZ, 10000.0f);
  pkt.phaseState = (uint8_t)s.phaseState;

  streamChr.notify((uint8_t*)&pkt, sizeof(pkt));
}

bool BleServer::sendRepSummary(const RepResult& r) {
  if (!Bluefruit.connected()) return false;

  RepSummaryPacket pkt;
  pkt.repNumber = r.repNumber;
  pkt.peakVelocity = toFixed16(r.peakVelocity, 1000.0f);
  pkt.meanVelocity = toFixed16(r.meanVelocity, 1000.0f);
  pkt.peakAcceleration = toFixed16(r.peakAcceleration, 100.0f);
  pkt.meanAcceleration = toFixed16(r.meanAcceleration, 100.0f);
  pkt.displacementM = toFixedU16(r.displacementM, 1000.0f);
  pkt.eccPeakVelocity = toFixed16(r.eccPeakVelocity, 1000.0f);
  pkt.eccMeanVelocity = toFixed16(r.eccMeanVelocity, 1000.0f);
  pkt.quality1 = r.quality1;
  pkt.correctionStatus = (uint8_t)r.correctionStatus;

  // notify(), NEVER indicate(): see the RepSummary comment at the top of
  // BleServer.h - indicate() blocks the entire loop() (and therefore
  // also IMU sampling) until the client confirms, with no timeout at
  // all. The caller (see VBT_Quaternions.ino) retries on the next
  // sample if this notify() fails (not connected, or notify buffer
  // full) - no rep is ever lost, and the loop never blocks.
  return repSummaryChr.notify((uint8_t*)&pkt, sizeof(pkt));
}

bool BleServer::sendCorrectedCurveChunk(const CorrectedCurveChunk& c) {
  if (!Bluefruit.connected()) return false;

  CorrectedCurveChunkPacket pkt;
  pkt.bracketId = c.bracketId;
  pkt.chunkIndex = c.chunkIndex;
  pkt.totalChunks = c.totalChunks;
  pkt.sampleCount = c.sampleCount;
  pkt.startTimestampMs = c.startTimestampMs;
  pkt.sampleIntervalMs = c.sampleIntervalMs;
  for (uint8_t i = 0; i < CorrectedCurveChunk::CAPACITY; i++) {
    pkt.correctedVelZ[i] = toFixed16(c.correctedVelZ[i], 1000.0f);
  }

  // Same scheme as sendRepSummary() above: notify(), never indicate();
  // the caller retries the block on the next sample if this fails.
  return correctedCurveChr.notify((uint8_t*)&pkt, sizeof(pkt));
}

void BleServer::sendBattery(float percent) {
  uint8_t pct = percentToU8(percent);

  batteryService.write(pct);
  if (Bluefruit.connected()) {
    batteryService.notify(pct);
  }
}

bool BleServer::takeStartRequested() {
  if (!startRequested) return false;
  startRequested = false;
  return true;
}

bool BleServer::takeStopRequested() {
  if (!stopRequested) return false;
  stopRequested = false;
  return true;
}

bool BleServer::takeCalibrateRequested() {
  if (!calibrateRequested) return false;
  calibrateRequested = false;
  return true;
}

bool BleServer::takeConfigUpdate(RuntimeConfig& outConfig) {
  if (!configUpdateAvailable) return false;
  configUpdateAvailable = false;
  outConfig = pendingConfig;
  return true;
}
