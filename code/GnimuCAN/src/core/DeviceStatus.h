// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdint.h>

#include "BleStats.h"
#include "CanFrame.h"
#include "ProbePolicy.h"
#include "ResetReason.h"
#include "RunStateMachine.h"
#include "SupplyMonitor.h"
#include "VehicleActivity.h"

namespace gnimu {

// What the CAN side has found, kept by App.
struct CanSummary {
  uint32_t trafficBitrate;  // the rate clean traffic was heard at, 0 for none yet
  uint32_t framesSeen;      // frames received since boot
  uint8_t probes;           // gateway probes made this boot
  uint8_t maxProbes;
  ProbeResult lastProbe;
};

// One polled channel as the reports show it, kept by App.
struct ObdChannelView {
  const char* name;
  const char* unit;
  uint8_t pid;
  bool supported;
  uint32_t requests;
  uint32_t replies;
  uint32_t timeouts;
  bool fresh;      // answered within the stale window
  uint32_t ageMs;  // since the last reply, when there's been one
  float value;     // decoded, when there's been a reply
};

constexpr size_t kMaxObdChannels = 4;

// What OBD polling has found, kept by App.
struct ObdSummary {
  bool ecuFound;
  uint32_t responseId;  // the ECU's reply ID, 0x7E8 for the engine ECU
  uint32_t requestId;   // its physical request ID
  uint8_t channelCount;
  ObdChannelView channels[kMaxObdChannels];
  VehicleActivity activity;
};

// The device's state at one moment, as the D report and the status line show
// it. Built once by Diagnostics and formatted immediately on the same loop
// pass.
//
// No default member values, so it stays an aggregate: `DeviceStatus s = {};`
// zeroes everything and callers (and tests) set fields by name.
struct DeviceStatus {
  uint64_t uptimeUs;
  ResetRecord reset;  // how this boot started; fixed for the whole run
  RunState state;
  SupplyStatus supply;
  CanStatus can;
  CanSummary canSummary;
  ObdSummary obd;
  struct Ble {
    bool connected;
    BleStats stats;
  } ble;
  uint32_t consoleDropped;  // console bytes lost to a host that stopped reading
};

}  // namespace gnimu
