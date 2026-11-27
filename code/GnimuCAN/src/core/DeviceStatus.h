// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdint.h>

#include "BleStats.h"
#include "CanFrame.h"
#include "Obd2Pids.h"
#include "ObdPoller.h"
#include "ProbePolicy.h"
#include "ResetReason.h"
#include "RunStateMachine.h"
#include "SupplyMonitor.h"
#include "VehicleActivity.h"

namespace gnimu {

// What the CAN side has found, kept by App.
struct CanSummary {
  bool noSupply;            // CAN is off: no 12V supply (USB alone)
  uint32_t trafficBitrate;  // the rate clean traffic was heard at, 0 for none yet
  uint32_t framesSeen;      // frames received since boot
  uint32_t probes;          // gateway probes since boot or the last vehicle found
  ProbeResult lastProbe;
};

// One polled channel as the reports show it, kept by App.
struct ObdChannelView {
  const char* name;
  const char* unit;
  uint8_t pid;
  ChannelState state;
  uint32_t periodMs;
  uint32_t requests;
  uint32_t replies;
  uint32_t timeouts;
  uint32_t replyAvgMs;  // request to reply, in can-job steps
  uint32_t replyMaxMs;
  bool fresh;      // answered within the stale window
  uint32_t ageMs;  // since the last reply, when there's been one
  float value;     // decoded, when there's been a reply

  bool polled() const { return state == ChannelState::Polling || state == ChannelState::Silent; }
};

// Room for every Mode 01 channel the decoders know, and their helpers
// (core/Obd2Pids.h).
constexpr size_t kMaxObdChannels = obd2::kPidDecoderCount + obd2::kHelperDecoderCount;

// What OBD polling has found, kept by App.
struct ObdSummary {
  bool ecuFound;
  bool nothingToPoll;   // ECUs answered, but none supports a polled PID: asking again later
  uint8_t answeredEcus; // bit n: 0x7E8 + n answered the last discovery request
  uint32_t responseId;  // the ECU's reply ID, 0x7E8 for the engine ECU
  uint32_t requestId;   // its physical request ID
  uint8_t channelCount;
  ObdChannelView channels[kMaxObdChannels];
  VehicleActivity activity;
};

// Where sleep stands, kept by App (docs/architecture.md, Sleep and the cutoff).
struct SleepView {
  bool stayAwake;         // the console's W: never sleep this boot
  bool noSupply;          // no 12 V: on USB, never sleeps
  float noSupplyV;        // ... below this
  bool parked;            // the parked conditions hold now
  uint32_t parkedForMs;   // ... for this long
  uint32_t sleepAfterMs;  // parked this long: sleep
  uint32_t sleepCount;    // times slept since power-up
  // The wake that started this boot, if one did.
  bool woke;
  const char* wokeFor;   // the reason, as text
  const char* asleepFor; // parked or below cutoff
  uint32_t checks;       // wake checks before it
  float sleptAtV;        // the awake reading when it went to sleep
  float checkMinV;       // the range the wake checks read
  float checkMaxV;
  float wokeAtV;         // the reading it woke on
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
  SleepView sleep;
  struct Ble {
    bool connected;
    BleStats stats;
  } ble;
  uint32_t consoleDropped;  // console bytes lost to a host that stopped reading
};

}  // namespace gnimu
