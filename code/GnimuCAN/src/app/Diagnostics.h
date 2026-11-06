// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "../core/RateTracker.h"
#include "../core/ResetReason.h"
#include "../core/RunStateMachine.h"
#include "../core/Scheduler.h"
#include "../core/SupplyMonitor.h"
#include "../core/TextOut.h"
#include "../peripherals/ble/BlePeripheral.h"
#include "Console.h"

namespace gnimu {

// The console's reporting side: the D report, the 1 s status line and the V
// text, plus the Schema and Device info documents a BLE client reads. It only
// reads the modules it reports on; App routes each command, clears its own
// state on Clear, and passes every command here too.
class Diagnostics {
 public:
  // Everything is referenced, not copied, and must outlive this object.
  Diagnostics(Console& console, const Job* jobs, const JobStats* stats, size_t jobCount,
              const uint32_t& loops, const RunStateMachine& machine, const SupplyMonitor& supply,
              const CanSummary& can, const ObdSummary& obd, const BlePeripheral& ble, const char* deviceId,
              const ResetRecord& reset)
      : console_(console),
        jobs_(jobs),
        stats_(stats),
        jobCount_(jobCount),
        loops_(loops),
        machine_(machine),
        supply_(supply),
        can_(can),
        obd_(obd),
        ble_(ble),
        deviceId_(deviceId),
        reset_(reset) {}

  // One poll's result: Report, Version, ToggleStatus and a count act here;
  // Clear re-baselines the status line's rate tracker.
  void handle(const ConsoleCommand& cmd);
  // Prints the status line when one is due.
  void tick(uint64_t nowUs);

  // What this device is: the build, the device ID and the settings it was
  // built with. The V command, and the top of the boot log.
  void printVersion();
  // The P command: the BLE protocol setup a fixed-layout app needs, one item
  // per line: service, notify characteristic, packet length, then each field
  // the device fills, in packet order.
  void printProtocol();

  // The Schema and Device info characteristics' JSON (Gnimu protocol,
  // docs/frame.md). Truncated, never overrun; return the length.
  size_t formatSchema(char* buf, size_t cap) const;
  size_t formatDeviceInfo(char* buf, size_t cap) const;

 private:
  void armStatus(uint32_t count);
  size_t formatVersion(char* buf, size_t cap) const;
  size_t formatProtocol(char* buf, size_t cap) const;
  DeviceStatus snapshot(uint64_t nowUs) const;
  void printReport();
  void printStatus(uint64_t nowUs);

  Console& console_;
  const Job* jobs_;
  const JobStats* stats_;
  size_t jobCount_;
  const uint32_t& loops_;
  const RunStateMachine& machine_;
  const SupplyMonitor& supply_;
  const CanSummary& can_;
  const ObdSummary& obd_;
  const BlePeripheral& ble_;
  const char* deviceId_;
  const ResetRecord& reset_;

  RateTracker frameRate_;  // status line: frames sent/s
  RateTracker channelRate_[kMaxObdChannels];  // status line: replies/s per channel
  // Status lines still to print: 0 = off. Both L and a typed count arm it,
  // never beyond kStatusMaxCount.
  uint32_t statusRemaining_ = 0;
  uint64_t nextStatusDueUs_ = 0;
};

}  // namespace gnimu
