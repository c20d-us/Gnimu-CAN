// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdint.h>

namespace gnimu {

// Starting: bring-up at boot, before the scheduler runs. WaitingForVehicle:
// the hardware is up but no ECU answers. Polling: an ECU answers and the
// vehicle fields are being polled. Whether the engine runs is a separate
// question (docs/architecture.md, Vehicle activity). Fault: a hardware or stack failure; it
// doesn't retry. A restart leaves the run altogether rather than being a
// state. Sleep (docs/architecture.md) comes with the power work.
enum class RunState : uint8_t { Starting, WaitingForVehicle, Polling, Fault };
enum class FaultReason : uint8_t { None, CanFailed, BleFailed };
enum class RestartReason : uint8_t { Requested };
// One past the last RestartReason; update it when adding one. Bounds the
// reason kept across a restart (core/ResetReason.h).
constexpr uint8_t kRestartReasonCount = static_cast<uint8_t>(RestartReason::Requested) + 1;

// What an event leads to. App carries it out: entering a state runs that
// state's fixed entry effects; Restart never returns.
struct Transition {
  enum class Kind : uint8_t { Stay, Enter, Restart };
  Kind kind;
  RunState to;        // Enter only
  RestartReason why;  // Restart only

  static Transition stay() { return {Kind::Stay, RunState::Starting, RestartReason::Requested}; }
  static Transition enter(RunState s) { return {Kind::Enter, s, RestartReason::Requested}; }
  static Transition restart(RestartReason r) { return {Kind::Restart, RunState::Starting, r}; }
};

// Which scheduler jobs run in a state.
struct JobSet {
  bool can;
  bool emit;
  bool power;
  bool ble;
  bool led;
  bool console;
};

inline JobSet jobsFor(RunState s) {
  switch (s) {
    case RunState::Starting:
      return {false, false, false, false, false, false};
    case RunState::WaitingForVehicle:
    case RunState::Polling:
      return {true, true, true, true, true, true};
    case RunState::Fault:
      // The supply keeps being watched; nothing else starts.
      return {false, false, true, false, true, true};
  }
  return {false, false, false, false, false, true};
}

// Run-state decisions only, no hardware: App applies each Transition. An
// event that doesn't apply in the current state returns Stay.
class RunStateMachine {
 public:
  Transition boot() { return enter(RunState::Starting); }

  // The result of bring-up: `failure` is the first thing that failed, or None.
  // With no vehicle found yet, the device waits for one.
  Transition started(FaultReason failure) {
    if (state_ != RunState::Starting) return Transition::stay();
    if (failure == FaultReason::None) return enter(RunState::WaitingForVehicle);
    fault_ = failure;
    return enter(RunState::Fault);
  }

  // An ECU answered. A car switching on is normal operation, so this needs no
  // restart.
  Transition vehicleFound() {
    if (state_ != RunState::WaitingForVehicle) return Transition::stay();
    return enter(RunState::Polling);
  }

  // The ECU stopped answering: the car switched off, most likely.
  Transition vehicleLost() {
    if (state_ != RunState::Polling) return Transition::stay();
    return enter(RunState::WaitingForVehicle);
  }

  Transition restartRequested() const { return Transition::restart(RestartReason::Requested); }

  RunState state() const { return state_; }
  FaultReason fault() const { return fault_; }

 private:
  Transition enter(RunState s) {
    state_ = s;
    return Transition::enter(s);
  }

  RunState state_ = RunState::Starting;
  FaultReason fault_ = FaultReason::None;
};

inline const char* stateName(RunState s) {
  switch (s) {
    case RunState::Starting:
      return "Starting";
    case RunState::WaitingForVehicle:
      return "Waiting for vehicle";
    case RunState::Polling:
      return "Polling";
    case RunState::Fault:
      return "Fault";
  }
  return "Unknown";
}

inline const char* faultMessage(FaultReason f) {
  switch (f) {
    case FaultReason::None:
      return "Fault: none";
    case FaultReason::CanFailed:
      return "Fault: the CAN controller failed to start \xE2\x9D\x8C";
    case FaultReason::BleFailed:
      return "Fault: BLE failed to start - nothing for a client to connect to \xE2\x9D\x8C";
  }
  return "Fault: unknown";
}

inline const char* restartReasonText(RestartReason r) {
  switch (r) {
    case RestartReason::Requested:
      return "Restart requested";
  }
  return "unknown";
}

// The Gnimu frame header's state code: 1 starting, 2 running, 3 waiting,
// 4 fault. The protocol's "running" is the same for every Gnimu device;
// Gnimu-CAN's Polling state sends it.
inline uint8_t frameStateCode(RunState s) {
  switch (s) {
    case RunState::Starting:
      return 1;
    case RunState::Polling:
      return 2;
    case RunState::WaitingForVehicle:
      return 3;
    case RunState::Fault:
      return 4;
  }
  return 0;
}

}  // namespace gnimu
