// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdint.h>

#include "RunStateMachine.h"

namespace gnimu {

// What the XIAO's single yellow LED shows. Patterns only; there's no color.
enum class LedState : uint8_t {
  Off,
  NoVehicle,    // waiting for the vehicle: a short flash every couple of seconds
  Advertising,  // polling, no client: slow blink
  Connected,    // polling, client connected: solid
  Fault,        // fast blink, for as long as the fault lasts
};

struct LedTiming {
  uint32_t blinkHalfMs;     // advertising: on, then off
  uint32_t faultHalfMs;     // fault: on, then off
  uint32_t flashOnMs;       // waiting for the vehicle: on for this long ...
  uint32_t flashPeriodMs;   // ... once per this period
};

inline LedState ledStateFor(RunState s, bool clientConnected) {
  switch (s) {
    case RunState::Starting:
      return LedState::Off;
    case RunState::WaitingForVehicle:
      return LedState::NoVehicle;
    case RunState::Polling:
      return clientConnected ? LedState::Connected : LedState::Advertising;
    case RunState::Fault:
      return LedState::Fault;
  }
  return LedState::Off;
}

// Every pattern restarts from "on" when its state is entered, so a change
// shows immediately.
class StatusLed {
 public:
  explicit StatusLed(const LedTiming& t) : t_(t) {}

  bool update(LedState s, uint32_t nowMs) {
    if (s != state_) {
      state_ = s;
      enteredMs_ = nowMs;
    }
    const uint32_t ms = nowMs - enteredMs_;
    switch (s) {
      case LedState::Off:
        return false;
      case LedState::NoVehicle:
        return ms % t_.flashPeriodMs < t_.flashOnMs;
      case LedState::Advertising:
        return (ms / t_.blinkHalfMs) % 2 == 0;
      case LedState::Connected:
        return true;
      case LedState::Fault:
        return (ms / t_.faultHalfMs) % 2 == 0;
    }
    return false;
  }

 private:
  LedTiming t_;
  LedState state_ = LedState::Off;
  uint32_t enteredMs_ = 0;
};

}  // namespace gnimu
