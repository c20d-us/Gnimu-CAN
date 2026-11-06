// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdint.h>

namespace gnimu {

// How a gateway probe went (docs/architecture.md, Bring-up).
enum class ProbeResult : uint8_t {
  None,      // no probe yet
  Answered,  // an ECU replied: the bus is up at the probe rate
  NoAck,     // nothing acknowledged the frame: no node, or the wrong bit rate
  NoReply,   // acknowledged, but no ECU answered in time
  Error,     // the controller couldn't send at all
};

inline const char* probeResultText(ProbeResult r) {
  switch (r) {
    case ProbeResult::None:
      return "none";
    case ProbeResult::Answered:
      return "an ECU answered";
    case ProbeResult::NoAck:
      return "no acknowledgement";
    case ProbeResult::NoReply:
      return "acknowledged, no reply";
    case ProbeResult::Error:
      return "controller error";
  }
  return "unknown";
}

struct ProbeConfig {
  float minV;           // supply at or above this ...
  uint32_t holdMs;      // ... for this long, with the bus silent, allows a probe
  uint32_t backoffMs;   // between tries
  uint8_t maxAttempts;  // per boot
};

// When to make a gateway probe: a silent bus with the supply high enough that
// the car may be on behind a gateway. At most maxAttempts single frames per
// boot, backoffMs apart, so a car that never answers sees very little.
class ProbePolicy {
 public:
  explicit ProbePolicy(const ProbeConfig& c) : c_(c) {}

  // Every supply reading.
  void supply(bool valid, float volts, uint32_t nowMs) {
    const bool high = valid && volts >= c_.minV;
    if (high && !high_) highSinceMs_ = nowMs;
    high_ = high;
  }

  // True when a probe should go now.
  bool due(bool busSilent, uint32_t nowMs) const {
    if (!busSilent || !high_ || attempts_ >= c_.maxAttempts) return false;
    if (nowMs - highSinceMs_ < c_.holdMs) return false;
    return attempts_ == 0 || nowMs - lastMs_ >= c_.backoffMs;
  }

  void record(ProbeResult r, uint32_t nowMs) {
    attempts_++;
    lastMs_ = nowMs;
    last_ = r;
  }

  // A vehicle was found and has since gone quiet: the car may come back later
  // in this boot, so the probe budget starts over.
  void reset() {
    attempts_ = 0;
    last_ = ProbeResult::None;
  }

  uint8_t attempts() const { return attempts_; }
  bool exhausted() const { return attempts_ >= c_.maxAttempts; }
  ProbeResult last() const { return last_; }
  const ProbeConfig& config() const { return c_; }

 private:
  ProbeConfig c_;
  bool high_ = false;
  uint32_t highSinceMs_ = 0;
  uint8_t attempts_ = 0;
  uint32_t lastMs_ = 0;
  ProbeResult last_ = ProbeResult::None;
};

}  // namespace gnimu
