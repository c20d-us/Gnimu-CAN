// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdint.h>

namespace gnimu {

// What the supply voltage says about the car.
enum class SupplyLevel : uint8_t {
  Unknown,        // no reading yet
  Parked,         // below the engine-running threshold
  EngineRunning,  // the alternator is charging
  BelowCutoff,    // low enough to protect the car's battery
};

inline const char* supplyLevelName(SupplyLevel l) {
  switch (l) {
    case SupplyLevel::Unknown:
      return "unknown";
    case SupplyLevel::Parked:
      return "parked";
    case SupplyLevel::EngineRunning:
      return "engine running";
    case SupplyLevel::BelowCutoff:
      return "below cutoff";
  }
  return "unknown";
}

struct SupplyConfig {
  float divider;          // supply volts per volt at the ADC pin
  float calibration;      // measured: true supply / reported supply
  float emaAlpha;         // smoothing per sample, (0, 1]; 1 = none
  float engineRunningV;   // at or above: engine running
  float cutoffV;          // below: the low-voltage cutoff
  uint32_t runningHoldMs; // how long a level must hold before it counts
  uint32_t cutoffHoldMs;
};

struct SupplyStatus {
  bool valid;         // at least one reading taken
  float volts;        // filtered and calibrated
  SupplyLevel level;  // after the hold times
};

// Turns raw ADC pin readings into the supply voltage and what it means. A
// level only changes once the voltage has stayed on the new side of its
// threshold for the hold time, so a dip at cranking or a spike doesn't count.
class SupplyMonitor {
 public:
  explicit SupplyMonitor(const SupplyConfig& c) : c_(c) {}

  void addSample(uint32_t pinMv, uint32_t nowMs) {
    const float v = static_cast<float>(pinMv) / 1000.0f * c_.divider * c_.calibration;
    if (!status_.valid) {
      status_.valid = true;
      status_.volts = v;
    } else {
      status_.volts += c_.emaAlpha * (v - status_.volts);
    }

    const SupplyLevel raw = status_.volts < c_.cutoffV          ? SupplyLevel::BelowCutoff
                            : status_.volts >= c_.engineRunningV ? SupplyLevel::EngineRunning
                                                                 : SupplyLevel::Parked;
    if (status_.level == SupplyLevel::Unknown) {
      // The first reading decides at once: boot needs a level to start from.
      status_.level = raw;
      candidate_ = raw;
      return;
    }
    if (raw != candidate_) {
      candidate_ = raw;
      candidateSinceMs_ = nowMs;
    }
    const uint32_t hold = raw == SupplyLevel::BelowCutoff ? c_.cutoffHoldMs : c_.runningHoldMs;
    if (raw != status_.level && nowMs - candidateSinceMs_ >= hold) status_.level = raw;
  }

  const SupplyStatus& status() const { return status_; }

 private:
  SupplyConfig c_;
  SupplyStatus status_ = {false, 0.0f, SupplyLevel::Unknown};
  SupplyLevel candidate_ = SupplyLevel::Unknown;
  uint32_t candidateSinceMs_ = 0;
};

}  // namespace gnimu
