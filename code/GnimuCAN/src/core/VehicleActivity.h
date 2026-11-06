// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdint.h>

#include "SupplyMonitor.h"

namespace gnimu {

// What the car is doing (docs/architecture.md, Vehicle activity), separate
// from whether the device can talk to it.
enum class VehicleActivity : uint8_t {
  Unknown,
  EngineRunning,
  IgnitionOn,  // an ECU answers, but the engine isn't turning
  Parked,
};

inline const char* vehicleActivityName(VehicleActivity a) {
  switch (a) {
    case VehicleActivity::Unknown:
      return "unknown";
    case VehicleActivity::EngineRunning:
      return "engine running";
    case VehicleActivity::IgnitionOn:
      return "ignition on";
    case VehicleActivity::Parked:
      return "parked";
  }
  return "unknown";
}

// RPM from the car overrides the supply voltage whenever it's fresh: cars
// with intelligent alternator control can run their engine at 12.4-12.8V,
// which voltage alone would call parked. Without fresh RPM, the supply level
// decides.
inline VehicleActivity vehicleActivity(bool rpmFresh, float rpm, float runningMinRpm, SupplyLevel supply) {
  if (rpmFresh) return rpm >= runningMinRpm ? VehicleActivity::EngineRunning : VehicleActivity::IgnitionOn;
  switch (supply) {
    case SupplyLevel::EngineRunning:
      return VehicleActivity::EngineRunning;
    case SupplyLevel::Parked:
    case SupplyLevel::BelowCutoff:
      return VehicleActivity::Parked;
    case SupplyLevel::Unknown:
      break;
  }
  return VehicleActivity::Unknown;
}

// Mode 01 decodes for the channels Gnimu-CAN polls, from the reply's A..D.
inline float decodeRpm(const uint8_t* d) { return ((d[0] << 8) | d[1]) / 4.0f; }
inline float decodePercent(const uint8_t* d) { return d[0] * 100.0f / 255.0f; }

}  // namespace gnimu
