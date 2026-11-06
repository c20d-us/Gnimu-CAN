// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdint.h>

namespace gnimu {

// The supply-sense ADC input: the vehicle supply through the carrier's
// divider. Returns millivolts at the pin, using the chip's factory ADC
// calibration; core/SupplyMonitor scales it to the supply.
struct SupplyAdc {
  static void begin();
  static uint32_t readPinMv();
};

}  // namespace gnimu
