// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdint.h>

namespace gnimu {
namespace board {

// The Gnimu-CAN 12V carrier (XIAO ESP32C6 + Adafruit CAN Pal). The breadboard
// prototype is wired the same way.
constexpr char kHardwareName[] = "12v";
constexpr char kHardwareRevision[] = "XIAO ESP32C6, 12V carrier v1";

constexpr uint8_t kSupplySensePin = 0;  // D0/A0: V12_SENSE
constexpr uint8_t kCanSilentPin = 19;   // D8: CAN Pal S, high = listen-only (R1 pull-up)
constexpr uint8_t kCanTxPin = 20;       // D9
constexpr uint8_t kCanRxPin = 18;       // D10
constexpr uint8_t kStatusLedPin = 15;   // the XIAO's yellow user LED, active low

// The supply divider: 100 kΩ over 10 kΩ, so the supply is 11 times the pin.
constexpr float kSupplyDivider = 11.0f;

}  // namespace board
}  // namespace gnimu
