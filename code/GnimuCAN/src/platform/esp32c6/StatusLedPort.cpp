// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

// Two-layer guard: ARDUINO_ARCH_ESP32 is a compiler flag, safe before any
// include; CONFIG_IDF_TARGET_ESP32C6 only exists once sdkconfig.h is included.
#if defined(ARDUINO_ARCH_ESP32)
#include "sdkconfig.h"
#if CONFIG_IDF_TARGET_ESP32C6

#include <Arduino.h>

#include "../StatusLedPort.h"
#include "BoardConfig.h"

namespace gnimu {

// Active low: the LED lights when the pin is driven LOW.
void StatusLedPort::begin() {
  pinMode(board::kStatusLedPin, OUTPUT);
  digitalWrite(board::kStatusLedPin, HIGH);
}

void StatusLedPort::write(bool on) { digitalWrite(board::kStatusLedPin, on ? LOW : HIGH); }

}  // namespace gnimu

#endif  // CONFIG_IDF_TARGET_ESP32C6
#endif  // ARDUINO_ARCH_ESP32
