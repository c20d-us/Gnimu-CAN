// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

// Two-layer guard: ARDUINO_ARCH_ESP32 is a compiler flag, safe before any
// include; CONFIG_IDF_TARGET_ESP32C6 only exists once sdkconfig.h is included.
#if defined(ARDUINO_ARCH_ESP32)
#include "sdkconfig.h"
#if CONFIG_IDF_TARGET_ESP32C6

#include <Arduino.h>

#include "../../app/SystemConfig.h"
#include "../ConsolePort.h"

namespace gnimu {

void consoleOpen() {
  // Serial is the C6's built-in USB serial port. With a cable attached but no
  // terminal reading, its buffer fills, and by default every write then waits
  // 100 ms for room, stalling the whole loop. With no timeout, output nobody
  // is reading is dropped instead (docs/architecture.md, Console).
  Serial.setTxTimeoutMs(0);
  Serial.begin(115200);
  // A host that's already attached shows up within a moment; without one,
  // boot carries on and its messages are simply not seen.
  const uint32_t start = millis();
  while (!Serial && millis() - start < config::kConsoleHostWaitMs) delay(10);
}

}  // namespace gnimu

#endif  // CONFIG_IDF_TARGET_ESP32C6
#endif  // ARDUINO_ARCH_ESP32
