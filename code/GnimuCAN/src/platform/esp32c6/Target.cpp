// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

// Two-layer guard: ARDUINO_ARCH_ESP32 is a compiler flag, safe before any
// include; CONFIG_IDF_TARGET_ESP32C6 only exists once sdkconfig.h is included.
#if defined(ARDUINO_ARCH_ESP32)
#include "sdkconfig.h"
#if CONFIG_IDF_TARGET_ESP32C6

#include "../Target.h"

namespace gnimu {

const char* targetName() { return "esp32c6"; }

}  // namespace gnimu

#endif  // CONFIG_IDF_TARGET_ESP32C6
#endif  // ARDUINO_ARCH_ESP32
