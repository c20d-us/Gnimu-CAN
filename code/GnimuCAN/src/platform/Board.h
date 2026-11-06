// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The board's pins and wiring, for code that isn't itself per-board (App).
#if defined(ARDUINO_ARCH_ESP32)
#include "sdkconfig.h"
#if CONFIG_IDF_TARGET_ESP32C6
#include "esp32c6/BoardConfig.h"
#else
#error "Gnimu-CAN builds for the ESP32-C6 (Seeed XIAO ESP32C6) only"
#endif
#else
#error "Gnimu-CAN builds for the ESP32-C6 (Seeed XIAO ESP32C6) only"
#endif
