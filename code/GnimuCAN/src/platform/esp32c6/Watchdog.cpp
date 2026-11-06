// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

// Two-layer guard: ARDUINO_ARCH_ESP32 is a compiler flag, safe before any
// include; CONFIG_IDF_TARGET_ESP32C6 only exists once sdkconfig.h is included.
#if defined(ARDUINO_ARCH_ESP32)
#include "sdkconfig.h"
#if CONFIG_IDF_TARGET_ESP32C6

#include <Arduino.h>
#include <esp_task_wdt.h>

#include "../../app/SystemConfig.h"
#include "../Watchdog.h"

namespace gnimu {

namespace {

// The same idle-task check the core configured. The C6 has one core.
#ifdef CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0
constexpr uint32_t kIdleCpu0 = 1u << 0;
#else
constexpr uint32_t kIdleCpu0 = 0;
#endif

}  // namespace

// The core starts the task watchdog but only has it watch the idle task; the
// loop task (where setup() runs too) isn't on it. Retimed to ours, keeping the
// panic, which prints the hung task's backtrace on the console before
// resetting.
bool Watchdog::start() {
  const esp_task_wdt_config_t cfg = {config::kWatchdogMs, kIdleCpu0, true};
  return esp_task_wdt_reconfigure(&cfg) == ESP_OK && esp_task_wdt_add(nullptr) == ESP_OK;
}

void Watchdog::feed() { (void)esp_task_wdt_reset(); }

}  // namespace gnimu

#endif  // CONFIG_IDF_TARGET_ESP32C6
#endif  // ARDUINO_ARCH_ESP32
