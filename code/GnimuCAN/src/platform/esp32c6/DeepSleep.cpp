// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

// Two-layer guard: ARDUINO_ARCH_ESP32 is a compiler flag, safe before any
// include; CONFIG_IDF_TARGET_ESP32C6 only exists once sdkconfig.h is included.
#if defined(ARDUINO_ARCH_ESP32)
#include "sdkconfig.h"
#if CONFIG_IDF_TARGET_ESP32C6

#include <Arduino.h>
#include <esp_attr.h>
#include <esp_sleep.h>
#include <esp_system.h>

#include "../DeepSleep.h"

namespace gnimu {

namespace {

// Marks the record as ours: RTC memory holds garbage after power-on.
constexpr uint32_t kMagic = 0x476E5350;  // "GnSP"

struct Kept {
  uint32_t magic;
  SleepRecord record;
};

RTC_NOINIT_ATTR Kept kept;

bool keptValid() { return kept.magic == kMagic; }

}  // namespace

bool DeepSleep::takeRecord(SleepRecord& out) {
  if (esp_reset_reason() != ESP_RST_DEEPSLEEP) {
    // Power-on, or a reset deep sleep didn't cause: start counting afresh.
    kept.magic = 0;
    return false;
  }
  if (!keptValid()) return false;
  out = kept.record;
  return true;
}

void DeepSleep::sleepFor(const SleepRecord& r, uint32_t seconds) {
  kept.record = r;
  kept.magic = kMagic;
  esp_sleep_enable_timer_wakeup(static_cast<uint64_t>(seconds) * 1000000ULL);
  esp_deep_sleep_start();
  for (;;) {
  }
}

uint32_t DeepSleep::sleepCount() { return keptValid() ? kept.record.sleepCount : 0; }

}  // namespace gnimu

#endif  // CONFIG_IDF_TARGET_ESP32C6
#endif  // ARDUINO_ARCH_ESP32
