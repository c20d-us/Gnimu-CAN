// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdint.h>

#include "../core/SleepPolicy.h"

namespace gnimu {

// What survives deep sleep: kept in RTC memory, which deep sleep retains but
// power loss doesn't.
struct SleepRecord {
  SleepKind kind;       // why the device is asleep
  float lowestV;        // the lowest wake-check reading since it went to sleep (kNoBaselineV: none yet)
  float sleptAtV;       // the supply when it went to sleep (the awake reading)
  float checkMinV;      // the lowest and highest a wake check has read,
  float checkMaxV;      // to compare with the awake reading
  uint32_t checks;      // wake checks since it went to sleep
  uint32_t sleepCount;  // times it has gone to sleep since power-up
};

// The ESP32-C6's deep sleep with a timer wake. Waking is a fresh boot.
struct DeepSleep {
  // The record from before this boot. False after power loss or any reset
  // other than a wake from deep sleep, when RTC memory holds nothing of ours.
  static bool takeRecord(SleepRecord& out);
  // Keeps `r` in RTC memory and sleeps for `seconds`. Never returns.
  [[noreturn]] static void sleepFor(const SleepRecord& r, uint32_t seconds);
  // Times the device has slept since power-up, kept across full boots after
  // a wake; 0 after power loss.
  static uint32_t sleepCount();
};

}  // namespace gnimu
