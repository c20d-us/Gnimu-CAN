// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdint.h>

namespace gnimu {

// Implemented in platform/esp32c6/Clock.cpp.
struct PlatformClock {
  // Starts the timebase; call once before the first nowUs().
  static void begin();
  // Monotonic microseconds since boot, 64-bit.
  static uint64_t nowUs();
  // Sleep until the deadline without busy-waiting. Granularity is one RTOS
  // tick (1 ms), and it sleeps at least one tick.
  static void sleepUntilUs(uint64_t deadlineUs);
};

}  // namespace gnimu
