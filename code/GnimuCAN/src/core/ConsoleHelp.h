// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

namespace gnimu {

// The command help at the end of the boot log, as '\n'-separated lines, each
// within 73 columns. The boot log is also read over BLE (the E command),
// where only the report commands work, so the rest are marked USB only.
// `maxCount`: the most status lines a number may ask for. Truncated, never
// overrun; returns the length.
inline size_t formatConsoleHelp(char* buf, size_t cap, uint32_t maxCount) {
  const unsigned long m = maxCount;
  const int n = snprintf(buf, cap,
                         "Commands, each ended by Enter:\n"
                         "  D report, E event log (E n: the last n), V version, P protocol\n"
                         "  A number 0-%lu: that many status lines, one a second; 0 stops\n"
                         "  USB only: C clear counters, L status on/off (up to %lu lines),\n"
                         "  R restart, W stay awake on/off\n",
                         m, m);
  if (n < 0 || cap == 0) return 0;
  return static_cast<size_t>(n) < cap ? static_cast<size_t>(n) : cap - 1;
}

}  // namespace gnimu
