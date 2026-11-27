// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace gnimu {

// The schedule for multi-PID polling (docs/architecture.md, Polling): one
// request every cycle, and each channel in the requests of every `every`th
// cycle, starting at cycle `offset`. Channel i is in cycle k's request when
// k % every[i] == offset[i].

// How many cycles the builder looks at: the schedule repeats after the least
// common multiple of the cadences, and this caps it.
constexpr uint32_t kMaxScheduleCycles = 1000;

inline uint32_t greatestCommonDivisor(uint32_t a, uint32_t b) {
  while (b != 0) {
    const uint32_t t = a % b;
    a = b;
    b = t;
  }
  return a;
}

// The cycles after which the schedule of the `active` channels repeats,
// capped at kMaxScheduleCycles.
inline uint32_t scheduleCycles(const uint32_t* every, const bool* active, size_t n) {
  uint32_t cycles = 1;
  for (size_t i = 0; i < n; i++) {
    if (!active[i] || every[i] == 0) continue;
    const uint64_t l = static_cast<uint64_t>(cycles) / greatestCommonDivisor(cycles, every[i]) * every[i];
    if (l >= kMaxScheduleCycles) return kMaxScheduleCycles;
    cycles = static_cast<uint32_t>(l);
  }
  return cycles;
}

// Picks each active channel's offset so the requests stay as small as
// possible: fastest cadence first (table order among equals), each taking the
// offset whose cycles are least loaded so far, the earliest among equals.
// Deterministic: the same table always gives the same schedule. Returns the
// most channels any one cycle's request carries.
inline uint32_t buildSchedule(const uint32_t* every, const bool* active, size_t n, uint32_t* offset) {
  uint8_t load[kMaxScheduleCycles] = {};
  const uint32_t cycles = scheduleCycles(every, active, n);
  bool placed[64] = {};
  uint32_t busiest = 0;
  for (size_t i = 0; i < n; i++) offset[i] = 0;
  for (size_t round = 0; round < n && round < 64; round++) {
    // The next channel to place: the fastest cadence not yet placed.
    size_t pick = n;
    for (size_t i = 0; i < n && i < 64; i++) {
      if (!active[i] || every[i] == 0 || placed[i]) continue;
      if (pick == n || every[i] < every[pick]) pick = i;
    }
    if (pick == n) break;
    placed[pick] = true;
    const uint32_t e = every[pick];
    uint32_t best = 0;
    uint32_t bestLoad = UINT32_MAX;
    for (uint32_t o = 0; o < e && o < cycles; o++) {
      uint32_t most = 0;
      for (uint32_t k = o; k < cycles; k += e)
        if (load[k] > most) most = load[k];
      if (most < bestLoad) {
        bestLoad = most;
        best = o;
      }
    }
    offset[pick] = best;
    for (uint32_t k = best; k < cycles; k += e) {
      if (load[k] < UINT8_MAX) load[k]++;
      if (load[k] > busiest) busiest = load[k];
    }
  }
  return busiest;
}

}  // namespace gnimu
