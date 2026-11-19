// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdint.h>

namespace gnimu {

// How long to wait before asking for an ECU again, on a bus with traffic
// where none has answered (docs/architecture.md, Bring-up). A car that has
// just switched on usually answers within the first few tries; after that,
// the device asks less and less often, so a bus that chatters without ever
// answering OBD (modules awake in a parked car, or a car that answers only
// on 29-bit IDs) sees a request every couple of minutes, not every few
// seconds. The count starts over when the bus goes quiet.
struct DiscoveryBackoff {
  uint32_t firstMs;     // between the first tries
  uint32_t thenMs;      // after that
  uint32_t maxMs;       // and from then on
  uint8_t triesPerStep; // tries at each interval before the next
};

// The wait after `failures` unanswered requests (1 or more).
inline uint32_t discoveryRetryMs(const DiscoveryBackoff& b, uint32_t failures) {
  if (failures <= b.triesPerStep) return b.firstMs;
  if (failures <= 2u * b.triesPerStep) return b.thenMs;
  return b.maxMs;
}

}  // namespace gnimu
