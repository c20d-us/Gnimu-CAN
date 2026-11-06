// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdint.h>

namespace gnimu {

// The BLE transport's counters, as the D report shows them.
struct BleStats {
  uint32_t sent = 0;       // frames the stack accepted
  uint32_t failed = 0;     // frames the stack rejected (notifications off, stack error)
  uint32_t refused = 0;    // frames not sent because the peer's MTU can't carry them
  uint32_t maxSendUs = 0;  // longest single send; C resets it
  uint16_t mtu = 0;        // the connected peer's MTU, 0 when not connected
};

}  // namespace gnimu
