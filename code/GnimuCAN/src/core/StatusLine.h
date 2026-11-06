// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "DeviceStatus.h"
#include "TextOut.h"

namespace gnimu {

// The periodic (1 Hz) status line: one terse row for watching the console
// scroll by, as opposed to the on-request D report's full table. Pipe-
// delimited "Label: value" cells.
// The status line's per-line rates: events since the previous line, as
// RateTracker measures them. Hz * 100, displayed rounded to a whole Hz.
struct StatusRates {
  uint32_t frameCentiHz;                     // frames sent over BLE
  uint32_t channelCentiHz[kMaxObdChannels];  // OBD replies, per polled channel
};

// The longest status line, newline included, plus its terminator.
// test_text checks the worst case fits.
constexpr size_t kStatusLineMaxBytes = 256;

inline size_t formatStatus(char *buf, size_t cap, const DeviceStatus &d, const StatusRates &r) {
  detail::Out o{buf, cap, 0};
  if (cap)
    buf[0] = '\0';
  const unsigned long uptimeS = static_cast<unsigned long>(d.uptimeUs / 1000000ULL);
  o.put("RT: %lus | %s | ", uptimeS, stateName(d.state));
  if (d.supply.valid) {
    const unsigned long centiV = static_cast<unsigned long>(d.supply.volts * 100.0f + 0.5f);
    o.put("Supply: %lu.%02luV %s | ", centiV / 100, centiV % 100, supplyLevelName(d.supply.level));
  } else {
    o.put("Supply: - | ");
  }
  if (d.canSummary.trafficBitrate) {
    o.put("CAN: %lu kbit/s | ", static_cast<unsigned long>(d.canSummary.trafficBitrate / 1000));
  } else {
    o.put("CAN: silent, probes %u/%u | ", static_cast<unsigned>(d.canSummary.probes),
          static_cast<unsigned>(d.canSummary.maxProbes));
  }
  if (d.obd.ecuFound) {
    o.put("OBD:");
    for (size_t i = 0; i < d.obd.channelCount && i < kMaxObdChannels; i++) {
      const ObdChannelView &c = d.obd.channels[i];
      if (!c.supported) continue;
      o.put(" %s ", c.name);
      if (c.fresh) {
        o.put("%ld", static_cast<long>(c.value + 0.5f));
      } else {
        o.put("-");
      }
      o.put(" %luHz", (static_cast<unsigned long>(r.channelCentiHz[i]) + 50) / 100);
    }
    o.put(" | ");
  }
  o.put("%s | ", vehicleActivityName(d.obd.activity));
  if (d.ble.connected) {
    o.put("BLE: %luHz\n", (static_cast<unsigned long>(r.frameCentiHz) + 50) / 100);
  } else {
    o.put("BLE: advertising\n");
  }
  return o.len;
}

}  // namespace gnimu
