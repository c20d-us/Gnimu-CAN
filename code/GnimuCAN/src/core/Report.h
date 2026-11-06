// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "BleStats.h"
#include "DeviceStatus.h"
#include "ResetReason.h"
#include "Scheduler.h"
#include "TextOut.h"

namespace gnimu {

// The scheduler's side of the D report: per-job stats and the loop count.
struct JobTable {
  const Job *jobs;
  const JobStats *stats;
  size_t count;
  uint32_t loops;
};

// The console's D-report buffer. test_report checks that the worst case
// (every counter at 10 digits) fits, so a report that grows fails a test
// rather than losing its last lines on the device.
constexpr size_t kReportMaxBytes = 2048;

// Text of the on-request diagnostics report, formatted into a caller buffer so
// it can be tested on a host. Output is truncated, never overrun.
inline size_t formatReport(char *buf, size_t cap, const JobTable &t, const DeviceStatus &d) {
  detail::Out o{buf, cap, 0};
  if (cap)
    buf[0] = '\0';

  // Column widths: at least wide enough for the header label, widened to fit
  // whatever value is actually present so the table stays aligned regardless
  // of job name length or how large a count has grown.
  size_t wJob = 3, wRuns = 4, wSkipped = 7, wOverruns = 8, wMaxUs = 6,
         wMeanUs = 7, wLateUs = 11;
  for (size_t i = 0; i < t.count; i++) {
    const JobStats &s = t.stats[i];
    const unsigned long meanUs = s.runs ? s.totalUs / s.runs : 0;
    const size_t nameLen = strlen(t.jobs[i].name);
    if (nameLen > wJob)
      wJob = nameLen;
    if (detail::decimalWidth(s.runs) > wRuns)
      wRuns = detail::decimalWidth(s.runs);
    if (detail::decimalWidth(s.skipped) > wSkipped)
      wSkipped = detail::decimalWidth(s.skipped);
    if (detail::decimalWidth(s.overruns) > wOverruns)
      wOverruns = detail::decimalWidth(s.overruns);
    if (detail::decimalWidth(s.maxUs) > wMaxUs)
      wMaxUs = detail::decimalWidth(s.maxUs);
    if (detail::decimalWidth(meanUs) > wMeanUs)
      wMeanUs = detail::decimalWidth(meanUs);
    if (detail::decimalWidth(s.maxLateUs) > wLateUs)
      wLateUs = detail::decimalWidth(s.maxLateUs);
  }
  // 7 columns, each framed as " | <cell>" plus the table's opening "| " and
  // closing " |": 2 + 2 + 6*3 = 22 characters of furniture around the widths.
  const size_t width =
      22 + wJob + wRuns + wSkipped + wOverruns + wMaxUs + wMeanUs + wLateUs;

  o.fill('=', width);
  o.put("\n| ");
  o.left("job", wJob);
  o.put(" | ");
  o.right("runs", wRuns);
  o.put(" | ");
  o.right("skipped", wSkipped);
  o.put(" | ");
  o.right("overruns", wOverruns);
  o.put(" | ");
  o.right("max μs", wMaxUs);
  o.put(" | ");
  o.right("mean μs", wMeanUs);
  o.put(" | ");
  o.right("max late μs", wLateUs);
  o.put(" |\n");
  o.fill('-', width);
  o.put("\n");

  for (size_t i = 0; i < t.count; i++) {
    const JobStats &s = t.stats[i];
    const unsigned long meanUs = s.runs ? s.totalUs / s.runs : 0;
    o.put("| ");
    o.left(t.jobs[i].name, wJob);
    o.put(" | ");
    o.right(static_cast<unsigned long>(s.runs), wRuns);
    o.put(" | ");
    o.right(static_cast<unsigned long>(s.skipped), wSkipped);
    o.put(" | ");
    o.right(static_cast<unsigned long>(s.overruns), wOverruns);
    o.put(" | ");
    o.right(static_cast<unsigned long>(s.maxUs), wMaxUs);
    o.put(" | ");
    o.right(meanUs, wMeanUs);
    o.put(" | ");
    o.right(static_cast<unsigned long>(s.maxLateUs), wLateUs);
    o.put(" |\n");
  }
  o.fill('=', width);
  o.put("\n");

  // RT and Loops split the bottom row in half, label left and value right in
  // each: "| " + left + " | " + right + " |" is 7 characters of furniture.
  // The narrowest table (68) leaves 30 per half, room for any 10-digit value.
  char rt[16];
  snprintf(rt, sizeof(rt), "%lus", static_cast<unsigned long>(d.uptimeUs / 1000000ULL));
  const size_t wLeft = (width - 7) / 2, wRight = width - 7 - wLeft;
  o.put("| RT:");
  o.right(rt, wLeft - 3);
  o.put(" | Loops:");
  o.right(static_cast<unsigned long>(t.loops), wRight - 6);
  o.put(" |\n");
  o.fill('=', width);
  o.put("\n");

  char reset[64];
  formatReset(reset, sizeof(reset), d.reset);
  o.put("Reset: %s\n", reset);
  o.put("State: %s\n", stateName(d.state));
  if (d.supply.valid) {
    const unsigned long centiV = static_cast<unsigned long>(d.supply.volts * 100.0f + 0.5f);
    o.put("Supply: %lu.%02luV, %s\n", centiV / 100, centiV % 100, supplyLevelName(d.supply.level));
  } else {
    o.put("Supply: no reading yet\n");
  }
  if (d.can.running) {
    o.put("CAN: %s at %lu kbit/s, TEC %lu, REC %lu, bus errors %lu, missed %lu%s\n",
          d.can.listenOnly ? "listening" : "active", static_cast<unsigned long>(d.can.bitrate / 1000),
          static_cast<unsigned long>(d.can.txErrors), static_cast<unsigned long>(d.can.rxErrors),
          static_cast<unsigned long>(d.can.busErrors), static_cast<unsigned long>(d.can.rxMissed),
          d.can.busOff ? ", BUS OFF" : "");
  } else {
    o.put("CAN: stopped\n");
  }
  if (d.canSummary.trafficBitrate) {
    o.put("CAN: traffic at %lu kbit/s, %lu frames\n", static_cast<unsigned long>(d.canSummary.trafficBitrate / 1000),
          static_cast<unsigned long>(d.canSummary.framesSeen));
  } else {
    o.put("CAN: no traffic heard, %lu frames\n", static_cast<unsigned long>(d.canSummary.framesSeen));
  }
  o.put("CAN: gateway probes %u of %u, last: %s\n", static_cast<unsigned>(d.canSummary.probes),
        static_cast<unsigned>(d.canSummary.maxProbes), probeResultText(d.canSummary.lastProbe));
  if (d.obd.ecuFound) {
    o.put("OBD: ECU 0x%03lX, requests to 0x%03lX\n", static_cast<unsigned long>(d.obd.responseId),
          static_cast<unsigned long>(d.obd.requestId));
  } else {
    o.put("OBD: no ECU\n");
  }
  for (size_t i = 0; i < d.obd.channelCount && i < kMaxObdChannels; i++) {
    const ObdChannelView &c = d.obd.channels[i];
    o.put("OBD: PID %02X %s: ", static_cast<unsigned>(c.pid), c.name);
    if (!d.obd.ecuFound) {
      o.put("not polling");
    } else if (!c.supported) {
      o.put("not supported");
    } else {
      if (c.fresh) {
        const long tenths = static_cast<long>(c.value * 10.0f + 0.5f);
        o.put("%ld.%ld %s", tenths / 10, tenths % 10, c.unit);
      } else {
        o.put("stale");
      }
      o.put(", requests %lu, replies %lu, timeouts %lu", static_cast<unsigned long>(c.requests),
            static_cast<unsigned long>(c.replies), static_cast<unsigned long>(c.timeouts));
    }
    o.put("\n");
  }
  o.put("Vehicle: %s\n", vehicleActivityName(d.obd.activity));
  o.put("BLE: %s, MTU %u, sent %lu, failed %lu, refused %lu, max send %lu\xCE\xBCs\n",
        d.ble.connected ? "connected" : "disconnected", static_cast<unsigned>(d.ble.stats.mtu),
        static_cast<unsigned long>(d.ble.stats.sent), static_cast<unsigned long>(d.ble.stats.failed),
        static_cast<unsigned long>(d.ble.stats.refused), static_cast<unsigned long>(d.ble.stats.maxSendUs));
  o.put("Console: dropped %lu bytes\n", static_cast<unsigned long>(d.consoleDropped));
  return o.len;
}

} // namespace gnimu
