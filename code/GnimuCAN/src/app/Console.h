// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "../core/ConsoleEntry.h"
#include "../core/EventLog.h"
#include "../core/Report.h"
#include "../core/StatusLine.h"
#include "SystemConfig.h"

namespace gnimu {

// One poll() result: a command letter, a status count, or neither.
struct ConsoleCommand {
  CommandKind kind = CommandKind::None;
  bool hasCount = false;
  uint32_t count = 0;    // valid only when hasCount
  bool hasArg = false;   // the command's own number: "E 20"
  uint32_t arg = 0;
};

// Serial commands in, diagnostics report out. Nothing prints unless asked.
class Console {
 public:
  void begin();
  // Reads waiting input as entries ended by Enter: 'D' report, 'C' clear,
  // 'V' version, 'P' protocol, 'L' toggles status reporting (at most kStatusMaxCount
  // lines), 'R' restarts, or a number 0..kStatusMaxCount arms exactly that
  // many status reports. Anything else is rejected whole, with a message. Each call
  // handles at most one entry and reads at most the longest valid entry's
  // bytes, so a paste can't hold up the loop; the rest waits for later calls.
  ConsoleCommand poll();
  void printReport(const JobTable& jobs, const DeviceStatus& status);
  // Text of several '\n'-separated lines, sent a line at a time like a report.
  void print(const char* text, size_t n) { emit(text, n); }
  // Boot-time and exception messages; one line each.
  void log(const char* fmt, ...) __attribute__((format(printf, 2, 3)));
  // Lets queued output reach the host before a reset.
  void flush();
  // Bytes thrown away because the host stopped reading; never cleared.
  uint32_t dropped() const { return dropped_; }

  // The E command: the boot log and the recent events, or only the last
  // `lastN` events.
  void printEvents(size_t lastN);
  // The same text into `buf`, lines ended by '\n', for a BLE reply.
  // Truncated, never overrun; returns the length.
  size_t formatEvents(char* buf, size_t cap, size_t lastN) const;

  // Every log() line is kept here, so it can be read later.
  using Events = EventLog<config::kBootLogBytes, config::kEventLogBytes>;
  const Events& events() const { return events_; }

 private:
  void emit(const char* text, size_t n);
  void sendLine(const char* line, size_t len);

  ConsoleEntry<config::kStatusMaxCount> entry_;
  uint32_t dropped_ = 0;
  Events events_;
};

}  // namespace gnimu
