// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

namespace gnimu {

// The console's log lines, kept so they can be read later (the E command,
// over USB or BLE): the first BootBytes of lines after a boot, kept whole,
// then a rolling RollBytes of the most recent, oldest dropped first. Each line
// is stamped with the uptime it was logged at. A line repeated straight after
// itself is kept once, with a count. Pure logic, called only from the loop.
template <size_t BootBytes, size_t RollBytes>
class EventLog {
 public:
  // The longest line kept; longer ones are cut short.
  static constexpr size_t kMaxLine = 192;

  // One console line, logged `ms` after boot.
  void add(uint32_t ms, const char* text, size_t len) {
    if (len > kMaxLine - 24) len = kMaxLine - 24;
    if (len == lastLen_ && len > 0 && memcmp(text, last_, len) == 0) {
      repeats_++;
      lastRepeatMs_ = ms;
      return;
    }
    flushRepeats();
    memcpy(last_, text, len);
    lastLen_ = len;
    char line[kMaxLine];
    const size_t n = stamp(line, ms, text, len);
    store(line, n);
  }

  // Calls `visit(text, len)` for each line, without its '\n', oldest first:
  // the boot lines, a separator, the rolling lines (only the last `lastN`
  // when it's non-zero), a pending repeat count, and how many older lines
  // were dropped.
  template <typename Visit>
  void forEachLine(Visit visit, size_t lastN = 0) const {
    if (lastN == 0) {
      visit("Boot log:", 9);
      eachIn(boot_, 0, bootLen_, BootBytes, visit, 0);
      visit("Events:", 7);
    }
    const size_t skip = lastN != 0 && lastN < rollLines_ ? rollLines_ - lastN : 0;
    eachIn(roll_, rollStart_, rollLen_, RollBytes, visit, skip);
    if (repeats_ > 0) {
      char note[64];
      const int n = snprintf(note, sizeof(note), "%10lu.%03lu  (repeated %lu more times)",
                             static_cast<unsigned long>(lastRepeatMs_ / 1000),
                             static_cast<unsigned long>(lastRepeatMs_ % 1000), static_cast<unsigned long>(repeats_));
      visit(note, n > 0 ? static_cast<size_t>(n) : 0);
    }
    if (dropped_ > 0 && lastN == 0) {
      char note[48];
      const int n = snprintf(note, sizeof(note), "(%lu older lines dropped)", static_cast<unsigned long>(dropped_));
      visit(note, n > 0 ? static_cast<size_t>(n) : 0);
    }
  }

  size_t bootBytes() const { return bootLen_; }
  size_t rollingLines() const { return rollLines_; }
  uint32_t dropped() const { return dropped_; }

 private:
  static size_t stamp(char* out, uint32_t ms, const char* text, size_t len) {
    const int n = snprintf(out, kMaxLine, "%10lu.%03lu  ", static_cast<unsigned long>(ms / 1000),
                           static_cast<unsigned long>(ms % 1000));
    size_t at = n > 0 ? static_cast<size_t>(n) : 0;
    if (at + len + 1 > kMaxLine) len = kMaxLine - at - 1;
    memcpy(out + at, text, len);
    at += len;
    out[at++] = '\n';
    return at;
  }

  void flushRepeats() {
    if (repeats_ == 0) return;
    char line[kMaxLine];
    char text[48];
    const int t = snprintf(text, sizeof(text), "(repeated %lu more times)", static_cast<unsigned long>(repeats_));
    repeats_ = 0;
    store(line, stamp(line, lastRepeatMs_, text, t > 0 ? static_cast<size_t>(t) : 0));
  }

  // A stamped line, ending in '\n': into the boot area while there's room
  // and nothing has gone to the rolling area yet, else the rolling area.
  void store(const char* line, size_t n) {
    if (!bootFull_ && bootLen_ + n <= BootBytes) {
      memcpy(boot_ + bootLen_, line, n);
      bootLen_ += n;
      return;
    }
    bootFull_ = true;
    if (n > RollBytes) return;
    while (RollBytes - rollLen_ < n) dropOldest();
    for (size_t i = 0; i < n; i++) roll_[(rollStart_ + rollLen_ + i) % RollBytes] = line[i];
    rollLen_ += n;
    rollLines_++;
  }

  void dropOldest() {
    while (rollLen_ > 0) {
      const char c = roll_[rollStart_];
      rollStart_ = (rollStart_ + 1) % RollBytes;
      rollLen_--;
      if (c == '\n') break;
    }
    rollLines_--;
    dropped_++;
  }

  template <typename Visit>
  static void eachIn(const char* buf, size_t start, size_t len, size_t cap, Visit& visit, size_t skip) {
    char line[kMaxLine];
    size_t n = 0;
    for (size_t i = 0; i < len; i++) {
      const char c = buf[(start + i) % cap];
      if (c != '\n') {
        if (n < kMaxLine) line[n++] = c;
        continue;
      }
      if (skip > 0) {
        skip--;
      } else {
        visit(line, n);
      }
      n = 0;
    }
  }

  char boot_[BootBytes] = {};
  size_t bootLen_ = 0;
  bool bootFull_ = false;
  char roll_[RollBytes] = {};
  size_t rollStart_ = 0;
  size_t rollLen_ = 0;
  size_t rollLines_ = 0;
  uint32_t dropped_ = 0;
  char last_[kMaxLine] = {};
  size_t lastLen_ = 0;
  uint32_t repeats_ = 0;
  uint32_t lastRepeatMs_ = 0;
};

}  // namespace gnimu
