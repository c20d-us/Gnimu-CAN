// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

// The console's event log: the boot lines kept whole, a rolling area that
// drops its oldest lines, repeats collapsed to one line and a count, and the
// last n events on request.

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include <string>
#include <vector>

#include "../code/GnimuCAN/src/core/EventLog.h"

using namespace gnimu;

typedef EventLog<128, 256> Log;

static std::vector<std::string> lines(const Log& log, size_t lastN = 0) {
  std::vector<std::string> out;
  log.forEachLine([&](const char* t, size_t n) { out.push_back(std::string(t, n)); }, lastN);
  return out;
}

static void add(Log& log, uint32_t ms, const char* text) { log.add(ms, text, strlen(text)); }

// Distinct lines until the boot area is full (identical ones would collapse).
static void fillBoot(Log& log) {
  for (int i = 0; i < 10; i++) {
    char t[32];
    snprintf(t, sizeof(t), "boot line %d", i);
    add(log, i, t);
  }
}

static bool endsWith(const std::string& s, const char* tail) {
  const size_t n = strlen(tail);
  return s.size() >= n && s.compare(s.size() - n, n, tail) == 0;
}

static void testBootThenRolling() {
  Log log;
  add(log, 1234, "boot one");
  add(log, 1300, "boot two");
  std::vector<std::string> l = lines(log);
  assert(l.size() == 4 && l[0] == "Boot log:" && l[3] == "Events:");
  assert(l[1] == "         1.234  boot one");  // seconds and milliseconds since boot
  // The boot area holds 128 bytes; once a line doesn't fit, everything
  // after goes to the rolling area, and the boot lines stay.
  for (int i = 0; i < 6; i++) {
    char t[32];
    snprintf(t, sizeof(t), "line %d", i);
    add(log, 2000 + i, t);
  }
  l = lines(log);
  assert(l[1].find("boot one") != std::string::npos && l[2].find("boot two") != std::string::npos);
  bool sawEvents = false, sawLast = false;
  for (const std::string& s : l) {
    if (s == "Events:") sawEvents = true;
    if (sawEvents && endsWith(s, "line 5")) sawLast = true;
  }
  assert(sawLast);
}

static void testRollingDropsOldest() {
  Log log;
  fillBoot(log);
  for (int i = 0; i < 40; i++) {
    char t[32];
    snprintf(t, sizeof(t), "event %02d", i);
    add(log, 1000 + i, t);
  }
  const std::vector<std::string> l = lines(log);
  assert(endsWith(l[l.size() - 2], "event 39"));
  assert(endsWith(l.back(), "older lines dropped)"));
  assert(log.dropped() > 0);
  // The oldest events went first: event 00 isn't there any more.
  for (const std::string& s : l) assert(!endsWith(s, "event 00"));
}

static void testRepeatsCollapse() {
  Log log;
  fillBoot(log);
  add(log, 5000, "no replies");
  for (int i = 1; i <= 36; i++) add(log, 5000 + i * 2000, "no replies");
  std::vector<std::string> l = lines(log);
  // Stored once; the count shows while it's still repeating.
  assert(endsWith(l[l.size() - 2], "no replies"));
  assert(endsWith(l.back(), "(repeated 36 more times)"));
  // A different line closes it off.
  add(log, 80000, "found it");
  l = lines(log);
  size_t found = 0;
  for (size_t i = 0; i < l.size(); i++)
    if (endsWith(l[i], "found it")) found = i;
  assert(found > 0 && endsWith(l[found - 1], "(repeated 36 more times)"));
}

static void testLastN() {
  Log log;
  fillBoot(log);
  for (int i = 0; i < 5; i++) {
    char t[32];
    snprintf(t, sizeof(t), "event %d", i);
    add(log, 1000 + i, t);
  }
  const std::vector<std::string> l = lines(log, 2);
  assert(l.size() == 2 && endsWith(l[0], "event 3") && endsWith(l[1], "event 4"));
  assert(lines(log, 50).size() == log.rollingLines());  // more than there are: all
}

static void testLongLineCutShort() {
  Log log;
  std::string longText(400, 'x');
  log.add(1, longText.c_str(), longText.size());
  const std::vector<std::string> l = lines(log);
  assert(l[1].size() < Log::kMaxLine);
}

int main() {
  testBootThenRolling();
  testRollingDropsOldest();
  testRepeatsCollapse();
  testLastN();
  testLongLineCutShort();
  printf("test_eventlog: ok\n");
  return 0;
}
