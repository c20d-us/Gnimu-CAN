// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include <initializer_list>

#include "../code/GnimuCAN/src/core/ConsoleEntry.h"

using namespace gnimu;

using Entry = ConsoleEntry<3600>;
static_assert(Entry::kCapacity == 6, "\"E \" and 3600's four digits");
static_assert(ConsoleEntry<300>::kCapacity == 5, "capacity follows the cap");
static_assert(ConsoleEntry<99999>::kCapacity == 7, "capacity follows the cap");

// Feeds every byte; returns the last non-None result (None if there was none).
static EntryResult feedAll(Entry& e, const char* s) {
  EntryResult last;
  for (; *s; s++) {
    const EntryResult r = e.feed(*s);
    if (r.kind != EntryKind::None) last = r;
  }
  return last;
}

static void testCommands() {
  Entry e;
  struct Case {
    const char* text;
    CommandKind kind;
  };
  const Case cases[] = {{"D\n", CommandKind::Report},
                        {"C\r", CommandKind::Clear},
                        {"V\r\n", CommandKind::Version},
                        {"P\n", CommandKind::Protocol},
                        {"L\n", CommandKind::ToggleStatus},
                        {"R\r", CommandKind::Restart},
                        {"W\n", CommandKind::ToggleStayAwake},
                        {"E\n", CommandKind::Events},
                        {"d\n", CommandKind::Report},  // lower case too
                        {"e\r\n", CommandKind::Events}};
  for (const Case& c : cases) {
    const EntryResult r = feedAll(e, c.text);
    assert(r.kind == EntryKind::Command && r.cmd == c.kind);
  }
}

static void testCounts() {
  Entry e;
  EntryResult r = feedAll(e, "3600\r\n");
  assert(r.kind == EntryKind::Count && r.count == 3600);
  r = feedAll(e, "0\n");
  assert(r.kind == EntryKind::Count && r.count == 0);
  r = feedAll(e, "007\n");
  assert(r.kind == EntryKind::Count && r.count == 7);
}

static void testTooLarge() {
  Entry e;
  assert(feedAll(e, "3601\n").kind == EntryKind::TooLarge);
  // More digits than fit is also just too large, not unrecognised.
  assert(feedAll(e, "123456789012\n").kind == EntryKind::TooLarge);
}

static void testUnrecognisedIsRejectedWhole() {
  Entry e;
  for (const char* s : {"Love\n", "DVL\n", "1g6\n", "1g6t\n", "x\n", " D\n", "D 5\n", "E x\n", "E 1x\n"}) {
    assert(feedAll(e, s).kind == EntryKind::Unrecognised);
  }
  feedAll(e, "1g6\n");
  assert(strcmp(e.text(), "1G6") == 0 && !e.truncated());  // letters shown in upper case
}

static void testLongEntryEchoesWhatFit() {
  Entry e;
  assert(feedAll(e, "Lovelier\n").kind == EntryKind::Unrecognised);
  assert(strcmp(e.text(), "LOVELI") == 0 && e.truncated());
}

static void testNonPrintableShownAsQuestionMark() {
  Entry e;
  assert(feedAll(e, "\x1b[A\n").kind == EntryKind::Unrecognised);
  assert(strcmp(e.text(), "?[A") == 0);
}

// CR+LF ends one entry, then an empty one; empty entries are None.
static void testEmptyEntriesAreNone() {
  Entry e;
  assert(e.feed('\r').kind == EntryKind::None);
  assert(e.feed('\n').kind == EntryKind::None);
  assert(e.feed('D').kind == EntryKind::None);  // not ended yet
  assert(e.feed('\r').kind == EntryKind::Command);
  assert(e.feed('\n').kind == EntryKind::None);
}

static void testResetAbandonsEntry() {
  Entry e;
  feedAll(e, "12");
  e.reset();
  const EntryResult r = feedAll(e, "3\n");
  assert(r.kind == EntryKind::Count && r.count == 3);
}

// E with a count: the last n events.
static void testEventsWithCount() {
  Entry e;
  EntryResult r = feedAll(e, "E 20\n");
  assert(r.kind == EntryKind::Command && r.cmd == CommandKind::Events && r.hasArg && r.arg == 20);
  r = feedAll(e, "e 3600\n");
  assert(r.kind == EntryKind::Command && r.hasArg && r.arg == 3600);
  r = feedAll(e, "E\n");
  assert(r.kind == EntryKind::Command && !r.hasArg);
  assert(feedAll(e, "E 0\n").kind == EntryKind::TooLarge);
  assert(feedAll(e, "E 3601\n").kind == EntryKind::TooLarge);
}

// Only commands that report may come from an unbonded BLE client.
static void testReportOnly() {
  assert(reportOnly(CommandKind::Report) && reportOnly(CommandKind::Version));
  assert(reportOnly(CommandKind::Protocol) && reportOnly(CommandKind::Events));
  assert(!reportOnly(CommandKind::Clear) && !reportOnly(CommandKind::Restart));
  assert(!reportOnly(CommandKind::ToggleStayAwake) && !reportOnly(CommandKind::ToggleStatus));
  assert(!reportOnly(CommandKind::None));
}

int main() {
  testCommands();
  testEventsWithCount();
  testReportOnly();
  testCounts();
  testTooLarge();
  testUnrecognisedIsRejectedWhole();
  testLongEntryEchoesWhatFit();
  testNonPrintableShownAsQuestionMark();
  testEmptyEntriesAreNone();
  testResetAbandonsEntry();
  printf("consoleentry: all tests passed\n");
  return 0;
}
