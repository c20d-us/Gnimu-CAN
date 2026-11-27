// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

// The BLE Console's replies (Gnimu protocol, docs/frame.md, Console): the
// echo line that starts every reply, "busy" naming the command it refuses
// (now, or once the reply going out has finished, the last of several), and
// every reply fitting its buffer with the echo, the full event log included.

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include <string>

#include "../code/GnimuCAN/src/app/SystemConfig.h"
#include "../code/GnimuCAN/src/core/ConsoleReply.h"
#include "../code/GnimuCAN/src/core/EventLog.h"
#include "../code/GnimuCAN/src/core/Report.h"

using namespace gnimu;

typedef char Echo[kConsoleEchoBytes + 1];
typedef ConsoleReply<config::kBleReplyBytes> Reply;

static std::string echoOf(const char* written) {
  Echo e;
  const size_t n = consoleEcho(e, written);
  return std::string(e, n);
}

// As App builds them: the echo line, the text, the end byte.
static std::string drain(Reply& r) {
  std::string out;
  while (r.sending()) {
    const char* data = nullptr;
    const size_t n = r.chunk(247, data);
    out.append(data, n);
    r.sent(n);
  }
  return out;
}

static std::string textReply(const char* written, const char* text) {
  static Reply r;
  Echo e;
  consoleEcho(e, written);
  r.begin();
  r.append(e, strlen(e));
  r.append(text, strlen(text));
  r.append("\n", 1);
  r.finish();
  return drain(r);
}

static void testEcho() {
  assert(echoOf("D") == "> D\n");
  assert(echoOf("e 20\r\n") == "> E 20\n");  // upper case, no line ending
  assert(echoOf("0") == "> 0\n");
  assert(echoOf("60\n") == "> 60\n");
  assert(echoOf("zz top") == "> ZZ TOP\n");  // unrecognised, echoed all the same
  assert(echoOf("\x1b[A") == "> ?[A\n");     // not printable: '?'
  assert(echoOf("\r\n").empty() && echoOf("\n").empty() && echoOf("").empty());  // no reply
  // Longer than 32 bytes: cut there, the written text not the parser's.
  const std::string longText(40, 'x');
  assert(echoOf(longText.c_str()) == "> " + std::string(32, 'X') + "\n");
}

// Every kind of reply starts with its echo line and ends with 0x04.
static void testEveryReplyEchoes() {
  const struct {
    const char* written;
    const char* text;
    const char* expected;
  } cases[] = {
      {"60", "status line once a second", "> 60\nstatus line once a second\n\x04"},
      {"0", "status line off", "> 0\nstatus line off\n\x04"},
      {"r", "not available over BLE without bonding", "> R\nnot available over BLE without bonding\n\x04"},
      {"99999", "number out of range", "> 99999\nnumber out of range\n\x04"},
      {"hello", "unrecognised command", "> HELLO\nunrecognised command\n\x04"},
      {"D", "busy", "> D\nbusy\n\x04"},
  };
  for (const auto& c : cases) assert(textReply(c.written, c.text) == c.expected);
  // A report: the echo, then the report text.
  static Reply r;
  Echo e;
  consoleEcho(e, "v");
  r.begin();
  r.append(e, strlen(e));
  const int n = snprintf(r.space(), r.room() + 1, "BUILD,version=0.1.0\n");
  r.commit(static_cast<size_t>(n));
  r.finish();
  assert(drain(r) == "> V\nBUILD,version=0.1.0\n\x04");
  // A live status line: "> n", the line.
  r.begin();
  r.append(kStatusLineEcho, strlen(kStatusLineEcho));
  r.append("RT: 1s | Polling\n", 17);
  r.finish();
  assert(drain(r) == "> n\nRT: 1s | Polling\n\x04");
}

static void testBusyNamesTheCommand() {
  ConsoleGate g(500);
  Echo e;
  assert(g.command("D", 1000, false, e) == ConsoleGate::Action::Run && std::string(e) == "> D\n");
  // Too soon: "busy" now, naming this command.
  assert(g.command("e 5", 1200, false, e) == ConsoleGate::Action::BusyNow && std::string(e) == "> E 5\n");
  // Half a second after the last that ran: runs.
  assert(g.command("V", 1500, false, e) == ConsoleGate::Action::Run);
  // While a reply goes out: refused afterwards; of several, only the last.
  assert(g.command("D", 3000, true, e) == ConsoleGate::Action::BusyLater);
  assert(g.command("p", 3010, true, e) == ConsoleGate::Action::BusyLater);
  assert(g.command("E 20", 3020, true, e) == ConsoleGate::Action::BusyLater);
  assert(g.takeBusy(e) && std::string(e) == "> E 20\n");
  assert(!g.takeBusy(e));  // once
  // Only CR or LF: nothing, not even busy.
  assert(g.command("\r\n", 4000, true, e) == ConsoleGate::Action::Ignore && !g.takeBusy(e));
  // Disconnecting forgets what's owed.
  g.command("D", 5000, true, e);
  g.cancel();
  assert(!g.takeBusy(e));
}

// The largest replies, with the echo: the event log full of the longest
// lines, and the worst-case D report.
static void testRepliesFitTheirBuffer() {
  static EventLog<config::kBootLogBytes, config::kEventLogBytes> log;
  char line[EventLog<1, 1>::kMaxLine];
  for (uint32_t i = 0; i < 2000; i++) {
    const int n = snprintf(line, sizeof(line), "%05u%0175d", i, 0);  // distinct, longer than kept
    log.add(4000000000u + i, line, static_cast<size_t>(n));
  }
  log.add(4000002000u, "repeat", 6);
  log.add(4000002001u, "repeat", 6);  // a pending "repeated" note too
  size_t eventBytes = 0;
  log.forEachLine([&](const char*, size_t n) { eventBytes += n + 1; });
  assert(eventBytes > config::kBootLogBytes + config::kEventLogBytes - 400);  // really full
  assert(kConsoleEchoBytes + eventBytes + 1 <= config::kBleReplyBytes);
  assert(kConsoleEchoBytes + kReportMaxBytes + 1 <= config::kBleReplyBytes);
}

int main() {
  testEcho();
  testEveryReplyEchoes();
  testBusyNamesTheCommand();
  testRepliesFitTheirBuffer();
  printf("test_bleconsole: ok\n");
  return 0;
}
