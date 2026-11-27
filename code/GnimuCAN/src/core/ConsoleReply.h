// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace gnimu {

// The longest command a Console reply echoes.
constexpr size_t kConsoleEchoMax = 32;
// Room for an echo line: "> ", the command, "\n".
constexpr size_t kConsoleEchoBytes = kConsoleEchoMax + 3;

// The first line of a Console reply, naming what it answers (Gnimu protocol,
// docs/frame.md, Console): "> " and the command as written, upper-cased,
// without trailing CR or LF, cut to kConsoleEchoMax bytes, any byte that
// isn't printable ASCII shown as '?', then "\n". `written` is NUL-terminated.
// Returns the length, or 0 when the command is empty once CR and LF are gone
// (a write that gets no reply).
inline size_t consoleEcho(char (&out)[kConsoleEchoBytes + 1], const char* written) {
  size_t n = strlen(written);
  while (n > 0 && (written[n - 1] == '\r' || written[n - 1] == '\n')) n--;
  if (n > kConsoleEchoMax) n = kConsoleEchoMax;
  // Only CR and LF anywhere: nothing to answer.
  bool any = false;
  for (size_t i = 0; i < n; i++) any = any || (written[i] != '\r' && written[i] != '\n');
  if (!any) {
    out[0] = '\0';
    return 0;
  }
  out[0] = '>';
  out[1] = ' ';
  for (size_t i = 0; i < n; i++) {
    char c = written[i];
    if (c >= 'a' && c <= 'z') c = static_cast<char>(c - 'a' + 'A');
    out[2 + i] = (c < 0x20 || c > 0x7E) ? '?' : c;
  }
  out[2 + n] = '\n';
  out[3 + n] = '\0';
  return n + 3;
}

// The echo line of a live status line (Gnimu protocol, Live status).
constexpr char kStatusLineEcho[] = "> n\n";

// One Console reply on its way to a BLE client (Gnimu protocol, docs/frame.md,
// Console): text, then the end byte 0x04, sent a notification at a time.
// Pure logic: the App fills it, asks for each chunk, and says when one went.
template <size_t Cap>
class ConsoleReply {
 public:
  static constexpr char kEnd = 0x04;

  // A new reply. Whatever was still unsent is dropped.
  void begin() {
    len_ = 0;
    sent_ = 0;
    open_ = true;
  }

  // Where the reply's text goes, and how much room is left (keeping one byte
  // for the end marker). Text written there counts once commit() says so.
  char* space() { return buf_ + len_; }
  size_t room() const { return Cap - 1 - len_; }
  void commit(size_t n) { len_ += n <= room() ? n : room(); }

  void append(const char* text, size_t n) {
    if (n > room()) n = room();
    memcpy(buf_ + len_, text, n);
    len_ += n;
  }

  // The reply is complete: the end marker goes after the text.
  void finish() {
    if (!open_) return;
    buf_[len_++] = kEnd;
    open_ = false;
  }

  // A finished reply still has bytes to send.
  bool sending() const { return !open_ && sent_ < len_; }

  // The next chunk for a peer with ATT MTU `mtu`: at most mtu - 3 bytes.
  size_t chunk(uint16_t mtu, const char*& data) const {
    data = buf_ + sent_;
    const size_t max = mtu > 3 ? mtu - 3u : 1u;
    const size_t left = len_ - sent_;
    return left < max ? left : max;
  }
  // The chunk just asked for went out.
  void sent(size_t n) { sent_ = sent_ + n <= len_ ? sent_ + n : len_; }

  // The client left: nothing more to send.
  void cancel() {
    len_ = sent_ = 0;
    open_ = false;
  }

 private:
  char buf_[Cap] = {};
  size_t len_ = 0;
  size_t sent_ = 0;
  bool open_ = false;
};

// Whether a BLE Console command runs now, or is refused "busy" (Gnimu
// protocol, Console: one reply at a time, commands at least `gapMs` apart).
// A command that arrives while a reply is going out is refused once it has
// finished, naming that command; of several, only the last. Pure logic.
class ConsoleGate {
 public:
  enum class Action : uint8_t {
    Ignore,    // only CR or LF: no reply
    Run,       // answer it now
    BusyNow,   // too soon after the last: "busy" now
    BusyLater  // a reply is going out: "busy" once it has (takeBusy)
  };

  explicit ConsoleGate(uint32_t gapMs) : gapMs_(gapMs) {}

  // A command as written; `echo` gets its echo line. `sending`: a reply is
  // still going out.
  Action command(const char* written, uint32_t nowMs, bool sending, char (&echo)[kConsoleEchoBytes + 1]) {
    if (consoleEcho(echo, written) == 0) return Action::Ignore;
    if (sending) {
      memcpy(busyEcho_, echo, sizeof(echo));
      owed_ = true;
      return Action::BusyLater;
    }
    if (seen_ && nowMs - lastMs_ < gapMs_) return Action::BusyNow;
    seen_ = true;
    lastMs_ = nowMs;
    return Action::Run;
  }

  // Once nothing is going out: the echo line of a command still owed "busy".
  bool takeBusy(char (&echo)[kConsoleEchoBytes + 1]) {
    if (!owed_) return false;
    owed_ = false;
    memcpy(echo, busyEcho_, sizeof(echo));
    return true;
  }

  // The client left: nothing is owed.
  void cancel() { owed_ = false; }

 private:
  uint32_t gapMs_;
  uint32_t lastMs_ = 0;
  bool seen_ = false;
  bool owed_ = false;
  char busyEcho_[kConsoleEchoBytes + 1] = {};
};

}  // namespace gnimu
