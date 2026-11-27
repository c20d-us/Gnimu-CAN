// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

// A BLE Console reply: text then the end byte 0x04, in chunks of at most
// MTU - 3 bytes, resent when the stack was busy, and dropped on disconnect.

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include <string>

#include "../code/GnimuCAN/src/core/ConsoleReply.h"

using namespace gnimu;

static std::string drain(ConsoleReply<1024>& r, uint16_t mtu) {
  std::string out;
  while (r.sending()) {
    const char* data = nullptr;
    const size_t n = r.chunk(mtu, data);
    assert(n > 0 && n <= static_cast<size_t>(mtu - 3));
    out.append(data, n);
    r.sent(n);
  }
  return out;
}

static void testChunksAndEnd() {
  ConsoleReply<1024> r;
  std::string text(500, 'a');
  r.begin();
  r.append(text.c_str(), text.size());
  assert(!r.sending());  // not until finished
  r.finish();
  const std::string out = drain(r, 247);
  assert(out.size() == 501 && out.back() == 0x04 && out.compare(0, 500, text) == 0);
}

static void testDefaultMtu() {
  ConsoleReply<1024> r;
  r.begin();
  r.append("hello world, this is longer than twenty bytes", 45);
  r.finish();
  int chunks = 0;
  while (r.sending()) {
    const char* data = nullptr;
    const size_t n = r.chunk(23, data);
    assert(n <= 20);
    r.sent(n);
    chunks++;
  }
  assert(chunks == 3);  // 46 bytes in 20-byte notifications
}

// Busy: the same chunk until it's sent.
static void testResendAfterBusy() {
  ConsoleReply<1024> r;
  r.begin();
  r.append("abcdef", 6);
  r.finish();
  const char* first = nullptr;
  const size_t n = r.chunk(8, first);  // 5 bytes
  const char* again = nullptr;
  assert(r.chunk(8, again) == n && again == first);  // not sent yet: the same
  r.sent(n);
  const char* next = nullptr;
  assert(r.chunk(8, next) == 2 && next[0] == 'f' && next[1] == 0x04);
}

static void testCancelAndFull() {
  ConsoleReply<16> r;
  r.begin();
  r.append("0123456789abcdefghij", 20);  // more than fits: cut, the end byte kept
  r.finish();
  ConsoleReply<16> copy = r;
  (void)copy;
  const char* data = nullptr;
  assert(r.chunk(247, data) == 16 && data[15] == 0x04);
  r.cancel();
  assert(!r.sending());
  // space/commit: text written in place.
  r.begin();
  const int n = snprintf(r.space(), r.room() + 1, "%s", "hi");
  r.commit(static_cast<size_t>(n));
  r.finish();
  assert(r.chunk(247, data) == 3 && memcmp(data, "hi\x04", 3) == 0);
}

int main() {
  testChunksAndEnd();
  testDefaultMtu();
  testResendAfterBusy();
  testCancelAndFull();
  printf("test_consolereply: ok\n");
  return 0;
}
