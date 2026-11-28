// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

// OBD discovery: every reply to one request is collected for P2, then the ECU
// to poll is chosen (0x7E8 first, then the most capable ECU supporting a
// wanted PID), and a round with nothing chosen backs off: every 5 s at
// first, then once a minute.

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "../code/GnimuCAN/src/core/ObdDiscovery.h"

using namespace gnimu;

// Every 5 s for 3 tries, then every 60 s (shortened from the firmware's 60
// fast tries).
static const ObdDiscoveryConfig kConfig = {50, {5000, 3, 60000}};
static const uint8_t kWanted[] = {0x0C, 0x11};  // RPM, then throttle

// PID 0x00 support bitmaps (top bit PID 0x01).
static uint32_t bit(uint8_t pid) { return 1u << (32 - pid); }
static const uint32_t kEngine = bit(0x05) | bit(0x0C) | bit(0x0D) | bit(0x0F) | bit(0x11);
static const uint32_t kTransmission = bit(0x01) | bit(0x0D);  // no RPM, no throttle

static EcuChoice discoveryRound(ObdDiscovery& d, uint32_t at, void (*replies)(ObdDiscovery&)) {
  assert(d.due(at));
  d.begin(at);
  replies(d);
  assert(!d.collected(at + 49));
  assert(d.collected(at + 50));
  return d.finish(kWanted, 2, at + 50);
}

static void testTransmissionFirstStillPicksTheEngine() {
  ObdDiscovery d(kConfig);
  d.restart(0);
  const EcuChoice c = discoveryRound(d, 0, [](ObdDiscovery& x) {
    x.reply(1, kTransmission);  // 0x7E9 answers first
    x.reply(0, kEngine);
  });
  assert(c.chosen && c.ecu == 0 && c.bitmap == kEngine);
  assert(c.answered == 0x03);
  // Every answering ECU's bitmap stays readable, for the log.
  assert(d.bitmap(0) == kEngine && d.bitmap(1) == kTransmission && d.bitmap(2) == 0 && d.bitmap(9) == 0);
}

static void testOnlyAnotherEcuHasRpm() {
  ObdDiscovery d(kConfig);
  d.restart(0);
  const EcuChoice c = discoveryRound(d, 0, [](ObdDiscovery& x) {
    x.reply(0, kTransmission);  // 0x7E8 without anything wanted
    x.reply(2, bit(0x0C));
  });
  assert(c.chosen && c.ecu == 2);
}

static void testMostCapableWinsAmongRpmEcus() {
  ObdDiscovery d(kConfig);
  d.restart(0);
  const EcuChoice c = discoveryRound(d, 0, [](ObdDiscovery& x) {
    x.reply(3, bit(0x0C));
    x.reply(1, bit(0x0C) | bit(0x05) | bit(0x0D));
  });
  assert(c.chosen && c.ecu == 1);
}

static void testThrottleOnlyIsStillPolled() {
  ObdDiscovery d(kConfig);
  d.restart(0);
  const EcuChoice c = discoveryRound(d, 0, [](ObdDiscovery& x) { x.reply(1, bit(0x11)); });
  assert(c.chosen && c.ecu == 1);
}

static void testSingleEngineReply() {
  ObdDiscovery d(kConfig);
  d.restart(0);
  const EcuChoice c = discoveryRound(d, 0, [](ObdDiscovery& x) { x.reply(0, kEngine); });
  assert(c.chosen && c.ecu == 0 && c.answered == 0x01 && d.failures() == 0);
}

// Nothing pollable: ask again after the backoff, not never.
static void testNothingToPollBacksOffAndAsksAgain() {
  ObdDiscovery d(kConfig);
  d.restart(0);
  EcuChoice c = discoveryRound(d, 0, [](ObdDiscovery& x) { x.reply(1, kTransmission); });
  assert(!c.chosen && c.answered == 0x02);
  assert(d.failures() == 1 && d.retryInMs() == 5000);
  assert(!d.due(5049) && d.due(5050));
  c = discoveryRound(d, 5050, [](ObdDiscovery& x) {
    x.reply(1, kTransmission);
    x.reply(0, kEngine);
  });
  assert(c.chosen && c.ecu == 0 && d.failures() == 0);
}

static void testNoReplyBacksOffLikeBefore() {
  ObdDiscovery d(kConfig);
  d.restart(1000);
  uint32_t t = 1000;
  const uint32_t waits[] = {5000, 5000, 5000, 60000, 60000, 60000, 60000, 60000};
  for (uint32_t w : waits) {
    const EcuChoice c = discoveryRound(d, t, [](ObdDiscovery&) {});
    assert(!c.chosen && c.answered == 0 && d.retryInMs() == w);
    t += 50 + w;
  }
  assert(d.failures() == 8);
  d.restart(t + 7);  // traffic confirmed again: from the start
  assert(d.due(t + 7) && d.failures() == 0);
}

static void testRepliesOutsideTheRoundAreIgnored() {
  ObdDiscovery d(kConfig);
  d.restart(0);
  d.reply(0, kEngine);  // before begin(): ignored
  const EcuChoice c = discoveryRound(d, 0, [](ObdDiscovery& x) { x.reply(9, kEngine); });  // out of range
  assert(!c.chosen && c.answered == 0);
  assert(!d.collecting() && !d.due(10));
}

static void testResponseIdText() {
  char buf[obd2::kResponseIdsTextMax];
  obd2::formatResponseIds(buf, sizeof(buf), 0x03);
  assert(strcmp(buf, "0x7E8, 0x7E9") == 0);
  obd2::formatResponseIds(buf, sizeof(buf), 0xFF);
  assert(strlen(buf) == sizeof(buf) - 3);  // eight IDs fit
  obd2::formatResponseIds(buf, sizeof(buf), 0);
  assert(buf[0] == '\0');
}

// Discovery on a bus nobody answers: every 5 s for 5 minutes, then once a
// minute.
static void testDiscoveryBackoff() {
  const DiscoveryBackoff b = {5000, 60, 60000};
  assert(discoveryRetryMs(b, 1) == 5000 && discoveryRetryMs(b, 60) == 5000);
  assert(discoveryRetryMs(b, 61) == 60000 && discoveryRetryMs(b, 1000000) == 60000);
  // An hour of a bus that never answers: 60 in the first 5 minutes, then 55
  // more, not 720.
  uint32_t t = 0, n = 0;
  while (t < 3600000) t += discoveryRetryMs(b, ++n);
  assert(n == 115);
}

int main() {
  testDiscoveryBackoff();
  testTransmissionFirstStillPicksTheEngine();
  testOnlyAnotherEcuHasRpm();
  testMostCapableWinsAmongRpmEcus();
  testThrottleOnlyIsStillPolled();
  testSingleEngineReply();
  testNothingToPollBacksOffAndAsksAgain();
  testNoReplyBacksOffLikeBefore();
  testRepliesOutsideTheRoundAreIgnored();
  testResponseIdText();
  printf("test_obddiscovery: ok\n");
  return 0;
}
