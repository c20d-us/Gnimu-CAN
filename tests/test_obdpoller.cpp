// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

// OBD-II polling: one request in flight, the most overdue channel first,
// replies and timeouts counted, unsupported PIDs skipped, the later support
// bitmaps, negative responses, channels that stop answering, the request
// cap and reply times; and the vehicle activity rule built on what polling
// finds.

#include <assert.h>
#include <stdio.h>

#include "../code/GnimuCAN/src/core/DiscoveryBackoff.h"
#include "../code/GnimuCAN/src/core/ObdPoller.h"
#include "../code/GnimuCAN/src/core/VehicleActivity.h"

using namespace gnimu;

static const PollChannelConfig kTwo[] = {{0x0C, 40, 2}, {0x11, 40, 1}};
static const PollerConfig kPoller = {50, 0, 3, 5000, 3};
// PID 00 bitmap with 0x0C and 0x11 set (and 0x05, 0x0D, 0x0F, 0x20).
static const uint32_t kBoth = 0x081A8001;

static CanFrame reply(uint32_t id, uint8_t pid, uint8_t a, uint8_t b) {
  CanFrame f = {};
  f.id = id;
  f.len = 8;
  const uint8_t d[8] = {4, 0x41, pid, a, b, 0x55, 0x55, 0x55};
  for (int i = 0; i < 8; i++) f.data[i] = d[i];
  return f;
}

static CanFrame bitmapReply(uint32_t id, uint8_t pid, uint32_t bitmap) {
  CanFrame f = {};
  f.id = id;
  f.len = 8;
  const uint8_t d[8] = {6,
                        0x41,
                        pid,
                        static_cast<uint8_t>(bitmap >> 24),
                        static_cast<uint8_t>(bitmap >> 16),
                        static_cast<uint8_t>(bitmap >> 8),
                        static_cast<uint8_t>(bitmap),
                        0x55};
  for (int i = 0; i < 8; i++) f.data[i] = d[i];
  return f;
}

static CanFrame negative(uint32_t id, uint8_t nrc) {
  CanFrame f = {};
  f.id = id;
  f.len = 8;
  const uint8_t d[8] = {3, 0x7F, 0x01, nrc, 0x55, 0x55, 0x55, 0x55};
  for (int i = 0; i < 8; i++) f.data[i] = d[i];
  return f;
}

static uint32_t bit(uint8_t pidInRange) { return 1u << (32 - pidInRange); }

static void testSupportBitmap() {
  assert(obd2::pidSupported(kBoth, 0x0C));
  assert(obd2::pidSupported(kBoth, 0x11));
  assert(obd2::pidSupported(kBoth, 0x05));
  assert(obd2::pidSupported(kBoth, 0x20));
  assert(!obd2::pidSupported(kBoth, 0x0B));
  assert(obd2::pidSupported(0x80000000u, 0x01));
  assert(!obd2::pidSupported(0xFFFFFFFFu, 0x00));
  assert(!obd2::pidSupported(0xFFFFFFFFu, 0x21));
}

static void testOneInFlight() {
  ObdPoller<2> p(kTwo, kPoller);
  p.start(0x7E0, kBoth, 1000);
  CanFrame req;
  assert(p.next(1000, req));
  assert(req.id == 0x7E0 && req.data[1] == 0x01);
  const uint8_t first = req.data[2];
  // Nothing more until it's answered.
  assert(!p.next(1001, req));
  // A reply for the other PID, from a non-OBD ID, or from another ECU doesn't count.
  assert(!p.onFrame(reply(0x7E8, first == 0x0C ? 0x11 : 0x0C, 1, 2), 1002));
  assert(!p.onFrame(reply(0x100, first, 1, 2), 1002));
  assert(!p.onFrame(reply(0x7E9, first, 1, 2), 1002));
  assert(p.onFrame(reply(0x7E8, first, 0x0C, 0x80), 1003));
  // The other channel goes next, then nothing until a period has passed.
  assert(p.next(1004, req));
  assert(req.data[2] != first);
  assert(p.onFrame(reply(0x7E8, req.data[2], 0x0C, 0x80), 1005));
  assert(!p.next(1006, req));
  assert(p.next(1040, req));
  const size_t i = first == 0x0C ? 0 : 1;
  assert(p.channel(i).replies == 1 && p.channel(i).valid && p.channel(i).lastReplyMs == 1003);
  assert(p.channel(i).replyMsTotal == 3 && p.channel(i).replyMsMax == 3);
}

static void testTimeout() {
  ObdPoller<2> p(kTwo, kPoller);
  p.start(0x7E0, kBoth, 0);
  CanFrame req;
  assert(p.next(0, req));
  p.checkTimeout(49);
  assert(!p.next(49, req));
  p.checkTimeout(50);
  assert(p.next(50, req));
  assert(p.channel(0).timeouts + p.channel(1).timeouts == 1);
  // A late reply after the timeout doesn't count.
  p.checkTimeout(100);
  assert(!p.onFrame(reply(0x7E8, req.data[2], 1, 2), 101));
  assert(p.channel(0).timeouts + p.channel(1).timeouts == 2);
  assert(p.silentForMs(2000) == 2000);
}

static void testUnsupportedAndOffSkipped() {
  const PollChannelConfig channels[] = {{0x0C, 40, 2}, {0x11, 40, 1}, {0x05, 0, 1}};
  ObdPoller<3> p(channels, kPoller);
  p.start(0x7E1, bit(0x0C) | bit(0x05), 0);
  assert(p.channel(0).state == ChannelState::Polling);
  assert(p.channel(1).state == ChannelState::Unsupported);
  assert(p.channel(2).state == ChannelState::Off);
  for (uint32_t t = 0; t < 1000; t += 10) {
    CanFrame req;
    if (p.next(t, req)) {
      assert(req.id == 0x7E1 && req.data[2] == 0x0C);
      assert(p.onFrame(reply(0x7E9, 0x0C, 0, 0), t + 1));
    }
  }
  assert(p.channel(1).requests == 0 && p.channel(2).requests == 0);
  assert(p.channel(0).requests == 25);  // every 40 ms over 1 s
}

// Two channels at different periods, polled on a 10 ms tick with instant
// replies, each get their own rate.
static void testFairShare() {
  const PollChannelConfig channels[] = {{0x0C, 20, 2}, {0x11, 100, 1}};
  ObdPoller<2> p(channels, kPoller);
  p.start(0x7E0, kBoth, 0);
  for (uint32_t t = 0; t < 10000; t += 10) {
    CanFrame req;
    if (p.next(t, req)) assert(p.onFrame(reply(0x7E8, req.data[2], 0, 0), t + 2));
  }
  assert(p.channel(0).replies >= 450 && p.channel(0).replies <= 501);
  assert(p.channel(1).replies >= 95 && p.channel(1).replies <= 101);
}

static void testStartClears() {
  ObdPoller<2> p(kTwo, kPoller);
  p.start(0x7E0, kBoth, 0);
  CanFrame req;
  assert(p.next(0, req));
  p.start(0x7E2, kBoth, 500);
  assert(p.channel(0).requests == 0 && p.channel(1).requests == 0);
  assert(!p.channel(0).valid && p.silentForMs(500) == 0);
  assert(p.next(500, req) && req.id == 0x7E2);
}

// Channels above 0x20 wait for the bitmaps that cover them, each read only
// when the one before says it exists.
static void testLaterBitmaps() {
  const PollChannelConfig channels[] = {{0x0C, 40, 2}, {0x2F, 1000, 1}, {0x49, 40, 1}, {0x5C, 1000, 1}};
  ObdPoller<4> p(channels, kPoller);
  p.start(0x7E0, bit(0x0C) | bit(0x20), 0);
  assert(p.channel(1).state == ChannelState::Unknown && !p.supportKnown());
  CanFrame req;
  assert(p.next(0, req) && req.data[2] == 0x20);  // the bitmap first
  assert(p.onFrame(bitmapReply(0x7E8, 0x20, bit(0x2F - 0x20) | bit(0x20)), 5));
  assert(p.channel(1).state == ChannelState::Polling);
  assert(p.next(10, req) && req.data[2] == 0x40);
  assert(p.onFrame(bitmapReply(0x7E8, 0x40, bit(0x49 - 0x40)), 15));  // no 0x60
  assert(p.channel(2).state == ChannelState::Polling);
  assert(p.channel(3).state == ChannelState::Unsupported);
  assert(p.supportKnown());
  assert(p.next(20, req) && req.data[2] != 0x60);

  // No bitmap 0x20 claimed: nothing above 0x20 is supported, and none is asked for.
  ObdPoller<4> q(channels, kPoller);
  q.start(0x7E0, bit(0x0C), 0);
  assert(q.supportKnown() && q.channel(1).state == ChannelState::Unsupported);
  assert(q.next(0, req) && req.data[2] == 0x0C);

  // A bitmap request that times out once the ECU is known to answer: the same.
  ObdPoller<4> r(channels, kPoller);
  r.start(0x7E0, bit(0x0C) | bit(0x20), 0, true);
  assert(r.next(0, req) && req.data[2] == 0x20);
  r.checkTimeout(50);
  assert(r.supportKnown() && r.channel(2).state == ChannelState::Unsupported);
}

static void testNegativeResponses() {
  ObdPoller<2> p(kTwo, kPoller);
  p.start(0x7E0, kBoth, 0);
  CanFrame req;
  // "Pending": keep waiting, past the usual timeout.
  assert(p.next(0, req));
  const uint8_t first = req.data[2];
  const size_t i = first == 0x0C ? 0 : 1;
  assert(p.onFrame(negative(0x7E8, obd2::kResponsePending), 40));
  p.checkTimeout(60);
  assert(!p.next(60, req));  // still in flight
  assert(p.onFrame(reply(0x7E8, first, 1, 2), 70));
  assert(p.channel(i).replies == 1 && p.channel(i).timeouts == 0);
  // "Busy": settled, not a timeout, asked again next period.
  assert(p.next(70, req) && req.data[2] != first);
  assert(p.onFrame(negative(0x7E8, obd2::kBusyRepeatRequest), 72));
  assert(p.channel(1 - i).timeouts == 0 && p.channel(1 - i).state == ChannelState::Polling);
  // "Not supported": dropped for the session.
  assert(p.next(110, req));
  const size_t j = req.data[2] == 0x0C ? 0 : 1;
  assert(p.onFrame(negative(0x7E8, 0x31), 112));
  assert(p.channel(j).state == ChannelState::Refused);
  for (uint32_t t = 120; t < 1000; t += 10) {
    if (p.next(t, req)) {
      assert(req.data[2] != p.channel(j).pid);
      p.onFrame(reply(0x7E8, req.data[2], 1, 2), t + 1);
    }
  }
}

// A channel that stops answering is asked only every 5 s, and the other
// keeps its rate; it comes back when it answers.
static void testSilentChannelBacksOff() {
  ObdPoller<2> p(kTwo, kPoller);
  p.start(0x7E0, kBoth, 0);
  uint32_t asked11 = 0;
  for (uint32_t t = 0; t < 10000; t += 10) {
    CanFrame req;
    p.checkTimeout(t);
    if (!p.next(t, req)) continue;
    if (req.data[2] == 0x0C) {
      p.onFrame(reply(0x7E8, 0x0C, 0, 0), t + 1);
    } else {
      asked11++;  // never answered
    }
  }
  assert(p.channel(1).state == ChannelState::Silent);
  assert(asked11 >= 3 && asked11 <= 6);  // 3 to go silent, then every 5 s
  assert(p.channel(0).replies > 200);
  // It answers again: back to its rate.
  CanFrame req;
  for (uint32_t t = 10000; t < 20000; t += 10) {
    p.checkTimeout(t);
    if (p.next(t, req)) p.onFrame(reply(0x7E8, req.data[2], 0, 0), t + 1);
    if (p.channel(1).state == ChannelState::Polling) break;
  }
  assert(p.channel(1).state == ChannelState::Polling);
}

// The request cap holds whatever the periods ask for.
static void testRequestCap() {
  const PollChannelConfig channels[] = {{0x0C, 10, 2}, {0x11, 10, 1}};
  const PollerConfig capped = {50, 20, 3, 5000, 3};  // at most 50 a second
  ObdPoller<2> p(channels, capped);
  p.start(0x7E0, kBoth, 0);
  uint32_t sent = 0;
  for (uint32_t t = 0; t < 1000; t += 1) {
    CanFrame req;
    if (p.next(t, req)) {
      sent++;
      p.onFrame(reply(0x7E8, req.data[2], 0, 0), t);
    }
  }
  assert(sent == 50);
}

// A reply too short for its PID is answered, but carries no value.
static void testShortReplyHasNoValue() {
  ObdPoller<2> p(kTwo, kPoller);
  p.start(0x7E0, bit(0x0C), 0);
  CanFrame req;
  assert(p.next(0, req) && req.data[2] == 0x0C);
  CanFrame shortReply = reply(0x7E8, 0x0C, 0x0C, 0);
  shortReply.data[0] = 3;  // mode, PID, one byte: RPM needs two
  assert(p.onFrame(shortReply, 2));
  assert(!p.channel(0).valid && p.channel(0).replies == 0);
}

// Found on the M2: the gateway passes only functional requests. Three
// unanswered physical requests, and polling asks everyone instead, still
// taking replies only from the polled ECU.
static void testAsksEveryoneWhenDirectGoesUnanswered() {
  ObdPoller<2> p(kTwo, kPoller);
  p.start(0x7E0, kBoth, 0);
  CanFrame req;
  uint32_t t = 0;
  for (int i = 0; i < 3; i++) {
    assert(p.next(t, req) && req.id == 0x7E0);
    t += 50;
    p.checkTimeout(t);
  }
  assert(p.askingEveryone() && p.sendId() == 0x7DF);
  assert(p.takeSwitchedToEveryone() && !p.takeSwitchedToEveryone());
  assert(p.channel(0).state == ChannelState::Polling && p.channel(1).state == ChannelState::Polling);
  assert(p.next(t, req) && req.id == 0x7DF);
  assert(!p.onFrame(reply(0x7E9, req.data[2], 1, 2), t + 5));  // another ECU: ignored
  assert(p.onFrame(reply(0x7E8, req.data[2], 1, 2), t + 6));
}

// An ECU that answers direct requests is never switched, whatever times out
// later.
static void testDirectAnsweredNeverSwitches() {
  ObdPoller<2> p(kTwo, kPoller);
  p.start(0x7E0, kBoth, 0);
  CanFrame req;
  assert(p.next(0, req) && p.onFrame(reply(0x7E8, req.data[2], 1, 2), 5));
  uint32_t t = 10;
  for (int i = 0; i < 10; i++) {
    if (p.next(t, req)) assert(req.id == 0x7E0);
    t += 60;
    p.checkTimeout(t);
  }
  assert(!p.askingEveryone() && !p.takeSwitchedToEveryone());
}

// Once found this boot, the next session asks everyone from the start.
static void testStartAskingEveryone() {
  ObdPoller<2> p(kTwo, kPoller);
  p.start(0x7E0, kBoth, 0, true);
  CanFrame req;
  assert(p.next(0, req) && req.id == 0x7DF && !p.takeSwitchedToEveryone());
}

// A support bitmap unanswered before anything has answered is asked again,
// to everyone once it switches, rather than given up on.
static void testBitmapRetriedUntilEveryone() {
  const PollChannelConfig channels[] = {{0x0C, 40, 2}, {0x2F, 1000, 1}};
  ObdPoller<2> p(channels, kPoller);
  p.start(0x7E0, bit(0x0C) | bit(0x20), 0);
  CanFrame req;
  uint32_t t = 0;
  for (int i = 0; i < 3; i++) {
    assert(p.next(t, req) && req.data[2] == 0x20 && req.id == 0x7E0);
    t += 50;
    p.checkTimeout(t);
  }
  assert(p.askingEveryone() && p.channel(1).state == ChannelState::Unknown);
  assert(p.next(t, req) && req.data[2] == 0x20 && req.id == 0x7DF);
  assert(p.onFrame(bitmapReply(0x7E8, 0x20, bit(0x2F - 0x20)), t + 5));
  assert(p.channel(1).state == ChannelState::Polling);
}

static void testActivity() {
  // Fresh RPM decides, whatever the supply says.
  assert(vehicleActivity(true, 800, 400, SupplyLevel::Parked) == VehicleActivity::EngineRunning);
  assert(vehicleActivity(true, 0, 400, SupplyLevel::EngineRunning) == VehicleActivity::IgnitionOn);
  assert(vehicleActivity(true, 400, 400, SupplyLevel::Unknown) == VehicleActivity::EngineRunning);
  // Otherwise the supply level.
  assert(vehicleActivity(false, 800, 400, SupplyLevel::EngineRunning) == VehicleActivity::EngineRunning);
  assert(vehicleActivity(false, 800, 400, SupplyLevel::Parked) == VehicleActivity::Parked);
  assert(vehicleActivity(false, 0, 400, SupplyLevel::BelowCutoff) == VehicleActivity::Parked);
  assert(vehicleActivity(false, 0, 400, SupplyLevel::Unknown) == VehicleActivity::Unknown);
}

// Discovery on a bus nobody answers: 5 s, then 30 s, then 2 minutes.
static void testDiscoveryBackoff() {
  const DiscoveryBackoff b = {5000, 30000, 120000, 3};
  assert(discoveryRetryMs(b, 1) == 5000 && discoveryRetryMs(b, 3) == 5000);
  assert(discoveryRetryMs(b, 4) == 30000 && discoveryRetryMs(b, 6) == 30000);
  assert(discoveryRetryMs(b, 7) == 120000 && discoveryRetryMs(b, 1000000) == 120000);
  // An hour of a bus that never answers: under 40 requests, not 720.
  uint32_t t = 0, n = 0;
  while (t < 3600000) t += discoveryRetryMs(b, ++n);
  assert(n < 40);
}

int main() {
  testDiscoveryBackoff();
  testSupportBitmap();
  testOneInFlight();
  testTimeout();
  testUnsupportedAndOffSkipped();
  testFairShare();
  testStartClears();
  testLaterBitmaps();
  testNegativeResponses();
  testSilentChannelBacksOff();
  testRequestCap();
  testShortReplyHasNoValue();
  testAsksEveryoneWhenDirectGoesUnanswered();
  testDirectAnsweredNeverSwitches();
  testStartAskingEveryone();
  testBitmapRetriedUntilEveryone();
  testActivity();
  printf("test_obdpoller: ok\n");
  return 0;
}
