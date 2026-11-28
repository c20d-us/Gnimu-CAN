// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

// Multi-PID polling: the request and flow control frames, splitting a reply,
// receiving a reply of several frames, the schedule built from the real
// channel table, and the poller asking for several PIDs per request: the
// cycle, flow control, PIDs left out, failures, replies that stop after
// their first frame, the fall back to one PID per request,
// and not falling back when the car has only gone quiet or the engine has
// stopped.

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "../code/GnimuCAN/src/app/SystemConfig.h"
#include "../code/GnimuCAN/src/core/IsoTp.h"
#include "../code/GnimuCAN/src/core/Obd2.h"
#include "../code/GnimuCAN/src/core/ObdPoller.h"
#include "../code/GnimuCAN/src/core/ObdSchedule.h"

using namespace gnimu;

static CanFrame frameOf(uint32_t id, const uint8_t* d, size_t n) {
  CanFrame f = {};
  f.id = id;
  f.len = 8;
  for (size_t i = 0; i < 8; i++) f.data[i] = i < n ? d[i] : 0x55;
  return f;
}

// The CAN frames carrying `payload` from `id`, as an ECU sends them: a single
// frame, or a first frame and consecutive frames.
static size_t framesFor(uint32_t id, const uint8_t* payload, size_t len, CanFrame* out) {
  uint8_t d[8];
  if (len <= 7) {
    d[0] = static_cast<uint8_t>(len);
    memcpy(d + 1, payload, len);
    out[0] = frameOf(id, d, len + 1);
    return 1;
  }
  d[0] = static_cast<uint8_t>(0x10 | (len >> 8));
  d[1] = static_cast<uint8_t>(len);
  memcpy(d + 2, payload, 6);
  out[0] = frameOf(id, d, 8);
  size_t n = 1, at = 6;
  uint8_t sn = 1;
  while (at < len) {
    const size_t take = len - at < 7 ? len - at : 7;
    d[0] = static_cast<uint8_t>(0x20 | sn);
    memcpy(d + 1, payload + at, take);
    out[n++] = frameOf(id, d, take + 1);
    at += take;
    sn = static_cast<uint8_t>((sn + 1) & 0x0F);
  }
  return n;
}

static void testFrames() {
  const uint8_t pids[] = {0x0C, 0x11, 0x0D};
  const CanFrame r = obd2::mode01Request(pids, 3, 0x7DF);
  const uint8_t want[8] = {0x04, 0x01, 0x0C, 0x11, 0x0D, 0x55, 0x55, 0x55};
  assert(r.id == 0x7DF && r.len == 8 && memcmp(r.data, want, 8) == 0);
  const CanFrame fc = obd2::flowControl(0x7E0);
  const uint8_t wantFc[8] = {0x30, 0x00, 0x00, 0x55, 0x55, 0x55, 0x55, 0x55};
  assert(fc.id == 0x7E0 && memcmp(fc.data, wantFc, 8) == 0);
}

static void testSplit() {
  const uint8_t pids[] = {0x04, 0x05, 0x0B, 0x0C};
  const uint8_t lengths[] = {1, 1, 1, 2};
  uint8_t off[4];
  // The ELM327 datasheet's example, and the same PIDs in another order.
  const uint8_t inOrder[] = {0x41, 0x04, 0x3F, 0x05, 0x44, 0x0B, 0x21, 0x0C, 0x17, 0xB8};
  assert(obd2::splitMode01Reply(inOrder, sizeof(inOrder), pids, lengths, 4, off));
  assert(off[0] == 2 && off[1] == 4 && off[2] == 6 && off[3] == 8);
  const uint8_t shuffled[] = {0x41, 0x0B, 0x21, 0x04, 0x3F, 0x0C, 0x17, 0xB8, 0x05, 0x44};
  assert(obd2::splitMode01Reply(shuffled, sizeof(shuffled), pids, lengths, 4, off));
  assert(shuffled[off[3]] == 0x17 && shuffled[off[3] + 1] == 0xB8 && shuffled[off[1]] == 0x44);
  // A PID left out: the rest still split.
  const uint8_t missing[] = {0x41, 0x0C, 0x17, 0xB8, 0x04, 0x3F};
  assert(obd2::splitMode01Reply(missing, sizeof(missing), pids, lengths, 4, off));
  assert(off[0] == 5 && off[1] == 0 && off[2] == 0 && off[3] == 2);
  // Untrustworthy: a PID not asked for, one repeated, a length that runs
  // past the end, a byte left over, nothing answered, another mode.
  const uint8_t stranger[] = {0x41, 0x0C, 0x17, 0xB8, 0x2F, 0x80};
  assert(!obd2::splitMode01Reply(stranger, sizeof(stranger), pids, lengths, 4, off));
  const uint8_t twice[] = {0x41, 0x04, 0x3F, 0x04, 0x3F};
  assert(!obd2::splitMode01Reply(twice, sizeof(twice), pids, lengths, 4, off));
  const uint8_t shortRpm[] = {0x41, 0x04, 0x3F, 0x0C, 0x17};
  assert(!obd2::splitMode01Reply(shortRpm, sizeof(shortRpm), pids, lengths, 4, off));
  const uint8_t extra[] = {0x41, 0x04, 0x3F, 0x00};
  assert(!obd2::splitMode01Reply(extra, sizeof(extra), pids, lengths, 4, off));
  const uint8_t none[] = {0x41};
  assert(!obd2::splitMode01Reply(none, sizeof(none), pids, lengths, 4, off));
  const uint8_t mode2[] = {0x42, 0x04, 0x3F};
  assert(!obd2::splitMode01Reply(mode2, sizeof(mode2), pids, lengths, 4, off));
}

static void testIsoTp() {
  using Rx = IsoTpReceiver<40>;
  Rx rx;
  // A single frame.
  const uint8_t sf[] = {0x04, 0x41, 0x0C, 0x1A, 0xF8};
  rx.reset();
  assert(rx.frame(sf, 8) == Rx::Step::Complete && rx.length() == 4 && rx.payload()[3] == 0xF8);
  // Several frames: flow control after the first, complete after the last.
  uint8_t payload[19];
  for (size_t i = 0; i < sizeof(payload); i++) payload[i] = static_cast<uint8_t>(i);
  CanFrame f[4];
  const size_t n = framesFor(0x7E8, payload, sizeof(payload), f);
  assert(n == 3);
  rx.reset();
  assert(rx.frame(f[0].data, 8) == Rx::Step::SendFlowControl);
  assert(rx.length() == 0);
  assert(rx.frame(f[1].data, 8) == Rx::Step::More);
  assert(rx.frame(f[2].data, 8) == Rx::Step::Complete);
  assert(rx.length() == sizeof(payload) && memcmp(rx.payload(), payload, sizeof(payload)) == 0);
  assert(rx.frame(f[2].data, 8) == Rx::Step::More);  // done: nothing more counts
  // Out of sequence: the message is lost, and stays lost.
  rx.reset();
  assert(rx.frame(f[0].data, 8) == Rx::Step::SendFlowControl);
  assert(rx.frame(f[2].data, 8) == Rx::Step::Failed);
  assert(rx.frame(f[1].data, 8) == Rx::Step::More && rx.length() == 0);
  // A consecutive frame or flow control with no first frame: not ours.
  rx.reset();
  assert(rx.frame(f[1].data, 8) == Rx::Step::More);
  const uint8_t fc[] = {0x30, 0x00, 0x00};
  assert(rx.frame(fc, 8) == Rx::Step::More);
  // A first frame for what a single frame holds, or more than fits.
  const uint8_t tiny[8] = {0x10, 0x07, 1, 2, 3, 4, 5, 6};
  rx.reset();
  assert(rx.frame(tiny, 8) == Rx::Step::Failed);
  const uint8_t huge[8] = {0x10, 41, 1, 2, 3, 4, 5, 6};
  rx.reset();
  assert(rx.frame(huge, 8) == Rx::Step::Failed);
  // A single frame in the middle of a message.
  rx.reset();
  assert(rx.frame(f[0].data, 8) == Rx::Step::SendFlowControl);
  assert(rx.frame(sf, 8) == Rx::Step::Failed);
}

// The real channel table (Config.h), as the firmware schedules it: the
// busiest request, and every cycle within it.
static void testRealSchedule() {
  const size_t n = config::kObdChannelCount;
  uint32_t every[n];
  uint32_t offset[n];
  bool active[n];
  for (size_t i = 0; i < n; i++) {
    uint32_t period = config::kHelperPeriodMs;  // the helper channels come last
    if (i < config::kObdPollCount) period = config::kObdPolls[i].periodMs;
    active[i] = period > 0;
    every[i] = period / config::kObdCycleMs;
  }
  const uint32_t busiest = buildSchedule(every, active, n, offset);
  const uint32_t cycles = scheduleCycles(every, active, n);
  assert(cycles == 250);
  assert(busiest == 3);
  assert(busiest <= config::kObdMaxPidsPerRequest);
  uint32_t most = 0;
  for (uint32_t k = 0; k < cycles; k++) {
    uint32_t load = 0;
    for (size_t i = 0; i < n; i++)
      if (active[i] && k % every[i] == offset[i]) load++;
    if (load > most) most = load;
    assert(load >= 2);  // RPM and Throttle, every cycle
  }
  assert(most == busiest);
  // The same table, the same schedule.
  uint32_t again[n];
  buildSchedule(every, active, n, again);
  assert(memcmp(offset, again, sizeof(again)) == 0);
}

static void testScheduleSpreads() {
  // Four channels every 4 cycles: one per cycle, not all in one.
  const uint32_t every[] = {4, 4, 4, 4, 0};
  const bool active[] = {true, true, true, true, false};
  uint32_t offset[5];
  assert(buildSchedule(every, active, 5, offset) == 1);
  assert(offset[0] == 0 && offset[1] == 1 && offset[2] == 2 && offset[3] == 3 && offset[4] == 0);
  assert(scheduleCycles(every, active, 5) == 4);
}

// A poller over five channels: RPM and Throttle every cycle (20 ms), Speed
// and Fuel every 5 (100 ms), Coolant every 50 (1 s).
static const PollChannelConfig kFive[] = {
    {0x0C, 20, 2}, {0x11, 20, 1}, {0x0D, 100, 1}, {0x2F, 100, 1}, {0x05, 1000, 1}};
static const PollerConfig kBundled = {50, 9, 3, 5000, 3, 20, 6, 3};
// PID 00 bitmap: 05, 0C, 0D and 11 (and 0x20, more bitmaps).
static const uint32_t kSupport = (1u << (32 - 0x05)) | (1u << (32 - 0x0C)) | (1u << (32 - 0x0D)) |
                                 (1u << (32 - 0x11));
// ... and PID 20's: 2F.
static const uint32_t kSupport20 = 1u << (32 - (0x2F - 0x20));

static CanFrame bitmapReply(uint8_t pid, uint32_t b) {
  const uint8_t d[] = {6, 0x41, pid, static_cast<uint8_t>(b >> 24), static_cast<uint8_t>(b >> 16),
                       static_cast<uint8_t>(b >> 8), static_cast<uint8_t>(b)};
  return frameOf(0x7E8, d, sizeof(d));
}

// The reply an ECU gives to `req`: each PID asked for with its data (RPM
// 0x1AF8, the others 0x40), except `leaveOut`.
static size_t answer(const CanFrame& req, CanFrame* out, uint8_t leaveOut = 0) {
  uint8_t p[32];
  size_t len = 0;
  p[len++] = 0x41;
  for (uint8_t i = 0; i + 1 < req.data[0]; i++) {
    const uint8_t pid = req.data[2 + i];
    if (pid == leaveOut) continue;
    p[len++] = pid;
    if (pid == 0x0C) {
      p[len++] = 0x1A;
      p[len++] = 0xF8;
    } else {
      p[len++] = 0x40;
    }
  }
  return framesFor(0x7E8, p, len, out);
}

static bool asks(const CanFrame& req, uint8_t pid) {
  for (uint8_t i = 0; i + 1 < req.data[0]; i++)
    if (req.data[2 + i] == pid) return true;
  return false;
}

// Feeds a reply's frames, checking the flow control asked for after the
// first goes to `fcId`.
static void deliver(ObdPoller<5>& p, const CanFrame* f, size_t n, uint32_t now, uint32_t fcId = 0x7E0) {
  for (size_t i = 0; i < n; i++) {
    p.onFrame(f[i], now);
    CanFrame fc;
    const bool asked = p.takeFlowControl(fc);
    assert(asked == (i == 0 && n > 1));
    if (asked) assert(fc.id == fcId && fc.data[0] == 0x30);
  }
}

// Starts a session and reads the PID 20 bitmap, so support is known and the
// ECU has answered: the next request is multi-PID.
static void startBundled(ObdPoller<5>& p, uint32_t now, bool askEveryone = false) {
  p.start(0x7E0, kSupport | 1u, now - 20, askEveryone);
  CanFrame req;
  assert(p.next(now - 20, req) && req.data[0] == 2 && req.data[2] == 0x20);
  p.onFrame(bitmapReply(0x20, kSupport20), now - 15);
  assert(p.supportKnown());
}

static void testBundledRequests() {
  ObdPoller<5> p(kFive, kBundled);
  uint32_t now = 1000;
  startBundled(p, now);
  CanFrame req;
  assert(p.next(now, req));
  assert(p.takeStartedBundling() && !p.takeStartedBundling());
  assert(p.bundleStats().active && p.bundleStats().busiest == 3);
  assert(p.channel(0).every == 1 && p.channel(2).every == 5 && p.channel(4).every == 50);
  // Cycle 0: RPM, Throttle and whichever 100 ms channel starts there.
  assert(req.id == 0x7E0 && req.data[1] == 0x01 && req.data[0] == 4);
  assert(asks(req, 0x0C) && asks(req, 0x11));
  CanFrame f[4];
  size_t n = answer(req, f);
  assert(n == 2);  // 8 bytes: a first frame and one more
  deliver(p, f, n, now + 8);
  assert(p.channel(0).valid && p.channel(0).data[0] == 0x1A && p.channel(0).data[1] == 0xF8);
  assert(p.channel(0).replies == 1 && p.channel(0).replyMsMax == 8);
  assert(p.bundleStats().replies == 1);
  // The next request waits for its cycle.
  assert(!p.next(now + 19, req));
  // The rest of a second of cycles, answered at once: RPM and Throttle in
  // every request, Speed and Fuel in every fifth, Coolant once.
  uint32_t requests = 1;
  uint32_t speed = 0, coolant = 0;
  for (now += 20; now < 2000; now++) {
    if (!p.next(now, req)) continue;
    requests++;
    assert(asks(req, 0x0C) && asks(req, 0x11));
    if (asks(req, 0x0D)) speed++;
    if (asks(req, 0x05)) coolant++;
    n = answer(req, f);
    deliver(p, f, n, now + 5);
  }
  assert(requests == 50);
  assert(speed == 9);
  assert(coolant == 1);
  assert(p.channel(0).replies == 50 && p.channel(0).timeouts == 0);
  assert(p.bundleStats().failures == 0 && p.bundleStats().late == 0);
}

static void testFlowControlToThePhysicalAddress() {
  // Asking everyone (0x7DF), flow control still goes to the ECU itself.
  ObdPoller<5> p(kFive, kBundled);
  startBundled(p, 1000, true);
  CanFrame req;
  assert(p.next(1000, req) && req.id == 0x7DF);
  CanFrame f[4];
  const size_t n = answer(req, f);
  deliver(p, f, n, 1005);  // checks the flow control's address
  assert(p.bundleStats().replies == 1);
}

static void testLeftOut() {
  ObdPoller<5> p(kFive, kBundled);
  uint32_t now = 1000;
  startBundled(p, now);
  CanFrame req;
  CanFrame f[4];
  // Throttle left out of three replies: it alone counts timeouts, and stops
  // answering, but keeps its turns; multi-PID polling is fine.
  for (int i = 0; i < 3; i++, now += 20) {
    assert(p.next(now, req) && asks(req, 0x11));
    const size_t n = answer(req, f, 0x11);
    deliver(p, f, n, now + 5);
  }
  assert(p.channel(1).timeouts == 3 && p.channel(1).state == ChannelState::Silent);
  assert(p.channel(0).replies == 3 && p.channel(0).timeouts == 0);
  assert(p.bundleStats().failures == 0);
  assert(p.next(now, req) && asks(req, 0x11));
  // It answers again.
  deliver(p, f, answer(req, f), now + 5);
  assert(p.channel(1).state == ChannelState::Polling && p.channel(1).replies == 1);
}

static void testFallBack() {
  ObdPoller<5> p(kFive, kBundled);
  uint32_t now = 1000;
  startBundled(p, now);
  CanFrame req;
  CanFrame f[4];
  // Two timeouts, then a good reply: the count starts over.
  for (int i = 0; i < 2; i++) {
    assert(p.next(now, req) && req.data[0] > 2);
    p.checkTimeout(now + 50);
    now += 60;
  }
  assert(p.bundleStats().failures == 2 && !p.takeFellBack());
  assert(p.channel(0).timeouts == 0);  // the request failed, not the channels
  assert(p.next(now, req));
  deliver(p, f, answer(req, f), now + 5);
  now += 20;
  // A reply with a PID not asked for, "not supported", a timeout: three in
  // a row.
  assert(p.next(now, req) && !asks(req, 0x46));
  const uint8_t stranger[] = {5, 0x41, 0x0C, 0x1A, 0xF8, 0x46, 0x80};
  p.onFrame(frameOf(0x7E8, stranger, sizeof(stranger)), now + 3);
  assert(p.bundleStats().failures == 3 && p.channel(0).replies == 1);
  now += 20;
  assert(p.next(now, req));
  const uint8_t refused[] = {3, 0x7F, 0x01, 0x12};
  p.onFrame(frameOf(0x7E8, refused, 4), now + 3);
  assert(p.channel(0).state == ChannelState::Polling);  // not refused: the request was
  now += 20;
  assert(p.next(now, req));
  p.checkTimeout(now + 50);
  assert(p.takeFellBack() && !p.takeFellBack());
  const BundleStats& b = p.bundleStats();
  assert(b.fellBack && !b.active && b.fault == BundleFault::Timeout && b.failures == 5);
  // One PID per request from now on.
  now += 60;
  assert(p.next(now, req) && req.data[0] == 2);
}

static void testBusyIsNotAFailure() {
  ObdPoller<5> p(kFive, kBundled);
  startBundled(p, 1000);
  CanFrame req;
  assert(p.next(1000, req));
  const uint8_t busy[] = {3, 0x7F, 0x01, 0x21};
  p.onFrame(frameOf(0x7E8, busy, 4), 1003);
  assert(p.bundleStats().failures == 0);
  // Response pending: still waiting, past the usual timeout.
  assert(p.next(1020, req));
  const uint8_t pending[] = {3, 0x7F, 0x01, 0x78};
  p.onFrame(frameOf(0x7E8, pending, 4), 1060);
  p.checkTimeout(1090);
  CanFrame f[4];
  deliver(p, f, answer(req, f), 1095);
  assert(p.bundleStats().failures == 0 && p.bundleStats().replies == 1);
}

static void testLateCycles() {
  ObdPoller<5> p(kFive, kBundled);
  uint32_t now = 1000;
  startBundled(p, now);
  CanFrame req;
  CanFrame f[4];
  assert(p.next(now, req));
  // A reply 45 ms later: the next cycle goes at once, from then on, with no
  // burst to catch up.
  deliver(p, f, answer(req, f), now + 45);
  assert(p.next(now + 45, req));
  assert(p.bundleStats().late == 1);
  deliver(p, f, answer(req, f), now + 50);
  assert(!p.next(now + 64, req));
  assert(p.next(now + 65, req));
}

static void testFullRequestCarriesOver() {
  // At most two PIDs per request: a cycle with three due sends the third in
  // the next request, ahead of that cycle's own.
  const PollChannelConfig three[] = {{0x0C, 20, 2}, {0x11, 20, 1}, {0x0D, 20, 1}};
  const PollerConfig two = {50, 9, 3, 5000, 3, 20, 2, 3};
  ObdPoller<3> p(three, two);
  p.start(0x7E0, kSupport, 1000);
  CanFrame req;
  // The first request goes one PID at a time, until the ECU answers.
  assert(p.next(1000, req) && req.data[0] == 2);
  const uint8_t rpm[] = {4, 0x41, 0x0C, 0x1A, 0xF8};
  p.onFrame(frameOf(0x7E8, rpm, 5), 1005);
  assert(p.next(1010, req) && req.data[0] == 3);
  assert(asks(req, 0x0C) && asks(req, 0x11) && !asks(req, 0x0D));
  const uint8_t r1[] = {6, 0x41, 0x0C, 0x1A, 0xF8, 0x11, 0x40};
  p.onFrame(frameOf(0x7E8, r1, 7), 1015);
  assert(p.next(1030, req) && asks(req, 0x0D) && asks(req, 0x0C) && !asks(req, 0x11));
}

// All five channels every cycle, so every reply spans two frames.
static const PollChannelConfig kFiveFast[] = {
    {0x0C, 20, 2}, {0x11, 20, 1}, {0x0D, 20, 1}, {0x2F, 20, 1}, {0x05, 20, 1}};

// A reply's first frame, then nothing: the flow control didn't get through.
static void stallAfterFirst(ObdPoller<5>& p, uint32_t& now) {
  CanFrame req;
  assert(p.next(now, req) && req.data[0] == 6);
  CanFrame f[4];
  assert(answer(req, f) == 2);
  p.onFrame(f[0], now + 5);
  CanFrame fc;
  assert(p.takeFlowControl(fc) && fc.id == 0x7E0);
  p.checkTimeout(now + 50);
  now += 60;
}

static void testNothingAfterFlowControl() {
  // The first frames come, the rest never do (a gateway dropping the flow
  // control): the ECU is answering, so no check first. One PID per request,
  // and the reason says why.
  ObdPoller<5> p(kFiveFast, kBundled);
  uint32_t now = 1000;
  startBundled(p, now, true);
  for (int i = 0; i < 3; i++) stallAfterFirst(p, now);
  assert(p.takeFellBack() && !p.checkAnswered());
  const BundleStats& b = p.bundleStats();
  assert(b.fellBack && b.fault == BundleFault::NoRest && b.failures == 3);
  CanFrame req;
  assert(p.next(now, req) && req.data[0] == 2);
}

static void testLostPartWayIsATimeout() {
  // Part of a reply after the flow control, then nothing: the flow control
  // got through, so it's a timeout, and it stays at the physical address.
  ObdPoller<5> p(kFiveFast, kBundled);
  uint32_t now = 1000;
  startBundled(p, now);
  for (int i = 0; i < 3; i++) {
    CanFrame req;
    assert(p.next(now, req));
    // A 19-byte reply: three frames, the last never sent.
    uint8_t payload[19] = {0x41, 0x0C, 0x1A, 0xF8, 0x11, 0x40, 0x0D, 0x40, 0x2F, 0x40, 0x05, 0x40};
    CanFrame f[4];
    assert(framesFor(0x7E8, payload, sizeof(payload), f) == 3);
    p.onFrame(f[0], now + 3);
    CanFrame fc;
    assert(p.takeFlowControl(fc));
    p.onFrame(f[1], now + 4);
    p.checkTimeout(now + 50);
    now += 60;
  }
  // All timeouts: one PID alone first, and it's answered; nothing more
  // goes until the caller says whether the engine runs. Running: the ECU
  // is there and doesn't take multi-PID requests.
  assert(!p.takeFellBack());
  CanFrame req;
  assert(p.next(now, req) && req.data[0] == 2 && req.data[2] == 0x0C);
  const uint8_t rpm[] = {4, 0x41, 0x0C, 0x1A, 0xF8};
  p.onFrame(frameOf(0x7E8, rpm, 5), now + 5);
  assert(p.checkAnswered() && !p.takeFellBack());
  assert(p.channel(0).data[0] == 0x1A);  // its value counts too, for the caller to judge
  assert(!p.next(now + 20, req) && !p.next(now + 100, req));
  p.settleCheck(true);
  assert(!p.checkAnswered() && p.takeFellBack());
  assert(p.bundleStats().fault == BundleFault::Timeout);
  assert(p.next(now + 100, req) && req.data[0] == 2);
  // A new session tries multi-PID requests again.
  startBundled(p, now + 1000);
  assert(!p.bundleStats().fellBack);
  assert(p.next(now + 1000, req) && req.data[0] > 2);
}

static void testEngineStoppedKeepsMultiPid() {
  // Switching off, the Santa Cruz's ECU stopped answering multi-PID
  // requests a couple of seconds before it went quiet, but still answered
  // RPM alone, at 0: the car shutting down, not refusing them.
  ObdPoller<5> p(kFiveFast, kBundled);
  p.setCheckChannel(1);  // asks for Throttle alone, say
  uint32_t now = 1000;
  startBundled(p, now);
  CanFrame req;
  for (int i = 0; i < 3; i++) {
    assert(p.next(now, req) && req.data[0] > 2);
    p.checkTimeout(now + 50);
    now += 60;
  }
  assert(p.next(now, req) && req.data[0] == 2 && req.data[2] == 0x11);
  const uint8_t thr[] = {3, 0x41, 0x11, 0x00};
  p.onFrame(frameOf(0x7E8, thr, 4), now + 5);
  assert(p.checkAnswered());
  p.settleCheck(false);
  assert(!p.checkAnswered() && !p.takeFellBack() && p.bundleStats().active);
  // Multi-PID again, and the count started over: two more timeouts aren't
  // enough to ask again.
  now += 60;
  for (int i = 0; i < 2; i++) {
    assert(p.next(now, req) && req.data[0] > 2);
    p.checkTimeout(now + 50);
    now += 60;
  }
  assert(p.next(now, req) && req.data[0] > 2);
}

static void testCarGoingQuietKeepsMultiPid() {
  // The car switches off: the ECU stops answering anything. Three timeouts,
  // then one PID alone goes unanswered too: not a multi-PID problem.
  ObdPoller<5> p(kFive, kBundled);
  uint32_t now = 1000;
  startBundled(p, now);
  CanFrame req;
  for (int i = 0; i < 3; i++) {
    assert(p.next(now, req) && req.data[0] > 2);
    p.checkTimeout(now + 50);
    now += 60;
  }
  assert(p.next(now, req) && req.data[0] == 2);
  p.checkTimeout(now + 50);
  now += 60;
  assert(!p.takeFellBack() && p.bundleStats().active && !p.bundleStats().fellBack);
  assert(p.bundleStats().failures == 3);
  // Still multi-PID; and if the car comes back, it carries on.
  assert(p.next(now, req) && req.data[0] > 2);
  CanFrame f[4];
  deliver(p, f, answer(req, f), now + 5);
  assert(p.bundleStats().replies == 1);
  // Two more timeouts don't fall back either: the count started over.
  now += 60;
  for (int i = 0; i < 2; i++) {
    assert(p.next(now, req) && req.data[0] > 2);
    p.checkTimeout(now + 50);
    now += 60;
  }
  assert(p.next(now, req) && req.data[0] > 2);
}

static void testBusyConfirmsTheEcu() {
  // One PID alone answered "busy" still shows the ECU is there.
  ObdPoller<5> p(kFive, kBundled);
  uint32_t now = 1000;
  startBundled(p, now);
  CanFrame req;
  for (int i = 0; i < 3; i++) {
    assert(p.next(now, req));
    p.checkTimeout(now + 50);
    now += 60;
  }
  assert(p.next(now, req) && req.data[0] == 2);
  const uint8_t busy[] = {3, 0x7F, 0x01, 0x21};
  p.onFrame(frameOf(0x7E8, busy, 4), now + 3);
  assert(p.checkAnswered());
  p.settleCheck(true);
  assert(p.takeFellBack());
}

// The car lost with a multi-PID request in flight (one goes every cycle):
// the next session's first reply, a support bitmap, is taken as such, not
// judged against the old request.
static void testNewSessionWithARequestInFlight() {
  ObdPoller<5> p(kFive, kBundled);
  uint32_t now = 1000;
  startBundled(p, now);
  CanFrame req;
  assert(p.next(now, req) && req.data[0] > 2);  // in flight, never answered
  now += 3000;
  startBundled(p, now);
  assert(p.bundleStats().failures == 0);
  assert(p.next(now, req) && req.data[0] > 2);  // multi-PID straight away
  CanFrame f[4];
  deliver(p, f, answer(req, f), now + 5);
  assert(p.bundleStats().replies == 1 && p.bundleStats().failures == 0);
}

static void testOnePidOnly() {
  // kObdMaxPidsPerRequest = 1.
  const PollerConfig one = {50, 9, 3, 5000, 3, 20, 1, 3};
  ObdPoller<5> p(kFive, one);
  p.start(0x7E0, kSupport | 1u, 1000);
  CanFrame req;
  assert(p.next(1000, req));
  p.onFrame(bitmapReply(0x20, kSupport20), 1005);
  for (uint32_t now = 1010; now < 1200; now += 10) {
    if (!p.next(now, req)) continue;
    assert(req.data[0] == 2);
    CanFrame f[4];
    deliver(p, f, answer(req, f), now + 2);
  }
  assert(p.channel(0).replies > 0 && p.channel(1).replies > 0);
  assert(!p.bundleStats().active && !p.takeStartedBundling());
}

int main() {
  testFrames();
  testSplit();
  testIsoTp();
  testRealSchedule();
  testScheduleSpreads();
  testBundledRequests();
  testFlowControlToThePhysicalAddress();
  testLeftOut();
  testFallBack();
  testBusyIsNotAFailure();
  testLateCycles();
  testFullRequestCarriesOver();
  testNothingAfterFlowControl();
  testLostPartWayIsATimeout();
  testCarGoingQuietKeepsMultiPid();
  testEngineStoppedKeepsMultiPid();
  testBusyConfirmsTheEcu();
  testNewSessionWithARequestInFlight();
  testOnePidOnly();
  printf("test_obdbundle: ok\n");
  return 0;
}
