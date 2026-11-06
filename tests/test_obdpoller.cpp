// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

// OBD-II polling: one request in flight, the most overdue channel first,
// replies and timeouts counted, unsupported PIDs skipped; and the vehicle
// activity rule built on what polling finds.

#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "../code/GnimuCAN/src/core/ObdPoller.h"
#include "../code/GnimuCAN/src/core/VehicleActivity.h"

using namespace gnimu;

static const uint8_t kPids[] = {0x0C, 0x11};
static const uint32_t kPeriods[] = {40, 40};
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

static void testSupportBitmap() {
  typedef ObdPoller<2> P;
  assert(P::isSupported(kBoth, 0x0C));
  assert(P::isSupported(kBoth, 0x11));
  assert(P::isSupported(kBoth, 0x05));
  assert(P::isSupported(kBoth, 0x20));
  assert(!P::isSupported(kBoth, 0x0B));
  assert(P::isSupported(0x80000000u, 0x01));
  assert(!P::isSupported(0xFFFFFFFFu, 0x00));
  assert(!P::isSupported(0xFFFFFFFFu, 0x21));
}

static void testOneInFlight() {
  ObdPoller<2> p(kPids, kPeriods, 50);
  p.start(0x7E0, kBoth, 1000);
  CanFrame req;
  assert(p.next(1000, req));
  assert(req.id == 0x7E0 && req.data[1] == 0x01);
  const uint8_t first = req.data[2];
  // Nothing more until it's answered.
  assert(!p.next(1001, req));
  // A reply for the other PID, or from a non-OBD ID, doesn't count.
  assert(!p.onFrame(reply(0x7E8, first == 0x0C ? 0x11 : 0x0C, 1, 2), 1002));
  assert(!p.onFrame(reply(0x100, first, 1, 2), 1002));
  assert(p.onFrame(reply(0x7E8, first, 0x0C, 0x80), 1003));
  // The other channel goes next, then nothing until a period has passed.
  assert(p.next(1004, req));
  assert(req.data[2] != first);
  assert(p.onFrame(reply(0x7E8, req.data[2], 0x0C, 0x80), 1005));
  assert(!p.next(1006, req));
  assert(p.next(1040, req));
  const size_t i = first == 0x0C ? 0 : 1;
  assert(p.channel(i).requests == 1 || p.channel(i).requests == 2);
  assert(p.channel(i).replies == 1 && p.channel(i).valid && p.channel(i).lastReplyMs == 1003);
}

static void testTimeout() {
  ObdPoller<2> p(kPids, kPeriods, 50);
  p.start(0x7E0, kBoth, 0);
  CanFrame req;
  assert(p.next(0, req));
  p.checkTimeout(49);
  assert(!p.next(49, req));
  p.checkTimeout(50);
  assert(p.next(50, req));
  uint32_t timeouts = p.channel(0).timeouts + p.channel(1).timeouts;
  assert(timeouts == 1);
  // A late reply after the timeout doesn't count.
  p.checkTimeout(100);
  assert(!p.onFrame(reply(0x7E8, req.data[2], 1, 2), 101));
  timeouts = p.channel(0).timeouts + p.channel(1).timeouts;
  assert(timeouts == 2);
  assert(p.silentForMs(2000) == 2000);
}

static void testUnsupportedSkipped() {
  ObdPoller<2> p(kPids, kPeriods, 50);
  p.start(0x7E1, 0x00100000u, 0);  // only PID 0x0C
  assert(p.channel(0).supported && !p.channel(1).supported);
  for (uint32_t t = 0; t < 1000; t += 10) {
    CanFrame req;
    if (p.next(t, req)) {
      assert(req.id == 0x7E1 && req.data[2] == 0x0C);
      assert(p.onFrame(reply(0x7E9, 0x0C, 0, 0), t + 1));
    }
  }
  assert(p.channel(1).requests == 0);
  assert(p.channel(0).requests == 25);  // every 40 ms over 1 s
}

// Two channels at different periods, polled on a 10 ms tick with instant
// replies, each get their own rate.
static void testFairShare() {
  const uint32_t periods[] = {20, 100};
  ObdPoller<2> p(kPids, periods, 50);
  p.start(0x7E0, kBoth, 0);
  for (uint32_t t = 0; t < 10000; t += 10) {
    CanFrame req;
    if (p.next(t, req)) assert(p.onFrame(reply(0x7E8, req.data[2], 0, 0), t + 2));
  }
  assert(p.channel(0).replies >= 450 && p.channel(0).replies <= 501);
  assert(p.channel(1).replies >= 95 && p.channel(1).replies <= 101);
}

static void testStartClears() {
  ObdPoller<2> p(kPids, kPeriods, 50);
  p.start(0x7E0, kBoth, 0);
  CanFrame req;
  assert(p.next(0, req));
  p.start(0x7E2, kBoth, 500);
  assert(p.channel(0).requests == 0 && p.channel(1).requests == 0);
  assert(!p.channel(0).valid && p.silentForMs(500) == 0);
  assert(p.next(500, req) && req.id == 0x7E2);
}

static void testDecode() {
  const uint8_t rpm[] = {0x0C, 0x80};  // 3200 / 4
  assert(fabsf(decodeRpm(rpm) - 800.0f) < 0.01f);
  const uint8_t full[] = {0xFF};
  assert(fabsf(decodePercent(full) - 100.0f) < 0.01f);
  const uint8_t none[] = {0};
  assert(decodePercent(none) == 0.0f);
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

int main() {
  testSupportBitmap();
  testOneInFlight();
  testTimeout();
  testUnsupportedSkipped();
  testFairShare();
  testStartClears();
  testDecode();
  testActivity();
  printf("test_obdpoller: ok\n");
  return 0;
}
