// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

// The OBD-II request/reply codec and the gateway probe's policy.

#include <assert.h>
#include <stdio.h>

#include "../code/GnimuCAN/src/core/Obd2.h"
#include "../code/GnimuCAN/src/core/ProbePolicy.h"

using namespace gnimu;

static CanFrame frame(uint32_t id, uint8_t len, const uint8_t (&d)[8]) {
  CanFrame f = {id, false, len, {}};
  for (int i = 0; i < 8; i++) f.data[i] = d[i];
  return f;
}

static void testRequestIsPaddedSingleFrame() {
  const CanFrame f = obd2::mode01Request(0x0C);
  assert(f.id == 0x7DF && !f.extended && f.len == 8);
  const uint8_t want[8] = {0x02, 0x01, 0x0C, 0x55, 0x55, 0x55, 0x55, 0x55};
  for (int i = 0; i < 8; i++) assert(f.data[i] == want[i]);
  assert(obd2::mode01Request(0x11, 0x7E0).id == 0x7E0);
}

static void testParsesRpmReply() {
  // Engine ECU, RPM: 4 bytes of payload after the PCI (41 0C A B).
  const CanFrame f = frame(0x7E8, 8, {0x04, 0x41, 0x0C, 0x1A, 0xF8, 0x55, 0x55, 0x55});
  obd2::Mode01Reply r;
  assert(obd2::parseMode01Reply(f, 0x0C, r));
  assert(r.ecu == 0 && r.pid == 0x0C && r.len == 2 && r.data[0] == 0x1A && r.data[1] == 0xF8);
  assert(r.data[2] == 0 && r.data[3] == 0);  // only the valid bytes are copied
}

static void testParsesSecondEcu() {
  const CanFrame f = frame(0x7E9, 8, {0x06, 0x41, 0x00, 0xBE, 0x1F, 0xA8, 0x13, 0x55});
  obd2::Mode01Reply r;
  assert(obd2::parseMode01Reply(f, 0x00, r));
  assert(r.ecu == 1 && r.len == 4);
}

static void testRejectsOtherFrames() {
  obd2::Mode01Reply r;
  // Not a response ID.
  assert(!obd2::parseMode01Reply(frame(0x7DF, 8, {0x04, 0x41, 0x0C, 1, 2, 0, 0, 0}), 0x0C, r));
  assert(!obd2::parseMode01Reply(frame(0x7F0, 8, {0x04, 0x41, 0x0C, 1, 2, 0, 0, 0}), 0x0C, r));
  // Another PID.
  assert(!obd2::parseMode01Reply(frame(0x7E8, 8, {0x04, 0x41, 0x0D, 1, 2, 0, 0, 0}), 0x0C, r));
  // A negative response (7F 01 12).
  assert(!obd2::parseMode01Reply(frame(0x7E8, 8, {0x03, 0x7F, 0x01, 0x12, 0, 0, 0, 0}), 0x0C, r));
  // A first frame of a multi-frame reply (PCI 1x).
  assert(!obd2::parseMode01Reply(frame(0x7E8, 8, {0x10, 0x14, 0x49, 0x02, 0, 0, 0, 0}), 0x0C, r));
  // A payload length longer than the frame.
  assert(!obd2::parseMode01Reply(frame(0x7E8, 4, {0x06, 0x41, 0x0C, 1, 0, 0, 0, 0}), 0x0C, r));
  // An extended (29-bit) frame.
  CanFrame ext = frame(0x7E8, 8, {0x04, 0x41, 0x0C, 1, 2, 0, 0, 0});
  ext.extended = true;
  assert(!obd2::parseMode01Reply(ext, 0x0C, r));
}

static ProbePolicy policy() { return ProbePolicy({12.4f, 3000, 30000, 5}); }

static void testProbeNeedsHighSupplyHeldAndSilentBus() {
  ProbePolicy p = policy();
  p.supply(true, 12.3f, 0);
  assert(!p.due(true, 10000));  // below the threshold
  p.supply(true, 12.5f, 10000);
  assert(!p.due(true, 12999));  // not held long enough
  assert(p.due(true, 13000));
  assert(!p.due(false, 13000));  // traffic on the bus: no need
  p.supply(false, 0.0f, 13000);
  assert(!p.due(true, 20000));  // no valid reading
}

static void testProbeBacksOffAndStops() {
  ProbePolicy p = policy();
  p.supply(true, 14.0f, 0);
  uint32_t now = 3000;
  for (uint8_t i = 0; i < 5; i++) {
    assert(p.due(true, now));
    p.record(ProbeResult::NoAck, now);
    assert(p.attempts() == i + 1);
    assert(!p.due(true, now + 29999));  // backing off
    now += 30000;
  }
  assert(p.exhausted());
  assert(!p.due(true, now + 1000000));  // never again this boot
  assert(p.last() == ProbeResult::NoAck);
}

static void testDippingSupplyRestartsTheHold() {
  ProbePolicy p = policy();
  p.supply(true, 12.6f, 0);
  p.supply(true, 12.2f, 2000);
  p.supply(true, 12.6f, 2500);
  assert(!p.due(true, 5000));
  assert(p.due(true, 5500));
}

int main() {
  testRequestIsPaddedSingleFrame();
  testParsesRpmReply();
  testParsesSecondEcu();
  testRejectsOtherFrames();
  testProbeNeedsHighSupplyHeldAndSilentBus();
  testProbeBacksOffAndStops();
  testDippingSupplyRestartsTheHold();
  printf("test_can: ok\n");
  return 0;
}
