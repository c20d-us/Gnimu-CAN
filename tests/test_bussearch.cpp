// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

// Finding the bus while waiting for a vehicle: listen windows, silence for
// gateway probes, confirmed traffic, discovery backoff, nothing to poll, and
// starting over when the bus goes quiet. Several of these cases were found on
// the bench before this logic had tests.

#include <assert.h>
#include <stdio.h>

#include "../code/GnimuCAN/src/core/BusSearch.h"

using namespace gnimu;

static const BusSearchConfig kConfig = {2000, 10000, {5000, 30000, 120000, 3}};
static const uint32_t k500 = 500000, k250 = 250000;

// Runs next() every 10 ms from `from` to `to` (bus clean, no probe wanted),
// switching windows when asked; returns the first action that isn't None or
// NextWindow, or None.
static BusAction runUntil(BusSearch& s, uint32_t from, uint32_t to, bool probeDue = false) {
  for (uint32_t t = from; t <= to; t += 10) {
    const BusAction a = s.next(t, true, probeDue && s.probeSilence());
    if (a == BusAction::NextWindow) {
      s.listen(BusSearch::otherRate(s.rate()), t);
    } else if (a != BusAction::None) {
      return a;
    }
  }
  return BusAction::None;
}

static void testGatewayedBoot() {
  // Silent at both rates in the boot listen: a probe may go straight away.
  BusSearch s(kConfig);
  s.bootListen(k500, 0, true, 300);
  assert(!s.probeSilence());
  s.bootListen(k250, 0, true, 600);
  assert(s.probeSilence());
  s.listen(k500, 600);
  assert(s.next(3000, true, true) == BusAction::Probe);
  // The probe answered: traffic at 500, found by probe, not by listening.
  assert(!s.confirmed(k500, false, 3100));
  assert(s.next(3100, true, false) == BusAction::Discover);
  assert(!s.heardByListening());
}

static void testHeardByListeningMeansNoProbes() {
  BusSearch s(kConfig);
  s.bootListen(k500, 12, true, 300);
  assert(s.confirmed(k500, true, 300));   // first time heard by listening
  assert(!s.confirmed(k500, true, 400));  // ... only once
  // The car switches off: quiet, then silence at both rates for a long time.
  s.ecuFound(true);
  s.vehicleLost();
  s.listen(s.rate(), 1000);
  assert(runUntil(s, 1000, 60000) == BusAction::None);
  assert(!s.probeSilence());  // never, this boot
}

// Found on the bench: losing the vehicle at 500 with the bus now at 250 sent
// a probe at 500 onto the live 250 bus within 20 ms.
static void testLiveBusAtOtherRateIsFoundBeforeAnyProbe() {
  BusSearch s(kConfig);
  s.bootListen(k500, 0, true, 300);
  s.bootListen(k250, 0, true, 600);
  s.confirmed(k500, false, 3000);  // a gatewayed car, found by probe
  s.ecuFound(true);
  // Polling at 500; the bus moves to 250 and the vehicle is lost.
  s.frame(5000);
  s.vehicleLost();
  s.listen(s.rate(), 7000);
  assert(!s.probeSilence());  // the frame at 5000 cleared the silence
  // The 500 window passes silent, then the 250 window hears frames.
  assert(s.next(9000, true, true && s.probeSilence()) == BusAction::NextWindow);
  assert(!s.probeSilence());  // only 500 is silent so far
  s.listen(k250, 9000);
  s.frame(9050);
  assert(s.next(9060, true, s.probeSilence()) == BusAction::Confirm);
}

static void testBusErrorsAreNotSilence() {
  BusSearch s(kConfig);
  s.listen(k500, 0);
  // A window with bus errors (traffic at the other rate) doesn't count.
  assert(s.next(2000, false, false) == BusAction::NextWindow);
  s.listen(k250, 2000);
  assert(s.next(4000, true, false) == BusAction::NextWindow);
  assert(!s.probeSilence());
  s.listen(k500, 4000);
  assert(s.next(6000, true, false) == BusAction::NextWindow);
  assert(s.probeSilence());
}

static void testDiscoveryBackoff() {
  BusSearch s(kConfig);
  s.confirmed(k500, true, 1000);
  uint32_t t = 1000;
  const uint32_t waits[] = {5000, 5000, 5000, 30000, 30000, 30000, 120000, 120000};
  for (uint32_t w : waits) {
    assert(s.next(t, true, false) == BusAction::Discover);
    s.frame(t);  // the bus keeps chattering
    assert(s.discoveryFailed(t) == w);
    s.frame(t + w - 1);
    assert(s.next(t + w - 1, true, false) == BusAction::None);
    t += w;
    s.frame(t);
  }
  assert(s.failures() == 8);
  // A car switching on (traffic again after quiet) starts from 5 s.
  s.quiet();
  s.confirmed(k500, true, t + 100);
  assert(s.discoveryFailed(t + 100) == 5000);
}

static void testNothingToPoll() {
  BusSearch s(kConfig);
  s.confirmed(k500, true, 0);
  s.ecuFound(false);
  assert(s.nothingToPoll());
  // Traffic continues: never asks again...
  for (uint32_t t = 10; t < 600000; t += 1000) {
    s.frame(t);
    assert(s.next(t, true, false) == BusAction::None);
  }
  // ... until the bus goes quiet, which starts over.
  assert(s.next(600000 + 10000, true, false) == BusAction::Quiet);
  s.quiet();
  assert(!s.nothingToPoll() && s.trafficRate() == 0);
}

static void testQuietStartsOver() {
  BusSearch s(kConfig);
  s.bootListen(k250, 5, true, 300);
  s.confirmed(k250, false, 300);  // say, found by probe: probes stay allowed
  s.frame(1000);
  assert(s.next(10999, true, false) != BusAction::Quiet);
  assert(s.next(11000, true, false) == BusAction::Quiet);
  s.quiet();
  // 10 s quiet at 250 counts as silent there; 500 still needs a window.
  assert(!s.probeSilence());
  s.listen(k250, 11000);
  assert(runUntil(s, 11000, 13000) == BusAction::None);  // the 250 window ends: switch to 500
  assert(runUntil(s, 13010, 15500, true) == BusAction::Probe);
  assert(s.silentForMs(15500) == 14500);
}

int main() {
  testGatewayedBoot();
  testHeardByListeningMeansNoProbes();
  testLiveBusAtOtherRateIsFoundBeforeAnyProbe();
  testBusErrorsAreNotSilence();
  testDiscoveryBackoff();
  testNothingToPoll();
  testQuietStartsOver();
  printf("test_bussearch: ok\n");
  return 0;
}
