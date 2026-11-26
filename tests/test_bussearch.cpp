// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

// Finding the bus while waiting for a vehicle: listen windows, silence for
// gateway probes, confirmed traffic, and starting over when the bus goes
// quiet. Several of these cases were found on the bench before this logic had
// tests. Discovery has its own tests (test_obddiscovery.cpp).

#include <assert.h>
#include <stdio.h>

#include "../code/GnimuCAN/src/core/BusSearch.h"

using namespace gnimu;

static const BusSearchConfig kConfig = {2000, 10000};
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

// Listens at 500 then 250 from `at`, a whole silent window at each.
static void silentAtBoth(BusSearch& s, uint32_t at) {
  s.listen(k500, at);
  assert(s.next(at + 2000, true, false) == BusAction::NextWindow);
  s.listen(k250, at + 2000);
  assert(!s.probeSilence());  // only 500 so far
  assert(s.next(at + 4000, true, false) == BusAction::NextWindow);
  assert(s.probeSilence());
  s.listen(k500, at + 4000);
}

static void testGatewayedBoot() {
  // Silent at both rates: a probe may go.
  BusSearch s(kConfig);
  silentAtBoth(s, 0);
  assert(s.next(4010, true, true) == BusAction::Probe);
  // The probe answered: traffic at 500, found by probe, not by listening.
  s.confirmed(k500);
  assert(s.next(3100, true, false) == BusAction::None);  // the vehicle source takes over
  assert(s.trafficRate() == k500);
}

// Found on the first car test: a port heard earlier that falls silent at both
// rates gets probes.
static void testHeardThenSilentAllowsProbes() {
  BusSearch s(kConfig);
  s.listen(k500, 0);
  s.frame(5);
  assert(s.next(10, true, false) == BusAction::Confirm);
  s.confirmed(k500);
  s.vehicleLost();  // polled for a while, then the replies stopped
  s.listen(k500, 1000);
  assert(runUntil(s, 1000, 5000) == BusAction::None);  // a whole window at each rate
  assert(s.probeSilence());
  assert(runUntil(s, 5010, 5010, true) == BusAction::Probe);
}

// Found on the bench: losing the vehicle at 500 with the bus now at 250 sent
// a probe at 500 onto the live 250 bus within 20 ms.
static void testLiveBusAtOtherRateIsFoundBeforeAnyProbe() {
  BusSearch s(kConfig);
  silentAtBoth(s, 0);
  s.confirmed(k500);  // a gatewayed car, found by probe
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

// A 250 kbit/s car: bus errors at 500 move the search to 250 well before the
// window is up, and its first clean frame there confirms it.
static void testErrorsSwitchRatesEarly() {
  BusSearch s(kConfig);
  s.listen(k500, 0);
  assert(s.next(100, false, false) == BusAction::None);  // not on the first error
  assert(s.next(200, false, false) == BusAction::NextWindow);
  s.listen(k250, 200);
  s.frame(203);
  assert(s.next(210, true, false) == BusAction::Confirm);
}

// No 12V: CAN off, and back to nothing found.
static void testRestartForgetsTheBus() {
  BusSearch s(kConfig);
  s.listen(k500, 0);
  s.frame(5);
  s.confirmed(k500);
  s.restart();
  assert(s.trafficRate() == 0 && !s.probeSilence());
  s.listen(k500, 100);
  assert(s.next(110, true, false) == BusAction::None);  // no frame in this window yet
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

// Traffic confirmed: the bus search has nothing to do until the bus goes
// quiet, however long that takes (the vehicle source asks for ECUs meanwhile).
static void testConfirmedTrafficWaitsForQuiet() {
  BusSearch s(kConfig);
  s.confirmed(k500);
  for (uint32_t t = 10; t < 600000; t += 1000) {
    s.frame(t);
    assert(s.next(t, true, false) == BusAction::None);
  }
  assert(s.next(599010 + 10000, true, false) == BusAction::Quiet);
  s.quiet();
  assert(s.trafficRate() == 0);
}

static void testQuietStartsOver() {
  BusSearch s(kConfig);
  s.listen(k250, 0);
  s.frame(300);
  s.confirmed(k250);
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
  testHeardThenSilentAllowsProbes();
  testLiveBusAtOtherRateIsFoundBeforeAnyProbe();
  testErrorsSwitchRatesEarly();
  testRestartForgetsTheBus();
  testBusErrorsAreNotSilence();
  testConfirmedTrafficWaitsForQuiet();
  testQuietStartsOver();
  printf("test_bussearch: ok\n");
  return 0;
}
