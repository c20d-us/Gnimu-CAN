// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

// Sleep: when the awake device decides the car is parked or the battery low,
// and what a timer wake decides from one supply reading and a listen.

#include <assert.h>
#include <stdio.h>

#include "../code/GnimuCAN/src/core/SleepPolicy.h"

using namespace gnimu;

static const SleepPolicyConfig kPolicy = {300000, 10000, 5000, 6.0f};
static const WakeConfig kWake = {13.2f, 11.8f, 12.1f, 0.3f, 6.0f};

static SupplyStatus supply(float v, SupplyLevel l) { return {true, v, l}; }

// Parked: no fresh RPM, supply parked, bus silent, the whole hold time.
static SleepInputs parked() { return {false, false, supply(12.6f, SupplyLevel::Parked), 20000, false}; }

static void testParkedAfterHold() {
  SleepPolicy p(kPolicy);
  assert(p.update(parked(), 1000) == SleepDecision::None);
  assert(p.parked() && p.parkedForMs(61000) == 60000);
  assert(p.update(parked(), 300999) == SleepDecision::None);
  assert(p.update(parked(), 301000) == SleepDecision::Parked);
}

static void testAnythingAwakeRestartsTheHold() {
  SleepPolicy p(kPolicy);
  p.update(parked(), 0);
  // Each of these breaks "parked" for a moment, and the hold starts over.
  SleepInputs rpm = parked();
  rpm.rpmFresh = true;
  SleepInputs running = parked();
  running.supply = supply(14.2f, SupplyLevel::EngineRunning);
  SleepInputs traffic = parked();
  traffic.busSilentMs = 50;
  const SleepInputs breaks[] = {rpm, running, traffic};
  uint32_t t = 0;
  for (const SleepInputs& b : breaks) {
    t += 200000;
    assert(p.update(b, t) == SleepDecision::None);
    assert(!p.parked());
    assert(p.update(parked(), t + 1) == SleepDecision::None);
    assert(p.update(parked(), t + 299000) == SleepDecision::None);
  }
  assert(p.update(parked(), t + 300001) == SleepDecision::Parked);
}

static void testStayAwakeAndNoReading() {
  SleepPolicy p(kPolicy);
  SleepInputs w = parked();
  w.stayAwake = true;
  assert(p.update(w, 0) == SleepDecision::None);
  assert(p.update(w, 1000000) == SleepDecision::None);
  w.supply = supply(11.0f, SupplyLevel::BelowCutoff);
  assert(p.update(w, 1000001) == SleepDecision::None);  // W holds off the cutoff too
  SleepInputs none = parked();
  none.supply = {false, 0.0f, SupplyLevel::Unknown};
  assert(p.update(none, 2000000) == SleepDecision::None);
}

static void testCutoff() {
  SleepPolicy p(kPolicy);
  SleepInputs low = parked();
  low.supply = supply(11.6f, SupplyLevel::BelowCutoff);
  low.busSilentMs = 0;  // the cutoff doesn't wait for a silent bus
  // Its own hold: 5 s below the cutoff, continuously.
  assert(p.update(low, 0) == SleepDecision::None);
  assert(p.update(low, 4999) == SleepDecision::None);
  assert(p.update(low, 5000) == SleepDecision::Cutoff);
  // Not while the engine runs by RPM (a failing alternator, say)...
  low.rpmFresh = low.engineRunningRpm = true;
  assert(p.update(low, 5001) == SleepDecision::None);
  // ... but ignition on with the engine stopped is no excuse, and the hold
  // starts over.
  low.engineRunningRpm = false;
  assert(p.update(low, 5002) == SleepDecision::None);
  assert(p.update(low, 10002) == SleepDecision::Cutoff);
}

// A boot during a cranking dip: SupplyMonitor's first reading sets the level
// at once, but the cutoff still needs its hold, and recovering resets it.
static void testCrankingDipAtBoot() {
  SleepPolicy p(kPolicy);
  SleepInputs dip = parked();
  dip.supply = supply(9.5f, SupplyLevel::BelowCutoff);
  assert(p.update(dip, 0) == SleepDecision::None);
  assert(p.update(dip, 3000) == SleepDecision::None);
  SleepInputs ok = parked();
  ok.supply = supply(14.1f, SupplyLevel::EngineRunning);
  assert(p.update(ok, 3100) == SleepDecision::None);
  assert(p.update(dip, 6000) == SleepDecision::None);
  assert(p.update(dip, 10999) == SleepDecision::None);
}

// On USB with no 12 V: never sleeps, and the report can say why.
static void testUsbOnly() {
  SleepPolicy p(kPolicy);
  SleepInputs usb = parked();
  usb.supply = supply(0.4f, SupplyLevel::BelowCutoff);
  for (uint32_t t = 0; t < 1000000; t += 1000) assert(p.update(usb, t) == SleepDecision::None);
  assert(p.noSupply() && !p.parked());
  usb.supply = supply(12.6f, SupplyLevel::Parked);
  p.update(usb, 1000000);
  assert(!p.noSupply());
}

static void testWakeBoots() {
  WakeDecision d = decideWake(kWake, SleepKind::Parked, 12.6f, 12.6f, 3);
  assert(d.result == WakeResult::Boot && d.reason == WakeReason::Traffic);
  d = decideWake(kWake, SleepKind::Parked, 12.6f, 13.2f, 0);
  assert(d.result == WakeResult::Boot && d.reason == WakeReason::Running);
  // Intelligent alternator control: charging, but below 13.2 V.
  d = decideWake(kWake, SleepKind::Parked, 12.5f, 12.85f, 0);
  assert(d.result == WakeResult::Boot && d.reason == WakeReason::Rose);
  // Traffic or engine-running wakes a device asleep for the cutoff too.
  d = decideWake(kWake, SleepKind::Cutoff, 11.5f, 14.0f, 0);
  assert(d.result == WakeResult::Boot);
}

static void testWakeSleepsAgain() {
  // A small rise is noise, and the baseline follows a falling battery.
  WakeDecision d = decideWake(kWake, SleepKind::Parked, 12.6f, 12.75f, 0);
  assert(d.result == WakeResult::SleepParked && d.lowestV == 12.6f);
  d = decideWake(kWake, SleepKind::Parked, 12.6f, 12.4f, 0);
  assert(d.result == WakeResult::SleepParked && d.lowestV == 12.4f);
  // So surface charge decaying after a drive never counts as a rise...
  float lowest = 13.0f;
  const float decay[] = {12.95f, 12.88f, 12.8f, 12.72f, 12.66f, 12.63f};
  for (float v : decay) {
    d = decideWake(kWake, SleepKind::Parked, lowest, v, 0);
    assert(d.result == WakeResult::SleepParked);
    lowest = d.lowestV;
  }
  // ... but charging from the lowest point does.
  d = decideWake(kWake, SleepKind::Parked, lowest, lowest + 0.31f, 0);
  assert(d.result == WakeResult::Boot && d.reason == WakeReason::Rose);
}

static void testFirstCheckSetsTheBaseline() {
  // Just asleep: no baseline, so even a reading well above the awake one
  // doesn't count as a rise; it becomes the baseline instead.
  WakeDecision d = decideWake(kWake, SleepKind::Parked, kNoBaselineV, 12.9f, 0);
  assert(d.result == WakeResult::SleepParked && d.lowestV == 12.9f);
  d = decideWake(kWake, SleepKind::Parked, d.lowestV, 13.15f, 0);
  assert(d.result == WakeResult::SleepParked);
  d = decideWake(kWake, SleepKind::Parked, 12.9f, 13.19f, 0);
  assert(d.result == WakeResult::SleepParked);
  // The other rules still apply on the first check.
  assert(decideWake(kWake, SleepKind::Parked, kNoBaselineV, 13.4f, 0).result == WakeResult::Boot);
  assert(decideWake(kWake, SleepKind::Parked, kNoBaselineV, 12.6f, 5).result == WakeResult::Boot);
  assert(decideWake(kWake, SleepKind::Parked, kNoBaselineV, 11.6f, 0).result == WakeResult::SleepCutoff);
  // No 12 V: on USB, so boot and stay up, whichever way it slept.
  WakeDecision usb = decideWake(kWake, SleepKind::Cutoff, 11.5f, 0.4f, 0);
  assert(usb.result == WakeResult::Boot && usb.reason == WakeReason::NoSupply);
}

static void testWakeCutoffHysteresis() {
  // Parked, the battery falls below the cutoff: the long interval.
  WakeDecision d = decideWake(kWake, SleepKind::Parked, 12.0f, 11.7f, 0);
  assert(d.result == WakeResult::SleepCutoff);
  // Hovering between the cutoff and recovery stays in the cutoff.
  d = decideWake(kWake, SleepKind::Cutoff, 11.7f, 11.95f, 0);
  assert(d.result == WakeResult::SleepCutoff && d.lowestV == 11.7f);
  d = decideWake(kWake, SleepKind::Cutoff, 11.7f, 12.1f, 0);
  assert(d.result == WakeResult::SleepCutoff);
  // Clearly recovered: back to the parked interval, from a fresh baseline.
  d = decideWake(kWake, SleepKind::Cutoff, 11.7f, 12.15f, 0);
  assert(d.result == WakeResult::SleepParked && d.reason == WakeReason::Recovered && d.lowestV == 12.15f);
  // A rise during the cutoff doesn't boot (only 13.2 V or traffic does), so
  // a charger's slow climb can't flip it awake below the running level.
  d = decideWake(kWake, SleepKind::Cutoff, 11.2f, 11.9f, 0);
  assert(d.result == WakeResult::SleepCutoff);
}

int main() {
  testParkedAfterHold();
  testAnythingAwakeRestartsTheHold();
  testStayAwakeAndNoReading();
  testCutoff();
  testCrankingDipAtBoot();
  testUsbOnly();
  testWakeBoots();
  testWakeSleepsAgain();
  testFirstCheckSetsTheBaseline();
  testWakeCutoffHysteresis();
  printf("test_sleeppolicy: ok\n");
  return 0;
}
