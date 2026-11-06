// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#include <assert.h>
#include <stdio.h>

#include "../code/GnimuCAN/src/core/StatusLed.h"

using namespace gnimu;

static const LedTiming kTiming = {1000, 100, 100, 2000};

static void testStatesMapToPatterns() {
  assert(ledStateFor(RunState::Starting, false) == LedState::Off);
  assert(ledStateFor(RunState::WaitingForVehicle, true) == LedState::NoVehicle);
  assert(ledStateFor(RunState::Polling, false) == LedState::Advertising);
  assert(ledStateFor(RunState::Polling, true) == LedState::Connected);
  assert(ledStateFor(RunState::Fault, true) == LedState::Fault);
}

static void testNoVehicleFlashes() {
  StatusLed led(kTiming);
  assert(led.update(LedState::NoVehicle, 5000));   // a new pattern starts "on"
  assert(led.update(LedState::NoVehicle, 5099));
  assert(!led.update(LedState::NoVehicle, 5100));
  assert(!led.update(LedState::NoVehicle, 6999));
  assert(led.update(LedState::NoVehicle, 7000));   // the next flash
}

static void testAdvertisingBlinks() {
  StatusLed led(kTiming);
  assert(led.update(LedState::Advertising, 0));
  assert(!led.update(LedState::Advertising, 1000));
  assert(led.update(LedState::Advertising, 2000));
}

static void testFaultIsFast() {
  StatusLed led(kTiming);
  assert(led.update(LedState::Fault, 0));
  assert(!led.update(LedState::Fault, 100));
  assert(led.update(LedState::Fault, 200));
}

static void testConnectedIsSolid() {
  StatusLed led(kTiming);
  for (uint32_t ms = 0; ms < 5000; ms += 250) assert(led.update(LedState::Connected, ms));
}

int main() {
  testStatesMapToPatterns();
  testNoVehicleFlashes();
  testAdvertisingBlinks();
  testFaultIsFast();
  testConnectedIsSolid();
  printf("test_statusled: ok\n");
  return 0;
}
