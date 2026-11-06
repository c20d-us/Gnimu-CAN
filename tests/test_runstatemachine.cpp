// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#include <assert.h>
#include <stdio.h>

#include "../code/GnimuCAN/src/core/ResetReason.h"
#include "../code/GnimuCAN/src/core/RunStateMachine.h"

using namespace gnimu;

static bool enters(const Transition& t, RunState s) { return t.kind == Transition::Kind::Enter && t.to == s; }

static void testBootWaitsForVehicle() {
  RunStateMachine m;
  assert(enters(m.boot(), RunState::Starting));
  assert(enters(m.started(FaultReason::None), RunState::WaitingForVehicle));
  assert(m.state() == RunState::WaitingForVehicle);
}

static void testBleFailureFaults() {
  RunStateMachine m;
  m.boot();
  assert(enters(m.started(FaultReason::BleFailed), RunState::Fault));
  assert(m.fault() == FaultReason::BleFailed);
  // A fault doesn't retry: the vehicle appearing changes nothing.
  assert(m.vehicleFound().kind == Transition::Kind::Stay);
}

static void testVehicleComesAndGoesWithoutRestart() {
  RunStateMachine m;
  m.boot();
  m.started(FaultReason::None);
  assert(enters(m.vehicleFound(), RunState::Polling));
  assert(m.vehicleFound().kind == Transition::Kind::Stay);
  assert(enters(m.vehicleLost(), RunState::WaitingForVehicle));
  assert(m.vehicleLost().kind == Transition::Kind::Stay);
  assert(enters(m.vehicleFound(), RunState::Polling));
}

static void testStartedOnlyOnce() {
  RunStateMachine m;
  m.boot();
  m.started(FaultReason::None);
  assert(m.started(FaultReason::BleFailed).kind == Transition::Kind::Stay);
  assert(m.state() == RunState::WaitingForVehicle);
}

static void testJobsPerState() {
  const JobSet starting = jobsFor(RunState::Starting);
  assert(!starting.emit && !starting.power && !starting.console);
  const JobSet waiting = jobsFor(RunState::WaitingForVehicle);
  assert(waiting.can && waiting.emit && waiting.power && waiting.ble && waiting.led && waiting.console);
  const JobSet fault = jobsFor(RunState::Fault);
  assert(!fault.can && !fault.emit && !fault.ble && fault.power && fault.led && fault.console);
}

static void testFrameStateCodes() {
  // The Gnimu frame's header codes: 1 starting, 2 running, 3 waiting, 4 fault.
  assert(frameStateCode(RunState::Starting) == 1);
  assert(frameStateCode(RunState::Polling) == 2);
  assert(frameStateCode(RunState::WaitingForVehicle) == 3);
  assert(frameStateCode(RunState::Fault) == 4);
}

static void testRestartCodeSurvivesOnlySoftwareResets() {
  const uint8_t code = restartCode(RestartReason::Requested);
  ResetRecord r = makeResetRecord(ResetCause::Software, code);
  assert(r.restartKnown && r.restart == RestartReason::Requested);
  char buf[64];
  formatReset(buf, sizeof(buf), r);
  assert(snprintf(nullptr, 0, "%s", buf) > 0);
  // After power-on the kept byte is garbage, so it's ignored.
  assert(!makeResetRecord(ResetCause::PowerOn, code).restartKnown);
  // Out of range is ignored too.
  assert(!makeResetRecord(ResetCause::Software, 200).restartKnown);
}

int main() {
  testBootWaitsForVehicle();
  testBleFailureFaults();
  testVehicleComesAndGoesWithoutRestart();
  testStartedOnlyOnce();
  testJobsPerState();
  testFrameStateCodes();
  testRestartCodeSurvivesOnlySoftwareResets();
  printf("test_runstatemachine: ok\n");
  return 0;
}
