// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "../code/GnimuCAN/src/core/SupplyMonitor.h"

using namespace gnimu;

namespace {

// The 12V carrier's divider and thresholds, no calibration or smoothing, so
// each test controls exactly what the monitor sees.
SupplyConfig plain() { return {11.0f, 1.0f, 1.0f, 13.2f, 11.8f, 3000, 5000}; }

// Pin millivolts for a supply voltage through the ÷11 divider.
uint32_t pinMv(float supplyV) { return static_cast<uint32_t>(supplyV / 11.0f * 1000.0f + 0.5f); }

bool near(float a, float b) { return fabsf(a - b) < 0.02f; }

}  // namespace

static void testFirstReadingDecidesAtOnce() {
  SupplyMonitor m(plain());
  assert(!m.status().valid && m.status().level == SupplyLevel::Unknown);
  m.addSample(pinMv(14.2f), 0);
  assert(m.status().valid && near(m.status().volts, 14.2f));
  assert(m.status().level == SupplyLevel::EngineRunning);
}

static void testCalibrationScales() {
  SupplyConfig c = plain();
  c.calibration = 1.01f;
  SupplyMonitor m(c);
  m.addSample(pinMv(13.0f), 0);
  assert(near(m.status().volts, 13.13f));
}

static void testLevelChangesOnlyAfterHold() {
  SupplyMonitor m(plain());
  m.addSample(pinMv(12.5f), 0);
  assert(m.status().level == SupplyLevel::Parked);
  // The engine starts: above the threshold, but not yet for the hold time.
  m.addSample(pinMv(14.0f), 1000);
  m.addSample(pinMv(14.0f), 3900);
  assert(m.status().level == SupplyLevel::Parked);
  m.addSample(pinMv(14.0f), 4000);
  assert(m.status().level == SupplyLevel::EngineRunning);
}

static void testDipRestartsTheHold() {
  SupplyMonitor m(plain());
  m.addSample(pinMv(12.5f), 0);
  m.addSample(pinMv(14.0f), 1000);
  m.addSample(pinMv(12.6f), 2000);  // a dip back below the threshold
  m.addSample(pinMv(14.0f), 2500);
  m.addSample(pinMv(14.0f), 5000);  // 4 s since the first rise, but 2.5 s since the last
  assert(m.status().level == SupplyLevel::Parked);
  m.addSample(pinMv(14.0f), 5500);
  assert(m.status().level == SupplyLevel::EngineRunning);
}

static void testCutoffUsesItsOwnHold() {
  SupplyMonitor m(plain());
  m.addSample(pinMv(12.2f), 0);
  m.addSample(pinMv(11.5f), 1000);
  m.addSample(pinMv(11.5f), 5900);
  assert(m.status().level == SupplyLevel::Parked);
  m.addSample(pinMv(11.5f), 6000);
  assert(m.status().level == SupplyLevel::BelowCutoff);
}

static void testSmoothingFollowsGradually() {
  SupplyConfig c = plain();
  c.emaAlpha = 0.5f;
  SupplyMonitor m(c);
  m.addSample(pinMv(12.0f), 0);
  m.addSample(pinMv(14.0f), 100);
  assert(near(m.status().volts, 13.0f));
}

int main() {
  testFirstReadingDecidesAtOnce();
  testCalibrationScales();
  testLevelChangesOnlyAfterHold();
  testDipRestartsTheHold();
  testCutoffUsesItsOwnHold();
  testSmoothingFollowsGradually();
  printf("test_supplymonitor: ok\n");
  return 0;
}
