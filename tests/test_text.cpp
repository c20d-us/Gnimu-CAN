// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

// The console's and BLE's text: the build line, the report and status line
// fit their buffers in the worst case, and the JSON documents come out as the
// Gnimu protocol specifies.

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "../code/GnimuCAN/src/core/BleJson.h"
#include "../code/GnimuCAN/src/core/BuildInfo.h"
#include "../code/GnimuCAN/src/core/Report.h"
#include "../code/GnimuCAN/src/core/StatusLine.h"

using namespace gnimu;

static void testBuildLine() {
  const BuildInfo b = {"v0.1.0", "7f7efe5f4d6a", "2026-11-04T17:27:43Z", "esp32c6"};
  char buf[128];
  formatBuildLine(buf, sizeof(buf), b);
  assert(strcmp(buf, "BUILD,version=v0.1.0,revision=7f7efe5f4d6a,utc=2026-11-04T17:27:43Z,target=esp32c6") == 0);
}

// Every counter at its widest, so the formatted text is as long as it gets.
static DeviceStatus worstCase() {
  DeviceStatus d = {};
  d.uptimeUs = UINT64_MAX / 2;
  d.reset = makeResetRecord(ResetCause::Software, restartCode(RestartReason::Requested));
  d.state = RunState::WaitingForVehicle;
  d.supply = {true, 36.0f, SupplyLevel::EngineRunning};
  d.ble.connected = true;
  d.ble.stats.sent = d.ble.stats.failed = d.ble.stats.refused = d.ble.stats.maxSendUs = UINT32_MAX;
  d.ble.stats.mtu = UINT16_MAX;
  d.consoleDropped = UINT32_MAX;
  d.obd.ecuFound = true;
  d.obd.responseId = 0x7EF;
  d.obd.requestId = 0x7E7;
  d.obd.channelCount = kMaxObdChannels;
  for (size_t i = 0; i < kMaxObdChannels; i++) {
    ObdChannelView& c = d.obd.channels[i];
    c = {"Throttle", "rpm", 0xFF, true, UINT32_MAX, UINT32_MAX, UINT32_MAX, true, UINT32_MAX, 16383.75f};
  }
  d.obd.activity = VehicleActivity::EngineRunning;
  return d;
}

static void testReportFits() {
  Job jobs[5];
  JobStats stats[5];
  const char* names[] = {"emit", "power", "ble", "led", "console"};
  for (size_t i = 0; i < 5; i++) {
    jobs[i] = {names[i], 1000, 1000, nullptr, nullptr};
    stats[i].runs = stats[i].overruns = stats[i].skipped = stats[i].maxUs = stats[i].maxLateUs = UINT32_MAX;
    stats[i].totalUs = UINT64_MAX / 2;
  }
  const JobTable t = {jobs, stats, 5, UINT32_MAX};
  static char buf[kReportMaxBytes];
  const size_t n = formatReport(buf, sizeof(buf), t, worstCase());
  assert(n + 1 < sizeof(buf));  // room to spare: nothing was cut off
  assert(strstr(buf, "Console: dropped") != nullptr);
}

static void testStatusLineFits() {
  char buf[kStatusLineMaxBytes];
  StatusRates r = {};
  r.frameCentiHz = UINT32_MAX;
  for (size_t i = 0; i < kMaxObdChannels; i++) r.channelCentiHz[i] = UINT32_MAX;
  const size_t n = formatStatus(buf, sizeof(buf), worstCase(), r);
  assert(n + 1 < sizeof(buf));
  assert(buf[n - 1] == '\n');
}

static void testSchemaJson() {
  const FillMask fills[] = {{"vehicle", 3}, {"power", 1}};
  char buf[kBleJsonMaxBytes];
  formatSchemaJson(buf, sizeof(buf), 1, "Gnimu-CAN", 20, fills, 2);
  assert(strcmp(buf, "{\"v\":1,\"d\":\"Gnimu-CAN\",\"r\":20,\"fill\":{\"vehicle\":3,\"power\":1}}") == 0);
}

static void testDeviceInfoJson() {
  const BuildInfo b = {"v0.1.0", "7f7efe5f4d6a", "2026-11-04T17:27:43Z", "esp32c6"};
  const CfgItem cfg[] = {{"hw", "12v", true}, {"emit_hz", "20", false}};
  char buf[kBleJsonMaxBytes];
  formatDeviceInfoJson(buf, sizeof(buf), b, cfg, 2);
  assert(strcmp(buf, "{\"v\":1,\"build\":{\"rev\":\"7f7efe5f4d6a\",\"utc\":\"2026-11-04T17:27:43Z\","
                     "\"target\":\"esp32c6\"},\"cfg\":{\"hw\":\"12v\",\"emit_hz\":20}}") == 0);
}

int main() {
  testBuildLine();
  testReportFits();
  testStatusLineFits();
  testSchemaJson();
  testDeviceInfoJson();
  printf("test_text: ok\n");
  return 0;
}
