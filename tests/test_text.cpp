// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

// The console's and BLE's text: the build line, the report and status line
// fit their buffers in the worst case, and the JSON documents come out as the
// Gnimu protocol specifies.

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include <initializer_list>

#include "../code/GnimuCAN/src/core/BleJson.h"
#include "../code/GnimuCAN/src/core/BuildInfo.h"
#include "../code/GnimuCAN/src/core/ConsoleHelp.h"
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
  d.obd.answeredEcus = 0xFF;
  d.obd.channelCount = kMaxObdChannels;
  for (size_t i = 0; i < kMaxObdChannels; i++) {
    ObdChannelView& c = d.obd.channels[i];
    c = {"Intake air", "\xC2\xB0" "C", 0xFF, ChannelState::Silent, UINT32_MAX, UINT32_MAX, UINT32_MAX,
         UINT32_MAX, UINT32_MAX, UINT32_MAX, true, UINT32_MAX, -16383.75f};
  }
  d.obd.activity = VehicleActivity::EngineRunning;
  d.sleep = {false, false, 99.0f, true, UINT32_MAX, UINT32_MAX, UINT32_MAX, true, "supply at the engine-running level",
             "below cutoff", UINT32_MAX, 99.99f, 99.99f, 99.99f, 99.99f};
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

// The widest a device's counters get in practice (a week of uptime and more):
// 8 or 9 digits for the counts, TEC and REC at their hardware maximum, the
// longest names and values. Every line below the job table stays within
// kReportLineColumns, so it reads unwrapped on a phone.
static DeviceStatus widestInPractice() {
  DeviceStatus d = worstCase();
  d.can = {true, true, true, 500000, 255, 255, 99999999, 99999999};
  d.canSummary.trafficBitrate = 500000;
  d.canSummary.framesSeen = 999999999;
  d.canSummary.probes = 99999999;
  d.canSummary.lastProbe = ProbeResult::NoReply;
  d.obd.requestId = obd2::kFunctionalRequestId;  // ", everyone" too
  for (size_t i = 0; i < kMaxObdChannels; i++) {
    ObdChannelView& c = d.obd.channels[i];
    c = {"Intake air", "km/h", 0xFF, i % 2 ? ChannelState::Silent : ChannelState::Polling, 99999, 99999999,
         99999999, 99999999, 9999, 9999, true, 99999999, -16383.75f};
  }
  d.ble.stats.sent = 999999999;
  d.ble.stats.failed = d.ble.stats.refused = 99999999;
  d.ble.stats.maxSendUs = 99999;
  d.ble.stats.mtu = 517;
  d.sleep.sleepCount = 99999999;
  d.sleep.checks = 99999999;
  return d;
}

// Display columns: UTF-8 continuation bytes take none.
static size_t columns(const char* s, size_t n) {
  size_t c = 0;
  for (size_t i = 0; i < n; i++)
    if ((static_cast<unsigned char>(s[i]) & 0xC0) != 0x80) c++;
  return c;
}

static void testReportLinesFitAPhone() {
  Job jobs[1] = {{"can", 5000, 2000, nullptr, nullptr}};
  JobStats stats[1] = {};
  const JobTable t = {jobs, stats, 1, 0};
  for (int nothingToPoll = 0; nothingToPoll < 2; nothingToPoll++) {
    DeviceStatus d = widestInPractice();
    if (nothingToPoll) {
      d.obd.ecuFound = false;
      d.obd.nothingToPoll = true;
    }
    static char buf[kReportMaxBytes];
    const size_t n = formatReport(buf, sizeof(buf), t, d);
    size_t start = 0, lines = 0;
    for (size_t i = 0; i < n; i++) {
      if (buf[i] != '\n') continue;
      const char first = buf[start];
      // The job table (rows, rules and the RT row) widens with its counters.
      if (first != '|' && first != '=' && first != '-') {
        if (columns(buf + start, i - start) > kReportLineColumns) {
          printf("too wide (%zu): %.*s\n", columns(buf + start, i - start), static_cast<int>(i - start), buf + start);
          assert(false);
        }
        lines++;
      }
      start = i + 1;
    }
    assert(lines > 20);
  }
  // The channel lines wrap onto an indented second line.
  static char buf[kReportMaxBytes];
  formatReport(buf, sizeof(buf), t, widestInPractice());
  assert(strstr(buf, "requests 99999999\n     replies 99999999") != nullptr);
  assert(strstr(buf, "NOT ANSWERING, every 99999 ms") != nullptr);
}

// The boot log's command help: every command, the number explained, the
// commands BLE refuses marked USB only, and every line within 73 columns.
static void testConsoleHelp() {
  char buf[512];
  const size_t n = formatConsoleHelp(buf, sizeof(buf), 3600);
  assert(n > 0 && n + 1 < sizeof(buf) && buf[n - 1] == '\n');
  for (const char* want : {"D report", "E event log", "E n", "V version", "P protocol", "A number 0-3600",
                           "0 stops", "USB only: C clear", "L status on/off (up to 3600 lines)", "R restart",
                           "W stay awake"})
    assert(strstr(buf, want) != nullptr);
  assert(strstr(buf, "N =") == nullptr);  // not an N command
  size_t start = 0;
  for (size_t i = 0; i < n; i++) {
    if (buf[i] != '\n') continue;
    assert(columns(buf + start, i - start) <= kReportLineColumns);
    start = i + 1;
  }
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
  // With the Console's commands.
  formatDeviceInfoJson(buf, sizeof(buf), b, cfg, 2, "[\"D\",\"E\"]");
  assert(strcmp(buf, "{\"v\":1,\"build\":{\"rev\":\"7f7efe5f4d6a\",\"utc\":\"2026-11-04T17:27:43Z\","
                     "\"target\":\"esp32c6\"},\"cfg\":{\"hw\":\"12v\",\"emit_hz\":20},"
                     "\"console\":[\"D\",\"E\"]}") == 0);
}

int main() {
  testBuildLine();
  testReportFits();
  testStatusLineFits();
  testReportLinesFitAPhone();
  testConsoleHelp();
  testSchemaJson();
  testDeviceInfoJson();
  printf("test_text: ok\n");
  return 0;
}
