// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#include "Diagnostics.h"

#include <stdio.h>

#include "../core/BleJson.h"
#include "../platform/Board.h"
#include "../platform/CanPort.h"
#include "../platform/Clock.h"
#include "../protocol/frame_v1.h"
#include "BuildInfo.h"
#include "SystemConfig.h"

namespace gnimu {

namespace {

namespace frame = gnimu_frame::v1;

// The next whole multiple of the status period in uptime, so RT steps by
// exactly one per line.
uint64_t nextStatusGridUs(uint64_t nowUs) {
  return (nowUs / config::kStatusPeriodUs + 1) * config::kStatusPeriodUs;
}

// The first line after arming lands a full period away, so its rate is
// measured over at least one whole second.
uint64_t firstStatusGridUs(uint64_t nowUs) {
  return nextStatusGridUs(nowUs + config::kStatusPeriodUs - 1);
}

// The fields Gnimu-CAN fills, for the schema's fill masks: the vehicle
// fields of the channels polled (Config.h's kObdPolls), and the supply.
uint32_t vehicleFillMask() {
  uint32_t m = 0;
  for (size_t i = 0; i < config::kObdPollCount; i++)
    if (config::kObdPolls[i].periodMs > 0) m |= 1u << frame::info(config::kObdPolls[i].field).bit;
  return m;
}
uint32_t powerFillMask() { return 1u << frame::info(frame::Field::SupplyVoltage).bit; }

const char* typeName(frame::Type t) {
  switch (t) {
    case frame::Type::U8:
      return "uint8";
    case frame::Type::I16:
      return "int16";
    case frame::Type::U16:
      return "uint16";
    case frame::Type::I32:
      return "int32";
    case frame::Type::F64:
      return "double";
  }
  return "?";
}

// Hundredths, rounded: the console prints integers only.

}  // namespace

void Diagnostics::handle(const ConsoleCommand& cmd) {
  switch (cmd.kind) {
    case CommandKind::None:
      break;
    case CommandKind::Report:
      printReport();
      break;
    case CommandKind::Clear:
      // The counters were just cleared, so the old baseline would read as a
      // backwards jump; measure the next line from now instead.
      if (statusRemaining_ != 0 || bleStatusRemaining_ != 0) {
        const uint64_t now = PlatformClock::nowUs();
        frameRate_.reset(ble_.stats().sent, now);
        for (size_t i = 0; i < obd_.channelCount; i++) channelRate_[i].reset(obd_.channels[i].replies, now);
      }
      break;
    case CommandKind::Version:
      printVersion();
      break;
    case CommandKind::Protocol:
      printProtocol();
      break;
    case CommandKind::Events:
      console_.printEvents(cmd.hasArg ? cmd.arg : 0);
      break;
    case CommandKind::Restart:
    case CommandKind::ToggleStayAwake:
      break;  // App acts on these
    case CommandKind::ToggleStatus:
      armStatus(statusRemaining_ != 0 ? 0 : config::kStatusMaxCount);
      break;
  }
  if (cmd.hasCount) armStatus(cmd.count);
}

size_t Diagnostics::tick(uint64_t nowUs, char* bleLine, size_t bleCap) {
  if ((statusRemaining_ == 0 && bleStatusRemaining_ == 0) || nowUs < nextStatusDueUs_) return 0;
  // One line for both outputs, so the rates are measured once per second.
  char line[kStatusLineMaxBytes];
  const size_t n = formatStatusLine(nowUs, line, sizeof(line));
  size_t bleLen = 0;
  if (statusRemaining_ > 0) {
    console_.print(line, n);
    statusRemaining_--;
  }
  if (bleStatusRemaining_ > 0) {
    bleLen = n < bleCap ? n : bleCap;
    memcpy(bleLine, line, bleLen);
    bleStatusRemaining_--;
  }
  nextStatusDueUs_ += config::kStatusPeriodUs;
  if (nextStatusDueUs_ <= nowUs) nextStatusDueUs_ = nextStatusGridUs(nowUs);
  return bleLen;
}

void Diagnostics::armBleStatus(uint32_t count) {
  const bool wasOff = statusRemaining_ == 0 && bleStatusRemaining_ == 0;
  bleStatusRemaining_ = count;
  if (count != 0 && wasOff) startStatusGrid();
}

size_t Diagnostics::formatBleReply(const ConsoleCommand& cmd, char* buf, size_t cap) {
  switch (cmd.kind) {
    case CommandKind::Report: {
      const JobTable jobs = {jobs_, stats_, jobCount_, loops_};
      return formatReport(buf, cap, jobs, snapshot(PlatformClock::nowUs()));
    }
    case CommandKind::Version:
      return formatVersion(buf, cap);
    case CommandKind::Protocol:
      return formatProtocol(buf, cap);
    case CommandKind::Events:
      return console_.formatEvents(buf, cap, cmd.hasArg ? cmd.arg : 0);
    case CommandKind::None:
    case CommandKind::Clear:
    case CommandKind::ToggleStatus:
    case CommandKind::Restart:
    case CommandKind::ToggleStayAwake:
      break;
  }
  return 0;
}

void Diagnostics::startStatusGrid() {
  const uint64_t now = PlatformClock::nowUs();
  frameRate_.reset(ble_.stats().sent, now);
  for (size_t i = 0; i < obd_.channelCount; i++) channelRate_[i].reset(obd_.channels[i].replies, now);
  nextStatusDueUs_ = firstStatusGridUs(now);
}

void Diagnostics::armStatus(uint32_t count) {
  const bool wasOff = statusRemaining_ == 0 && bleStatusRemaining_ == 0;
  statusRemaining_ = count;
  if (count != 0 && wasOff) startStatusGrid();
}

DeviceStatus Diagnostics::snapshot(uint64_t nowUs) const {
  DeviceStatus s = {};
  s.uptimeUs = nowUs;
  s.reset = reset_;
  s.state = machine_.state();
  s.supply = supply_.status();
  s.can = CanPort::status();
  s.canSummary = can_;
  s.obd = obd_;
  s.sleep = sleep_;
  s.ble.connected = ble_.connected();
  s.ble.stats = ble_.stats();
  s.consoleDropped = console_.dropped();
  return s;
}

void Diagnostics::printReport() {
  const JobTable jobs = {jobs_, stats_, jobCount_, loops_};
  console_.printReport(jobs, snapshot(PlatformClock::nowUs()));
}

size_t Diagnostics::formatStatusLine(uint64_t nowUs, char* buf, size_t cap) {
  StatusRates rates = {};
  rates.frameCentiHz = frameRate_.sampleCentiHz(ble_.stats().sent, nowUs);
  for (size_t i = 0; i < obd_.channelCount; i++)
    rates.channelCentiHz[i] = channelRate_[i].sampleCentiHz(obd_.channels[i].replies, nowUs);
  return formatStatus(buf, cap, snapshot(nowUs), rates);
}

void Diagnostics::printVersion() {
  static char text[1024];
  console_.print(text, formatVersion(text, sizeof(text)));
}

size_t Diagnostics::formatVersion(char* buf, size_t cap) const {
  detail::Out o{buf, cap, 0};
  char line[160];
  formatBuildLine(line, sizeof(line), buildInfo());
  o.put("%s\n", line);
  o.put("Device ID: %s\n", deviceId_);
  const CentiVolts probeV = centiVolts(config::kGatewayProbeMinV);
  o.put("Config: hardware %s, vehicle obd2, CAN auto 500/250 kbit/s, gateway probe at %lu.%02luV\n",
        board::kHardwareName, probeV.whole, probeV.hundredths);
  o.put("Config: BLE \"%s %s\", Gnimu frame v%u, %u bytes at %lu Hz\n", config::kBleNamePrefix,
        deviceId_, static_cast<unsigned>(frame::kSpecVersion), static_cast<unsigned>(frame::kFrameBytes),
        static_cast<unsigned long>(config::kEmitRateHz));
  const unsigned cal = static_cast<unsigned>(config::kSupplyCalibration * 1000.0f + 0.5f);
  const CentiVolts run = centiVolts(config::kEngineRunningV), cutoff = centiVolts(config::kLowVoltageCutoffV);
  o.put("Config: supply calibration %u.%03u, engine running %lu.%02luV, cutoff %lu.%02luV\n", cal / 1000,
        cal % 1000, run.whole, run.hundredths, cutoff.whole, cutoff.hundredths);
  return o.len;
}

void Diagnostics::printProtocol() {
  static char text[1024];
  console_.print(text, formatProtocol(text, sizeof(text)));
}

size_t Diagnostics::formatProtocol(char* buf, size_t cap) const {
  // The fields Gnimu-CAN fills, in packet order (by offset).
  struct Channel {
    char name[32];
    const char* type;
    unsigned offset;
    unsigned scale;
  };
  // Device time, then each polled channel and the supply, in packet order.
  Channel channels[config::kObdPollCount + 2] = {};
  size_t n = 0;
  auto add = [&](const char* name, const char* unit, frame::Field f) {
    Channel& c = channels[n++];
    snprintf(c.name, sizeof(c.name), unit ? "%s (%s)" : "%s", name, unit);
    c.type = typeName(frame::info(f).type);
    c.offset = frame::info(f).offset;
    c.scale = frame::info(f).scale;
  };
  snprintf(channels[n].name, sizeof(channels[n].name), "Device time (ms)");
  channels[n].type = "int32";
  channels[n].offset = static_cast<unsigned>(frame::kDeviceTimeMsOffset);
  channels[n++].scale = 1;
  for (size_t i = 0; i < config::kObdPollCount; i++) {
    if (config::kObdPolls[i].periodMs == 0) continue;
    const obd2::PidDecoder* d = obd2::decoderFor(config::kObdPolls[i].field);
    add(d->name, d->field == frame::Field::EngineSpeed ? nullptr : d->unit, d->field);
  }
  add("Supply voltage", "V", frame::Field::SupplyVoltage);
  // By offset, as the packet lays them out.
  for (size_t i = 1; i < n; i++)
    for (size_t j = i; j > 0 && channels[j].offset < channels[j - 1].offset; j--) {
      const Channel t = channels[j];
      channels[j] = channels[j - 1];
      channels[j - 1] = t;
    }
  // Laid out like AutoXDL's custom sensor form: the device settings, then one
  // row per channel with the form's columns in its order.
  size_t wName = 4;  // "Name"
  for (size_t i = 0; i < n; i++)
    if (detail::displayWidth(channels[i].name) > wName) wName = detail::displayWidth(channels[i].name);
  detail::Out o{buf, cap, 0};
  o.put("Gnimu CAN Protocol\n");
  o.put("  Service         %s\n", kGnimuServiceUuid);
  o.put("  Notify          %s\n", kGnimuFrameV1Uuid);
  o.put("  Packet length   %u\n\n", static_cast<unsigned>(frame::kFrameBytes));
  o.put("  ");
  o.left("Name", wName + 2);
  o.put("Type    Offset  Scale\n");
  for (size_t i = 0; i < n; i++) {
    const Channel& c = channels[i];
    o.put("  ");
    o.left(c.name, wName + 2);
    o.left(c.type, 6);
    o.right(static_cast<unsigned long>(c.offset), 8);
    o.right(static_cast<unsigned long>(c.scale), 7);
    o.put("\n");
  }
  return o.len;
}

size_t Diagnostics::formatSchema(char* buf, size_t cap) const {
  const FillMask fills[] = {
      {"vehicle", vehicleFillMask()},
      {"power", powerFillMask()},
  };
  return formatSchemaJson(buf, cap, frame::kSpecVersion, config::kBleNamePrefix, config::kEmitRateHz, fills,
                          sizeof(fills) / sizeof(fills[0]));
}

size_t Diagnostics::formatDeviceInfo(char* buf, size_t cap) const {
  char emit[12], run[12], cutoff[12], cal[12];
  snprintf(emit, sizeof(emit), "%lu", static_cast<unsigned long>(config::kEmitRateHz));
  const CentiVolts r = centiVolts(config::kEngineRunningV), c = centiVolts(config::kLowVoltageCutoffV);
  snprintf(run, sizeof(run), "%lu.%02lu", r.whole, r.hundredths);
  snprintf(cutoff, sizeof(cutoff), "%lu.%02lu", c.whole, c.hundredths);
  const unsigned k = static_cast<unsigned>(config::kSupplyCalibration * 1000.0f + 0.5f);
  snprintf(cal, sizeof(cal), "%u.%03u", k / 1000, k % 1000);
  // The PIDs polled, from Config.h's kObdPolls: "0C,11,...".
  char pids[3 * config::kObdChannelCount + 1] = "";
  size_t at = 0;
  for (size_t i = 0; i < config::kObdPollCount; i++) {
    if (config::kObdPolls[i].periodMs == 0) continue;
    const obd2::PidDecoder* d = obd2::decoderFor(config::kObdPolls[i].field);
    at += static_cast<size_t>(snprintf(pids + at, sizeof(pids) - at, "%s%02X", at ? "," : "", d->pid));
  }
  const CfgItem cfg[] = {
      {"hw", board::kHardwareName, true},
      {"vehicle", "obd2", true},
      {"can", "auto 500/250", true},
      {"probe_kbps", "500", false},
      {"emit_hz", emit, false},
      {"run_v", run, false},
      {"cutoff_v", cutoff, false},
      {"supply_cal", cal, false},
      {"pids", pids, true},
  };
  // The commands the BLE Console accepts: the report-only ones, and a number
  // for status lines.
  return formatDeviceInfoJson(buf, cap, buildInfo(), cfg, sizeof(cfg) / sizeof(cfg[0]),
                              "[\"D\",\"E\",\"V\",\"P\",\"n\"]");
}

}  // namespace gnimu
