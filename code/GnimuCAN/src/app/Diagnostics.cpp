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

// The fields Gnimu-CAN v1 fills, for the schema's fill masks.
constexpr frame::Field kVehicleFields[] = {frame::Field::EngineSpeed, frame::Field::ThrottlePosition};
constexpr frame::Field kPowerFields[] = {frame::Field::SupplyVoltage};

uint32_t fillMask(const frame::Field* fields, size_t n) {
  uint32_t m = 0;
  for (size_t i = 0; i < n; i++) m |= 1u << frame::info(fields[i]).bit;
  return m;
}

// Hundredths, rounded: the console prints integers only.
unsigned centi(float v) { return static_cast<unsigned>(v * 100.0f + 0.5f); }

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
      if (statusRemaining_ != 0) {
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
    case CommandKind::Restart:
      break;  // App acts on it
    case CommandKind::ToggleStatus:
      armStatus(statusRemaining_ != 0 ? 0 : config::kStatusMaxCount);
      break;
  }
  if (cmd.hasCount) armStatus(cmd.count);
}

void Diagnostics::tick(uint64_t nowUs) {
  if (statusRemaining_ == 0 || nowUs < nextStatusDueUs_) return;
  printStatus(nowUs);
  nextStatusDueUs_ += config::kStatusPeriodUs;
  if (nextStatusDueUs_ <= nowUs) nextStatusDueUs_ = nextStatusGridUs(nowUs);
  statusRemaining_--;
}

void Diagnostics::armStatus(uint32_t count) {
  statusRemaining_ = count;
  if (count != 0) {
    const uint64_t now = PlatformClock::nowUs();
    frameRate_.reset(ble_.stats().sent, now);
    for (size_t i = 0; i < obd_.channelCount; i++) channelRate_[i].reset(obd_.channels[i].replies, now);
    nextStatusDueUs_ = firstStatusGridUs(now);
  }
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
  s.ble.connected = ble_.connected();
  s.ble.stats = ble_.stats();
  s.consoleDropped = console_.dropped();
  return s;
}

void Diagnostics::printReport() {
  const JobTable jobs = {jobs_, stats_, jobCount_, loops_};
  console_.printReport(jobs, snapshot(PlatformClock::nowUs()));
}

void Diagnostics::printStatus(uint64_t nowUs) {
  StatusRates rates = {};
  rates.frameCentiHz = frameRate_.sampleCentiHz(ble_.stats().sent, nowUs);
  for (size_t i = 0; i < obd_.channelCount; i++)
    rates.channelCentiHz[i] = channelRate_[i].sampleCentiHz(obd_.channels[i].replies, nowUs);
  console_.printStatus(snapshot(nowUs), rates);
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
  const unsigned probeV = centi(config::kGatewayProbeMinV);
  o.put("Config: hardware %s, vehicle obd2, CAN auto 500/250 kbit/s, gateway probe at %u.%02uV\n",
        board::kHardwareName, probeV / 100, probeV % 100);
  o.put("Config: BLE \"%s %s\", Gnimu frame v%u, %u bytes at %lu Hz\n", config::kBleNamePrefix,
        deviceId_, static_cast<unsigned>(frame::kSpecVersion), static_cast<unsigned>(frame::kFrameBytes),
        static_cast<unsigned long>(config::kEmitRateHz));
  const unsigned cal = static_cast<unsigned>(config::kSupplyCalibration * 1000.0f + 0.5f);
  const unsigned run = centi(config::kEngineRunningV), cutoff = centi(config::kLowVoltageCutoffV);
  o.put("Config: supply calibration %u.%03u, engine running %u.%02uV, cutoff %u.%02uV\n", cal / 1000,
        cal % 1000, run / 100, run % 100, cutoff / 100, cutoff % 100);
  return o.len;
}

void Diagnostics::printProtocol() {
  char text[512];
  console_.print(text, formatProtocol(text, sizeof(text)));
}

size_t Diagnostics::formatProtocol(char* buf, size_t cap) const {
  // The fields Gnimu-CAN fills, in packet order (by offset).
  struct Channel {
    const char* name;
    const char* type;
    unsigned offset;
    unsigned scale;
  };
  const Channel channels[] = {
      {"Device time (ms)", "int32", static_cast<unsigned>(frame::kDeviceTimeMsOffset), 1},
      {"RPM", "int16", frame::info(frame::Field::EngineSpeed).offset, frame::info(frame::Field::EngineSpeed).scale},
      {"Throttle (%)", "int16", frame::info(frame::Field::ThrottlePosition).offset,
       frame::info(frame::Field::ThrottlePosition).scale},
      {"Supply voltage (V)", "uint16", frame::info(frame::Field::SupplyVoltage).offset,
       frame::info(frame::Field::SupplyVoltage).scale},
  };
  // Laid out like AutoXDL's custom sensor form: the device settings, then one
  // row per channel with the form's columns in its order.
  size_t wName = 4;  // "Name"
  for (const Channel& c : channels)
    if (detail::displayWidth(c.name) > wName) wName = detail::displayWidth(c.name);
  detail::Out o{buf, cap, 0};
  o.put("Gnimu CAN Protocol\n");
  o.put("  Service         %s\n", kGnimuServiceUuid);
  o.put("  Notify          %s\n", kGnimuFrameV1Uuid);
  o.put("  Packet length   %u\n\n", static_cast<unsigned>(frame::kFrameBytes));
  o.put("  ");
  o.left("Name", wName + 2);
  o.put("Type    Offset  Scale\n");
  for (const Channel& c : channels) {
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
      {"vehicle", fillMask(kVehicleFields, sizeof(kVehicleFields) / sizeof(kVehicleFields[0]))},
      {"power", fillMask(kPowerFields, sizeof(kPowerFields) / sizeof(kPowerFields[0]))},
  };
  return formatSchemaJson(buf, cap, frame::kSpecVersion, config::kBleNamePrefix, config::kEmitRateHz, fills,
                          sizeof(fills) / sizeof(fills[0]));
}

size_t Diagnostics::formatDeviceInfo(char* buf, size_t cap) const {
  char emit[12], run[12], cutoff[12], cal[12];
  snprintf(emit, sizeof(emit), "%lu", static_cast<unsigned long>(config::kEmitRateHz));
  const unsigned r = centi(config::kEngineRunningV), c = centi(config::kLowVoltageCutoffV);
  snprintf(run, sizeof(run), "%u.%02u", r / 100, r % 100);
  snprintf(cutoff, sizeof(cutoff), "%u.%02u", c / 100, c % 100);
  const unsigned k = static_cast<unsigned>(config::kSupplyCalibration * 1000.0f + 0.5f);
  snprintf(cal, sizeof(cal), "%u.%03u", k / 1000, k % 1000);
  const CfgItem cfg[] = {
      {"hw", board::kHardwareName, true},
      {"vehicle", "obd2", true},
      {"can", "auto 500/250", true},
      {"probe_kbps", "500", false},
      {"emit_hz", emit, false},
      {"run_v", run, false},
      {"cutoff_v", cutoff, false},
      {"supply_cal", cal, false},
      {"pids", "0C,11", true},
  };
  return formatDeviceInfoJson(buf, cap, buildInfo(), cfg, sizeof(cfg) / sizeof(cfg[0]));
}

}  // namespace gnimu
