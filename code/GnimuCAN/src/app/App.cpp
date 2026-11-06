// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#include "App.h"

#include <Arduino.h>
#include <stdio.h>

#include "../core/BleJson.h"
#include "../core/DeviceId.h"
#include "../core/Obd2.h"
#include "../platform/Board.h"
#include "../platform/CanPort.h"
#include "../platform/HardwareId.h"
#include "../platform/ResetInfo.h"
#include "../platform/StatusLedPort.h"
#include "../platform/SupplyAdc.h"
#include "../platform/Watchdog.h"
#include "../protocol/frame_v1.h"
#include "BuildInfo.h"

namespace gnimu {

// C++11 needs a definition for a static constexpr array that's indexed at runtime.
constexpr JobSpec<App> App::kJobSpecs[App::kJobs];

namespace {

namespace frame = gnimu_frame::v1;

uint32_t nowMs() { return static_cast<uint32_t>(PlatformClock::nowUs() / 1000); }

// Gnimu-CAN fills the vehicle and power blocks.
constexpr uint8_t kCapabilities = frame::kCapVehicle | frame::kCapPower;

constexpr uint32_t otherRate(uint32_t bitrate) { return bitrate == 500000 ? 250000 : 500000; }

// The polled channels, in App's kRpmChannel, kThrottleChannel order.
const uint8_t kPollPids[] = {0x0C, 0x11};
const uint32_t kPollPeriodsMs[] = {config::kRpmPeriodMs, config::kThrottlePeriodMs};
const char* const kPollNames[] = {"RPM", "Throttle"};
const char* const kPollUnits[] = {"rpm", "%"};

void logSupply(Console& console, const SupplyStatus& s) {
  const unsigned long centiV = static_cast<unsigned long>(s.volts * 100.0f + 0.5f);
  console.log("Supply: %lu.%02luV, %s", centiV / 100, centiV % 100, supplyLevelName(s.level));
}

}  // namespace

App::App()
    : jobs_(this, kJobSpecs),
      scheduler_(jobs_.table()),
      supply_({board::kSupplyDivider, config::kSupplyCalibration, config::kSupplyEmaAlpha,
               config::kEngineRunningV, config::kLowVoltageCutoffV, config::kSupplyRunningHoldMs,
               config::kSupplyCutoffHoldMs}),
      probe_({config::kGatewayProbeMinV, config::kProbeHoldMs, config::kProbeBackoffMs,
              config::kProbeMaxAttempts}),
      poller_(kPollPids, kPollPeriodsMs, config::kObdReplyTimeoutMs),
      statusLed_({config::kLedBlinkHalfMs, config::kLedFaultHalfMs, config::kLedFlashOnMs,
                  config::kLedFlashPeriodMs}),
      diagnostics_(console_, scheduler_.jobs(), scheduler_.allStats(), kJobs, loops_, machine_, supply_,
                   canSummary_, obdSummary_, ble_, deviceId_, reset_) {
  // Here, not in the class: GCC only evaluates it once App is complete.
  static_assert(jobSpecsValid(kJobSpecs), "job specs out of id order, unnamed, or with a zero period");
  static_assert(kPollChannels <= kMaxObdChannels, "more polled channels than ObdSummary holds");
  static_assert(sizeof(kPollPids) == kPollChannels, "kPollPids out of step with the channel enum");
}

void App::begin() {
  // First, so boot is covered too: a hang anywhere from here on resets the
  // chip. Each boot step below feeds it.
  const bool watchdog = Watchdog::start();
  reset_ = makeResetRecord(ResetInfo::cause(), ResetInfo::takeRestartCode());
  PlatformClock::begin();
  StatusLedPort::begin();
  console_.begin();
  Watchdog::feed();
  decimalDeviceId(hardwareId(), deviceId_);
  diagnostics_.printVersion();
  console_.log("Gnimu-CAN starting");
  char reset[64];
  formatReset(reset, sizeof(reset), reset_);
  console_.log("Reset: %s", reset);
  if (watchdog) {
    console_.log("Watchdog: %lu ms \xE2\x9C\x85", static_cast<unsigned long>(config::kWatchdogMs));
  } else {
    console_.log("Watchdog: not started \xE2\x9D\x8C");
  }

  apply(machine_.boot());

  for (size_t i = 0; i < kJobs; i++) {
    if (scheduler_.enabled(i)) {
      console_.log("Job %s: every %lu us, budget %lu us", kJobSpecs[i].name,
                   static_cast<unsigned long>(kJobSpecs[i].periodUs),
                   static_cast<unsigned long>(kJobSpecs[i].budgetUs));
    } else {
      console_.log("Job %s: disabled", kJobSpecs[i].name);
    }
  }
  scheduler_.start();
  console_.log("%s.", stateName(machine_.state()));
  console_.log("Commands end with Enter: D = report, C = clear statistics counters, V = version, "
               "P = protocol, R = restart.");
  console_.log("L = 1s status on/off (up to %lu lines), N = N status reports (0-%lu).",
               static_cast<unsigned long>(config::kStatusMaxCount),
               static_cast<unsigned long>(config::kStatusMaxCount));
}

void App::apply(Transition t) {
  // Only Starting produces a follow-up, so this runs at most twice.
  while (t.kind == Transition::Kind::Enter) {
    Watchdog::feed();
    setJobs(jobsFor(t.to));
    t = enter(t.to);
  }
  if (t.kind == Transition::Kind::Restart) restart(t.why);
}

Transition App::enter(RunState s) {
  switch (s) {
    case RunState::Starting: {
      // The supply first: it decides nothing yet, but the boot log should
      // show it, and the first reading seeds the filter.
      SupplyAdc::begin();
      supply_.addSample(SupplyAdc::readPinMv(), nowMs());
      logSupply(console_, supply_.status());
      loggedLevel_ = supply_.status().level;
      Watchdog::feed();

      if (!startCan()) return machine_.started(FaultReason::CanFailed);
      Watchdog::feed();

      // BLE last: advertise only once there's something to stream.
      static char schema[kBleJsonMaxBytes];
      static char deviceInfo[kBleJsonMaxBytes];
      diagnostics_.formatSchema(schema, sizeof(schema));
      diagnostics_.formatDeviceInfo(deviceInfo, sizeof(deviceInfo));
      char name[32];
      snprintf(name, sizeof(name), "%s %s", config::kBleNamePrefix, deviceId_);
      const BleIdentity id = {name,          config::kBleNamePrefix,   config::kBleManufacturer,
                              deviceId_,     buildInfo().version,      board::kHardwareRevision,
                              schema,        deviceInfo};
      const bool advertising = ble_.begin(id, console_);
      return machine_.started(advertising ? FaultReason::None : FaultReason::BleFailed);
    }
    case RunState::WaitingForVehicle:
    case RunState::Polling:
      break;
    case RunState::Fault:
      // Fatal: log it and start nothing else. The console stays up so the
      // device still answers D and V.
      console_.log("%s", faultMessage(machine_.fault()));
      break;
  }
  return Transition::stay();
}

void App::setJobs(const JobSet& j) {
  scheduler_.setEnabled(kCanJob, j.can);
  scheduler_.setEnabled(kEmitJob, j.emit);
  scheduler_.setEnabled(kPowerJob, j.power);
  scheduler_.setEnabled(kBleJob, j.ble);
  scheduler_.setEnabled(kLedJob, j.led);
  scheduler_.setEnabled(kConsoleJob, j.console);
}

bool App::startCan() {
  canSummary_.maxProbes = config::kProbeMaxAttempts;
  CanPort::init();
  // The rate check only ever listens: the transceiver stays silent, and the
  // controller doesn't even acknowledge frames.
  const uint32_t rates[] = {500000, 250000};
  for (uint32_t rate : rates) {
    if (!CanPort::start(rate, true)) {
      console_.log("CAN: controller failed to start \xE2\x9D\x8C");
      return false;
    }
    const uint32_t startMs = nowMs();
    uint32_t frames = 0;
    CanFrame f;
    while (nowMs() - startMs < config::kCanBootListenMs) {
      while (CanPort::receive(f)) frames++;
      delay(5);
    }
    canSummary_.framesSeen += frames;
    if (frames > 0 && CanPort::status().busErrors == 0) {
      console_.log("CAN: controller started, listen-only \xE2\x9C\x85");
      confirmTraffic(rate);
      return true;
    }
    Watchdog::feed();
  }
  console_.log("CAN: controller started, listen-only \xE2\x9C\x85");
  console_.log("CAN: no traffic at 500 or 250 kbit/s - waiting for the vehicle");
  listenAt(500000);
  return true;
}

void App::listenAt(uint32_t bitrate) {
  CanPort::start(bitrate, true);
  listenBitrate_ = bitrate;
  windowStartMs_ = nowMs();
  windowFrames_ = 0;
}

void App::confirmTraffic(uint32_t bitrate) {
  canSummary_.trafficBitrate = bitrate;
  if (CanPort::status().bitrate != bitrate || !CanPort::status().listenOnly) CanPort::start(bitrate, true);
  listenBitrate_ = bitrate;
  console_.log("CAN: traffic at %lu kbit/s \xE2\x9C\x85", static_cast<unsigned long>(bitrate / 1000));
  // Ask for an ECU straight away.
  nextDiscoveryMs_ = nowMs();
  discoveryTries_ = 0;
}

void App::discover() {
  // Active from here: the rate is confirmed, by listening or by a probe the
  // car answered, so transmitting at it is safe.
  const uint32_t now = nowMs();
  CanPort::start(canSummary_.trafficBitrate, false);
  stepReplied_ = false;
  step_ = CanStep::Discovering;
  stepStartMs_ = now;
  if (!CanPort::send(obd2::mode01Request(0x00))) finishDiscovery(false, now);
}

void App::finishDiscovery(bool answered, uint32_t now) {
  step_ = CanStep::Idle;
  const uint32_t rate = canSummary_.trafficBitrate;
  if (!answered) {
    discoveryTries_++;
    if (discoveryTries_ >= config::kDiscoveryMaxTries) {
      console_.log("OBD: no ECU answered after %u tries - checking the bus again",
                   static_cast<unsigned>(discoveryTries_));
      canSummary_.trafficBitrate = 0;
      listenAt(rate);
    } else {
      console_.log("OBD: no ECU answered at %lu kbit/s - asking again in %lu s",
                   static_cast<unsigned long>(rate / 1000),
                   static_cast<unsigned long>(config::kDiscoveryRetryMs / 1000));
      CanPort::start(rate, true);
      nextDiscoveryMs_ = now + config::kDiscoveryRetryMs;
    }
    return;
  }

  const obd2::Mode01Reply& reply = stepReply_;
  const uint32_t bitmap = (static_cast<uint32_t>(reply.data[0]) << 24) | (static_cast<uint32_t>(reply.data[1]) << 16) |
                          (static_cast<uint32_t>(reply.data[2]) << 8) | reply.data[3];
  const uint32_t responseId = obd2::kFirstResponseId + reply.ecu;
  // An ECU's physical request ID is its response ID minus 8.
  const uint32_t requestId = responseId - 8;
  console_.log("OBD: ECU 0x%03lX answered, polling via 0x%03lX \xE2\x9C\x85", static_cast<unsigned long>(responseId),
               static_cast<unsigned long>(requestId));
  poller_.start(requestId, bitmap, now);
  for (size_t i = 0; i < kPollChannels; i++) {
    const PollChannel& c = poller_.channel(i);
    console_.log("OBD: PID %02X %s %s", static_cast<unsigned>(c.pid), kPollNames[i],
                 c.supported ? "supported \xE2\x9C\x85" : "not supported - not polled \xE2\x9D\x8C");
  }
  obdSummary_.ecuFound = true;
  obdSummary_.responseId = responseId;
  obdSummary_.requestId = requestId;
  discoveryTries_ = 0;
  // A car that comes back later in this boot gets the full probe budget.
  probe_.reset();
  canSummary_.probes = 0;
  canSummary_.lastProbe = ProbeResult::None;
  apply(machine_.vehicleFound());
}

// One run's worth of the request in progress: takes a reply the drain loop
// found, or moves on when the controller reports, or times out.
void App::continueStep(uint32_t now) {
  const uint32_t elapsed = now - stepStartMs_;
  switch (step_) {
    case CanStep::Idle:
      return;
    case CanStep::ProbeSending: {
      // A reply means the frame went out, whatever the alerts say.
      if (stepReplied_) {
        finishProbe(ProbeResult::Answered);
        return;
      }
      const TxOutcome tx = CanPort::txOutcome();
      if (tx == TxOutcome::Sent) {
        step_ = CanStep::ProbeWaiting;
        stepStartMs_ = now;
      } else if (tx == TxOutcome::Failed || elapsed >= config::kProbeTxConfirmMs) {
        finishProbe(ProbeResult::NoAck);
      }
      return;
    }
    case CanStep::ProbeWaiting:
      if (stepReplied_) {
        finishProbe(ProbeResult::Answered);
      } else if (elapsed >= config::kProbeReplyMs) {
        finishProbe(ProbeResult::NoReply);
      }
      return;
    case CanStep::Discovering:
      if (stepReplied_) {
        finishDiscovery(true, now);
      } else if (elapsed >= config::kObdReplyTimeoutMs) {
        finishDiscovery(false, now);
      }
      return;
  }
}

void App::vehicleLost() {
  console_.log("OBD: no replies for %lu s - waiting for the vehicle",
               static_cast<unsigned long>(config::kVehicleLostMs / 1000));
  obdSummary_.ecuFound = false;
  canSummary_.trafficBitrate = 0;
  listenAt(listenBitrate_);
  apply(machine_.vehicleLost());
}

void App::refreshObdSummary(uint32_t nowMs) {
  obdSummary_.channelCount = kPollChannels;
  for (size_t i = 0; i < kPollChannels; i++) {
    const PollChannel& c = poller_.channel(i);
    ObdChannelView& v = obdSummary_.channels[i];
    v.name = kPollNames[i];
    v.unit = kPollUnits[i];
    v.pid = c.pid;
    v.supported = c.supported;
    v.requests = c.requests;
    v.replies = c.replies;
    v.timeouts = c.timeouts;
    v.ageMs = c.valid ? nowMs - c.lastReplyMs : 0;
    v.fresh = obdSummary_.ecuFound && c.valid && v.ageMs <= config::kChannelStaleMs;
    v.value = !c.valid ? 0.0f : i == kRpmChannel ? decodeRpm(c.data) : decodePercent(c.data);
  }
  obdSummary_.activity = activity();
}

VehicleActivity App::activity() const {
  const ObdChannelView& rpm = obdSummary_.channels[kRpmChannel];
  return vehicleActivity(rpm.fresh, rpm.value, config::kEngineRunningMinRpm, supply_.status().level);
}

void App::makeProbe() {
  // The rate check found nothing to listen to, so this is the one place the
  // firmware transmits at a rate it hasn't confirmed: one single-shot frame,
  // with the transceiver silent again as soon as the probe is over.
  stepReplied_ = false;
  step_ = CanStep::ProbeSending;
  stepStartMs_ = nowMs();
  if (!CanPort::start(config::kGatewayProbeBitrate, false) || !CanPort::sendSingleShot(obd2::mode01Request(0x00)))
    finishProbe(ProbeResult::Error);
}

void App::finishProbe(ProbeResult r) {
  step_ = CanStep::Idle;
  CanPort::stop();
  const uint32_t rate = config::kGatewayProbeBitrate;
  probe_.record(r, nowMs());
  canSummary_.probes = probe_.attempts();
  canSummary_.lastProbe = r;
  console_.log("CAN: gateway probe %u of %u at %lu kbit/s: %s%s", static_cast<unsigned>(probe_.attempts()),
               static_cast<unsigned>(config::kProbeMaxAttempts), static_cast<unsigned long>(rate / 1000),
               probeResultText(r), r == ProbeResult::Answered ? " \xE2\x9C\x85" : "");
  if (r == ProbeResult::Answered) {
    confirmTraffic(rate);
    return;
  }
  if (probe_.exhausted()) console_.log("CAN: no more gateway probes until the next boot");
  listenAt(listenBitrate_);
}

void App::restart(RestartReason why) {
  console_.log("%s - restarting", restartReasonText(why));
  ResetInfo::storeRestartCode(restartCode(why));
  console_.flush();
  ESP.restart();
  for (;;) {
  }
}

void App::tick() {
  // Every pass, in every state: a job that never returns stops the feeding.
  Watchdog::feed();
  loops_++;
  PlatformClock::sleepUntilUs(scheduler_.tick());
}

// Every 10 ms while the device is up. Polling: matches replies, times out the
// request in flight, sends the next, and notices the car going quiet.
// Waiting: until clean traffic has been heard, keeps checking the bit rate
// and makes gateway probes when the policy allows; then asks for an ECU.
void App::canJob() {
  const uint32_t now = nowMs();
  const bool polling = machine_.state() == RunState::Polling;
  CanFrame f;
  while (CanPort::receive(f)) {
    canSummary_.framesSeen++;
    windowFrames_++;
    if (polling) {
      poller_.onFrame(f, now);
    } else if (step_ != CanStep::Idle && !stepReplied_) {
      obd2::Mode01Reply r;
      if (obd2::parseMode01Reply(f, 0x00, r) && r.len == 4) {
        stepReplied_ = true;
        stepReply_ = r;
      }
    }
  }

  if (polling) {
    poller_.checkTimeout(now);
    refreshObdSummary(now);
    if (poller_.silentForMs(now) >= config::kVehicleLostMs) {
      vehicleLost();
      return;
    }
    CanFrame request;
    if (poller_.next(now, request)) CanPort::send(request);
    return;
  }

  refreshObdSummary(now);
  if (step_ != CanStep::Idle) {
    continueStep(now);
    return;
  }
  if (canSummary_.trafficBitrate != 0) {
    if (static_cast<int32_t>(now - nextDiscoveryMs_) >= 0) discover();
    return;
  }
  if (windowFrames_ > 0 && CanPort::status().busErrors == 0) {
    confirmTraffic(listenBitrate_);
    return;
  }
  if (probe_.due(true, now)) {
    makeProbe();
    return;
  }
  if (now - windowStartMs_ >= config::kCanRateWindowMs) listenAt(otherRate(listenBitrate_));
}

// One Gnimu frame per period while a client is connected, whether or not
// anything changed: the app gets a steady rate whatever the car does.
void App::emitJob() {
  if (!ble_.connected()) return;
  frame::Encoder enc;
  enc.begin(static_cast<int32_t>(nowMs() & 0x7FFFFFFF), sequence_++, kCapabilities,
            frameStateCode(machine_.state()));
  const SupplyStatus& s = supply_.status();
  if (s.valid) enc.set(frame::Field::SupplyVoltage, s.volts);
  const ObdChannelView& rpm = obdSummary_.channels[kRpmChannel];
  const ObdChannelView& throttle = obdSummary_.channels[kThrottleChannel];
  if (rpm.fresh) enc.set(frame::Field::EngineSpeed, rpm.value);
  if (throttle.fresh) enc.set(frame::Field::ThrottlePosition, throttle.value);
  ble_.send(enc.finish(), frame::kFrameBytes);
}

// Every 100 ms, in every state: the supply is always watched.
void App::powerJob() {
  supply_.addSample(SupplyAdc::readPinMv(), nowMs());
  const SupplyStatus& s = supply_.status();
  probe_.supply(s.valid, s.volts, nowMs());
  if (s.level != loggedLevel_) {
    loggedLevel_ = s.level;
    logSupply(console_, s);
    // The cutoff's action (sleep) comes with the power work; until then it's
    // reported, not acted on.
    if (s.level == SupplyLevel::BelowCutoff) console_.log("Power: below cutoff - sleep not implemented yet");
  }
  const VehicleActivity a = activity();
  if (a != loggedActivity_) {
    loggedActivity_ = a;
    console_.log("Vehicle: %s", vehicleActivityName(a));
  }
}

void App::bleJob() { ble_.update(); }

void App::ledJob() {
  StatusLedPort::write(statusLed_.update(ledStateFor(machine_.state(), ble_.connected()), nowMs()));
}

void App::consoleJob() {
  const ConsoleCommand cmd = console_.poll();
  switch (cmd.kind) {
    case CommandKind::Clear:
      clearStats();
      break;
    case CommandKind::Restart:
      apply(machine_.restartRequested());
      break;
    case CommandKind::None:
    case CommandKind::Report:
    case CommandKind::Version:
    case CommandKind::Protocol:
    case CommandKind::ToggleStatus:
      break;
  }
  diagnostics_.handle(cmd);
  diagnostics_.tick(PlatformClock::nowUs());
}

void App::clearStats() {
  scheduler_.clearStats();
  loops_ = 0;
  ble_.clearMaxSend();
}

}  // namespace gnimu
