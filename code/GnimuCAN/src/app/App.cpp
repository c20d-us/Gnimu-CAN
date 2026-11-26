// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#include "App.h"

#include <Arduino.h>
#include <stdio.h>

#include "../core/BleJson.h"
#include "../core/DeviceId.h"
#include "../core/Obd2.h"
#include "../core/TextOut.h"
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


// The polled channels, in App's kRpmChannel, kThrottleChannel order.
const uint8_t kPollPids[] = {0x0C, 0x11};
const uint32_t kPollPeriodsMs[] = {config::kRpmPeriodMs, config::kThrottlePeriodMs};
const char* const kPollNames[] = {"RPM", "Throttle"};
const char* const kPollUnits[] = {"rpm", "%"};

void logSupply(Console& console, const SupplyStatus& s) {
  const CentiVolts v = centiVolts(s.volts);
  console.log("Supply: %lu.%02luV, %s", v.whole, v.hundredths, supplyLevelName(s.level));
}

}  // namespace

App::App()
    : jobs_(this, kJobSpecs),
      scheduler_(jobs_.table()),
      supply_({board::kSupplyDivider, config::kSupplyCalibration, config::kSupplyEmaAlpha,
               config::kEngineRunningV, config::kLowVoltageCutoffV, config::kSupplyRunningHoldMs,
               config::kSupplyCutoffHoldMs}),
      probe_({config::kGatewayProbeMinV, config::kProbeHoldMs, config::kProbeIntervalMs}),
      search_({config::kCanRateWindowMs, config::kBusSilentMs}),
      discovery_({config::kObdReplyTimeoutMs,
                  {config::kDiscoveryRetryMs, config::kDiscoveryBackoffMs, config::kDiscoveryMaxBackoffMs,
                   config::kDiscoveryTriesPerStep}}),
      poller_(kPollPids, kPollPeriodsMs, config::kObdReplyTimeoutMs),
      sleepPolicy_({config::kSleepAfterParkedS * 1000, config::kBusSilentMs, config::kSupplyCutoffHoldMs,
                    config::kNoSupplyV, config::kSleepGraceMs}),
      statusLed_({config::kLedBlinkHalfMs, config::kLedFaultHalfMs, config::kLedFlashOnMs,
                  config::kLedFlashPeriodMs}),
      diagnostics_(console_, scheduler_.jobs(), scheduler_.allStats(), kJobs, loops_, machine_, supply_,
                   canSummary_, obdSummary_, sleepView_, ble_, deviceId_, reset_) {
  // Here, not in the class: GCC only evaluates it once App is complete.
  static_assert(jobSpecsValid(kJobSpecs), "job specs out of id order, unnamed, or with a zero period");
  static_assert(kPollChannels <= kMaxObdChannels, "more polled channels than ObdSummary holds");
  static_assert(sizeof(kPollPids) == kPollChannels, "kPollPids out of step with the channel enum");
}

void App::begin() {
  // First, so boot is covered too: a hang anywhere from here on resets the
  // chip. Each boot step below feeds it.
  const bool watchdog = Watchdog::start();
  // Before anything visible: most timer wakes go straight back to sleep.
  SleepRecord sleepRecord;
  if (DeepSleep::takeRecord(sleepRecord)) wakeCheck(sleepRecord);
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
  // The sleep report's fixed fields: settings, and this boot's wake.
  sleepView_.noSupplyV = config::kNoSupplyV;
  sleepView_.sleepAfterMs = sleepPolicy_.config().parkedHoldMs;
  sleepView_.sleepCount = DeepSleep::sleepCount();
  sleepView_.woke = wokeFromSleep_;
  if (wokeFromSleep_) {
    sleepView_.wokeFor = wakeReasonText(wokeFor_);
    sleepView_.asleepFor = sleepKindName(wake_.kind);
    sleepView_.checks = wake_.checks;
    sleepView_.sleptAtV = wake_.sleptAtV;
    sleepView_.checkMinV = wake_.checkMinV;
    sleepView_.checkMaxV = wake_.checkMaxV;
    sleepView_.wokeAtV = wokeAtV_;
    const CentiVolts v = centiVolts(wokeAtV_), lo = centiVolts(wake_.lowestV);
    console_.log("Sleep: woke after %lu checks asleep %s: %s (%lu.%02luV, lowest %lu.%02luV) \xE2\x9C\x85",
                 static_cast<unsigned long>(wake_.checks), sleepKindName(wake_.kind), wakeReasonText(wokeFor_),
                 v.whole, v.hundredths, lo.whole, lo.hundredths);
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
               "P = protocol, R = restart, W = stay awake on/off.");
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
  canSummary_.noSupply = true;  // until canJob() sees 12V
  CanPort::init();
  // Only a check that the controller starts (a hardware fault otherwise). The
  // search for the bus begins in canJob() once there's a 12V supply.
  if (!CanPort::start(500000, true)) {
    console_.log("CAN: controller failed to start \xE2\x9D\x8C");
    return false;
  }
  CanPort::stop();
  console_.log("CAN: controller ok \xE2\x9C\x85");
  if (!hasSupply()) console_.log("CAN: no 12V supply (USB alone) - off until it arrives");
  return true;
}

bool App::hasSupply() const {
  const SupplyStatus& s = supply_.status();
  return s.valid && s.volts >= config::kNoSupplyV;
}

void App::canOn() {
  canOn_ = true;
  canSummary_.noSupply = false;
  console_.log("CAN: 12V supply - looking for the vehicle at 500 and 250 kbit/s");
  listenAt(500000);
}

void App::canOff() {
  canOn_ = false;
  canSummary_.noSupply = true;
  step_ = CanStep::Idle;
  CanPort::stop();
  search_.restart();
  canSummary_.trafficBitrate = 0;
  obdSummary_.ecuFound = false;
  obdSummary_.nothingToPoll = false;
  obdSummary_.answeredEcus = 0;
  console_.log("CAN: no 12V supply - off until it returns");
  apply(machine_.vehicleLost());
}

void App::listenAt(uint32_t bitrate) {
  CanPort::start(bitrate, true);
  search_.listen(bitrate, nowMs());
}

void App::confirmTraffic(uint32_t bitrate) {
  // Discovery starts straight away (canJob() asks once discovery_ is due).
  search_.confirmed(bitrate);
  discovery_.restart(nowMs());
  canSummary_.trafficBitrate = bitrate;
  if (CanPort::status().bitrate != bitrate || !CanPort::status().listenOnly) CanPort::start(bitrate, true);
  console_.log("CAN: traffic at %lu kbit/s \xE2\x9C\x85", static_cast<unsigned long>(bitrate / 1000));
}

void App::discover() {
  // Active from here: the rate is confirmed, by listening or by a probe the
  // car answered, so transmitting at it is safe.
  const uint32_t now = nowMs();
  CanPort::start(search_.trafficRate(), false);
  step_ = CanStep::Discovering;
  discovery_.begin(now);
  // Not queued: the round ends at once with no replies, and backs off.
  if (!CanPort::send(obd2::mode01Request(0x00))) finishDiscovery(now);
}

void App::finishDiscovery(uint32_t now) {
  step_ = CanStep::Idle;
  const uint32_t rate = search_.trafficRate();
  const EcuChoice c = discovery_.finish(kPollPids, kPollChannels, now);
  obdSummary_.answeredEcus = c.answered;
  if (c.answered == 0) {
    console_.log("OBD: no ECU answered at %lu kbit/s (try %lu) - asking again in %lu s",
                 static_cast<unsigned long>(rate / 1000), static_cast<unsigned long>(discovery_.failures()),
                 static_cast<unsigned long>(discovery_.retryInMs() / 1000));
    CanPort::start(rate, true);
    return;
  }
  char ids[obd2::kResponseIdsTextMax];
  obd2::formatResponseIds(ids, sizeof(ids), c.answered);
  console_.log("OBD: answered: %s", ids);

  if (!c.chosen) {
    // Nothing to ask for: stay waiting, and ask again after the backoff.
    console_.log("OBD: no ECU supports the polled PIDs (try %lu) - asking again in %lu s \xE2\x9D\x8C",
                 static_cast<unsigned long>(discovery_.failures()),
                 static_cast<unsigned long>(discovery_.retryInMs() / 1000));
    CanPort::start(rate, true);
    obdSummary_.nothingToPoll = true;
    return;
  }

  const uint32_t responseId = obd2::kFirstResponseId + c.ecu;
  // An ECU's physical request ID is its response ID minus 8.
  const uint32_t requestId = responseId - 8;
  poller_.start(requestId, c.bitmap, now);
  console_.log("OBD: ECU 0x%03lX chosen, polling via 0x%03lX \xE2\x9C\x85", static_cast<unsigned long>(responseId),
               static_cast<unsigned long>(requestId));
  for (size_t i = 0; i < kPollChannels; i++) {
    const PollChannel& ch = poller_.channel(i);
    console_.log("OBD: PID %02X %s %s", static_cast<unsigned>(ch.pid), kPollNames[i],
                 ch.supported ? "supported \xE2\x9C\x85" : "not supported - not polled \xE2\x9D\x8C");
  }
  obdSummary_.ecuFound = true;
  obdSummary_.nothingToPoll = false;
  obdSummary_.responseId = responseId;
  obdSummary_.requestId = requestId;
  // The probe count starts over; if the car is lost later, the first probe
  // may go as soon as the bus is silent.
  probe_.reset();
  canSummary_.probes = 0;
  canSummary_.lastProbe = ProbeResult::None;
  apply(machine_.vehicleFound());
}

// One run's worth of the request in progress: takes a reply the drain loop
// found, or moves on when the controller reports, or times out.
void App::continueStep(uint32_t now) {
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
      } else if (tx == TxOutcome::Failed || now - stepStartMs_ >= config::kProbeTxConfirmMs) {
        finishProbe(ProbeResult::NoAck);
      }
      return;
    }
    case CanStep::ProbeWaiting:
      if (stepReplied_) {
        finishProbe(ProbeResult::Answered);
      } else if (now - stepStartMs_ >= config::kProbeReplyMs) {
        finishProbe(ProbeResult::NoReply);
      }
      return;
    case CanStep::Discovering:
      if (discovery_.collected(now)) finishDiscovery(now);
      return;
  }
}

void App::vehicleLost() {
  console_.log("OBD: no replies for %lu s - waiting for the vehicle",
               static_cast<unsigned long>(config::kVehicleLostMs / 1000));
  obdSummary_.ecuFound = false;
  canSummary_.trafficBitrate = 0;
  search_.vehicleLost();
  listenAt(search_.rate());
  apply(machine_.vehicleLost());
}

void App::trafficStopped() {
  console_.log("CAN: no frames for %lu s - listening again",
               static_cast<unsigned long>(config::kBusSilentMs / 1000));
  canSummary_.trafficBitrate = 0;
  obdSummary_.nothingToPoll = false;
  obdSummary_.answeredEcus = 0;
  search_.quiet();
  listenAt(search_.rate());
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
  console_.log("CAN: gateway probe %lu at %lu kbit/s: %s%s", static_cast<unsigned long>(probe_.attempts()),
               static_cast<unsigned long>(rate / 1000), probeResultText(r),
               r == ProbeResult::Answered ? " \xE2\x9C\x85" : "");
  if (r == ProbeResult::Answered) {
    confirmTraffic(rate);
    return;
  }
  listenAt(search_.rate());
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
  // No 12V: on USB alone, on the bench or being flashed, not in a car. CAN
  // stays off until the supply arrives, and goes off again if it leaves.
  if (!hasSupply()) {
    if (canOn_) canOff();
    refreshObdSummary(now);
    return;
  }
  if (!canOn_) canOn();
  const bool polling = machine_.state() == RunState::Polling;
  CanFrame f;
  while (CanPort::receive(f)) {
    canSummary_.framesSeen++;
    search_.frame(now);
    if (polling) {
      poller_.onFrame(f, now);
    } else if (step_ != CanStep::Idle) {
      obd2::Mode01Reply r;
      if (obd2::parseMode01Reply(f, 0x00, r) && r.len == 4) {
        if (step_ == CanStep::Discovering) {
          discovery_.reply(r.ecu, (static_cast<uint32_t>(r.data[0]) << 24) | (static_cast<uint32_t>(r.data[1]) << 16) |
                                      (static_cast<uint32_t>(r.data[2]) << 8) | r.data[3]);
        } else {
          stepReplied_ = true;
        }
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
  // Bus errors matter only while still looking for traffic.
  const bool clean = search_.trafficRate() != 0 || CanPort::status().busErrors == 0;
  switch (search_.next(now, clean, probe_.due(search_.probeSilence(), now))) {
    case BusAction::None:
      // Traffic confirmed: the OBD side asks for its ECUs when due.
      if (search_.trafficRate() != 0 && discovery_.due(now)) discover();
      break;
    case BusAction::Confirm:
      confirmTraffic(search_.rate());
      break;
    case BusAction::Probe:
      makeProbe();
      break;
    case BusAction::NextWindow:
      listenAt(BusSearch::otherRate(search_.rate()));
      break;
    case BusAction::Quiet:
      // The car has switched off: start over, rather than keep asking a bus
      // nobody is on (whose requests nobody would acknowledge).
      trafficStopped();
      break;
  }
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
  }
  const VehicleActivity a = activity();
  if (a != loggedActivity_) {
    loggedActivity_ = a;
    console_.log("Vehicle: %s", vehicleActivityName(a));
  }

  const uint32_t now = nowMs();
  const ObdChannelView& rpm = obdSummary_.channels[kRpmChannel];
  const SleepInputs in = {rpm.fresh, rpm.fresh && rpm.value >= config::kEngineRunningMinRpm, s,
                          search_.silentForMs(now), stayAwake_};
  const SleepDecision d = sleepPolicy_.update(in, now);
  // What changes as it runs; the rest of sleepView_ is set once, in begin().
  sleepView_.stayAwake = stayAwake_;
  sleepView_.parked = sleepPolicy_.parked();
  sleepView_.noSupply = sleepPolicy_.noSupply();
  sleepView_.parkedForMs = sleepPolicy_.parkedForMs(now);
  if (d == SleepDecision::Parked) goToSleep(SleepKind::Parked);
  if (d == SleepDecision::Cutoff) goToSleep(SleepKind::Cutoff);
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
    case CommandKind::ToggleStayAwake:
      stayAwake_ = !stayAwake_;
      console_.log(stayAwake_ ? "Sleep: staying awake until W again or a restart"
                              : "Sleep: allowed again when parked");
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

uint32_t App::checkIntervalS(SleepKind kind) {
  return kind == SleepKind::Parked ? config::kParkedCheckIntervalS : config::kCutoffCheckIntervalS;
}

void App::wakeCheck(SleepRecord r) {
  // The supply first: averaged, then through the same calibration and
  // filter seed as a full boot uses.
  SupplyAdc::begin();
  uint64_t sumMv = 0;
  for (uint32_t i = 0; i < config::kWakeSupplySamples; i++) {
    sumMv += SupplyAdc::readPinMv();
    delayMicroseconds(500);
  }
  supply_.addSample(static_cast<uint32_t>(sumMv / config::kWakeSupplySamples), nowMs());
  const float volts = supply_.status().volts;

  // Then listen, at each bit rate in turn. Listen-only: the transceiver
  // stays silent, so a parked car sees nothing from the device.
  CanPort::init();
  uint32_t frames = 0;
  const uint32_t rates[] = {500000, 250000};
  for (uint32_t rate : rates) {
    if (frames > 0 || !CanPort::start(rate, true)) continue;
    const uint32_t startMs = nowMs();
    CanFrame f;
    while (nowMs() - startMs < config::kWakeListenMs) {
      while (CanPort::receive(f)) frames++;
      delay(2);
    }
    CanPort::stop();
  }

  r.checks++;
  if (volts < r.checkMinV) r.checkMinV = volts;
  if (volts > r.checkMaxV) r.checkMaxV = volts;
  // The check reads higher than the awake device (kWakeCheckOffsetV), so its
  // levels are raised to match; readings stay raw, so the D report shows the
  // real gap. The rise rule compares check readings only, so needs no offset.
  const float off = config::kWakeCheckOffsetV;
  const WakeConfig wc = {config::kEngineRunningV + off, config::kLowVoltageCutoffV + off,
                         config::kCutoffRecoverV + off, config::kWakeRiseV, config::kNoSupplyV};
  const WakeDecision d = decideWake(wc, r.kind, r.lowestV, volts, frames);
  r.lowestV = d.lowestV;
  if (d.result == WakeResult::Boot) {
    wokeFromSleep_ = true;
    wake_ = r;
    wokeFor_ = d.reason;
    wokeAtV_ = volts;
    if (frames > 0) search_.frame(nowMs());
    return;
  }
  r.kind = d.result == WakeResult::SleepParked ? SleepKind::Parked : SleepKind::Cutoff;
  DeepSleep::sleepFor(r, checkIntervalS(r.kind));
}

void App::goToSleep(SleepKind kind) {
  const SupplyStatus& s = supply_.status();
  const CentiVolts v = centiVolts(s.volts);
  if (kind == SleepKind::Parked) {
    console_.log("Sleep: parked for %lu s at %lu.%02luV - sleeping, checking every %lu s",
                 static_cast<unsigned long>(config::kSleepAfterParkedS), v.whole, v.hundredths,
                 static_cast<unsigned long>(checkIntervalS(kind)));
  } else {
    const CentiVolts cut = centiVolts(config::kLowVoltageCutoffV);
    console_.log("Sleep: supply %lu.%02luV below the %lu.%02luV cutoff - sleeping, checking every %lu s", v.whole,
                 v.hundredths, cut.whole, cut.hundredths, static_cast<unsigned long>(checkIntervalS(kind)));
  }
  console_.flush();
  CanPort::stop();
  StatusLedPort::write(false);
  // No rise baseline yet: the first wake check sets it (kNoBaselineV).
  const SleepRecord r = {kind, kNoBaselineV, s.volts, 99.0f, 0.0f, 0, DeepSleep::sleepCount() + 1};
  DeepSleep::sleepFor(r, checkIntervalS(kind));
}

void App::clearStats() {
  scheduler_.clearStats();
  loops_ = 0;
  ble_.clearMaxSend();
}

}  // namespace gnimu
