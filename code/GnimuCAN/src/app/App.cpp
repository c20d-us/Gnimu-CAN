// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#include "App.h"

#include <Arduino.h>
#include <stdio.h>

#include "../core/BleJson.h"
#include "../core/ConsoleHelp.h"
#include "../core/DeviceId.h"
#include "../core/Obd2.h"
#include "../core/Report.h"
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


// The OBD channels: Config.h's kObdPolls, with each one's decoder, then a
// slot for each helper PID (core/Obd2Pids.h), polled only when an enabled
// channel subtracts it.
constexpr size_t kChannels = config::kObdChannelCount;
struct PollTable {
  PollChannelConfig channels[kChannels];
  const obd2::PidDecoder* decoders[kChannels];
  const obd2::PidDecoder* alts[kChannels];  // the second-choice PID's decoder, or nullptr
  bool fills[kChannels];      // fills its frame field (helpers don't)
  size_t minus[kChannels];    // the helper channel subtracted; kChannels for none
  uint8_t wanted[kChannels];  // enabled PIDs in table order: what discovery looks for
  size_t wantedCount;
  size_t rpm;                 // the engine-speed channel; kChannels if there's none
};

const PollTable& pollTable() {
  static const PollTable t = [] {
    PollTable p = {};
    p.rpm = kChannels;
    for (size_t h = 0; h < obd2::kHelperDecoderCount; h++) {
      const size_t i = config::kObdPollCount + h;
      p.decoders[i] = &obd2::kHelperDecoders[h];
      p.channels[i] = {obd2::kHelperDecoders[h].pid, 0, obd2::kHelperDecoders[h].length};
      p.minus[i] = kChannels;
    }
    for (size_t i = 0; i < config::kObdPollCount; i++) {
      const obd2::PidDecoder* d = obd2::decoderFor(config::kObdPolls[i].field);
      const uint32_t period = config::kObdPolls[i].periodMs;
      p.decoders[i] = d;
      p.alts[i] = obd2::altFor(d->field);
      p.channels[i] = {d->pid, period, d->length};
      p.fills[i] = true;
      p.minus[i] = kChannels;
      if (period > 0) p.wanted[p.wantedCount++] = d->pid;
      if (d->field == frame::Field::EngineSpeed) p.rpm = i;
      if (d->minusPid == 0) continue;
      for (size_t h = 0; h < obd2::kHelperDecoderCount; h++) {
        if (obd2::kHelperDecoders[h].pid != d->minusPid) continue;
        p.minus[i] = config::kObdPollCount + h;
        if (period > 0) p.channels[p.minus[i]].periodMs = config::kHelperPeriodMs;
      }
    }
    return p;
  }();
  return t;
}

// How long a channel's value counts as fresh: kStalePeriods of its period,
// and never less than kChannelStaleMs.
uint32_t staleAfterMs(uint32_t periodMs) {
  return periodMs * config::kStalePeriods > config::kChannelStaleMs ? periodMs * config::kStalePeriods
                                                                    : config::kChannelStaleMs;
}

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
                  {config::kDiscoveryRetryMs, config::kDiscoveryFastForMs / config::kDiscoveryRetryMs,
                   config::kDiscoverySlowRetryMs}}),
      // The request cap as a minimum gap, 1 ms under so the can job's tick
      // jitter can't make a request wait a whole extra tick.
      poller_(pollTable().channels, {config::kObdReplyTimeoutMs, 1000 / config::kMaxObdRequestsPerSec - 1,
                                     config::kSilentAfterTimeouts, config::kSilentRetryMs,
                                     config::kAskEveryoneAfterTimeouts, config::kObdCycleMs,
                                     static_cast<uint8_t>(config::kObdMaxPidsPerRequest),
                                     config::kBundleFallBackAfter}),
      sleepPolicy_({config::kSleepAfterParkedS * 1000, config::kBusSilentMs, config::kSupplyCutoffHoldMs,
                    config::kNoSupplyV, config::kSleepGraceMs}),
      statusLed_({config::kLedBlinkHalfMs, config::kLedFaultHalfMs, config::kLedFlashOnMs,
                  config::kLedFlashPeriodMs}),
      diagnostics_(console_, scheduler_.jobs(), scheduler_.allStats(), kJobs, loops_, machine_, supply_,
                   canSummary_, obdSummary_, sleepView_, ble_, deviceId_, reset_) {
  // Here, not in the class: GCC only evaluates it once App is complete.
  static_assert(jobSpecsValid(kJobSpecs), "job specs out of id order, unnamed, or with a zero period");
  for (size_t i = 0; i < kChannels; i++) {
    const obd2::PidDecoder* alt = pollTable().alts[i];
    if (alt) poller_.setAlternate(i, alt->pid, alt->length);
  }
  // Before falling back to one PID per request, RPM alone says whether the
  // engine is running.
  if (pollTable().rpm < kChannels) poller_.setCheckChannel(pollTable().rpm);
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
  // The command help, a log line per line, so the event log keeps it too.
  char help[512];
  const size_t helpLen = formatConsoleHelp(help, sizeof(help), config::kStatusMaxCount);
  for (size_t start = 0, i = 0; i < helpLen; i++) {
    if (help[i] != '\n') continue;
    console_.log("%.*s", static_cast<int>(i - start), help + start);
    start = i + 1;
  }
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
  const EcuChoice c = discovery_.finish(pollTable().wanted, pollTable().wantedCount, now);
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
  // What each answering ECU offers, once a boot: the same car each time.
  if (!bitmapsLogged_) {
    bitmapsLogged_ = true;
    for (uint8_t e = 0; e < 8; e++)
      if (c.answered & (1u << e)) logBitmap(obd2::kFirstResponseId + e, 0x00, discovery_.bitmap(e));
  }

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
  poller_.start(requestId, c.bitmap, now, askEveryone_);
  console_.log("OBD: ECU 0x%03lX chosen, polling via 0x%03lX \xE2\x9C\x85", static_cast<unsigned long>(responseId),
               static_cast<unsigned long>(poller_.sendId()));
  supportLogged_ = false;
  logSupport();
  obdSummary_.ecuFound = true;
  obdSummary_.nothingToPoll = false;
  obdSummary_.responseId = responseId;
  obdSummary_.requestId = poller_.sendId();
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

// Once every channel's support is known (the later bitmaps read), which are
// polled, once per vehicle session.
void App::logSupport() {
  if (supportLogged_ || !poller_.supportKnown()) return;
  supportLogged_ = true;
  // The chosen ECU's later bitmaps, those read, once a boot.
  if (!laterBitmapsLogged_) {
    laterBitmapsLogged_ = true;
    for (unsigned base = 0x20; base < 0x100; base += 0x20) {
      uint32_t bitmap = 0;
      if (poller_.laterBitmap(static_cast<uint8_t>(base), bitmap))
        logBitmap(poller_.requestId() + 8, static_cast<uint8_t>(base), bitmap);
    }
  }
  for (size_t i = 0; i < kChannels; i++) {
    const PollChannel& ch = poller_.channel(i);
    if (ch.state == ChannelState::Off) continue;
    console_.log("OBD: PID %02X %s %s", static_cast<unsigned>(ch.pid), pollTable().decoders[i]->name,
                 ch.asked() ? "supported \xE2\x9C\x85" : "not supported - not polled \xE2\x9D\x8C");
  }
}

// A support bitmap and the PIDs it lists, a console line at a time.
void App::logBitmap(uint32_t responseId, uint8_t base, uint32_t bitmap) {
  char text[obd2::kSupportLinesMax];
  const size_t len = obd2::formatSupportLines(text, sizeof(text), responseId, base, bitmap, kReportLineColumns);
  for (size_t start = 0, i = 0; i < len; i++) {
    if (text[i] != '\n') continue;
    console_.log("%.*s", static_cast<int>(i - start), text + start);
    start = i + 1;
  }
}

// Once multi-PID polling has started, the schedule it follows: which
// channels go how often, once per vehicle session.
void App::logSchedule() {
  if (!poller_.takeStartedBundling()) return;
  console_.log("OBD: several PIDs per request, one every %lu ms, the busiest with %u",
               static_cast<unsigned long>(config::kObdCycleMs), static_cast<unsigned>(poller_.bundleStats().busiest));
  // One line per cadence, fastest first.
  uint32_t shown = 0;
  for (;;) {
    uint32_t every = 0;
    for (size_t i = 0; i < kChannels; i++) {
      const uint32_t e = poller_.channel(i).every;
      if (e > shown && (every == 0 || e < every)) every = e;
    }
    if (every == 0) return;
    char names[160];
    size_t len = 0;
    names[0] = '\0';
    for (size_t i = 0; i < kChannels && len < sizeof(names); i++) {
      if (poller_.channel(i).every != every) continue;
      const int w = snprintf(names + len, sizeof(names) - len, "%s%s", len ? ", " : "", pollTable().decoders[i]->name);
      if (w > 0) len += static_cast<size_t>(w);
    }
    console_.log("OBD: every %lu ms: %s", static_cast<unsigned long>(every * config::kObdCycleMs), names);
    shown = every;
  }
}

void App::refreshObdSummary(uint32_t nowMs) {
  obdSummary_.channelCount = kChannels;
  for (size_t i = 0; i < kChannels; i++) {
    const PollChannel& c = poller_.channel(i);
    // The second-choice PID's decoder, when the channel is asking for it.
    const obd2::PidDecoder* alt = pollTable().alts[i];
    const obd2::PidDecoder& d = alt && c.pid == alt->pid ? *alt : *pollTable().decoders[i];
    ObdChannelView& v = obdSummary_.channels[i];
    v.name = d.name;
    v.unit = d.unit;
    v.pid = c.pid;
    v.state = c.state;
    v.periodMs = c.targetPeriodMs;
    v.requests = c.requests;
    v.replies = c.replies;
    v.timeouts = c.timeouts;
    v.replyAvgMs = c.replies ? c.replyMsTotal / c.replies : 0;
    v.replyMaxMs = c.replyMsMax;
    v.ageMs = c.valid ? nowMs - c.lastReplyMs : 0;
    v.fresh = obdSummary_.ecuFound && c.valid && v.ageMs <= staleAfterMs(c.targetPeriodMs);
    v.value = c.valid ? obd2::decode(d, c.data) : 0.0f;
    const size_t m = pollTable().minus[i];
    if (m < kChannels) {
      // A difference: fresh only while the subtracted helper is too.
      const PollChannel& h = poller_.channel(m);
      const bool helperFresh = h.valid && nowMs - h.lastReplyMs <= staleAfterMs(h.targetPeriodMs);
      v.fresh = v.fresh && helperFresh;
      if (h.valid) v.value -= obd2::decode(*pollTable().decoders[m], h.data);
    }
  }
  obdSummary_.bundles = poller_.bundleStats();
  obdSummary_.bundling = config::kObdMaxPidsPerRequest > 1;
  obdSummary_.activity = activity();
}

// Fresh RPM at or above the running threshold: the engine is running.
bool App::rpmFresh(float& rpm) const {
  if (pollTable().rpm >= kChannels) return false;
  const ObdChannelView& v = obdSummary_.channels[pollTable().rpm];
  rpm = v.value;
  return v.fresh;
}

VehicleActivity App::activity() const {
  float rpm = 0.0f;
  const bool fresh = rpmFresh(rpm);
  return vehicleActivity(fresh, rpm, config::kEngineRunningMinRpm, supply_.status().level);
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

// Every 5 ms while the device is up. Polling: matches replies, times out the
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
      // A reply's first frame: the rest follows once the ECU has flow control.
      CanFrame fc;
      if (poller_.takeFlowControl(fc)) CanPort::send(fc);
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
    if (poller_.takeSwitchedToEveryone()) {
      // Remembered for the rest of the boot: the same car, the same gateway.
      askEveryone_ = true;
      obdSummary_.requestId = poller_.sendId();
      console_.log("OBD: no replies to requests sent to 0x%03lX - asking everyone (0x%03lX) instead",
                   static_cast<unsigned long>(poller_.requestId()), static_cast<unsigned long>(poller_.sendId()));
    }
    if (poller_.checkAnswered()) {
      // Multi-PID requests went unanswered, one PID alone didn't: is the
      // engine running? With no RPM channel to tell by, the answer is enough.
      refreshObdSummary(now);
      float rpm = 0.0f;
      const bool running = pollTable().rpm >= kChannels || (rpmFresh(rpm) && rpm >= config::kEngineRunningMinRpm);
      poller_.settleCheck(running);
      if (!running) {
        console_.log("OBD: multi-PID requests unanswered with the engine stopped (%ld rpm) - keeping them",
                     static_cast<long>(rpm));
      }
    }
    if (poller_.takeFellBack()) {
      console_.log("OBD: multi-PID requests failed %u times in a row (%s) - one PID per request until the car is found again",
                   static_cast<unsigned>(config::kBundleFallBackAfter),
                   bundleFaultText(poller_.bundleStats().fault));
    }
    logSupport();
    logSchedule();
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
  for (size_t i = 0; i < kChannels; i++) {
    const ObdChannelView& v = obdSummary_.channels[i];
    if (v.fresh && pollTable().fills[i]) enc.set(pollTable().decoders[i]->field, v.value);
  }
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
  float rpm = 0.0f;
  const bool rpmIsFresh = rpmFresh(rpm);
  const SleepInputs in = {rpmIsFresh, rpmIsFresh && rpm >= config::kEngineRunningMinRpm, s,
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

// Every 10 ms: BLE housekeeping, and one notification of a Console reply
// when one is going out. One per run keeps the frames, notified by the emit
// job, ahead of it.
void App::bleJob() {
  ble_.update();
  if (!ble_.connected()) {
    // The client left: whatever it asked for goes with it.
    bleReply_.cancel();
    bleGate_.cancel();
    diagnostics_.armBleStatus(0);
    return;
  }
  if (bleReply_.sending()) {
    const char* data = nullptr;
    const size_t n = bleReply_.chunk(ble_.mtu(), data);
    switch (ble_.sendConsole(data, n)) {
      case ConsoleSend::Sent:
        bleReply_.sent(n);
        break;
      case ConsoleSend::Busy:
        break;  // the stack is full: the same chunk next run
      case ConsoleSend::NotListening:
        bleReply_.cancel();
        break;
    }
  } else {
    // The last command refused while that reply went out.
    char echo[kConsoleEchoBytes + 1];
    if (bleGate_.takeBusy(echo)) bleText(echo, "busy");
  }
}

// A one-line Console reply to the BLE client, after the echo line of the
// command it answers.
void App::bleText(const char* echo, const char* text) {
  bleReply_.begin();
  bleReply_.append(echo, strlen(echo));
  bleReply_.append(text, strlen(text));
  bleReply_.append("\n", 1);
  bleReply_.finish();
}

// A Console command a BLE client has written: report-only commands answered,
// anything else refused (Gnimu protocol, docs/frame.md, Console).
void App::bleCommand() {
  char raw[kBleCommandMax + 1];
  if (!ble_.takeCommand(raw)) return;
  // Every reply starts by naming the command it answers.
  char echo[kConsoleEchoBytes + 1];
  switch (bleGate_.command(raw, nowMs(), bleReply_.sending(), echo)) {
    case ConsoleGate::Action::Ignore:     // only CR or LF
    case ConsoleGate::Action::BusyLater:  // answered when the reply going out has finished
      return;
    case ConsoleGate::Action::BusyNow:
      bleText(echo, "busy");
      return;
    case ConsoleGate::Action::Run:
      break;
  }
  bleEntry_.reset();
  EntryResult r;
  for (size_t i = 0; raw[i] != '\0' && r.kind == EntryKind::None; i++) r = bleEntry_.feed(raw[i]);
  if (r.kind == EntryKind::None) r = bleEntry_.feed('\n');  // a write needs no line ending
  switch (r.kind) {
    case EntryKind::None:
      break;
    case EntryKind::Command:
      if (!reportOnly(r.cmd)) {
        bleText(echo, "not available over BLE without bonding");
      } else {
        ConsoleCommand cmd;
        cmd.kind = r.cmd;
        cmd.hasArg = r.hasArg;
        cmd.arg = r.arg;
        bleReply_.begin();
        bleReply_.append(echo, strlen(echo));
        bleReply_.commit(diagnostics_.formatBleReply(cmd, bleReply_.space(), bleReply_.room()));
        bleReply_.finish();
      }
      break;
    case EntryKind::Count:
      diagnostics_.armBleStatus(r.count);
      bleText(echo, r.count ? "status line once a second" : "status line off");
      break;
    case EntryKind::TooLarge:
      bleText(echo, "number out of range");
      break;
    case EntryKind::Unrecognised:
      bleText(echo, "unrecognised command");
      break;
  }
}

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
    case CommandKind::Events:
      break;
  }
  diagnostics_.handle(cmd);
  bleCommand();
  char line[kStatusLineMaxBytes];
  const size_t n = diagnostics_.tick(PlatformClock::nowUs(), line, sizeof(line));
  // A status line for the BLE client, unless a reply is still going out.
  if (n > 0 && !bleReply_.sending()) {
    bleReply_.begin();
    bleReply_.append(kStatusLineEcho, strlen(kStatusLineEcho));
    bleReply_.append(line, n);
    bleReply_.append("\n", 1);
    bleReply_.finish();
  }
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
