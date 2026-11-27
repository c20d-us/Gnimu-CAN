// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "../core/ConsoleReply.h"
#include "../core/DeviceStatus.h"
#include "../core/BusSearch.h"
#include "../core/ObdDiscovery.h"
#include "../core/ObdPoller.h"
#include "../core/ProbePolicy.h"
#include "../core/ResetReason.h"
#include "../core/RunStateMachine.h"
#include "../core/Scheduler.h"
#include "../core/SleepPolicy.h"
#include "../core/StatusLed.h"
#include "../core/SupplyMonitor.h"
#include "../peripherals/ble/BlePeripheral.h"
#include "../platform/Clock.h"
#include "../platform/DeepSleep.h"
#include "Console.h"
#include "Diagnostics.h"
#include "SystemConfig.h"

namespace gnimu {

class App {
 public:
  App();
  void begin();
  void tick();

 private:
  // Job ids: indices into kJobSpecs and the scheduler, in priority order.
  enum : size_t { kCanJob, kEmitJob, kPowerJob, kBleJob, kLedJob, kConsoleJob, kJobs };

  void canJob();
  void emitJob();
  void powerJob();
  void bleJob();
  void bleCommand();
  void bleText(const char* echo, const char* text);
  void ledJob();
  void consoleJob();
  // One line per job, each naming its id; the constructor checks the table at
  // compile time (id order, names, positive periods).
  static constexpr JobSpec<App> kJobSpecs[kJobs] = {
      {kCanJob, "can", config::kCanPeriodUs, config::kCanBudgetUs, &App::canJob},
      {kEmitJob, "emit", config::kEmitPeriodUs, config::kEmitBudgetUs, &App::emitJob},
      {kPowerJob, "power", config::kPowerPeriodUs, config::kPowerBudgetUs, &App::powerJob},
      {kBleJob, "ble", config::kBlePeriodUs, config::kBleBudgetUs, &App::bleJob},
      {kLedJob, "led", config::kLedPeriodUs, config::kLedBudgetUs, &App::ledJob},
      {kConsoleJob, "console", config::kConsolePeriodUs, config::kConsoleBudgetUs, &App::consoleJob},
  };
  // Carries out a state-machine Transition: entering a state sets its jobs and
  // runs its entry effects, following Starting's bring-up result on into
  // WaitingForVehicle or Fault. Restart doesn't return.
  void apply(Transition t);
  // One state's fixed entry effects. Returns the follow-up transition
  // (Starting reports the bring-up result); Stay for the others.
  Transition enter(RunState s);
  void setJobs(const JobSet& j);
  // Boot: checks the controller starts. False if it won't.
  bool startCan();
  // A 12V supply is present (not USB alone): CAN may run.
  bool hasSupply() const;
  // The supply arrived: start looking for the bus. Or it left: stop CAN and
  // forget what was found, back to waiting.
  void canOn();
  void canOff();
  // Listens (silently) at `bitrate`, starting a new rate-check window.
  void listenAt(uint32_t bitrate);
  // Clean frames confirm a rate: heard by listening, or answered a probe.
  void confirmTraffic(uint32_t bitrate);
  // Gateway probe and ECU discovery each send a request and return; later
  // can-job runs take the replies or time them out (continueStep), so
  // nothing waits inside a job.
  void makeProbe();
  // Asks the bus for its ECUs (Mode 01 PID 0x00). When one that supports a
  // polled PID answers, polling starts.
  void discover();
  void continueStep(uint32_t nowMs);
  void finishProbe(ProbeResult r);
  void finishDiscovery(uint32_t nowMs);
  // The polled ECU stopped answering: back to waiting, and listening.
  void vehicleLost();
  // Waiting, with traffic confirmed, and the bus has gone quiet: the car has
  // switched off or gone to sleep. Starts over: rate check, probes,
  // discovery from its first interval.
  void trafficStopped();
  // Copies what polling knows into obdSummary_ for the reports and the frame.
  void refreshObdSummary(uint32_t nowMs);
  VehicleActivity activity() const;
  // The engine-speed channel's value; true when it's fresh.
  bool rpmFresh(float& rpm) const;
  void logSupport();
  void logBitmap(uint32_t responseId, uint8_t base, uint32_t bitmap);
  void logSchedule();
  // A timer wake from deep sleep: a quick supply reading and a listen on the
  // bus, then either back to sleep (never returns) or on into a full boot.
  void wakeCheck(SleepRecord r);
  // Goes into deep sleep for `kind`'s interval. Never returns.
  [[noreturn]] void goToSleep(SleepKind kind);
  static uint32_t checkIntervalS(SleepKind kind);
  [[noreturn]] void restart(RestartReason why);
  // Console C: the counters App owns.
  void clearStats();

  MemberJobs<App, kJobs> jobs_;
  Scheduler<kJobs, PlatformClock> scheduler_;
  BlePeripheral ble_;
  Console console_;
  char deviceId_[11] = {};  // 10 digits; also the BLE name suffix and DIS serial
  SupplyMonitor supply_;
  SupplyLevel loggedLevel_ = SupplyLevel::Unknown;
  ProbePolicy probe_;
  CanSummary canSummary_ = {};
  // Finding the bus, the gateway probe's silence and ECU discovery
  // (core/BusSearch.h); this App does the CAN calls it asks for.
  BusSearch search_;
  // Finding the ECU to poll: every reply to one request, then a choice
  // (core/ObdDiscovery.h).
  ObdDiscovery discovery_;
  // The OBD channels, in Config.h's kObdPolls order, then helper PIDs.
  ObdPoller<config::kObdChannelCount> poller_;
  bool supportLogged_ = false;  // this session's supported channels have been logged
  bool bitmapsLogged_ = false;  // the answering ECUs' PID 0x00 bitmaps: logged once a boot
  bool laterBitmapsLogged_ = false;  // ... and the chosen ECU's later ones
  bool askEveryone_ = false;    // the ECU answers only requests to 0x7DF: found this boot
  // The BLE Console (Gnimu protocol, docs/frame.md, Console).
  ConsoleReply<config::kBleReplyBytes> bleReply_;
  ConsoleEntry<config::kStatusMaxCount> bleEntry_;
  ConsoleGate bleGate_{config::kBleCommandGapMs};
  ObdSummary obdSummary_ = {};
  VehicleActivity loggedActivity_ = VehicleActivity::Unknown;
  // The request in progress, if any.
  enum class CanStep : uint8_t {
    Idle,
    ProbeSending,  // probe frame queued; waiting to hear it went out
    ProbeWaiting,  // probe acknowledged; waiting for an ECU's reply
    Discovering,  // discovery request sent; collecting every ECU's reply (discovery_)
  };
  CanStep step_ = CanStep::Idle;
  bool canOn_ = false;  // CAN is running: there's a 12V supply
  uint32_t stepStartMs_ = 0;
  bool stepReplied_ = false;  // a PID 0x00 reply arrived during a probe
  // Sleep.
  SleepPolicy sleepPolicy_;
  SleepView sleepView_ = {};
  bool stayAwake_ = false;      // the console's W
  bool wokeFromSleep_ = false;  // this boot follows a wake check that found the car awake
  SleepRecord wake_ = {};       // ... and what that check saw
  WakeReason wokeFor_ = WakeReason::None;
  float wokeAtV_ = 0.0f;
  StatusLed statusLed_;
  RunStateMachine machine_;
  uint8_t sequence_ = 0;  // the frame header's sequence number
  uint32_t loops_ = 0;
  ResetRecord reset_ = {};  // set first thing in begin()
  // Last: it references most of the members above.
  Diagnostics diagnostics_;
};

}  // namespace gnimu
