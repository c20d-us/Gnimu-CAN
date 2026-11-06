// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "../core/DeviceStatus.h"
#include "../core/ObdPoller.h"
#include "../core/ProbePolicy.h"
#include "../core/ResetReason.h"
#include "../core/RunStateMachine.h"
#include "../core/Scheduler.h"
#include "../core/StatusLed.h"
#include "../core/SupplyMonitor.h"
#include "../peripherals/ble/BlePeripheral.h"
#include "../platform/Clock.h"
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
  // Boot: listens at each bit rate in turn. False if the controller won't
  // start at all.
  bool startCan();
  // Listens (silently) at `bitrate`, starting a new rate-check window.
  void listenAt(uint32_t bitrate);
  // Clean frames in the current window confirm its rate.
  void confirmTraffic(uint32_t bitrate);
  // Gateway probe and ECU discovery each send one request and return; later
  // can-job runs take the reply or time it out (continueStep), so nothing
  // waits inside a job.
  void makeProbe();
  // Asks the bus for an ECU (Mode 01 PID 0x00). When one answers, polling
  // starts.
  void discover();
  void continueStep(uint32_t nowMs);
  void finishProbe(ProbeResult r);
  void finishDiscovery(bool answered, uint32_t nowMs);
  // The polled ECU stopped answering: back to waiting, and listening.
  void vehicleLost();
  // Copies what polling knows into obdSummary_ for the reports and the frame.
  void refreshObdSummary(uint32_t nowMs);
  VehicleActivity activity() const;
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
  uint32_t listenBitrate_ = 0;
  uint32_t windowStartMs_ = 0;
  uint32_t windowFrames_ = 0;
  // Polled channels, in the order of kPollPids.
  enum : size_t { kRpmChannel, kThrottleChannel, kPollChannels };
  ObdPoller<kPollChannels> poller_;
  ObdSummary obdSummary_ = {};
  uint32_t nextDiscoveryMs_ = 0;
  uint8_t discoveryTries_ = 0;
  VehicleActivity loggedActivity_ = VehicleActivity::Unknown;
  // The request in progress, if any.
  enum class CanStep : uint8_t {
    Idle,
    ProbeSending,  // probe frame queued; waiting to hear it went out
    ProbeWaiting,  // probe acknowledged; waiting for an ECU's reply
    Discovering,   // discovery request sent; waiting for an ECU's reply
  };
  CanStep step_ = CanStep::Idle;
  uint32_t stepStartMs_ = 0;
  bool stepReplied_ = false;          // a PID 0x00 reply arrived during the step
  obd2::Mode01Reply stepReply_ = {};  // ... and this is it
  StatusLed statusLed_;
  RunStateMachine machine_;
  uint8_t sequence_ = 0;  // the frame header's sequence number
  uint32_t loops_ = 0;
  ResetRecord reset_ = {};  // set first thing in begin()
  // Last: it references most of the members above.
  Diagnostics diagnostics_;
};

}  // namespace gnimu
