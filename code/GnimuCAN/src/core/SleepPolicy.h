// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdint.h>

#include "SupplyMonitor.h"

namespace gnimu {

// Why the device sleeps, which also sets how often it wakes to check
// (docs/architecture.md, Sleep and the cutoff).
enum class SleepKind : uint8_t {
  Parked,  // the car is parked: check often, to catch it starting
  Cutoff,  // the battery is low: check rarely, to protect it
};

inline const char* sleepKindName(SleepKind k) { return k == SleepKind::Parked ? "parked" : "below cutoff"; }

// What the awake device sees, once per power-job run.
struct SleepInputs {
  bool rpmFresh;          // an ECU answered RPM within the stale window
  bool engineRunningRpm;  // ... and at or above the running threshold
  SupplyStatus supply;
  uint32_t busSilentMs;   // since the last CAN frame of any kind
  bool stayAwake;         // the console's W
};

struct SleepPolicyConfig {
  uint32_t parkedHoldMs;  // parked this long, continuously, before sleeping
  uint32_t busSilentMs;   // no frames for this long counts as a silent bus
  uint32_t cutoffHoldMs;  // below the cutoff this long, continuously, before sleeping
  float noSupplyV;        // below: the device can only be running from USB
  uint32_t graceMs;       // after the supply arrives, no sleep decisions this long
};

enum class SleepDecision : uint8_t { None, Parked, Cutoff };

// When the awake device should go to sleep. Parked: no fresh RPM, the supply
// below the engine-running level, and a silent bus, all held for the parked
// time. A connected client doesn't keep it awake: a phone left in the car
// would otherwise hold it at full current indefinitely. Cutoff: the supply
// below the low-voltage cutoff for the cutoff hold, timed here so that it
// applies straight after boot too (SupplyMonitor sets its first level at
// once, so a boot during a cranking dip would otherwise sleep at once),
// unless fresh RPM says the engine is running, when the device's draw
// doesn't matter next to the car's own. Never below noSupplyV: there's no
// 12 V connected and the device is on USB, so there's no battery to protect
// (a bench session, or a laptop flashing it).
class SleepPolicy {
 public:
  explicit SleepPolicy(const SleepPolicyConfig& c) : c_(c) {}

  SleepDecision update(const SleepInputs& in, uint32_t nowMs) {
    noSupply_ = in.supply.valid && in.supply.volts < c_.noSupplyV;
    const bool supplied = in.supply.valid && !noSupply_;
    if (supplied && !supplied_) suppliedSinceMs_ = nowMs;
    supplied_ = supplied;
    // A grace after the supply arrives (power-up, or switched on with USB
    // already connected): the supply level lags the voltage by its hold
    // times, so it can still say "below cutoff" from 0V on USB, and the car's
    // voltage may still be ramping.
    if (in.stayAwake || !supplied || nowMs - suppliedSinceMs_ < c_.graceMs) {
      parked_ = false;
      low_ = false;
      return SleepDecision::None;
    }
    if (in.supply.level == SupplyLevel::BelowCutoff && !in.engineRunningRpm) {
      parked_ = false;
      if (!low_) {
        low_ = true;
        lowSinceMs_ = nowMs;
      }
      return nowMs - lowSinceMs_ >= c_.cutoffHoldMs ? SleepDecision::Cutoff : SleepDecision::None;
    }
    low_ = false;
    const bool parkedNow = !in.rpmFresh && in.supply.level != SupplyLevel::EngineRunning &&
                           in.supply.level != SupplyLevel::Unknown && in.busSilentMs >= c_.busSilentMs;
    if (!parkedNow) {
      parked_ = false;
      return SleepDecision::None;
    }
    if (!parked_) {
      parked_ = true;
      parkedSinceMs_ = nowMs;
    }
    return nowMs - parkedSinceMs_ >= c_.parkedHoldMs ? SleepDecision::Parked : SleepDecision::None;
  }

  // For the reports: whether the parked conditions hold, and for how long.
  bool parked() const { return parked_; }
  bool noSupply() const { return noSupply_; }
  uint32_t parkedForMs(uint32_t nowMs) const { return parked_ ? nowMs - parkedSinceMs_ : 0; }
  const SleepPolicyConfig& config() const { return c_; }

 private:
  SleepPolicyConfig c_;
  bool parked_ = false;
  uint32_t parkedSinceMs_ = 0;
  bool low_ = false;
  uint32_t lowSinceMs_ = 0;
  bool noSupply_ = false;
  bool supplied_ = false;
  uint32_t suppliedSinceMs_ = 0;
};

struct WakeConfig {
  float engineRunningV;  // at or above: the car is running
  float cutoffV;         // below: the battery is low
  float recoverV;        // above, after a cutoff: recovered (hysteresis)
  float riseV;           // this much above the lowest reading since sleeping: charging has started
  float noSupplyV;       // below: no 12 V, the device is on USB
};

enum class WakeResult : uint8_t {
  Boot,         // the car looks awake: boot fully
  SleepParked,  // sleep again, the short interval
  SleepCutoff,  // sleep again, the long interval
};

enum class WakeReason : uint8_t { None, Traffic, Running, Rose, Recovered, Low, Unchanged, NoSupply };

inline const char* wakeReasonText(WakeReason r) {
  switch (r) {
    case WakeReason::None:
      return "none";
    case WakeReason::Traffic:
      return "CAN traffic";
    case WakeReason::Running:
      return "supply at the engine-running level";
    case WakeReason::Rose:
      return "supply rose since sleeping";
    case WakeReason::Recovered:
      return "supply recovered above the cutoff";
    case WakeReason::Low:
      return "supply below the cutoff";
    case WakeReason::Unchanged:
      return "still parked";
    case WakeReason::NoSupply:
      return "no 12V supply - running from USB";
  }
  return "unknown";
}

// No baseline yet: the device has just gone to sleep. The awake device's own
// reading isn't used, because a wake check reads higher than the awake device
// does (0.05 V at 12.8 V, 0.13 V at 14.2 V on the bench: the awake draw,
// with BLE on, shifts the reading). The first check sets the baseline, so
// every comparison is between readings taken the same way.
constexpr float kNoBaselineV = 1000.0f;

struct WakeDecision {
  WakeResult result;
  WakeReason reason;
  float lowestV;  // the lowest wake-check reading since sleeping, for the next check
};

// The quick check on a timer wake: a supply reading and a brief listen on the
// bus, never a transmission, so a parked car sees nothing from the device.
// A gatewayed car with intelligent alternator control can run at 12.4-12.8V
// with a silent OBD bus, which looks parked; the rise rule catches the
// alternator starting to charge after a start. The baseline is the lowest
// wake-check reading since sleeping (kNoBaselineV before the first), so the
// slow fall of a parked battery's surface charge never looks like a rise.
// The cost: a car started before the first check can't trigger the rise
// rule, only the engine-running level.
inline WakeDecision decideWake(const WakeConfig& c, SleepKind asleepFor, float lowestV, float volts,
                               uint32_t frames) {
  const float lowest = volts < lowestV ? volts : lowestV;
  if (frames > 0) return {WakeResult::Boot, WakeReason::Traffic, lowest};
  // No 12 V: something else (USB) is powering the device, so stay up.
  if (volts < c.noSupplyV) return {WakeResult::Boot, WakeReason::NoSupply, lowest};
  if (volts >= c.engineRunningV) return {WakeResult::Boot, WakeReason::Running, lowest};
  if (asleepFor == SleepKind::Cutoff) {
    // Hysteresis: back to the parked interval only once clearly recovered.
    if (volts > c.recoverV) return {WakeResult::SleepParked, WakeReason::Recovered, volts};
    return {WakeResult::SleepCutoff, WakeReason::Low, lowest};
  }
  if (volts < c.cutoffV) return {WakeResult::SleepCutoff, WakeReason::Low, volts};
  if (volts >= lowestV + c.riseV) return {WakeResult::Boot, WakeReason::Rose, lowest};
  return {WakeResult::SleepParked, WakeReason::Unchanged, lowest};
}

}  // namespace gnimu
