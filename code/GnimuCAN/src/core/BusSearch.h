// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdint.h>

#include "DiscoveryBackoff.h"

namespace gnimu {

// What the App should do next while waiting for a vehicle (docs/
// architecture.md, Bring-up).
enum class BusAction : uint8_t {
  None,
  Confirm,     // clean frames in this window: traffic at its rate
  Probe,       // make a gateway probe
  NextWindow,  // this window is over: listen at the other rate
  Discover,    // ask for an ECU
  Quiet,       // the bus has gone quiet: start over (call quiet())
};

struct BusSearchConfig {
  uint32_t windowMs;  // one listen window at a rate
  uint32_t quietMs;   // no frames this long, with traffic confirmed: the car is off
  DiscoveryBackoff backoff;
};

// Finding the car's bus while waiting for a vehicle: listen windows at each
// bit rate, silence for gateway probes, confirmed traffic, ECU discovery with
// its backoff, and starting over when the bus goes quiet. Pure logic: the App
// starts the controller, sends and receives, and reports here; next() says
// what to do. The probe's own timing (supply hold, backoff, budget) stays in
// ProbePolicy; this says only whether the bus is silent enough for one.
class BusSearch {
 public:
  explicit BusSearch(const BusSearchConfig& c) : c_(c) {}

  // Listening at `rate` from now: a new window. Silence already heard stays.
  void listen(uint32_t rate, uint32_t nowMs) {
    rate_ = rate;
    windowStartMs_ = nowMs;
    windowFrames_ = 0;
  }

  // The boot listen at `rate`: `frames` heard, `clean` with no bus errors. A
  // clean silent listen counts as a silent window at that rate.
  void bootListen(uint32_t rate, uint32_t frames, bool clean, uint32_t nowMs) {
    rate_ = rate;
    if (frames > 0) {
      lastFrameMs_ = nowMs;
      silentAt_ = 0;
    } else if (clean) {
      silentAt_ |= rateBit(rate);
    }
  }

  // Any frame received, in any state.
  void frame(uint32_t nowMs) {
    lastFrameMs_ = nowMs;
    windowFrames_++;
    silentAt_ = 0;
  }

  // Traffic confirmed at `rate`, by listening or by a probe the car answered:
  // discovery starts now, from its first interval. True the first time this
  // boot that traffic is heard by listening (no more probes from then on).
  bool confirmed(uint32_t rate, bool byListening, uint32_t nowMs) {
    rate_ = rate;
    trafficRate_ = rate;
    nextDiscoveryMs_ = nowMs;
    failures_ = 0;
    nothingToPoll_ = false;
    const bool first = byListening && !heardByListening_;
    if (byListening) heardByListening_ = true;
    return first;
  }

  // No ECU answered discovery: the wait before asking again.
  uint32_t discoveryFailed(uint32_t nowMs) {
    failures_++;
    const uint32_t waitMs = discoveryRetryMs(c_.backoff, failures_);
    nextDiscoveryMs_ = nowMs + waitMs;
    return waitMs;
  }

  // An ECU answered discovery. With nothing to poll, discovery stops until the
  // bus goes quiet: polling would lose the "vehicle" in 2 s and find it again,
  // round and round.
  void ecuFound(bool anythingToPoll) {
    failures_ = 0;
    nothingToPoll_ = !anythingToPoll;
  }

  // Polling, and the ECU stopped answering: back to looking for traffic.
  // Then listen() at rate().
  void vehicleLost() { trafficRate_ = 0; }

  // What to do now, while waiting with no request in progress. `clean`: no
  // bus errors since the window began. `probeDue`: ProbePolicy allows a probe
  // now, given probeSilence().
  BusAction next(uint32_t nowMs, bool clean, bool probeDue) {
    if (trafficRate_ != 0) {
      if (nowMs - lastFrameMs_ >= c_.quietMs) return BusAction::Quiet;
      if (!nothingToPoll_ && static_cast<int32_t>(nowMs - nextDiscoveryMs_) >= 0) return BusAction::Discover;
      return BusAction::None;
    }
    if (windowFrames_ > 0 && clean) return BusAction::Confirm;
    if (probeDue) return BusAction::Probe;
    if (nowMs - windowStartMs_ >= c_.windowMs) {
      if (windowFrames_ == 0 && clean) silentAt_ |= rateBit(rate_);
      return BusAction::NextWindow;
    }
    return BusAction::None;
  }

  // After Quiet: start over, then listen() at rate(). Quiet for quietMs at
  // the traffic's rate counts as a silent window there; the other rate still
  // has to be listened to.
  void quiet() {
    trafficRate_ = 0;
    failures_ = 0;
    nothingToPoll_ = false;
    silentAt_ = rateBit(rate_);
  }

  // A gateway probe goes onto the bus at a rate nobody has confirmed, so only
  // when it's silent at both rates (a whole window at each with no frame, so
  // a live bus at the other rate is always found first), and never once the
  // bus has been heard by listening: that port isn't gatewayed.
  bool probeSilence() const { return !heardByListening_ && silentAt_ == kBoth; }

  uint32_t rate() const { return rate_; }                 // listening at
  uint32_t trafficRate() const { return trafficRate_; }   // confirmed; 0 for none
  bool nothingToPoll() const { return nothingToPoll_; }
  uint32_t failures() const { return failures_; }         // unanswered discovery requests
  bool heardByListening() const { return heardByListening_; }
  uint32_t silentForMs(uint32_t nowMs) const { return nowMs - lastFrameMs_; }

  static uint32_t otherRate(uint32_t rate) { return rate == 500000 ? 250000 : 500000; }

 private:
  static uint8_t rateBit(uint32_t rate) { return rate == 500000 ? 1 : 2; }
  static constexpr uint8_t kBoth = 3;

  BusSearchConfig c_;
  uint32_t rate_ = 500000;
  uint32_t windowStartMs_ = 0;
  uint32_t windowFrames_ = 0;
  uint32_t lastFrameMs_ = 0;
  uint8_t silentAt_ = 0;  // bit 0: 500 kbit/s, bit 1: 250 kbit/s
  bool heardByListening_ = false;
  uint32_t trafficRate_ = 0;
  uint32_t nextDiscoveryMs_ = 0;
  uint32_t failures_ = 0;
  bool nothingToPoll_ = false;
};

}  // namespace gnimu
