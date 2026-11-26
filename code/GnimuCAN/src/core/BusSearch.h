// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdint.h>

namespace gnimu {

// What the App should do next while waiting for a vehicle (docs/
// architecture.md, Bring-up).
enum class BusAction : uint8_t {
  None,        // nothing for the bus search: with traffic confirmed, the vehicle source decides
  Confirm,     // clean frames in this window: traffic at its rate
  Probe,       // make a gateway probe
  NextWindow,  // this window is over (time up, or bus errors): listen at the other rate
  Quiet,       // the bus has gone quiet: start over (call quiet())
};

struct BusSearchConfig {
  uint32_t windowMs;  // one listen window at a rate
  uint32_t quietMs;   // no frames this long, with traffic confirmed: the car is off
};

// Finding the car's bus while waiting for a vehicle: listen windows at each
// bit rate, silence for gateway probes, confirmed traffic, and starting over
// when the bus goes quiet. It knows nothing of ECUs: what happens once
// traffic is confirmed belongs to the vehicle source (OBD discovery and
// polling here; a CAN tap build would decode broadcast frames instead).
// Pure logic: the App starts the controller, receives, and reports here;
// next() says what to do. The probe's own timing (supply hold, interval)
// stays in ProbePolicy; this says only whether the bus is silent enough for
// one.
class BusSearch {
 public:
  explicit BusSearch(const BusSearchConfig& c) : c_(c) {}

  // Listening at `rate` from now: a new window. Silence already heard stays.
  void listen(uint32_t rate, uint32_t nowMs) {
    rate_ = rate;
    windowStartMs_ = nowMs;
    windowFrames_ = 0;
  }

  // CAN switched off (no 12V): start from nothing, then listen() again.
  void restart() {
    trafficRate_ = 0;
    windowFrames_ = 0;
    silentAt_ = 0;
  }

  // Any frame received, in any state.
  void frame(uint32_t nowMs) {
    lastFrameMs_ = nowMs;
    windowFrames_++;
    silentAt_ = 0;
  }

  // Traffic confirmed at `rate`, by listening or by a probe the car answered.
  void confirmed(uint32_t rate) {
    rate_ = rate;
    trafficRate_ = rate;
  }

  // Polling, and the ECU stopped answering: back to looking for traffic.
  // Then listen() at rate().
  void vehicleLost() { trafficRate_ = 0; }

  // What to do now, while waiting with no request in progress. `clean`: no
  // bus errors since the window began. `probeDue`: ProbePolicy allows a probe
  // now, given probeSilence().
  BusAction next(uint32_t nowMs, bool clean, bool probeDue) {
    if (trafficRate_ != 0) return nowMs - lastFrameMs_ >= c_.quietMs ? BusAction::Quiet : BusAction::None;
    if (windowFrames_ > 0 && clean) return BusAction::Confirm;
    // Bus errors at this rate: likely traffic at the other one, so go and
    // listen there rather than wait out the window. Not at once, so a bus
    // that shows errors at both rates (CAN FD frames, say) switches a few
    // times a second, not every run.
    if (!clean && nowMs - windowStartMs_ >= c_.windowMs / 10) return BusAction::NextWindow;
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
    silentAt_ = rateBit(rate_);
  }

  // A gateway probe goes onto the bus at a rate nobody has confirmed, so only
  // when it's silent at both rates: a whole window at each with no frame and
  // no bus error, so a live bus at the other rate is always found first.
  // Having heard the bus earlier doesn't rule a probe out: some gatewayed
  // ports carry traffic only for a few seconds after a diagnostic request.
  bool probeSilence() const { return silentAt_ == kBoth; }

  uint32_t rate() const { return rate_; }                // listening at
  uint32_t trafficRate() const { return trafficRate_; }  // confirmed; 0 for none
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
  uint32_t trafficRate_ = 0;
};

}  // namespace gnimu
