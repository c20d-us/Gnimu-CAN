// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "DiscoveryBackoff.h"
#include "Obd2.h"

namespace gnimu {

struct ObdDiscoveryConfig {
  uint32_t collectMs;  // how long replies to one request are collected: P2, 50 ms
  DiscoveryBackoff backoff;
};

// What one round of discovery found (docs/architecture.md, Bring-up).
struct EcuChoice {
  uint8_t answered;  // bit n: the ECU on 0x7E8 + n replied
  bool chosen;       // an ECU supports at least one wanted PID
  uint8_t ecu;       // ... this one: index from 0x7E8
  uint32_t bitmap;   // ... and its PID 0x00 support bitmap (PIDs 0x01-0x20)
};

// Finding the ECU to poll, on the OBD-II port: OBD builds only; a CAN tap
// build never transmits, so it has no discovery.
//
// It follows ISO 15765-4 (4.2.2): one functional Mode 01 PID 0x00 request,
// then every reply that arrives within P2 (50 ms) is collected, since a car
// usually has more than one ECU that answers, and which answers first varies.
//
// The ECU to poll is chosen the way python-OBD finds the engine: the ECU on
// 0x7E8 (ISO 15765-4 recommends 0x7E0/0x7E8 for the engine), if it supports
// anything wanted; otherwise, for each wanted PID in priority order, the ECU
// supporting it with the most PIDs supported in all.
//
// No reply, or no ECU supporting anything wanted: it asks again after the
// backoff (DiscoveryBackoff), which starts over with restart(). Pure logic:
// the App sends the request and hands over the replies.
class ObdDiscovery {
 public:
  static constexpr size_t kMaxEcus = 8;  // 0x7E8-0x7EF

  explicit ObdDiscovery(const ObdDiscoveryConfig& c) : c_(c) {}

  // Traffic confirmed at a rate (by listening or a probe): ask straight away,
  // and count failures from zero.
  void restart(uint32_t nowMs) {
    nextMs_ = nowMs;
    failures_ = 0;
    collecting_ = false;
  }

  // A request should go now.
  bool due(uint32_t nowMs) const { return !collecting_ && static_cast<int32_t>(nowMs - nextMs_) >= 0; }

  // The request has just been sent: collect replies from now.
  void begin(uint32_t nowMs) {
    collecting_ = true;
    startMs_ = nowMs;
    answered_ = 0;
    for (size_t i = 0; i < kMaxEcus; i++) bitmaps_[i] = 0;
  }

  // A PID 0x00 reply from the ECU on 0x7E8 + `ecu`. A repeat from the same
  // ECU replaces its bitmap.
  void reply(uint8_t ecu, uint32_t bitmap) {
    if (!collecting_ || ecu >= kMaxEcus) return;
    answered_ |= static_cast<uint8_t>(1u << ecu);
    bitmaps_[ecu] = bitmap;
  }

  bool collecting() const { return collecting_; }
  // The collection window is over: call finish().
  bool collected(uint32_t nowMs) const { return collecting_ && nowMs - startMs_ >= c_.collectMs; }

  // Ends the round. `wanted`: the PIDs to poll, most important first. With
  // nothing chosen (no reply, or nothing wanted supported), the next request
  // waits for the backoff: retryInMs() says how long.
  EcuChoice finish(const uint8_t* wanted, size_t wantedCount, uint32_t nowMs) {
    collecting_ = false;
    EcuChoice c = {answered_, false, 0, 0};
    int best = -1;
    if ((answered_ & 1u) && supportsAny(bitmaps_[0], wanted, wantedCount)) best = 0;
    for (size_t w = 0; best < 0 && w < wantedCount; w++) {
      int bestBits = -1;
      for (size_t e = 0; e < kMaxEcus; e++) {
        if (!(answered_ & (1u << e)) || !supports(bitmaps_[e], wanted[w])) continue;
        const int bits = bitCount(bitmaps_[e]);
        if (bits > bestBits) {
          bestBits = bits;
          best = static_cast<int>(e);
        }
      }
    }
    if (best >= 0) {
      c.chosen = true;
      c.ecu = static_cast<uint8_t>(best);
      c.bitmap = bitmaps_[best];
      failures_ = 0;
    } else {
      failures_++;
      retryMs_ = discoveryRetryMs(c_.backoff, failures_);
      nextMs_ = nowMs + retryMs_;
    }
    return c;
  }

  // The last round's PID 0x00 bitmap from the ECU on 0x7E8 + `ecu`; 0 if it
  // didn't answer (the round's answered mask says which did).
  uint32_t bitmap(uint8_t ecu) const { return ecu < kMaxEcus ? bitmaps_[ecu] : 0; }

  uint32_t failures() const { return failures_; }  // rounds in a row with nothing chosen
  uint32_t retryInMs() const { return retryMs_; }  // the wait after the last of them

 private:
  static bool supports(uint32_t bitmap, uint8_t pid) { return obd2::pidSupported(bitmap, pid); }
  static bool supportsAny(uint32_t bitmap, const uint8_t* wanted, size_t n) {
    for (size_t i = 0; i < n; i++)
      if (supports(bitmap, wanted[i])) return true;
    return false;
  }
  static int bitCount(uint32_t v) {
    int n = 0;
    for (; v; v &= v - 1) n++;
    return n;
  }

  ObdDiscoveryConfig c_;
  uint32_t nextMs_ = 0;
  uint32_t failures_ = 0;
  uint32_t retryMs_ = 0;
  bool collecting_ = false;
  uint32_t startMs_ = 0;
  uint8_t answered_ = 0;
  uint32_t bitmaps_[kMaxEcus] = {};
};

}  // namespace gnimu
