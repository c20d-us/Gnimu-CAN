// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "CanFrame.h"
#include "Obd2.h"

namespace gnimu {

// One polled Mode 01 PID.
struct PollChannel {
  uint8_t pid;
  uint32_t targetPeriodMs;  // how often it should be asked for
  bool supported;           // the car says it has it (PID 0x00 bitmap)
  // What polling has found:
  uint32_t requests;
  uint32_t replies;
  uint32_t timeouts;
  bool valid;               // at least one reply since polling started
  uint32_t lastReplyMs;
  uint8_t len;
  uint8_t data[4];          // the latest reply's A..D
};

// Polls Mode 01 PIDs one request at a time (docs/architecture.md, Polling):
// sends the next request only once the last one has been answered or has
// timed out, and picks whichever supported channel is most overdue against
// its target period. Pure logic: the caller sends the frames it returns and
// feeds back what arrives.
template <size_t N>
class ObdPoller {
 public:
  ObdPoller(const uint8_t (&pids)[N], const uint32_t (&periodsMs)[N], uint32_t timeoutMs)
      : timeoutMs_(timeoutMs) {
    for (size_t i = 0; i < N; i++) {
      ch_[i] = PollChannel();
      ch_[i].pid = pids[i];
      ch_[i].targetPeriodMs = periodsMs[i];
    }
  }

  // A new vehicle session: the ECU's physical request ID, and which channels
  // its support bitmap (PIDs 0x01-0x20, from PID 0x00's reply) includes.
  // Clears everything learned from the previous session.
  void start(uint32_t requestId, uint32_t supportBitmap, uint32_t nowMs) {
    requestId_ = requestId;
    inFlight_ = false;
    lastAnyReplyMs_ = nowMs;
    for (size_t i = 0; i < N; i++) {
      PollChannel& c = ch_[i];
      c.supported = isSupported(supportBitmap, c.pid);
      c.requests = c.replies = c.timeouts = 0;
      c.valid = false;
      c.len = 0;
      memset(c.data, 0, sizeof(c.data));
      lastAskedMs_[i] = nowMs - c.targetPeriodMs;  // every channel due at once
    }
  }

  // True, filling `out`, when a request should go now.
  bool next(uint32_t nowMs, CanFrame& out) {
    if (inFlight_) return false;
    size_t best = N;
    uint64_t bestScore = 0;
    for (size_t i = 0; i < N; i++) {
      const PollChannel& c = ch_[i];
      if (!c.supported) continue;
      const uint32_t waited = nowMs - lastAskedMs_[i];
      if (waited < c.targetPeriodMs) continue;
      // Overdue relative to its own period, in thousandths: a 1 Hz channel
      // 2 s late beats a 20 Hz channel 60 ms late.
      const uint64_t score = static_cast<uint64_t>(waited) * 1000 / c.targetPeriodMs;
      if (best == N || score > bestScore) {
        best = i;
        bestScore = score;
      }
    }
    if (best == N) return false;
    out = obd2::mode01Request(ch_[best].pid, requestId_);
    ch_[best].requests++;
    lastAskedMs_[best] = nowMs;
    inFlight_ = true;
    inFlightIndex_ = best;
    sentMs_ = nowMs;
    return true;
  }

  // Every received frame. True when it answered the request in flight.
  bool onFrame(const CanFrame& f, uint32_t nowMs) {
    if (!inFlight_) return false;
    PollChannel& c = ch_[inFlightIndex_];
    obd2::Mode01Reply r;
    if (!obd2::parseMode01Reply(f, c.pid, r)) return false;
    c.replies++;
    c.valid = true;
    c.lastReplyMs = nowMs;
    c.len = r.len;
    memcpy(c.data, r.data, sizeof(c.data));
    lastAnyReplyMs_ = nowMs;
    inFlight_ = false;
    return true;
  }

  // Gives up on a request that has waited longer than the timeout.
  void checkTimeout(uint32_t nowMs) {
    if (inFlight_ && nowMs - sentMs_ >= timeoutMs_) {
      ch_[inFlightIndex_].timeouts++;
      inFlight_ = false;
    }
  }

  // How long since any channel was answered: the car has gone quiet.
  uint32_t silentForMs(uint32_t nowMs) const { return nowMs - lastAnyReplyMs_; }

  const PollChannel& channel(size_t i) const { return ch_[i]; }
  size_t channelCount() const { return N; }
  uint32_t requestId() const { return requestId_; }

  // Whether `pid` (0x01-0x20) is set in a PID 0x00 support bitmap, where the
  // bitmap's top bit is PID 0x01.
  static bool isSupported(uint32_t bitmap, uint8_t pid) {
    if (pid < 0x01 || pid > 0x20) return false;
    return (bitmap >> (32 - pid)) & 1u;
  }

 private:
  PollChannel ch_[N];
  uint32_t lastAskedMs_[N] = {};
  uint32_t timeoutMs_;
  uint32_t requestId_ = obd2::kFunctionalRequestId;
  bool inFlight_ = false;
  size_t inFlightIndex_ = 0;
  uint32_t sentMs_ = 0;
  uint32_t lastAnyReplyMs_ = 0;
};

}  // namespace gnimu
