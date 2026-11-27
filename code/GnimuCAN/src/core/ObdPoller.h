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

// One channel to poll: a Mode 01 PID, how often, and how many data bytes a
// reply must carry to decode.
struct PollChannelConfig {
  uint8_t pid;
  uint32_t periodMs;  // 0: off
  uint8_t minBytes;
};

struct PollerConfig {
  uint32_t timeoutMs;       // a request unanswered this long has timed out
  uint32_t minGapMs;        // at least this long between requests: the total-rate cap
  uint8_t silentAfter;      // timeouts in a row before a channel counts as not answering
  uint32_t silentRetryMs;   // ... and is asked this often until it answers again
  uint8_t askEveryoneAfter; // timeouts in a row, with no reply yet, before switching to 0x7DF
};

enum class ChannelState : uint8_t {
  Off,          // not configured (period 0)
  Unknown,      // above PID 0x20, waiting for the support bitmap that covers it
  Unsupported,  // the ECU's support bitmaps don't list it
  Polling,
  Silent,       // supported, but stopped answering: asked only every silentRetryMs
  Refused,      // the ECU answered "not supported": not asked again this session
};

// One polled channel and what polling has found.
struct PollChannel {
  uint8_t pid;
  uint32_t targetPeriodMs;  // how often it should be asked for; 0 = off
  uint8_t minBytes;
  ChannelState state;
  uint32_t requests;
  uint32_t replies;
  uint32_t timeouts;
  uint8_t timeoutsInRow;
  bool valid;  // at least one reply since polling started
  uint32_t lastReplyMs;
  uint8_t len;
  uint8_t data[4];  // the latest reply's A..D
  uint32_t replyMsTotal;  // request to reply, summed over the replies; in the
  uint32_t replyMsMax;    // ... caller's tick steps, so a resolution of its period

  bool asked() const { return state == ChannelState::Polling || state == ChannelState::Silent; }
};

// Polls Mode 01 PIDs one request at a time (docs/architecture.md, Polling):
// sends the next request only once the last one has been answered or has
// timed out, and picks whichever channel is most overdue against its target
// period. Before polling channels above PID 0x20, it reads the support
// bitmaps that cover them (PIDs 0x20, 0x40, 0x60), each only when the one
// before says it exists. Replies count only from the polled ECU. A negative
// response "not supported" drops the channel; "busy" or "pending" don't count
// as a timeout.
//
// Requests go to the ECU's physical address. Some gateways pass only
// functional requests (0x7DF) from the OBD port, so if the first few requests
// of a session all time out, it asks everyone instead, still taking replies
// only from the polled ECU. Pure logic: the caller sends the frames it
// returns and feeds back what arrives.
template <size_t N>
class ObdPoller {
 public:
  ObdPoller(const PollChannelConfig (&channels)[N], const PollerConfig& c) : c_(c) {
    for (size_t i = 0; i < N; i++) {
      ch_[i] = PollChannel();
      ch_[i].pid = channels[i].pid;
      ch_[i].targetPeriodMs = channels[i].periodMs;
      ch_[i].minBytes = channels[i].minBytes;
      if (channels[i].periodMs > 0 && channels[i].pid > maxPid_) maxPid_ = channels[i].pid;
    }
  }

  // A new vehicle session: the ECU's physical request ID, its PID 0x00
  // support bitmap (PIDs 0x01-0x20), and whether to ask everyone (0x7DF) from
  // the start. Clears everything learned before.
  void start(uint32_t requestId, uint32_t supportBitmap, uint32_t nowMs, bool askEveryone = false) {
    requestId_ = requestId;
    askEveryone_ = askEveryone;
    switchedToEveryone_ = false;
    answered_ = false;
    timeoutsInRow_ = 0;
    inFlight_ = false;
    lastAnyReplyMs_ = nowMs;
    lastSentMs_ = nowMs - c_.minGapMs;
    for (size_t i = 0; i < N; i++) {
      PollChannel& c = ch_[i];
      c.state = c.targetPeriodMs == 0 ? ChannelState::Off : ChannelState::Unknown;
      c.requests = c.replies = c.timeouts = 0;
      c.timeoutsInRow = 0;
      c.valid = false;
      c.len = 0;
      memset(c.data, 0, sizeof(c.data));
      c.replyMsTotal = c.replyMsMax = 0;
      lastAskedMs_[i] = nowMs - c.targetPeriodMs;  // every channel due at once
    }
    applyBitmap(0x00, supportBitmap);
  }

  // True, filling `out`, when a request should go now.
  bool next(uint32_t nowMs, CanFrame& out) {
    if (inFlight_ || nowMs - lastSentMs_ < c_.minGapMs) return false;
    if (nextBitmap_ != 0) return send(N, nextBitmap_, nowMs, out);
    size_t best = N;
    uint64_t bestScore = 0;
    for (size_t i = 0; i < N; i++) {
      const PollChannel& c = ch_[i];
      if (!c.asked()) continue;
      const uint32_t period = c.state == ChannelState::Silent && c_.silentRetryMs > c.targetPeriodMs
                                  ? c_.silentRetryMs
                                  : c.targetPeriodMs;
      const uint32_t waited = nowMs - lastAskedMs_[i];
      if (waited < period) continue;
      // Overdue relative to its own period, in thousandths: a 1 Hz channel
      // 2 s late beats a 20 Hz channel 60 ms late.
      const uint64_t score = static_cast<uint64_t>(waited) * 1000 / period;
      if (best == N || score > bestScore) {
        best = i;
        bestScore = score;
      }
    }
    if (best == N) return false;
    ch_[best].requests++;
    lastAskedMs_[best] = nowMs;
    return send(best, ch_[best].pid, nowMs, out);
  }

  // Every received frame. True when it settled the request in flight.
  bool onFrame(const CanFrame& f, uint32_t nowMs) {
    if (!inFlight_ || f.id != requestId_ + 8) return false;
    answered_ = true;
    timeoutsInRow_ = 0;
    uint8_t nrc = 0;
    if (obd2::parseNegativeReply(f, nrc)) return negative(nrc, nowMs);
    obd2::Mode01Reply r;
    if (!obd2::parseMode01Reply(f, inFlightPid_, r)) return false;
    lastAnyReplyMs_ = nowMs;
    inFlight_ = false;
    if (inFlightIndex_ == N) {
      // A support bitmap.
      if (r.len == 4) {
        applyBitmap(inFlightPid_, (static_cast<uint32_t>(r.data[0]) << 24) | (static_cast<uint32_t>(r.data[1]) << 16) |
                                      (static_cast<uint32_t>(r.data[2]) << 8) | r.data[3]);
      } else {
        endBitmaps();
      }
      return true;
    }
    PollChannel& c = ch_[inFlightIndex_];
    if (r.len < c.minBytes) return true;  // too short to decode: answered, but no value
    const uint32_t replyMs = nowMs - sentMs_;
    c.replies++;
    c.replyMsTotal += replyMs;
    if (replyMs > c.replyMsMax) c.replyMsMax = replyMs;
    c.timeoutsInRow = 0;
    c.state = ChannelState::Polling;
    c.valid = true;
    c.lastReplyMs = nowMs;
    c.len = r.len;
    memcpy(c.data, r.data, sizeof(c.data));
    return true;
  }

  // Gives up on a request that has waited longer than the timeout.
  void checkTimeout(uint32_t nowMs) {
    if (!inFlight_ || nowMs - sentMs_ < c_.timeoutMs) return;
    inFlight_ = false;
    if (!askEveryone_ && !answered_ && ++timeoutsInRow_ >= c_.askEveryoneAfter) {
      // Nothing has answered a physical request: ask everyone, and start the
      // channels' timeout counts over.
      askEveryone_ = switchedToEveryone_ = true;
      for (size_t i = 0; i < N; i++) {
        ch_[i].timeoutsInRow = 0;
        if (ch_[i].state == ChannelState::Silent) ch_[i].state = ChannelState::Polling;
      }
      if (inFlightIndex_ == N) return;  // a bitmap: asked again, to everyone
      ch_[inFlightIndex_].timeouts++;
      return;
    }
    if (inFlightIndex_ == N) {
      // Unanswered before anything has answered: asked again (to everyone,
      // once it switches). Otherwise there's no such bitmap, and the channels
      // it would cover are unsupported.
      if (askEveryone_ || answered_) endBitmaps();
      return;
    }
    PollChannel& c = ch_[inFlightIndex_];
    c.timeouts++;
    if (c.timeoutsInRow < 255) c.timeoutsInRow++;
    if (c.timeoutsInRow >= c_.silentAfter) c.state = ChannelState::Silent;
  }

  // Every channel's support is known: no bitmap still to read.
  bool supportKnown() const {
    for (size_t i = 0; i < N; i++)
      if (ch_[i].state == ChannelState::Unknown) return false;
    return true;
  }

  // How long since the ECU last answered anything: the car has gone quiet.
  uint32_t silentForMs(uint32_t nowMs) const { return nowMs - lastAnyReplyMs_; }

  const PollChannel& channel(size_t i) const { return ch_[i]; }
  size_t channelCount() const { return N; }
  uint32_t requestId() const { return requestId_; }
  // Where requests go: the ECU's physical address, or 0x7DF.
  uint32_t sendId() const { return askEveryone_ ? obd2::kFunctionalRequestId : requestId_; }
  bool askingEveryone() const { return askEveryone_; }
  // True once, after it has switched to asking everyone.
  bool takeSwitchedToEveryone() {
    const bool s = switchedToEveryone_;
    switchedToEveryone_ = false;
    return s;
  }

 private:
  bool send(size_t index, uint8_t pid, uint32_t nowMs, CanFrame& out) {
    out = obd2::mode01Request(pid, sendId());
    inFlight_ = true;
    inFlightIndex_ = index;
    inFlightPid_ = pid;
    sentMs_ = lastSentMs_ = nowMs;
    return true;
  }

  // The support bitmap for PIDs base+1 .. base+0x20; its last bit says
  // whether the next one exists.
  void applyBitmap(uint8_t base, uint32_t bitmap) {
    for (size_t i = 0; i < N; i++) {
      PollChannel& c = ch_[i];
      if (c.state == ChannelState::Off || c.pid <= base || c.pid > base + 0x20) continue;
      c.state = obd2::pidSupported(bitmap, static_cast<uint8_t>(c.pid - base)) ? ChannelState::Polling
                                                                               : ChannelState::Unsupported;
    }
    const uint32_t nextBase = base + 0x20u;
    if (maxPid_ > nextBase && nextBase < 0x100 && obd2::pidSupported(bitmap, 0x20)) {
      nextBitmap_ = static_cast<uint8_t>(nextBase);
    } else {
      endBitmaps();
    }
  }

  // No more bitmaps: channels still waiting for one aren't supported.
  void endBitmaps() {
    nextBitmap_ = 0;
    for (size_t i = 0; i < N; i++)
      if (ch_[i].state == ChannelState::Unknown) ch_[i].state = ChannelState::Unsupported;
  }

  bool negative(uint8_t nrc, uint32_t nowMs) {
    lastAnyReplyMs_ = nowMs;
    if (nrc == obd2::kResponsePending) {
      sentMs_ = nowMs;  // still working on it: wait again
      return true;
    }
    inFlight_ = false;
    if (inFlightIndex_ == N) {
      endBitmaps();
      return true;
    }
    if (obd2::isNotSupported(nrc)) ch_[inFlightIndex_].state = ChannelState::Refused;
    // Anything else (busy, conditions not right): asked again next period,
    // not counted as a timeout.
    return true;
  }

  PollerConfig c_;
  PollChannel ch_[N];
  uint32_t lastAskedMs_[N] = {};
  uint8_t maxPid_ = 0;  // the highest PID with a period: how many bitmaps to read
  uint8_t nextBitmap_ = 0;  // the support bitmap to ask for next; 0 = none
  uint32_t requestId_ = obd2::kFunctionalRequestId;
  bool askEveryone_ = false;
  bool switchedToEveryone_ = false;
  bool answered_ = false;       // the ECU has answered something this session
  uint8_t timeoutsInRow_ = 0;   // across all requests, while nothing has answered
  bool inFlight_ = false;
  size_t inFlightIndex_ = 0;  // N: a support bitmap
  uint8_t inFlightPid_ = 0;
  uint32_t sentMs_ = 0;
  uint32_t lastSentMs_ = 0;
  uint32_t lastAnyReplyMs_ = 0;
};

}  // namespace gnimu
