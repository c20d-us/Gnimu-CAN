// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "CanFrame.h"
#include "IsoTp.h"
#include "Obd2.h"
#include "Obd2Pids.h"
#include "ObdSchedule.h"

namespace gnimu {

// One channel to poll: a Mode 01 PID, how often, and how many data bytes
// its reply has, as SAE J1979 defines it: a reply must carry exactly that
// many, and a reply to several PIDs is split by it.
struct PollChannelConfig {
  uint8_t pid;
  uint32_t periodMs;  // 0: off
  uint8_t length;
};

struct PollerConfig {
  uint32_t timeoutMs;       // a request unanswered this long has timed out
  uint32_t minGapMs;        // at least this long between requests: the total-rate cap
  uint8_t silentAfter;      // timeouts in a row before a channel counts as not answering
  uint32_t silentRetryMs;   // ... and is asked this often until it answers again
  uint8_t askEveryoneAfter; // timeouts in a row, with no reply yet, before switching to 0x7DF
  uint32_t cycleMs;         // multi-PID polling: one request per cycle
  uint8_t maxPerRequest;    // ... of up to this many PIDs; 1: one PID per request, always
  uint8_t fallBackAfter;    // multi-PID failures in a row before one PID per request
};

// Why multi-PID polling stopped for the boot.
enum class BundleFault : uint8_t {
  None,
  Timeout,       // no complete reply in time
  NoRest,        // a reply's first frame, then nothing after the flow control
  Malformed,     // a reply that didn't reassemble or split exactly
  NotSupported,  // the ECU answered "not supported" to the request
};

inline const char* bundleFaultText(BundleFault f) {
  switch (f) {
    case BundleFault::Timeout:
      return "no reply in time, though one PID alone is answered";
    case BundleFault::NoRest:
      return "nothing after flow control";
    case BundleFault::Malformed:
      return "replies that didn't parse";
    case BundleFault::NotSupported:
      return "refused by the ECU";
    case BundleFault::None:
      break;
  }
  return "none";
}

// How multi-PID polling is going this session.
struct BundleStats {
  bool active;          // asking for several PIDs per request now
  bool fellBack;        // stopped for the boot: one PID per request
  BundleFault fault;    // ... because of three of these in a row
  uint8_t busiest;      // the most PIDs one cycle's request carries
  uint32_t requests;
  uint32_t pidsAsked;   // across all requests
  uint32_t replies;     // complete, split replies
  uint32_t failures;
  uint32_t late;        // cycles that started more than a cycle late, the schedule restarted from then
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
  uint32_t every;         // multi-PID polling: in every this-many cycles' request ...
  uint32_t offset;        // ... from this cycle

  bool asked() const { return state == ChannelState::Polling || state == ChannelState::Silent; }
};

// Polls Mode 01 PIDs one request at a time (docs/architecture.md, Polling):
// sends the next request only once the last one has been answered or has
// timed out. Before polling channels above PID 0x20, it reads the support
// bitmaps that cover them (PIDs 0x20, 0x40, 0x60), each only when the one
// before says it exists. Replies count only from the polled ECU.
//
// Once every channel's support is known and the ECU has answered, it asks
// for several PIDs per request, on a fixed schedule: one request every
// cycle, carrying each channel whose turn it is (ObdSchedule.h). A channel
// due when a request is already full goes in the next. The reply may be
// several CAN frames: the caller sends the flow control it asks for
// (takeFlowControl). A reply that leaves a PID out counts as a timeout for
// that channel only. A request that times out, a reply that doesn't parse
// exactly, or "not supported" counts against multi-PID polling; after
// fallBackAfter in a row, it asks for one PID per request for the rest of
// the session (takeFellBack). When those were all timeouts, the ECU may be
// going quiet (the car switching off), so it first asks for one PID alone,
// the check channel (RPM): unanswered, it carries on as it was, and the car
// is found gone once nothing has answered for a while. Answered, the caller
// decides (checkAnswered, settleCheck): with the engine running, the ECU
// doesn't take multi-PID requests, so it falls back; with it stopped, the
// car is likely shutting down, so it carries on. Each session starts with
// multi-PID requests again.
//
// One PID per request, it picks whichever channel is most overdue against
// its target period. A negative response "not supported" drops the channel;
// "busy" or "pending" don't count as a timeout.
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
      length_[i] = channels[i].length;
      own_[i] = channels[i];
      if (channels[i].periodMs > 0 && channels[i].pid > maxPid_) maxPid_ = channels[i].pid;
    }
  }

  // A second-choice PID for channel `i`: asked for instead when the ECU's
  // support bitmaps don't list the channel's own, but do list this one (some
  // engines report coolant only as 0x67). Set before start().
  void setAlternate(size_t i, uint8_t pid, uint8_t length) {
    if (i >= N) return;
    alt_[i] = {pid, own_[i].periodMs, length};
    if (own_[i].periodMs > 0 && pid > maxPid_) maxPid_ = pid;
  }

  // A new vehicle session: the ECU's physical request ID, its PID 0x00
  // support bitmap (PIDs 0x01-0x20), and whether to ask everyone (0x7DF)
  // from the start. Clears everything learned before.
  void start(uint32_t requestId, uint32_t supportBitmap, uint32_t nowMs, bool askEveryone = false) {
    requestId_ = requestId;
    askEveryone_ = askEveryone;
    bundling_ = c_.maxPerRequest > 1;
    scheduled_ = startedBundling_ = fellBack_ = flowControl_ = confirming_ = false;
    checkAnswered_ = false;
    onlyTimeouts_ = true;
    bundle_ = BundleStats();
    laterRead_ = 0;
    bundleFailuresInRow_ = 0;
    switchedToEveryone_ = false;
    answered_ = false;
    timeoutsInRow_ = 0;
    inFlight_ = false;
    lastAnyReplyMs_ = nowMs;
    lastSentMs_ = nowMs - c_.minGapMs;
    for (size_t i = 0; i < N; i++) {
      PollChannel& c = ch_[i];
      c.state = c.targetPeriodMs == 0 ? ChannelState::Off : ChannelState::Unknown;
      use(i, own_[i]);  // its own PID, until the bitmaps say otherwise
      ownSeen_[i] = altSeen_[i] = Seen::NotYet;
      c.requests = c.replies = c.timeouts = 0;
      c.timeoutsInRow = 0;
      c.valid = false;
      c.len = 0;
      memset(c.data, 0, sizeof(c.data));
      c.replyMsTotal = c.replyMsMax = 0;
      c.every = c.offset = 0;
      due_[i] = false;
      lastAskedMs_[i] = nowMs - c.targetPeriodMs;  // every channel due at once
    }
    applyBitmap(0x00, supportBitmap);
  }

  // True, filling `out`, when a request should go now.
  bool next(uint32_t nowMs, CanFrame& out) {
    if (inFlight_ || nowMs - lastSentMs_ < c_.minGapMs) return false;
    if (nextBitmap_ != 0) return send(N, nextBitmap_, nowMs, out);
    if (checkAnswered_) return false;  // waiting for settleCheck()
    if (bundling_ && answered_ && supportKnown()) return confirming_ ? confirmRequest(nowMs, out) : nextBundle(nowMs, out);
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
    if (obd2::parseNegativeReply(f, nrc)) {
      negative(nrc, nowMs);
      if (confirming_ && !inFlight_) confirmed();  // answered, if only "busy"
      return true;
    }
    // Every reply the same way, whatever was asked for: reassembled, then
    // split against the PIDs asked for.
    switch (rx_.frame(f.data, f.len)) {
      case Rx::Step::More:
        return false;
      case Rx::Step::SendFlowControl:
        flowControl_ = true;
        return false;
      case Rx::Step::Failed:
        return unparsed();
      case Rx::Step::Complete:
        break;
    }
    // What was asked for: the multi-PID request's channels, or one PID (a
    // support bitmap's four bytes, or a channel's).
    uint8_t pids[obd2::kMaxPidsPerRequest];
    uint8_t lengths[obd2::kMaxPidsPerRequest];
    uint8_t offsets[obd2::kMaxPidsPerRequest];
    size_t count = bundleCount_;
    for (size_t j = 0; j < count; j++) {
      pids[j] = ch_[bundleIndex_[j]].pid;
      lengths[j] = length_[bundleIndex_[j]];
    }
    if (count == 0) {
      pids[0] = inFlightPid_;
      lengths[0] = inFlightIndex_ == N ? 4 : length_[inFlightIndex_];
      count = 1;
    }
    const uint8_t* payload = rx_.payload();
    if (!obd2::splitMode01Reply(payload, rx_.length(), pids, lengths, count, offsets)) return unparsed();
    lastAnyReplyMs_ = nowMs;
    inFlight_ = false;
    const uint32_t replyMs = nowMs - sentMs_;
    if (bundleCount_ > 0) {
      bundleReplied(payload, lengths, offsets, replyMs, nowMs);
      return true;
    }
    const uint8_t* data = payload + offsets[0];
    if (inFlightIndex_ == N) {
      // A support bitmap.
      const uint32_t bitmap = (static_cast<uint32_t>(data[0]) << 24) | (static_cast<uint32_t>(data[1]) << 16) |
                              (static_cast<uint32_t>(data[2]) << 8) | data[3];
      const size_t slot = inFlightPid_ / 0x20u - 1;
      if (slot < kLaterBitmaps) {
        later_[slot] = bitmap;
        laterRead_ |= static_cast<uint8_t>(1u << slot);
      }
      applyBitmap(inFlightPid_, bitmap);
      return true;
    }
    if (confirming_) confirmed();
    record(ch_[inFlightIndex_], data, lengths[0], replyMs, nowMs);
    return true;
  }

  // Gives up on a request that has waited longer than the timeout.
  void checkTimeout(uint32_t nowMs) {
    if (!inFlight_ || nowMs - sentMs_ < c_.timeoutMs) return;
    if (bundleCount_ > 0) {
      bundleFailed(rx_.stalledAfterFirst() ? BundleFault::NoRest : BundleFault::Timeout);
      return;
    }
    inFlight_ = false;
    if (confirming_) {
      // One PID alone goes unanswered too: the ECU has gone quiet, not
      // refused multi-PID requests. Carry on as before.
      confirming_ = false;
      bundleFailuresInRow_ = 0;
      onlyTimeouts_ = true;
      return;
    }
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

  // The chosen ECU's support bitmap for `base` (0x20, 0x40, 0x60) this
  // session: true, filling `out`, once it has answered for it.
  bool laterBitmap(uint8_t base, uint32_t& out) const {
    const size_t i = base / 0x20u - 1;
    if (base == 0 || base % 0x20 != 0 || i >= kLaterBitmaps || !(laterRead_ & (1u << i))) return false;
    out = later_[i];
    return true;
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

  // True, filling `out`, when a reply's first frame needs flow control sent
  // back to the ECU: to its physical address, as ISO 15765-4 requires, even
  // when the request went to everyone.
  bool takeFlowControl(CanFrame& out) {
    if (!flowControl_) return false;
    flowControl_ = false;
    out = obd2::flowControl(requestId_);
    return true;
  }
  // True once, when multi-PID polling has started this session: the schedule
  // is built (each channel's every and offset).
  bool takeStartedBundling() {
    const bool s = startedBundling_;
    startedBundling_ = false;
    return s;
  }
  // The channel asked for alone before falling back (RPM): its value says
  // whether the engine is running. Set before start(); without one, the
  // first on the schedule.
  void setCheckChannel(size_t i) { checkChannel_ = i; }

  // The one PID asked for alone was answered (even "busy"): the caller says
  // whether the engine is running (settleCheck) before anything more is
  // sent.
  bool checkAnswered() const { return checkAnswered_; }
  // Running: the ECU answers, but not multi-PID requests, so it's one PID
  // per request for the session. Stopped: the car is likely shutting down,
  // so it carries on with multi-PID requests, the count started over.
  void settleCheck(bool engineRunning) {
    if (!checkAnswered_) return;
    checkAnswered_ = false;
    if (engineRunning) {
      fallBack(BundleFault::Timeout);
      return;
    }
    confirming_ = false;
    bundleFailuresInRow_ = 0;
    onlyTimeouts_ = true;
  }

  // True once, when multi-PID polling has stopped for the session.
  bool takeFellBack() {
    const bool s = fellBack_;
    fellBack_ = false;
    return s;
  }
  const BundleStats& bundleStats() const { return bundle_; }

 private:
  bool send(size_t index, uint8_t pid, uint32_t nowMs, CanFrame& out) {
    out = obd2::mode01Request(pid, sendId());
    rx_.reset();
    flowControl_ = false;
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
      if (ch_[i].state != ChannelState::Unknown) continue;
      seeIn(base, bitmap, own_[i].pid, ownSeen_[i]);
      seeIn(base, bitmap, alt_[i].pid, altSeen_[i]);
      resolve(i, false);
    }
    const uint32_t nextBase = base + 0x20u;
    if (maxPid_ > nextBase && nextBase < 0x100 && obd2::pidSupported(bitmap, 0x20)) {
      nextBitmap_ = static_cast<uint8_t>(nextBase);
    } else {
      endBitmaps();
    }
  }

  // No more bitmaps: a PID still waiting for one isn't supported.
  void endBitmaps() {
    nextBitmap_ = 0;
    for (size_t i = 0; i < N; i++) resolve(i, true);
  }

  enum class Seen : uint8_t { NotYet, Unsupported, Supported };

  // Whether a bitmap for PIDs base+1 .. base+0x20 covers `pid`, and what it
  // says.
  static void seeIn(uint8_t base, uint32_t bitmap, uint8_t pid, Seen& seen) {
    if (pid == 0 || pid <= base || pid > base + 0x20) return;
    seen = obd2::pidSupported(bitmap, static_cast<uint8_t>(pid - base)) ? Seen::Supported : Seen::Unsupported;
  }

  // Settles a channel still waiting on the bitmaps, once they've said
  // enough: its own PID if supported, else its second choice if that is,
  // else unsupported. `final`: no more bitmaps come, so anything not seen
  // isn't supported.
  void resolve(size_t i, bool final) {
    PollChannel& c = ch_[i];
    if (c.state != ChannelState::Unknown) return;
    if (ownSeen_[i] == Seen::Supported) {
      c.state = ChannelState::Polling;
      return;
    }
    if (ownSeen_[i] == Seen::NotYet && !final) return;
    if (altSeen_[i] == Seen::Supported) {
      use(i, alt_[i]);
      c.state = ChannelState::Polling;
      return;
    }
    if (alt_[i].pid != 0 && altSeen_[i] == Seen::NotYet && !final) return;
    c.state = ChannelState::Unsupported;
  }

  // Asks for `p`'s PID for channel `i`.
  void use(size_t i, const PollChannelConfig& p) {
    ch_[i].pid = p.pid;
    length_[i] = p.length;
  }

  // A reply that didn't reassemble or split. To a multi-PID request, it
  // counts against multi-PID polling. To one PID, it's not the reply (a
  // late one to an earlier request, or too short): the request waits on,
  // and times out if nothing better comes.
  bool unparsed() {
    if (bundleCount_ > 0) {
      bundleFailed(BundleFault::Malformed);
      return true;
    }
    rx_.reset();
    return false;
  }

  // A multi-PID request's reply, split: each channel's value, or a timeout
  // for each the ECU left out.
  void bundleReplied(const uint8_t* payload, const uint8_t* lengths, const uint8_t* offsets, uint32_t replyMs,
                     uint32_t nowMs) {
    bundleFailuresInRow_ = 0;
    onlyTimeouts_ = true;
    bundle_.replies++;
    for (size_t j = 0; j < bundleCount_; j++) {
      PollChannel& c = ch_[bundleIndex_[j]];
      if (offsets[j] == 0) {
        // Left out: as if this channel alone had timed out.
        c.timeouts++;
        if (c.timeoutsInRow < 255) c.timeoutsInRow++;
        if (c.timeoutsInRow >= c_.silentAfter) c.state = ChannelState::Silent;
        continue;
      }
      record(c, payload + offsets[j], lengths[j], replyMs, nowMs);
    }
    bundleCount_ = 0;
  }

  // A multi-PID request failed; enough in a row, and it's one PID per
  // request for the rest of the session.
  void bundleFailed(BundleFault fault) {
    inFlight_ = false;
    bundleCount_ = 0;
    bundle_.failures++;
    if (fault != BundleFault::Timeout) onlyTimeouts_ = false;
    if (++bundleFailuresInRow_ < c_.fallBackAfter) return;
    if (onlyTimeouts_) {
      confirming_ = true;  // ask for one PID alone before deciding
      return;
    }
    fallBack(fault);
  }

  void fallBack(BundleFault fault) {
    confirming_ = false;
    bundling_ = false;
    bundle_.active = false;
    bundle_.fellBack = fellBack_ = true;
    bundle_.fault = fault;
  }

  // The ECU answered the one PID asked for alone, after multi-PID requests
  // timed out: the caller settles what that means.
  void confirmed() { checkAnswered_ = true; }

  // One PID alone, the check channel (else the first on the schedule), to
  // see whether the ECU still answers anything.
  bool confirmRequest(uint32_t nowMs, CanFrame& out) {
    size_t pick = checkChannel_ < N && scheduledChannel(checkChannel_) ? checkChannel_ : N;
    for (size_t i = 0; i < N && pick == N; i++)
      if (scheduledChannel(i)) pick = i;
    if (pick == N) {
      fallBack(BundleFault::Timeout);  // nothing on the schedule to ask for
      return false;
    }
    ch_[pick].requests++;
    return send(pick, ch_[pick].pid, nowMs, out);
  }

  // The next multi-PID request, once its cycle has come: the channels whose
  // turn it is, after any left over from a full request before.
  bool nextBundle(uint32_t nowMs, CanFrame& out) {
    if (!scheduled_) startSchedule(nowMs);
    if (static_cast<int32_t>(nowMs - nextCycleMs_) < 0) return false;
    if (nowMs - nextCycleMs_ >= c_.cycleMs) {
      // More than a cycle behind (a slow reply): carry on from now rather
      // than catch up in a burst. Every channel keeps its turns.
      bundle_.late++;
      nextCycleMs_ = nowMs;
    }
    for (size_t i = 0; i < N; i++) {
      if (!scheduledChannel(i) || due_[i] || cycle_ % ch_[i].every != ch_[i].offset) continue;
      due_[i] = true;
      dueCycle_[i] = cycle_;
    }
    cycle_++;
    nextCycleMs_ += c_.cycleMs;
    // Up to maxPerRequest of the due channels: the longest waiting first,
    // then in table order.
    uint8_t pids[obd2::kMaxPidsPerRequest];
    bundleCount_ = 0;
    const size_t most = c_.maxPerRequest < obd2::kMaxPidsPerRequest ? c_.maxPerRequest : obd2::kMaxPidsPerRequest;
    while (bundleCount_ < most) {
      size_t pick = N;
      for (size_t i = 0; i < N; i++) {
        if (!due_[i] || !scheduledChannel(i)) continue;
        if (pick == N || cycle_ - dueCycle_[i] > cycle_ - dueCycle_[pick]) pick = i;
      }
      if (pick == N) break;
      due_[pick] = false;
      ch_[pick].requests++;
      bundleIndex_[bundleCount_] = pick;
      pids[bundleCount_++] = ch_[pick].pid;
    }
    if (bundleCount_ == 0) return false;
    bundle_.requests++;
    bundle_.pidsAsked += bundleCount_;
    rx_.reset();
    flowControl_ = false;
    out = obd2::mode01Request(pids, bundleCount_, sendId());
    inFlight_ = true;
    inFlightIndex_ = N;
    sentMs_ = lastSentMs_ = nowMs;
    return true;
  }

  // Channels on the schedule: supported, and not refused. One that has
  // stopped answering keeps its turns; leaving it in costs a byte.
  bool scheduledChannel(size_t i) const {
    return ch_[i].every > 0 && (ch_[i].state == ChannelState::Polling || ch_[i].state == ChannelState::Silent);
  }

  void startSchedule(uint32_t nowMs) {
    uint32_t every[N];
    uint32_t offset[N];
    bool active[N];
    for (size_t i = 0; i < N; i++) {
      active[i] = ch_[i].asked();
      every[i] = ch_[i].targetPeriodMs / c_.cycleMs;
      if (every[i] == 0) every[i] = 1;
    }
    bundle_.busiest = static_cast<uint8_t>(buildSchedule(every, active, N, offset));
    for (size_t i = 0; i < N; i++) {
      ch_[i].every = active[i] ? every[i] : 0;
      ch_[i].offset = active[i] ? offset[i] : 0;
      due_[i] = false;
    }
    scheduled_ = startedBundling_ = bundle_.active = true;
    cycle_ = 0;
    nextCycleMs_ = nowMs;
  }

  void record(PollChannel& c, const uint8_t* data, uint8_t len, uint32_t replyMs, uint32_t nowMs) {
    c.replies++;
    c.replyMsTotal += replyMs;
    if (replyMs > c.replyMsMax) c.replyMsMax = replyMs;
    c.timeoutsInRow = 0;
    c.state = ChannelState::Polling;
    c.valid = true;
    c.lastReplyMs = nowMs;
    c.len = len < sizeof(c.data) ? len : sizeof(c.data);
    memset(c.data, 0, sizeof(c.data));
    memcpy(c.data, data, c.len);
  }

  bool negative(uint8_t nrc, uint32_t nowMs) {
    lastAnyReplyMs_ = nowMs;
    if (nrc == obd2::kResponsePending) {
      sentMs_ = nowMs;  // still working on it: wait again
      return true;
    }
    if (bundleCount_ > 0) {
      // To a multi-PID request: "not supported" is about the request, not
      // its channels. Anything else (busy): its channels come again on
      // their next turn.
      if (obd2::isNotSupported(nrc)) {
        bundleFailed(BundleFault::NotSupported);
      } else {
        inFlight_ = false;
        bundleCount_ = 0;
      }
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

  using Rx = IsoTpReceiver<1 + obd2::kMaxPidsPerRequest * (1 + obd2::kLongestPid)>;

  PollerConfig c_;
  PollChannel ch_[N];
  uint8_t length_[N] = {};
  PollChannelConfig own_[N] = {};  // each channel's own PID
  PollChannelConfig alt_[N] = {};  // ... and its second choice; PID 0 for none
  Seen ownSeen_[N] = {};
  Seen altSeen_[N] = {};
  // The later support bitmaps (0x20, 0x40, ... 0xE0) read this session.
  static constexpr size_t kLaterBitmaps = 7;
  uint32_t later_[kLaterBitmaps] = {};
  uint8_t laterRead_ = 0;
  uint32_t lastAskedMs_[N] = {};
  // Multi-PID polling.
  bool bundling_ = false;     // allowed this session, until it falls back
  bool scheduled_ = false;    // the schedule is built
  bool startedBundling_ = false;
  bool fellBack_ = false;
  bool flowControl_ = false;  // a first frame came: flow control to send
  uint8_t bundleFailuresInRow_ = 0;
  bool onlyTimeouts_ = true;  // ... and all of them timeouts
  bool confirming_ = false;   // asking one PID alone before falling back
  bool checkAnswered_ = false;  // ... answered: waiting for settleCheck()
  size_t checkChannel_ = N;
  BundleStats bundle_ = {};
  uint32_t cycle_ = 0;
  uint32_t nextCycleMs_ = 0;
  bool due_[N] = {};
  uint32_t dueCycle_[N] = {};
  size_t bundleIndex_[obd2::kMaxPidsPerRequest] = {};
  size_t bundleCount_ = 0;    // channels in the multi-PID request in flight; 0: none
  Rx rx_;
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
