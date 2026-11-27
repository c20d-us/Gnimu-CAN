// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace gnimu {

// Receives one ISO-TP (ISO 15765-2) message of up to Cap bytes, a CAN frame
// at a time: a single frame, or a first frame and its consecutive frames.
// Just enough for a reply to one OBD request: one message at a time, one
// flow control after the first frame (block size 0, so the sender never
// waits for another), and any fault ends the message. Timing is the
// caller's: it gives up on a message that takes too long, and calls reset()
// before each request. Pure logic.
template <size_t Cap>
class IsoTpReceiver {
 public:
  enum class Step : uint8_t {
    More,             // nothing to do yet (or a frame that isn't part of this message)
    SendFlowControl,  // a first frame: send flow control, then more follows
    Complete,         // the message is in: payload(), length()
    Failed            // a malformed frame, or one out of sequence: the message is lost
  };

  void reset() {
    state_ = State::Idle;
    len_ = total_ = 0;
  }

  // One frame from the sender: `data`, `n` bytes of it.
  Step frame(const uint8_t* data, uint8_t n) {
    if (n < 1 || state_ == State::Done || state_ == State::Lost) return Step::More;
    const uint8_t type = data[0] >> 4;
    if (type == 0) {
      // A single frame: the whole message.
      const size_t l = data[0] & 0x0F;
      if (state_ != State::Idle || l < 1 || l + 1 > n || l > Cap) return fail();
      memcpy(buf_, data + 1, l);
      len_ = total_ = l;
      state_ = State::Done;
      return Step::Complete;
    }
    if (type == 1) {
      // A first frame: a 12-bit length, then the first six bytes. Anything
      // that fits a single frame mustn't come this way.
      if (state_ != State::Idle || n < 8) return fail();
      const size_t l = (static_cast<size_t>(data[0] & 0x0F) << 8) | data[1];
      if (l < 8 || l > Cap) return fail();
      memcpy(buf_, data + 2, 6);
      len_ = 6;
      total_ = l;
      next_ = 1;
      state_ = State::Collecting;
      return Step::SendFlowControl;
    }
    if (type == 2) {
      // A consecutive frame: the next in sequence, or the message is lost.
      if (state_ != State::Collecting) return Step::More;
      if ((data[0] & 0x0F) != next_ || n < 2) return fail();
      size_t take = n - 1u;
      if (take > total_ - len_) take = total_ - len_;
      memcpy(buf_ + len_, data + 1, take);
      len_ += take;
      next_ = static_cast<uint8_t>((next_ + 1) & 0x0F);
      if (len_ < total_) return Step::More;
      state_ = State::Done;
      return Step::Complete;
    }
    return Step::More;  // a flow control, or not ISO-TP: not ours
  }

  // A first frame came, and nothing after it: the sender is still waiting
  // for flow control, or never got it.
  bool stalledAfterFirst() const { return state_ == State::Collecting && next_ == 1; }

  const uint8_t* payload() const { return buf_; }
  size_t length() const { return state_ == State::Done ? total_ : 0; }

 private:
  enum class State : uint8_t { Idle, Collecting, Done, Lost };

  Step fail() {
    state_ = State::Lost;
    return Step::Failed;
  }

  uint8_t buf_[Cap] = {};
  size_t len_ = 0;
  size_t total_ = 0;
  uint8_t next_ = 0;
  State state_ = State::Idle;
};

}  // namespace gnimu
