// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdint.h>

#include "../core/CanFrame.h"
#include "../core/ProbePolicy.h"

namespace gnimu {

// How a single-shot frame went, as far as the controller knows yet.
enum class TxOutcome : uint8_t {
  Pending,  // still waiting to go, or no word from the controller yet
  Sent,     // on the bus and acknowledged
  Failed,   // not acknowledged, or the controller couldn't send it
};

// The ESP32-C6's TWAI (CAN) controller and the CAN Pal transceiver's silent
// pin. The transceiver stays listen-only (S high, held by the carrier's
// pull-up) except during a transmit the firmware has decided to make.
struct CanPort {
  // Configures the silent pin (released: listen-only). Call once at boot.
  static void init();
  // Starts the controller at `bitrate` (500000 or 250000), listen-only or
  // normal, with the transceiver to match: silent for listen-only, active for
  // normal. Stops any earlier start first. False if the driver wouldn't start.
  static bool start(uint32_t bitrate, bool listenOnly);
  // Stops the controller and silences the transceiver.
  static void stop();
  // Queues one frame (retransmitted until acknowledged, as normal). Needs a
  // normal-mode start. False if the queue is full or the controller isn't
  // able to send.
  static bool send(const CanFrame& f);
  // Takes the next received frame, if any. Never waits.
  static bool receive(CanFrame& f);
  // Queues one single-shot frame: sent once, never retransmitted, so a wrong
  // bit rate puts at most one bad frame on the bus. Needs a normal-mode
  // start. Never waits; txOutcome() says how it went.
  static bool sendSingleShot(const CanFrame& f);
  // How the last single-shot frame went. Never waits; call it on later runs
  // until it stops saying Pending.
  static TxOutcome txOutcome();
  static CanStatus status();
};

}  // namespace gnimu
