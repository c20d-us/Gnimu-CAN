// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdint.h>

namespace gnimu {

// One classic CAN frame, independent of the controller's driver types.
struct CanFrame {
  uint32_t id;
  bool extended;  // 29-bit ID
  uint8_t len;    // 0-8
  uint8_t data[8];
};

// The controller's view of the bus, for the D report.
struct CanStatus {
  bool running;        // the controller is started
  bool listenOnly;     // started in listen-only mode
  bool busOff;
  uint32_t bitrate;    // as started, 0 when stopped
  uint32_t txErrors;   // the controller's transmit error counter
  uint32_t rxErrors;   // its receive error counter
  uint32_t busErrors;  // bus errors seen since this start
  uint32_t rxMissed;   // frames lost to a full receive queue since this start
};

}  // namespace gnimu
