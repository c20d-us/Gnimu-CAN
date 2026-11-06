// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdint.h>
#include <string.h>

#include "CanFrame.h"

namespace gnimu {
namespace obd2 {

// OBD-II over CAN (ISO 15765-4), 11-bit IDs, Mode 01 single frames.
constexpr uint32_t kFunctionalRequestId = 0x7DF;  // every ECU listens
constexpr uint32_t kFirstResponseId = 0x7E8;      // ECU n answers on 0x7E8 + n
constexpr uint32_t kLastResponseId = 0x7EF;
constexpr uint8_t kModeCurrentData = 0x01;
constexpr uint8_t kPositiveResponse = 0x40;  // added to the mode in a reply
constexpr uint8_t kPadByte = 0x55;           // ISO 15765-4's recommended padding

// A Mode 01 request for `pid` to `requestId` (the functional 0x7DF, or an
// ECU's physical ID once one is known): an ISO-TP single frame of length 2,
// padded to 8 bytes.
inline CanFrame mode01Request(uint8_t pid, uint32_t requestId = kFunctionalRequestId) {
  CanFrame f = {requestId, false, 8, {0x02, kModeCurrentData, pid, kPadByte, kPadByte, kPadByte, kPadByte, kPadByte}};
  return f;
}

struct Mode01Reply {
  uint8_t ecu;      // responder index: CAN ID - 0x7E8
  uint8_t pid;
  uint8_t len;      // valid bytes in data
  uint8_t data[4];  // A..D
};

// True, filling `out`, when `f` is a single-frame positive reply to a Mode 01
// request for `pid` from an ECU in 0x7E8-0x7EF. Anything else (other traffic,
// another PID, a multi-frame reply, a negative response) is false.
inline bool parseMode01Reply(const CanFrame& f, uint8_t pid, Mode01Reply& out) {
  if (f.extended || f.id < kFirstResponseId || f.id > kLastResponseId || f.len < 3) return false;
  const uint8_t pci = f.data[0];
  const uint8_t payload = pci & 0x0F;
  // A single frame (PCI high nibble 0) holding at least the mode and PID, and
  // no longer than the CAN frame.
  if ((pci >> 4) != 0 || payload < 2 || payload + 1u > f.len) return false;
  if (f.data[1] != kModeCurrentData + kPositiveResponse || f.data[2] != pid) return false;
  out.ecu = static_cast<uint8_t>(f.id - kFirstResponseId);
  out.pid = pid;
  out.len = static_cast<uint8_t>(payload - 2 > 4 ? 4 : payload - 2);
  memset(out.data, 0, sizeof(out.data));
  memcpy(out.data, &f.data[3], out.len);
  return true;
}

}  // namespace obd2
}  // namespace gnimu
