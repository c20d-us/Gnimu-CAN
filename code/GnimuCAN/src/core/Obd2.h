// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "CanFrame.h"

namespace gnimu {
namespace obd2 {

// OBD-II over CAN (ISO 15765-4), 11-bit IDs, Mode 01.
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

// The most PIDs one Mode 01 request may ask for (SAE J1979, over CAN only).
constexpr size_t kMaxPidsPerRequest = 6;

// A Mode 01 request for `count` PIDs at once (1 to kMaxPidsPerRequest): a
// single frame, padded to 8 bytes. The reply carries each supported PID with
// its data, in any order (splitMode01Reply).
inline CanFrame mode01Request(const uint8_t* pids, size_t count, uint32_t requestId) {
  if (count > kMaxPidsPerRequest) count = kMaxPidsPerRequest;
  CanFrame f = {requestId, false, 8, {static_cast<uint8_t>(1 + count), kModeCurrentData, kPadByte, kPadByte, kPadByte,
                                      kPadByte, kPadByte, kPadByte}};
  for (size_t i = 0; i < count; i++) f.data[2 + i] = pids[i];
  return f;
}

// The ISO-TP flow control a receiver sends to `id` after a first frame:
// continue, send the rest with no limit on the block size, no gap between
// frames.
inline CanFrame flowControl(uint32_t id) {
  CanFrame f = {id, false, 8, {0x30, 0x00, 0x00, kPadByte, kPadByte, kPadByte, kPadByte, kPadByte}};
  return f;
}

// Splits a reply to a request for several PIDs: its payload is 0x41, then
// each PID the ECU supports followed by its data bytes, in any order, and
// nothing after. `pids` and `lengths` are what was asked for and each one's
// data length; `offsets[i]` gets where pids[i]'s data starts in `payload`, or
// 0 when the reply left it out. False, when the reply doesn't split exactly
// (a PID not asked for or repeated, a length that doesn't add up, nothing
// answered): the whole reply is untrustworthy.
inline bool splitMode01Reply(const uint8_t* payload, size_t len, const uint8_t* pids, const uint8_t* lengths,
                             size_t count, uint8_t* offsets) {
  for (size_t j = 0; j < count; j++) offsets[j] = 0;
  if (len < 1 || payload[0] != kModeCurrentData + kPositiveResponse) return false;
  size_t at = 1;
  size_t answered = 0;
  while (at < len) {
    size_t j = 0;
    while (j < count && pids[j] != payload[at]) j++;
    if (j == count || offsets[j] != 0 || at + 1 + lengths[j] > len) return false;
    offsets[j] = static_cast<uint8_t>(at + 1);
    at += 1 + lengths[j];
    answered++;
  }
  return answered > 0;
}

// Whether `pid` (0x01-0x20) is set in a PID 0x00 support bitmap, where the
// bitmap's top bit is PID 0x01.
inline bool pidSupported(uint32_t bitmap, uint8_t pid) {
  if (pid < 0x01 || pid > 0x20) return false;
  return (bitmap >> (32 - pid)) & 1u;
}

// Room for formatResponseIds(): eight "0x7E8, " entries and the terminator.
constexpr size_t kResponseIdsTextMax = 8 * 7 + 1;

// The response IDs in `mask` (bit n: 0x7E8 + n), as "0x7E8, 0x7E9". Empty
// for no bits.
inline void formatResponseIds(char* out, size_t cap, uint8_t mask) {
  size_t len = 0;
  out[0] = '\0';
  for (uint8_t n = 0; n < 8; n++) {
    if (!(mask & (1u << n)) || len >= cap) continue;
    const int w = snprintf(out + len, cap - len, "%s0x%03lX", len ? ", " : "",
                           static_cast<unsigned long>(kFirstResponseId + n));
    if (w > 0) len += static_cast<size_t>(w);
  }
}

// Room for formatSupportLines(): the first line, and up to 32 PIDs wrapped
// over the next two.
constexpr size_t kSupportLinesMax = 192;

// A support bitmap as console lines, each within `columns` (at least 40):
// "OBD: 0x7E8 PIDs 01-20 (BE1FA813): 01 03 04 ..." then the PIDs it lists,
// wrapped onto lines indented by 5 spaces; "none" for no bits. `base` is the
// bitmap's PID (0x00, 0x20, ...), so it covers base + 1 to base + 0x20. Lines
// end in '\n'. Truncated, never overrun; returns the length.
inline size_t formatSupportLines(char* out, size_t cap, uint32_t responseId, uint8_t base, uint32_t bitmap,
                                 size_t columns) {
  if (cap == 0) return 0;
  out[0] = '\0';
  int w = snprintf(out, cap, "OBD: 0x%03lX PIDs %02X-%02X (%08lX):", static_cast<unsigned long>(responseId),
                   static_cast<unsigned>(base + 1), static_cast<unsigned>(base + 0x20),
                   static_cast<unsigned long>(bitmap));
  if (w < 0) return 0;
  size_t len = static_cast<size_t>(w) < cap ? static_cast<size_t>(w) : cap - 1;
  size_t col = len;
  const auto put = [&](const char* text) {
    const int n = snprintf(out + len, cap - len, "%s", text);
    if (n > 0) len += static_cast<size_t>(n) < cap - len ? static_cast<size_t>(n) : cap - len - 1;
  };
  if (bitmap == 0) put(" none");
  for (uint8_t i = 1; i <= 0x20; i++) {
    if (!pidSupported(bitmap, i)) continue;
    if (col + 3 > columns) {
      put("\n    ");
      col = 4;
    }
    char pid[4];
    snprintf(pid, sizeof(pid), " %02X", static_cast<unsigned>((base + i) & 0xFF));
    put(pid);
    col += 3;
  }
  put("\n");
  return len;
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

// Negative responses: 7F, the requested mode, then a response code.
constexpr uint8_t kNegativeResponse = 0x7F;
constexpr uint8_t kResponsePending = 0x78;  // still working on it: wait again
constexpr uint8_t kBusyRepeatRequest = 0x21;

// Codes that mean the ECU doesn't support what was asked.
inline bool isNotSupported(uint8_t nrc) { return nrc == 0x11 || nrc == 0x12 || nrc == 0x31; }

// True, filling `nrc`, when `f` is a single-frame negative response to a Mode
// 01 request from an ECU in 0x7E8-0x7EF.
inline bool parseNegativeReply(const CanFrame& f, uint8_t& nrc) {
  if (f.extended || f.id < kFirstResponseId || f.id > kLastResponseId || f.len < 4) return false;
  if (f.data[0] != 0x03 || f.data[1] != kNegativeResponse || f.data[2] != kModeCurrentData) return false;
  nrc = f.data[3];
  return true;
}

}  // namespace obd2
}  // namespace gnimu
