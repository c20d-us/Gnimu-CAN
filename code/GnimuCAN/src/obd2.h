#pragma once

#include <Arduino.h>

// OBD-II (ISO 15765-4) Mode 01 requests over 11-bit CAN.
namespace obd2 {

constexpr uint32_t FUNCTIONAL_REQUEST_ID = 0x7DF;
constexpr uint32_t FIRST_RESPONSE_ID = 0x7E8;
constexpr uint32_t LAST_RESPONSE_ID = 0x7EF;

struct Reading {
  uint8_t mode;     // response mode with 0x40 removed (0x01 for Mode 01)
  uint8_t pid;
  uint8_t ecu;      // responder index: CAN ID - 0x7E8
  uint8_t len;      // number of valid bytes in data
  uint8_t data[4];  // raw A..D bytes, decoded by the receiver
  uint32_t timestampMs;
};

// Sends a Mode 01 request for pid and waits for the first single-frame
// answer. Returns false if no ECU responded within timeoutMs.
bool requestPid(uint8_t pid, uint32_t timeoutMs, Reading &out);

}  // namespace obd2
