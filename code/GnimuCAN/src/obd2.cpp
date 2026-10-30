#include "obd2.h"

#include "can_bus.h"

namespace obd2 {

constexpr uint8_t MODE_CURRENT_DATA = 0x01;
constexpr uint8_t POSITIVE_RESPONSE_OFFSET = 0x40;
constexpr uint8_t PAD_BYTE = 0x55;

bool requestPid(uint8_t pid, uint32_t timeoutMs, Reading &out) {
  // ISO-TP single frame: PCI length, mode, PID, padding.
  const uint8_t request[8] = {0x02, MODE_CURRENT_DATA, pid, PAD_BYTE, PAD_BYTE, PAD_BYTE, PAD_BYTE, PAD_BYTE};

  // Drop stale frames so an old answer isn't mistaken for this one.
  twai_message_t msg;
  while (can_bus::receive(msg, 0)) {
  }

  if (!can_bus::send(FUNCTIONAL_REQUEST_ID, request, sizeof(request))) {
    return false;
  }

  const uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    if (!can_bus::receive(msg, timeoutMs - (millis() - start))) {
      break;
    }
    if (msg.extd || msg.rtr || msg.identifier < FIRST_RESPONSE_ID || msg.identifier > LAST_RESPONSE_ID) {
      continue;
    }
    if (msg.data_length_code < 3) {
      continue;
    }

    const uint8_t pci = msg.data[0];
    const uint8_t payloadLen = pci & 0x0F;
    // Single frames only; every Mode 01 PID fits in one.
    if ((pci >> 4) != 0 || payloadLen < 2 || payloadLen + 1 > msg.data_length_code) {
      continue;
    }
    if (msg.data[1] != (MODE_CURRENT_DATA + POSITIVE_RESPONSE_OFFSET) || msg.data[2] != pid) {
      continue;
    }

    out.mode = MODE_CURRENT_DATA;
    out.pid = pid;
    out.ecu = msg.identifier - FIRST_RESPONSE_ID;
    out.len = min<uint8_t>(payloadLen - 2, sizeof(out.data));
    memcpy(out.data, &msg.data[3], out.len);
    out.timestampMs = millis();
    return true;
  }
  return false;
}

}  // namespace obd2
