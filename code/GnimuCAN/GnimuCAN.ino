// Gnimu-CAN: polls OBD-II PIDs over CAN and publishes them over BLE.

#include "src/ble_link.h"
#include "src/can_bus.h"
#include "Config.h"
#include "src/obd2.h"

static size_t nextPid = 0;
static bool canReady = false;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nGnimu-CAN starting");

  canReady = can_bus::begin(CAN_TX_PIN, CAN_RX_PIN, CAN_BITRATE);
  ble_link::begin();
}

void loop() {
  // Only load the vehicle bus when someone is listening.
  if (!canReady || !ble_link::isConnected()) {
    delay(100);
    return;
  }

  can_bus::service();

  const uint8_t pid = OBD_PIDS[nextPid];
  nextPid = (nextPid + 1) % (sizeof(OBD_PIDS) / sizeof(OBD_PIDS[0]));

  obd2::Reading reading;
  if (obd2::requestPid(pid, OBD_RESPONSE_TIMEOUT_MS, reading)) {
    ble_link::publish(reading);
    Serial.printf("[obd] pid %02X ecu %u len %u: %02X %02X %02X %02X\n", reading.pid, reading.ecu, reading.len,
                  reading.data[0], reading.data[1], reading.data[2], reading.data[3]);
  }

  delay(OBD_POLL_GAP_MS);
}
