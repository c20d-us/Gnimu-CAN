#include "ble_link.h"

#include <BLEDevice.h>
#include <BLEServer.h>

#include "../Config.h"

namespace ble_link {

static BLECharacteristic *readingChar = nullptr;
static volatile bool connected = false;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *) override {
    connected = true;
    Serial.println("[ble] client connected");
  }

  void onDisconnect(BLEServer *) override {
    connected = false;
    Serial.println("[ble] client disconnected, advertising");
    BLEDevice::startAdvertising();
  }
};

void begin() {
  BLEDevice::init(BLE_DEVICE_NAME);

  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService *service = server->createService(BLE_SERVICE_UUID);
  readingChar = service->createCharacteristic(
    BLE_READING_CHAR_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(BLE_SERVICE_UUID);
  advertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.printf("[ble] advertising as \"%s\"\n", BLE_DEVICE_NAME);
}

bool isConnected() {
  return connected;
}

void publish(const obd2::Reading &reading) {
  if (readingChar == nullptr) {
    return;
  }

  // 9-byte header + up to 4 data bytes: fits the default 20-byte ATT payload.
  uint8_t frame[13];
  frame[0] = BLE_PROTOCOL_VERSION;
  frame[1] = reading.mode;
  frame[2] = reading.pid;
  frame[3] = reading.ecu;
  frame[4] = reading.timestampMs & 0xFF;
  frame[5] = (reading.timestampMs >> 8) & 0xFF;
  frame[6] = (reading.timestampMs >> 16) & 0xFF;
  frame[7] = (reading.timestampMs >> 24) & 0xFF;
  frame[8] = reading.len;
  memcpy(&frame[9], reading.data, reading.len);

  readingChar->setValue(frame, 9 + reading.len);
  readingChar->notify();
}

}  // namespace ble_link
