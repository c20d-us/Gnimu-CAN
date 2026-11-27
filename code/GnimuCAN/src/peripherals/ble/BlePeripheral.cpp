// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#if defined(ARDUINO_ARCH_ESP32)

#include "BlePeripheral.h"

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>

#include <atomic>

#include "../../platform/Clock.h"

#if !defined(CONFIG_NIMBLE_ENABLED)
#error "BlePeripheral expects the NimBLE stack, as the ESP32-C6 core builds by default"
#endif

namespace gnimu {

namespace {

constexpr char kDeviceInfoServiceUuid[] = "0000180a-0000-1000-8000-00805f9b34fb";
// The largest ATT MTU we accept. The frame needs the peer to negotiate one
// a few bytes over its length; iOS offers 185.
constexpr uint16_t kLocalMtu = 247;

BLEServer* server = nullptr;
BLECharacteristic* frameChar = nullptr;
BLECharacteristic* consoleChar = nullptr;
std::atomic<bool> linkUp{false};
std::atomic<uint16_t> peerMtu{23};  // ATT default until the exchange raises it

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    // Reset the MTU before publishing the link, so send() never pairs a new
    // connection with the previous peer's MTU.
    peerMtu.store(23, std::memory_order_release);
    linkUp.store(true, std::memory_order_release);
  }
  void onDisconnect(BLEServer*) override { linkUp.store(false, std::memory_order_release); }
  void onMtuChanged(BLEServer*, ble_gap_conn_desc*, uint16_t mtu) override {
    peerMtu.store(mtu, std::memory_order_release);
  }
};

// notify() returns nothing; it reports what it did through onStatus(), called
// synchronously from inside notify() on the caller's (loop) task.
using NotifyStatus = BLECharacteristicCallbacks::Status;
NotifyStatus lastNotifyStatus = NotifyStatus::ERROR_NO_CLIENT;

class FrameCallbacks : public BLECharacteristicCallbacks {
  void onStatus(BLECharacteristic*, Status s, uint32_t) override { lastNotifyStatus = s; }
};

// A written Console command, handed from the BLE host's task to the loop: the
// host writes it only while `commandReady` is clear, and the loop clears it
// once it has copied the command out. A command written before the last was
// taken is dropped.
char pendingCommand[kBleCommandMax + 1] = {};
std::atomic<bool> commandReady{false};
NotifyStatus lastConsoleStatus = NotifyStatus::ERROR_NO_CLIENT;

class ConsoleCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    if (commandReady.load(std::memory_order_acquire)) return;
    const String v = c->getValue();
    const size_t n = v.length() < kBleCommandMax ? v.length() : kBleCommandMax;
    memcpy(pendingCommand, v.c_str(), n);
    pendingCommand[n] = '\0';
    commandReady.store(true, std::memory_order_release);
  }
  void onStatus(BLECharacteristic*, Status s, uint32_t) override { lastConsoleStatus = s; }
};

void addReadOnly(BLEService* service, const char* uuid, const char* value) {
  service->createCharacteristic(uuid, BLECharacteristic::PROPERTY_READ)
      ->setValue(reinterpret_cast<uint8_t*>(const_cast<char*>(value)), strlen(value));
}

}  // namespace

bool BlePeripheral::begin(const BleIdentity& id, Console& log) {
  link_.setLog(log);
  if (started_) return true;
  if (!BLEDevice::init(id.name)) {
    log.log("BLE: stack failed to start \xE2\x9D\x8C");
    return false;
  }
  BLEDevice::setMTU(kLocalMtu);

  server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService* gnimu = server->createService(kGnimuServiceUuid);
  frameChar = gnimu->createCharacteristic(kGnimuFrameV1Uuid,
                                          BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  frameChar->setCallbacks(new FrameCallbacks());
  addReadOnly(gnimu, kGnimuSchemaUuid, id.schemaJson);
  addReadOnly(gnimu, kGnimuDeviceInfoUuid, id.deviceInfoJson);
  consoleChar = gnimu->createCharacteristic(kGnimuConsoleUuid, BLECharacteristic::PROPERTY_WRITE |
                                                                   BLECharacteristic::PROPERTY_WRITE_NR |
                                                                   BLECharacteristic::PROPERTY_NOTIFY);
  consoleChar->setCallbacks(new ConsoleCallbacks());
  gnimu->start();

  BLEService* info = server->createService(kDeviceInfoServiceUuid);
  addReadOnly(info, "00002a29-0000-1000-8000-00805f9b34fb", id.manufacturer);
  addReadOnly(info, "00002a24-0000-1000-8000-00805f9b34fb", id.model);
  addReadOnly(info, "00002a25-0000-1000-8000-00805f9b34fb", id.serial);
  addReadOnly(info, "00002a26-0000-1000-8000-00805f9b34fb", id.firmwareRevision);
  addReadOnly(info, "00002a27-0000-1000-8000-00805f9b34fb", id.hardwareRevision);
  info->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(kGnimuServiceUuid);
  adv->setScanResponse(true);
  // The library restarts advertising from its own disconnect event, so a
  // connect and disconnect between two update() passes can't strand it.
  server->advertiseOnDisconnect(true);
  started_ = adv->start();
  log.log("BLE: advertising as \"%s\" %s", id.name, started_ ? "\xE2\x9C\x85" : "\xE2\x9D\x8C");
  return started_;
}

void BlePeripheral::update() {
  if (!started_) return;
  link_.edges(linkUp.load(std::memory_order_acquire));
}

bool BlePeripheral::connected() const { return started_ && linkUp.load(std::memory_order_acquire); }

bool BlePeripheral::send(const uint8_t* data, size_t len) {
  if (!connected()) return false;
  // A notification carries MTU - 3 bytes; the stack would cut anything longer
  // short, so it's refused here instead.
  if (!link_.admit(len, peerMtu.load(std::memory_order_acquire))) return false;
  const uint64_t start = PlatformClock::nowUs();
  lastNotifyStatus = NotifyStatus::ERROR_NO_CLIENT;
  frameChar->setValue(const_cast<uint8_t*>(data), len);
  frameChar->notify();
  // Connected but not (yet) subscribed, like a scanner app before notify is
  // switched on: nothing to send to, and not a failure.
  if (lastNotifyStatus == NotifyStatus::ERROR_NO_SUBSCRIBER) return false;
  return link_.sent(lastNotifyStatus == NotifyStatus::SUCCESS_NOTIFY, start);
}

bool BlePeripheral::takeCommand(char (&out)[kBleCommandMax + 1]) {
  if (!commandReady.load(std::memory_order_acquire)) return false;
  memcpy(out, pendingCommand, sizeof(out));
  commandReady.store(false, std::memory_order_release);
  return true;
}

ConsoleSend BlePeripheral::sendConsole(const char* data, size_t len) {
  if (!connected() || consoleChar == nullptr) return ConsoleSend::NotListening;
  lastConsoleStatus = NotifyStatus::ERROR_NO_CLIENT;
  consoleChar->setValue(reinterpret_cast<uint8_t*>(const_cast<char*>(data)), len);
  consoleChar->notify();
  switch (lastConsoleStatus) {
    case NotifyStatus::SUCCESS_NOTIFY:
      return ConsoleSend::Sent;
    case NotifyStatus::ERROR_NO_SUBSCRIBER:
    case NotifyStatus::ERROR_NO_CLIENT:
      return ConsoleSend::NotListening;
    default:
      return ConsoleSend::Busy;
  }
}

uint16_t BlePeripheral::mtu() const { return connected() ? peerMtu.load(std::memory_order_acquire) : 23; }

BleStats BlePeripheral::stats() const {
  BleStats s = link_.stats();
  s.mtu = connected() ? peerMtu.load(std::memory_order_acquire) : 0;
  return s;
}

}  // namespace gnimu

#endif  // ARDUINO_ARCH_ESP32
