// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

// ============================================================================
// TOOL: Board check
//
// Confirms a XIAO ESP32C6 builds, flashes and runs from the command line, and
// that its basic hardware works:
//
//   - prints the chip, flash, MAC, core and reset reason at boot;
//   - blinks the yellow user LED once a second;
//   - advertises over BLE as "Gnimu-CAN check XXXX" (the last four MAC
//     digits), so a phone scanner such as nRF Connect or LightBlue can see it;
//   - prints a status line once a second: uptime, free heap, the A0 voltage
//     (and the supply it implies through the carrier's ÷11 divider), and
//     whether a BLE central is connected;
//   - echoes each line typed on the console, and restarts on "R".
//
// It touches no CAN pins. D8 (CAN_SILENT) is left as an input so the CAN Pal's
// pull-up keeps it listen-only.
//
// Target: Seeed XIAO ESP32C6, serial console at 115200.
// Build and flash: scripts/tool.sh board_check --flash --monitor
// ============================================================================

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <esp_mac.h>
#include <esp_system.h>

#if !defined(CONFIG_IDF_TARGET_ESP32C6)
#error "board_check supports the ESP32-C6 only"
#endif

namespace {

constexpr uint8_t kLedPin = LED_BUILTIN;  // GPIO15, active low
constexpr uint8_t kSensePin = A0;         // V12_SENSE on the carrier
constexpr float kDivider = 11.0f;         // 100k + 10k
constexpr uint32_t kStatusPeriodMs = 1000;
constexpr uint32_t kSerialWaitMs = 3000;

bool bleConnected = false;
bool bleOk = false;
String lineBuffer;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *) override { bleConnected = true; }

  void onDisconnect(BLEServer *) override {
    bleConnected = false;
    BLEDevice::getAdvertising()->start();
  }
};

const char *resetReasonName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON: return "power-on";
    case ESP_RST_EXT: return "external pin";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "interrupt watchdog";
    case ESP_RST_TASK_WDT: return "task watchdog";
    case ESP_RST_WDT: return "other watchdog";
    case ESP_RST_DEEPSLEEP: return "deep sleep wake";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_USB: return "USB";
    case ESP_RST_JTAG: return "JTAG";
    default: return "unknown";
  }
}

void printBoardInfo() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_BT);

  Serial.println();
  Serial.println("Gnimu-CAN board check");
  Serial.printf("Chip: %s rev %u.%u, %u core(s), %lu MHz\n", ESP.getChipModel(), ESP.getChipRevision() / 100,
                ESP.getChipRevision() % 100, ESP.getChipCores(), (unsigned long)ESP.getCpuFreqMHz());
  Serial.printf("Flash: %lu KB at %lu MHz\n", (unsigned long)(ESP.getFlashChipSize() / 1024),
                (unsigned long)(ESP.getFlashChipSpeed() / 1000000));
  Serial.printf("BLE MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.printf("Heap: %lu bytes free\n", (unsigned long)ESP.getFreeHeap());
  Serial.printf("Core: Arduino-ESP32 %s, ESP-IDF %s\n", ESP_ARDUINO_VERSION_STR, ESP.getSdkVersion());
  Serial.printf("Reset: %s\n", resetReasonName(esp_reset_reason()));
}

void startBle() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_BT);
  char name[32];
  snprintf(name, sizeof(name), "Gnimu-CAN check %02X%02X", mac[4], mac[5]);

  BLEDevice::init(name);
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());
  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->setScanResponse(true);
  bleOk = advertising->start();
  Serial.printf("BLE: advertising as \"%s\" %s\n", name, bleOk ? "✅" : "❌");
}

void printStatus() {
  const uint32_t pinMv = analogReadMilliVolts(kSensePin);
  Serial.printf("up %lus | heap %lu | A0 %.3fV (supply %.2fV) | BLE %s\n", (unsigned long)(millis() / 1000),
                (unsigned long)ESP.getFreeHeap(), pinMv / 1000.0f, pinMv * kDivider / 1000.0f,
                !bleOk ? "failed" : bleConnected ? "connected" : "advertising");
}

void handleLine(const String &line) {
  if (line == "R") {
    Serial.println("Restarting");
    Serial.flush();
    ESP.restart();
  }
  Serial.printf("Echo: '%s' (R = restart)\n", line.c_str());
}

void pollConsole() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r' || c == '\n') {
      if (lineBuffer.length() > 0) {
        handleLine(lineBuffer);
        lineBuffer = "";
      }
    } else if (lineBuffer.length() < 64) {
      lineBuffer += c;
    }
  }
}

}  // namespace

void setup() {
  pinMode(kLedPin, OUTPUT);
  digitalWrite(kLedPin, HIGH);  // off
  pinMode(D8, INPUT);
  analogReadResolution(12);

  Serial.begin(115200);
  const uint32_t start = millis();
  while (!Serial && millis() - start < kSerialWaitMs) {
    delay(10);
  }

  printBoardInfo();
  startBle();
  Serial.println("Status every second. Type a line and press Enter to test console input.");
}

void loop() {
  static uint32_t lastStatus = 0;
  const uint32_t now = millis();

  digitalWrite(kLedPin, (now / 500) % 2 ? HIGH : LOW);

  if (now - lastStatus >= kStatusPeriodMs) {
    lastStatus = now;
    printStatus();
  }

  pollConsole();
  delay(5);
}
