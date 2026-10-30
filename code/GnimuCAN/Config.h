#pragma once

#include <Arduino.h>
#include <driver/gpio.h>

// ---- CAN transceiver wiring (XIAO ESP32C6) ----
// The C6 has no on-board transceiver; wire an external 3.3 V part
// (SN65HVD230, TCAN332, ...) to these pins.
constexpr gpio_num_t CAN_TX_PIN = GPIO_NUM_16;  // XIAO D6
constexpr gpio_num_t CAN_RX_PIN = GPIO_NUM_17;  // XIAO D7

// Most OBD-II CAN vehicles run 500 kbit/s; some older ones use 250 kbit/s.
constexpr uint32_t CAN_BITRATE = 500000;

// ---- OBD-II polling ----
constexpr uint32_t OBD_RESPONSE_TIMEOUT_MS = 50;  // per request
constexpr uint32_t OBD_POLL_GAP_MS = 10;          // pause between requests

// Mode 01 PIDs polled round-robin.
constexpr uint8_t OBD_PIDS[] = {
  0x0C,  // Engine RPM
  0x0D,  // Vehicle speed
  0x05,  // Coolant temperature
  0x11,  // Throttle position
  0x04,  // Calculated engine load
  0x0F,  // Intake air temperature
};

// ---- BLE ----
constexpr const char *BLE_DEVICE_NAME = "Gnimu-CAN";
constexpr const char *BLE_SERVICE_UUID = "9074beb4-2f1a-4b5f-9818-fe2d2277ef05";
constexpr const char *BLE_READING_CHAR_UUID = "0752d3c3-ceaf-462f-a944-bcc06c57284b";

// Version byte at the start of every notification; bump on format changes.
// See docs/ble-protocol.md.
constexpr uint8_t BLE_PROTOCOL_VERSION = 1;
