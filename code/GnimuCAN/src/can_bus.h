#pragma once

#include <Arduino.h>
#include <driver/twai.h>

// Thin wrapper around the ESP32 TWAI (CAN) controller, 11-bit IDs only.
namespace can_bus {

bool begin(gpio_num_t txPin, gpio_num_t rxPin, uint32_t bitrate);

bool send(uint32_t id, const uint8_t *data, uint8_t len);

// Waits up to timeoutMs for a frame. Returns false on timeout.
bool receive(twai_message_t &msg, uint32_t timeoutMs);

// Restarts the controller after bus-off; call regularly from loop().
void service();

}  // namespace can_bus
