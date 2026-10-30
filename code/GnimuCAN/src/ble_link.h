#pragma once

#include <Arduino.h>

#include "obd2.h"

// BLE GATT server exposing one notify characteristic for readings.
namespace ble_link {

void begin();

bool isConnected();

// Packs the reading per docs/ble-protocol.md and notifies subscribers.
void publish(const obd2::Reading &reading);

}  // namespace ble_link
