// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "../../app/Console.h"
#include "../../core/BleStats.h"
#include "BleLink.h"

namespace gnimu {

// The Gnimu protocol's UUIDs, the same on every Gnimu device
// (Gnimu-protocol, docs/frame.md).
constexpr char kGnimuServiceUuid[] = "9074beb4-2f1a-4b5f-9818-fe2d2277ef05";
constexpr char kGnimuFrameV1Uuid[] = "0752d3c3-ceaf-462f-a944-bcc06c57284b";
constexpr char kGnimuSchemaUuid[] = "1c9b3945-10eb-4fd0-a5a4-b9d80842380d";
constexpr char kGnimuDeviceInfoUuid[] = "6d0adfef-8218-488a-a5f9-1ba05f422c22";

// What the peripheral shows a client: its name, the Device Information
// Service strings, and the two JSON documents. The stack copies every value,
// so none needs to outlive begin().
struct BleIdentity {
  const char* name;  // advertised
  const char* model;
  const char* manufacturer;
  const char* serial;
  const char* firmwareRevision;
  const char* hardwareRevision;
  const char* schemaJson;
  const char* deviceInfoJson;
};

// The Gnimu service on the C6's NimBLE stack: the frame (notify, read), the
// schema and Device info (read), and the Device Information Service. One client
// at a time: advertising stops while one is connected and resumes when it
// leaves. Stack callbacks run on the BLE host's task, so they only update
// atomics; everything else, including logging, happens on the loop.
class BlePeripheral {
 public:
  // Builds the GATT table and starts advertising. True once advertising.
  bool begin(const BleIdentity& id, Console& log);
  // Loop housekeeping: logs connection edges.
  void update();
  bool connected() const;
  // Notifies one frame. False (not sent) when there's no connection, the
  // peer's MTU can't carry the whole frame, or the stack didn't take it
  // (notifications not enabled, for one).
  bool send(const uint8_t* data, size_t len);
  BleStats stats() const;
  void clearMaxSend() { link_.clearMaxSend(); }

 private:
  BleLink link_;  // MTU refusal, send counters, connection-edge logging
  bool started_ = false;
};

}  // namespace gnimu
