// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

namespace gnimu {

// The XIAO's yellow user LED. core/StatusLed decides what to show; this only
// renders it.
struct StatusLedPort {
  static void begin();  // configures the pin, LED off
  static void write(bool on);
};

}  // namespace gnimu
