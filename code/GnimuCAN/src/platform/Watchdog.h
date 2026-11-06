// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

namespace gnimu {

// The hardware watchdog, config::kWatchdogMs.
struct Watchdog {
  // First thing at boot, every boot. False if it isn't running afterwards.
  // Called before the console exists, so the caller logs the result.
  static bool start();
  // Restarts the countdown: once per loop pass, and around anything during
  // boot that can block for over a second.
  static void feed();
};

}  // namespace gnimu
