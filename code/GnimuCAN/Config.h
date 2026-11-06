// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Gnimu-CAN settings: the values chosen for a build. Each one is checked when
// the firmware compiles (src/app/SystemConfig.h), so a value out of range
// fails the build with a message saying why. The constants the firmware's
// timing and safety depend on are in src/app/SystemConfig.h, not here.

#include <stdint.h>

namespace gnimu {
namespace config {

// BLE

// Gnimu frames sent per second while a client is subscribed. 20 to start; up
// to about 30 is worthwhile, a little above the GNSS rate of the app's own
// position source (docs/architecture.md, Emit rate).
constexpr uint32_t kEmitRateHz = 20;
// Advertised name is "<prefix> <10-digit device ID>"; the prefix is also the
// Device Information model string and the schema's product name.
constexpr char kBleNamePrefix[] = "Gnimu-CAN";
// Device Information manufacturer string.
constexpr char kBleManufacturer[] = "Gnimu";

// Supply voltage

// The ADC reads slightly off; this corrects it. Measure the supply with a
// meter or a bench supply's display, read the Supply value from the D report,
// and set this to true / reported. 1.0 = no correction. The first bench board
// read about 1% low.
constexpr float kSupplyCalibration = 1.01f;
// At or above: the alternator is charging, so the engine is running.
constexpr float kEngineRunningV = 13.2f;
// Below: protect the car's battery.
constexpr float kLowVoltageCutoffV = 11.8f;

// CAN

// The gateway probe (docs/architecture.md, Bring-up): with the bus silent and
// the supply at or above this, the car may be on behind a gateway, so the
// device sends one single-shot OBD-II request to see. Below the 13.2V
// engine-running threshold on purpose: cars with intelligent alternator
// control can run their engine at 12.4-12.8V.
constexpr float kGatewayProbeMinV = 12.4f;
// The probe's bit rate, which nearly every gatewayed car uses.
constexpr uint32_t kGatewayProbeBitrate = 500000;

// OBD-II polling

// How often each channel is asked for. The car's reply rate is shared between
// the channels, so these are targets: the D report and status line show the
// rate each one actually gets.
constexpr uint32_t kRpmPeriodMs = 40;       // 25 Hz
constexpr uint32_t kThrottlePeriodMs = 40;  // 25 Hz
// At or above this, with fresh RPM, the engine is running whatever the supply
// voltage says.
constexpr float kEngineRunningMinRpm = 400.0f;

}  // namespace config
}  // namespace gnimu
