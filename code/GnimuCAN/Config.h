// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Gnimu-CAN settings: the values chosen for a build. Each one is checked when
// the firmware compiles (src/app/SystemConfig.h), so a value out of range
// fails the build with a message saying why. The constants the firmware's
// timing and safety depend on are in src/app/SystemConfig.h, not here.

#include <stdint.h>

#include "src/protocol/layout_v1.h"

namespace gnimu {
namespace config {

// BLE

// Gnimu frames sent per second while a client is subscribed. The target is
// 30-50, above the GNSS rate of the app's own position source
// (docs/architecture.md, Emit rate).
constexpr uint32_t kEmitRateHz = 30;
// Advertised name is "<prefix> <10-digit device ID>"; the prefix is also the
// Device Information model string and the schema's product name.
constexpr char kBleNamePrefix[] = "Gnimu-CAN";
// Device Information manufacturer string.
constexpr char kBleManufacturer[] = "Gnimu Labs";

// Supply voltage

// The ADC and the divider read slightly off; this corrects it. Measure the
// supply with a meter at the board's 12V input, read the Supply value from
// the D report, and multiply this by meter / reported. 1.0 = no correction.
// The breadboard read about 1% low; the first 12V carrier v1.0 board about
// 2.8% low before correction (a best fit to 12.1, 12.7 and 14.3V metered,
// within 0.04V at each).
constexpr float kSupplyCalibration = 1.028f;
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

// Sleep (docs/architecture.md, Sleep and the cutoff). Placeholders until the
// parked current and the car's voltage after shutdown are measured on the
// real board.

// Parked (no fresh RPM, supply below engine running, bus silent) for this
// long: sleep. A connected client doesn't keep the device awake.
constexpr uint32_t kSleepAfterParkedS = 300;
// How often a sleeping device wakes to check whether the car has started.
// Shorter catches a start sooner; each check costs about half a second awake.
constexpr uint32_t kParkedCheckIntervalS = 30;
// The same, after the low-voltage cutoff: rarely, to protect the battery.
constexpr uint32_t kCutoffCheckIntervalS = 600;
// After a cutoff, the supply must rise above this to count as recovered, so
// a battery hovering at the cutoff can't flip the device in and out of it.
constexpr float kCutoffRecoverV = 12.1f;
// A supply this much above the lowest wake-check reading since sleeping (the
// first check sets it) means the alternator has started charging: wake fully. Catches a gatewayed car whose
// engine runs below the engine-running level (intelligent alternator control).
constexpr float kWakeRiseV = 0.3f;
// A wake check reads the supply this much higher than the awake device does
// (it runs with BLE and CAN off, so the board draws less): the check's
// engine-running, cutoff and recovery levels are raised by it, so they mean
// the same voltages as when awake. Per board: with the supply unchanged, it's
// "checks read" minus "slept at" in the D report's wake line. The breadboard
// read 0.05-0.15 V; measure the real board.
constexpr float kWakeCheckOffsetV = 0.10f;

// OBD-II polling

// The Mode 01 channels: which to poll and how often. A period of 0 turns a
// channel off. Periods are targets: the car's reply rate is shared between
// the channels, and the D report and status line show the rate each one
// actually gets. Order matters: ECU discovery prefers the ECU supporting the
// first channels, and the status line shows the first four polled. How each
// PID decodes is fixed by the standard (src/core/Obd2Pids.h).
struct ObdPoll {
  gnimu_frame::v1::Field field;
  uint32_t periodMs;
};
using F = gnimu_frame::v1::Field;
constexpr ObdPoll kObdPolls[] = {
    {F::EngineSpeed, 40},            // PID 0C, 25 Hz
    {F::ThrottlePosition, 50},       // PID 11, 20 Hz
    {F::AcceleratorPedal, 100},      // PID 49, 10 Hz
    {F::VehicleSpeed, 100},          // PID 0D, 10 Hz
    {F::Boost, 100},                 // PID 0B minus 33 (read every 5 s), 10 Hz
    {F::CoolantTemp, 1000},          // PID 05, 1 Hz
    {F::IntakeAirTemp, 1000},        // PID 0F, 1 Hz
    {F::OilTemp, 1000},              // PID 5C, 1 Hz
    {F::ChargeAirCoolerTemp, 1000},  // PID 77, 1 Hz
    {F::AmbientTemp, 5000},          // PID 46, every 5 s
    {F::FuelLevel, 5000},            // PID 2F, every 5 s
};
// About 79 requests a second in all. The M2 answered 69 a second with
// replies in under 10 ms (docs/car-test.md), and kMaxObdRequestsPerSec caps
// the total. A car that can't keep up gives each channel a share in
// proportion to its period.
// At most this many requests a second across all channels, whatever the
// periods add up to, so the car's diagnostic bus never sees more.
constexpr uint32_t kMaxObdRequestsPerSec = 100;
// At or above this, with fresh RPM, the engine is running whatever the supply
// voltage says.
constexpr float kEngineRunningMinRpm = 400.0f;

}  // namespace config
}  // namespace gnimu
