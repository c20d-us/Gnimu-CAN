// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// System constants: the values the firmware's timing, safety and hardware
// handling depend on, as opposed to the settings chosen for a build. Those
// are in Config.h next to GnimuCAN.ino, included here first so code includes
// only this file. The checks at the end cover both.

#include <stddef.h>
#include <stdint.h>

#include "../../Config.h"

namespace gnimu {
namespace config {

// Scheduler jobs: how often each runs, and the time budget past which a run
// counts as an overrun in the D report.

// CAN: drains the controller's receive queue, checks the bit rate, and makes
// gateway probes. A probe blocks the job for up to about 70 ms; it happens at
// most 5 times per boot.
constexpr uint32_t kCanPeriodUs = 10000;
constexpr uint32_t kCanBudgetUs = 2000;
// The frame: one Gnimu frame per period while a client is subscribed.
constexpr uint32_t kEmitPeriodUs = 1000000 / kEmitRateHz;
constexpr uint32_t kEmitBudgetUs = 2000;
// The supply voltage: one ADC reading per period, filtered.
constexpr uint32_t kPowerPeriodUs = 100000;
constexpr uint32_t kPowerBudgetUs = 1000;
// BLE housekeeping: connection-edge logging.
constexpr uint32_t kBlePeriodUs = 10000;
constexpr uint32_t kBleBudgetUs = 1000;
constexpr uint32_t kLedPeriodUs = 25000;
constexpr uint32_t kLedBudgetUs = 500;
constexpr uint32_t kConsolePeriodUs = 10000;
constexpr uint32_t kConsoleBudgetUs = 2000;

// Hardware watchdog: fed every loop pass and between the boot steps.
constexpr uint32_t kWatchdogMs = 5000;

// Console

// How long boot waits for a USB host before carrying on without one.
constexpr uint32_t kConsoleHostWaitMs = 1000;
// Longest one console line waits for room in the output buffer before it's
// dropped, so a host that holds the port open without reading costs the loop
// little.
constexpr uint32_t kConsoleWaitUs = 500;
// Periodic status line cadence ('L' or a typed count).
constexpr uint64_t kStatusPeriodUs = 1000000;
// Largest status-line count accepted from the console (3600 = an hour of 1 s
// lines), and the length of an L run, so status reporting always ends by
// itself.
constexpr uint32_t kStatusMaxCount = 3600;

// CAN bring-up

// Boot listens this long at each bit rate before deciding the bus is silent.
constexpr uint32_t kCanBootListenMs = 300;
// While waiting for the vehicle, it keeps listening, switching between the
// bit rates this often until it hears clean traffic.
constexpr uint32_t kCanRateWindowMs = 2000;
// Gateway probe: the supply must stay at or above kGatewayProbeMinV this long
// first; then at most kProbeMaxAttempts tries per boot, kProbeBackoffMs apart,
// each waiting kProbeReplyMs (the ISO 15765-4 P2 limit) for an ECU's reply.
constexpr uint32_t kProbeHoldMs = 3000;
constexpr uint32_t kProbeBackoffMs = 30000;
constexpr uint8_t kProbeMaxAttempts = 5;
constexpr uint32_t kProbeReplyMs = 50;
// How long the controller gets to report the probe frame sent or failed: a
// single 8-byte frame takes well under 1 ms at 250 kbit/s.
constexpr uint32_t kProbeTxConfirmMs = 20;

// OBD-II polling

// How long one request waits for its reply: the ISO 15765-4 P2 limit.
constexpr uint32_t kObdReplyTimeoutMs = 50;
// No reply to any channel for this long: the car has gone quiet (switched
// off, most likely), and the device goes back to waiting for it.
constexpr uint32_t kVehicleLostMs = 2000;
// A channel's value goes into the frame only while it's at most this old.
constexpr uint32_t kChannelStaleMs = 1000;
// With traffic on the bus but no ECU answering, how long before asking again,
// and how many unanswered tries before going back to checking the bit rate.
constexpr uint32_t kDiscoveryRetryMs = 5000;
constexpr uint8_t kDiscoveryMaxTries = 3;

// Supply voltage

// EMA smoothing per reading at 10 Hz: about 1 s to settle, which hides ADC
// noise (about +/-0.1 V of supply on the bench) without hiding a real change.
constexpr float kSupplyEmaAlpha = 0.2f;
// How long the supply must stay above the engine-running threshold (or back
// below it) before the level changes: long enough to ignore cranking dips.
constexpr uint32_t kSupplyRunningHoldMs = 3000;
// How long it must stay below the cutoff before the cutoff counts.
constexpr uint32_t kSupplyCutoffHoldMs = 5000;

// Status LED patterns.
constexpr uint32_t kLedBlinkHalfMs = 1000;    // polling, no client: 1 s on, 1 s off
constexpr uint32_t kLedFaultHalfMs = 100;     // fault: 5 Hz
constexpr uint32_t kLedFlashOnMs = 100;       // waiting for the vehicle: a 100 ms flash ...
constexpr uint32_t kLedFlashPeriodMs = 2000;  // ... every 2 s

// Checks: the settings in Config.h first, then the system constants.

static_assert(kEmitRateHz >= 1 && kEmitRateHz <= 50, "emit rate must be 1-50 Hz");
static_assert(sizeof(kBleNamePrefix) >= 2 && sizeof(kBleNamePrefix) <= 20,
              "BLE name prefix must be 1-19 characters so the advertised name fits");
static_assert(sizeof(kBleManufacturer) >= 2, "BLE manufacturer string must not be empty");
static_assert(kSupplyCalibration > 0.9f && kSupplyCalibration < 1.1f,
              "supply calibration should be within 10% of 1.0");
static_assert(kLowVoltageCutoffV > 9.0f && kLowVoltageCutoffV < kEngineRunningV &&
                  kEngineRunningV < 15.0f,
              "voltage thresholds must be ordered: 9 V < cutoff < engine running < 15 V");
static_assert(kGatewayProbeMinV > kLowVoltageCutoffV && kGatewayProbeMinV <= kEngineRunningV,
              "the gateway probe threshold must sit between the cutoff and engine running");
static_assert(kGatewayProbeBitrate == 500000 || kGatewayProbeBitrate == 250000,
              "the gateway probe runs at 500 or 250 kbit/s");
static_assert(kRpmPeriodMs >= 10 && kThrottlePeriodMs >= 10, "poll periods must be at least 10 ms");
static_assert(kChannelStaleMs > kRpmPeriodMs && kChannelStaleMs > kThrottlePeriodMs,
              "a channel must stay fresh for longer than its poll period");
static_assert(kVehicleLostMs > kObdReplyTimeoutMs * 4, "vehicle-lost window too short to be meaningful");
static_assert(kSupplyEmaAlpha > 0.0f && kSupplyEmaAlpha <= 1.0f, "alpha must be in (0, 1]");
static_assert(kStatusPeriodUs > 0, "status period must be positive");
static_assert(kLedPeriodUs / 1000 <= kLedFaultHalfMs,
              "LED job must run at least once per fault half-period or the blink aliases");
static_assert(kLedPeriodUs / 1000 <= kLedFlashOnMs, "LED job must run at least once per flash");
static_assert(kLedFlashOnMs < kLedFlashPeriodMs, "the flash must be shorter than its period");

}  // namespace config
}  // namespace gnimu
