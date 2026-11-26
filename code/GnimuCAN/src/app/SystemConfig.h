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

// CAN: drains the controller's receive queue, checks the bit rate, makes
// gateway probes and asks for an ECU. Nothing waits inside it: a request is
// sent, and later runs take the reply or time it out.
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

// While waiting for the vehicle, it listens, switching between the bit rates
// this often (or sooner, at the first bus error) until it hears clean
// traffic. Traffic at the rate it's listening at is confirmed by its first
// clean frame, so finding a live bus takes milliseconds, not a window.
constexpr uint32_t kCanRateWindowMs = 2000;
// Gateway probe: the supply must stay at or above kGatewayProbeMinV this long
// first; then one single-shot frame every kProbeIntervalMs while the bus stays
// silent, each waiting kProbeReplyMs for an ECU's reply. Longer than the 50 ms
// ISO 15765-4 P2 limit: a gateway passing the request on may answer later,
// and waiting costs nothing.
constexpr uint32_t kProbeHoldMs = 3000;
constexpr uint32_t kProbeIntervalMs = 10000;
constexpr uint32_t kProbeReplyMs = 1000;
// How long the controller gets to report a probe frame sent or failed: a
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
// With traffic on the bus but no ECU answering, how long before asking again:
// every 5 s for 3 tries, every 30 s for the next 3, then every 2 minutes
// (core/DiscoveryBackoff.h). Starts over when the bus goes quiet.
constexpr uint32_t kDiscoveryRetryMs = 5000;
constexpr uint32_t kDiscoveryBackoffMs = 30000;
constexpr uint32_t kDiscoveryMaxBackoffMs = 120000;
constexpr uint8_t kDiscoveryTriesPerStep = 3;

// Sleep

// No CAN frame for this long counts as a silent bus, one of the parked
// conditions.
constexpr uint32_t kBusSilentMs = 10000;
// A wake check: supply readings averaged, then a listen at each bit rate.
constexpr uint32_t kWakeSupplySamples = 32;
constexpr uint32_t kWakeListenMs = 150;
// Below this the regulator can't run the XIAO, so the device must be on USB
// with no 12 V connected: it never sleeps then (no battery to protect).
constexpr float kNoSupplyV = 6.0f;
// No sleep decisions this long after the supply arrives (at power-up, or a
// switch-on with USB already connected): the supply level settles and the
// car's voltage ramps first.
constexpr uint32_t kSleepGraceMs = 30000;

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
static_assert(kProbeReplyMs > kProbeTxConfirmMs && kProbeReplyMs < kProbeIntervalMs,
              "a probe waits longer for a reply than for the frame to go, and ends before the next is due");
static_assert(kRpmPeriodMs >= 10 && kThrottlePeriodMs >= 10, "poll periods must be at least 10 ms");
static_assert(kChannelStaleMs > kRpmPeriodMs && kChannelStaleMs > kThrottlePeriodMs,
              "a channel must stay fresh for longer than its poll period");
static_assert(kDiscoveryRetryMs <= kDiscoveryBackoffMs && kDiscoveryBackoffMs <= kDiscoveryMaxBackoffMs,
              "discovery backoff must not get shorter");
static_assert(kDiscoveryTriesPerStep >= 1, "at least one discovery try per backoff step");
static_assert(kVehicleLostMs > kObdReplyTimeoutMs * 4, "vehicle-lost window too short to be meaningful");
static_assert(kSleepAfterParkedS >= 60, "sleeping less than a minute after parking would sleep at traffic lights");
static_assert(kParkedCheckIntervalS >= 5 && kParkedCheckIntervalS <= 300,
              "parked check interval must be 5-300 s: shorter wastes power, longer misses starts");
static_assert(kCutoffCheckIntervalS >= kParkedCheckIntervalS, "the cutoff checks no more often than parked");
static_assert(kCutoffRecoverV > kLowVoltageCutoffV && kCutoffRecoverV < kEngineRunningV,
              "cutoff recovery must sit between the cutoff and engine running");
static_assert(kWakeCheckOffsetV >= 0.0f && kWakeCheckOffsetV <= 0.5f,
              "the wake-check offset should be 0-0.5 V; more means the reading itself needs fixing");
static_assert(kWakeRiseV >= 0.1f && kWakeRiseV <= 1.0f, "wake rise must be 0.1-1.0 V: below 0.1 is ADC noise");
static_assert(kBusSilentMs >= kVehicleLostMs, "the bus must be silent longer than the vehicle-lost window");
static_assert(kNoSupplyV > 1.0f && kNoSupplyV < kLowVoltageCutoffV - 2.0f,
              "the no-supply level must sit well below the cutoff and above ADC noise at 0 V");
static_assert(kWakeListenMs >= 50, "a wake listen shorter than 50 ms can miss slow traffic");
static_assert(kSupplyEmaAlpha > 0.0f && kSupplyEmaAlpha <= 1.0f, "alpha must be in (0, 1]");
static_assert(kStatusPeriodUs > 0, "status period must be positive");
static_assert(kLedPeriodUs / 1000 <= kLedFaultHalfMs,
              "LED job must run at least once per fault half-period or the blink aliases");
static_assert(kLedPeriodUs / 1000 <= kLedFlashOnMs, "LED job must run at least once per flash");
static_assert(kLedFlashOnMs < kLedFlashPeriodMs, "the flash must be shorter than its period");

}  // namespace config
}  // namespace gnimu
