// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "../protocol/layout_v1.h"

namespace gnimu {
namespace obd2 {

namespace frame = gnimu_frame::v1;

// How a Mode 01 PID's reply becomes a frame field's value, as SAE J1979
// defines it: value = raw * scale + offset, where raw is one data byte, or two
// (A * 256 + B), starting at data byte `start` (0 for A). A field that's the
// difference of two PIDs (boost: manifold minus barometric pressure) names
// the second as `minusPid`, polled slowly alongside (kHelperDecoders). These
// are fixed by the standard, the same on every car; which PIDs are polled,
// and how often, is Config.h's kObdPolls.
struct PidDecoder {
  frame::Field field;  // the frame field it fills
  uint8_t pid;
  uint8_t start;       // first data byte used: 0 = A
  uint8_t bytes;       // 1 or 2
  float scale;
  float offset;
  const char* name;    // for the console
  const char* unit;
  uint8_t minusPid;    // 0, or a helper PID whose value is subtracted
};

// Every frame field with a Mode 01 equivalent.
constexpr PidDecoder kPidDecoders[] = {
    {frame::Field::EngineSpeed, 0x0C, 0, 2, 0.25f, 0.0f, "RPM", "rpm", 0},
    {frame::Field::ThrottlePosition, 0x11, 0, 1, 100.0f / 255.0f, 0.0f, "Throttle", "%", 0},
    {frame::Field::AcceleratorPedal, 0x49, 0, 1, 100.0f / 255.0f, 0.0f, "Pedal", "%", 0},
    {frame::Field::VehicleSpeed, 0x0D, 0, 1, 1.0f, 0.0f, "Speed", "km/h", 0},
    {frame::Field::CoolantTemp, 0x05, 0, 1, 1.0f, -40.0f, "Coolant", "°C", 0},
    {frame::Field::IntakeAirTemp, 0x0F, 0, 1, 1.0f, -40.0f, "Intake air", "°C", 0},
    {frame::Field::AmbientTemp, 0x46, 0, 1, 1.0f, -40.0f, "Ambient", "°C", 0},
    {frame::Field::OilTemp, 0x5C, 0, 1, 1.0f, -40.0f, "Oil temp", "°C", 0},
    // Byte A says which sensors exist; B is charge air cooler bank 1 sensor 1.
    {frame::Field::ChargeAirCoolerTemp, 0x77, 1, 1, 1.0f, -40.0f, "CAC temp", "°C", 0},
    {frame::Field::FuelLevel, 0x2F, 0, 1, 100.0f / 255.0f, 0.0f, "Fuel", "%", 0},
    // Manifold absolute pressure minus barometric pressure. PID 0x0B tops out
    // at 255 kPa absolute.
    {frame::Field::Boost, 0x0B, 0, 1, 1.0f, 0.0f, "Boost", "kPa", 0x33},
};
constexpr size_t kPidDecoderCount = sizeof(kPidDecoders) / sizeof(kPidDecoders[0]);

// PIDs polled only to be subtracted from another (minusPid), with no frame
// field of their own. Their `field` is unused.
constexpr PidDecoder kHelperDecoders[] = {
    {frame::Field::Boost, 0x33, 0, 1, 1.0f, 0.0f, "Barometric", "kPa", 0},
};
constexpr size_t kHelperDecoderCount = sizeof(kHelperDecoders) / sizeof(kHelperDecoders[0]);

// The helper decoder for `pid`, or nullptr.
inline const PidDecoder* helperFor(uint8_t pid) {
  for (size_t i = 0; i < kHelperDecoderCount; i++)
    if (kHelperDecoders[i].pid == pid) return &kHelperDecoders[i];
  return nullptr;
}

// The decoder for `field`, or nullptr when it has none.
inline const PidDecoder* decoderFor(frame::Field field) {
  for (size_t i = 0; i < kPidDecoderCount; i++)
    if (kPidDecoders[i].field == field) return &kPidDecoders[i];
  return nullptr;
}

// Compile-time form, for Config.h's checks.
constexpr bool hasDecoder(frame::Field field, size_t i = 0) {
  return i < kPidDecoderCount && (kPidDecoders[i].field == field || hasDecoder(field, i + 1));
}

// Data bytes a reply needs to decode.
inline uint8_t bytesNeeded(const PidDecoder& d) { return static_cast<uint8_t>(d.start + d.bytes); }

// The value from a reply's data bytes A..D (at least bytesNeeded()).
inline float decode(const PidDecoder& d, const uint8_t* data) {
  const uint32_t raw = d.bytes == 2 ? (static_cast<uint32_t>(data[d.start]) << 8) | data[d.start + 1] : data[d.start];
  return static_cast<float>(raw) * d.scale + d.offset;
}

}  // namespace obd2
}  // namespace gnimu
