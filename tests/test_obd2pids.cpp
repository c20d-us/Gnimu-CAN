// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

// The Mode 01 decoders, against example replies worked from SAE J1979's
// formulas.

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "../code/GnimuCAN/src/core/Obd2Pids.h"

using namespace gnimu;
using F = gnimu_frame::v1::Field;

static float decodeField(F field, uint8_t a, uint8_t b = 0, uint8_t c = 0) {
  const obd2::PidDecoder* d = obd2::decoderFor(field);
  assert(d != nullptr);
  const uint8_t data[4] = {a, b, c, 0};
  return obd2::decode(*d, data);
}

static bool near(float a, float b) { return fabsf(a - b) < 0.01f; }

static void testExamples() {
  assert(near(decodeField(F::EngineSpeed, 0x0C, 0x80), 800.0f));  // (12*256 + 128) / 4
  assert(near(decodeField(F::EngineSpeed, 0xFF, 0xFF), 16383.75f));
  assert(near(decodeField(F::ThrottlePosition, 0xFF), 100.0f));
  assert(near(decodeField(F::ThrottlePosition, 0), 0.0f));
  assert(near(decodeField(F::AcceleratorPedal, 0x80), 50.2f));
  assert(near(decodeField(F::VehicleSpeed, 120), 120.0f));
  assert(near(decodeField(F::CoolantTemp, 0), -40.0f));
  assert(near(decodeField(F::CoolantTemp, 130), 90.0f));
  assert(near(decodeField(F::IntakeAirTemp, 70), 30.0f));
  assert(near(decodeField(F::AmbientTemp, 60), 20.0f));
  assert(near(decodeField(F::OilTemp, 140), 100.0f));
  assert(near(decodeField(F::ChargeAirCoolerTemp, 0x01, 85), 45.0f));  // A: sensors present; B: the temperature
  assert(near(decodeField(F::FuelLevel, 0x80), 50.2f));
}

// Boost: manifold absolute pressure (0x0B) minus barometric pressure (0x33),
// read from a helper decoder.
static void testBoost() {
  const obd2::PidDecoder* boost = obd2::decoderFor(F::Boost);
  assert(boost && boost->pid == 0x0B && boost->minusPid == 0x33);
  const obd2::PidDecoder* baro = obd2::helperFor(0x33);
  assert(baro && baro->minusPid == 0);
  const uint8_t map[4] = {180, 0, 0, 0}, atmosphere[4] = {101, 0, 0, 0}, idle[4] = {35, 0, 0, 0};
  assert(near(obd2::decode(*boost, map) - obd2::decode(*baro, atmosphere), 79.0f));   // under boost
  assert(near(obd2::decode(*boost, idle) - obd2::decode(*baro, atmosphere), -66.0f));  // vacuum at idle
  // Every minusPid has a helper.
  for (size_t i = 0; i < obd2::kPidDecoderCount; i++)
    if (obd2::kPidDecoders[i].minusPid) assert(obd2::helperFor(obd2::kPidDecoders[i].minusPid) != nullptr);
}

// Second choices: coolant from 0x67 (A: sensors present, B: sensor 1),
// filling the same field, with a name and unit to match.
static void testAlternates() {
  const obd2::PidDecoder* own = obd2::decoderFor(F::CoolantTemp);
  const obd2::PidDecoder* alt = obd2::altFor(F::CoolantTemp);
  assert(own && alt && alt->pid == 0x67 && alt->length == 3 && alt->start + alt->bytes == 2);
  assert(alt->field == own->field && strcmp(alt->name, own->name) == 0 && strcmp(alt->unit, own->unit) == 0);
  const uint8_t reply[4] = {0x01, 0x7B, 0x00, 0};
  assert(near(obd2::decode(*alt, reply), 83.0f));
  assert(obd2::altFor(F::EngineSpeed) == nullptr);
  // Each fills a field with a decoder of its own, by a different PID.
  for (size_t i = 0; i < obd2::kAltDecoderCount; i++) {
    const obd2::PidDecoder* d = obd2::decoderFor(obd2::kAltDecoders[i].field);
    assert(d && d->pid != obd2::kAltDecoders[i].pid && obd2::kAltDecoders[i].minusPid == 0);
  }
}

static void testTable() {
  for (size_t i = 0; i < obd2::kPidDecoderCount; i++) {
    const obd2::PidDecoder& d = obd2::kPidDecoders[i];
    assert(d.bytes == 1 || d.bytes == 2);
    assert(d.start + d.bytes <= d.length && d.start + d.bytes <= 4);  // within the reply, and A..D
    assert(d.pid != 0x00 && d.pid != 0x20 && d.pid != 0x40 && d.pid != 0x60);  // not a bitmap
    for (size_t j = i + 1; j < obd2::kPidDecoderCount; j++) {
      assert(obd2::kPidDecoders[j].field != d.field);
      assert(obd2::kPidDecoders[j].pid != d.pid);
    }
    assert(obd2::hasDecoder(d.field));
  }
  assert(!obd2::hasDecoder(F::OilPressure) && obd2::decoderFor(F::Gear) == nullptr);
}

int main() {
  testExamples();
  testBoost();
  testAlternates();
  testTable();
  printf("test_obd2pids: ok\n");
  return 0;
}
