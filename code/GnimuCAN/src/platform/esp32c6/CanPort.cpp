// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

// Two-layer guard: ARDUINO_ARCH_ESP32 is a compiler flag, safe before any
// include; CONFIG_IDF_TARGET_ESP32C6 only exists once sdkconfig.h is included.
#if defined(ARDUINO_ARCH_ESP32)
#include "sdkconfig.h"
#if CONFIG_IDF_TARGET_ESP32C6

#include <Arduino.h>
#include <driver/twai.h>
#include <string.h>

#include "../CanPort.h"
#include "BoardConfig.h"

namespace gnimu {

namespace {

bool running = false;
bool listenOnlyMode = true;
uint32_t runningBitrate = 0;
uint32_t busErrorsAtStart = 0;
uint32_t rxMissedAtStart = 0;

// The transceiver's S pin. Released (input), the carrier's pull-up holds it
// high: silent. Driven low: the transceiver can transmit.
void setSilent(bool silent) {
  if (silent) {
    pinMode(board::kCanSilentPin, INPUT);
  } else {
    pinMode(board::kCanSilentPin, OUTPUT);
    digitalWrite(board::kCanSilentPin, LOW);
  }
}

bool timingFor(uint32_t bitrate, twai_timing_config_t& out) {
  switch (bitrate) {
    case 500000: {
      const twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
      out = t;
      return true;
    }
    case 250000: {
      const twai_timing_config_t t = TWAI_TIMING_CONFIG_250KBITS();
      out = t;
      return true;
    }
    default:
      return false;
  }
}

twai_status_info_t driverStatus() {
  twai_status_info_t s = {};
  if (running) twai_get_status_info(&s);
  return s;
}

}  // namespace

void CanPort::init() { setSilent(true); }

bool CanPort::start(uint32_t bitrate, bool listenOnly) {
  stop();
  twai_timing_config_t timing;
  if (!timingFor(bitrate, timing)) return false;
  setSilent(listenOnly);
  twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(
      static_cast<gpio_num_t>(board::kCanTxPin), static_cast<gpio_num_t>(board::kCanRxPin),
      listenOnly ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL);
  general.rx_queue_len = 32;
  general.tx_queue_len = 1;
  general.alerts_enabled = TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_BUS_OFF;
  const twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&general, &timing, &filter) != ESP_OK) {
    setSilent(true);
    return false;
  }
  if (twai_start() != ESP_OK) {
    twai_driver_uninstall();
    setSilent(true);
    return false;
  }
  running = true;
  listenOnlyMode = listenOnly;
  runningBitrate = bitrate;
  const twai_status_info_t s = driverStatus();
  busErrorsAtStart = s.bus_error_count;
  rxMissedAtStart = s.rx_missed_count;
  return true;
}

void CanPort::stop() {
  if (running) {
    twai_stop();
    twai_driver_uninstall();
    running = false;
    runningBitrate = 0;
  }
  setSilent(true);
}

bool CanPort::send(const CanFrame& f) {
  if (!running || listenOnlyMode) return false;
  twai_message_t m = {};
  m.identifier = f.id;
  m.extd = f.extended;
  m.data_length_code = f.len;
  memcpy(m.data, f.data, f.len);
  return twai_transmit(&m, 0) == ESP_OK;
}

bool CanPort::receive(CanFrame& f) {
  if (!running) return false;
  twai_message_t m;
  if (twai_receive(&m, 0) != ESP_OK) return false;
  f.id = m.identifier;
  f.extended = m.extd;
  f.len = m.data_length_code > 8 ? 8 : m.data_length_code;
  memcpy(f.data, m.data, f.len);
  return true;
}

bool CanPort::sendSingleShot(const CanFrame& f) {
  if (!running || listenOnlyMode) return false;
  // Clear any alerts left from earlier frames, so txOutcome() reports this one.
  uint32_t stale = 0;
  twai_read_alerts(&stale, 0);
  twai_message_t m = {};
  m.identifier = f.id;
  m.extd = f.extended;
  m.ss = 1;
  m.data_length_code = f.len;
  memcpy(m.data, f.data, f.len);
  return twai_transmit(&m, 0) == ESP_OK;
}

TxOutcome CanPort::txOutcome() {
  if (!running) return TxOutcome::Failed;
  uint32_t alerts = 0;
  if (twai_read_alerts(&alerts, 0) != ESP_OK) return TxOutcome::Pending;
  if (alerts & TWAI_ALERT_TX_SUCCESS) return TxOutcome::Sent;
  if (alerts & (TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_OFF)) return TxOutcome::Failed;
  return TxOutcome::Pending;
}

CanStatus CanPort::status() {
  const twai_status_info_t s = driverStatus();
  CanStatus c = {};
  c.running = running;
  c.listenOnly = listenOnlyMode;
  c.busOff = running && s.state == TWAI_STATE_BUS_OFF;
  c.bitrate = runningBitrate;
  c.txErrors = s.tx_error_counter;
  c.rxErrors = s.rx_error_counter;
  c.busErrors = s.bus_error_count - busErrorsAtStart;
  c.rxMissed = s.rx_missed_count - rxMissedAtStart;
  return c;
}

}  // namespace gnimu

#endif  // CONFIG_IDF_TARGET_ESP32C6
#endif  // ARDUINO_ARCH_ESP32
