// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

// ============================================================================
// TOOL: Fake ECU
//
// A bench stand-in for a car's engine ECU, on a second XIAO ESP32C6 and CAN Pal
// wired like the Gnimu-CAN carrier (D9 TX, D10 RX, D8 to the CAN Pal's S).
// It answers OBD-II Mode 01 requests on 0x7DF (functional) and 0x7E0
// (physical) from 0x7E8, with values that change over time:
//
//   PID 0x00  supported PIDs 01-20 (05, 0C, 0D, 0F, 11)
//   PID 0x05  coolant temperature, 90 °C
//   PID 0x0C  RPM, sweeping 800-6500 and back over 20 s
//   PID 0x0D  vehicle speed, 0-120 km/h with the RPM sweep
//   PID 0x0F  intake air temperature, 30 °C
//   PID 0x11  throttle, a 0-100 % sawtooth every 5 s
//   PID 0x20  supported PIDs 21-40 (46)
//   PID 0x46  ambient air temperature, 20 °C
//
// Any other PID gets no answer, as on a real car. In "broadcast" mode it also
// sends a frame on 0x100 every 10 ms, the kind of chatter an ungatewayed bus
// carries, so Gnimu-CAN's listen-only rate check has something to hear. In
// "gateway" mode it stays silent until asked, like an OBD port behind a
// gateway.
//
// Console commands (end with Enter):
//   GATEWAY       silent until asked (Gnimu-CAN must use its gateway probe)
//   BROADCAST     chatter on 0x100 (Gnimu-CAN finds the rate by listening)
//   RATE 500      or RATE 250: the bus bit rate, kbit/s
//   DELAY n       wait n ms (0-100) before each reply
//   OFF / ON      stop answering entirely (the car switched off), or resume
//   S             print the status line now
//
// Target: Seeed XIAO ESP32C6, serial console at 115200. Leave the CAN Pal's
// termination switch ON on the bench (one 120 ohm end each side), and OFF
// before either board goes near a car.
// Build and flash: scripts/tool.sh fake_ecu --flash --monitor --port <port>
// ============================================================================

#include <Arduino.h>
#include <driver/twai.h>
#include <math.h>

#if !defined(CONFIG_IDF_TARGET_ESP32C6)
#error "fake_ecu supports the ESP32-C6 only"
#endif

namespace {

constexpr gpio_num_t kTxPin = GPIO_NUM_20;  // D9
constexpr gpio_num_t kRxPin = GPIO_NUM_18;  // D10
constexpr uint8_t kSilentPin = 19;          // D8: CAN Pal S; low = transmit
constexpr uint8_t kLedPin = LED_BUILTIN;    // GPIO15, active low

constexpr uint32_t kFunctionalId = 0x7DF;
constexpr uint32_t kPhysicalId = 0x7E0;
constexpr uint32_t kReplyId = 0x7E8;
constexpr uint32_t kChatterId = 0x100;
constexpr uint32_t kChatterPeriodMs = 10;
constexpr uint32_t kStatusPeriodMs = 1000;

uint32_t bitrate = 500000;
bool gatewayMode = false;
bool answering = true;
uint32_t replyDelayMs = 2;

uint32_t requests = 0;
uint32_t replies = 0;
uint32_t unsupported = 0;
uint32_t chatterSent = 0;
uint32_t ledOffAtMs = 0;
String line;

bool startCan() {
  twai_stop();
  twai_driver_uninstall();
  const twai_timing_config_t t500 = TWAI_TIMING_CONFIG_500KBITS();
  const twai_timing_config_t t250 = TWAI_TIMING_CONFIG_250KBITS();
  const twai_timing_config_t timing = bitrate == 250000 ? t250 : t500;
  // No-ACK mode: our frames complete even when nothing acknowledges them. On a
  // two-node bench, Gnimu-CAN listening silently can't acknowledge, and a
  // normal-mode sender would abort every frame with an error flag and retry,
  // so the listener would never receive one. On a car, other ECUs acknowledge.
  // This node still acknowledges frames it receives.
  twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(kTxPin, kRxPin, TWAI_MODE_NO_ACK);
  general.rx_queue_len = 32;
  general.tx_queue_len = 8;
  const twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&general, &timing, &filter) != ESP_OK) return false;
  return twai_start() == ESP_OK;
}

// The values a running engine might report, changing over time.
float rpmNow(uint32_t ms) {
  const float phase = (ms % 20000) / 20000.0f;  // 0..1 over 20 s
  const float tri = phase < 0.5f ? phase * 2.0f : 2.0f - phase * 2.0f;
  return 800.0f + tri * (6500.0f - 800.0f);
}

float throttleNow(uint32_t ms) { return (ms % 5000) / 50.0f; }  // 0..100 % every 5 s

// Mode 01 data bytes (A, B, ...) for `pid`, or 0 if unsupported.
uint8_t pidData(uint8_t pid, uint32_t ms, uint8_t (&d)[4]) {
  switch (pid) {
    case 0x00:  // supported 01-20: 05, 0C, 0D, 0F, 11 and 20
      // Bit 7 of A is PID 01, bit 0 of D is PID 20.
      d[0] = 0x08;  // 05
      d[1] = 0x1A;  // 0C, 0D, 0F
      d[2] = 0x80;  // 11
      d[3] = 0x01;  // 20: more support data follows
      return 4;
    case 0x20:  // supported 21-40: none here, 41-60 next
      d[0] = 0x00;
      d[1] = 0x00;
      d[2] = 0x00;
      d[3] = 0x01;  // 40
      return 4;
    case 0x40:  // supported 41-60: 46
      d[0] = 0x04;
      d[1] = 0x00;
      d[2] = 0x00;
      d[3] = 0x00;
      return 4;
    case 0x05:
      d[0] = 90 + 40;
      return 1;
    case 0x0C: {
      const uint16_t raw = static_cast<uint16_t>(rpmNow(ms) * 4.0f);
      d[0] = raw >> 8;
      d[1] = raw & 0xFF;
      return 2;
    }
    case 0x0D:
      d[0] = static_cast<uint8_t>((rpmNow(ms) - 800.0f) / (6500.0f - 800.0f) * 120.0f);
      return 1;
    case 0x0F:
      d[0] = 30 + 40;
      return 1;
    case 0x11:
      d[0] = static_cast<uint8_t>(throttleNow(ms) * 255.0f / 100.0f);
      return 1;
    case 0x46:
      d[0] = 20 + 40;
      return 1;
    default:
      return 0;
  }
}

void reply(uint8_t pid) {
  uint8_t d[4] = {};
  const uint8_t n = pidData(pid, millis(), d);
  if (n == 0) {
    unsupported++;
    return;
  }
  if (replyDelayMs) delay(replyDelayMs);
  twai_message_t m = {};
  m.identifier = kReplyId;
  m.data_length_code = 8;
  m.data[0] = static_cast<uint8_t>(2 + n);  // ISO-TP single frame: mode, PID, data
  m.data[1] = 0x41;
  m.data[2] = pid;
  for (uint8_t i = 0; i < 8 - 3; i++) m.data[3 + i] = i < n ? d[i] : 0x55;
  if (twai_transmit(&m, pdMS_TO_TICKS(10)) == ESP_OK) {
    replies++;
    digitalWrite(kLedPin, LOW);
    ledOffAtMs = millis() + 20;
  }
}

void handleFrame(const twai_message_t& m) {
  if (m.extd || (m.identifier != kFunctionalId && m.identifier != kPhysicalId)) return;
  if (m.data_length_code < 3 || m.data[0] != 0x02 || m.data[1] != 0x01) return;
  requests++;
  if (answering) reply(m.data[2]);
}

void sendChatter() {
  twai_message_t m = {};
  m.identifier = kChatterId;
  m.data_length_code = 8;
  const uint32_t ms = millis();
  memcpy(m.data, &ms, sizeof(ms));
  if (twai_transmit(&m, 0) == ESP_OK) chatterSent++;
}

void printStatus() {
  twai_status_info_t s = {};
  twai_get_status_info(&s);
  Serial.printf("up %lus | %s, %lu kbit/s, %s, delay %lu ms | requests %lu, replies %lu, unsupported %lu, chatter %lu | "
                "TEC %lu REC %lu%s\n",
                millis() / 1000, gatewayMode ? "gateway" : "broadcast", bitrate / 1000, answering ? "answering" : "OFF",
                replyDelayMs, requests, replies, unsupported, chatterSent, s.tx_error_counter, s.rx_error_counter,
                s.state == TWAI_STATE_BUS_OFF ? " BUS OFF" : "");
}

void handleLine(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  if (cmd == "GATEWAY") {
    gatewayMode = true;
  } else if (cmd == "BROADCAST") {
    gatewayMode = false;
  } else if (cmd == "OFF") {
    answering = false;
  } else if (cmd == "ON") {
    answering = true;
  } else if (cmd == "RATE 500" || cmd == "RATE 250") {
    bitrate = cmd.endsWith("500") ? 500000 : 250000;
    Serial.printf("CAN: %s\n", startCan() ? "restarted" : "failed to restart");
  } else if (cmd.startsWith("DELAY ")) {
    const long v = cmd.substring(6).toInt();
    if (v >= 0 && v <= 100) replyDelayMs = static_cast<uint32_t>(v);
  } else if (cmd != "S") {
    Serial.printf("Unknown '%s': GATEWAY, BROADCAST, RATE 500|250, DELAY n, OFF, ON, S\n", cmd.c_str());
    return;
  }
  printStatus();
}

}  // namespace

void setup() {
  pinMode(kLedPin, OUTPUT);
  digitalWrite(kLedPin, HIGH);
  // This board always transmits: hold the CAN Pal out of silent mode.
  pinMode(kSilentPin, OUTPUT);
  digitalWrite(kSilentPin, LOW);

  // Never block on a USB host that isn't reading (see Gnimu-CAN's
  // docs/architecture.md, Console).
  Serial.setTxTimeoutMs(0);
  Serial.begin(115200);
  const uint32_t start = millis();
  while (!Serial && millis() - start < 1000) delay(10);

  Serial.println();
  Serial.println("Gnimu-CAN fake ECU");
  Serial.printf("CAN: %s at %lu kbit/s\n", startCan() ? "started" : "FAILED", bitrate / 1000);
  Serial.println("Commands: GATEWAY, BROADCAST, RATE 500|250, DELAY n, OFF, ON, S");
  printStatus();
}

void loop() {
  static uint32_t lastChatter = 0;
  static uint32_t lastStatus = 0;
  const uint32_t now = millis();

  twai_message_t m;
  while (twai_receive(&m, 0) == ESP_OK) handleFrame(m);

  if (!gatewayMode && answering && now - lastChatter >= kChatterPeriodMs) {
    lastChatter = now;
    sendChatter();
  }
  if (ledOffAtMs && now >= ledOffAtMs) {
    digitalWrite(kLedPin, HIGH);
    ledOffAtMs = 0;
  }
  if (now - lastStatus >= kStatusPeriodMs * 5) {
    lastStatus = now;
    printStatus();
  }

  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r' || c == '\n') {
      if (line.length()) handleLine(line);
      line = "";
    } else if (line.length() < 32) {
      line += c;
    }
  }
  delay(1);
}
