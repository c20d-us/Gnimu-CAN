// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

// ============================================================================
// TOOL: Voltage log
//
// A car-side data logger for a complete Gnimu-CAN unit (or the breadboard
// device): once a second it appends one line to a log in flash, so the supply
// voltage and the OBD port's CAN activity can be recorded through a whole
// drive and afterwards, with no laptop attached:
//
//   t_s        seconds since this boot
//   v_avg      supply voltage, the mean of 100 readings over the second
//   v_min      lowest of those readings (cranking dips)
//   v_max      highest (charging spikes)
//   frames     CAN frames received in the second
//   bus_errors CAN bus errors in the second (non-zero: wrong bit rate, or a
//              wiring problem)
//
// Each boot starts a new section with a "# boot" comment line, so a brown-out
// while cranking shows up as a new section rather than lost data.
//
// CAN is listen-only, with the transceiver left silent (D8 released, so the
// carrier's pull-up R1 holds the CAN Pal's S pin high): the device never
// transmits and can't disturb the car's bus.
//
// The log lives in the flash data partition (LittleFS), about 1.5 MB: roughly
// 16 hours of samples. It survives power cycles and reflashing this sketch,
// and logging stops cleanly when it's full.
//
// Console commands (end with Enter):
//   STATUS       samples logged, space used, boots, CAN bit rate
//   DUMP         prints the whole log between "#BEGIN" and "#END" lines;
//                scripts/tool.sh voltage_log --dump <file> saves it to a file
//   RATE 500     or RATE 250: the CAN bit rate to listen at (default 500)
//   CLEAR LOG    erases the log
//   QUIET / LIVE stop or resume the live line printed each second
//
// LED: a short blink each second while logging; fast blinking if the log
// can't be written (flash failed or full).
//
// Target: Seeed XIAO ESP32C6, serial console at 115200.
// Build and flash: scripts/tool.sh voltage_log --flash --monitor
// ============================================================================

#include <Arduino.h>
#include <FS.h>
#include <LittleFS.h>
#include <driver/twai.h>
#include <esp_system.h>

#if !defined(CONFIG_IDF_TARGET_ESP32C6)
#error "voltage_log supports the ESP32-C6 only"
#endif

namespace {

// The 12V carrier's pins and divider, as in Gnimu-CAN's BoardConfig.h.
constexpr uint8_t kSensePin = 0;          // A0: supply through 100k + 10k
constexpr float kDivider = 11.0f;
constexpr float kCalibration = 1.01f;     // Config.h kSupplyCalibration
constexpr gpio_num_t kTxPin = GPIO_NUM_20;  // D9
constexpr gpio_num_t kRxPin = GPIO_NUM_18;  // D10
constexpr uint8_t kSilentPin = 19;        // D8: released, so R1 holds S high
constexpr uint8_t kLedPin = LED_BUILTIN;  // GPIO15, active low

constexpr uint32_t kSamplePeriodMs = 10;  // ADC readings, 100 per line
constexpr uint32_t kLinePeriodMs = 1000;
constexpr size_t kMinFreeBytes = 16 * 1024;  // stop logging below this
constexpr uint32_t kDumpStallMs = 2000;      // give up a dump if the host stops reading

const char* const kLogPath = "/log.csv";
const char* const kBootPath = "/boots.txt";

bool fsOk = false;
bool logFull = false;
bool live = true;
uint32_t bootNumber = 0;
uint32_t bitrate = 500000;
bool canOk = false;
File logFile;

// The current second's readings.
uint32_t sampleCount = 0;
uint64_t sumMv = 0;
uint32_t minMv = UINT32_MAX;
uint32_t maxMv = 0;
uint32_t framesThisLine = 0;
uint32_t busErrorsAtLineStart = 0;

uint32_t linesThisBoot = 0;
uint32_t nextSampleMs = 0;
uint32_t nextLineMs = 0;
uint32_t ledOffAtMs = 0;
String input;

float mvToVolts(uint32_t mv) { return mv / 1000.0f * kDivider * kCalibration; }

const char* resetReasonName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:
      return "power-on";
    case ESP_RST_BROWNOUT:
      return "brown-out";
    case ESP_RST_SW:
      return "software";
    case ESP_RST_PANIC:
      return "panic";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:
      return "watchdog";
    case ESP_RST_USB:
      return "usb";
    case ESP_RST_DEEPSLEEP:
      return "deep sleep";
    default:
      return "other";
  }
}

uint32_t busErrorCount() {
  twai_status_info_t s = {};
  if (canOk && twai_get_status_info(&s) == ESP_OK) return s.bus_error_count;
  return 0;
}

bool startCan() {
  twai_stop();
  twai_driver_uninstall();
  const twai_timing_config_t t500 = TWAI_TIMING_CONFIG_500KBITS();
  const twai_timing_config_t t250 = TWAI_TIMING_CONFIG_250KBITS();
  const twai_timing_config_t timing = bitrate == 250000 ? t250 : t500;
  twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(kTxPin, kRxPin, TWAI_MODE_LISTEN_ONLY);
  general.rx_queue_len = 128;
  const twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  canOk = twai_driver_install(&general, &timing, &filter) == ESP_OK && twai_start() == ESP_OK;
  busErrorsAtLineStart = busErrorCount();
  return canOk;
}

size_t freeBytes() { return LittleFS.totalBytes() - LittleFS.usedBytes(); }

// Appends one line to the log and flushes it, so a power cut loses at most
// the line being written.
void appendLine(const char* line) {
  if (!fsOk || logFull) return;
  if (freeBytes() < kMinFreeBytes) {
    logFull = true;
    Serial.println("Log full - logging stopped. DUMP it, then CLEAR LOG.");
    return;
  }
  if (!logFile) logFile = LittleFS.open(kLogPath, FILE_APPEND);
  if (!logFile) {
    fsOk = false;
    Serial.println("Log: can't open the file - logging stopped");
    return;
  }
  logFile.print(line);
  logFile.flush();
}

uint32_t nextBootNumber() {
  uint32_t n = 0;
  File f = LittleFS.open(kBootPath, FILE_READ);
  if (f) {
    n = static_cast<uint32_t>(f.readString().toInt());
    f.close();
  }
  n++;
  f = LittleFS.open(kBootPath, FILE_WRITE);
  if (f) {
    f.print(n);
    f.close();
  }
  return n;
}

void writeBootHeader() {
  char line[160];
  snprintf(line, sizeof(line), "# boot %lu, reset %s, CAN listen-only at %lu kbit/s%s, calibration %.3f\n",
           static_cast<unsigned long>(bootNumber), resetReasonName(esp_reset_reason()),
           static_cast<unsigned long>(bitrate / 1000), canOk ? "" : " (FAILED)", kCalibration);
  appendLine(line);
  appendLine("t_s,v_avg,v_min,v_max,frames,bus_errors\n");
}

void finishLine(uint32_t nowMs) {
  const uint32_t busErrorsNow = busErrorCount();
  const uint32_t busErrors = busErrorsNow - busErrorsAtLineStart;
  busErrorsAtLineStart = busErrorsNow;
  const float avg = sampleCount ? mvToVolts(static_cast<uint32_t>(sumMv / sampleCount)) : 0.0f;
  const float lo = sampleCount ? mvToVolts(minMv) : 0.0f;
  const float hi = sampleCount ? mvToVolts(maxMv) : 0.0f;
  char line[96];
  snprintf(line, sizeof(line), "%lu,%.2f,%.2f,%.2f,%lu,%lu\n", static_cast<unsigned long>(nowMs / 1000), avg, lo,
           hi, static_cast<unsigned long>(framesThisLine), static_cast<unsigned long>(busErrors));
  appendLine(line);
  linesThisBoot++;
  if (live) {
    Serial.printf("%lus  %.2fV (%.2f-%.2f)  frames %lu  bus errors %lu%s\n", static_cast<unsigned long>(nowMs / 1000),
                  avg, lo, hi, static_cast<unsigned long>(framesThisLine), static_cast<unsigned long>(busErrors),
                  (fsOk && !logFull) ? "" : "  NOT LOGGING");
  }
  sampleCount = 0;
  sumMv = 0;
  minMv = UINT32_MAX;
  maxMv = 0;
  framesThisLine = 0;
  if (fsOk && !logFull) {
    digitalWrite(kLedPin, LOW);
    ledOffAtMs = nowMs + 30;
  }
}

// Writes `len` bytes to the console, waiting for room: a dump must not drop
// data the way live output may. False if the host stopped reading.
bool writeAll(const uint8_t* data, size_t len) {
  uint32_t stalledSince = millis();
  while (len > 0) {
    const int room = Serial.availableForWrite();
    if (room <= 0) {
      if (millis() - stalledSince > kDumpStallMs) return false;
      delay(1);
      continue;
    }
    const size_t n = len < static_cast<size_t>(room) ? len : static_cast<size_t>(room);
    const size_t wrote = Serial.write(data, n);
    if (wrote > 0) stalledSince = millis();
    data += wrote;
    len -= wrote;
  }
  return true;
}

bool writeAll(const char* s) { return writeAll(reinterpret_cast<const uint8_t*>(s), strlen(s)); }

void dump() {
  if (logFile) logFile.close();  // reopened by the next append
  File f = LittleFS.open(kLogPath, FILE_READ);
  const size_t size = f ? f.size() : 0;
  char head[64];
  snprintf(head, sizeof(head), "#BEGIN %lu bytes\n", static_cast<unsigned long>(size));
  bool ok = writeAll(head);
  uint8_t buf[256];
  while (ok && f && f.available()) {
    const size_t n = f.read(buf, sizeof(buf));
    if (n == 0) break;
    ok = writeAll(buf, n);
  }
  if (f) f.close();
  if (ok) writeAll("#END\n");
}

void printStatus() {
  size_t logBytes = 0;
  if (fsOk) {
    if (logFile) logFile.flush();
    File f = LittleFS.open(kLogPath, FILE_READ);
    if (f) {
      logBytes = f.size();
      f.close();
    }
  }
  Serial.printf("Boot %lu, up %lus | log %s: %lu bytes, %lu lines this boot, %lu KB free of %lu KB | "
                "CAN %s at %lu kbit/s, live %s\n",
                static_cast<unsigned long>(bootNumber), millis() / 1000,
                !fsOk ? "FAILED" : logFull ? "FULL" : "ok", static_cast<unsigned long>(logBytes),
                static_cast<unsigned long>(linesThisBoot),
                fsOk ? static_cast<unsigned long>(freeBytes() / 1024) : 0UL,
                fsOk ? static_cast<unsigned long>(LittleFS.totalBytes() / 1024) : 0UL,
                canOk ? "listening" : "FAILED", static_cast<unsigned long>(bitrate / 1000), live ? "on" : "off");
}

void handleLine(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  if (cmd == "STATUS" || cmd == "S") {
    printStatus();
  } else if (cmd == "DUMP") {
    dump();
  } else if (cmd == "RATE 500" || cmd == "RATE 250") {
    bitrate = cmd.endsWith("500") ? 500000 : 250000;
    Serial.printf("CAN: %s at %lu kbit/s\n", startCan() ? "listening" : "FAILED", static_cast<unsigned long>(bitrate / 1000));
    char line[64];
    snprintf(line, sizeof(line), "# CAN rate changed to %lu kbit/s\n", static_cast<unsigned long>(bitrate / 1000));
    appendLine(line);
  } else if (cmd == "CLEAR LOG") {
    if (logFile) logFile.close();
    LittleFS.remove(kLogPath);
    logFull = false;
    linesThisBoot = 0;
    Serial.println("Log cleared");
    writeBootHeader();
  } else if (cmd == "QUIET") {
    live = false;
  } else if (cmd == "LIVE") {
    live = true;
  } else if (cmd.length()) {
    Serial.printf("Unknown '%s': STATUS, DUMP, RATE 500|250, CLEAR LOG, QUIET, LIVE\n", cmd.c_str());
  }
}

}  // namespace

void setup() {
  pinMode(kLedPin, OUTPUT);
  digitalWrite(kLedPin, HIGH);
  pinMode(kSilentPin, INPUT);  // released: listen-only, held by R1

  // Never block on a USB host that isn't reading: in the car, nothing is.
  Serial.setTxTimeoutMs(0);
  Serial.begin(115200);
  const uint32_t start = millis();
  while (!Serial && millis() - start < 1000) delay(10);

  analogReadResolution(12);
  Serial.println();
  Serial.println("Gnimu-CAN voltage log");
  fsOk = LittleFS.begin(true);  // formats the partition on first use
  if (fsOk) {
    bootNumber = nextBootNumber();
    Serial.printf("Log: %lu KB free of %lu KB, boot %lu\n", static_cast<unsigned long>(freeBytes() / 1024),
                  static_cast<unsigned long>(LittleFS.totalBytes() / 1024), static_cast<unsigned long>(bootNumber));
  } else {
    Serial.println("Log: flash file system FAILED - not logging");
  }
  Serial.printf("CAN: %s at %lu kbit/s\n", startCan() ? "listening" : "FAILED", static_cast<unsigned long>(bitrate / 1000));
  writeBootHeader();
  Serial.println("Commands: STATUS, DUMP, RATE 500|250, CLEAR LOG, QUIET, LIVE");
  nextSampleMs = millis();
  nextLineMs = nextSampleMs + kLinePeriodMs;
}

void loop() {
  const uint32_t now = millis();

  twai_message_t m;
  while (canOk && twai_receive(&m, 0) == ESP_OK) framesThisLine++;

  if (static_cast<int32_t>(now - nextSampleMs) >= 0) {
    nextSampleMs += kSamplePeriodMs;
    const uint32_t mv = analogReadMilliVolts(kSensePin);
    sampleCount++;
    sumMv += mv;
    if (mv < minMv) minMv = mv;
    if (mv > maxMv) maxMv = mv;
  }

  if (static_cast<int32_t>(now - nextLineMs) >= 0) {
    nextLineMs += kLinePeriodMs;
    finishLine(now);
  }

  // LED: the per-second blink, or fast blinking when the log can't be written.
  if (!fsOk || logFull) {
    digitalWrite(kLedPin, (now / 100) % 2 ? LOW : HIGH);
  } else if (ledOffAtMs && static_cast<int32_t>(now - ledOffAtMs) >= 0) {
    digitalWrite(kLedPin, HIGH);
    ledOffAtMs = 0;
  }

  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r' || c == '\n') {
      if (input.length()) handleLine(input);
      input = "";
    } else if (input.length() < 32) {
      input += c;
    }
  }
  delay(1);
}
