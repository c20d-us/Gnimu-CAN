#include "can_bus.h"

namespace can_bus {

static bool timingFor(uint32_t bitrate, twai_timing_config_t &out) {
  switch (bitrate) {
    case 125000: out = TWAI_TIMING_CONFIG_125KBITS(); return true;
    case 250000: out = TWAI_TIMING_CONFIG_250KBITS(); return true;
    case 500000: out = TWAI_TIMING_CONFIG_500KBITS(); return true;
    case 1000000: out = TWAI_TIMING_CONFIG_1MBITS(); return true;
    default: return false;
  }
}

bool begin(gpio_num_t txPin, gpio_num_t rxPin, uint32_t bitrate) {
  twai_timing_config_t timing;
  if (!timingFor(bitrate, timing)) {
    Serial.printf("[can] unsupported bitrate %lu\n", (unsigned long)bitrate);
    return false;
  }

  twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(txPin, rxPin, TWAI_MODE_NORMAL);
  general.rx_queue_len = 32;

  // Accept everything; OBD response filtering happens in software.
  twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&general, &timing, &filter) != ESP_OK) {
    Serial.println("[can] driver install failed");
    return false;
  }
  if (twai_start() != ESP_OK) {
    Serial.println("[can] start failed");
    return false;
  }
  Serial.printf("[can] started at %lu bit/s\n", (unsigned long)bitrate);
  return true;
}

bool send(uint32_t id, const uint8_t *data, uint8_t len) {
  twai_message_t msg = {};
  msg.identifier = id;
  msg.data_length_code = len;
  memcpy(msg.data, data, len);
  return twai_transmit(&msg, pdMS_TO_TICKS(10)) == ESP_OK;
}

bool receive(twai_message_t &msg, uint32_t timeoutMs) {
  return twai_receive(&msg, pdMS_TO_TICKS(timeoutMs)) == ESP_OK;
}

void service() {
  twai_status_info_t status;
  if (twai_get_status_info(&status) != ESP_OK) {
    return;
  }
  if (status.state == TWAI_STATE_BUS_OFF) {
    Serial.println("[can] bus-off, recovering");
    twai_initiate_recovery();
  } else if (status.state == TWAI_STATE_STOPPED) {
    // Recovery finished; the controller has to be started again.
    twai_start();
  }
}

}  // namespace can_bus
