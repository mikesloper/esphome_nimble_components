#include "renogy_parser.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/log.h"
#include "renogy_rover_utilities.h"

#include <vector>

namespace esphome::nimble_renogy {

static const char *const TAG = "renogy_parser";

static text_sensor::TextSensor *g_charging_status_{nullptr};
static sensor::Sensor *g_battery_voltage_{nullptr};
static sensor::Sensor *g_battery_current_{nullptr};
static sensor::Sensor *g_battery_temperature_{nullptr};
static sensor::Sensor *g_pv_voltage_{nullptr};
static sensor::Sensor *g_pv_current_{nullptr};
static sensor::Sensor *g_pv_power_{nullptr};
static sensor::Sensor *g_load_voltage_{nullptr};
static sensor::Sensor *g_load_current_{nullptr};
static sensor::Sensor *g_total_current_{nullptr};
static std::string g_device_model_;

void renogy_parser_set_entities(text_sensor::TextSensor *charging_status, sensor::Sensor *battery_voltage,
                                sensor::Sensor *battery_current, sensor::Sensor *battery_temperature,
                                sensor::Sensor *pv_voltage, sensor::Sensor *pv_current, sensor::Sensor *pv_power,
                                sensor::Sensor *load_voltage, sensor::Sensor *load_current,
                                sensor::Sensor *total_current) {
  g_charging_status_ = charging_status;
  g_battery_voltage_ = battery_voltage;
  g_battery_current_ = battery_current;
  g_battery_temperature_ = battery_temperature;
  g_pv_voltage_ = pv_voltage;
  g_pv_current_ = pv_current;
  g_pv_power_ = pv_power;
  g_load_voltage_ = load_voltage;
  g_load_current_ = load_current;
  g_total_current_ = total_current;
}

void renogy_parser_set_device_model(const std::string &model) { g_device_model_ = model; }

static void publish_(sensor::Sensor *s, float v) {
  if (s != nullptr)
    s->publish_state(v);
}

void renogy_parser_handle_notify(const uint8_t *data, size_t len) {
  if (len < 73) {
    ESP_LOGD(TAG, "Notify too short: %zu bytes", len);
    return;
  }

  std::vector<uint8_t> payload(data, data + len);
  auto parsed = HandleRoverData(payload);
  if (parsed.empty())
    return;

  ESP_LOGV(TAG, "Parsed Renogy charging info (%zu bytes)", len);

  if (g_charging_status_ != nullptr) {
    int status_code = static_cast<int>(parsed["charging_status_code"]);
    g_charging_status_->publish_state(getChargerStatusStr(status_code));
  }

  publish_(g_battery_voltage_, parsed["battery_voltage"]);
  publish_(g_battery_temperature_, parsed["battery_temperature"]);
  publish_(g_pv_voltage_, parsed["pv_voltage"]);
  publish_(g_pv_power_, parsed["pv_power"]);
  publish_(g_pv_current_, parsed["pv_current"]);
  publish_(g_load_voltage_, parsed["load_voltage"]);
  publish_(g_load_current_, parsed["load_current"]);

  float battery_current = parsed["battery_current"];
  if (g_device_model_ == "DCCX0S") {
    float load_i = parsed["load_current"];
    float load_v = parsed["load_voltage"];
    float pv_i = parsed["pv_current"];
    float pv_v = parsed["pv_voltage"];
    if (load_i != 0 && load_v != 0 && pv_i != 0 && pv_v != 0) {
      battery_current = (pv_i * pv_v) / (load_i * load_v) * battery_current;
    }
  }

  publish_(g_battery_current_, battery_current);
  publish_(g_total_current_, parsed["battery_current"]);
}

}  // namespace esphome::nimble_renogy
