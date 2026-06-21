#include "sunster_parser.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/number/number.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/log.h"

#include "air_heater_ble.h"
#include "air_heater_state.h"

#include <vector>

static const char *const TAG = "sunster_parser";

static esphome::binary_sensor::BinarySensor *s_running_{nullptr};
static esphome::text_sensor::TextSensor *s_glow_plug_{nullptr};
static esphome::text_sensor::TextSensor *s_mode_{nullptr};
static esphome::sensor::Sensor *s_error_code_{nullptr};
static esphome::sensor::Sensor *s_battery_voltage_{nullptr};
static esphome::sensor::Sensor *s_altitude_{nullptr};
static esphome::sensor::Sensor *s_running_status_code_{nullptr};
static esphome::sensor::Sensor *s_room_temperature_{nullptr};
static esphome::sensor::Sensor *s_heating_temperature_{nullptr};
static esphome::number::Number *s_heater_level_{nullptr};
static esphome::number::Number *s_heater_temperature_{nullptr};

namespace esphome::nimble_sunster {

void sunster_parser_set_entities(
    binary_sensor::BinarySensor *running, text_sensor::TextSensor *glow_plug, text_sensor::TextSensor *mode,
    sensor::Sensor *error_code, sensor::Sensor *battery_voltage, sensor::Sensor *altitude,
    sensor::Sensor *running_status_code, sensor::Sensor *room_temperature, sensor::Sensor *heating_temperature,
    number::Number *heater_level, number::Number *heater_temperature) {
  s_running_ = running;
  s_glow_plug_ = glow_plug;
  s_mode_ = mode;
  s_error_code_ = error_code;
  s_battery_voltage_ = battery_voltage;
  s_altitude_ = altitude;
  s_running_status_code_ = running_status_code;
  s_room_temperature_ = room_temperature;
  s_heating_temperature_ = heating_temperature;
  s_heater_level_ = heater_level;
  s_heater_temperature_ = heater_temperature;
}

void sunster_parser_handle_notify(const uint8_t *data, size_t len) {
  if (data == nullptr || len == 0)
    return;

  static const char *const states[] = {"Off", "Self test running", "Ignition", "Heating", "Shutting down"};

  std::vector<uint8_t> x(data, data + len);

  if (x[0] == 170)
    return;

  diesel_heater_ble::HeaterState state;
  if (!diesel_heater_ble::ResponseParser::parse(x, state)) {
    ESP_LOGD(TAG, "Failed to parse response");
    return;
  }

  ESP_LOGD(TAG, "running state: %d", state.runningstate);

  if (s_running_ != nullptr)
    s_running_->publish_state(state.runningstate != 0);
  if (s_heating_temperature_ != nullptr)
    s_heating_temperature_->publish_state(state.casetemp);
  if (s_error_code_ != nullptr)
    s_error_code_->publish_state(state.errcode);
  if (s_mode_ != nullptr)
    s_mode_->publish_state(state.runningmode == 2 ? "Automatic" : "Level");
  if (s_altitude_ != nullptr)
    s_altitude_->publish_state(state.altitude);
  if (s_room_temperature_ != nullptr)
    s_room_temperature_->publish_state(state.cabtemp);
  if (s_battery_voltage_ != nullptr)
    s_battery_voltage_->publish_state(state.supplyvoltage);
  if (s_heater_level_ != nullptr)
    s_heater_level_->publish_state(state.setlevel);
  if (s_heater_temperature_ != nullptr)
    s_heater_temperature_->publish_state(state.settemp);

  if (state.runningstep < sizeof(states) / sizeof(states[0])) {
    if (s_running_status_code_ != nullptr)
      s_running_status_code_->publish_state(state.runningstep);
    if (s_glow_plug_ != nullptr)
      s_glow_plug_->publish_state(states[state.runningstep]);
  } else {
    ESP_LOGD(TAG, "Unknown running step: %02d", state.runningstep);
  }
}

}  // namespace esphome::nimble_sunster
