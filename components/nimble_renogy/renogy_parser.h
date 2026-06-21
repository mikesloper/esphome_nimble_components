#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace esphome::sensor {
class Sensor;
}
namespace esphome::text_sensor {
class TextSensor;
}

namespace esphome::nimble_renogy {

void renogy_parser_set_entities(
    text_sensor::TextSensor *charging_status, sensor::Sensor *battery_voltage, sensor::Sensor *battery_current,
    sensor::Sensor *battery_temperature, sensor::Sensor *pv_voltage, sensor::Sensor *pv_current,
    sensor::Sensor *pv_power, sensor::Sensor *load_voltage, sensor::Sensor *load_current,
    sensor::Sensor *total_current);

void renogy_parser_set_device_model(const std::string &model);
void renogy_parser_handle_notify(const uint8_t *data, size_t len);

}  // namespace esphome::nimble_renogy
