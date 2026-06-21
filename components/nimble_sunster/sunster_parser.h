#pragma once

#include <cstddef>
#include <cstdint>

namespace esphome::binary_sensor {
class BinarySensor;
}
namespace esphome::sensor {
class Sensor;
}
namespace esphome::text_sensor {
class TextSensor;
}
namespace esphome::number {
class Number;
}

namespace esphome::nimble_sunster {

void sunster_parser_set_entities(
    binary_sensor::BinarySensor *running, text_sensor::TextSensor *glow_plug, text_sensor::TextSensor *mode,
    sensor::Sensor *error_code, sensor::Sensor *battery_voltage, sensor::Sensor *altitude,
    sensor::Sensor *running_status_code, sensor::Sensor *room_temperature, sensor::Sensor *heating_temperature,
    number::Number *heater_level, number::Number *heater_temperature);

void sunster_parser_handle_notify(const uint8_t *data, size_t len);

}  // namespace esphome::nimble_sunster
