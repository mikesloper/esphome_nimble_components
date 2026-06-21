#pragma once

#include <map>
#include <string>

namespace esphome::sensor {
class Sensor;
}

extern std::map<int, int> elm327_pid_status;

void init_elm327_pid_status();
void handle_eml_reponse(const std::string &raw);

namespace esphome::nimble_elm327 {

void elm327_parser_set_sensors(esphome::sensor::Sensor *rpm, esphome::sensor::Sensor *kph,
                                esphome::sensor::Sensor *coolant, esphome::sensor::Sensor *voltage,
                                esphome::sensor::Sensor *uptime);

}  // namespace esphome::nimble_elm327
