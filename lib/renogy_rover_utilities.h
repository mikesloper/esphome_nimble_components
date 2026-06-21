#pragma once

#include "esphome/core/log.h"
#include <cmath>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

inline std::vector<uint8_t> GetRoverRequest() {
  // Modbus read: device 255, function 3, register 256 (0x0100), 34 words
  return {255, 3, 1, 0, 0, 34, 209, 241};
}

inline uint16_t renogy_bytes_to_int(const std::vector<uint8_t> &data, size_t offset, size_t length) {
  uint16_t result = 0;
  for (size_t i = 0; i < length; i++) {
    result = (result << 8) | data[offset + i];
  }
  return result;
}

inline float renogy_parse_temperature(uint8_t raw_value) {
  int8_t sign = raw_value >> 7;
  float celsius = (sign == 1) ? -(static_cast<float>(raw_value) - 128.0f) : static_cast<float>(raw_value);
  return celsius;
}

inline std::map<std::string, float> renogy_parse_charging_info(const std::vector<uint8_t> &data) {
  std::map<std::string, float> output;
  if (data.size() < 69) {
    ESP_LOGE("renogy", "Received data is too short (%zu bytes)", data.size());
    return output;
  }

  output["battery_voltage"] = renogy_bytes_to_int(data, 5, 2) * 0.1f;
  output["battery_current"] = renogy_bytes_to_int(data, 7, 2) * 0.01f;
  output["battery_temperature"] = renogy_parse_temperature(data[10]);
  output["pv_voltage"] = renogy_bytes_to_int(data, 17, 2) * 0.1f;
  output["pv_power"] = renogy_bytes_to_int(data, 21, 2);
  output["pv_current"] = renogy_bytes_to_int(data, 19, 2) * 0.01f;
  output["charging_status_code"] = data[68];
  output["load_voltage"] = renogy_bytes_to_int(data, 11, 2) * 0.1f;
  output["load_current"] = renogy_bytes_to_int(data, 13, 2) * 0.01f;
  return output;
}

inline std::map<std::string, float> HandleRoverData(const std::vector<uint8_t> &data) {
  return renogy_parse_charging_info(data);
}

inline const char *getChargerStatusStr(int status_code) {
  switch (status_code) {
    case 0:
      return "inactive";
    case 1:
      return "active";
    case 2:
      return "mppt";
    case 3:
      return "equalizing";
    case 4:
      return "boost";
    case 5:
      return "floating";
    case 6:
      return "current limiting";
    default:
      return "unknown";
  }
}
