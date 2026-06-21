#pragma once

#include "victron_types.h"
#include <cmath>

namespace esphome::nimble_victron {

inline float vic_to_float(vic_16bit_0_01 val) {
  if (static_cast<int16_t>(val) == 0x7FFF)
    return NAN;
  return 0.01f * static_cast<int16_t>(val);
}

inline float vic_to_float(vic_16bit_0_01_positive val) {
  if (static_cast<uint16_t>(val) == 0xFFFF)
    return NAN;
  return 0.01f * static_cast<uint16_t>(val);
}

inline float vic_to_float(vic_16bit_0_1 val) {
  if (static_cast<int16_t>(val) == 0x7FFF)
    return NAN;
  return 0.1f * static_cast<int16_t>(val);
}

inline float vic_to_float(vic_16bit_1_positive val) {
  if (static_cast<uint16_t>(val) == 0xFFFF)
    return NAN;
  return static_cast<float>(static_cast<uint16_t>(val));
}

inline float vic_to_float(vic_22bit_0_001 val) {
  if (static_cast<int32_t>(val) == 0x3FFFFF)
    return NAN;
  return 0.001f * static_cast<int32_t>(val);
}

inline float vic_to_float(vic_20bit_0_1_negative val) {
  if (static_cast<uint32_t>(val) == 0xFFFFF)
    return NAN;
  return -0.1f * static_cast<uint32_t>(val);
}

inline float vic_to_float(vic_10bit_0_1_positive val) {
  if (static_cast<uint16_t>(val) == 0x3FF)
    return NAN;
  return 0.1f * static_cast<uint16_t>(val);
}

inline float vic_to_float(vic_temperature_16bit val) {
  if (static_cast<uint16_t>(val) == 0xFFFF)
    return NAN;
  return 0.01f * static_cast<uint16_t>(val) - 273.15f;
}

}  // namespace esphome::nimble_victron
