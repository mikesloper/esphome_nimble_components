#pragma once

#include <cstdint>

// Victron BLE advertisement types (from esphome-victron_ble / extra-manufacturer-data spec).
// Decode logic adapted from https://github.com/Fabian-Schmidt/esphome-victron_ble

namespace esphome::nimble_victron {

static constexpr uint16_t VICTRON_MANUFACTURER_ID = 0x02E1;
static constexpr size_t VICTRON_ENCRYPTED_DATA_MAX_SIZE = 16;

enum class VICTRON_MANUFACTURER_RECORD_TYPE : uint8_t {
  PRODUCT_ADVERTISEMENT = 0x10,
};

enum class VICTRON_BLE_RECORD_TYPE : uint8_t {
  SOLAR_CHARGER = 0x01,
  BATTERY_MONITOR = 0x02,
};

enum vic_22bit_0_001 : int32_t;
enum vic_20bit_0_1_negative : uint32_t;
enum vic_16bit_0_01 : int16_t;
enum vic_16bit_0_01_positive : uint16_t;
enum vic_16bit_0_1 : int16_t;
enum vic_16bit_1_positive : uint16_t;
enum vic_temperature_16bit : uint16_t;
enum vic_10bit_0_1_positive : uint16_t;
enum vic_9bit_0_1_negative : uint16_t;

enum class VE_REG_BMV_AUX_INPUT : uint8_t {
  VE_REG_DC_CHANNEL2_VOLTAGE = 0x0,
  VE_REG_BATTERY_MID_POINT_VOLTAGE = 0x1,
  VE_REG_BAT_TEMPERATURE = 0x2,
  NONE = 0x3,
};

struct VICTRON_BLE_MANUFACTURER_DATA {
  VICTRON_MANUFACTURER_RECORD_TYPE manufacturer_record_type;
  uint8_t manufacturer_record_length;
  uint16_t product_id;
} __attribute__((packed));

struct VICTRON_BLE_RECORD_BASE {
  VICTRON_BLE_MANUFACTURER_DATA manufacturer_base;
  VICTRON_BLE_RECORD_TYPE record_type;
  uint8_t data_counter_lsb;
  uint8_t data_counter_msb;
  uint8_t encryption_key_0;
} __attribute__((packed));

struct VICTRON_BLE_RECORD_BATTERY_MONITOR {
  vic_16bit_1_positive time_to_go;
  vic_16bit_0_01 battery_voltage;
  uint16_t alarm_reason;
  union {
    vic_16bit_0_01 aux_voltage;
    vic_16bit_0_01_positive mid_voltage;
    vic_temperature_16bit temperature;
  } aux_input;
  VE_REG_BMV_AUX_INPUT aux_input_type : 2;
  vic_22bit_0_001 battery_current : 22;
  vic_20bit_0_1_negative consumed_ah : 20;
  vic_10bit_0_1_positive state_of_charge : 10;
} __attribute__((packed));

struct VICTRON_BLE_RECORD_SOLAR_CHARGER {
  uint8_t device_state;
  uint8_t charger_error;
  vic_16bit_0_01 battery_voltage;
  vic_16bit_0_1 battery_current;
  vic_16bit_0_01_positive yield_today;
  vic_16bit_1_positive pv_power;
  vic_9bit_0_1_negative load_current : 9;
} __attribute__((packed));

}  // namespace esphome::nimble_victron
