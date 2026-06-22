#pragma once

#include "esphome/components/nimble_host/nimble_host.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "victron_types.h"

#include <array>
#include <cstdint>
#include <vector>

struct ble_gap_disc_desc;

namespace esphome::nimble_victron {

class NimbleVictron;

class NimbleVictronScanner {
 public:
  static void register_device(NimbleVictron *device);
  static void on_adv(const struct ble_gap_disc_desc &disc, void *context);

 private:
  static void handle_advertisement_(const struct ble_gap_disc_desc &disc, const uint8_t *payload, size_t payload_len);

  static std::vector<NimbleVictron *> devices_;
};

class NimbleVictron : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override;

  void set_nimble_host(nimble_host::NimbleHost *host) { this->host_ = host; }
  void set_address(uint64_t address) { this->address_ = address; }
  void set_bindkey(const std::array<uint8_t, 16> &bindkey) { this->bindkey_ = bindkey; }

  void set_battery_voltage_sensor(sensor::Sensor *s) { this->battery_voltage_sensor_ = s; }
  void set_battery_current_sensor(sensor::Sensor *s) { this->battery_current_sensor_ = s; }
  void set_state_of_charge_sensor(sensor::Sensor *s) { this->state_of_charge_sensor_ = s; }
  void set_consumed_ah_sensor(sensor::Sensor *s) { this->consumed_ah_sensor_ = s; }
  void set_time_to_go_sensor(sensor::Sensor *s) { this->time_to_go_sensor_ = s; }
  void set_aux_voltage_sensor(sensor::Sensor *s) { this->aux_voltage_sensor_ = s; }
  void set_pv_power_sensor(sensor::Sensor *s) { this->pv_power_sensor_ = s; }
  void set_solar_battery_voltage_sensor(sensor::Sensor *s) { this->solar_battery_voltage_sensor_ = s; }
  void set_solar_battery_current_sensor(sensor::Sensor *s) { this->solar_battery_current_sensor_ = s; }

  bool parse_advertisement(const uint8_t *payload, size_t payload_len);

  uint64_t get_address() const { return this->address_; }

 protected:
  bool decrypt_payload_(const uint8_t *crypted, size_t crypted_len, uint8_t *out, uint8_t counter_lsb,
                        uint8_t counter_msb);
  void publish_battery_monitor_(const VICTRON_BLE_RECORD_BATTERY_MONITOR *rec);
  void publish_solar_charger_(const VICTRON_BLE_RECORD_SOLAR_CHARGER *rec);
  void ensure_gap_registered_();

  nimble_host::NimbleHost *host_{nullptr};
  uint64_t address_{0};
  std::array<uint8_t, 16> bindkey_{};
  uint16_t last_data_counter_{0xFFFF};

  sensor::Sensor *battery_voltage_sensor_{nullptr};
  sensor::Sensor *battery_current_sensor_{nullptr};
  sensor::Sensor *state_of_charge_sensor_{nullptr};
  sensor::Sensor *consumed_ah_sensor_{nullptr};
  sensor::Sensor *time_to_go_sensor_{nullptr};
  sensor::Sensor *aux_voltage_sensor_{nullptr};
  sensor::Sensor *pv_power_sensor_{nullptr};
  sensor::Sensor *solar_battery_voltage_sensor_{nullptr};
  sensor::Sensor *solar_battery_current_sensor_{nullptr};
  bool device_registered_{false};
  static bool adv_listener_registered_;
};

}  // namespace esphome::nimble_victron
