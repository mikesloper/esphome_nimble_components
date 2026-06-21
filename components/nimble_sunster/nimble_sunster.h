#pragma once

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include <cstdint>
#include <string>

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
namespace esphome::nimble_host {
class NimbleHost;
}

namespace esphome::nimble_sunster {

class NimbleSunster : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override;

  void set_nimble_host(nimble_host::NimbleHost *host) { this->host_ = host; }
  void set_address(uint64_t address) { this->address_ = address; }
  void set_service_uuid(const std::string &uuid) { this->service_uuid_ = uuid; }
  void set_characteristic_uuid(const std::string &uuid) { this->characteristic_uuid_ = uuid; }
  void set_passkey(uint32_t passkey) { this->passkey_ = passkey; }
  void set_auto_connect(bool auto_connect) { this->auto_connect_ = auto_connect; }

  void set_running_binary_sensor(binary_sensor::BinarySensor *s) { this->running_binary_sensor_ = s; }
  void set_glow_plug_text_sensor(text_sensor::TextSensor *s) { this->glow_plug_text_sensor_ = s; }
  void set_mode_text_sensor(text_sensor::TextSensor *s) { this->mode_text_sensor_ = s; }
  void set_error_code_sensor(sensor::Sensor *s) { this->error_code_sensor_ = s; }
  void set_battery_voltage_sensor(sensor::Sensor *s) { this->battery_voltage_sensor_ = s; }
  void set_altitude_sensor(sensor::Sensor *s) { this->altitude_sensor_ = s; }
  void set_running_status_code_sensor(sensor::Sensor *s) { this->running_status_code_sensor_ = s; }
  void set_room_temperature_sensor(sensor::Sensor *s) { this->room_temperature_sensor_ = s; }
  void set_heating_temperature_sensor(sensor::Sensor *s) { this->heating_temperature_sensor_ = s; }
  void set_heater_level_number(number::Number *n) { this->heater_level_number_ = n; }
  void set_heater_temperature_number(number::Number *n) { this->heater_temperature_number_ = n; }

  bool is_connected() const { return this->conn_handle_ != 0xFFFF; }
  uint64_t get_address() const { return this->address_; }
  uint8_t get_own_addr_type() const { return this->own_addr_type_; }
  void set_scanning(bool scanning) { this->scanning_ = scanning; }
  bool is_scanning() const { return this->scanning_; }
  void reset_scan_adv_count() { this->scan_adv_count_ = 0; }
  void increment_scan_adv_count() { this->scan_adv_count_++; }
  uint32_t get_scan_adv_count() const { return this->scan_adv_count_; }
  void write(const uint8_t *data, size_t len);
  void on_notify_data(const uint8_t *data, size_t len);
  void start_connect();
  void schedule_reconnect();
  void enable_notifications();
  void discover_services_();
  void start_char_discovery_(uint16_t conn_handle);
  bool parse_uuid_(const std::string &uuid_str, void *out_uuid) const;
  uint16_t target_uuid16_(const std::string &uuid_str) const;

  std::string service_uuid_;
  std::string characteristic_uuid_;
  uint32_t passkey_{0};

  uint16_t conn_handle_{0xFFFF};
  uint16_t svc_start_handle_{0};
  uint16_t svc_end_handle_{0};
  uint16_t char_handle_{0};
  uint16_t notify_val_handle_{0};
  uint32_t discovered_svc_count_{0};

 protected:
  nimble_host::NimbleHost *host_{nullptr};
  binary_sensor::BinarySensor *running_binary_sensor_{nullptr};
  text_sensor::TextSensor *glow_plug_text_sensor_{nullptr};
  text_sensor::TextSensor *mode_text_sensor_{nullptr};
  sensor::Sensor *error_code_sensor_{nullptr};
  sensor::Sensor *battery_voltage_sensor_{nullptr};
  sensor::Sensor *altitude_sensor_{nullptr};
  sensor::Sensor *running_status_code_sensor_{nullptr};
  sensor::Sensor *room_temperature_sensor_{nullptr};
  sensor::Sensor *heating_temperature_sensor_{nullptr};
  number::Number *heater_level_number_{nullptr};
  number::Number *heater_temperature_number_{nullptr};
  uint64_t address_{0};
  bool auto_connect_{true};

  uint32_t reconnect_at_ms_{0};
  uint8_t own_addr_type_{0};
  bool connect_attempted_{false};
  bool scanning_{false};
  uint32_t scan_adv_count_{0};
};

template<typename... Ts>
class NimbleSunsterWriteAction : public Parented<NimbleSunster>, public Action<Ts...> {
 public:
  void set_data(const uint8_t *data, size_t len) {
    this->data_ = data;
    this->len_ = len;
  }

  void play(const Ts &...x) override { this->parent_->write(this->data_, this->len_); }

 protected:
  const uint8_t *data_{nullptr};
  size_t len_{0};
};

}  // namespace esphome::nimble_sunster
