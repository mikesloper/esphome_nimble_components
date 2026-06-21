#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include <cstdint>
#include <string>

namespace esphome::sensor {
class Sensor;
}
namespace esphome::text_sensor {
class TextSensor;
}
namespace esphome::nimble_host {
class NimbleHost;
}

namespace esphome::nimble_renogy {

class NimbleRenogy : public PollingComponent {
 public:
  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override;

  void set_nimble_host(nimble_host::NimbleHost *host) { this->host_ = host; }
  void set_address(uint64_t address) { this->address_ = address; }
  void set_notify_service_uuid(const std::string &uuid) { this->notify_service_uuid_ = uuid; }
  void set_notify_uuid(const std::string &uuid) { this->notify_uuid_ = uuid; }
  void set_write_service_uuid(const std::string &uuid) { this->write_service_uuid_ = uuid; }
  void set_write_uuid(const std::string &uuid) { this->write_uuid_ = uuid; }
  void set_passkey(uint32_t passkey) { this->passkey_ = passkey; }
  void set_auto_connect(bool auto_connect) { this->auto_connect_ = auto_connect; }
  void set_device_model(const std::string &model) { this->device_model_ = model; }

  void set_charging_status_text_sensor(text_sensor::TextSensor *s) { this->charging_status_text_sensor_ = s; }
  void set_battery_voltage_sensor(sensor::Sensor *s) { this->battery_voltage_sensor_ = s; }
  void set_battery_current_sensor(sensor::Sensor *s) { this->battery_current_sensor_ = s; }
  void set_battery_temperature_sensor(sensor::Sensor *s) { this->battery_temperature_sensor_ = s; }
  void set_pv_voltage_sensor(sensor::Sensor *s) { this->pv_voltage_sensor_ = s; }
  void set_pv_current_sensor(sensor::Sensor *s) { this->pv_current_sensor_ = s; }
  void set_pv_power_sensor(sensor::Sensor *s) { this->pv_power_sensor_ = s; }
  void set_load_voltage_sensor(sensor::Sensor *s) { this->load_voltage_sensor_ = s; }
  void set_load_current_sensor(sensor::Sensor *s) { this->load_current_sensor_ = s; }
  void set_total_current_sensor(sensor::Sensor *s) { this->total_current_sensor_ = s; }

  bool is_connected() const { return this->conn_handle_ != 0xFFFF; }
  uint64_t get_address() const { return this->address_; }
  uint8_t get_own_addr_type() const { return this->own_addr_type_; }
  void set_scanning(bool scanning) { this->scanning_ = scanning; }
  bool is_scanning() const { return this->scanning_; }
  void reset_scan_adv_count() { this->scan_adv_count_ = 0; }
  void increment_scan_adv_count() { this->scan_adv_count_++; }
  uint32_t get_scan_adv_count() const { return this->scan_adv_count_; }

  void on_notify_data(const uint8_t *data, size_t len);
  void start_connect();
  void schedule_reconnect();
  void enable_notifications();
  void discover_services_();
  void start_notify_char_discovery_(uint16_t conn_handle);
  void start_write_char_discovery_(uint16_t conn_handle);
  void try_finish_gatt_setup_(uint16_t conn_handle);
  bool parse_uuid_(const std::string &uuid_str, void *out_uuid) const;
  uint16_t target_uuid16_(const std::string &uuid_str) const;
  bool send_poll_request_();

  std::string notify_service_uuid_;
  std::string notify_uuid_;
  std::string write_service_uuid_;
  std::string write_uuid_;
  std::string device_model_;
  uint32_t passkey_{0};

  uint16_t conn_handle_{0xFFFF};
  uint16_t notify_svc_start_{0};
  uint16_t notify_svc_end_{0};
  uint16_t write_svc_start_{0};
  uint16_t write_svc_end_{0};
  uint16_t notify_val_handle_{0};
  uint16_t write_val_handle_{0};
  uint32_t discovered_svc_count_{0};
  bool notify_svc_is_16bit_{false};
  bool write_svc_is_16bit_{false};
  bool notify_char_is_16bit_{false};
  bool write_char_is_16bit_{false};
  bool gatt_ready_{false};

 protected:
  nimble_host::NimbleHost *host_{nullptr};
  text_sensor::TextSensor *charging_status_text_sensor_{nullptr};
  sensor::Sensor *battery_voltage_sensor_{nullptr};
  sensor::Sensor *battery_current_sensor_{nullptr};
  sensor::Sensor *battery_temperature_sensor_{nullptr};
  sensor::Sensor *pv_voltage_sensor_{nullptr};
  sensor::Sensor *pv_current_sensor_{nullptr};
  sensor::Sensor *pv_power_sensor_{nullptr};
  sensor::Sensor *load_voltage_sensor_{nullptr};
  sensor::Sensor *load_current_sensor_{nullptr};
  sensor::Sensor *total_current_sensor_{nullptr};

  uint64_t address_{0};
  bool auto_connect_{true};
  uint32_t reconnect_at_ms_{0};
  uint8_t own_addr_type_{0};
  bool connect_attempted_{false};
  bool scanning_{false};
  uint32_t scan_adv_count_{0};
};

}  // namespace esphome::nimble_renogy
