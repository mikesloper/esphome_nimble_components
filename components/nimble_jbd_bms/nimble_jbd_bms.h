#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include <cstdint>
#include <string>
#include <vector>

namespace esphome::nimble_host {
class NimbleHost;
}

namespace esphome::nimble_jbd_bms {

class NimbleJbdBms : public PollingComponent {
 public:
  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override;

  void set_nimble_host(nimble_host::NimbleHost *host) { this->host_ = host; }
  void set_address(uint64_t address) { this->address_ = address; }
  void set_service_uuid(const std::string &uuid) { this->service_uuid_ = uuid; }
  void set_notify_uuid(const std::string &uuid) { this->notify_uuid_ = uuid; }
  void set_control_uuid(const std::string &uuid) { this->control_uuid_ = uuid; }
  void set_passkey(uint32_t passkey) { this->passkey_ = passkey; }
  void set_auto_connect(bool auto_connect) { this->auto_connect_ = auto_connect; }
  void set_password(const std::string &password) {
    this->password_ = password;
    this->enable_authentication_ = !password.empty();
  }
  void set_authentication_timeout(uint32_t timeout_ms) { this->auth_timeout_ms_ = timeout_ms; }

  void set_state_of_charge_sensor(sensor::Sensor *s) { this->state_of_charge_sensor_ = s; }
  void set_total_voltage_sensor(sensor::Sensor *s) { this->total_voltage_sensor_ = s; }
  void set_current_sensor(sensor::Sensor *s) { this->current_sensor_ = s; }
  void set_power_sensor(sensor::Sensor *s) { this->power_sensor_ = s; }
  void set_charging_power_sensor(sensor::Sensor *s) { this->charging_power_sensor_ = s; }
  void set_discharging_power_sensor(sensor::Sensor *s) { this->discharging_power_sensor_ = s; }
  void set_nominal_capacity_sensor(sensor::Sensor *s) { this->nominal_capacity_sensor_ = s; }
  void set_charging_cycles_sensor(sensor::Sensor *s) { this->charging_cycles_sensor_ = s; }
  void set_capacity_remaining_sensor(sensor::Sensor *s) { this->capacity_remaining_sensor_ = s; }
  void set_min_cell_voltage_sensor(sensor::Sensor *s) { this->min_cell_voltage_sensor_ = s; }
  void set_max_cell_voltage_sensor(sensor::Sensor *s) { this->max_cell_voltage_sensor_ = s; }
  void set_min_voltage_cell_sensor(sensor::Sensor *s) { this->min_voltage_cell_sensor_ = s; }
  void set_max_voltage_cell_sensor(sensor::Sensor *s) { this->max_voltage_cell_sensor_ = s; }
  void set_delta_cell_voltage_sensor(sensor::Sensor *s) { this->delta_cell_voltage_sensor_ = s; }
  void set_average_cell_voltage_sensor(sensor::Sensor *s) { this->average_cell_voltage_sensor_ = s; }
  void set_cell_count_sensor(sensor::Sensor *s) { this->cell_count_sensor_ = s; }
  void set_cell_voltage_sensor(uint8_t cell, sensor::Sensor *s) { this->cells_[cell].cell_voltage_sensor_ = s; }
  void set_temperature_sensor(uint8_t temperature, sensor::Sensor *s) {
    this->temperatures_[temperature].temperature_sensor_ = s;
  }
  void set_charging_switch(switch_::Switch *s) { this->charging_switch_ = s; }
  void set_discharging_switch(switch_::Switch *s) { this->discharging_switch_ = s; }
  void set_errors_text_sensor(text_sensor::TextSensor *s) { this->errors_text_sensor_ = s; }
  void set_operation_status_text_sensor(text_sensor::TextSensor *s) { this->operation_status_text_sensor_ = s; }

  bool is_connected() const { return this->conn_handle_ != 0xFFFF; }
  bool is_gatt_ready() const { return this->gatt_ready_; }
  uint64_t get_address() const { return this->address_; }
  uint8_t get_own_addr_type() const { return this->own_addr_type_; }
  void set_scanning(bool scanning) { this->scanning_ = scanning; }
  bool is_scanning() const { return this->scanning_; }
  void reset_scan_adv_count() { this->scan_adv_count_ = 0; }
  void increment_scan_adv_count() { this->scan_adv_count_++; }
  uint32_t get_scan_adv_count() const { return this->scan_adv_count_; }

  bool send_command(uint8_t command, uint8_t address, const uint8_t *data = nullptr, uint8_t data_len = 0);
  bool write_register(uint8_t address, uint16_t value);
  bool change_mosfet_status(uint8_t address, uint8_t bitmask, bool state);

  void on_notify_data(const uint8_t *data, size_t len);
  void start_connect();
  void schedule_reconnect();
  void enable_notifications();
  void discover_services_();
  void start_char_discovery_(uint16_t conn_handle);
  bool parse_uuid_(const std::string &uuid_str, void *out_uuid) const;
  uint16_t target_uuid16_(const std::string &uuid_str) const;

  std::string service_uuid_;
  std::string notify_uuid_;
  std::string control_uuid_;
  uint32_t passkey_{0};

  uint16_t conn_handle_{0xFFFF};
  uint16_t svc_start_handle_{0};
  uint16_t svc_end_handle_{0};
  uint16_t notify_val_handle_{0};
  uint16_t control_val_handle_{0};
  uint32_t discovered_svc_count_{0};

  bool gatt_ready_{false};
  uint8_t peer_addr_[6]{};
  std::vector<uint8_t> frame_buffer_;

  enum class AuthState {
    NOT_AUTHENTICATED,
    SENDING_APP_KEY,
    REQUESTING_RANDOM,
    SENDING_PASSWORD,
    REQUESTING_ROOT_RANDOM,
    SENDING_ROOT_PASSWORD,
    AUTHENTICATED
  };
  AuthState authentication_state_{AuthState::NOT_AUTHENTICATED};
  uint8_t random_byte_{0};

 protected:
  void assemble_(const uint8_t *data, uint16_t length);
  void on_jbd_bms_data_(const uint8_t &function, const std::vector<uint8_t> &data);
  void on_hardware_info_data_(const std::vector<uint8_t> &data);
  void on_cell_info_data_(const std::vector<uint8_t> &data);
  void handle_auth_response_(uint8_t command, const uint8_t *data, uint8_t data_len);
  void start_authentication_();
  void send_app_key_();
  void request_random_byte_();
  void send_user_password_();
  void send_root_password_();
  void send_auth_frame_(const uint8_t *frame, size_t length);
  void check_auth_timeout_();
  void track_online_status_();
  void reset_online_status_tracker_();
  void publish_device_unavailable_();
  void publish_state_(sensor::Sensor *sensor, float value);
  void publish_state_(switch_::Switch *obj, bool state);
  void publish_state_(text_sensor::TextSensor *text_sensor, const std::string &state);
  std::vector<uint8_t> build_frame_(uint8_t command, uint8_t address, const uint8_t *data, uint8_t data_len) const;
  uint16_t chksum_(const uint8_t data[], uint16_t len) const;
  uint8_t auth_chksum_(const uint8_t *data, uint16_t length) const;
  std::string bitmask_to_string_(const char *const messages[], uint8_t messages_size, uint16_t mask) const;

  nimble_host::NimbleHost *host_{nullptr};
  sensor::Sensor *state_of_charge_sensor_{nullptr};
  sensor::Sensor *total_voltage_sensor_{nullptr};
  sensor::Sensor *current_sensor_{nullptr};
  sensor::Sensor *power_sensor_{nullptr};
  sensor::Sensor *charging_power_sensor_{nullptr};
  sensor::Sensor *discharging_power_sensor_{nullptr};
  sensor::Sensor *nominal_capacity_sensor_{nullptr};
  sensor::Sensor *charging_cycles_sensor_{nullptr};
  sensor::Sensor *capacity_remaining_sensor_{nullptr};
  sensor::Sensor *min_cell_voltage_sensor_{nullptr};
  sensor::Sensor *max_cell_voltage_sensor_{nullptr};
  sensor::Sensor *min_voltage_cell_sensor_{nullptr};
  sensor::Sensor *max_voltage_cell_sensor_{nullptr};
  sensor::Sensor *delta_cell_voltage_sensor_{nullptr};
  sensor::Sensor *average_cell_voltage_sensor_{nullptr};
  sensor::Sensor *cell_count_sensor_{nullptr};
  switch_::Switch *charging_switch_{nullptr};
  switch_::Switch *discharging_switch_{nullptr};
  text_sensor::TextSensor *errors_text_sensor_{nullptr};
  text_sensor::TextSensor *operation_status_text_sensor_{nullptr};

  struct Cell {
    sensor::Sensor *cell_voltage_sensor_{nullptr};
  } cells_[32];

  struct Temperature {
    sensor::Sensor *temperature_sensor_{nullptr};
  } temperatures_[6];

  uint64_t address_{0};
  bool auto_connect_{true};
  uint8_t no_response_count_{0};
  uint8_t mosfet_status_{255};
  uint32_t reconnect_at_ms_{0};
  uint8_t own_addr_type_{0};
  bool connect_attempted_{false};
  bool scanning_{false};
  uint32_t scan_adv_count_{0};
  bool enable_authentication_{false};
  std::string password_{};
  uint32_t auth_timeout_start_{0};
  uint32_t auth_timeout_ms_{10000};
};

}  // namespace esphome::nimble_jbd_bms
