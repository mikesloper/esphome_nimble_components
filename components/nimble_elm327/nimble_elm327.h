#pragma once

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/components/nimble_gap/nimble_gap.h"
#include <cstdint>
#include <string>

namespace esphome::sensor {
class Sensor;
}

namespace esphome::nimble_host {
class NimbleHost;
}

struct ble_gap_event;

namespace esphome::nimble_elm327 {

class NimbleElm327 : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override;

  void set_nimble_host(nimble_host::NimbleHost *host) { this->host_ = host; }
  void set_address(uint64_t address) { this->address_ = address; }
  void set_service_uuid(const std::string &uuid) { this->service_uuid_ = uuid; }
  void set_notify_uuid(const std::string &uuid) { this->notify_uuid_ = uuid; }
  void set_write_uuid(const std::string &uuid) { this->write_uuid_ = uuid; }
  void set_passkey(uint32_t passkey) { this->passkey_ = passkey; }
  void set_auto_connect(bool auto_connect) { this->auto_connect_ = auto_connect; }
  void set_rpm_sensor(sensor::Sensor *s) { this->rpm_sensor_ = s; }
  void set_kph_sensor(sensor::Sensor *s) { this->kph_sensor_ = s; }
  void set_coolant_sensor(sensor::Sensor *s) { this->coolant_sensor_ = s; }
  void set_voltage_sensor(sensor::Sensor *s) { this->voltage_sensor_ = s; }
  void set_uptime_sensor(sensor::Sensor *s) { this->uptime_sensor_ = s; }

  bool is_connected() const { return this->conn_handle_ != 0xFFFF; }
  uint64_t get_address() const { return this->address_; }
  void write(const uint8_t *data, size_t len);
  void on_notify_data(const uint8_t *data, size_t len);
  void start_connect();
  void schedule_reconnect();
  void on_scan_timeout_();
  int handle_gap_event_(struct ble_gap_event *event);
  void enable_notifications();
  bool parse_uuid_(const std::string &uuid_str, void *out_uuid) const;

  std::string service_uuid_;
  std::string notify_uuid_;
  std::string write_uuid_;
  uint32_t passkey_{0};

  uint16_t conn_handle_{0xFFFF};
  uint16_t svc_start_handle_{0};
  uint16_t svc_end_handle_{0};
  uint16_t write_handle_{0};
  uint16_t notify_val_handle_{0};

 protected:
  nimble_host::NimbleHost *host_{nullptr};
  nimble_gap::NimbleGapClient gap_client_{};
  sensor::Sensor *rpm_sensor_{nullptr};
  sensor::Sensor *kph_sensor_{nullptr};
  sensor::Sensor *coolant_sensor_{nullptr};
  sensor::Sensor *voltage_sensor_{nullptr};
  sensor::Sensor *uptime_sensor_{nullptr};
  uint64_t address_{0};
  bool auto_connect_{true};

  uint32_t reconnect_at_ms_{0};
  bool connect_attempted_{false};
  bool gap_registered_{false};

  void ensure_gap_registered_();
};

template<typename... Ts>
class NimbleElm327WriteAction : public Parented<NimbleElm327>, public Action<Ts...> {
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

}  // namespace esphome::nimble_elm327
