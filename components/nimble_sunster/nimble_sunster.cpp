#include "nimble_sunster.h"
#include "sunster_parser.h"
#include "esphome/components/nimble_gap/nimble_gap.h"
#include "esphome/components/nimble_host/nimble_host.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"

extern "C" {
#include "host/ble_att.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "os/os_mbuf.h"
}

#include <algorithm>
#include <cstring>
#include <string>

namespace esphome::nimble_sunster {

using esphome::nimble_gap::global_nimble_gap;

static const char *const TAG = "nimble_sunster";

static int gap_event_trampoline_(struct ble_gap_event *event, void *context) {
  return static_cast<NimbleSunster *>(context)->handle_gap_event_(event);
}

static void scan_timeout_trampoline_(void *context) {
  static_cast<NimbleSunster *>(context)->on_scan_timeout_();
}

static bool uuid_matches_(const NimbleSunster *self, const ble_uuid_any_t *discovered, const std::string &target_str) {
  ble_uuid_any_t target{};
  if (!self->parse_uuid_(target_str, &target))
    return false;
  if (ble_uuid_cmp(&discovered->u, &target.u) == 0)
    return true;

  uint16_t target16 = self->target_uuid16_(target_str);
  if (target16 == 0)
    return false;

  if (discovered->u.type == BLE_UUID_TYPE_16)
    return BLE_UUID16(&discovered->u)->value == target16;

  if (discovered->u.type == BLE_UUID_TYPE_128) {
    const uint8_t *v = BLE_UUID128(&discovered->u)->value;
    uint16_t disc16 = (uint16_t) v[12] | ((uint16_t) v[13] << 8);
    return disc16 == target16;
  }

  return false;
}

static void log_uuid_(const char *label, const ble_uuid_t *uuid) {
  char uuid_buf[BLE_UUID_STR_LEN];
  ble_uuid_to_str(uuid, uuid_buf);
  ESP_LOGI(TAG, "%s: %s", label, uuid_buf);
}

uint16_t NimbleSunster::target_uuid16_(const std::string &uuid_str) const {
  std::string s = uuid_str;
  s.erase(std::remove(s.begin(), s.end(), '-'), s.end());
  if (s.length() <= 4)
    return static_cast<uint16_t>(std::stoul(s, nullptr, 16));
  if (s.length() == 32)
    return static_cast<uint16_t>(std::stoul(s.substr(4, 4), nullptr, 16));
  return 0;
}

bool NimbleSunster::parse_uuid_(const std::string &uuid_str, void *out_uuid) const {
  auto *out = static_cast<ble_uuid_any_t *>(out_uuid);
  std::string s = uuid_str;
  s.erase(std::remove(s.begin(), s.end(), '-'), s.end());

  if (s.length() <= 4) {
    uint16_t u16 = static_cast<uint16_t>(std::stoul(s, nullptr, 16));
    out->u.type = BLE_UUID_TYPE_16;
    out->u16.value = u16;
    return true;
  }

  if (s.length() == 32) {
    uint8_t u128[16];
    for (int i = 0; i < 16; i++) {
      u128[15 - i] = static_cast<uint8_t>(std::stoul(s.substr(i * 2, 2), nullptr, 16));
    }
    out->u.type = BLE_UUID_TYPE_128;
    memcpy(out->u128.value, u128, sizeof(u128));
    return true;
  }

  return false;
}

float NimbleSunster::get_setup_priority() const { return setup_priority::BUS + 1.0f; }

void NimbleSunster::ensure_gap_registered_() {
  if (this->gap_registered_) {
    return;
  }
  this->gap_client_.context = this;
  this->gap_client_.address = this->address_;
  this->gap_client_.log_tag = TAG;
  this->gap_client_.on_gap_event = gap_event_trampoline_;
  this->gap_client_.on_scan_timeout = scan_timeout_trampoline_;
  esphome::nimble_gap::register_gatt_client_once(&this->gap_client_, this->gap_registered_);
}

void NimbleSunster::setup() {
  if (this->host_ == nullptr) {
    ESP_LOGE(TAG, "NimBLE host not set");
    this->mark_failed();
    return;
  }

  this->ensure_gap_registered_();

  sunster_parser_set_entities(
      this->running_binary_sensor_, this->glow_plug_text_sensor_, this->mode_text_sensor_, this->error_code_sensor_,
      this->battery_voltage_sensor_, this->altitude_sensor_, this->running_status_code_sensor_,
      this->room_temperature_sensor_, this->heating_temperature_sensor_, this->heater_level_number_,
      this->heater_temperature_number_);

  this->host_->add_on_sync_callback([this]() {
    if (this->auto_connect_ && this->host_->is_active()) {
      this->start_connect();
    }
  });
}

void NimbleSunster::loop() {
  this->ensure_gap_registered_();
  if (!this->host_->is_active() || !this->host_->is_synced())
    return;

  if (this->reconnect_at_ms_ != 0 && millis() >= this->reconnect_at_ms_) {
    this->reconnect_at_ms_ = 0;
    this->start_connect();
    return;
  }

  if (this->auto_connect_ && !this->connect_attempted_ && this->conn_handle_ == 0xFFFF &&
      this->reconnect_at_ms_ == 0) {
    this->start_connect();
  }
}

void NimbleSunster::dump_config() {
  uint8_t mac[6];
  for (int i = 0; i < 6; i++) {
    mac[i] = (this->address_ >> ((5 - i) * 8)) & 0xFF;
  }
  char mac_buf[MAC_ADDRESS_PRETTY_BUFFER_SIZE];
  format_mac_addr_upper(mac, mac_buf);

  ESP_LOGCONFIG(TAG, "NimBLE Sunster heater:");
  ESP_LOGCONFIG(TAG, "  MAC: %s", mac_buf);
  ESP_LOGCONFIG(TAG, "  Connected: %s", YESNO(this->is_connected()));
}

static int disc_chr_cb(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_chr *chr,
                       void *arg);
static int disc_svc_cb(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_svc *svc,
                       void *arg);
static int mtu_cb(uint16_t conn_handle, const struct ble_gatt_error *error, uint16_t mtu, void *arg);

static int disc_chr_cb(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_chr *chr,
                       void *arg) {
  auto *self = static_cast<NimbleSunster *>(arg);
  if (error->status == BLE_HS_EDONE) {
    if (self->char_handle_ != 0) {
      self->enable_notifications();
    } else {
      ESP_LOGE(TAG, "Required characteristic not found");
      self->schedule_reconnect();
    }
    return 0;
  }
  if (error->status != 0)
    return error->status;

  if (chr == nullptr)
    return 0;

  log_uuid_("Discovered GATT characteristic", &chr->uuid.u);

  if (uuid_matches_(self, &chr->uuid, self->characteristic_uuid_)) {
    self->char_handle_ = chr->val_handle;
    self->notify_val_handle_ = chr->val_handle;
    ESP_LOGI(TAG, "Matched heater characteristic handle=%d", chr->val_handle);
  }
  return 0;
}

void NimbleSunster::start_char_discovery_(uint16_t conn_handle) {
  if (this->svc_start_handle_ == 0)
    return;

  uint16_t char16 = this->target_uuid16_(this->characteristic_uuid_);
  if (char16 != 0) {
    ble_uuid_any_t char_uuid{};
    char_uuid.u.type = BLE_UUID_TYPE_16;
    char_uuid.u16.value = char16;
    ESP_LOGI(TAG, "Discovering characteristic 0x%04x in handles %u-%u", char16, this->svc_start_handle_,
             this->svc_end_handle_);
    int rc = ble_gattc_disc_chrs_by_uuid(conn_handle, this->svc_start_handle_, this->svc_end_handle_, &char_uuid.u,
                                        disc_chr_cb, this);
    if (rc == 0)
      return;
    ESP_LOGW(TAG, "ble_gattc_disc_chrs_by_uuid failed: %d, falling back to discover all characteristics", rc);
  }

  int rc = ble_gattc_disc_all_chrs(conn_handle, this->svc_start_handle_, this->svc_end_handle_, disc_chr_cb, this);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_gattc_disc_all_chrs failed: %d", rc);
    this->schedule_reconnect();
  }
}

static int disc_svc_cb(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_svc *svc,
                       void *arg) {
  auto *self = static_cast<NimbleSunster *>(arg);
  if (error->status == BLE_HS_EDONE) {
    ESP_LOGI(TAG, "GATT service discovery finished, saw %u service(s)", self->discovered_svc_count_);
    if (self->svc_start_handle_ != 0) {
      self->start_char_discovery_(conn_handle);
    } else {
      ESP_LOGE(TAG, "Service %s not found", self->service_uuid_.c_str());
      self->schedule_reconnect();
    }
    return 0;
  }
  if (error->status != 0)
    return error->status;

  if (svc == nullptr)
    return 0;

  self->discovered_svc_count_++;
  log_uuid_("Discovered GATT service", &svc->uuid.u);

  if (!uuid_matches_(self, &svc->uuid, self->service_uuid_))
    return 0;

  self->svc_start_handle_ = svc->start_handle;
  self->svc_end_handle_ = svc->end_handle;
  ESP_LOGI(TAG, "Matched heater service handles=%u-%u", svc->start_handle, svc->end_handle);
  return 0;
}

void NimbleSunster::discover_services_() {
  this->discovered_svc_count_ = 0;
  this->svc_start_handle_ = 0;
  this->svc_end_handle_ = 0;
  this->char_handle_ = 0;
  this->notify_val_handle_ = 0;

  uint16_t svc16 = this->target_uuid16_(this->service_uuid_);
  if (svc16 != 0) {
    ble_uuid_any_t service_uuid{};
    service_uuid.u.type = BLE_UUID_TYPE_16;
    service_uuid.u16.value = svc16;
    ESP_LOGI(TAG, "Discovering service 0x%04x by UUID", svc16);
    int rc = ble_gattc_disc_svc_by_uuid(this->conn_handle_, &service_uuid.u, disc_svc_cb, this);
    if (rc == 0)
      return;
    ESP_LOGW(TAG, "ble_gattc_disc_svc_by_uuid failed: %d, falling back to discover all services", rc);
  }

  ble_uuid_any_t service_uuid{};
  if (this->parse_uuid_(this->service_uuid_, &service_uuid)) {
    ESP_LOGI(TAG, "Discovering service by 128-bit UUID");
    int rc = ble_gattc_disc_svc_by_uuid(this->conn_handle_, &service_uuid.u, disc_svc_cb, this);
    if (rc == 0)
      return;
    ESP_LOGW(TAG, "128-bit ble_gattc_disc_svc_by_uuid failed: %d, falling back to discover all services", rc);
  }

  int rc = ble_gattc_disc_all_svcs(this->conn_handle_, disc_svc_cb, this);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_gattc_disc_all_svcs failed: %d", rc);
    this->schedule_reconnect();
  }
}

static int mtu_cb(uint16_t conn_handle, const struct ble_gatt_error *error, uint16_t mtu, void *arg) {
  auto *self = static_cast<NimbleSunster *>(arg);
  if (error->status != 0) {
    ESP_LOGW(TAG, "MTU exchange status=%d", error->status);
  } else {
    ESP_LOGI(TAG, "MTU negotiated: %u", mtu);
  }
  self->discover_services_();
  return 0;
}

int NimbleSunster::handle_gap_event_(struct ble_gap_event *event) {
  switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
      if (event->connect.status == 0) {
        this->conn_handle_ = event->connect.conn_handle;
        this->gap_client_.conn_handle = this->conn_handle_;
        this->svc_start_handle_ = 0;
        this->svc_end_handle_ = 0;
        this->char_handle_ = 0;
        this->notify_val_handle_ = 0;
        this->discovered_svc_count_ = 0;
        ESP_LOGI(TAG, "Connected, conn_handle=%d", this->conn_handle_);
        int rc = ble_gattc_exchange_mtu(this->conn_handle_, mtu_cb, this);
        if (rc != 0) {
          ESP_LOGW(TAG, "ble_gattc_exchange_mtu failed: %d, discovering services anyway", rc);
          this->discover_services_();
        }
      } else {
        const char *reason = event->connect.status == BLE_HS_ETIMEOUT ? "timeout" : "error";
        ESP_LOGW(TAG, "Connection failed, status=%d (%s)", event->connect.status, reason);
        this->schedule_reconnect();
      }
      return 0;

    case BLE_GAP_EVENT_DISCONNECT:
      ESP_LOGW(TAG, "Disconnected, reason=%d", event->disconnect.reason);
      this->conn_handle_ = 0xFFFF;
      this->gap_client_.conn_handle = 0xFFFF;
      this->svc_start_handle_ = 0;
      this->svc_end_handle_ = 0;
      this->char_handle_ = 0;
      this->notify_val_handle_ = 0;
      this->discovered_svc_count_ = 0;
      this->schedule_reconnect();
      return 0;

    case BLE_GAP_EVENT_NOTIFY_RX:
      if (event->notify_rx.om != nullptr) {
        this->on_notify_data(event->notify_rx.om->om_data, OS_MBUF_PKTLEN(event->notify_rx.om));
      }
      return 0;

    case BLE_GAP_EVENT_PASSKEY_ACTION:
      if (this->passkey_ == 0)
        return 0;
      {
        struct ble_sm_io pkey{};
        pkey.action = event->passkey.params.action;
        if (pkey.action == BLE_SM_IOACT_INPUT || pkey.action == BLE_SM_IOACT_DISP) {
          pkey.passkey = this->passkey_;
          ble_sm_inject_io(event->passkey.conn_handle, &pkey);
          ESP_LOGI(TAG, "Injected passkey %u", this->passkey_);
        }
      }
      return 0;

    default:
      return 0;
  }
}

void NimbleSunster::on_scan_timeout_() { this->schedule_reconnect(); }

void NimbleSunster::enable_notifications() {
  if (this->conn_handle_ == 0xFFFF || this->notify_val_handle_ == 0)
    return;

  uint8_t cccd[2] = {1, 0};
  int rc = ble_gattc_write_flat(this->conn_handle_, this->notify_val_handle_ + 1, cccd, sizeof(cccd), nullptr, nullptr);
  if (rc != 0) {
    ESP_LOGE(TAG, "CCCD write failed: %d", rc);
    return;
  }

  ESP_LOGI(TAG, "Enabled Sunster heater notifications");
}

void NimbleSunster::start_connect() {
  if (!this->host_->is_active() || !this->host_->is_synced())
    return;
  if (this->conn_handle_ != 0xFFFF)
    return;
  if (this->connect_attempted_)
    return;
  if (global_nimble_gap == nullptr || !this->gap_registered_)
    return;

  this->connect_attempted_ = true;
  global_nimble_gap->request_connect_scan(&this->gap_client_);
}

void NimbleSunster::schedule_reconnect() {
  if (global_nimble_gap != nullptr) {
    global_nimble_gap->cancel_connect_scan(&this->gap_client_);
  }
  this->connect_attempted_ = false;
  if (this->conn_handle_ != 0xFFFF) {
    ESP_LOGI(TAG, "Disconnecting to retry GATT setup");
    ble_gap_terminate(this->conn_handle_, BLE_ERR_REM_USER_CONN_TERM);
    return;
  }
  this->reconnect_at_ms_ = millis() + 5000;
}

void NimbleSunster::write(const uint8_t *data, size_t len) {
  if (this->conn_handle_ == 0xFFFF) {
    ESP_LOGD(TAG, "Cannot write, not connected");
    return;
  }
  if (this->char_handle_ == 0) {
    ESP_LOGD(TAG, "Cannot write, GATT not ready");
    return;
  }
  int rc = ble_gattc_write_flat(this->conn_handle_, this->char_handle_, data, len, nullptr, nullptr);
  if (rc != 0) {
    ESP_LOGW(TAG, "Write failed: %d", rc);
  }
}

void NimbleSunster::on_notify_data(const uint8_t *data, size_t len) {
  sunster_parser_handle_notify(data, len);
}

}  // namespace esphome::nimble_sunster
