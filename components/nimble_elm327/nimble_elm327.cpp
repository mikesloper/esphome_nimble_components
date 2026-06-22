#include "nimble_elm327.h"
#include "elm327_parser.h"
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
#include <map>
#include <string>

namespace esphome::nimble_elm327 {

using esphome::nimble_gap::global_nimble_gap;

static const char *const TAG = "nimble_elm327";

static int gap_event_trampoline_(struct ble_gap_event *event, void *context) {
  return static_cast<NimbleElm327 *>(context)->handle_gap_event_(event);
}

static void scan_timeout_trampoline_(void *context) {
  static_cast<NimbleElm327 *>(context)->on_scan_timeout_();
}

static bool uuid_matches_(const ble_uuid_any_t *uuid, const ble_uuid_any_t *target) {
  return ble_uuid_cmp(&uuid->u, &target->u) == 0;
}

bool NimbleElm327::parse_uuid_(const std::string &uuid_str, void *out_uuid) const {
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

void NimbleElm327::ensure_gap_registered_() {
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

// Run before nimble_host (BUS) so on_sync callbacks are registered before the stack syncs.
float NimbleElm327::get_setup_priority() const { return setup_priority::BUS + 1.0f; }

void NimbleElm327::setup() {
  if (this->host_ == nullptr) {
    ESP_LOGE(TAG, "NimBLE host not set");
    this->mark_failed();
    return;
  }

  this->ensure_gap_registered_();

  elm327_parser_set_sensors(this->rpm_sensor_, this->kph_sensor_, this->coolant_sensor_, this->voltage_sensor_,
                            this->uptime_sensor_);

  this->host_->add_on_sync_callback([this]() {
    if (this->auto_connect_ && this->host_->is_active()) {
      this->start_connect();
    }
  });
}

void NimbleElm327::loop() {
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

void NimbleElm327::dump_config() {
  uint8_t mac[6];
  for (int i = 0; i < 6; i++) {
    mac[i] = (this->address_ >> ((5 - i) * 8)) & 0xFF;
  }
  char mac_buf[MAC_ADDRESS_PRETTY_BUFFER_SIZE];
  format_mac_addr_upper(mac, mac_buf);

  ESP_LOGCONFIG(TAG, "NimBLE ELM327:");
  ESP_LOGCONFIG(TAG, "  MAC: %s", mac_buf);
  ESP_LOGCONFIG(TAG, "  Connected: %s", YESNO(this->is_connected()));
}


static int disc_chr_cb(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_chr *chr,
                       void *arg);
static int disc_svc_cb(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_svc *svc,
                       void *arg);

static int disc_chr_cb(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_chr *chr,
                       void *arg) {
  auto *self = static_cast<NimbleElm327 *>(arg);
  if (error->status == BLE_HS_EDONE) {
    if (self->notify_val_handle_ != 0 && self->write_handle_ != 0) {
      self->enable_notifications();
    } else {
      ESP_LOGE(TAG, "Required characteristics not found");
      self->schedule_reconnect();
    }
    return 0;
  }
  if (error->status != 0)
    return error->status;

  ble_uuid_any_t notify_uuid{};
  ble_uuid_any_t write_uuid{};
  if (!self->parse_uuid_(self->notify_uuid_, &notify_uuid) || !self->parse_uuid_(self->write_uuid_, &write_uuid))
    return 0;

  if (uuid_matches_(&chr->uuid, &notify_uuid)) {
    self->notify_val_handle_ = chr->val_handle;
    char uuid_buf[BLE_UUID_STR_LEN];
    ble_uuid_to_str(&chr->uuid.u, uuid_buf);
    ESP_LOGI(TAG, "Found notify characteristic %s handle=%d", uuid_buf, chr->val_handle);
  }
  if (uuid_matches_(&chr->uuid, &write_uuid)) {
    self->write_handle_ = chr->val_handle;
    char uuid_buf[BLE_UUID_STR_LEN];
    ble_uuid_to_str(&chr->uuid.u, uuid_buf);
    ESP_LOGI(TAG, "Found write characteristic %s handle=%d", uuid_buf, chr->val_handle);
  }
  return 0;
}

static int disc_svc_cb(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_svc *svc,
                       void *arg) {
  auto *self = static_cast<NimbleElm327 *>(arg);
  if (error->status == BLE_HS_EDONE) {
    if (self->svc_start_handle_ != 0) {
      ESP_LOGI(TAG, "Service discovery complete, discovering characteristics (handles %u-%u)",
               self->svc_start_handle_, self->svc_end_handle_);
      int rc = ble_gattc_disc_all_chrs(conn_handle, self->svc_start_handle_, self->svc_end_handle_, disc_chr_cb, arg);
      if (rc != 0) {
        ESP_LOGE(TAG, "ble_gattc_disc_all_chrs failed: %d", rc);
        self->schedule_reconnect();
      }
    } else {
      ESP_LOGE(TAG, "Service %s not found", self->service_uuid_.c_str());
      self->schedule_reconnect();
    }
    return 0;
  }
  if (error->status != 0)
    return error->status;

  ble_uuid_any_t service_uuid{};
  if (!self->parse_uuid_(self->service_uuid_, &service_uuid))
    return 0;

  if (!uuid_matches_(&svc->uuid, &service_uuid))
    return 0;

  self->svc_start_handle_ = svc->start_handle;
  self->svc_end_handle_ = svc->end_handle;
  char uuid_buf[BLE_UUID_STR_LEN];
  ble_uuid_to_str(&svc->uuid.u, uuid_buf);
  ESP_LOGI(TAG, "Found service %s handles=%u-%u", uuid_buf, svc->start_handle, svc->end_handle);
  return 0;
}

int NimbleElm327::handle_gap_event_(struct ble_gap_event *event) {
  switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
      if (event->connect.status == 0) {
        this->conn_handle_ = event->connect.conn_handle;
        this->gap_client_.conn_handle = this->conn_handle_;
        this->svc_start_handle_ = 0;
        this->svc_end_handle_ = 0;
        this->write_handle_ = 0;
        this->notify_val_handle_ = 0;
        ESP_LOGI(TAG, "Connected, conn_handle=%d", this->conn_handle_);
        int rc = ble_gattc_disc_all_svcs(this->conn_handle_, disc_svc_cb, this);
        if (rc != 0) {
          ESP_LOGE(TAG, "ble_gattc_disc_all_svcs failed: %d", rc);
          this->schedule_reconnect();
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
      this->write_handle_ = 0;
      this->notify_val_handle_ = 0;
      this->schedule_reconnect();
      return 0;

    case BLE_GAP_EVENT_NOTIFY_RX:
      if (event->notify_rx.om != nullptr) {
        this->on_notify_data(event->notify_rx.om->om_data, OS_MBUF_PKTLEN(event->notify_rx.om));
      }
      return 0;

    case BLE_GAP_EVENT_PASSKEY_ACTION: {
      struct ble_sm_io pkey{};
      pkey.action = event->passkey.params.action;
      if (pkey.action == BLE_SM_IOACT_INPUT || pkey.action == BLE_SM_IOACT_DISP) {
        pkey.passkey = this->passkey_;
        ble_sm_inject_io(event->passkey.conn_handle, &pkey);
        ESP_LOGI(TAG, "Injected passkey %u", this->passkey_);
      }
      return 0;
    }

    default:
      return 0;
  }
}

void NimbleElm327::on_scan_timeout_() { this->schedule_reconnect(); }

void NimbleElm327::enable_notifications() {
  if (this->conn_handle_ == 0xFFFF || this->notify_val_handle_ == 0)
    return;

  uint8_t cccd[2] = {1, 0};
  int rc = ble_gattc_write_flat(this->conn_handle_, this->notify_val_handle_ + 1, cccd, sizeof(cccd), nullptr, nullptr);
  if (rc != 0) {
    ESP_LOGE(TAG, "CCCD write failed: %d", rc);
    return;
  }

  init_elm327_pid_status();
  ESP_LOGI(TAG, "Enabled ELM327 notifications");
}

void NimbleElm327::start_connect() {
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

void NimbleElm327::schedule_reconnect() {
  if (global_nimble_gap != nullptr) {
    global_nimble_gap->cancel_connect_scan(&this->gap_client_);
  }
  this->connect_attempted_ = false;
  if (this->conn_handle_ != 0xFFFF) {
    ble_gap_terminate(this->conn_handle_, BLE_ERR_REM_USER_CONN_TERM);
    return;
  }
  this->reconnect_at_ms_ = millis() + 5000;
}

void NimbleElm327::write(const uint8_t *data, size_t len) {
  if (this->conn_handle_ == 0xFFFF || this->write_handle_ == 0) {
    ESP_LOGD(TAG, "Cannot write, not connected");
    return;
  }
  int rc = ble_gattc_write_flat(this->conn_handle_, this->write_handle_, data, len, nullptr, nullptr);
  if (rc != 0) {
    ESP_LOGW(TAG, "Write failed: %d", rc);
  }
}

void NimbleElm327::on_notify_data(const uint8_t *data, size_t len) {
  std::string payload(reinterpret_cast<const char *>(data), len);
  handle_eml_reponse(payload);
}

}  // namespace esphome::nimble_elm327
