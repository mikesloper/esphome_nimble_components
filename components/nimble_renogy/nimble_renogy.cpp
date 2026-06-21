#include "nimble_renogy.h"
#include "renogy_parser.h"
#include "esphome/components/nimble_host/nimble_host.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"
#include "renogy_rover_utilities.h"

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

namespace esphome::nimble_renogy {

static const char *const TAG = "nimble_renogy";

NimbleRenogy *g_renogy = nullptr;

static bool uuid_matches_exact_16_(const NimbleRenogy *self, const ble_uuid_any_t *discovered,
                                   const std::string &target_str) {
  if (discovered->u.type != BLE_UUID_TYPE_16)
    return false;
  uint16_t target16 = self->target_uuid16_(target_str);
  return target16 != 0 && BLE_UUID16(&discovered->u)->value == target16;
}

static bool uuid_matches_128_alias_(const NimbleRenogy *self, const ble_uuid_any_t *discovered,
                                    const std::string &target_str) {
  if (discovered->u.type != BLE_UUID_TYPE_128)
    return false;
  uint16_t target16 = self->target_uuid16_(target_str);
  if (target16 == 0)
    return false;
  const uint8_t *v = BLE_UUID128(&discovered->u)->value;
  uint16_t disc16 = (uint16_t) v[12] | ((uint16_t) v[13] << 8);
  return disc16 == target16;
}

static void log_uuid_(const char *label, const ble_uuid_t *uuid) {
  char uuid_buf[BLE_UUID_STR_LEN];
  ble_uuid_to_str(uuid, uuid_buf);
  ESP_LOGI(TAG, "%s: %s", label, uuid_buf);
}

uint16_t NimbleRenogy::target_uuid16_(const std::string &uuid_str) const {
  std::string s = uuid_str;
  s.erase(std::remove(s.begin(), s.end(), '-'), s.end());
  if (s.length() <= 4)
    return static_cast<uint16_t>(std::stoul(s, nullptr, 16));
  if (s.length() == 32)
    return static_cast<uint16_t>(std::stoul(s.substr(4, 4), nullptr, 16));
  return 0;
}

bool NimbleRenogy::parse_uuid_(const std::string &uuid_str, void *out_uuid) const {
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

float NimbleRenogy::get_setup_priority() const { return setup_priority::BUS + 1.0f; }

void NimbleRenogy::setup() {
  g_renogy = this;
  if (this->host_ == nullptr) {
    ESP_LOGE(TAG, "NimBLE host not set");
    this->mark_failed();
    return;
  }

  renogy_parser_set_entities(
      this->charging_status_text_sensor_, this->battery_voltage_sensor_, this->battery_current_sensor_,
      this->battery_temperature_sensor_, this->pv_voltage_sensor_, this->pv_current_sensor_, this->pv_power_sensor_,
      this->load_voltage_sensor_, this->load_current_sensor_, this->total_current_sensor_);
  renogy_parser_set_device_model(this->device_model_);

  this->host_->add_on_sync_callback([this]() {
    if (this->auto_connect_ && this->host_->is_active()) {
      this->start_connect();
    }
  });
}

void NimbleRenogy::loop() {
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

static int disc_chr_cb(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_chr *chr,
                       void *arg);
static int disc_svc_cb(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_svc *svc,
                       void *arg);
static int mtu_cb(uint16_t conn_handle, const struct ble_gatt_error *error, uint16_t mtu, void *arg);
static int gap_event(struct ble_gap_event *event, void *arg);

static bool peer_mac_matches_(uint64_t address, const uint8_t *addr_val) {
  for (int i = 0; i < 6; i++) {
    uint8_t expected = (address >> (i * 8)) & 0xFF;
    if (addr_val[i] != expected) {
      return false;
    }
  }
  return true;
}

static bool is_connectable_adv_(uint8_t event_type) {
  return event_type == BLE_HCI_ADV_RPT_EVTYPE_ADV_IND || event_type == BLE_HCI_ADV_RPT_EVTYPE_DIR_IND ||
         event_type == BLE_HCI_ADV_RPT_EVTYPE_SCAN_RSP || event_type == BLE_HCI_ADV_RPT_EVTYPE_SCAN_IND;
}

static int disc_chr_cb(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_chr *chr,
                       void *arg) {
  auto *self = static_cast<NimbleRenogy *>(arg);
  if (error->status == BLE_HS_EDONE) {
    if (self->notify_val_handle_ != 0 && self->write_val_handle_ == 0 && self->write_svc_start_ != 0) {
      self->start_write_char_discovery_(conn_handle);
      return 0;
    }
    self->try_finish_gatt_setup_(conn_handle);
    return 0;
  }
  if (error->status != 0)
    return error->status;
  if (chr == nullptr)
    return 0;

  log_uuid_("Discovered GATT characteristic", &chr->uuid.u);

  if (uuid_matches_exact_16_(self, &chr->uuid, self->notify_uuid_)) {
    self->notify_val_handle_ = chr->val_handle;
    self->notify_char_is_16bit_ = true;
    ESP_LOGI(TAG, "Matched notify characteristic handle=%d (16-bit)", chr->val_handle);
  } else if (!self->notify_char_is_16bit_ && uuid_matches_128_alias_(self, &chr->uuid, self->notify_uuid_)) {
    self->notify_val_handle_ = chr->val_handle;
    ESP_LOGI(TAG, "Matched notify characteristic handle=%d (128-bit alias)", chr->val_handle);
  }
  if (uuid_matches_exact_16_(self, &chr->uuid, self->write_uuid_)) {
    self->write_val_handle_ = chr->val_handle;
    self->write_char_is_16bit_ = true;
    ESP_LOGI(TAG, "Matched write characteristic handle=%d (16-bit)", chr->val_handle);
  } else if (!self->write_char_is_16bit_ && uuid_matches_128_alias_(self, &chr->uuid, self->write_uuid_)) {
    self->write_val_handle_ = chr->val_handle;
    ESP_LOGI(TAG, "Matched write characteristic handle=%d (128-bit alias)", chr->val_handle);
  }
  return 0;
}

void NimbleRenogy::start_notify_char_discovery_(uint16_t conn_handle) {
  if (this->notify_svc_start_ == 0)
    return;
  ESP_LOGI(TAG, "Discovering notify characteristics (handles %u-%u)", this->notify_svc_start_, this->notify_svc_end_);
  int rc = ble_gattc_disc_all_chrs(conn_handle, this->notify_svc_start_, this->notify_svc_end_, disc_chr_cb, this);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_gattc_disc_all_chrs (notify) failed: %d", rc);
    this->schedule_reconnect();
  }
}

void NimbleRenogy::start_write_char_discovery_(uint16_t conn_handle) {
  if (this->write_svc_start_ == 0)
    return;
  ESP_LOGI(TAG, "Discovering write characteristics (handles %u-%u)", this->write_svc_start_, this->write_svc_end_);
  int rc = ble_gattc_disc_all_chrs(conn_handle, this->write_svc_start_, this->write_svc_end_, disc_chr_cb, this);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_gattc_disc_all_chrs (write) failed: %d", rc);
    this->schedule_reconnect();
  }
}

void NimbleRenogy::try_finish_gatt_setup_(uint16_t conn_handle) {
  if (this->notify_val_handle_ != 0 && this->write_val_handle_ != 0) {
    this->enable_notifications();
    return;
  }
  ESP_LOGE(TAG, "Required characteristics not found (notify=%u write=%u)", this->notify_val_handle_,
           this->write_val_handle_);
  this->schedule_reconnect();
}

static int disc_svc_cb(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_svc *svc,
                       void *arg) {
  auto *self = static_cast<NimbleRenogy *>(arg);
  if (error->status == BLE_HS_EDONE) {
    ESP_LOGI(TAG, "GATT service discovery finished, saw %u service(s)", self->discovered_svc_count_);
    if (self->notify_svc_start_ != 0) {
      self->start_notify_char_discovery_(conn_handle);
    } else if (self->write_svc_start_ != 0) {
      self->start_write_char_discovery_(conn_handle);
    } else {
      ESP_LOGE(TAG, "Renogy notify/write services not found");
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

  if (uuid_matches_exact_16_(self, &svc->uuid, self->notify_service_uuid_)) {
    self->notify_svc_start_ = svc->start_handle;
    self->notify_svc_end_ = svc->end_handle;
    self->notify_svc_is_16bit_ = true;
    ESP_LOGI(TAG, "Matched notify service handles=%u-%u (16-bit)", svc->start_handle, svc->end_handle);
  } else if (!self->notify_svc_is_16bit_ && uuid_matches_128_alias_(self, &svc->uuid, self->notify_service_uuid_)) {
    self->notify_svc_start_ = svc->start_handle;
    self->notify_svc_end_ = svc->end_handle;
    ESP_LOGI(TAG, "Matched notify service handles=%u-%u (128-bit alias)", svc->start_handle, svc->end_handle);
  }
  if (uuid_matches_exact_16_(self, &svc->uuid, self->write_service_uuid_)) {
    self->write_svc_start_ = svc->start_handle;
    self->write_svc_end_ = svc->end_handle;
    self->write_svc_is_16bit_ = true;
    ESP_LOGI(TAG, "Matched write service handles=%u-%u (16-bit)", svc->start_handle, svc->end_handle);
  } else if (!self->write_svc_is_16bit_ && uuid_matches_128_alias_(self, &svc->uuid, self->write_service_uuid_)) {
    self->write_svc_start_ = svc->start_handle;
    self->write_svc_end_ = svc->end_handle;
    ESP_LOGI(TAG, "Matched write service handles=%u-%u (128-bit alias)", svc->start_handle, svc->end_handle);
  }
  return 0;
}

void NimbleRenogy::discover_services_() {
  this->discovered_svc_count_ = 0;
  this->notify_svc_start_ = 0;
  this->notify_svc_end_ = 0;
  this->write_svc_start_ = 0;
  this->write_svc_end_ = 0;
  this->notify_val_handle_ = 0;
  this->write_val_handle_ = 0;
  this->notify_svc_is_16bit_ = false;
  this->write_svc_is_16bit_ = false;
  this->notify_char_is_16bit_ = false;
  this->write_char_is_16bit_ = false;
  this->gatt_ready_ = false;

  int rc = ble_gattc_disc_all_svcs(this->conn_handle_, disc_svc_cb, this);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_gattc_disc_all_svcs failed: %d", rc);
    this->schedule_reconnect();
  }
}

static int mtu_cb(uint16_t conn_handle, const struct ble_gatt_error *error, uint16_t mtu, void *arg) {
  auto *self = static_cast<NimbleRenogy *>(arg);
  if (error->status != 0) {
    ESP_LOGW(TAG, "MTU exchange status=%d", error->status);
  } else {
    ESP_LOGI(TAG, "MTU negotiated: %u", mtu);
  }
  self->discover_services_();
  return 0;
}

static int gap_event(struct ble_gap_event *event, void *arg) {
  auto *self = static_cast<NimbleRenogy *>(arg);

  switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
      const auto &disc = event->disc;
      self->increment_scan_adv_count();

      char mac_buf[MAC_ADDRESS_PRETTY_BUFFER_SIZE];
      format_mac_addr_upper(disc.addr.val, mac_buf);
      const bool mac_match = peer_mac_matches_(self->get_address(), disc.addr.val);

      if (self->get_scan_adv_count() <= 5 || mac_match) {
        ESP_LOGI(TAG, "BLE adv #%u: %s type=%d event=%d%s", self->get_scan_adv_count(), mac_buf, disc.addr.type,
                 disc.event_type, mac_match ? " (target)" : "");
      }

      if (!mac_match || !is_connectable_adv_(disc.event_type))
        return 0;

      ESP_LOGI(TAG, "Found Renogy device in scan, connecting...");

      int rc = ble_gap_disc_cancel();
      if (rc != 0) {
        ESP_LOGW(TAG, "ble_gap_disc_cancel failed: %d", rc);
        return 0;
      }
      self->set_scanning(false);

      struct ble_gap_conn_params params{};
      params.scan_itvl = 0x0010;
      params.scan_window = 0x0010;
      params.itvl_min = BLE_GAP_INITIAL_CONN_ITVL_MIN;
      params.itvl_max = BLE_GAP_INITIAL_CONN_ITVL_MAX;
      params.latency = 0;
      params.supervision_timeout = 0x0100;
      params.min_ce_len = 0x0010;
      params.max_ce_len = 0x0300;

      rc = ble_gap_connect(self->get_own_addr_type(), &disc.addr, 30000, &params, gap_event, self);
      if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_connect failed: %d", rc);
        self->schedule_reconnect();
      } else {
        ESP_LOGI(TAG, "Connecting to Renogy device...");
      }
      return 0;
    }

    case BLE_GAP_EVENT_DISC_COMPLETE:
      self->set_scanning(false);
      if (self->conn_handle_ == 0xFFFF && event->disc_complete.reason != BLE_HS_EPREEMPTED) {
        ESP_LOGW(TAG, "Scan finished, saw %u BLE advertisers, Renogy not found (reason=%d)",
                 self->get_scan_adv_count(), event->disc_complete.reason);
        self->reset_scan_adv_count();
        self->schedule_reconnect();
      }
      return 0;

    case BLE_GAP_EVENT_CONNECT:
      if (event->connect.status == 0) {
        self->conn_handle_ = event->connect.conn_handle;
        self->notify_svc_start_ = 0;
        self->notify_svc_end_ = 0;
        self->write_svc_start_ = 0;
        self->write_svc_end_ = 0;
        self->notify_val_handle_ = 0;
        self->write_val_handle_ = 0;
        self->notify_svc_is_16bit_ = false;
        self->write_svc_is_16bit_ = false;
        self->notify_char_is_16bit_ = false;
        self->write_char_is_16bit_ = false;
        self->discovered_svc_count_ = 0;
        self->gatt_ready_ = false;
        ESP_LOGI(TAG, "Connected, conn_handle=%d", self->conn_handle_);
        int rc = ble_gattc_exchange_mtu(self->conn_handle_, mtu_cb, self);
        if (rc != 0) {
          ESP_LOGW(TAG, "ble_gattc_exchange_mtu failed: %d, discovering services anyway", rc);
          self->discover_services_();
        }
      } else {
        ESP_LOGW(TAG, "Connection failed, status=%d", event->connect.status);
        self->schedule_reconnect();
      }
      return 0;

    case BLE_GAP_EVENT_DISCONNECT:
      ESP_LOGW(TAG, "Disconnected, reason=%d", event->disconnect.reason);
      self->conn_handle_ = 0xFFFF;
      self->notify_svc_start_ = 0;
      self->notify_svc_end_ = 0;
      self->write_svc_start_ = 0;
      self->write_svc_end_ = 0;
      self->notify_val_handle_ = 0;
      self->write_val_handle_ = 0;
      self->notify_svc_is_16bit_ = false;
      self->write_svc_is_16bit_ = false;
      self->notify_char_is_16bit_ = false;
      self->write_char_is_16bit_ = false;
      self->discovered_svc_count_ = 0;
      self->gatt_ready_ = false;
      self->schedule_reconnect();
      return 0;

    case BLE_GAP_EVENT_NOTIFY_RX:
      if (g_renogy != nullptr && event->notify_rx.om != nullptr) {
        g_renogy->on_notify_data(event->notify_rx.om->om_data, OS_MBUF_PKTLEN(event->notify_rx.om));
      }
      return 0;

    case BLE_GAP_EVENT_PASSKEY_ACTION:
      if (self->passkey_ == 0)
        return 0;
      {
        struct ble_sm_io pkey{};
        pkey.action = event->passkey.params.action;
        if (pkey.action == BLE_SM_IOACT_INPUT || pkey.action == BLE_SM_IOACT_DISP) {
          pkey.passkey = self->passkey_;
          ble_sm_inject_io(event->passkey.conn_handle, &pkey);
          ESP_LOGI(TAG, "Injected passkey %u", self->passkey_);
        }
      }
      return 0;

    default:
      return 0;
  }
}

static int cccd_write_cb(uint16_t conn_handle, const struct ble_gatt_error *error, struct ble_gatt_attr *attr,
                         void *arg) {
  auto *self = static_cast<NimbleRenogy *>(arg);
  if (error->status != 0) {
    ESP_LOGE(TAG, "CCCD write failed: status=%d", error->status);
    self->schedule_reconnect();
    return 0;
  }
  self->gatt_ready_ = true;
  ESP_LOGI(TAG, "Enabled Renogy notifications (notify=%u write=%u)", self->notify_val_handle_,
           self->write_val_handle_);
  self->send_poll_request_();
  return 0;
}

void NimbleRenogy::enable_notifications() {
  if (this->conn_handle_ == 0xFFFF || this->notify_val_handle_ == 0)
    return;

  uint8_t cccd[2] = {1, 0};
  int rc = ble_gattc_write_flat(this->conn_handle_, this->notify_val_handle_ + 1, cccd, sizeof(cccd), cccd_write_cb,
                                this);
  if (rc != 0) {
    ESP_LOGE(TAG, "CCCD write failed: %d", rc);
    this->schedule_reconnect();
  }
}

void NimbleRenogy::start_connect() {
  if (!this->host_->is_active() || !this->host_->is_synced())
    return;
  if (this->conn_handle_ != 0xFFFF)
    return;
  if (this->connect_attempted_)
    return;

  int rc = ble_hs_id_infer_auto(0, &this->own_addr_type_);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_hs_id_infer_auto failed: %d", rc);
    return;
  }

  if (this->is_scanning()) {
    ble_gap_disc_cancel();
    this->set_scanning(false);
  }

  struct ble_gap_disc_params disc_params{};
  disc_params.filter_duplicates = 1;
  disc_params.passive = 0;
  disc_params.itvl = 0;
  disc_params.window = 0;

  this->connect_attempted_ = true;
  this->set_scanning(true);
  this->reset_scan_adv_count();

  rc = ble_gap_disc(this->own_addr_type_, 30000, &disc_params, gap_event, this);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_gap_disc failed: %d", rc);
    this->connect_attempted_ = false;
    this->set_scanning(false);
    this->schedule_reconnect();
  } else {
    ESP_LOGI(TAG, "Scanning for Renogy device...");
  }
}

void NimbleRenogy::schedule_reconnect() {
  if (this->is_scanning()) {
    ble_gap_disc_cancel();
    this->set_scanning(false);
  }
  this->connect_attempted_ = false;
  this->gatt_ready_ = false;
  if (this->conn_handle_ != 0xFFFF) {
    ESP_LOGI(TAG, "Disconnecting to retry GATT setup");
    ble_gap_terminate(this->conn_handle_, BLE_ERR_REM_USER_CONN_TERM);
    return;
  }
  this->reconnect_at_ms_ = millis() + 5000;
}

void NimbleRenogy::update() {
  if (!this->gatt_ready_) {
    ESP_LOGV(TAG, "Not ready for polling");
    return;
  }
  this->send_poll_request_();
}

bool NimbleRenogy::send_poll_request_() {
  if (!this->gatt_ready_ || this->write_val_handle_ == 0)
    return false;

  auto request = GetRoverRequest();
  ESP_LOGD(TAG, "Sending Renogy poll request (%zu bytes) to handle %u", request.size(), this->write_val_handle_);
  int rc = ble_gattc_write_flat(this->conn_handle_, this->write_val_handle_, request.data(), request.size(), nullptr,
                                nullptr);
  if (rc != 0) {
    ESP_LOGW(TAG, "Poll write failed: %d", rc);
    return false;
  }
  return true;
}

void NimbleRenogy::on_notify_data(const uint8_t *data, size_t len) {
  ESP_LOGD(TAG, "Renogy notify: %zu bytes", len);
  renogy_parser_handle_notify(data, len);
}

void NimbleRenogy::dump_config() {
  uint8_t mac[6];
  for (int i = 0; i < 6; i++) {
    mac[i] = (this->address_ >> (i * 8)) & 0xFF;
  }
  char mac_buf[MAC_ADDRESS_PRETTY_BUFFER_SIZE];
  format_mac_addr_upper(mac, mac_buf);

  ESP_LOGCONFIG(TAG, "NimBLE Renogy:");
  ESP_LOGCONFIG(TAG, "  MAC: %s", mac_buf);
  ESP_LOGCONFIG(TAG, "  Notify service: %s char: %s", this->notify_service_uuid_.c_str(), this->notify_uuid_.c_str());
  ESP_LOGCONFIG(TAG, "  Write service: %s char: %s", this->write_service_uuid_.c_str(), this->write_uuid_.c_str());
  ESP_LOGCONFIG(TAG, "  Device model: %s", this->device_model_.c_str());
  ESP_LOGCONFIG(TAG, "  Connected: %s", YESNO(this->is_connected()));
  ESP_LOGCONFIG(TAG, "  GATT ready: %s", YESNO(this->gatt_ready_));
}

}  // namespace esphome::nimble_renogy
