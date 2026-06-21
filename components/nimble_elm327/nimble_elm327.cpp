#include "nimble_elm327.h"
#include "elm327_parser.h"
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

static const char *const TAG = "nimble_elm327";

NimbleElm327 *g_elm327 = nullptr;

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

// Run before nimble_host (BUS) so on_sync callbacks are registered before the stack syncs.
float NimbleElm327::get_setup_priority() const { return setup_priority::BUS + 1.0f; }

void NimbleElm327::setup() {
  g_elm327 = this;
  if (this->host_ == nullptr) {
    ESP_LOGE(TAG, "NimBLE host not set");
    this->mark_failed();
    return;
  }

  elm327_parser_set_sensors(this->rpm_sensor_, this->kph_sensor_, this->coolant_sensor_, this->voltage_sensor_,
                            this->uptime_sensor_);

  this->host_->add_on_sync_callback([this]() {
    if (this->auto_connect_ && this->host_->is_active()) {
      this->start_connect();
    }
  });
}

void NimbleElm327::loop() {
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
static int gap_event(struct ble_gap_event *event, void *arg);

static bool peer_mac_matches_(uint64_t address, const uint8_t *addr_val) {
  // ESPHome stores MAC as uint64 MSB-first; NimBLE ble_addr_t.val is LSB-first.
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

static int gap_event(struct ble_gap_event *event, void *arg) {
  auto *self = static_cast<NimbleElm327 *>(arg);

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

      if (!mac_match || !is_connectable_adv_(disc.event_type)) {
        return 0;
      }

      ESP_LOGI(TAG, "Found ELM327 in scan, connecting...");

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
        ESP_LOGI(TAG, "Connecting to ELM327...");
      }
      return 0;
    }

    case BLE_GAP_EVENT_DISC_COMPLETE:
      self->set_scanning(false);
      if (self->conn_handle_ == 0xFFFF && event->disc_complete.reason != BLE_HS_EPREEMPTED) {
        ESP_LOGW(TAG, "Scan finished, saw %u BLE advertisers, ELM327 not found (reason=%d)",
                 self->get_scan_adv_count(), event->disc_complete.reason);
        self->reset_scan_adv_count();
        self->schedule_reconnect();
      }
      return 0;

    case BLE_GAP_EVENT_CONNECT:
      if (event->connect.status == 0) {
        self->conn_handle_ = event->connect.conn_handle;
        self->svc_start_handle_ = 0;
        self->svc_end_handle_ = 0;
        self->write_handle_ = 0;
        self->notify_val_handle_ = 0;
        ESP_LOGI(TAG, "Connected, conn_handle=%d", self->conn_handle_);
        int rc = ble_gattc_disc_all_svcs(self->conn_handle_, disc_svc_cb, self);
        if (rc != 0) {
          ESP_LOGE(TAG, "ble_gattc_disc_all_svcs failed: %d", rc);
          self->schedule_reconnect();
        }
      } else {
        const char *reason = event->connect.status == BLE_HS_ETIMEOUT ? "timeout" : "error";
        ESP_LOGW(TAG, "Connection failed, status=%d (%s)", event->connect.status, reason);
        self->schedule_reconnect();
      }
      return 0;

    case BLE_GAP_EVENT_DISCONNECT:
      ESP_LOGW(TAG, "Disconnected, reason=%d", event->disconnect.reason);
      self->conn_handle_ = 0xFFFF;
      self->svc_start_handle_ = 0;
      self->svc_end_handle_ = 0;
      self->write_handle_ = 0;
      self->notify_val_handle_ = 0;
      self->schedule_reconnect();
      return 0;

    case BLE_GAP_EVENT_NOTIFY_RX:
      if (g_elm327 != nullptr && event->notify_rx.om != nullptr) {
        g_elm327->on_notify_data(event->notify_rx.om->om_data, OS_MBUF_PKTLEN(event->notify_rx.om));
      }
      return 0;

    case BLE_GAP_EVENT_PASSKEY_ACTION: {
      struct ble_sm_io pkey{};
      pkey.action = event->passkey.params.action;
      if (pkey.action == BLE_SM_IOACT_INPUT || pkey.action == BLE_SM_IOACT_DISP) {
        pkey.passkey = self->passkey_;
        ble_sm_inject_io(event->passkey.conn_handle, &pkey);
        ESP_LOGI(TAG, "Injected passkey %u", self->passkey_);
      }
      return 0;
    }

    default:
      return 0;
  }
}

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
    ESP_LOGI(TAG, "Scanning for ELM327...");
  }
}

void NimbleElm327::schedule_reconnect() {
  if (this->is_scanning()) {
    ble_gap_disc_cancel();
    this->set_scanning(false);
  }
  this->connect_attempted_ = false;
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
