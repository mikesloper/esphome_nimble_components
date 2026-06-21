#include "nimble_jbd_bms.h"
#include "esphome/components/nimble_host/nimble_host.h"
#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
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
#include <cmath>
#include <cstring>
#include <string>

namespace esphome::nimble_jbd_bms {

static const char *const TAG = "nimble_jbd_bms";

NimbleJbdBms *g_jbd_bms = nullptr;

static const uint8_t MAX_NO_RESPONSE_COUNT = 10;
static const uint16_t MAX_RESPONSE_SIZE = 128;

static const uint8_t JBD_PKT_START = 0xDD;
static const uint8_t JBD_PKT_END = 0x77;
static const uint8_t JBD_CMD_READ = 0xA5;
static const uint8_t JBD_CMD_WRITE = 0x5A;

static const uint8_t JBD_AUTH_PKT_START = 0xFF;
static const uint8_t JBD_AUTH_PKT_SECOND = 0xAA;

static const uint8_t JBD_AUTH_SEND_APP_KEY = 0x15;
static const uint8_t JBD_AUTH_GET_RANDOM = 0x17;
static const uint8_t JBD_AUTH_SEND_PASSWORD = 0x18;
static const uint8_t JBD_AUTH_SEND_ROOT_PASSWORD = 0x1D;

static const uint8_t JBD_CMD_HWINFO = 0x03;
static const uint8_t JBD_CMD_CELLINFO = 0x04;
static const uint8_t JBD_CMD_MOS = 0xE1;

static const uint8_t JBD_MOS_CHARGE = 0x01;
static const uint8_t JBD_MOS_DISCHARGE = 0x02;

static const uint8_t ERRORS_SIZE = 16;
static constexpr const char *const ERRORS[ERRORS_SIZE] = {
    "Cell overvoltage", "Cell undervoltage", "Pack overvoltage", "Pack undervoltage",
    "Charging over temperature", "Charging under temperature", "Discharging over temperature",
    "Discharging under temperature", "Charging overcurrent", "Discharging overcurrent", "Short circuit",
    "IC front-end error", "Mosfet Software Lock", "Charge timeout Close", "Unknown (0x0E)", "Unknown (0x0F)",
};

static const uint8_t OPERATION_STATUS_SIZE = 8;
static constexpr const char *const OPERATION_STATUS[OPERATION_STATUS_SIZE] = {
    "Charging", "Discharging", "Unknown (0x04)", "Unknown (0x08)",
    "Unknown (0x10)", "Unknown (0x20)", "Unknown (0x40)", "Unknown (0x80)",
};

static const uint8_t ROOT_PASSWORD[] = {0x4a, 0x42, 0x44, 0x62, 0x74, 0x70, 0x77, 0x64,
                                        0x21, 0x40, 0x23, 0x32, 0x30, 0x32, 0x33};

static bool uuid_matches_(const NimbleJbdBms *self, const ble_uuid_any_t *discovered, const std::string &target_str) {
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

uint16_t NimbleJbdBms::target_uuid16_(const std::string &uuid_str) const {
  std::string s = uuid_str;
  s.erase(std::remove(s.begin(), s.end(), '-'), s.end());
  if (s.length() <= 4)
    return static_cast<uint16_t>(std::stoul(s, nullptr, 16));
  if (s.length() == 32)
    return static_cast<uint16_t>(std::stoul(s.substr(4, 4), nullptr, 16));
  return 0;
}

bool NimbleJbdBms::parse_uuid_(const std::string &uuid_str, void *out_uuid) const {
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

float NimbleJbdBms::get_setup_priority() const { return setup_priority::BUS + 1.0f; }

void NimbleJbdBms::setup() {
  g_jbd_bms = this;
  if (this->host_ == nullptr) {
    ESP_LOGE(TAG, "NimBLE host not set");
    this->mark_failed();
    return;
  }

  this->host_->add_on_sync_callback([this]() {
    if (this->auto_connect_ && this->host_->is_active()) {
      this->start_connect();
    }
  });
}

void NimbleJbdBms::loop() {
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
  auto *self = static_cast<NimbleJbdBms *>(arg);
  if (error->status == BLE_HS_EDONE) {
    if (self->notify_val_handle_ != 0 && self->control_val_handle_ != 0) {
      self->enable_notifications();
    } else {
      ESP_LOGE(TAG, "Required characteristics not found (notify=%u control=%u)", self->notify_val_handle_,
               self->control_val_handle_);
      self->schedule_reconnect();
    }
    return 0;
  }
  if (error->status != 0)
    return error->status;
  if (chr == nullptr)
    return 0;

  log_uuid_("Discovered GATT characteristic", &chr->uuid.u);

  if (uuid_matches_(self, &chr->uuid, self->notify_uuid_)) {
    self->notify_val_handle_ = chr->val_handle;
    ESP_LOGI(TAG, "Matched notify characteristic handle=%d", chr->val_handle);
  }
  if (uuid_matches_(self, &chr->uuid, self->control_uuid_)) {
    self->control_val_handle_ = chr->val_handle;
    ESP_LOGI(TAG, "Matched control characteristic handle=%d", chr->val_handle);
  }
  return 0;
}

void NimbleJbdBms::start_char_discovery_(uint16_t conn_handle) {
  if (this->svc_start_handle_ == 0)
    return;

  int rc = ble_gattc_disc_all_chrs(conn_handle, this->svc_start_handle_, this->svc_end_handle_, disc_chr_cb, this);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_gattc_disc_all_chrs failed: %d", rc);
    this->schedule_reconnect();
  }
}

static int disc_svc_cb(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_svc *svc,
                       void *arg) {
  auto *self = static_cast<NimbleJbdBms *>(arg);
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
  ESP_LOGI(TAG, "Matched JBD service handles=%u-%u", svc->start_handle, svc->end_handle);
  return 0;
}

void NimbleJbdBms::discover_services_() {
  this->discovered_svc_count_ = 0;
  this->svc_start_handle_ = 0;
  this->svc_end_handle_ = 0;
  this->notify_val_handle_ = 0;
  this->control_val_handle_ = 0;
  this->gatt_ready_ = false;

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

  int rc = ble_gattc_disc_all_svcs(this->conn_handle_, disc_svc_cb, this);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_gattc_disc_all_svcs failed: %d", rc);
    this->schedule_reconnect();
  }
}

static int mtu_cb(uint16_t conn_handle, const struct ble_gatt_error *error, uint16_t mtu, void *arg) {
  auto *self = static_cast<NimbleJbdBms *>(arg);
  if (error->status != 0) {
    ESP_LOGW(TAG, "MTU exchange status=%d", error->status);
  } else {
    ESP_LOGI(TAG, "MTU negotiated: %u", mtu);
  }
  self->discover_services_();
  return 0;
}

static int gap_event(struct ble_gap_event *event, void *arg) {
  auto *self = static_cast<NimbleJbdBms *>(arg);

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

      ESP_LOGI(TAG, "Found JBD BMS in scan, connecting...");

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
        ESP_LOGI(TAG, "Connecting to JBD BMS...");
      }
      return 0;
    }

    case BLE_GAP_EVENT_DISC_COMPLETE:
      self->set_scanning(false);
      if (self->conn_handle_ == 0xFFFF && event->disc_complete.reason != BLE_HS_EPREEMPTED) {
        ESP_LOGW(TAG, "Scan finished, saw %u BLE advertisers, BMS not found (reason=%d)", self->get_scan_adv_count(),
                 event->disc_complete.reason);
        self->reset_scan_adv_count();
        self->schedule_reconnect();
      }
      return 0;

    case BLE_GAP_EVENT_CONNECT:
      if (event->connect.status == 0) {
        self->conn_handle_ = event->connect.conn_handle;
        self->svc_start_handle_ = 0;
        self->svc_end_handle_ = 0;
        self->notify_val_handle_ = 0;
        self->control_val_handle_ = 0;
        self->discovered_svc_count_ = 0;
        self->gatt_ready_ = false;
        self->frame_buffer_.clear();
        self->authentication_state_ = NimbleJbdBms::AuthState::NOT_AUTHENTICATED;
        self->random_byte_ = 0;

        struct ble_gap_conn_desc desc{};
        if (ble_gap_conn_find(event->connect.conn_handle, &desc) == 0) {
          memcpy(self->peer_addr_, desc.peer_id_addr.val, 6);
        }

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
      self->svc_start_handle_ = 0;
      self->svc_end_handle_ = 0;
      self->notify_val_handle_ = 0;
      self->control_val_handle_ = 0;
      self->discovered_svc_count_ = 0;
      self->gatt_ready_ = false;
      self->frame_buffer_.clear();
      self->authentication_state_ = NimbleJbdBms::AuthState::NOT_AUTHENTICATED;
      self->schedule_reconnect();
      return 0;

    case BLE_GAP_EVENT_NOTIFY_RX:
      if (g_jbd_bms != nullptr && event->notify_rx.om != nullptr) {
        g_jbd_bms->on_notify_data(event->notify_rx.om->om_data, OS_MBUF_PKTLEN(event->notify_rx.om));
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

void NimbleJbdBms::enable_notifications() {
  if (this->conn_handle_ == 0xFFFF || this->notify_val_handle_ == 0)
    return;

  uint8_t cccd[2] = {1, 0};
  int rc = ble_gattc_write_flat(this->conn_handle_, this->notify_val_handle_ + 1, cccd, sizeof(cccd), nullptr, nullptr);
  if (rc != 0) {
    ESP_LOGE(TAG, "CCCD write failed: %d", rc);
    this->schedule_reconnect();
    return;
  }

  this->gatt_ready_ = true;
  ESP_LOGI(TAG, "Enabled JBD BMS notifications");

  if (this->enable_authentication_) {
    this->start_authentication_();
  } else {
    this->send_command(JBD_CMD_READ, JBD_CMD_HWINFO);
  }
}

void NimbleJbdBms::start_connect() {
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
    ESP_LOGI(TAG, "Scanning for JBD BMS...");
  }
}

void NimbleJbdBms::schedule_reconnect() {
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

void NimbleJbdBms::update() {
  this->track_online_status_();
  if (!this->gatt_ready_) {
    ESP_LOGV(TAG, "Not ready for polling");
    return;
  }

  if (this->enable_authentication_ && this->authentication_state_ != AuthState::AUTHENTICATED) {
    if (this->authentication_state_ == AuthState::NOT_AUTHENTICATED) {
      this->start_authentication_();
    } else {
      this->check_auth_timeout_();
    }
    return;
  }

  this->send_command(JBD_CMD_READ, JBD_CMD_HWINFO);
}

void NimbleJbdBms::on_notify_data(const uint8_t *data, size_t len) {
  this->assemble_(data, static_cast<uint16_t>(len));
}

bool NimbleJbdBms::send_command(uint8_t command, uint8_t address, const uint8_t *data, uint8_t data_len) {
  if (!this->gatt_ready_ || this->control_val_handle_ == 0)
    return false;

  auto frame = this->build_frame_(command, address, data, data_len);
  ESP_LOGV(TAG, "Send command: %s", format_hex_pretty(frame.data(), frame.size()).c_str());
  int rc = ble_gattc_write_flat(this->conn_handle_, this->control_val_handle_, frame.data(), frame.size(), nullptr,
                                  nullptr);
  if (rc != 0) {
    ESP_LOGW(TAG, "Write failed: %d", rc);
    return false;
  }
  return true;
}

bool NimbleJbdBms::write_register(uint8_t address, uint16_t value) {
  uint8_t data[2] = {(uint8_t) (value >> 8), (uint8_t) value};
  return this->send_command(JBD_CMD_WRITE, address, data, 2);
}

bool NimbleJbdBms::change_mosfet_status(uint8_t address, uint8_t bitmask, bool state) {
  if (this->mosfet_status_ == 255) {
    ESP_LOGE(TAG, "Unable to change MOSFET status because it's unknown");
    return false;
  }

  uint16_t value = (this->mosfet_status_ & (~(1 << bitmask))) | ((uint8_t) state << bitmask);
  this->mosfet_status_ = value;
  value ^= (1 << 0);
  value ^= (1 << 1);
  return this->write_register(address, value);
}

void NimbleJbdBms::assemble_(const uint8_t *data, uint16_t length) {
  if (this->frame_buffer_.size() > MAX_RESPONSE_SIZE) {
    ESP_LOGW(TAG, "Maximum response size exceeded");
    this->frame_buffer_.clear();
  }

  if (length >= 5 && data[0] == JBD_AUTH_PKT_START && data[1] == JBD_AUTH_PKT_SECOND) {
    uint8_t command = data[2];
    uint8_t data_len = data[3];
    uint8_t expected_frame_len = 4 + data_len + 1;
    if (length >= expected_frame_len) {
      uint8_t computed_crc = this->auth_chksum_(data + 2, 2 + data_len);
      uint8_t remote_crc = data[4 + data_len];
      if (computed_crc == remote_crc) {
        this->handle_auth_response_(command, data + 4, data_len);
      } else {
        ESP_LOGW(TAG, "Auth frame checksum failed! 0x%02X != 0x%02X", computed_crc, remote_crc);
      }
    }
    return;
  }

  bool is_new_frame = false;
  if (length >= 3 && data[0] == JBD_PKT_START && data[2] == 0x00) {
    is_new_frame = true;
  }
  if (is_new_frame) {
    this->frame_buffer_.clear();
  }

  this->frame_buffer_.insert(this->frame_buffer_.end(), data, data + length);

  if (this->frame_buffer_.size() >= 7 && this->frame_buffer_[0] == JBD_PKT_START &&
      this->frame_buffer_.back() == JBD_PKT_END) {
    const uint8_t *raw = &this->frame_buffer_[0];
    uint8_t function = raw[1];
    uint16_t data_len = raw[3];
    uint16_t frame_len = 4 + data_len + 3;

    if (frame_len == this->frame_buffer_.size()) {
      uint16_t computed_crc = this->chksum_(raw + 2, data_len + 2);
      uint16_t remote_crc = (uint16_t(raw[frame_len - 3]) << 8) | uint16_t(raw[frame_len - 2]);
      if (computed_crc == remote_crc) {
        std::vector<uint8_t> frame_data(this->frame_buffer_.begin() + 4, this->frame_buffer_.end() - 3);
        this->on_jbd_bms_data_(function, frame_data);
      } else {
        ESP_LOGW(TAG, "CRC check failed! 0x%04X != 0x%04X", computed_crc, remote_crc);
      }
    } else {
      ESP_LOGW(TAG, "Invalid frame length: expected %d, got %zu", frame_len, this->frame_buffer_.size());
    }
    this->frame_buffer_.clear();
  }
}

void NimbleJbdBms::on_jbd_bms_data_(const uint8_t &function, const std::vector<uint8_t> &data) {
  this->reset_online_status_tracker_();

  switch (function) {
    case JBD_CMD_HWINFO:
      this->on_hardware_info_data_(data);
      this->send_command(JBD_CMD_READ, JBD_CMD_CELLINFO);
      break;
    case JBD_CMD_CELLINFO:
      this->on_cell_info_data_(data);
      break;
    default:
      ESP_LOGW(TAG, "Unhandled response function 0x%02X", function);
      break;
  }
}

void NimbleJbdBms::on_cell_info_data_(const std::vector<uint8_t> &data) {
  auto jbd_get_16bit = [&](size_t i) -> uint16_t {
    return (uint16_t(data[i + 0]) << 8) | uint16_t(data[i + 1]);
  };

  uint8_t data_len = data.size();
  if (data_len < 2 || data_len > 64 || (data_len % 2) != 0) {
    ESP_LOGW(TAG, "Skipping cell info frame because of invalid length: %d", data_len);
    return;
  }

  uint8_t cells = std::min(data_len / 2, 32);
  float min_cell_voltage = 100.0f;
  float max_cell_voltage = -100.0f;
  float average_cell_voltage = 0.0f;
  uint8_t min_voltage_cell = 0;
  uint8_t max_voltage_cell = 0;

  for (uint8_t i = 0; i < cells; i++) {
    float cell_voltage = static_cast<float>(jbd_get_16bit(i * 2)) * 0.001f;
    average_cell_voltage += cell_voltage;
    if (cell_voltage < min_cell_voltage) {
      min_cell_voltage = cell_voltage;
      min_voltage_cell = i + 1;
    }
    if (cell_voltage > max_cell_voltage) {
      max_cell_voltage = cell_voltage;
      max_voltage_cell = i + 1;
    }
    this->publish_state_(this->cells_[i].cell_voltage_sensor_, cell_voltage);
  }
  average_cell_voltage /= cells;

  this->publish_state_(this->min_cell_voltage_sensor_, min_cell_voltage);
  this->publish_state_(this->max_cell_voltage_sensor_, max_cell_voltage);
  this->publish_state_(this->max_voltage_cell_sensor_, static_cast<float>(max_voltage_cell));
  this->publish_state_(this->min_voltage_cell_sensor_, static_cast<float>(min_voltage_cell));
  this->publish_state_(this->delta_cell_voltage_sensor_, max_cell_voltage - min_cell_voltage);
  this->publish_state_(this->average_cell_voltage_sensor_, average_cell_voltage);
}

void NimbleJbdBms::on_hardware_info_data_(const std::vector<uint8_t> &data) {
  auto jbd_get_16bit = [&](size_t i) -> uint16_t {
    return (uint16_t(data[i + 0]) << 8) | uint16_t(data[i + 1]);
  };

  float total_voltage = jbd_get_16bit(0) * 0.01f;
  this->publish_state_(this->total_voltage_sensor_, total_voltage);

  float current = static_cast<float>(static_cast<int16_t>(jbd_get_16bit(2))) * 0.01f;
  float power = total_voltage * current;
  this->publish_state_(this->current_sensor_, current);
  this->publish_state_(this->power_sensor_, power);
  this->publish_state_(this->charging_power_sensor_, std::max(0.0f, power));
  this->publish_state_(this->discharging_power_sensor_, std::abs(std::min(0.0f, power)));

  this->publish_state_(this->capacity_remaining_sensor_, static_cast<float>(jbd_get_16bit(4)) * 0.01f);
  this->publish_state_(this->nominal_capacity_sensor_, static_cast<float>(jbd_get_16bit(6)) * 0.01f);
  this->publish_state_(this->charging_cycles_sensor_, static_cast<float>(jbd_get_16bit(8)));

  uint16_t errors_bitmask = jbd_get_16bit(16);
  this->publish_state_(this->errors_text_sensor_, this->bitmask_to_string_(ERRORS, ERRORS_SIZE, errors_bitmask));

  this->publish_state_(this->state_of_charge_sensor_, static_cast<float>(data[19]));

  uint8_t operation_status = data[20];
  this->mosfet_status_ = operation_status;
  this->publish_state_(this->operation_status_text_sensor_,
                       this->bitmask_to_string_(OPERATION_STATUS, OPERATION_STATUS_SIZE, operation_status));
  this->publish_state_(this->charging_switch_, operation_status & JBD_MOS_CHARGE);
  this->publish_state_(this->discharging_switch_, operation_status & JBD_MOS_DISCHARGE);

  this->publish_state_(this->cell_count_sensor_, static_cast<float>(data[21]));

  uint8_t temperature_sensors = std::min(data[22], static_cast<uint8_t>(6));
  for (uint8_t i = 0; i < temperature_sensors; i++) {
    this->publish_state_(this->temperatures_[i].temperature_sensor_,
                         static_cast<float>(jbd_get_16bit(23 + (i * 2)) - 2731) * 0.1f);
  }
}

void NimbleJbdBms::start_authentication_() {
  ESP_LOGI(TAG, "Starting authentication flow");
  this->authentication_state_ = AuthState::SENDING_APP_KEY;
  this->auth_timeout_start_ = millis();
  this->send_app_key_();
}

void NimbleJbdBms::send_app_key_() {
  uint8_t frame[11] = {JBD_AUTH_PKT_START, JBD_AUTH_PKT_SECOND, JBD_AUTH_SEND_APP_KEY, 0x06,
                       0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x00};
  frame[10] = this->auth_chksum_(frame + 2, 8);
  this->send_auth_frame_(frame, sizeof(frame));
}

void NimbleJbdBms::request_random_byte_() {
  uint8_t frame[5] = {JBD_AUTH_PKT_START, JBD_AUTH_PKT_SECOND, JBD_AUTH_GET_RANDOM, 0x00, 0x00};
  frame[4] = this->auth_chksum_(frame + 2, 2);
  this->send_auth_frame_(frame, sizeof(frame));
}

void NimbleJbdBms::send_user_password_() {
  std::string password_str = this->password_.empty() ? "123123" : this->password_;
  uint8_t frame[11] = {JBD_AUTH_PKT_START, JBD_AUTH_PKT_SECOND, JBD_AUTH_SEND_PASSWORD, 0x06};
  for (int i = 0; i < 6; i++) {
    frame[4 + i] = ((this->peer_addr_[i] ^ static_cast<uint8_t>(password_str[i])) + this->random_byte_) & 255;
  }
  frame[10] = this->auth_chksum_(frame + 2, 8);
  this->send_auth_frame_(frame, sizeof(frame));
}

void NimbleJbdBms::send_root_password_() {
  uint8_t frame[20];
  frame[0] = JBD_AUTH_PKT_START;
  frame[1] = JBD_AUTH_PKT_SECOND;
  frame[2] = JBD_AUTH_SEND_ROOT_PASSWORD;
  frame[3] = sizeof(ROOT_PASSWORD);
  for (size_t i = 0; i < sizeof(ROOT_PASSWORD); i++) {
    uint8_t mac_byte = (i < 6) ? this->peer_addr_[i] : 0x00;
    frame[4 + i] = ((mac_byte ^ ROOT_PASSWORD[i]) + this->random_byte_) & 255;
  }
  frame[4 + sizeof(ROOT_PASSWORD)] = this->auth_chksum_(frame + 2, 2 + sizeof(ROOT_PASSWORD));
  this->send_auth_frame_(frame, 5 + sizeof(ROOT_PASSWORD));
}

void NimbleJbdBms::send_auth_frame_(const uint8_t *frame, size_t length) {
  if (!this->gatt_ready_ || this->control_val_handle_ == 0)
    return;
  ble_gattc_write_flat(this->conn_handle_, this->control_val_handle_, frame, length, nullptr, nullptr);
}

void NimbleJbdBms::handle_auth_response_(uint8_t command, const uint8_t *data, uint8_t data_len) {
  switch (command) {
    case JBD_AUTH_SEND_APP_KEY:
      switch (data[0]) {
        case 0x00:
          this->authentication_state_ = AuthState::REQUESTING_RANDOM;
          this->request_random_byte_();
          break;
        case 0x02:
          ESP_LOGI(TAG, "App key accepted, no password required");
          this->authentication_state_ = AuthState::AUTHENTICATED;
          this->send_command(JBD_CMD_READ, JBD_CMD_HWINFO);
          break;
        default:
          ESP_LOGE(TAG, "App key rejected: 0x%02X", data[0]);
          this->authentication_state_ = AuthState::NOT_AUTHENTICATED;
          break;
      }
      break;
    case JBD_AUTH_GET_RANDOM:
      this->random_byte_ = data[0];
      if (this->authentication_state_ == AuthState::REQUESTING_RANDOM) {
        this->authentication_state_ = AuthState::SENDING_PASSWORD;
        this->send_user_password_();
      } else if (this->authentication_state_ == AuthState::REQUESTING_ROOT_RANDOM) {
        this->authentication_state_ = AuthState::SENDING_ROOT_PASSWORD;
        this->send_root_password_();
      }
      break;
    case JBD_AUTH_SEND_PASSWORD:
      if (data[0] == 0x00) {
        this->authentication_state_ = AuthState::REQUESTING_ROOT_RANDOM;
        this->request_random_byte_();
      } else {
        ESP_LOGE(TAG, "Password rejected");
        this->authentication_state_ = AuthState::NOT_AUTHENTICATED;
      }
      break;
    case JBD_AUTH_SEND_ROOT_PASSWORD:
      if (data[0] == 0x00) {
        ESP_LOGI(TAG, "Authentication successful");
        this->authentication_state_ = AuthState::AUTHENTICATED;
        this->send_command(JBD_CMD_READ, JBD_CMD_HWINFO);
      } else {
        ESP_LOGE(TAG, "Root password rejected");
        this->authentication_state_ = AuthState::NOT_AUTHENTICATED;
      }
      break;
    default:
      break;
  }
}

void NimbleJbdBms::check_auth_timeout_() {
  if (this->authentication_state_ == AuthState::NOT_AUTHENTICATED ||
      this->authentication_state_ == AuthState::AUTHENTICATED) {
    return;
  }
  if (millis() - this->auth_timeout_start_ > this->auth_timeout_ms_) {
    ESP_LOGW(TAG, "Authentication timeout, resetting");
    this->authentication_state_ = AuthState::NOT_AUTHENTICATED;
  }
}

void NimbleJbdBms::track_online_status_() {
  if (this->no_response_count_ < MAX_NO_RESPONSE_COUNT) {
    this->no_response_count_++;
  }
  if (this->no_response_count_ == MAX_NO_RESPONSE_COUNT) {
    this->publish_device_unavailable_();
    this->no_response_count_++;
  }
}

void NimbleJbdBms::reset_online_status_tracker_() {
  this->no_response_count_ = 0;
}

void NimbleJbdBms::publish_device_unavailable_() {
  this->publish_state_(this->errors_text_sensor_, "Offline");
  this->publish_state_(this->state_of_charge_sensor_, NAN);
  this->publish_state_(this->total_voltage_sensor_, NAN);
  this->publish_state_(this->current_sensor_, NAN);
  this->publish_state_(this->power_sensor_, NAN);
  this->publish_state_(this->min_cell_voltage_sensor_, NAN);
  this->publish_state_(this->max_cell_voltage_sensor_, NAN);
  this->publish_state_(this->delta_cell_voltage_sensor_, NAN);
  for (auto &cell : this->cells_) {
    this->publish_state_(cell.cell_voltage_sensor_, NAN);
  }
  for (auto &temperature : this->temperatures_) {
    this->publish_state_(temperature.temperature_sensor_, NAN);
  }
}

void NimbleJbdBms::dump_config() {
  uint8_t mac[6];
  for (int i = 0; i < 6; i++) {
    mac[i] = (this->address_ >> (i * 8)) & 0xFF;
  }
  char mac_buf[MAC_ADDRESS_PRETTY_BUFFER_SIZE];
  format_mac_addr_upper(mac, mac_buf);

  ESP_LOGCONFIG(TAG, "NimBLE JBD BMS:");
  ESP_LOGCONFIG(TAG, "  MAC: %s", mac_buf);
  ESP_LOGCONFIG(TAG, "  Service UUID: %s", this->service_uuid_.c_str());
  ESP_LOGCONFIG(TAG, "  Notify UUID: %s", this->notify_uuid_.c_str());
  ESP_LOGCONFIG(TAG, "  Control UUID: %s", this->control_uuid_.c_str());
  ESP_LOGCONFIG(TAG, "  Connected: %s", YESNO(this->is_connected()));
  ESP_LOGCONFIG(TAG, "  GATT ready: %s", YESNO(this->gatt_ready_));
}

void NimbleJbdBms::publish_state_(sensor::Sensor *sensor, float value) {
  if (sensor == nullptr)
    return;
  sensor->publish_state(value);
}

void NimbleJbdBms::publish_state_(switch_::Switch *obj, bool state) {
  if (obj == nullptr)
    return;
  obj->publish_state(state);
}

void NimbleJbdBms::publish_state_(text_sensor::TextSensor *text_sensor, const std::string &state) {
  if (text_sensor == nullptr)
    return;
  text_sensor->publish_state(state);
}

std::vector<uint8_t> NimbleJbdBms::build_frame_(uint8_t command, uint8_t address, const uint8_t *data,
                                                  uint8_t data_len) const {
  std::vector<uint8_t> frame(data_len + 7);
  frame[0] = JBD_PKT_START;
  frame[1] = command;
  frame[2] = address;
  frame[3] = data_len;
  for (uint8_t i = 0; i < data_len; i++)
    frame[4 + i] = data[i];
  auto crc = this->chksum_(frame.data() + 2, data_len + 2);
  frame[4 + data_len] = crc >> 8;
  frame[5 + data_len] = crc >> 0;
  frame[6 + data_len] = JBD_PKT_END;
  return frame;
}

uint16_t NimbleJbdBms::chksum_(const uint8_t data[], uint16_t len) const {
  uint16_t checksum = 0x00;
  for (uint16_t i = 0; i < len; i++) {
    checksum = checksum - data[i];
  }
  return checksum;
}

uint8_t NimbleJbdBms::auth_chksum_(const uint8_t *data, uint16_t length) const {
  uint8_t checksum = 0;
  for (uint16_t i = 0; i < length; i++) {
    checksum += data[i];
  }
  return checksum;
}

std::string NimbleJbdBms::bitmask_to_string_(const char *const messages[], uint8_t messages_size,
                                              uint16_t mask) const {
  std::string values;
  if (mask) {
    for (int i = 0; i < messages_size; i++) {
      if (mask & (1 << i)) {
        values.append(messages[i]);
        values.append(";");
      }
    }
    if (!values.empty()) {
      values.pop_back();
    }
  }
  return values;
}

}  // namespace esphome::nimble_jbd_bms
