#include "nimble_gap.h"
#include "esphome/components/nimble_host/nimble_host.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

extern "C" {
#include "host/ble_gap.h"
#include "host/ble_hs.h"
}

namespace esphome::nimble_gap {

static const char *const TAG = "nimble_gap";

NimbleGapCoordinator *global_nimble_gap = nullptr;

bool NimbleGapCoordinator::peer_mac_matches(uint64_t address, const uint8_t *addr_val) {
  for (int i = 0; i < 6; i++) {
    uint8_t expected = (address >> (i * 8)) & 0xFF;
    if (addr_val[i] != expected) {
      return false;
    }
  }
  return true;
}

bool NimbleGapCoordinator::is_connectable_adv(uint8_t event_type) {
  return event_type == BLE_HCI_ADV_RPT_EVTYPE_ADV_IND || event_type == BLE_HCI_ADV_RPT_EVTYPE_DIR_IND ||
         event_type == BLE_HCI_ADV_RPT_EVTYPE_SCAN_RSP || event_type == BLE_HCI_ADV_RPT_EVTYPE_SCAN_IND;
}

float NimbleGapCoordinator::get_setup_priority() const { return setup_priority::BUS + 2.0f; }

void NimbleGapCoordinator::set_nimble_host(nimble_host::NimbleHost *host) { this->host_ = host; }

void NimbleGapCoordinator::setup() {
  global_nimble_gap = this;
  if (this->host_ == nullptr) {
    ESP_LOGE(TAG, "NimBLE host not set");
    this->mark_failed();
    return;
  }

  this->host_->add_on_sync_callback([this]() {
    if (this->host_->is_active()) {
      this->resume_scan_if_needed_();
    }
  });
}

void NimbleGapCoordinator::loop() {
  if (this->host_ == nullptr || !this->host_->is_active() || !this->host_->is_synced()) {
    return;
  }
  this->resume_scan_if_needed_();
}

void NimbleGapCoordinator::dump_config() {
  ESP_LOGCONFIG(TAG, "NimBLE GAP coordinator:");
  ESP_LOGCONFIG(TAG, "  GATT clients: %u", (unsigned) this->gatt_clients_.size());
  ESP_LOGCONFIG(TAG, "  Adv listeners: %u", (unsigned) this->adv_listeners_.size());
  ESP_LOGCONFIG(TAG, "  Scanning: %s", YESNO(this->scanning_));
  ESP_LOGCONFIG(TAG, "  Connecting: %s", YESNO(this->connecting_));
}

void NimbleGapCoordinator::register_gatt_client(NimbleGapClient *client) {
  this->gatt_clients_.push_back(client);
}

void NimbleGapCoordinator::register_adv_listener(NimbleGapAdvHandler handler, void *context) {
  this->adv_listeners_.emplace_back(handler, context);
}

void NimbleGapCoordinator::request_connect_scan(NimbleGapClient *client) {
  if (client == nullptr || client->conn_handle != 0xFFFF) {
    return;
  }
  client->wants_connect = true;
  ESP_LOGI(TAG, "Client %s requested connect scan", client->log_tag != nullptr ? client->log_tag : "?");
  this->resume_scan_if_needed_();
}

void NimbleGapCoordinator::cancel_connect_scan(NimbleGapClient *client) {
  if (client == nullptr) {
    return;
  }
  client->wants_connect = false;
  if (!this->connecting_ && this->scanning_ && !this->needs_scan_()) {
    this->stop_scanning_();
  }
}

bool NimbleGapCoordinator::any_client_wants_connect_() const {
  for (auto *client : this->gatt_clients_) {
    if (client->wants_connect && client->conn_handle == 0xFFFF) {
      return true;
    }
  }
  return false;
}

bool NimbleGapCoordinator::needs_scan_() const {
  return this->any_client_wants_connect_() || !this->adv_listeners_.empty();
}

void NimbleGapCoordinator::stop_scanning_() {
  if (!this->scanning_) {
    return;
  }
  ble_gap_disc_cancel();
  this->scanning_ = false;
}

void NimbleGapCoordinator::ensure_scanning_() {
  if (this->scanning_ || this->connecting_ || !this->needs_scan_()) {
    return;
  }

  int rc = ble_hs_id_infer_auto(0, &this->own_addr_type_);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_hs_id_infer_auto failed: %d", rc);
    return;
  }

  struct ble_gap_disc_params disc_params{};
  disc_params.filter_duplicates = this->adv_listeners_.empty() ? 1 : 0;
  disc_params.passive = 0;
  disc_params.itvl = 0;
  disc_params.window = 0;

  const int32_t duration =
      this->adv_listeners_.empty() ? 30000 : BLE_HS_FOREVER;

  this->scanning_ = true;
  rc = ble_gap_disc(this->own_addr_type_, duration, &disc_params, gap_event_, this);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_gap_disc failed: %d", rc);
    this->scanning_ = false;
    return;
  }

  ESP_LOGI(TAG, "Shared BLE scan started (%u GATT client(s), %u adv listener(s))", (unsigned) this->gatt_clients_.size(),
           (unsigned) this->adv_listeners_.size());
}

void NimbleGapCoordinator::resume_scan_if_needed_() {
  if (!this->needs_scan_()) {
    return;
  }
  this->ensure_scanning_();
}

NimbleGapClient *NimbleGapCoordinator::find_client_by_conn_handle_(uint16_t conn_handle) {
  for (auto *client : this->gatt_clients_) {
    if (client->conn_handle == conn_handle) {
      return client;
    }
  }
  return nullptr;
}

int NimbleGapCoordinator::gap_event_(struct ble_gap_event *event, void *arg) {
  auto *self = static_cast<NimbleGapCoordinator *>(arg);

  switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
      const auto &disc = event->disc;

      for (const auto &listener : self->adv_listeners_) {
        if (listener.first != nullptr) {
          listener.first(disc, listener.second);
        }
      }

      if (self->connecting_) {
        return 0;
      }

      if (self->any_client_wants_connect_()) {
        self->scan_adv_count_++;
      }

      for (auto *client : self->gatt_clients_) {
        if (!client->wants_connect || client->conn_handle != 0xFFFF) {
          continue;
        }

        const bool mac_match = peer_mac_matches(client->address, disc.addr.val);

        if (self->scan_adv_count_ <= 5 || mac_match) {
          char mac_buf[MAC_ADDRESS_PRETTY_BUFFER_SIZE];
          format_mac_addr_upper(disc.addr.val, mac_buf);
          ESP_LOGI(TAG, "BLE adv #%u: %s type=%d event=%d%s", self->scan_adv_count_, mac_buf, disc.addr.type,
                   disc.event_type, mac_match ? " (target)" : "");
        }

        if (!mac_match || !is_connectable_adv(disc.event_type)) {
          continue;
        }

        ESP_LOGI(TAG, "Found %s in scan, connecting...", client->log_tag != nullptr ? client->log_tag : "GATT client");

        self->stop_scanning_();
        self->connecting_ = true;
        self->pending_client_ = client;

        struct ble_gap_conn_params params{};
        params.scan_itvl = 0x0010;
        params.scan_window = 0x0010;
        params.itvl_min = BLE_GAP_INITIAL_CONN_ITVL_MIN;
        params.itvl_max = BLE_GAP_INITIAL_CONN_ITVL_MAX;
        params.latency = 0;
        params.supervision_timeout = 0x0100;
        params.min_ce_len = 0x0010;
        params.max_ce_len = 0x0300;

        int rc = ble_gap_connect(self->own_addr_type_, &disc.addr, 30000, &params, gap_event_, self);
        if (rc != 0) {
          ESP_LOGE(TAG, "ble_gap_connect failed: %d", rc);
          self->connecting_ = false;
          self->pending_client_ = nullptr;
          if (client->on_scan_timeout != nullptr) {
            client->on_scan_timeout(client->context);
          }
          self->resume_scan_if_needed_();
        }
        return 0;
      }
      return 0;
    }

    case BLE_GAP_EVENT_DISC_COMPLETE:
      self->scanning_ = false;
      if (event->disc_complete.reason == BLE_HS_EPREEMPTED) {
        return 0;
      }

      for (auto *client : self->gatt_clients_) {
        if (client->wants_connect && client->conn_handle == 0xFFFF && client->on_scan_timeout != nullptr) {
          ESP_LOGW(TAG, "Scan finished, %s not found (saw %u adv, reason=%d)", client->log_tag, self->scan_adv_count_,
                   event->disc_complete.reason);
          client->on_scan_timeout(client->context);
        }
      }

      self->scan_adv_count_ = 0;

      self->resume_scan_if_needed_();
      return 0;

    case BLE_GAP_EVENT_CONNECT: {
      NimbleGapClient *client = self->pending_client_;
      self->connecting_ = false;
      self->pending_client_ = nullptr;

      if (client == nullptr) {
        ESP_LOGW(TAG, "Unexpected CONNECT event with no pending client");
        self->resume_scan_if_needed_();
        return 0;
      }

      if (event->connect.status == 0) {
        client->conn_handle = event->connect.conn_handle;
        client->wants_connect = false;
      }

      if (client->on_gap_event != nullptr) {
        client->on_gap_event(event, client->context);
      }

      self->resume_scan_if_needed_();
      return 0;
    }

    case BLE_GAP_EVENT_DISCONNECT: {
      NimbleGapClient *client = self->find_client_by_conn_handle_(event->disconnect.conn.conn_handle);
      if (client == nullptr) {
        return 0;
      }
      client->conn_handle = 0xFFFF;
      if (client->on_gap_event != nullptr) {
        client->on_gap_event(event, client->context);
      }
      self->resume_scan_if_needed_();
      return 0;
    }

    case BLE_GAP_EVENT_NOTIFY_RX: {
      NimbleGapClient *client = self->find_client_by_conn_handle_(event->notify_rx.conn_handle);
      if (client != nullptr && client->on_gap_event != nullptr) {
        return client->on_gap_event(event, client->context);
      }
      return 0;
    }

    case BLE_GAP_EVENT_PASSKEY_ACTION: {
      NimbleGapClient *client = self->find_client_by_conn_handle_(event->passkey.conn_handle);
      if (client != nullptr && client->on_gap_event != nullptr) {
        return client->on_gap_event(event, client->context);
      }
      return 0;
    }

    default:
      return 0;
  }
}

}  // namespace esphome::nimble_gap
