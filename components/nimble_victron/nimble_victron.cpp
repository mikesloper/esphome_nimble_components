#include "nimble_victron.h"
#include "victron_values.h"
#include "esphome/components/nimble_gap/nimble_gap.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

extern "C" {
#include "host/ble_gap.h"
#include "host/ble_hs.h"
#include "host/ble_hs_adv.h"
#include "mbedtls/aes.h"
}

namespace esphome::nimble_victron {

using esphome::nimble_gap::global_nimble_gap;

static const char *const TAG = "nimble_victron";

std::vector<NimbleVictron *> NimbleVictronScanner::devices_;
bool NimbleVictron::adv_listener_registered_{false};

float NimbleVictron::get_setup_priority() const { return setup_priority::BUS + 1.0f; }

void NimbleVictron::ensure_gap_registered_() {
  if (!this->device_registered_) {
    NimbleVictronScanner::register_device(this);
    this->device_registered_ = true;
  }
  esphome::nimble_gap::register_adv_listener_once(&NimbleVictronScanner::on_adv, nullptr,
                                                  NimbleVictron::adv_listener_registered_);
}

void NimbleVictron::setup() {
  if (this->host_ == nullptr) {
    ESP_LOGE(TAG, "NimBLE host not set");
    this->mark_failed();
    return;
  }

  this->ensure_gap_registered_();
}

void NimbleVictron::loop() { this->ensure_gap_registered_(); }

void NimbleVictron::dump_config() {
  uint8_t mac[6];
  for (int i = 0; i < 6; i++) {
    mac[i] = (this->address_ >> ((5 - i) * 8)) & 0xFF;
  }
  char mac_buf[MAC_ADDRESS_PRETTY_BUFFER_SIZE];
  format_mac_addr_upper(mac, mac_buf);
  ESP_LOGCONFIG(TAG, "NimBLE Victron:");
  ESP_LOGCONFIG(TAG, "  MAC: %s", mac_buf);
}

void NimbleVictronScanner::register_device(NimbleVictron *device) {
  devices_.push_back(device);
}

void NimbleVictronScanner::on_adv(const struct ble_gap_disc_desc &disc, void *context) {
  struct ble_hs_adv_fields fields{};
  if (ble_hs_adv_parse_fields(&fields, disc.data, disc.length_data) != 0) {
    return;
  }
  if (fields.mfg_data == nullptr || fields.mfg_data_len < 2) {
    return;
  }

  uint16_t company_id = fields.mfg_data[0] | (static_cast<uint16_t>(fields.mfg_data[1]) << 8);
  if (company_id != VICTRON_MANUFACTURER_ID) {
    return;
  }

  const uint8_t *payload = fields.mfg_data + 2;
  size_t payload_len = fields.mfg_data_len - 2;
  handle_advertisement_(disc, payload, payload_len);
}

void NimbleVictronScanner::handle_advertisement_(const struct ble_gap_disc_desc &disc, const uint8_t *payload,
                                                 size_t payload_len) {
  for (auto *device : devices_) {
    if (!nimble_gap::NimbleGapCoordinator::peer_mac_matches(device->get_address(), disc.addr.val)) {
      continue;
    }
    if (device->parse_advertisement(payload, payload_len)) {
      return;
    }
  }
}

bool NimbleVictron::decrypt_payload_(const uint8_t *crypted, size_t crypted_len, uint8_t *out, uint8_t counter_lsb,
                                     uint8_t counter_msb) {
  mbedtls_aes_context ctx;
  mbedtls_aes_init(&ctx);
  int status = mbedtls_aes_setkey_enc(&ctx, this->bindkey_.data(), this->bindkey_.size() * 8);
  if (status != 0) {
    ESP_LOGE(TAG, "mbedtls_aes_setkey_enc failed: %d", status);
    mbedtls_aes_free(&ctx);
    return false;
  }

  size_t nc_offset = 0;
  uint8_t nonce_counter[16] = {counter_lsb, counter_msb, 0};
  uint8_t stream_block[16] = {0};

  status = mbedtls_aes_crypt_ctr(&ctx, crypted_len, &nc_offset, nonce_counter, stream_block, crypted, out);
  mbedtls_aes_free(&ctx);
  if (status != 0) {
    ESP_LOGE(TAG, "mbedtls_aes_crypt_ctr failed: %d", status);
    return false;
  }
  return true;
}

void NimbleVictron::publish_battery_monitor_(const VICTRON_BLE_RECORD_BATTERY_MONITOR *rec) {
  if (this->battery_voltage_sensor_ != nullptr) {
    this->battery_voltage_sensor_->publish_state(vic_to_float(rec->battery_voltage));
  }
  if (this->battery_current_sensor_ != nullptr) {
    this->battery_current_sensor_->publish_state(vic_to_float(rec->battery_current));
  }
  if (this->state_of_charge_sensor_ != nullptr) {
    this->state_of_charge_sensor_->publish_state(vic_to_float(rec->state_of_charge));
  }
  if (this->consumed_ah_sensor_ != nullptr) {
    this->consumed_ah_sensor_->publish_state(vic_to_float(rec->consumed_ah));
  }
  if (this->time_to_go_sensor_ != nullptr) {
    this->time_to_go_sensor_->publish_state(vic_to_float(rec->time_to_go));
  }
  if (this->aux_voltage_sensor_ != nullptr &&
      rec->aux_input_type == VE_REG_BMV_AUX_INPUT::VE_REG_DC_CHANNEL2_VOLTAGE) {
    this->aux_voltage_sensor_->publish_state(vic_to_float(rec->aux_input.aux_voltage));
  }
}

void NimbleVictron::publish_solar_charger_(const VICTRON_BLE_RECORD_SOLAR_CHARGER *rec) {
  if (this->pv_power_sensor_ != nullptr) {
    this->pv_power_sensor_->publish_state(vic_to_float(rec->pv_power));
  }
  if (this->solar_battery_voltage_sensor_ != nullptr) {
    this->solar_battery_voltage_sensor_->publish_state(vic_to_float(rec->battery_voltage));
  }
  if (this->solar_battery_current_sensor_ != nullptr) {
    this->solar_battery_current_sensor_->publish_state(vic_to_float(rec->battery_current));
  }
}

bool NimbleVictron::parse_advertisement(const uint8_t *payload, size_t payload_len) {
  if (payload_len <= sizeof(VICTRON_BLE_RECORD_BASE) ||
      payload_len > sizeof(VICTRON_BLE_RECORD_BASE) + VICTRON_ENCRYPTED_DATA_MAX_SIZE) {
    return false;
  }

  const auto *base = reinterpret_cast<const VICTRON_BLE_RECORD_BASE *>(payload);
  if (base->manufacturer_base.manufacturer_record_type != VICTRON_MANUFACTURER_RECORD_TYPE::PRODUCT_ADVERTISEMENT) {
    return false;
  }

  if (base->encryption_key_0 != this->bindkey_[0]) {
    ESP_LOGW(TAG, "Bindkey mismatch (expected first byte %02X, got %02X)", this->bindkey_[0], base->encryption_key_0);
    return false;
  }

  uint16_t counter = base->data_counter_lsb | (static_cast<uint16_t>(base->data_counter_msb) << 8);
  if (counter == this->last_data_counter_) {
    return true;
  }

  const uint8_t *crypted = payload + sizeof(VICTRON_BLE_RECORD_BASE);
  const size_t crypted_len = payload_len - sizeof(VICTRON_BLE_RECORD_BASE);

  uint8_t decrypted[VICTRON_ENCRYPTED_DATA_MAX_SIZE] = {0};
  if (!this->decrypt_payload_(crypted, crypted_len, decrypted, base->data_counter_lsb, base->data_counter_msb)) {
    return false;
  }

  switch (base->record_type) {
    case VICTRON_BLE_RECORD_TYPE::BATTERY_MONITOR:
      if (crypted_len >= sizeof(VICTRON_BLE_RECORD_BATTERY_MONITOR)) {
        this->publish_battery_monitor_(reinterpret_cast<const VICTRON_BLE_RECORD_BATTERY_MONITOR *>(decrypted));
        ESP_LOGD(TAG, "Battery monitor update (counter=%u)", counter);
      }
      break;
    case VICTRON_BLE_RECORD_TYPE::SOLAR_CHARGER:
      if (crypted_len >= sizeof(VICTRON_BLE_RECORD_SOLAR_CHARGER)) {
        this->publish_solar_charger_(reinterpret_cast<const VICTRON_BLE_RECORD_SOLAR_CHARGER *>(decrypted));
        ESP_LOGD(TAG, "Solar charger update (counter=%u)", counter);
      }
      break;
    default:
      ESP_LOGV(TAG, "Unsupported record type %02X", static_cast<uint8_t>(base->record_type));
      return false;
  }

  this->last_data_counter_ = counter;
  return true;
}

}  // namespace esphome::nimble_victron
