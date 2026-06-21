#include "nimble_host.h"
#include "esphome/core/log.h"

extern "C" {
#include "esp_hosted.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
}

namespace esphome::nimble_host {

static const char *const TAG = "nimble_host";

NimbleHost *global_nimble_host = nullptr;

static void nimble_host_task(void *param);

static void on_reset(int reason) { ESP_LOGW(TAG, "NimBLE reset, reason=%d", reason); }

static void on_sync(void) {
  int rc = ble_hs_util_ensure_addr(0);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_hs_util_ensure_addr failed: %d", rc);
    return;
  }

  if (global_nimble_host != nullptr) {
    global_nimble_host->set_synced(true);
    global_nimble_host->call_sync_callbacks();
  }
  ESP_LOGI(TAG, "NimBLE host synced");
}

float NimbleHost::get_setup_priority() const { return setup_priority::BUS; }

void NimbleHost::setup() {
  global_nimble_host = this;
  if (this->enable_on_boot_) {
    this->enable();
  }
}

void NimbleHost::enable() {
  if (this->state_ != HostState::DISABLED) {
    return;
  }
  this->state_ = HostState::PENDING_ENABLE;
}

void NimbleHost::disable() {
  // Deinit not implemented yet — hosted NimBLE teardown is rarely needed on this platform.
  ESP_LOGW(TAG, "NimBLE disable not implemented");
}

void NimbleHost::loop() {
  if (this->state_ != HostState::PENDING_ENABLE) {
    return;
  }

  ESP_LOGI(TAG, "Enabling NimBLE host (after WiFi settle)...");
  this->start_host_();
  this->state_ = HostState::ACTIVE;
}

void NimbleHost::start_host_() {
  esp_hosted_connect_to_slave();

  if (esp_hosted_bt_controller_init() != ESP_OK) {
    ESP_LOGE(TAG, "esp_hosted_bt_controller_init failed");
    this->mark_failed();
    return;
  }

  if (esp_hosted_bt_controller_enable() != ESP_OK) {
    ESP_LOGE(TAG, "esp_hosted_bt_controller_enable failed");
    this->mark_failed();
    return;
  }

  esp_err_t ret = nimble_port_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "nimble_port_init failed: %s", esp_err_to_name(ret));
    this->mark_failed();
    return;
  }

  ble_hs_cfg.reset_cb = on_reset;
  ble_hs_cfg.sync_cb = on_sync;
  ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
  ble_hs_cfg.sm_io_cap = BLE_HS_IO_KEYBOARD_ONLY;
  ble_hs_cfg.sm_bonding = 1;
  ble_hs_cfg.sm_mitm = 1;
  ble_hs_cfg.sm_sc = 0;

  nimble_port_freertos_init(nimble_host_task);
}

void NimbleHost::dump_config() {
  ESP_LOGCONFIG(TAG, "NimBLE Host:");
  ESP_LOGCONFIG(TAG, "  Active: %s", YESNO(this->is_active()));
  ESP_LOGCONFIG(TAG, "  Synced: %s", YESNO(this->synced_));
}

void NimbleHost::call_sync_callbacks() {
  for (auto &cb : this->on_sync_callbacks_) {
    cb();
  }
}

static void nimble_host_task(void *param) {
  ESP_LOGI(TAG, "NimBLE host task started");
  nimble_port_run();
  nimble_port_freertos_deinit();
}

}  // namespace esphome::nimble_host
