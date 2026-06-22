#pragma once

#include "esphome/core/component.h"
#include <cstdint>
#include <vector>

struct ble_gap_event;
struct ble_gap_disc_desc;

namespace esphome::nimble_host {
class NimbleHost;
}

namespace esphome::nimble_gap {

using NimbleGapEventHandler = int (*)(struct ble_gap_event *event, void *context);
using NimbleGapScanTimeoutHandler = void (*)(void *context);
using NimbleGapAdvHandler = void (*)(const struct ble_gap_disc_desc &disc, void *context);

struct NimbleGapClient {
  void *context{nullptr};
  uint64_t address{0};
  uint16_t conn_handle{0xFFFF};
  bool wants_connect{false};
  uint32_t scan_adv_count{0};
  const char *log_tag{nullptr};
  NimbleGapEventHandler on_gap_event{nullptr};
  NimbleGapScanTimeoutHandler on_scan_timeout{nullptr};
};

class NimbleGapCoordinator : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override;

  void set_nimble_host(nimble_host::NimbleHost *host);

  void register_gatt_client(NimbleGapClient *client);
  void register_adv_listener(NimbleGapAdvHandler handler, void *context);

  void request_connect_scan(NimbleGapClient *client);
  void cancel_connect_scan(NimbleGapClient *client);

  bool is_scanning() const { return this->scanning_; }

  static bool peer_mac_matches(uint64_t address, const uint8_t *addr_val);
  static bool is_connectable_adv(uint8_t event_type);

 protected:
  void ensure_scanning_();
  void stop_scanning_();
  void resume_scan_if_needed_();
  bool needs_scan_() const;
  bool any_client_wants_connect_() const;
  NimbleGapClient *find_client_by_conn_handle_(uint16_t conn_handle);
  static int gap_event_(struct ble_gap_event *event, void *arg);

  nimble_host::NimbleHost *host_{nullptr};
  std::vector<NimbleGapClient *> gatt_clients_;
  std::vector<std::pair<NimbleGapAdvHandler, void *>> adv_listeners_;

  bool scanning_{false};
  bool connecting_{false};
  NimbleGapClient *pending_client_{nullptr};
  uint8_t own_addr_type_{0};
  uint32_t scan_adv_count_{0};
};

extern NimbleGapCoordinator *global_nimble_gap;

inline bool register_gatt_client_once(NimbleGapClient *client, bool &registered) {
  if (registered || client == nullptr || global_nimble_gap == nullptr) {
    return registered;
  }
  global_nimble_gap->register_gatt_client(client);
  registered = true;
  return true;
}

inline bool register_adv_listener_once(NimbleGapAdvHandler handler, void *context, bool &registered) {
  if (registered || handler == nullptr || global_nimble_gap == nullptr) {
    return registered;
  }
  global_nimble_gap->register_adv_listener(handler, context);
  registered = true;
  return true;
}

}  // namespace esphome::nimble_gap
