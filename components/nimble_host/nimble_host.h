#pragma once

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include <cstdint>
#include <functional>
#include <vector>

namespace esphome::nimble_host {

class NimbleHost : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override;

  void enable();
  void disable();
  bool is_active() const { return this->state_ == HostState::ACTIVE; }

  bool is_synced() const { return this->synced_; }
  void set_synced(bool synced) { this->synced_ = synced; }

  void set_enable_on_boot(bool enable_on_boot) { this->enable_on_boot_ = enable_on_boot; }

  using SyncCallback = std::function<void()>;
  void add_on_sync_callback(SyncCallback &&callback) { this->on_sync_callbacks_.push_back(std::move(callback)); }
  void call_sync_callbacks();

 protected:
  enum class HostState : uint8_t {
    DISABLED,
    PENDING_ENABLE,
    ACTIVE,
  };

  void start_host_();

  HostState state_{HostState::DISABLED};
  bool synced_{false};
  bool enable_on_boot_{false};
  std::vector<SyncCallback> on_sync_callbacks_;
};

extern NimbleHost *global_nimble_host;

template<typename... Ts>
class NimbleHostEnableAction : public Parented<NimbleHost>, public Action<Ts...> {
 public:
  void play(const Ts &...x) override { this->parent_->enable(); }
};

}  // namespace esphome::nimble_host
