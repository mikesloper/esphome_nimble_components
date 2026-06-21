#pragma once

#include "esphome/components/switch/switch.h"
#include "esphome/core/component.h"

namespace esphome::nimble_jbd_bms {

class NimbleJbdBms;

class NimbleJbdSwitch : public switch_::Switch, public Component {
 public:
  void dump_config() override;
  void write_state(bool state) override;

  void set_parent(NimbleJbdBms *parent) { this->parent_ = parent; }
  void set_address(uint8_t address) { this->address_ = address; }
  void set_bitmask(uint8_t bitmask) { this->bitmask_ = bitmask; }

 protected:
  NimbleJbdBms *parent_{nullptr};
  uint8_t address_{0};
  uint8_t bitmask_{0};
};

}  // namespace esphome::nimble_jbd_bms
