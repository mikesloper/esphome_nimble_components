#include "nimble_jbd_switch.h"
#include "nimble_jbd_bms.h"
#include "esphome/core/log.h"

namespace esphome::nimble_jbd_bms {

static const char *const TAG = "nimble_jbd_bms.switch";

void NimbleJbdSwitch::dump_config() { LOG_SWITCH("", "Nimble JBD BMS Switch", this); }

void NimbleJbdSwitch::write_state(bool state) {
  if (this->parent_ == nullptr)
    return;
  if (this->parent_->change_mosfet_status(this->address_, this->bitmask_, state)) {
    this->publish_state(state);
  }
}

}  // namespace esphome::nimble_jbd_bms
