#include "esphome/core/log.h"
#include "inchanet_wallbox_switch.h"

namespace esphome {
namespace inchanet_wallbox {

static const char *const TAG = "inchanet_wallbox.switch";

void InchanetWallboxSwitch::setup() {
  this->publish_state(this->parent_->get_enabled_3_phase());
}

void InchanetWallboxSwitch::write_state(bool state) {
  ESP_LOGI(TAG, "Setting switch: %s", ONOFF(state));
  this->parent_->set_enabled_3_phase(state);
  this->publish_state(state);
}

void InchanetWallboxSwitch::dump_config() {
  LOG_SWITCH("", "Inchanet Wallbox Switch", this);
}

}  // namespace inchanet_wallbox
}  // namespace esphome
