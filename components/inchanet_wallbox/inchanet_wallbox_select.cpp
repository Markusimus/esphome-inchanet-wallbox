#include "inchanet_wallbox_select.h"
#include "esphome/core/log.h"

namespace esphome {
namespace inchanet_wallbox {

static const char *const TAG = "inchanet_wallbox.select";

void InchanetWallboxSelect::setup() {
  auto options = this->traits.get_options();
  auto mappings = this->mappings_;
  ChargingCurrentOption enum_value = CHARGING_DISABLED;
  if (this->select_type_ == MAX_CHARGE_CURRENT) {
    enum_value = this->parent_->get_max_charging_current();
  } 
  if (this->select_type_ == DEFAUT_CHARGE_CURRENT) {
    enum_value = this->parent_->get_default_charging_current();
  }
  auto it = std::find(mappings.cbegin(), mappings.cend(), enum_value);
  if (it == mappings.end()) {
    ESP_LOGW(TAG, "Invalid value %u", enum_value);
    return;
  }
  size_t mapping_idx = std::distance(mappings.cbegin(), it);
  auto value = this->at(mapping_idx);
  this->publish_state(value.value());
}

void InchanetWallboxSelect::control(const std::string &value) {
  this->publish_state(value);

  auto idx = this->index_of(value);
  if (idx.has_value()) {
    ChargingCurrentOption mapping = this->mappings_.at(idx.value());
    if (this->select_type_ == MAX_CHARGE_CURRENT) {
      this->parent_->set_max_charging_current(mapping);
      return;
    }
    if (this->select_type_ == DEFAUT_CHARGE_CURRENT) {
      this->parent_->set_default_charging_current(mapping);
      return;
    }
    ESP_LOGW(TAG, "Invalid select type %u", this->select_type_);
  }

  ESP_LOGW(TAG, "Invalid value %s", value.c_str());
}

void InchanetWallboxSelect::dump_config() {
  LOG_SELECT("", "InchanetWallbox Select", this);
}

}  // namespace tuya
}  // namespace esphome
