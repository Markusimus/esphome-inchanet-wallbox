#pragma once

#include "esphome/core/component.h"
#include "esphome/components/inchanet_wallbox/inchanet_wallbox.h"
#include "esphome/components/select/select.h"

#include <vector>

namespace esphome {
namespace inchanet_wallbox {

class InchanetWallboxSelect : public select::Select, public Component {
 public:
  void setup() override;
  void dump_config() override;

  void set_inchanet_wallbox_parent(InchanetWallboxComponent *parent) { this->parent_ = parent; }
  void set_select_type (SelectType s) { this->select_type_ = s; }
  void set_select_mappings(std::vector<ChargingCurrentOption> mappings) { this->mappings_ = std::move(mappings); }

 protected:
  void control(const std::string &value) override;

  InchanetWallboxComponent *parent_;
  SelectType select_type_;
  std::vector<ChargingCurrentOption> mappings_;
};

}  // namespace inchanet_wallbox
}  // namespace esphome
