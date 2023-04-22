#pragma once

#include "esphome/core/component.h"
#include "esphome/components/inchanet_wallbox/inchanet_wallbox.h"
#include "esphome/components/switch/switch.h"

namespace esphome {
namespace inchanet_wallbox {

class InchanetWallboxSwitch : public switch_::Switch, public Component {
 public:
  void setup() override;
  void dump_config() override;

  void set_inchanet_wallbox_parent(InchanetWallboxComponent *parent) { this->parent_ = parent; }

 protected:
  void write_state(bool state) override;

  InchanetWallboxComponent *parent_;
};

}  // namespace inchanet_wallbox
}  // namespace esphome
