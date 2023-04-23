#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/select/select.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace inchanet_wallbox {

enum SelectType {
  MAX_CHARGE_CURRENT = 0,
  DEFAUT_CHARGE_CURRENT = 1,
};

enum ChargingCurrentOption {
  CHARGING_DISABLED = 0,
  CHARGING_CURRENT_6A = 6,
  CHARGING_CURRENT_7A = 7,
  CHARGING_CURRENT_8A = 8,
  CHARGING_CURRENT_9A = 9,
  CHARGING_CURRENT_10A = 10,
  CHARGING_CURRENT_11A = 11,
  CHARGING_CURRENT_12A = 12,
  CHARGING_CURRENT_13A = 13,
  CHARGING_CURRENT_14A = 14,
  CHARGING_CURRENT_15A = 15,
  CHARGING_CURRENT_16A = 16,
};

class InchanetWallboxComponent : public uart::UARTDevice, public PollingComponent {
  public:
    void setup() override;
    void update() override;
    void dump_config() override;

    void set_evse_id(uint32_t s) { evse_id_ = s; }
    void set_voltage_l1_sensor(sensor::Sensor *s) { voltage_l1_sensor_ = s; }
    void set_voltage_l2_sensor(sensor::Sensor *s) { voltage_l2_sensor_ = s; }
    void set_voltage_l3_sensor(sensor::Sensor *s) { voltage_l3_sensor_ = s; }
    void set_current_l1_sensor(sensor::Sensor *s) { current_l1_sensor_ = s; }
    void set_current_l2_sensor(sensor::Sensor *s) { current_l2_sensor_ = s; }
    void set_current_l3_sensor(sensor::Sensor *s) { current_l3_sensor_ = s; }
    void set_power_factor_l1_sensor(sensor::Sensor *s) { power_factor_l1_sensor_ = s; }
    void set_power_factor_l2_sensor(sensor::Sensor *s) { power_factor_l2_sensor_ = s; }
    void set_power_factor_l3_sensor(sensor::Sensor *s) { power_factor_l3_sensor_ = s; }
    void set_frequency_l1_sensor(sensor::Sensor *s) { frequency_l1_sensor_ = s; }
    void set_frequency_l2_sensor(sensor::Sensor *s) { frequency_l2_sensor_ = s; }
    void set_frequency_l3_sensor(sensor::Sensor *s) { frequency_l3_sensor_ = s; }
    void set_external_current_l1_sensor(sensor::Sensor *s) { external_current_l1_sensor_ = s; }
    void set_external_current_l2_sensor(sensor::Sensor *s) { external_current_l2_sensor_ = s; }
    void set_external_current_l3_sensor(sensor::Sensor *s) { external_current_l3_sensor_ = s; }
    void set_wh_in_this_session_sensor(sensor::Sensor *s) { wh_in_this_session_sensor_ = s; }
    void set_wh_all_time_sensor(sensor::Sensor *s) { wh_all_time_sensor_ = s; }
    void set_evse_temperature_sensor(sensor::Sensor *s) { evse_temperature_sensor_ = s; }
    void set_state_sensor(text_sensor::TextSensor *s) { state_sensor_ = s; }
    void set_state_of_charging_sensor(text_sensor::TextSensor *s) { state_of_charging_sensor_ = s; }
    void set_state_of_electric_vehicle_sensor(text_sensor::TextSensor *s) { state_of_electric_vehicle_sensor_ = s; }
    void set_state_of_sockets_lock_sensor(text_sensor::TextSensor *s) { state_of_sockets_lock_sensor_ = s; }
    void set_warnings_sensor(text_sensor::TextSensor *s) { warnings_sensor_ = s; }
    void set_serious_errors_sensor(text_sensor::TextSensor *s) { serious_errors_sensor_ = s; }
    void set_measured_pp_resistance_sensor(text_sensor::TextSensor *s) { measured_pp_resistance_sensor_ = s; }
    void set_enable_3_phase_switch(switch_::Switch *s) { enable_3_phase_switch_ = s; }
    void set_enabled_3_phase(bool s) { enabled_3_phase_ = s; }
    void set_max_charging_current(ChargingCurrentOption s) { max_charging_current_ = s; }
    void set_default_charging_current(ChargingCurrentOption s) { default_charging_current_ = s; }
    bool get_enabled_3_phase() { return enabled_3_phase_; }
    ChargingCurrentOption get_max_charging_current () { return max_charging_current_; }
    ChargingCurrentOption get_default_charging_current () { return default_charging_current_; }

  protected:

    sensor::Sensor *voltage_l1_sensor_;
    sensor::Sensor *voltage_l2_sensor_;
    sensor::Sensor *voltage_l3_sensor_;
    sensor::Sensor *current_l1_sensor_;
    sensor::Sensor *current_l2_sensor_;
    sensor::Sensor *current_l3_sensor_;
    sensor::Sensor *power_factor_l1_sensor_;
    sensor::Sensor *power_factor_l2_sensor_;
    sensor::Sensor *power_factor_l3_sensor_;
    sensor::Sensor *frequency_l1_sensor_;
    sensor::Sensor *frequency_l2_sensor_;
    sensor::Sensor *frequency_l3_sensor_;
    sensor::Sensor *external_current_l1_sensor_;
    sensor::Sensor *external_current_l2_sensor_;
    sensor::Sensor *external_current_l3_sensor_;
    sensor::Sensor *wh_in_this_session_sensor_;
    sensor::Sensor *wh_all_time_sensor_;
    sensor::Sensor *evse_temperature_sensor_;

    text_sensor::TextSensor *state_sensor_;
    text_sensor::TextSensor *state_of_charging_sensor_;
    text_sensor::TextSensor *state_of_electric_vehicle_sensor_;
    text_sensor::TextSensor *state_of_sockets_lock_sensor_;
    text_sensor::TextSensor *warnings_sensor_;
    text_sensor::TextSensor *serious_errors_sensor_;
    text_sensor::TextSensor *measured_pp_resistance_sensor_;

    switch_::Switch *enable_3_phase_switch_;

    uint32_t evse_id_ = 0;

    bool enabled_3_phase_ = true;
    ChargingCurrentOption max_charging_current_ = CHARGING_CURRENT_16A;
    ChargingCurrentOption default_charging_current_ = CHARGING_CURRENT_16A;

    unsigned int Calc_CRC32_sw(unsigned char* data, unsigned int length);
    void create_packet(uint8_t *packet_array, uint32_t ID, uint8_t charging_type, uint8_t max_Amps, uint8_t third_rele, uint8_t default_Amps);
    std::string decode_state_of_ev(uint8_t state);
    std::string decode_state_of_charging(uint8_t state);
    std::string decode_state_of_lock(uint8_t state);
    std::string decode_state_of_PP(uint8_t state);
};

} // inchanet_wallbox
} // esphome