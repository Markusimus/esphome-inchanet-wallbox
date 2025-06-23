#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/select/select.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace inchanet_wallbox {

enum PacketFinderState {
  WAITING_FOR_START_BYTE_1,
  WAITING_FOR_START_BYTE_2,
  READING_PACKET,
};

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

struct __attribute__((packed)) EvseResponseFields {
    uint16_t start_byte; // 0
    uint16_t length; // 2-3
    uint32_t evse_id; // 4-7
    uint8_t state_of_ev; // 8
    uint8_t state_of_charging; // 9
    uint8_t warnings; // 10
    uint8_t errors; // 11
    uint16_t voltage_l1; // 12-13
    uint16_t voltage_l2; // 14-15
    uint16_t voltage_l3; // 16-17
    int16_t current_l1; // 18-19
    int16_t current_l2; // 20-21
    int16_t current_l3; // 22-23
    uint32_t wh_session; // 24-27
    uint32_t wh_total; // 28-31
    uint8_t state_of_lock; // 32
    int8_t temperature; // 33
    int8_t external_temperature; // 34
    int8_t emergency_stop; // 35
    uint8_t state_of_pp_res; // 36
    int8_t pf_l1; // 37
    int8_t pf_l2; // 38
    int8_t pf_l3; // 39
    uint16_t freq_l1; // 40-41
    uint16_t freq_l2; // 42-43
    uint16_t freq_l3; // 44-45
    uint32_t last_rfid; // 46-49
    uint8_t rfid_read_counter; // 50
    int16_t ext_curr_l1; // 51-52
    int16_t ext_curr_l2; // 53-54
    int16_t ext_curr_l3; // 55-56
    uint8_t cp_pwm_width; // 57
    int8_t cp_voltage_high; // 58
    int8_t cp_voltage_low; // 59
    uint32_t crc32; // 60-63
};

union EvseResponsePacket {
  uint8_t raw[64];
  EvseResponseFields fields;
};

/**
 * @struct EvseCommandFields
 * @brief Represents the command packet sent to the EVSE.
 *
 * This structure defines the fields for a command sent to the EVSE (Electric Vehicle Supply Equipment).
 * It is packed to ensure byte alignment for serial communication.
 */
struct __attribute__((packed)) EvseCommandFields {
    /**
     * @var start_byte
     * @brief The starting bytes of the packet. Must be 0x54 0xDD.
     */
    uint16_t start_byte;
    /**
     * @var length
     * @brief The length of the packet in bytes. Should be 16.
     */
    uint16_t length;
    /**
     * @var evse_id
     * @brief The ID of the EVSE. 0 for broadcast, or the serial number for a direct command.
     */
    uint32_t evse_id;
    /**
     * @var charging_type
     * @brief The type of charging to be performed.
     * @details
     * - 0x00: Keep the current state.
     * - 0x01: Charging not enabled.
     * - 0x02: Charge 1-phase enabled.
     * - 0x03: Charge 3-phase enabled.
     */
    uint8_t charging_type;
    /**
     * @var max_amps
     * @brief The maximum allowed charging current in Amperes.
     * @details Allowed values are 0, or 6 through 16.
     */
    uint8_t max_amps;
    /**
     * @var third_rele
     * @brief Controls an external relay.
     */
    uint8_t third_rele; // External relay
    /**
     * @var default_amps
     * @brief The default charging current if communication with the EVSE is lost.
     * @details The most significant bit (MSB) determines the phase: 1 for 1-phase, 0 for 3-phase.
     * The 7 least significant bits (LSB) represent the max current (0, or 6-16A).
     */
    uint8_t default_amps;
    /**
     * @var crc32
     * @brief 32-bit CRC checksum for packet integrity.
     */
    uint32_t crc32;
};


union EvseCommandPacket {
  uint8_t raw[16];
  EvseCommandFields fields;
};

/*
Type of request:
0 - SW version - string
1 - HW version - string
2 - manufacturer ID - string
3 - max current of the device - bin (uin8_t)
4 - type of RCD (A/A-EV/none) - string
5 - grid type (TN-S/IT/...) - string
6 - button selectable currents - bin (X*uint8_t)
7 - type of current measurement - string
"8 - type of external regulation
      (temperature / 0-10V / 2V) - string"
"9 - type of locking mechanism
      (solenoid/Phoenix Contact) - string"
"10 - plug type (Type 2 / Type 1 
        cable/socket) - string"
11 - LED control algorithm - string
12 - default amps - bin (including MSB)(uint16_t)
13 - default amps - string
14 - max device amps - string
15 - EVSE mode - string
16 - EVSE phases - string
"17 - type of communication on main 
         port (U3) - string"
"18 - type of communication on secondary 
         port (U1) - string"
19 - photovoltaic mode - string 
20 - bin name - string (up to 243 bytes)
21 - bootloader version - string
22 - FW CRC sum - bin (uint32_t)
23 - number of currently connected phases - bin (uint8_t)
24 - number of synchronnous phases - bin (uint8_t)
25 - max amps for breaker of local DLM - bin (uint16_t)
"26 - photovoltaic mode - bin (int8_t)
       (-1 = FVE modes not supported
       0 = max power 
       1 = hybrid-mode
       2 = photovoltaic-only)"
"0xFFFF - general request for ID 
               - every EVSE will respond
               - if there is more EVSEs on one bus, 
                  it will lead to undefined result"
*/

struct __attribute__((packed)) VendorRequestPacketFields {
    uint16_t start_byte;    // 2 start bytes (0x44 0xCC)
    uint16_t length;        // 2B length
    uint32_t evse_id;       // 4B EVSE ID
    uint16_t request_type;  // 2B type of request
    uint16_t spare;         // 2B spare
    uint32_t crc32;         // CRC32
};

union VendorRequestPacket {
  uint8_t raw[16];
  VendorRequestPacketFields fields;
};

struct __attribute__((packed)) VendorResponsePacketFields {
    uint16_t start_byte;    // 2 start bytes
    uint16_t length;        // 2B length
    uint32_t evse_id;       // 4B EVSE ID
    uint8_t confirmation;   // 1B confirmation
    char vendor_string[23]; // 23B string
    uint32_t crc32;         // CRC32
};

union VendorResponsePacket {
  uint8_t raw[32];
  VendorResponsePacketFields fields;
};

class InchanetWallboxComponent : public uart::UARTDevice, public PollingComponent {
  public:
    void setup() override;
    void update() override;
    void dump_config() override;

    void set_flow_control_pin(GPIOPin *flow_control_pin) { this->flow_control_pin_ = flow_control_pin; }
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

    bool send_vendor_request_packet(uint16_t request_type, VendorResponsePacket &response);

  protected:

    GPIOPin *flow_control_pin_{nullptr};
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
    bool vendorIdReacived = false;

    bool enabled_3_phase_ = true;
    ChargingCurrentOption max_charging_current_ = CHARGING_CURRENT_6A;
    ChargingCurrentOption default_charging_current_ = CHARGING_CURRENT_6A;
    EvseResponsePacket evse_packet_;
    PacketFinderState packet_finder_state_ = WAITING_FOR_START_BYTE_1;
    uint8_t bytes_read_ = 0;

    unsigned int Calc_CRC32_sw(unsigned char* data, unsigned int length);
    std::string decode_state_of_ev(uint8_t state);
    std::string decode_state_of_charging(uint8_t state);
    std::string decode_state_of_lock(uint8_t state);
    std::string decode_state_of_PP(uint8_t state);
    std::string decode_warnings(uint8_t state);
    std::string decode_errors(uint8_t state);
    void read_control_packet();
    void send_command_packet();
    /**
     * @brief Logs a data packet as a hex string for debugging.
     *
     * @param data Pointer to the byte array.
     * @param size The number of bytes in the array.
     */
    void log_packet(const uint8_t *data, size_t size);
};

} // inchanet_wallbox
} // esphome
