#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/log.h"
#include "inchanet_wallbox.h"
#include <string>
#include <format>

namespace esphome {
namespace inchanet_wallbox {

static const char *TAG = "inchanet_wallbox";

static const uint8_t USART_BUFFER_IN_SIZE_LONG = 64;
static const uint8_t SMALL_PACKET_OUT_SIZE = 16;

static const uint32_t EVSE_ID = 22292;

void InchanetWallboxComponent::setup() {
  // nothing to do here
}

void InchanetWallboxComponent::update() {
  uint8_t buffer[100];
  uint8_t buffer_index = 0;

  // vycistime buffer
  for (int i = 0; i < sizeof(buffer); i++) {buffer[i] = 0;}

  // odesleme zpravu
  uint8_t USART_buffer_out[SMALL_PACKET_OUT_SIZE];

  uint8_t max_Amps;
  uint8_t third_rele;
  uint8_t charging_type;
  uint8_t default_Amps;

  if (this->enabled_3_phase_) {
    // 3-phase
    charging_type = 3;
    max_Amps = static_cast<uint8_t>(this->max_charging_current_);
    third_rele = 0;
    default_Amps = static_cast<uint8_t>(this->default_charging_current_);
  }
  else {
    // 1-phase
    charging_type = 2;
    max_Amps = static_cast<uint8_t>(this->max_charging_current_);
    third_rele = 0;
    default_Amps = static_cast<uint8_t>(this->default_charging_current_) | 0x80;
  }

  create_packet(USART_buffer_out, EVSE_ID, charging_type, max_Amps, third_rele, default_Amps);

  write_array(USART_buffer_out, SMALL_PACKET_OUT_SIZE);

  // cteme dokud je co cist
  bool byl_5x = false;
  while (available() && buffer_index < sizeof(buffer)) {
    uint8_t data;
    read_byte(&data);
    if (0x54 == data) {byl_5x = true;}
    if (true == byl_5x) {
      buffer[buffer_index++] = data;
    }
  }

  if (buffer_index > USART_BUFFER_IN_SIZE_LONG - 1) {
    // kontrola havicky
    if (buffer[0] == 0x54 &&
        buffer[1] == 0xDC &&
        buffer[2] == uint8_t(USART_BUFFER_IN_SIZE_LONG) &&
        buffer[3] == uint8_t(USART_BUFFER_IN_SIZE_LONG >> 8)) {

      // check CRC
      uint32_t computed_crc32 =   Calc_CRC32_sw(buffer, (USART_BUFFER_IN_SIZE_LONG >> 2)-1);
      uint32_t received_crc32 =   *(uint32_t*)(buffer + USART_BUFFER_IN_SIZE_LONG -4);

      if(received_crc32 == computed_crc32){
        // CRC OK
        char tmp_hex[3];
        float tmp_float = 0;
        string tmp_state;

        // state of electric vehicle - dekodujeme hodnoty 
        switch(buffer[9]) {
          case 0:
            tmp_state = "0x00 - EV not connected";
            break;
          case 1:
            tmp_state = "0x01 - EV connected";
            break;
          case 2:
            tmp_state = "0x02 - EV wants to charge";
            break;
          case 3:
            tmp_state = "0x03 - EV needs to ventilate";
            break;
          case 4:
            tmp_state = "0x04 - error state";
            break;
          default:
            tmp_state = std::format("%02X - Unknown", buffer[9]);
            break;
        }
        this->state_of_electric_vehicle_sensor_->publish_state(tmp_state);
        // state of charging - 0x00 - not charging, 0x01 - charging 1-phase, 0x02 - charging 3-phase
        sprintf(&tmp_hex[0], "%02X", buffer[9]);
        this->state_of_charging_sensor_->publish_state(tmp_hex);
        // warnings - HEX
        sprintf(&tmp_hex[0], "%02X", buffer[10]);
        this->warnings_sensor_->publish_state(tmp_hex);
        // serious errors - HEX
        sprintf(&tmp_hex[0], "%02X", buffer[11]);
        this->serious_errors_sensor_->publish_state(tmp_hex);
        // measured voltage
        this->voltage_l1_sensor_->publish_state(0.25 * ((buffer[13] << 8) | buffer[12]));
        this->voltage_l2_sensor_->publish_state(0.25 * ((buffer[15] << 8) | buffer[14]));
        this->voltage_l3_sensor_->publish_state(0.25 * ((buffer[17] << 8) | buffer[16]));
        // measured current
        this->current_l1_sensor_->publish_state(0.1 * (int16_t)((buffer[19] << 8) | buffer[18]));
        this->current_l2_sensor_->publish_state(0.1 * (int16_t)((buffer[21] << 8) | buffer[20]));
        this->current_l3_sensor_->publish_state(0.1 * (int16_t)((buffer[23] << 8) | buffer[22]));
        // Wh in this session
        this->wh_in_this_session_sensor_->publish_state ((buffer[27] << 8) | (buffer[26] << 16) | (buffer[25] << 8) | buffer[24]);
        // All time Wh charged
        this->wh_all_time_sensor_->publish_state ((buffer[31] << 8) | (buffer[30] << 16) | (buffer[29] << 8) | buffer[28]);
        // socket's lock state 
        sprintf(&tmp_hex[0], "%02X", buffer[32]);
        this->state_of_sockets_lock_sensor_->publish_state(tmp_hex);
        // EVSE temperature
        this->evse_temperature_sensor_->publish_state((int8_t)buffer[33]);
        // measured PP resistance
        sprintf(&tmp_hex[0], "%02X", buffer[36]);
        this->measured_pp_resistance_sensor_->publish_state(tmp_hex);
        // power factor L1 L2 L3
        this->power_factor_l1_sensor_->publish_state(0.01 * (int8_t)buffer[37]);
        this->power_factor_l2_sensor_->publish_state(0.01 * (int8_t)buffer[38]);
        this->power_factor_l3_sensor_->publish_state(0.01 * (int8_t)buffer[39]);
        // frequency L1 L2 L3
        this->frequency_l1_sensor_->publish_state(0.1 * (int16_t)((buffer[41] << 8) | buffer[40]));
        this->frequency_l2_sensor_->publish_state(0.1 * (int16_t)((buffer[43] << 8) | buffer[42]));
        this->frequency_l3_sensor_->publish_state(0.1 * (int16_t)((buffer[45] << 8) | buffer[44]));
        // external current L1 L2 L3
        this->external_current_l1_sensor_->publish_state(0.1 * (int16_t)((buffer[52] << 8) | buffer[51]));
        this->external_current_l2_sensor_->publish_state(0.1 * (int16_t)((buffer[54] << 8) | buffer[53]));
        this->external_current_l3_sensor_->publish_state(0.1 * (int16_t)((buffer[56] << 8) | buffer[55]));

        // publikujeme prijata data jako HEX
        char hex_string[sizeof(buffer) * 3 + 1];
        for (int i = 0; i < sizeof(hex_string); i++) {hex_string[i] = 0x00;}
        for (int i = 0; i < USART_BUFFER_IN_SIZE_LONG; i++) {sprintf(&hex_string[i * 3], "%02X ", buffer[i]);}
        this->state_sensor_->publish_state(hex_string);
      }
      else {
        ESP_LOGW(TAG, "CRC NOK");
      }
    }
    else {
      ESP_LOGW(TAG, "Invalid packet header");
    }
  }
  else {
      ESP_LOGW(TAG, "Too short packet");
  }
}

void InchanetWallboxComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Inchanet Wallbox Component");
}

/**
  * tested on little-endian architecture (x86 a STM32 ARM)
  * length is in words, not bytes
  */
unsigned int InchanetWallboxComponent::Calc_CRC32_sw(unsigned char* data, unsigned int length){
  unsigned int polynomial = 0x04C11DB7; /* divisor is 32bit */
  unsigned int crc = 0xFFFFFFFF; /* CRC value is 32bit */

  for(unsigned int i = 0; i < length; i++){
      for(unsigned int index = 0; index < 4; index++){
          unsigned int b = data[i*4 + (3-index)];
          crc ^= (unsigned int)(b << 24); /* move byte into MSB of 32bit CRC */
          
          for (int i = 0; i < 8; i++){
              if ((crc & 0x80000000) != 0){ /* test for MSB = bit 31 */
                  crc = (crc << 1) ^ polynomial;
              }else{
                  crc <<= 1;
              }
          }
      }
  }

  return crc;
}

void InchanetWallboxComponent::create_packet(uint8_t *packet_array, uint32_t ID, uint8_t charging_type, uint8_t max_Amps, uint8_t third_rele, uint8_t default_Amps){
  packet_array[0] = 0x54;
  packet_array[1] = 0xDD;
  packet_array[2] = SMALL_PACKET_OUT_SIZE;
  packet_array[3] = 0;
  packet_array[4] = ID;
  packet_array[5] = (ID >> 8);
  packet_array[6] = (ID >> 16);
  packet_array[7] = (ID >> 24);

  packet_array[8] = charging_type;
  packet_array[9] = max_Amps;
  packet_array[10] = third_rele;
  packet_array[11] = default_Amps;
  
  uint32_t crc = Calc_CRC32_sw(packet_array, (SMALL_PACKET_OUT_SIZE - 4) >> 2);
  
  packet_array[SMALL_PACKET_OUT_SIZE -4] = uint8_t(crc);
  packet_array[SMALL_PACKET_OUT_SIZE -3] = uint8_t(crc >> 8);
  packet_array[SMALL_PACKET_OUT_SIZE -2] = uint8_t(crc >> 16);
  packet_array[SMALL_PACKET_OUT_SIZE -1] = uint8_t(crc >> 24);
}

} // inchanet_wallbox
} // esphome