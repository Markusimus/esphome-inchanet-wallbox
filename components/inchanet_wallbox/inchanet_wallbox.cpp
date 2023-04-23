#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/log.h"
#include "inchanet_wallbox.h"
#include <cstring>

namespace esphome {
namespace inchanet_wallbox {

static const char *TAG = "inchanet_wallbox";

static const uint8_t USART_BUFFER_IN_SIZE_LONG = 64;
static const uint8_t SMALL_PACKET_OUT_SIZE = 16;

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

  create_packet(USART_buffer_out, this->evse_id_, charging_type, max_Amps, third_rele, default_Amps);

  write_array(USART_buffer_out, SMALL_PACKET_OUT_SIZE);

  // delay
  delay(10);

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

        // state of electric vehicle
        this->state_of_electric_vehicle_sensor_->publish_state(
          this->decode_state_of_ev (buffer[8]));
        
        // state of charging
        this->state_of_charging_sensor_->publish_state(
          this->decode_state_of_charging(buffer[9])
        );
        // warnings - HEX
        this->warnings_sensor_->publish_state(
          this->decode_warnings(buffer[10]));

        // serious errors - HEX
        // 0x00 - no errors
        // 0x01 - relay stuck closed
        // 0x02 - relay could not close
        // 0x04 - error on RCD
        // 0x08 - error on PE/N wires
        // 0x10 - overvoltage
        // 0x20 - overcurrent
        // 0x40 - temperature too high (80°+)
        // 0x80 - unsupported charging mode (i.e. ventilation needed or error on PWM voltage or input phase shorted)
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
        this->state_of_sockets_lock_sensor_->publish_state(
          this->decode_state_of_lock(buffer[32]));

        // EVSE temperature
        this->evse_temperature_sensor_->publish_state((int8_t)buffer[33]);

        // measured PP resistance
        this->measured_pp_resistance_sensor_->publish_state(
          this->decode_state_of_PP(buffer[36]));

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

std::string InchanetWallboxComponent::decode_state_of_ev(uint8_t state) {
  switch(state) {
    case 0: return "00 - EV not connected";
    case 1: return "01 - EV connected";
    case 2: return "02 - EV wants to charge";
    case 3: return "03 - EV needs to ventilate";
    case 4: return "04 - error state";
    default: 
      char buff[30];
      sprintf(buff, "%02X - unknown state", state);
      return buff;
  }
}

std::string InchanetWallboxComponent::decode_state_of_charging(uint8_t state) {
  switch(state) {
    case 0: return "00 - not charging";
    case 1: return "01 - charging 1-phase";
    case 2: return "02 - charging 3-phase";
    default: 
      char buff[30];
      sprintf(buff, "%02X - unknown state", state);
      return buff;
  }
}

std::string InchanetWallboxComponent::decode_state_of_lock(uint8_t state) {
  switch(state) {
    case 0: return "00 - unknown lock state";
    case 1: return "01 - trying to lock";
    case 2: return "02 - trying to unlock";
    case 3: return "03 - locked";
    case 4: return "04 - unlocked";
    case 5: return "05 - error";
    default: 
      char buff[30];
      sprintf(buff, "%02X - unknown state", state);
      return buff;
  }
}

std::string InchanetWallboxComponent::decode_state_of_PP(uint8_t state) {
  switch(state) {
    case 0: return "00 - <220 ohm - 64A or EVSE set to cable type";
    case 1: return "01 - 220 ohm - 32A";
    case 2: return "02 - 470 ohm - 25A";
    case 3: return "03 - 680 ohm - 20A";
    case 4: return "04 - 1000 ohm - 16A";
    case 5: return "05 - 1500 ohm - 13A";
    case 6: return "06 - >1500 ohm - 6A (plug not connected)";
    default: 
      char buff[30];
      sprintf(buff, "%02X - unknown value", state);
      return buff;
  }
}

std::string InchanetWallboxComponent::decode_warnings(uint8_t state) {
  // zakodujeme stav
  char buff[1000];
  sprintf(buff, "%02X - ", state);
  size_t pos = strlen (buff);
  // doplnime textove vysvetlivky
  if (0x00 != (state & 0x01)) {
    sprintf(&buff[pos], ", 0x01 - relay B (phase 2 and/or 3) could not close");
    pos = strlen (buff);
  }
  if (0x00 != (state & 0x02)) {
    sprintf(&buff[pos], ", 0x02 - low voltage or missing phase (doesn't work in US/JP device)");
    pos = strlen (buff);
  }
  if (0x00 != (state & 0x04)) {
    sprintf(&buff[pos], ", 0x04 - charging paused by DLM (including 0-5A from master)");
    pos = strlen (buff);
  }
  if (0x00 != (state & 0x08)) {
    sprintf(&buff[pos], ", 0x08 - charging slows down due to higher temperature (70-80°C)");
    pos = strlen (buff);
  }
  if (0x00 != (state & 0x10)) {
    sprintf(&buff[pos], ", 0x10 - error in communication with external current measurement");
    pos = strlen (buff);
  }
  if (0x00 != (state & 0x20)) {
    sprintf(&buff[pos], ", 0x20 - inicialization underway");
    pos = strlen (buff);
  }
  if (0x00 != (state & 0x40)) {
    sprintf(&buff[pos], ", 0x40 - problem on socket's pin lock");
    pos = strlen (buff);
  }
  if (0x00 != (state & 0x80)) {
    sprintf(&buff[pos], ", 0x80 - receiving data from another communication channel");
    pos = strlen (buff);
  }
  return buff;
}

} // inchanet_wallbox
} // esphome