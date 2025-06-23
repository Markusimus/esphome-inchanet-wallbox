#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/log.h"
#include "inchanet_wallbox.h"
#include <cstring>
#include <string>

namespace esphome {
namespace inchanet_wallbox {

static const char *TAG = "inchanet_wallbox";

static const uint8_t START_BYTE_1 = 0x54;
static const uint8_t START_BYTE_2 = 0xDC;
static const uint8_t USART_BUFFER_IN_SIZE_LONG = 64;
static const uint8_t SMALL_PACKET_OUT_SIZE = 16;

void InchanetWallboxComponent::setup() {
  // nothing to do here
  if (this->flow_control_pin_ != nullptr) {
    this->flow_control_pin_->setup();
  }

  vendorIdReacived = false;
}

void InchanetWallboxComponent::update() 
{
  if (!vendorIdReacived)
  {
    VendorResponsePacket responsePacket;
    vendorIdReacived = send_vendor_request_packet(0xFFFF, responsePacket);
    
  } else
  {
    read_control_packet();
    send_command_packet();
  }

}

void InchanetWallboxComponent::send_command_packet()
{
  // Transmit Buffer
  EvseCommandPacket command_packet;

  uint8_t max_Amps;
  uint8_t third_rele;
  uint8_t charging_type;
  uint8_t default_Amps;

  if (this->enabled_3_phase_)
  {
    // 3-phase
    charging_type = 3;
    max_Amps = static_cast<uint8_t>(this->max_charging_current_);
    third_rele = 0;
    default_Amps = static_cast<uint8_t>(this->default_charging_current_);
  }
  else
  {
    // 1-phase
    charging_type = 2;
    max_Amps = static_cast<uint8_t>(this->max_charging_current_);
    third_rele = 0;
    default_Amps = static_cast<uint8_t>(this->default_charging_current_) | 0x80;
  }

  // TODO: Do not change type or rele
  charging_type = 0;
  third_rele = 0;

  // Create packet
  command_packet.fields.start_byte = 0xDD54;
  command_packet.fields.length = SMALL_PACKET_OUT_SIZE;
  command_packet.fields.evse_id = this->evse_id_;
  command_packet.fields.charging_type = charging_type;
  command_packet.fields.max_amps = max_Amps;
  command_packet.fields.third_rele = third_rele;
  command_packet.fields.default_amps = default_Amps;
  command_packet.fields.crc32 = Calc_CRC32_sw(command_packet.raw, (SMALL_PACKET_OUT_SIZE - 4) >> 2);


  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(true);

  this->write_array(command_packet.raw, SMALL_PACKET_OUT_SIZE);
  this->flush();

  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(false);

  log_packet(command_packet.raw, SMALL_PACKET_OUT_SIZE );

  // write_array(command_packet.raw, SMALL_PACKET_OUT_SIZE);
}

void InchanetWallboxComponent::log_packet(const uint8_t *data, size_t size) {
  if (data == nullptr || size == 0) {
    return;
  }

  std::string hex_string;
  // Pre-allocate string to avoid reallocations. Each byte is 3 chars ("XX ").
  hex_string.reserve(size * 3);

  for (size_t i = 0; i < size; ++i) {
    char buffer[4];
    // Use snprintf for safety, it writes a null terminator.
    snprintf(buffer, sizeof(buffer), "%02X ", data[i]);
    hex_string.append(buffer);
  }

  if (!hex_string.empty()) {
    // Remove trailing space for cleaner output
    hex_string.pop_back();
  }

  // Log the resulting string. ESP_LOGD is a macro that calls a printf-like function.
  // Passing the string as an argument is safer than passing it as the format string.
  ESP_LOGD(TAG, "%s", hex_string.c_str());
}

void InchanetWallboxComponent::read_control_packet()
{
  uint8_t byte;
  while (available())
  {
    read_byte(&byte);
    ESP_LOGD(TAG, "%x", byte);
    switch (this->packet_finder_state_)
    {
    case WAITING_FOR_START_BYTE_1:
      if (byte == START_BYTE_1)
      {
        this->packet_finder_state_ = WAITING_FOR_START_BYTE_2;
      }
      break;
    case WAITING_FOR_START_BYTE_2:
      if (byte == START_BYTE_2)
      {
        this->packet_finder_state_ = READING_PACKET;
        this->evse_packet_.raw[0] = START_BYTE_1;
        this->evse_packet_.raw[1] = START_BYTE_2;
        this->bytes_read_ = 2;
      }
      else
      {
        this->packet_finder_state_ = WAITING_FOR_START_BYTE_1;
      }
      break;
    case READING_PACKET:
      this->evse_packet_.raw[this->bytes_read_] = byte;
      this->bytes_read_++;

      if (this->bytes_read_ == USART_BUFFER_IN_SIZE_LONG)
      {
        this->packet_finder_state_ = WAITING_FOR_START_BYTE_1;

        if (this->evse_packet_.fields.length == USART_BUFFER_IN_SIZE_LONG)
        {
          uint32_t computed_crc32 = Calc_CRC32_sw(this->evse_packet_.raw, (USART_BUFFER_IN_SIZE_LONG / 4) - 1);

          if (this->evse_packet_.fields.crc32 == computed_crc32)
          {
            // CRC OK
            this->state_of_electric_vehicle_sensor_->publish_state(this->decode_state_of_ev(this->evse_packet_.fields.state_of_ev));
            this->state_of_charging_sensor_->publish_state(this->decode_state_of_charging(this->evse_packet_.fields.state_of_charging));
            this->warnings_sensor_->publish_state(this->decode_warnings(this->evse_packet_.fields.warnings));
            this->serious_errors_sensor_->publish_state(this->decode_errors(this->evse_packet_.fields.errors));
            this->voltage_l1_sensor_->publish_state(this->evse_packet_.fields.voltage_l1 * 0.25f);
            this->voltage_l2_sensor_->publish_state(this->evse_packet_.fields.voltage_l2 * 0.25f);
            this->voltage_l3_sensor_->publish_state(this->evse_packet_.fields.voltage_l3 * 0.25f);
            this->current_l1_sensor_->publish_state(this->evse_packet_.fields.current_l1 * 0.1f);
            this->current_l2_sensor_->publish_state(this->evse_packet_.fields.current_l2 * 0.1f);
            this->current_l3_sensor_->publish_state(this->evse_packet_.fields.current_l3 * 0.1f);
            this->wh_in_this_session_sensor_->publish_state(this->evse_packet_.fields.wh_session);
            this->wh_all_time_sensor_->publish_state(this->evse_packet_.fields.wh_total);
            this->state_of_sockets_lock_sensor_->publish_state(this->decode_state_of_lock(this->evse_packet_.fields.state_of_lock));
            this->evse_temperature_sensor_->publish_state(this->evse_packet_.fields.temperature);
            this->measured_pp_resistance_sensor_->publish_state(this->decode_state_of_PP(this->evse_packet_.fields.state_of_pp_res));
            this->power_factor_l1_sensor_->publish_state(this->evse_packet_.fields.pf_l1 * 0.01f);
            this->power_factor_l2_sensor_->publish_state(this->evse_packet_.fields.pf_l2 * 0.01f);
            this->power_factor_l3_sensor_->publish_state(this->evse_packet_.fields.pf_l3 * 0.01f);
            this->frequency_l1_sensor_->publish_state(this->evse_packet_.fields.freq_l1 * 0.1f);
            this->frequency_l2_sensor_->publish_state(this->evse_packet_.fields.freq_l2 * 0.1f);
            this->frequency_l3_sensor_->publish_state(this->evse_packet_.fields.freq_l3 * 0.1f);
            this->external_current_l1_sensor_->publish_state(this->evse_packet_.fields.ext_curr_l1 * 0.1f);
            this->external_current_l2_sensor_->publish_state(this->evse_packet_.fields.ext_curr_l2 * 0.1f);
            this->external_current_l3_sensor_->publish_state(this->evse_packet_.fields.ext_curr_l3 * 0.1f);

            // char hex_string[sizeof(this->evse_packet_.raw) * 3 + 1];
            // for (int i = 0; i < sizeof(this->evse_packet_.raw); i++)
            // {
            //   sprintf(&hex_string[i * 3], "%02X ", this->evse_packet_.raw[i]);
            // }
            // this->state_sensor_->publish_state(hex_string);
          }
          else
          {
            ESP_LOGW(TAG, "CRC NOK");
          }
        }
        else
        {
          ESP_LOGW(TAG, "Invalid packet header");
        }
      }
      break;
    }
  }
}

void InchanetWallboxComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Inchanet Wallbox Component");
  LOG_PIN("  Flow Control Pin: ", this->flow_control_pin_);
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
  sprintf(buff, "%02X ", state);
  size_t pos = strlen (buff);
  // doplnime textove vysvetlivky
  if (0x00 != (state & 0x01)) {
    sprintf(&buff[pos], ", relay B (phase 2 and/or 3) could not close");
    pos = strlen (buff);
  }
  if (0x00 != (state & 0x02)) {
    sprintf(&buff[pos], ", low voltage or missing phase (doesn't work in US/JP device)");
    pos = strlen (buff);
  }
  if (0x00 != (state & 0x04)) {
    sprintf(&buff[pos], ", charging paused by DLM (including 0-5A from master)");
    pos = strlen (buff);
  }
  if (0x00 != (state & 0x08)) {
    sprintf(&buff[pos], ", charging slows down due to higher temperature (70-80°C)");
    pos = strlen (buff);
  }
  if (0x00 != (state & 0x10)) {
    sprintf(&buff[pos], ", error in communication with external current measurement");
    pos = strlen (buff);
  }
  if (0x00 != (state & 0x20)) {
    sprintf(&buff[pos], ", inicialization underway");
    pos = strlen (buff);
  }
  if (0x00 != (state & 0x40)) {
    sprintf(&buff[pos], ", problem on socket's pin lock");
    pos = strlen (buff);
  }
  if (0x00 != (state & 0x80)) {
    sprintf(&buff[pos], ", receiving data from another communication channel");
    pos = strlen (buff);
  }
  if (pos > 3) {
    buff[3] = '=';
  }
  return buff;
}

std::string InchanetWallboxComponent::decode_errors(uint8_t state) {
  // zakodujeme stav
  char buff[1000];
  sprintf(buff, "%02X ", state);
  size_t pos = strlen (buff);
  // doplnime textove vysvetlivky
  if (0x00 != (state & 0x01)) {
    sprintf(&buff[pos], ", relay stuck closed");
    pos = strlen (buff);
  }
  if (0x00 != (state & 0x02)) {
    sprintf(&buff[pos], ", relay could not close");
    pos = strlen (buff);
  }
  if (0x00 != (state & 0x04)) {
    sprintf(&buff[pos], ", error on RCD");
    pos = strlen (buff);
  }
  if (0x00 != (state & 0x08)) {
    sprintf(&buff[pos], ", error on PE/N wires");
    pos = strlen (buff);
  }
  if (0x00 != (state & 0x10)) {
    sprintf(&buff[pos], ", overvoltage");
    pos = strlen (buff);
  }
  if (0x00 != (state & 0x20)) {
    sprintf(&buff[pos], ", overcurrent");
    pos = strlen (buff);
  }
  if (0x00 != (state & 0x40)) {
    sprintf(&buff[pos], ", temperature too high (80°+)");
    pos = strlen (buff);
  }
  if (0x00 != (state & 0x80)) {
    sprintf(&buff[pos], ", unsupported charging mode (i.e. ventilation needed or error on PWM voltage or input phase shorted)");
    pos = strlen (buff);
  }
  if (pos > 3) {
    buff[3] = '=';
  }
  return buff;
}

bool InchanetWallboxComponent::send_vendor_request_packet(uint16_t request_type, VendorResponsePacket &response) {
    VendorRequestPacket request_packet;
    request_packet.fields.start_byte = 0xCC44;
    request_packet.fields.length = sizeof(VendorRequestPacket);
    request_packet.fields.evse_id = this->evse_id_;
    request_packet.fields.request_type = request_type;
    request_packet.fields.spare = 0;
    request_packet.fields.crc32 = Calc_CRC32_sw(request_packet.raw, (sizeof(VendorRequestPacket) - 4) >> 2);

    if (this->flow_control_pin_ != nullptr)
        this->flow_control_pin_->digital_write(true);

    this->write_array(request_packet.raw, sizeof(VendorRequestPacket));
    this->flush();

    if (this->flow_control_pin_ != nullptr)
        this->flow_control_pin_->digital_write(false);

    log_packet(request_packet.raw, sizeof(VendorRequestPacket));

    uint8_t buffer[sizeof(VendorResponsePacket)];
    size_t bytes_read = 0;
    uint32_t start_time = millis();

    while (bytes_read < sizeof(VendorResponsePacket) && (millis() - start_time) < 1000) {
        if (available()) {
            read_byte(&buffer[bytes_read]);
            bytes_read++;
        }
    }

    if (bytes_read > 0)
      log_packet(buffer, sizeof(VendorResponsePacket));

    if (bytes_read == sizeof(VendorResponsePacket)) {
        memcpy(response.raw, buffer, sizeof(VendorResponsePacket));
        uint32_t computed_crc32 = Calc_CRC32_sw(response.raw, (sizeof(VendorResponsePacket) - 4) >> 2);
        if (response.fields.start_byte == 0xCC44 && response.fields.crc32 == computed_crc32) {
            return true;
        }
    }

    return false;
}

} // inchanet_wallbox
} // esphome
