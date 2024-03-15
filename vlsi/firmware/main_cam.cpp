// (c) 2021-2023 Philipp Ruppel

// #define SERIAL_UART

#include "gpio.hpp"
#include "messages.hpp"
#include "mt9p001.hpp"
#include "net.hpp"
#include "serial.hpp"
#include "thermometer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

#define PIN_SCCB_SCL 0
#define PIN_SCCB_SDA 1
#define PIN_CAM_TRIGGER 2
#define PIN_CAM_RESET 3

#define PIN_LED1A 4
#define PIN_LED1K 5
#define PIN_LED2A 6
#define PIN_LED2K 7

#define g_ram_controller ((uint32_t *)(void *)0xB1000000)
#define g_ram_config ((uint32_t *)(void *)0xB2000000)
#define g_ram_data32 ((uint32_t *)(void *)0xB3000000)
#define g_ram_data8 ((uint8_t *)(void *)0xB3000000)

#define g_reg_pcam ((uint32_t *)0xB0000000)
#define g_reg_pcam_info_header_lower (g_reg_pcam[1])
#define g_reg_pcam_info_header_upper (g_reg_pcam[2])
#define g_reg_pcam_info_contents_lower (g_reg_pcam[3])
#define g_reg_pcam_info_contents_upper (g_reg_pcam[4])
#define g_reg_pcam_data_header (g_reg_pcam[5])
#define g_reg_pcam_enable_tx (g_reg_pcam[6])
#define g_reg_pcam_packet_count (g_reg_pcam[7])
#define g_reg_pcam_black_level (g_reg_pcam[8])

void main() {

  Serial.begin(1000000);

  pinMode(PIN_LED1A, OUTPUT);
  pinMode(PIN_LED1K, OUTPUT);
  pinMode(PIN_LED2A, OUTPUT);
  pinMode(PIN_LED2K, OUTPUT);

  digitalWrite(PIN_LED1A, 1);
  digitalWrite(PIN_LED1K, 0);
  digitalWrite(PIN_LED2A, 1);
  digitalWrite(PIN_LED2K, 0);

  delay(100);

  g_reg_net_phase_shift = 4;

  Serial.println("reset ram");
  g_ram_controller[0] = 0;
  delay(50);
  g_ram_controller[0] = 1;
  delay(50);

  Serial.println("ram reset done");

  Serial.println("config ram");
  // delay(100);
  // set drive strength
  // g_ram_config[0] = 0b00001010; // 1/4
  g_ram_config[0] = 0b00001001; // 1/2
  // g_ram_config[0] = 0b00001000; // full

  // g_ram_config[8] = 0; // burst

  // latency | slow refresh | no refresh
  g_ram_config[4] = 0b01001100;
  // g_ram_config[4] = 0b0000010;

  Serial.println("ram configured");

  pinMode(PIN_LED1A, OUTPUT);
  pinMode(PIN_LED1K, OUTPUT);
  pinMode(PIN_LED2A, OUTPUT);
  pinMode(PIN_LED2K, OUTPUT);

  digitalWrite(PIN_LED1A, 1);
  digitalWrite(PIN_LED1K, 0);
  digitalWrite(PIN_LED2A, 1);
  digitalWrite(PIN_LED2K, 0);

  delay(50);

  digitalWrite(PIN_LED1A, 0);
  digitalWrite(PIN_LED1K, 1);
  digitalWrite(PIN_LED2A, 0);
  digitalWrite(PIN_LED2K, 1);

  pinMode(PIN_CAM_RESET, OUTPUT); // reset
  digitalWrite(PIN_CAM_RESET, LOW);
  delay(50);
  digitalWrite(PIN_CAM_RESET, HIGH);
  delay(50);

  Serial.println("init camera");

  MT9P001 image_sensor(PIN_SCCB_SCL, PIN_SCCB_SDA);

  pinMode(PIN_CAM_TRIGGER, OUTPUT); // trigger
  digitalWrite(PIN_CAM_TRIGGER, HIGH);

  image_sensor.writeRegister(image_sensor.Read_Mode_1, // 256
                             (1 << 8)                  // snapshot
                                                       //||
                             //(1 << 9) // invert trigger
  );

  Serial.println("camera initialized");

  int frame_index = 0;

  bool led_flag = 0;

  delay(50);

  g_reg_pcam_data_header = 0xBA2FA166;

  g_reg_pcam_enable_tx = 1;

  bool event_pending = false;
  CameraMessage event_message;
  uint32_t event_time = 0;

  uint32_t temperature = 0;

  uint32_t last_width = -1;
  uint32_t last_height = -1;
  uint32_t last_shutter = -1;
  uint32_t last_addrmode = -1;

  uint16_t last_rgain = -1, last_ggain = -1, last_bgain = -1;

  ImageInfo info;

  NetworkPacketReceiver<8> receiver;
  while (true) {

    {
      uint32_t current_time = micros();
      if (receiver.receive()) {

        auto *packet = (const CameraMessage *)receiver.data();

        if (packet->magic == 0xCA62) {

          if (verify_message_checksum(*packet)) {

            event_pending = true;
            event_message = *packet;
            event_time = current_time;

            uint32_t skip = (packet->skip + 1);

            uint32_t left = packet->left;
            uint32_t top = packet->top;
            uint32_t width = packet->width;
            uint32_t height = packet->height;

            info = ImageInfo{0};
            info.magic = 0x8C53;
            info.width = width;
            info.height = height;
            info.left = left;
            info.top = top;
            info.temperature = (Thermometer.read() & 0b111111);
            info.skip = packet->skip;

            info.timestamp = packet->timestamp;
            update_message_checksum(info);

            Serial.print("verification time ");
            Serial.println(micros() - current_time);
          }
        }
      }
    }

    {
      uint32_t current_time = micros();
      if (event_pending && current_time - event_time >= event_message.delay) {

        event_pending = false;
        auto *packet = &event_message;

        uint32_t skip = (packet->skip + 1);

        uint32_t left = packet->left;
        uint32_t top = packet->top;
        uint32_t width = packet->width;
        uint32_t height = packet->height;

        uint32_t width_skip = width / skip;
        uint32_t height_skip = height / skip;

        image_sensor.writeRegister(image_sensor.Column_Start, left);
        image_sensor.writeRegister(image_sensor.Row_Start, top);

        if (width != last_width) {
          last_width = width;
          image_sensor.writeRegister(image_sensor.Column_Size, width - 1);
        }

        if (height != last_height) {
          last_height = height;
          image_sensor.writeRegister(image_sensor.Row_Size, height - 1);
        }

        uint16_t higain =
            (packet->digital_gain << 8) | (packet->double_gain << 6);

        uint16_t rgain = (higain | packet->analog_gain_red);
        uint16_t ggain = (higain | packet->analog_gain_green);
        uint16_t bgain = (higain | packet->analog_gain_blue);

        if (rgain != last_rgain) {
          last_rgain = rgain;
          image_sensor.writeRegister(image_sensor.Red_Gain, rgain);
        }

        if (ggain != last_ggain) {
          last_ggain = ggain;
          image_sensor.writeRegister(image_sensor.Green1_Gain, ggain);
          image_sensor.writeRegister(image_sensor.Green2_Gain, ggain);
        }

        if (bgain != last_bgain) {
          last_bgain = bgain;
          image_sensor.writeRegister(image_sensor.Blue_Gain, bgain);
        }

        if (packet->shutter != last_shutter) {
          last_shutter = packet->shutter;
          image_sensor.writeRegister(image_sensor.Shutter_Width_Lower,
                                     packet->shutter);
        }

        {
          uint32_t addrmode = (packet->skip | (packet->binning << 4));
          if (addrmode != last_addrmode) {
            last_addrmode = addrmode;
            image_sensor.writeRegister(image_sensor.Column_Address_Mode,
                                       addrmode);
            image_sensor.writeRegister(image_sensor.Row_Address_Mode, addrmode);
          }
        }

        g_reg_pcam_info_header_upper = info.v32[0];
        g_reg_pcam_info_header_lower = info.v32[1];
        g_reg_pcam_info_contents_upper = info.v32[2];
        g_reg_pcam_info_contents_lower = info.v32[3];

        g_reg_pcam_packet_count =
            1 + ((uint32_t(width_skip) * uint32_t(height_skip) + 63) / 64);
        g_reg_pcam_black_level = packet->blacklevel;

        digitalWrite(PIN_CAM_TRIGGER, LOW);
        digitalWrite(PIN_CAM_TRIGGER, HIGH);

        led_flag = !led_flag;
        if (led_flag) {
          digitalWrite(PIN_LED1A, 0);
          digitalWrite(PIN_LED1K, 1);
          digitalWrite(PIN_LED2A, 1);
          digitalWrite(PIN_LED2K, 0);
        } else {
          digitalWrite(PIN_LED1A, 1);
          digitalWrite(PIN_LED1K, 0);
          digitalWrite(PIN_LED2A, 0);
          digitalWrite(PIN_LED2K, 1);
        }

        uint32_t ready_time = micros();

        Serial.print("trigger ");
        Serial.print(current_time - event_time);
        Serial.print(" ");
        Serial.print(ready_time - event_time);
        Serial.println();

        temperature = Thermometer.read();
      }
    }
  }
}
