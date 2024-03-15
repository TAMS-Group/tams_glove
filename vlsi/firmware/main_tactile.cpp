// (c) 2021-2023 Philipp Ruppel

#include "clock.hpp"
#include "flash.hpp"
#include "gpio.hpp"
#include "hx711.hpp"
#include "icm42605.hpp"
#include "ina219.hpp"
#include "led.hpp"
#include "nrf24.hpp"
#include "pca9536.hpp"
#include "serial.hpp"
#include "thermometer.hpp"
#include "wire.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdio.h>
#include <string>

#include <initializer_list>

class LongPressDetector {
  uint32_t delay = 0;
  bool prev_pressed = false;
  uint32_t presstime = 0;
  bool initrd = false;

public:
  LongPressDetector(uint32_t delay) : delay(delay) {}
  bool check(bool pressed) {
    if (!pressed) {
      initrd = true;
    }
    if (!initrd) {
      return false;
    }
    uint32_t t = millis();
    if (pressed && !prev_pressed) {
      presstime = t;
    }
    prev_pressed = pressed;
    return pressed && ((t - presstime) >= delay);
  }
};

class UndervoltageDetector {
  int threshold = 0;
  int debounce = 0;
  int counter = 0;

public:
  UndervoltageDetector(int threshold, int debounce)
      : threshold(threshold), debounce(debounce) {}
  bool check(int voltage) {
    if (voltage < threshold) {
      counter++;
    } else {
      counter = 0;
    }
    return counter >= debounce;
  }
};

int main() {

  LED.setColor(100, 50, 0);

  Serial.begin(1000000);

  const uintptr_t g_tactile_base = 0x70000000;
  volatile auto *g_tactile_config = (volatile uint32_t *)(g_tactile_base);
  volatile auto &g_tactile_freq_swap = g_tactile_config[2];
  volatile auto &g_tactile_freq_base = g_tactile_config[3];
  volatile auto &g_tactile_freq_step = g_tactile_config[4];
  volatile auto &g_tactile_dac_scale = g_tactile_config[5];
  volatile auto &g_tactile_data_fill = g_tactile_config[6];
  volatile auto &g_tactile_data_frame = g_tactile_config[7];
  volatile auto *g_tactile_data =
      (volatile int32_t *)(g_tactile_base + 0x00030000);

  g_tactile_freq_swap = 50;

  g_tactile_freq_step = 25;

  g_tactile_freq_base = 1000;

  g_tactile_dac_scale = 205;

  int nrf_pin_ce = 6;
  int nrf_pin_csn = 5;
  int nrf_pin_sck = 4;
  int nrf_pin_mosi = 3;
  int nrf_pin_miso = 2;
  NRF24 radio(nrf_pin_ce, nrf_pin_csn, nrf_pin_sck, nrf_pin_mosi, nrf_pin_miso);
  radio.begin(false);

  Wire wire(0, 1);

  INA219 ina(&wire);
  PCA9536 pca(&wire);

  ICM42605 imu(&wire);

  const uint32_t full_voltage = 4200;
  const uint32_t undervoltage_threshold = 3300;
  const uint32_t voltage_yellow = 3500;
  const uint32_t voltage_red = 3400;

  LongPressDetector power_off_long_press(1000);
  UndervoltageDetector undervoltage_detector(undervoltage_threshold, 5);

  uint32_t prev_swap_frame = 0;

  uint32_t stat_time = 100;
  uint32_t time_stat = 0;

  const uint32_t tac_word_count = 16 * 16;
  uint32_t tac_word_index = 0;
  uint32_t tac_packet_index = 0;

  while (true) {

    uint32_t time = millis();

    if (uint32_t(time - time_stat) > uint32_t(stat_time)) {

      time_stat = time;

      auto pins = pca.readPins();
      bool button_pressed = ((pins & (1 << 0)) != 0);
      bool bat_on = ((pins & (1 << 1)) != 0);
      bool charging = !((pins & (1 << 2)) != 0);
      bool power_present = !((pins & (1 << 3)) != 0);

      int battery_voltage_mv = ina.readBusVoltage_mV();
      int shunt_voltage_mv = ina.readShuntVoltage_mV();

      uint8_t core_temperature = Thermometer.read();

      if (!power_present && (power_off_long_press.check(button_pressed) ||
                             undervoltage_detector.check(battery_voltage_mv))) {
        LED.setColor(0, 0, 100);
        for (int i = 0; i < 5; i++)
          pca.setDirections(0);
        delay(stat_time + 1);
        continue;
      } else {

        if (button_pressed) {
          LED.setColor(255, 255, 255);
        } else {
          if (power_present) {
            if (charging) {
              LED.setColor(10, 8, 0);
            } else {
              LED.setColor(0, 5, 10);
            }
          } else {
            if (battery_voltage_mv < voltage_red) {
              LED.setColor(10, 0, 0);
            } else if (battery_voltage_mv < voltage_yellow) {
              LED.setColor(10, 8, 0);
            } else {
              LED.setColor(0, 10, 0);
            }
          }
        }

        pca.setDirections(0b0010);
        pca.writePins(0b0010);
      }

      {
        struct Packet {

          uint32_t id;

          uint32_t millis;

          uint16_t battery_voltage_mv;
          int16_t shunt_voltage_mv;

          uint8_t core_temperature;
          uint8_t flags;
          uint16_t reserved1;

          int16_t imu_temp_accel_gyro[7];
          uint16_t reserved2;
        };

        Packet packet = {0};

        packet.id = 0x8da0cdef;

        packet.millis = millis();

        packet.battery_voltage_mv = battery_voltage_mv;
        packet.shunt_voltage_mv = shunt_voltage_mv;
        packet.core_temperature = core_temperature;
        packet.flags = 0;
        packet.flags |= (button_pressed ? 1 : 0);
        packet.flags |= (bat_on ? 2 : 0);
        packet.flags |= (charging ? 4 : 0);
        packet.flags |= (power_present ? 8 : 0);

        imu.readTempAccelGyro(packet.imu_temp_accel_gyro);

        radio.transmit(&packet);
      }
    }

    {
      uint32_t frame = g_tactile_data_frame;
      if (frame != prev_swap_frame) {
        prev_swap_frame = frame;
        tac_word_index = 0;
        tac_packet_index = 0;
      }
    }

    if (tac_word_index < tac_word_count) {

      uint32_t packet[8];
      packet[0] = (0x1e662200 | tac_packet_index);
      tac_packet_index++;
      for (size_t j = 0; j < 7; j++) {
        uint32_t v;
        if (tac_word_index < tac_word_count) {
          v = g_tactile_data[tac_word_index++];

        } else {
          v = 0;
        }
        packet[j + 1] = v;
      }
      radio.transmit(packet);
    }
  }
}
