// (c) 2021-2023 Philipp Ruppel

#include "clock.hpp"
#include "flash.hpp"
#include "gpio.hpp"
#include "hx711.hpp"
#include "ina219.hpp"
#include "led.hpp"
#include "messages.hpp"
#include "net.hpp"
#include "pca9536.hpp"
#include "wire.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdio.h>
#include <string>

#include <array>
#include <initializer_list>

int main() {

  g_reg_phase_shift = 2;

  static constexpr uint32_t led_color_startup = 0x002000;
  static constexpr uint32_t led_color_offline = 0x10;

  static constexpr size_t led_count = 9;

  static constexpr size_t port_count = 8;
  static constexpr size_t device_config_size_32 = sizeof(CameraMessage) / 4;

  static constexpr uint32_t connection_timeout = 10 * 1000 * 1000;

  for (size_t i = 0; i < led_count; i++) {
    LED.setColor(i, 0x002000);
  }
  delay(100);

  struct DeviceInfo {
    CameraMessage config;
    uint32_t last_update = 0;
    uint8_t type = 0;
  };
  std::array<DeviceInfo, port_count> devices;

  bool usb_connected = true;
  uint32_t last_usb_time = 0;

  auto set_leds_offline = []() {
    for (size_t i = 0; i < led_count; i++) {
      LED.setColor(i, led_color_offline);
    }
  };
  set_leds_offline();

  USBPacketReceiver<16> receiver;

  uint32_t frame_time = 0;
  uint32_t last_frame = 0;

  uint32_t last_poll = 0;
  uint32_t poll_delay = 20 * 1000;

  while (true) {

    uint32_t current_time = micros();

    for (size_t idev = 0; idev < devices.size(); idev++) {
      auto &device = devices[idev];
      if (device.type != DeviceType::None) {
        if (current_time - device.last_update > connection_timeout) {
          device.type = DeviceType::None;
        }
      }
    }

    if (usb_connected && current_time - last_usb_time > connection_timeout) {
      usb_connected = false;
      set_leds_offline();
    }

    if (receiver.receive()) {
      auto *packet_data = (const uint32_t *)receiver.data();

      if (!usb_connected) {
        usb_connected = true;
      }
      last_usb_time = current_time;

      uint32_t cmd = packet_data[0];

      if (cmd == 0x5bdf4276 && receiver.size() >= 2) {
        auto *hub_message = (const HubMessage *)(packet_data + 1);
        size_t port = hub_message->port;
        if (port < port_count) {
          auto &device = devices[port];
          device.type = hub_message->type;
          frame_time = hub_message->frametime;
          device.last_update = current_time;
          for (size_t i = 0; i < device_config_size_32; i++) {
            ((uint32_t *)&device.config)[i] = packet_data[i + 2];
          }
        }
      }

      if (cmd == 0x2C43D703) {
        auto *led_message = (const LedMessage *)packet_data;
        for (size_t i = 0; i < led_count; i++) {
          LED.setColor(i, led_message->leds[i * 3 + 0],
                       led_message->leds[i * 3 + 1],
                       led_message->leds[i * 3 + 2]);
        }
      }

      continue;
    }

    if (usb_connected && current_time - last_frame > frame_time) {
      last_frame += frame_time;
      if (current_time - last_frame > frame_time) {
        last_frame = current_time;
      }
      for (size_t idev = 0; idev < devices.size(); idev++) {
        auto &device = devices[idev];
        if (device.type == DeviceType::Camera) {
          device.config.timestamp = current_time;
          update_message_checksum(device.config);
          send_network_message(device.config, idev);
        }
      }
      continue;
    }

    if (usb_connected && current_time - last_poll > poll_delay) {
      last_poll += poll_delay;
      if (current_time - last_poll > poll_delay) {
        last_poll = current_time;
      }
      ReceiverMessage m = {0};
      m.magic = 0x2925;
      m.timestamp = current_time;
      update_message_checksum(m);
      for (size_t idev = 0; idev < devices.size(); idev++) {
        auto &device = devices[idev];
        if (device.type == DeviceType::Radio) {

          send_network_message(m, idev);
        }
      }
      continue;
    }
  }
}
