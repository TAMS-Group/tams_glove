// (c) 2023-2024 Philipp Ruppel

#pragma once

#include <cstdint>
#include <cstddef>

namespace mittenwire {

struct DeviceType {
  static constexpr uint32_t None = 0;
  static constexpr uint32_t Camera = 1;
  static constexpr uint32_t Tactile = 2;
};

struct HubMessage {
  uint32_t frametime : 20;
  uint32_t port : 3;
  uint32_t type : 2;
};
static_assert(sizeof(HubMessage) == 4);

struct CameraMessage {
  uint16_t magic;
  uint16_t shutter;

  uint32_t delay : 20;
  uint32_t analog_gain_red : 6;
  uint32_t analog_gain_green : 6;

  uint32_t left : 12;
  uint32_t width : 12;
  uint32_t double_gain : 1;
  uint32_t digital_gain : 7;

  uint32_t timestamp;

  uint32_t height : 11;
  uint32_t blacklevel : 12;
  uint32_t analog_gain_blue : 6;
  uint32_t reserved : 3;

  uint32_t skip : 3;
  uint32_t binning : 2;
  uint32_t top : 11;
  uint32_t checksum : 16;
};
static_assert(sizeof(CameraMessage) == 24);

union ImageInfo {
  uint32_t v32[4];
  struct {
    uint16_t magic;
    uint16_t left;

    uint32_t timestamp : 32;

    uint32_t width : 12;
    uint32_t top : 11;
    uint32_t temperature : 6;
    uint32_t skip : 3;

    uint16_t height : 11;
    uint16_t reserved_2 : 5;

    uint16_t checksum;
  };
};
static_assert(sizeof(ImageInfo) == 16);

uint16_t compute_message_checksum(const void *data, size_t size) {
  uint32_t ret = 0x61D209A2;
  for (size_t i = 0; i < size; i++) {
    ret += ((const uint8_t *)data)[i];
    ret *= 0x7549C58F;
  }
  return (ret >> 16) & 0xffff;
}

template <class T>
void update_message_checksum(T &message) {
  message.checksum = 0;
  message.checksum = compute_message_checksum(&message, sizeof(T));
}

template <class T>
bool verify_message_checksum(T message) {
  auto checksum = message.checksum;
  update_message_checksum(message);
  return checksum == message.checksum;
}

}  // namespace mittenwire