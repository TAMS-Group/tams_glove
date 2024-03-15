// (c) 2023-2024 Philipp Ruppel

#include <camera.hpp>

#include <messages.hpp>
#include <log.hpp>

#include <chrono>
#include <bitset>

namespace mittenwire {

void Camera::process(const Packet &msg) {
  static constexpr size_t packet_size = 72;
  static constexpr size_t payload_size = 64;
  static constexpr size_t payload_words_32 = 64 / 4;

  auto *data8 = (const uint8_t *)msg.data.data();
  auto *data16 = (const uint16_t *)msg.data.data();
  auto *data32 = (const uint32_t *)msg.data.data();
  auto *data64 = (const uint64_t *)msg.data.data();
  size_t len64 = msg.data.size() / 8;

  if (data32[0] == 0xBA2FA166) {
    if (frame_started) {
      if (msg.data.size() != packet_size) {
        MTW_LOG_ERROR("packet size error " << msg.data.size()
                                           << " != " << packet_size << " flags "
                                           << msg.flags);
        image_buffer.valid = false;
        return;
      }

      uint32_t tail = data32[17];

      size_t packet_index = (tail & (1024 * 1024 - 1));

      uint32_t checksum_a = packet_index;
      for (size_t i = 0; i < payload_size; i++) {
        checksum_a = checksum_a * 41 + msg.data[i + 4];
      }
      checksum_a = (checksum_a & 4095);

      uint32_t checksum_b = (tail >> 20);

      size_t offset = packet_index - 1;

      if (offset >= image_buffer.data.size()) {
        MTW_LOG_ERROR("image data offset out of range "
                      << offset << " " << data32[1] << " " << data32[1]);
      }

      if (checksum_a != checksum_b) {
        MTW_LOG_ERROR("data checksum error " << checksum_a
                                             << " != " << checksum_b);
        image_buffer.valid = false;

        return;
      }

      if (image_buffer.data.empty()) {
        receive_start_time = std::chrono::steady_clock::now();
      }

      if (prev_offset >= 0 && offset != prev_offset + 1) {
        MTW_LOG_ERROR("lost " << offset - prev_offset - 1
                              << " image data packets from " << prev_offset
                              << " to " << offset);
        image_buffer.valid = false;
      }
      prev_offset = offset;
      offset *= payload_size;

      if (offset < image_buffer.data.size()) {
        std::memset(image_buffer.data.data() + offset, 0x80,
                    std::min(payload_size, image_buffer.data.size() - offset));

        std::memcpy(image_buffer.data.data() + offset, msg.data.data() + 4,
                    std::min(msg.data.size() - (packet_size - payload_size),
                             image_buffer.data.size() - offset));

      } else {
        MTW_LOG_ERROR("image data offset out of range "
                      << offset << " " << data32[1] << " " << data32[1]);
      }

      size_t packet_count =
          1 + ((image_buffer.width * image_buffer.height + 63) / 64);
      if (packet_index + 1 == packet_count) {
        frame_started = false;

        {
          uint8_t prev = 0;
          for (size_t i = 0; i < image_buffer.data.size(); i++) {
            uint8_t next = image_buffer.data[i] - prev * 31;
            prev = image_buffer.data[i];
            image_buffer.data[i] = next ^ (0b10101010);
          }
        }

        callback(image_buffer);
        image_buffer.data.clear();
      }
    }
    return;
  }

  if (data16[0] == 0x8C53 && msg.data.size() == sizeof(ImageInfo)) {
    auto &info = *(const ImageInfo *)msg.data.data();

    bool checksum_valid = verify_message_checksum(info);

    if (!checksum_valid) {
      ImageInfo i2 = info;
      update_message_checksum(i2);
      MTW_LOG_ERROR("invalid image info checksum " << info.checksum
                                                   << " != " << i2.checksum);
      for (size_t i = 0; i < sizeof(ImageInfo); i++) {
        MTW_LOG_INFO("packet data " << i << " " << std::hex
                                    << (uint32_t)data8[i] << std::dec);
      }
    }

    if (checksum_valid && info.width < 5000 && info.height < 5000 &&
        info.width >= 2 && info.height >= 2) {
      uint32_t skip = info.skip + 1;
      image_buffer.skip = skip;
      image_buffer.left = info.left;
      image_buffer.top = info.top;
      image_buffer.width = info.width / skip;
      image_buffer.height = info.height / skip;
      image_buffer.channel = msg.channel;
      image_buffer.request_timestamp = info.timestamp;
      image_buffer.valid = true;

      image_buffer.data.clear();
      image_buffer.data.resize(image_buffer.width * image_buffer.height, 0xff);

      static std::array<int16_t, 64> thermotable = {
          -58, -56, -54, -52, -45, -44, -43, -42, -41, -40, -39, -38, -37,
          -36, -30, -20, -10, -4,  0,   4,   10,  21,  22,  23,  24,  25,
          26,  27,  28,  29,  40,  50,  60,  70,  76,  80,  81,  82,  83,
          84,  85,  86,  87,  88,  89,  95,  96,  97,  98,  99,  100, 101,
          102, 103, 104, 105, 106, 107, 108, 116, 120, 124, 128, 132,
      };
      image_buffer.temperature = thermotable[info.temperature & 0b111111];

      frame_started = true;
      prev_offset = -1;
    } else {
      image_buffer.valid = false;
      frame_started = false;
      prev_offset = -1;
    }

    return;
  }

  MTW_LOG_ERROR("unknown packet " << std::hex << data16[0] << " " << data16[1]
                                  << std::dec);
}

}  // namespace mittenwire