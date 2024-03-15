// (c) 2023-2024 Philipp Ruppel

#pragma once

#include "master.hpp"
#include "object.hpp"
#include "node.hpp"
#include "message.hpp"

namespace mittenwire {

struct ImageMessage : Message {
  std::vector<uint8_t> data;
  size_t top = 0;
  size_t left = 0;
  size_t width = 0;
  size_t height = 0;
  float temperature = 0.0f;
  uint32_t request_timestamp = 0;
  uint8_t skip = 0;
  bool valid = false;
};

class Camera : public Node {
  ImageMessage image_buffer;
  int32_t prev_offset = -1;
  bool frame_started = false;
  std::chrono::steady_clock::time_point receive_start_time;
  std::function<void(const ImageMessage&)> callback;

 public:
  Camera(const std::function<void(const ImageMessage&)>& callback)
      : callback(callback) {}
  virtual void process(const Packet& message) override;
};

}  // namespace mittenwire
