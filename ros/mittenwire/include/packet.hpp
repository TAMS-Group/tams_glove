// (c) 2023-2024 Philipp Ruppel

#pragma once

#include <stdint.h>
#include <boost/container/small_vector.hpp>
#include <string>

namespace mittenwire {

class Packet {
 public:
  boost::container::small_vector<uint8_t, 128> data;
  uint16_t flags = 0;
  uint16_t channel = 0;
  std::string str() const {
    std::string s;
    for (auto &c : data) {
      if (c == 0) {
        break;
      }
      s.push_back(c & 0x7f);
    }
    return s;
  }
};

}  // namespace mittenwire