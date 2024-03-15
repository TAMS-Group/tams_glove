// (c) 2023-2024 Philipp Ruppel

#pragma once

#include <cstddef>

namespace mittenwire {

struct Message {
  size_t channel = 0;
  virtual ~Message() {}
};

}  // namespace mittenwire