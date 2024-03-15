// (c) 2023-2024 Philipp Ruppel

#pragma once

#include "packet.hpp"
#include "object.hpp"
#include "message.hpp"

namespace mittenwire {

class Node : public Object<Node> {
 public:
  virtual void process(const Packet& message);
};

}  // namespace mittenwire