// (c) 2023-2024 Philipp Ruppel

#pragma once

#include "node.hpp"
#include "master.hpp"
#include "message.hpp"

#include <vector>
#include <memory>
#include <mutex>
#include <deque>

namespace mittenwire {

class Hub : public Object<Hub> {
  struct Impl {
    std::mutex mutex;
    std::vector<std::vector<std::shared_ptr<Node>>> node_map;
  };

  std::shared_ptr<Master> _master;
  std::shared_ptr<Impl> _impl = std::make_shared<Impl>();

 public:
  Hub(const std::shared_ptr<Master>& master);
  ~Hub();
  void connect(size_t channel, const std::shared_ptr<Node>& node);
  void disconnect(size_t channel, const std::shared_ptr<Node>& node);
  // std::unique_ptr<Message> receive();
};

}  // namespace mittenwire