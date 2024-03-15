// (c) 2023-2024 Philipp Ruppel

#include <hub.hpp>

#include <log.hpp>

namespace mittenwire {

Hub::Hub(const std::shared_ptr<Master>& master) {
  _master = master;
  auto impl = _impl;
  _master->add_packet_listener(impl, [impl](const Packet& packet) {
    std::vector<std::shared_ptr<Node>> nodes;
    {
      std::unique_lock<std::mutex> lock(impl->mutex);
      if (packet.channel < impl->node_map.size()) {
        nodes = impl->node_map[packet.channel];
      }
    }
    for (auto& n : nodes) {
      n->process(packet);
    }
  });
}

Hub::~Hub() {
  MTW_LOG_INFO("destroying hub");
  _master->remove_packet_listener(_impl);
  MTW_LOG_INFO("hub destroyed");
}

void Hub::connect(size_t channel, const std::shared_ptr<Node>& node) {
  std::unique_lock<std::mutex> lock(_impl->mutex);
  if (_impl->node_map.size() <= channel) {
    _impl->node_map.resize(channel + 1);
  }
  _impl->node_map[channel].push_back(node);
}

void Hub::disconnect(size_t channel, const std::shared_ptr<Node>& node) {
  std::unique_lock<std::mutex> lock(_impl->mutex);
  if (channel < _impl->node_map.size()) {
    auto& v = _impl->node_map[channel];
    v.erase(std::remove(v.begin(), v.end(), node), v.end());
  }
}

}  // namespace mittenwire