// (c) 2023-2024 Philipp Ruppel

#pragma once

#include "packet.hpp"
#include "object.hpp"
#include "superspeed.hpp"

#include <ros/ros.h>

#include <boost/container/small_vector.hpp>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>
#include <unordered_map>

namespace mittenwire {

class Master : public Object<Master> {
  std::shared_ptr<superspeed::Device> _device;

  std::deque<std::vector<uint8_t>> _rx_buffers;
  std::thread _rx_thread;
  std::mutex _rx_mutex;
  std::condition_variable _rx_condition;

  std::thread _packet_thread;

  std::shared_ptr<superspeed::Reader> _reader;

  volatile bool _exit_flag = false;

  std::mutex _listener_mutex;
  typedef std::unordered_map<std::shared_ptr<void>,
                             std::function<void(const Packet&)>>
      ListenerMap;
  typedef std::shared_ptr<const ListenerMap> ListenerMapPointer;
  ListenerMapPointer _listener_map = std::make_shared<ListenerMap>();

 public:
  Master(const std::shared_ptr<superspeed::Device>& device, size_t buffer_count,
         size_t buffer_size);
  ~Master();
  void add_packet_listener(const std::shared_ptr<void>& listener,
                           const std::function<void(const Packet&)>& callback);
  void remove_packet_listener(const std::shared_ptr<void>& listener);
};

}  // namespace mittenwire