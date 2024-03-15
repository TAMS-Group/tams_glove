// (c) 2023-2024 Philipp Ruppel

#include <master.hpp>

#include <log.hpp>
#include <utils.hpp>

namespace mittenwire {

constexpr bool g_master_framesync_debug = false;

inline static void scan_message(const Packet &message) {
  if (g_master_framesync_debug) {
    union U {
      struct {
        uint32_t sync32a;
        uint32_t sync32b;
      };
      uint64_t sync64;
      char sync8[9] = "EMARFMAC";
    } u;
    auto *data32 = (const uint32_t *)message.data.data();
    size_t n32 = message.data.size() / 4;
    for (size_t i = 0; i + 1 < n32; i++) {
      if (data32[i] == u.sync32b && data32[i + 1] == u.sync32a) {
        MTW_LOG_INFO("sync1 "
                     << std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::system_clock::now().time_since_epoch())
                            .count());
      }
    }
  }
}

Master::Master(const std::shared_ptr<superspeed::Device> &device,
               size_t buffer_count, size_t buffer_size) {
  _device = device;

  _reader = std::make_shared<superspeed::Reader>(
      device,
      [this](const void *data, size_t size) {
        if (g_master_framesync_debug) {
          union U {
            struct {
              uint32_t sync32a;
              uint32_t sync32b;
            };
            uint64_t sync64;
            char sync8[9] = "EMARFMAC";
          } u;
          auto *data32 = (const uint32_t *)data;
          size_t n32 = size / 4;
          for (size_t i = 0; i + 1 < n32; i++) {
            if (data32[i] == u.sync32b && data32[i + 1] == u.sync32a) {
              MTW_LOG_INFO(
                  "sync0 "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::system_clock::now().time_since_epoch())
                         .count()
                  << " " << i << " " << n32);
            }
          }
        }
        if (size > 0) {
          std::unique_lock<std::mutex> lock(_rx_mutex);
          _rx_buffers.emplace_back((const uint8_t *)data,
                                   (const uint8_t *)data + size);
          if (_rx_buffers.size() > 100000) {
            _rx_buffers.clear();
            MTW_LOG_ERROR("rx buffer overflow");
          }
          _rx_condition.notify_all();
        }
      },
      buffer_count, buffer_size);

  _packet_thread = std::thread([this]() {
    {
      set_current_thread_name("master packet thread");
      std::vector<uint32_t> recv_buffer;
      size_t recv_index = 0;
      size_t transfer_index = 0;
      auto _recv = [this, &recv_buffer, &recv_index, &transfer_index] {
        if (recv_index < recv_buffer.size()) {
          return recv_buffer[recv_index++];
        }
        recv_index = 0;
        recv_buffer.clear();
        {
          std::unique_lock<std::mutex> lock(_rx_mutex);
          while (recv_buffer.empty()) {
            if (_exit_flag) {
              return uint32_t(0);
            }
            if (!_rx_buffers.empty()) {
              auto &b = _rx_buffers.front();
              recv_buffer.resize(b.size() / 4);
              memcpy(recv_buffer.data(), b.data(), recv_buffer.size() * 4);
              _rx_buffers.pop_front();
              transfer_index++;
            } else {
              _rx_condition.wait(lock);
            }
          }
        }
        return recv_buffer[recv_index++];
      };

      struct PacketBuffer {
        std::vector<uint32_t> data;
        size_t start_transfer = 0;
      };

      std::vector<PacketBuffer> packet_buffers(16);

      while (true) {
        uint32_t header = _recv();
        if (_exit_flag) break;
        uint32_t header_masked = (header & 0xffff0000);
        if (header_masked == 0x23010000) {
          size_t len64 = (header & 0xff);
          size_t channel = ((header >> 8) & 15);
          bool end = ((header >> 12) & 1);
          auto &pbuf = packet_buffers.at(channel);
          if (pbuf.data.empty()) {
            pbuf.start_transfer = transfer_index;
          }
          for (size_t i = 0; i < len64; i++) {
            uint32_t v;
            v = _recv();
            pbuf.data.push_back(v);
            v = _recv();
            pbuf.data.push_back(v);
          }
          if (_exit_flag) break;
          if (end) {
            size_t bytecount = pbuf.data.size() * 4;
            Packet msg;
            msg.channel = channel;
            msg.data.resize(bytecount);
            memcpy(msg.data.data(), pbuf.data.data(), bytecount);
            if (pbuf.start_transfer != transfer_index) {
              msg.flags |= 64;
            }
            pbuf.data.clear();
            scan_message(msg);

            ListenerMapPointer listener_map;
            {
              std::unique_lock<std::mutex> lock(_listener_mutex);
              listener_map = _listener_map;
            }
            for (auto &l : *listener_map) {
              l.second(msg);
            }
          }
          if (pbuf.data.size() > 1000000) {
            MTW_LOG_ERROR("pbuf overflow " << channel);
            pbuf.data.clear();
          }
          continue;
        } else {
          MTW_LOG_INFO("resync " << header << " " << header_masked);
        }
      }
    }
    MTW_LOG_INFO("master packet thread exit");
  });
}

Master::~Master() {
  MTW_LOG_INFO("master shutting down tamsnet");
  MTW_LOG_INFO("master notifying threads");
  {
    std::unique_lock<std::mutex> lock(_rx_mutex);
    _exit_flag = true;
    _rx_condition.notify_all();
  }
  MTW_LOG_INFO("master joining rx thread");
  _reader.reset();
  MTW_LOG_INFO("master joining packet thread");
  _packet_thread.join();
  MTW_LOG_INFO("master tamsnet shut down");
}

void Master::add_packet_listener(
    const std::shared_ptr<void> &listener,
    const std::function<void(const Packet &)> &callback) {
  ListenerMap m;
  {
    std::unique_lock<std::mutex> lock(_listener_mutex);
    m = *_listener_map;
  }
  m[listener] = callback;
  {
    std::unique_lock<std::mutex> lock(_listener_mutex);
    _listener_map = std::make_shared<ListenerMap>(m);
  }
}

void Master::remove_packet_listener(const std::shared_ptr<void> &listener) {
  ListenerMap m;
  {
    std::unique_lock<std::mutex> lock(_listener_mutex);
    m = *_listener_map;
  }
  m.erase(listener);
  {
    std::unique_lock<std::mutex> lock(_listener_mutex);
    _listener_map = std::make_shared<ListenerMap>(m);
  }
}

}  // namespace mittenwire