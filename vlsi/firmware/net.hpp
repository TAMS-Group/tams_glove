// (c) 2021-2023 Philipp Ruppel

#pragma once

#include <array>
#include <cstddef>

#define g_reg_net ((uint32_t *)0x70000000)
#define g_reg_net_phase_shift (g_reg_net[6])
template <size_t max_length> class NetworkPacketReceiver {
  std::array<uint32_t, max_length> _buffer;
  size_t _received_count = 0;

public:
  bool receive() {
    while (g_reg_net[4]) {
      if (_received_count < max_length)
        _buffer[_received_count++] = g_reg_net[1];
      if (_received_count < max_length)
        _buffer[_received_count++] = g_reg_net[2];
      bool end = g_reg_net[3];
      g_reg_net[5] = 1;
      if (end) {
        _received_count = 0;
        return true;
      }
    }
    return false;
  }
  const void *data() const { return _buffer.data(); }
  size_t size() const { return _received_count; }
};

#define g_reg_usb_rx ((volatile uint32_t *)0x70000000)
#define g_reg_usb_rx_data (g_reg_usb_rx[1])
#define g_reg_usb_rx_nempty (g_reg_usb_rx[2])
#define g_reg_usb_rx_read (g_reg_usb_rx[3])
template <size_t max_length> class USBPacketReceiver {
  std::array<uint32_t, max_length> _buffer;
  size_t _packet_size = 0;
  size_t _received_count = 0;

public:
  bool receive() {
    while (g_reg_usb_rx_nempty) {
      uint32_t word = g_reg_usb_rx_data;
      g_reg_usb_rx_read = 1;
      if (_received_count < _packet_size) {
        _buffer[_received_count] = word;
        _received_count++;
        if (_received_count == _packet_size) {
          return true;
        }
      } else if ((word & 0xfffffff0) == 0xb1c596c0) {
        _packet_size = std::min(max_length, size_t(word & 0xf));
        _received_count = 0;
      }
    }
    return false;
  }
  const void *data() const { return _buffer.data(); }
  size_t size() const { return _packet_size; }
};

#define g_reg_net_tx ((volatile uint32_t *)0x90000000)
#define g_reg_phase_shift (g_reg_net_tx[5])

void send_network_word(uint32_t a, uint32_t b, uint32_t channel, bool end) {
  while (g_reg_net_tx[4] != 0) {
  }
  g_reg_net_tx[1] = a;
  g_reg_net_tx[2] = b;
  g_reg_net_tx[3] = end;
  g_reg_net_tx[4] = (1 << channel);
}

template <class T>
void send_network_message(const T &message, uint32_t channel) {
  constexpr size_t size32 = sizeof(T) / 4;
  for (size_t iword32 = 0; iword32 < size32; iword32 += 2) {
    send_network_word(
        ((iword32 + 0 < size32) ? ((const uint32_t *)&message)[iword32 + 0]
                                : 0),
        ((iword32 + 1 < size32) ? ((const uint32_t *)&message)[iword32 + 1]
                                : 0),
        channel, (iword32 + 2 >= size32));
  }
}