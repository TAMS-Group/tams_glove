// (c) 2021-2023 Philipp Ruppel

#include "clock.hpp"
#include "gpio.hpp"
#include "led.hpp"
#include "messages.hpp"
#include "net.hpp"
#include "nrf24.hpp"
#include "serial.hpp"

#include <array>

template <class Element, size_t Size> class RingBuffer {
  size_t _write_pointer = 0;
  size_t _read_pointer = 0;
  std::array<Element, Size> _elements;

  static size_t _next(size_t i) {
    if (i < Size - 1) {
      return i + 1;
    } else {
      return 0;
    }
  }

public:
  bool empty() const { return _read_pointer == _write_pointer; }
  bool full() const { return _next(_write_pointer) == _read_pointer; }
  void push(const Element &value) {
    if (!full()) {
      _elements[_write_pointer] = value;
      _write_pointer = _next(_write_pointer);
    }
  }
  void pop() {
    if (!empty()) {
      _read_pointer = _next(_read_pointer);
    }
  }
  Element &peek() { return _elements[_read_pointer]; }
  const Element &peek() const { return _elements[_read_pointer]; }
  size_t size() const {
    if (_write_pointer >= _read_pointer) {
      return _write_pointer - _read_pointer;
    } else {
      return Size + _write_pointer - _read_pointer;
    }
  }
  size_t capacity() const { return Size; }
};

int main() {

  Serial.begin(1000000);

  Serial.println("hello radio");

  g_reg_net_phase_shift = 4;

  NetworkPacketReceiver<8> receiver;

  NRF24 nrf(9, 10, 7, 8, 5);
  std::array<uint32_t, 10> packbuf;

  nrf.begin(true);

  LED.setColor(0, 0, 50);

  bool ledflag = 0;

  RingBuffer<std::array<uint32_t, 10>, 64> buffer;

  size_t burst = 0;

  size_t irecv = 0;

  uint32_t time_shift = 0;

  for (;;) {

    if (receiver.receive()) {
      uint32_t current_time = micros();
      auto &m = *(const ReceiverMessage *)receiver.data();
      if (m.magic == 0x2925 && verify_message_checksum(m)) {
        time_shift = m.timestamp - current_time;

        burst = buffer.capacity();
      }
    }

    if (nrf.receive(packbuf.data() + 2)) {
      packbuf[0] = 0x79A90F58;
      packbuf[1] = micros() + time_shift;
      for (size_t i = 2; i < packbuf.size(); i += 2) {
        std::swap(packbuf[i], packbuf[i + 1]);
      }
      buffer.push(packbuf);
    }

    if (burst > 0) {
      if (!buffer.empty()) {
        send_network_message(buffer.peek(), 0);
        buffer.pop();
        burst--;
      } else {
        burst = 0;
      }
    }
  }
}