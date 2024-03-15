// (c) 2021-2023 Philipp Ruppel

#pragma once

#include "clock.hpp"
#include <cstddef>
#include <type_traits>

#define BIN 2
#define OCT 8
#define DEC 10
#define HEX 16

#ifdef SERIAL_UART
volatile auto &g_serial_wdata = *(uint32_t *)0x40000000;
class SerialBaseImpl {
  uint32_t _cycles_per_baud = 1;
  uint32_t _wdata_cycles = 0x00010000;
  uint32_t _last_send_time = 0;
  uint32_t _wait_cycles = 0;

public:
  void begin(uint32_t rate) {
    g_serial_wdata = 0xffffffff;
    g_serial_wdata = 0xffffffff;
    _cycles_per_baud = clockCyclesPerSecond() / rate;
    _wdata_cycles = ((_cycles_per_baud << 16) | 0b1111110000000001);
    _wait_cycles = _cycles_per_baud * 11;
  }

  void write(uint8_t v) {
    while (cycles() - _last_send_time < _wait_cycles) {
    }
    g_serial_wdata = (_wdata_cycles | (uint32_t(v) << 2));
    _last_send_time = cycles();
  }
};

#else

volatile auto &g_jtag_wdata = *(uint32_t *)0x50000000;
class SerialBaseImpl {
  uint32_t _delay_cycles = 0;

public:
  void begin(uint32_t rate) {
    _delay_cycles = clockCyclesPerSecond() / rate * 8;
  }
  void write(uint8_t v) {
    g_jtag_wdata = v;
    delayCycles(_delay_cycles);
  }
};

#endif

class SerialImpl : public SerialBaseImpl {

public:
  void print(const char *str) {
    while (*str) {
      write(*str);
      str++;
    }
  }

private:
  template <class T> void _print_impl(T v, uint32_t base = 10) {
    if (v < T(0)) {
      write('-');
      v = -v;
    }
    char buffer[32];
    size_t i = 0;
    do {
      uint32_t digit;
      if (base == 16) {
        digit = (v & 0xf);
        v >>= 4;
      } else if (base == 10) {
        digit = v % 10;
        v /= 10;
      } else {
        digit = v % base;
        v /= base;
      }
      if (digit < 10) {
        buffer[i] = ('0' + digit);
      } else {
        buffer[i] = ('A' + (digit - 10));
      }
      i++;
    } while (v);
    do {
      i--;
      write(buffer[i]);
    } while (i > 0);
  }

public:
  template <class T>
  auto print(const T &v, uint32_t base = DEC) ->
      typename std::enable_if<std::is_integral<T>::value, void>::type {
    _print_impl(v, base);
  }

  void println() { write('\n'); }

  template <class... TT> void println(const TT &...tt) {
    print(tt...);
    println();
  }
};

SerialImpl Serial;
