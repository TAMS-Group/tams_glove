// (c) 2021-2023 Philipp Ruppel

#pragma once

#include "clock.hpp"
#include "gpio.hpp"
#include "serial.hpp"

class Wire {

  int _pin_scl = -1;
  int _pin_sda = -1;

  void _set_scl(bool scl) { pinMode(_pin_scl, scl ? INPUT : OUTPUT); }
  void _set_sda(bool sda) { pinMode(_pin_sda, sda ? INPUT : OUTPUT); }
  bool _sda() { return digitalRead(_pin_sda); }

  void _tick() { delayMicroseconds(1); }

  void _write_bit(bool bit) {

    _set_sda(bit ? 1 : 0);
    _tick();
    _set_scl(1);
    _tick();
    _set_scl(0);
    _tick();
  }

  bool _read_bit() {

    _set_sda(1);
    _tick();
    _set_scl(1);
    _tick();
    bool ack = _sda();
    _set_scl(0);
    _tick();

    return ack;
  }

public:
  Wire(int pin_scl, int pin_sda) : _pin_scl(pin_scl), _pin_sda(pin_sda) {
    pinMode(pin_sda, INPUT);
    pinMode(pin_scl, INPUT);
    digitalWrite(pin_sda, LOW);
    digitalWrite(pin_scl, LOW);
  }

  void start() {
    _set_scl(1);
    _set_sda(1);
    _tick();
    _set_sda(0);
    _tick();
    _set_scl(0);
    _tick();
  }

  void stop() {
    _set_scl(0);
    _tick();
    _set_sda(0);
    _tick();
    _set_scl(1);
    _tick();
    _set_sda(1);
    _tick();
  }

  bool write8(uint8_t value) {
    for (size_t i = 0; i < 8; i++) {
      _write_bit((value & 0x80) != 0);
      value = value << 1;
    }
    return _read_bit() == 0;
  }

  uint8_t read8(bool ack = 1) {
    uint8_t ret = 0;
    for (size_t i = 0; i < 8; i++) {
      ret <<= 1;
      ret |= _read_bit();
    }
    _write_bit(!ack);
    _tick();
    return ret;
  }

  uint16_t read16(bool ack = 1) {
    uint16_t ret = 0;
    ret |= read8(ack);
    ret <<= 8;
    ret |= read8(ack);
    return ret;
  }

  bool checkAddress(uint8_t addr) {
    start();
    bool ret = write8(addr);
    stop();
    return ret;
  }

  void scanAddresses() {
    for (int addr = 0; addr < 256; addr += 1) {
      Serial.print(addr, BIN);
      Serial.print(" ");
      Serial.print(addr, HEX);
      Serial.print(" ");
      Serial.print(addr);
      Serial.print(" ");
      Serial.print(checkAddress(addr));
      Serial.println();
    }
  }
};
