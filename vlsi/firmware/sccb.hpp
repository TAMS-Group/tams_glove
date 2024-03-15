
// (c) 2021 Philipp Ruppel

#pragma once

#include "clock.hpp"
#include "gpio.hpp"
#include "serial.hpp"

#include <cstddef>

class SCCB {

  int _pin_scl = -1;
  int _pin_sda = -1;

  inline void _set_scl(bool scl) { pinMode(_pin_scl, scl ? INPUT : OUTPUT); }
  inline void _set_sda(bool sda) { pinMode(_pin_sda, sda ? INPUT : OUTPUT); }
  inline bool _sda() { return digitalRead(_pin_sda); }

  inline void _tick() {}

  uint8_t _address = 0;

  void _write_bit(bool bit) {
    _set_sda(bit);
    _tick();
    _set_scl(1);
    _tick();
    _set_scl(0);
    _tick();
  }

  void _start() {
    _set_scl(1);
    _tick();
    _set_sda(1);
    _tick();
    _set_sda(0);
    _tick();
    _set_scl(0);
    _tick();
  }

  void _stop() {
    _set_scl(0);
    _set_sda(0);
    _tick();
    _set_scl(1);
    _tick();
    _set_sda(1);
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

  bool _write_byte(uint8_t value) {
    for (size_t i = 0; i < 8; i++) {
      _write_bit((value & 0x80) != 0);
      value = value << 1;
    }
    return _read_bit() == 0;
  }

  uint8_t _read_byte(bool nack) {
    uint8_t ret = 0;
    for (size_t i = 0; i < 8; i++) {
      ret <<= 1;
      ret |= _read_bit();
    }
    _write_bit(!nack);
    _tick();
    return ret;
  }

public:
  SCCB(int pin_scl, int pin_sda, uint8_t address)
      : _pin_scl(pin_scl), _pin_sda(pin_sda), _address(address) {
    pinMode(pin_sda, INPUT);
    pinMode(pin_scl, INPUT);
    digitalWrite(pin_sda, LOW);
    digitalWrite(pin_scl, LOW);
  }

  uint16_t readRegister(uint8_t reg) {

    _start();
    _write_byte(_address | 0);
    _write_byte(reg);
    _stop();

    _start();
    _write_byte(_address | 1);
    uint16_t v = 0;
    v |= (uint16_t(_read_byte(1)) << 8);
    v |= _read_byte(0);
    _stop();

    return v;
  }

  void writeRegister(uint8_t reg, uint16_t val) {

    for (size_t i = 0; i < 8; i++) {

      _start();

      _write_byte(_address | 0);

      _write_byte(reg);

      _write_byte(val >> 8);
      _write_byte(val & 0xff);

      _stop();

      auto v = readRegister(reg);
      if (v == val) {

        break;
      }
    }
  }

  bool checkAddress(uint8_t addr) {
    _start();
    bool ret = _write_byte(addr);
    _stop();
    return ret;
  }

  void printRegister(uint16_t reg) { auto v = readRegister(reg); }

  void printRegister(const char *name, uint16_t addr) {
    Serial.print(name);
    Serial.print(" ");
    printRegister(addr);
  }

  void scanAddresses() {
    for (uint8_t addr = 1; addr < 128; addr += 2) {
      Serial.print("SCCB scan ");
      Serial.print(addr);
      Serial.print(" ");
      Serial.print(checkAddress(addr));
      Serial.println();
    }
  }
};
