// (c) 2021-2023 Philipp Ruppel

#pragma once

#include "gpio.hpp"

#include <cstddef>

class Flash {
  int _cs, _sck, _mosi, _miso;

  void _beginTransaction() {

    digitalWrite(_sck, LOW);
    pinMode(_sck, OUTPUT);

    digitalWrite(_cs, HIGH);
    pinMode(_cs, OUTPUT);

    digitalWrite(_cs, LOW);
  }

  void _write8(uint8_t byte) {
    pinMode(_mosi, OUTPUT);
    for (int i = 0; i < 8; i++) {
      digitalWrite(_sck, LOW);
      digitalWrite(_mosi, (byte & 0x80) ? HIGH : LOW);
      digitalWrite(_sck, HIGH);
      byte <<= 1;
    }
    pinMode(_mosi, INPUT);
  }

  uint8_t _read8() {
    uint8_t ret = 0;
    for (int i = 0; i < 8; i++) {
      ret <<= 1;
      digitalWrite(_sck, HIGH);
      digitalWrite(_sck, LOW);
      ret |= digitalRead(_miso);
    }
    return ret;
  }

  void _readBytes(void *dest, size_t count) {
    for (size_t i = 0; i < count; i++) {
      ((uint8_t *)dest)[i] = _read8();
    }
  }

  void _endTransaction() { digitalWrite(_cs, HIGH); }

  void _enableWrite() {
    _beginTransaction();
    _write8(0x06);
    _endTransaction();
  }

  bool _isBusy() {
    _beginTransaction();
    _write8(0x05);
    uint8_t status = _read8();
    _endTransaction();
    return (status & 1);
  }

  void _waitNotBusy() {
    while (_isBusy()) {
    }
  }

  void _writeAddress24(uint32_t address) {
    _write8(address >> 16);
    _write8(address >> 8);
    _write8(address >> 0);
  }

public:
  Flash(int cs, int sck, int mosi, int miso)
      : _cs(cs), _sck(sck), _miso(miso), _mosi(mosi) {
    digitalWrite(_cs, HIGH);
    pinMode(_cs, OUTPUT);
    pinMode(_sck, INPUT);
    pinMode(_miso, INPUT);
    pinMode(_mosi, INPUT);
  }

  uint16_t readDeviceID() {
    _beginTransaction();
    _write8(0x90);
    _write8(0x00);
    _write8(0x00);
    _write8(0x00);
    uint16_t ret = 0;
    _readBytes(&ret, sizeof(ret));
    _endTransaction();
    return ret;
  }

  void readSlow(uint32_t address, void *buffer, size_t size) {
    _beginTransaction();
    _write8(0x03);
    _writeAddress24(address);
    _readBytes(buffer, size);
    _endTransaction();
  }

  void readFastDual(uint32_t address, void *buffer, size_t size) {
    _beginTransaction();
    _write8(0x3b);
    _writeAddress24(address);
    _read8();
    for (size_t i = 0; i < size; i++) {
      uint8_t byte = 0;
      for (int i = 0; i < 4; i++) {
        byte <<= 1;
        digitalWrite(_sck, HIGH);
        digitalWrite(_sck, LOW);
        byte |= digitalRead(_miso);
        byte <<= 1;
        byte |= digitalRead(_mosi);
      }
      ((uint8_t *)buffer)[i] = byte;
    }
    _endTransaction();
  }

  void read(uint32_t address, void *buffer, size_t size) {
    readFastDual(address, buffer, size);
  }

  void writePage(uint32_t address, const void *data, size_t size) {
    _enableWrite();
    _beginTransaction();
    _write8(0x02);
    _writeAddress24(address);
    for (size_t i = 0; i < size; i++) {
      _write8(((const uint8_t *)data)[i]);
    }
    _endTransaction();
    _waitNotBusy();
  }

  void eraseChip() {
    _enableWrite();
    _beginTransaction();
    _write8(0x60);
    _endTransaction();
    _waitNotBusy();
  }

  void eraseSector(uint32_t address) {
    _enableWrite();
    _beginTransaction();
    _write8(0x20);
    _writeAddress24(address);
    _endTransaction();
    _waitNotBusy();
  }
};