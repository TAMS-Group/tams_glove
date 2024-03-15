// (c) 2021-2023 Philipp Ruppel

#pragma once

#include "wire.hpp"

class PCA9536 {
  Wire *_wire = nullptr;

  uint8_t readReg(uint8_t reg) {
    _wire->start();
    _wire->write8(0x82);
    _wire->write8(reg);
    _wire->stop();

    _wire->start();
    _wire->write8(0x83);
    uint8_t v = _wire->read8();
    _wire->stop();

    return v;
  }

  void writeReg(uint8_t reg, uint8_t value) {
    _wire->start();
    _wire->write8(0x82);
    _wire->write8(reg);
    _wire->write8(value);
    _wire->stop();
  }

public:
  PCA9536(Wire *wire) : _wire(wire) { writeReg(2, 0); }

  uint8_t readPins() { return readReg(0) & 0b1111; }

  void writePins(uint8_t v) { writeReg(1, v); }

  void setDirections(uint8_t v) { writeReg(3, ~v); }

  int readPin(int i) { return ((readPins() & (1 << i)) != 0) ? 1 : 0; }
};