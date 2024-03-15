// (c) 2021-2023 Philipp Ruppel

#pragma once

#include "wire.hpp"

class INA219 {
  Wire *_wire = nullptr;

  int readReg(int reg) {
    _wire->start();
    _wire->write8(0x80);
    _wire->write8(reg);
    _wire->stop();

    _wire->start();
    _wire->write8(0x81);
    uint16_t v = _wire->read16();
    _wire->stop();

    return v;
  }

public:
  INA219(Wire *wire) : _wire(wire) {}

  int readBusVoltage_mV() { return (readReg(2) >> 3) * 4; }

  int readShuntVoltage_mV() { return (int16_t)readReg(1) / 100; }
};