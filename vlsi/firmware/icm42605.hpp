// (c) 2021-2023 Philipp Ruppel

#pragma once

#include "wire.hpp"

#include <array>

class ICM42605 {
  Wire *_wire = nullptr;

public:
  static constexpr uint8_t PWR_MGMT0 = 0x4E;
  static constexpr uint8_t ACCEL_DATA_X1 = 31;
  static constexpr uint8_t TEMP_DATA1 = 0x1D;

  ICM42605(Wire *wire) : _wire(wire) {
    writeRegister(PWR_MGMT0, 1 + 2 + 4 + 8);
  }

  void readMeasurements(uint8_t base, int16_t *data, size_t count) {

    _wire->start();
    _wire->write8(0xD0);
    _wire->write8(base);
    _wire->stop();

    _wire->start();
    _wire->write8(0xD1);
    for (size_t i = 0; i < count; i++) {
      uint16_t upper = _wire->read8(1);
      uint16_t lower = _wire->read8(i + 1 < count);
      data[i] = ((upper << 8) | lower);
    }
    _wire->stop();
  }

  void writeRegister(uint8_t reg, uint8_t value) {
    _wire->start();
    _wire->write8(0xD0);
    _wire->write8(reg);
    _wire->write8(value);
    _wire->stop();
  }

  uint8_t readRegister(uint8_t reg) {

    _wire->start();
    _wire->write8(0xD0);
    _wire->write8(reg);
    _wire->stop();

    _wire->start();
    _wire->write8(0xD1);
    uint8_t v = _wire->read8(0);
    _wire->stop();

    return v;
  }

  std::array<int16_t, 6> readAccelGyro() {
    std::array<int16_t, 6> dat;
    readMeasurements(ACCEL_DATA_X1, dat.data(), dat.size());
    return dat;
  }

  inline void readTempAccelGyro(int16_t *dat) {
    readMeasurements(TEMP_DATA1, dat, 7);
  }
};