// (c) 2021-2023 Philipp Ruppel

#pragma once

#include "clock.hpp"
#include <cstddef>
#include <type_traits>

volatile auto *g_thermometer = (uint8_t *)0x40000000;
class ThermometerDevice {
public:
  inline uint8_t read() {
    g_thermometer[4 * 1] = 1;
    while (!(g_thermometer[4 * 2] & 0b10000000)) {
    }
    return g_thermometer[4 * 2];
  }
};

ThermometerDevice Thermometer;
