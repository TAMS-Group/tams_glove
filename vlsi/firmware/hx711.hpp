// (c) 2021-2023 Philipp Ruppel

#pragma once

#include "clock.hpp"
#include "gpio.hpp"

class HX711 {
  int pin_clk = -1;
  int pin_dat = -1;

public:
  HX711(int pin_clk, int pin_dat) : pin_clk(pin_clk), pin_dat(pin_dat) {
    pinMode(pin_clk, OUTPUT);
    pinMode(pin_dat, INPUT);
  }

  bool available() { return (digitalRead(pin_dat) == LOW); }

  int32_t read() {
    uint32_t ret = 0;
    uint32_t mask = (1ul << 31);
    while (!available()) {
    }
    for (int i = 0; i < 25; i++) {
      digitalWrite(pin_clk, HIGH);
      delayMicroseconds(1);
      digitalWrite(pin_clk, LOW);
      delayMicroseconds(1);
      int n = 15;
      int v = 0;
      for (int j = 0; j < n; j++) {
        v += digitalRead(pin_dat);
      }
      if (v > n / 2)
        ret |= mask;
      mask >>= 1;
    }
    while (mask) {
      ret |= mask;
      mask >>= 1;
    }
    return ret;
  }
};
