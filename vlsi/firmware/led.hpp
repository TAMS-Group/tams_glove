// (c) 2021-2023 Philipp Ruppel

#pragma once

#include <cstddef>
#include <stdint.h>

static volatile auto *g_reg_led = (uint32_t *)0x20000000;

class LEDController {

  int max(int a, int b) { return a > b ? a : b; }

  int min(int a, int b) { return a < b ? a : b; }

  int abs(int v) { return v >= 0 ? v : -v; }

public:
  void setColor(size_t i, int r, int g, int b) {
    r = min(255, max(0, r));
    g = min(255, max(0, g));
    b = min(255, max(0, b));
    g_reg_led[i] = ((r << 16) | (g << 8) | (b << 0));
  }

  void setColor(size_t i, uint32_t packed) { g_reg_led[i] = packed; }

  void setColor(int r, int g, int b) { setColor(0, r, g, b); }
};

LEDController LED;
