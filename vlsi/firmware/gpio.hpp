// (c) 2021-2023 Philipp Ruppel

#pragma once

#include <cstdint>

static constexpr int LOW = 0;
static constexpr int HIGH = 1;

static constexpr int INPUT = 0;
static constexpr int OUTPUT = 1;

static volatile auto *g_base_gpio = (volatile uint32_t *)0x10000000;

static inline void selectInputPin(int pin) { g_base_gpio[7] = pin; }

static inline int readSelectedInputPin() { return g_base_gpio[8]; }

static inline int digitalRead(int pin) {
  selectInputPin(pin);
  return readSelectedInputPin();
}

static inline void selectOutputPin(int pin) { g_base_gpio[5] = pin; }

static inline void writeSelectedOutputPin(int val) { g_base_gpio[6] = val; }

static inline void shiftDataOut(int clock_pin, int data_pin,
                                uint8_t data_value) {
  selectOutputPin(clock_pin);
  writeSelectedOutputPin(LOW);
  g_base_gpio[11] = data_pin;
  g_base_gpio[9] = data_value;
  for (int i = 0; i < 8; i++) {
    g_base_gpio[10] = 1;
    writeSelectedOutputPin(HIGH);
    writeSelectedOutputPin(LOW);
  }
}

static inline uint8_t shiftDataIn(int clock_pin, int data_pin) {
  g_base_gpio[11] = data_pin;
  selectOutputPin(clock_pin);
  writeSelectedOutputPin(LOW);
  for (int i = 0; i < 8; i++) {
    writeSelectedOutputPin(HIGH);
    g_base_gpio[12] = 1;
    writeSelectedOutputPin(LOW);
  }
  return g_base_gpio[13];
}

static inline void digitalWrite(int pin, int val) {
  selectOutputPin(pin);
  writeSelectedOutputPin(val);
}

static inline void pinMode(int pin, int mode) {
  if (pin < 0)
    return;
  int v = g_base_gpio[3];
  if (mode == OUTPUT) {
    v |= (1 << pin);
  }
  if (mode == INPUT) {
    v &= ~(1 << pin);
  }
  g_base_gpio[3] = v;
}
