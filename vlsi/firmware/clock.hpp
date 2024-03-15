// (c) 2021-2023 Philipp Ruppel

#pragma once

#include "stdint.h"

static constexpr uintptr_t g_clock_base = 0x60000000;

static uint32_t clockCyclesPerSecond() {
  return *(volatile uint32_t *)(g_clock_base + 4 * 1);
}

static uint32_t cycles() {
  return *(volatile uint32_t *)(g_clock_base + 4 * 2);
}

static uint32_t micros() {
  return *(volatile uint32_t *)(g_clock_base + 4 * 3);
}

static uint32_t millis() {
  return *(volatile uint32_t *)(g_clock_base + 4 * 4);
}

static uint32_t clockCyclesPerMicrosecond() {
  return clockCyclesPerSecond() / 1000000;
}

static uint32_t clockCyclesToMicroseconds(uint32_t a) {
  return a / clockCyclesPerMicrosecond();
}

static uint32_t microsecondsToClockCycles(uint32_t a) {
  return a * clockCyclesPerMicrosecond();
}

static void delayCycles(uint32_t wait) {
  uint32_t start = cycles();
  while (cycles() - start < wait) {
  }
}

static void delayMicroseconds(uint32_t wait) {
  uint32_t start = micros();
  while (micros() - start < wait) {
  }
}

static void delay(uint32_t wait) {
  uint32_t start = millis();
  while (millis() - start < wait) {
  }
}
