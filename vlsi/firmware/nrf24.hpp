// (c) 2023 Philipp Ruppel

#include "clock.hpp"
#include "gpio.hpp"

#include <cstddef>

class NRF24 {

  int _pin_ce = -1;
  int _pin_csn = -1;

  int _spi_speed = 2000000;

  int _pin_mosi = -1;
  int _pin_miso = -1;
  int _pin_sck = -1;

  void beginSPI() {

    digitalWrite(_pin_sck, LOW);
    digitalWrite(_pin_mosi, LOW);
    digitalWrite(_pin_csn, LOW);
  }

  void endSPI() {
    digitalWrite(_pin_csn, HIGH);
    digitalWrite(_pin_sck, LOW);
    digitalWrite(_pin_mosi, LOW);
  }

  void writeSPI(uint8_t v) { shiftDataOut(_pin_sck, _pin_mosi, v); }

  uint8_t readSPI() { return shiftDataIn(_pin_sck, _pin_miso); }

public:
  NRF24(int pin_ce, int pin_csn, int pin_sck, int pin_mosi, int pin_miso)
      : _pin_ce(pin_ce), _pin_csn(pin_csn), _pin_sck(pin_sck),
        _pin_mosi(pin_mosi), _pin_miso(pin_miso) {

    pinMode(_pin_ce, OUTPUT);
    digitalWrite(_pin_ce, LOW);

    pinMode(_pin_csn, OUTPUT);
    digitalWrite(_pin_csn, HIGH);

    pinMode(_pin_mosi, OUTPUT);
    digitalWrite(_pin_mosi, LOW);

    pinMode(_pin_miso, INPUT);

    pinMode(_pin_sck, OUTPUT);
    digitalWrite(_pin_sck, LOW);
  }

  void begin(bool prx) {

    delay(100);

    writeRegister(5, 40);

    writeRegister(6, 0b00001110);

    writeRegister(4, 0);

    writeRegister(1, 0);

    writeRegister(0, 0b01111010 | (prx ? 1 : 0));

    writeRegister(3, 0b01);

    writeRegister(0x1C, 0);
    writeRegister(0x1D, 0);

    if (prx) {
      writeRegister(0x0A, "abcde", 5);
      writeRegister(0x11, 32);
      writeRegister(0x02, 0b0001);

    } else {
      writeRegister(0x10, "abcde", 5);
      flushTX();
    }

    digitalWrite(_pin_ce, HIGH);
    delay(100);
    digitalWrite(_pin_ce, LOW);
    delay(100);
    digitalWrite(_pin_ce, HIGH);
  }

  void flushTX() {
    beginSPI();
    writeSPI(0b11100001);
    endSPI();
  }

  uint8_t readStatus() {
    beginSPI();
    uint8_t ret = readSPI();
    endSPI();
    return ret;
  }

  uint8_t readRegister(uint8_t reg) {
    beginSPI();
    writeSPI(reg & 0x1f);
    uint8_t ret = readSPI();
    endSPI();
    return ret;
  }

  void readRegister(uint8_t reg, void *out, size_t len) {
    beginSPI();
    writeSPI(reg & 0x1f);
    uint8_t *out8 = (uint8_t *)out;
    for (size_t i = 0; i < len; i++) {
      out8[i] = readSPI();
    }
    endSPI();
  }

  void writeRegister(uint8_t reg, uint8_t value) {
    beginSPI();
    writeSPI((reg & 0x1f) | 0x20);
    writeSPI(value);
    endSPI();
  }

  void writeRegister(uint8_t reg, const void *in, size_t len) {
    beginSPI();
    writeSPI((reg & 0x1f) | 0x20);
    const uint8_t *in8 = (const uint8_t *)in;
    for (size_t i = 0; i < len; i++) {
      writeSPI(in8[i]);
    }
    endSPI();
  }

  void transmit(const void *data) {

    while ((readStatus() & 0b1) == 0b1) {
    }

    beginSPI();
    writeSPI(0b10100000);
    const uint8_t *data8 = (const uint8_t *)data;
    for (size_t i = 0; i < 32; i++) {
      writeSPI(data8[i]);
    }
    endSPI();
  }

  bool receive(void *buf) {

    auto stat = readStatus();
    if ((stat & 0b1110) == 0b1110) {
      return false;
    }

    beginSPI();
    writeSPI(0b01100001);
    uint8_t *out8 = (uint8_t *)buf;
    for (size_t i = 0; i < 32; i++) {
      out8[i] = readSPI();
    }
    endSPI();

    return true;
  }
};