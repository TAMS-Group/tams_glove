// (c) 2021-2023 Philipp Ruppel

#pragma once

#include "sccb.hpp"

#include <cstddef>
#include <cstdint>

struct MT9P001 : public SCCB {
  static constexpr uint8_t Chip_Version = 0x00;
  static constexpr uint8_t Row_Start = 0x01;
  static constexpr uint8_t Column_Start = 0x02;
  static constexpr uint8_t Row_Size = 0x03;
  static constexpr uint8_t Column_Size = 0x04;
  static constexpr uint8_t Horizontal_Blanking = 0x05;
  static constexpr uint8_t Vertical_Blanking = 0x06;
  static constexpr uint8_t Output_Control = 0x07;
  static constexpr uint8_t Shutter_Width_Upper = 0x08;
  static constexpr uint8_t Shutter_Width_Lower = 0x09;
  static constexpr uint8_t Pixel_Clock_Control = 0x0A;
  static constexpr uint8_t Restart = 0x0B;
  static constexpr uint8_t Shutter_Delay = 0x0C;
  static constexpr uint8_t Reset = 0x0D;
  static constexpr uint8_t PLL_Control = 0x10;
  static constexpr uint8_t PLL_Config_1 = 0x11;
  static constexpr uint8_t PLL_Config_2 = 0x12;
  static constexpr uint8_t Read_Mode_1 = 0x1E;
  static constexpr uint8_t Read_Mode_2 = 0x20;
  static constexpr uint8_t Row_Address_Mode = 0x22;
  static constexpr uint8_t Column_Address_Mode = 0x23;
  static constexpr uint8_t Green1_Gain = 0x2B;
  static constexpr uint8_t Blue_Gain = 0x2C;
  static constexpr uint8_t Red_Gain = 0x2D;
  static constexpr uint8_t Green2_Gain = 0x2E;
  static constexpr uint8_t Global_Gain = 0x35;

  MT9P001(int pin_scl, int pin_sda) : SCCB(pin_scl, pin_sda, 0xBA) {}
};