// 2024-2024 Philipp Ruppel

#include "Vshfloat_vl.h"

#include <iostream>
#include <random>

uint32_t pack_sample(int32_t i, int32_t q) {
  uint32_t exp = 0;
  while ((((i & 0xC0000000) == 0xC0000000) || ((i & 0xC0000000) == 0)) &&
         (((q & 0xC0000000) == 0xC0000000) || ((q & 0xC0000000) == 0)) &&
         (exp < 31)) {
    i <<= 1;
    q <<= 1;
    exp++;
  }
  return exp | (uint32_t(i & 0xfff00000)) | (uint32_t(q & 0xfff00000) >> 12);
}

std::pair<int32_t, int32_t> unpack_sample(uint32_t v) {
  uint32_t exp = (v & 0xff);
  int32_t i = (v & 0xfff00000);
  int32_t q = ((v << 12) & 0xfff00000);
  return std::make_pair(i >> exp, q >> exp);
}

#define ASSERT_STR(x) #x

#define ASSERT(x, ...)                                                         \
  if (!(x)) {                                                                  \
    __VA_ARGS__;                                                               \
    throw std::runtime_error(std::string("assertion failed: ") +               \
                             ASSERT_STR(x));                                   \
  }

int main(int argc, char **argv) {

  bool verbose = 0;

  Vshfloat_vl vshfloat;
  vshfloat.clk = 0;
  vshfloat.in_index = 0;
  vshfloat.in_value_i = 0;
  vshfloat.in_value_q = 0;
  vshfloat.in_strobe = 0;
  vshfloat.eval();

  std::cout << std::hex << std::endl;

  std::mt19937 rng(7);
  size_t in = 1000 * 1000 * 1000 * 10;
  for (size_t it = 0; it < in; it++) {

    if (it % 1000000 == 0) {
      std::cout << std::dec;
      std::cout << it << " " << in << " " << it * 100.0 / in << "%"
                << std::endl;
      std::cout << std::hex;
    }

    uint32_t index = rng();

    int32_t xi = (rng() >> (rng() % 40));
    int32_t xq = (rng() >> (rng() % 40));

    uint32_t xpack = pack_sample(xi, xq);
    auto [yi, yq] = unpack_sample(xpack);
    uint32_t ypack = pack_sample(yi, yq);
    if (verbose) {
      std::cout << xi << " " << xq << " " << std::endl;
      std::cout << yi << " " << yq << " " << std::endl;
      std::cout << xpack << std::endl;
      std::cout << ypack << std::endl;
    }
    ASSERT(xpack == ypack);

    vshfloat.in_index = index;
    vshfloat.in_value_i = xi;
    vshfloat.in_value_q = xq;
    vshfloat.in_strobe = 1;
    vshfloat.clk = 1;
    vshfloat.eval();
    vshfloat.in_strobe = 0;
    vshfloat.clk = 0;
    vshfloat.eval();
    while (!vshfloat.out_strobe) {
      vshfloat.clk = 1;
      vshfloat.eval();
      vshfloat.clk = 0;
      vshfloat.eval();
    }
    ASSERT(vshfloat.out_index == index);
    if (verbose) {
      std::cout << vshfloat.out_pack << " " << vshfloat.out_dbg_exp << " "
                << vshfloat.out_dbg_i << " " << vshfloat.out_dbg_q << std::endl;
    }
    ASSERT(vshfloat.out_pack == xpack, std::cout << xi << " " << xq << " "
                                                 << vshfloat.out_pack << " "
                                                 << xpack << std::endl);

    if (verbose) {
      std::cout << std::endl;
    }
  }
}