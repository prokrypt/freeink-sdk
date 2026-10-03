#pragma once

// FreeInk SDK — frontlight brightness curve, Arduino-free so apps and host
// tests can compute the duty FrontlightManager drives (e.g. to weigh LED drain).
//
// Perception-weighted percent -> duty, gamma 1.6554: 16-bit fixed-point table
// of round(65535 * (pct/100)^1.6554). The exponent is log(2*1023)/log(100),
// picked so on a 10-bit duty range 1% rounds up to exactly 1 LSB (the dimmest
// possible light) and every 1% step maps to a distinct duty — gamma 2 was too
// steep and collapsed 1-4% into the same single-LSB duty.

#include <cstdint>

namespace FrontlightGamma {

inline constexpr uint16_t TABLE[101] = {
    0,     32,    101,   197,   318,   460,   622,   803,   1001,  1217,  1449,  1697,  1960,  2237,  2529,
    2835,  3155,  3488,  3834,  4193,  4565,  4949,  5345,  5753,  6173,  6604,  7047,  7502,  7967,  8444,
    8931,  9429,  9938,  10457, 10987, 11527, 12077, 12638, 13208, 13789, 14379, 14979, 15588, 16208, 16836,
    17474, 18122, 18779, 19445, 20120, 20804, 21497, 22200, 22911, 23631, 24360, 25097, 25843, 26598, 27362,
    28134, 28914, 29703, 30500, 31306, 32120, 32942, 33772, 34611, 35457, 36312, 37175, 38045, 38924, 39811,
    40705, 41608, 42518, 43436, 44361, 45295, 46236, 47185, 48141, 49105, 50076, 51055, 52042, 53036, 54037,
    55046, 56062, 57086, 58117, 59155, 60200, 61253, 62313, 63380, 64454, 65535};

// Duty (0..full) for a 0-100 brightness %, at least 1 LSB when lit.
constexpr uint32_t perceptualDuty(uint32_t pct, uint32_t full) {
  if (pct == 0) return 0;
  if (pct > 100) pct = 100;
  const uint32_t duty = (full * TABLE[pct] + 32767u) / 65535u;
  return duty ? duty : 1u;
}

}  // namespace FrontlightGamma
