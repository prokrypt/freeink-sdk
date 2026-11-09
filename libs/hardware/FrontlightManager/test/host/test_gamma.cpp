// FrontlightGamma::perceptualDuty16: whole % unchanged, monotone, sub-LSB
// steps.
#include <FrontlightGamma.h>

#include <cassert>
#include <cstdio>

int main() {
  using namespace FrontlightGamma;
  constexpr uint32_t full = 1023;  // X4 Pro: 10-bit
  for (uint32_t p = 0; p <= 100; ++p) assert(perceptualDuty16(p * 256, full) == perceptualDuty(p, full) * 16);
  uint32_t prev = 0, distinct = 0;
  for (uint32_t q = 0; q <= 25600; ++q) {
    const uint32_t d = perceptualDuty16(q, full);
    assert(d >= prev);
    distinct += d != prev;
    prev = d;
  }
  assert(perceptualDuty16(30000, full) == full * 16);  // clamps above 100%
  // 0..1% (0..1 LSB) gets all 16 dither steps; a 10-bit integer duty gives 1.
  for (uint32_t s = 1; s < 16; ++s) {
    bool hit = false;
    for (uint32_t q = 0; q < 256; ++q) hit |= perceptualDuty16(q, full) == s;
    assert(hit);
  }
  std::printf("gamma: ok (%u distinct duty16 values)\n", static_cast<unsigned>(distinct));
  return 0;
}
