// Compile-only check of the UltraChip LUT DC-balance rule:
//   c++ -std=c++17 -fsyntax-only -I src test/host/test_lut_balance.cpp
#include "lut/UltraChipLutBalance.h"

using namespace freeink::lutbalance;

namespace {

constexpr LutSet du(uint8_t frames, uint8_t kk) {
  const uint8_t selectors[kRows] = {0x00, 0x00, 0x80, 0x40, kk};
  LutSet s{};
  for (uint8_t r = 0; r < kRows; ++r) {
    s.row[r][0] = selectors[r];
    s.row[r][1] = frames;
    s.row[r][5] = 0x01;
  }
  return s;
}

// DU with KK hold: a true transition LUT, but one-way under a complement scrub.
static_assert(balanced(du(3, 0x00), Policy::Transition), "KW -3 / WK +3 round trip balances");
static_assert(!balanced(du(3, 0x00), Policy::Absolute), "KW alone is a one-way push");
// DU with KK re-drive (BlackRedriveLut): held blacks gain +frames every refresh.
static_assert(!balanced(du(3, 0x40), Policy::Transition), "KK re-drive is one-way");

// Absolute row: VDL 2 frames then VDH 2 frames nets zero.
constexpr LutSet absoluteBlack() {
  LutSet s{};
  for (uint8_t r = Ww; r < kRows; ++r) {
    s.row[r][0] = 0x90;  // A = VDL, B = VDH
    s.row[r][1] = 2;
    s.row[r][2] = 2;
    s.row[r][5] = 1;
  }
  return s;
}
static_assert(balanced(absoluteBlack(), Policy::Absolute), "VDL 2 + VDH 2 balances");

// Repeat counts multiply the group; RP=0 must balance on its own.
constexpr LutSet repeated(uint8_t rp) {
  LutSet s = absoluteBlack();
  s.row[Ww][6] = 0x40;  // second group: VDH 1 frame
  s.row[Ww][7] = 1;
  s.row[Ww][11] = rp;
  return s;
}
static_assert(!balanced(repeated(0), Policy::Absolute), "RP=0 group may still run once");
static_assert(!balanced(repeated(2), Policy::Absolute), "+1 x 2 repeats is unbalanced");

// VCOM drive shifts every pixel; VCOM floating (11) does not drive.
constexpr LutSet vcom(uint8_t level) {
  LutSet s = absoluteBlack();
  s.row[Vcom][0] = level;
  s.row[Vcom][1] = 2;
  s.row[Vcom][5] = 1;
  return s;
}
static_assert(!balanced(vcom(0x40), Policy::Absolute), "VCOM VDH offsets every pixel");
static_assert(balanced(vcom(0xC0), Policy::Absolute), "floating VCOM is not drive");

// VDHR has its own amplitude and is rejected.
constexpr LutSet vdhr() {
  LutSet s = absoluteBlack();
  s.row[Kk][0] = 0xD0;  // A = VDHR
  return s;
}
static_assert(!balanced(vdhr(), Policy::Absolute), "VDHR is not modeled");

}  // namespace

int main() { return 0; }
