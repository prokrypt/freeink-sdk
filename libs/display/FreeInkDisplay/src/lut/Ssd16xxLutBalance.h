#pragma once

#include <stddef.h>
#include <stdint.h>

// Compile-time DC-balance check for SSD16xx register LUTs (SSD1677 command
// 0x32 plus its voltage tail). Same rule as UltraChipLutBalance.h: a pixel
// that nets a DC drive on every refresh builds charge that can damage the
// panel. Every table the SSD1677 driver can upload is static_asserted with
// balanced() next to its definition, and setCustomLut() refuses any other.
//
// Layout (110 bytes): VS rows LUT0..LUT3 (pixel levels) and LUT4 (VCOM), 10
// group bytes each (phase A in bits 7:6 .. D in 1:0); then 10 groups of
// [TP_A, TP_B, TP_C, TP_D, RP]; 5 frame-rate bytes; VGH, VSH1, VSH2, VSL, VCOM.
// Source levels: 00 VSS, 01 VSH1, 10 VSL, 11 VSH2. VCOM row: 00 DCVCOM,
// 01 VSH1+DCVCOM, 10 VSL+DCVCOM, 11 floating. A group runs RP+1 times.
//
// Unit: frames x repeats, VSH1 = +1, VSL = -1, which holds only while
// |VSH1| == |VSL|: the tail must set VSH1 0x41 (+15 V) and VSL 0x32 (-15 V).
// VSH2 has its own amplitude and is rejected. The gray rows select a gray
// level, not a transition, so every row must net zero on its own; tolerance
// is zero, as residue adds up with every refresh.
namespace freeink {
namespace ssd16xxbalance {

constexpr size_t kGroups = 10;
constexpr size_t kPixelRows = 4;
constexpr size_t kVcomRow = 4;
constexpr size_t kTimingStart = 5 * kGroups;
constexpr size_t kLutBytes = 110;
constexpr uint8_t kVsh1Plus15V = 0x41;
constexpr uint8_t kVslMinus15V = 0x32;

// Net drive of VS row `row`; false if it uses VSH2 (a pixel row).
constexpr bool rowNet(const unsigned char* lut, size_t row, int32_t& net) {
  net = 0;
  for (size_t g = 0; g < kGroups; ++g) {
    const uint8_t levels = lut[row * kGroups + g];
    const unsigned char* tp = lut + kTimingStart + g * 5;
    int32_t group = 0;
    for (uint8_t phase = 0; phase < 4; ++phase) {
      const uint8_t level = static_cast<uint8_t>((levels >> (6 - 2 * phase)) & 0x03);
      if (level == 0x01) group += tp[phase];
      if (level == 0x02) group -= tp[phase];
      if (level == 0x03 && row != kVcomRow && tp[phase] != 0) return false;
    }
    net += group * (static_cast<int32_t>(tp[4]) + 1);
  }
  return true;
}

template <size_t N>
constexpr bool balanced(const unsigned char (&lut)[N]) {
  if (N < kLutBytes || lut[106] != kVsh1Plus15V || lut[108] != kVslMinus15V) return false;
  int32_t vcom = 0;
  if (!rowNet(lut, kVcomRow, vcom) || vcom != 0) return false;
  for (size_t row = 0; row < kPixelRows; ++row) {
    int32_t net = 0;
    if (!rowNet(lut, row, net) || net != 0) return false;
  }
  return true;
}

}  // namespace ssd16xxbalance
}  // namespace freeink
