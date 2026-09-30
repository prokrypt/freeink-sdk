#pragma once

#include <stddef.h>
#include <stdint.h>

// Compile-time DC-balance gate for UltraChip register LUTs (UC8179 R20h-R24h;
// UC8253 uses the same 6-byte group layout). A pixel that sees a net DC drive
// on every refresh builds charge that does not decay between refreshes and can
// permanently damage the panel, so every register LUT a driver uploads must be
// built through checkedTable() / checkedGenerator() below, which static_assert
// the balance rule. writeLutSet() in each driver only accepts CheckedLuts, and
// CheckedLuts can only be made by those two functions.
//
// OTP waveforms (PSR REG=0) are the panel maker's own and are not uploaded, so
// they are outside this gate. Feeding the OTP a fake OLD plane (e.g. the
// target's complement) changes which OTP waveform runs and is not visible here.
//
// Row format, per group of 6 bytes: [levels, TP_A, TP_B, TP_C, TP_D, RP].
// levels = A in bits 7:6, B 5:4, C 3:2, D 1:0. Source rows: 00 GND, 01 VDH,
// 10 VDL, 11 VDHR. VCOM row: 00 VCOM_DC, 01 VDH+VCOM_DC, 10 VDL+VCOM_DC,
// 11 floating. TP = frames, RP = group repeat count.
//
// Unit: frames x repeats, VDH = +1, VDL = -1, which assumes |VDH| == |VDL|
// (UC8179 PWR VDHS == VDLS: 0x3F/0x3F in the direct-gray config, and the
// OTP/reset values on the B/W path). A pixel's drive is source minus VCOM.
// VDHR has its own amplitude and is rejected rather than guessed. RP=0 is
// ambiguous between "skip" and "run once", so a group with RP=0 must balance
// on its own and the rule holds under either reading.
//
// Tolerance is zero: frame counts are integers and any residue adds up with
// every refresh (one 15 V frame per refresh never cancels), so there is no
// safe nonzero allowance.
namespace freeink {
namespace lutbalance {

constexpr size_t kRowBytes = 42;
constexpr size_t kGroupBytes = 6;
constexpr size_t kRows = 5;

// Rows in register order: R20h VCOM, R21h WW, R22h KW, R23h WK, R24h KK.
enum Row : uint8_t { Vcom = 0, Ww = 1, Kw = 2, Wk = 3, Kk = 4 };

struct LutSet {
  uint8_t row[kRows][kRowBytes];
};

enum class Policy : uint8_t {
  // Differential B/W: the OLD plane is the true previous image, so a pixel's
  // history is a chain of KW/WK plus holds. Holds (WW, KK) must be 0 and a
  // round trip KW + WK must be 0, so charge stays bounded by the pixel state.
  Transition,
  // Every row nets 0 on its own. Required for absolute gray (planes select a
  // level, not a transition) and for any refresh whose OLD plane is not the
  // true previous image (complement scrubs), since the LUT row that runs then
  // says nothing about where the pixel came from.
  Absolute,
};

struct RowDrive {
  int32_t net = 0;
  int32_t rpZeroNet = 0;
  bool vdhr = false;
};

constexpr RowDrive rowDrive(const uint8_t* row, bool vcom) {
  RowDrive d;
  for (size_t g = 0; g + kGroupBytes <= kRowBytes; g += kGroupBytes) {
    const uint8_t levels = row[g];
    const uint8_t repeat = row[g + 5];
    int32_t group = 0;
    for (uint8_t phase = 0; phase < 4; ++phase) {
      const uint8_t level = static_cast<uint8_t>((levels >> (6 - 2 * phase)) & 0x03);
      const int32_t frames = row[g + 1 + phase];
      if (level == 0x01) group += frames;
      if (level == 0x02) group -= frames;
      if (level == 0x03 && !vcom && frames != 0) d.vdhr = true;  // VCOM 11 = floating, no drive
    }
    if (repeat == 0) d.rpZeroNet += group;
    d.net += group * repeat;
  }
  return d;
}

// Net drive a pixel on row `r` sees (source minus VCOM); false if unmodeled.
constexpr bool pixelNet(const LutSet& s, uint8_t r, int32_t& net) {
  const RowDrive com = rowDrive(s.row[Vcom], true);
  const RowDrive src = rowDrive(s.row[r], false);
  if (src.vdhr || com.rpZeroNet != 0 || src.rpZeroNet != 0) return false;
  net = src.net - com.net;
  return true;
}

constexpr bool balanced(const LutSet& s, Policy p) {
  int32_t net[kRows] = {};
  for (uint8_t r = Ww; r < kRows; ++r) {
    if (!pixelNet(s, r, net[r])) return false;
  }
  if (rowDrive(s.row[Vcom], true).net != 0) return false;
  if (p == Policy::Absolute) return net[Ww] == 0 && net[Kw] == 0 && net[Wk] == 0 && net[Kk] == 0;
  return net[Ww] == 0 && net[Kk] == 0 && net[Kw] + net[Wk] == 0;
}

using Generator = LutSet (*)(uint8_t frames);

// A generator must balance for every frame count it can be called with.
template <Generator Make>
constexpr bool balancedForAllFrames(Policy p) {
  for (uint16_t f = 0; f <= 0xFF; ++f) {
    if (!balanced(Make(static_cast<uint8_t>(f)), p)) return false;
  }
  return true;
}

// Rows of a table indexed some other way (e.g. by gray level), remapped into
// register order R20h..R24h.
template <size_t N>
constexpr LutSet fromRows(const uint8_t (&rows)[N][kRowBytes], const uint8_t (&order)[kRows]) {
  LutSet s{};
  for (size_t r = 0; r < kRows; ++r)
    for (size_t i = 0; i < kRowBytes; ++i) s.row[r][i] = rows[order[r]][i];
  return s;
}

class CheckedLuts {
 public:
  const uint8_t* row(size_t r) const { return _set->row[r]; }

 private:
  constexpr explicit CheckedLuts(const LutSet* set) : _set(set) {}
  const LutSet* _set;

  template <const LutSet& S, Policy P>
  friend constexpr CheckedLuts checkedTable();
  template <Generator Make, Policy P>
  friend CheckedLuts checkedGenerator(uint8_t frames, LutSet& storage);
  template <const LutSet& S>
  friend constexpr CheckedLuts vendorTable();
};

template <const LutSet& S, Policy P>
constexpr CheckedLuts checkedTable() {
  static_assert(balanced(S, P), "register LUT is not DC balanced (see UltraChipLutBalance.h)");
  return CheckedLuts(&S);
}

// Runtime frame count: the generator is proven for all 256 values at compile
// time, then run into caller-owned storage (210 bytes, one refresh's worth).
template <Generator Make, Policy P>
CheckedLuts checkedGenerator(uint8_t frames, LutSet& storage) {
  static_assert(balancedForAllFrames<Make>(P), "register LUT generator is not DC balanced for every frame count");
  storage = Make(frames);
  return CheckedLuts(&storage);
}

// Byte-exact vendor (OEM firmware) tables that fail the rule. Exempt only by
// an explicit decision; each use must say why in a comment at the call site.
template <const LutSet& S>
constexpr CheckedLuts vendorTable() {
  return CheckedLuts(&S);
}

}  // namespace lutbalance
}  // namespace freeink
