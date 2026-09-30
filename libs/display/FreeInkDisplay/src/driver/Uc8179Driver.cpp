#include "Uc8179Driver.h"

#include <Arduino.h>
#include <BoardConfig.h>
#include <Logging.h>
#include <string.h>

#include "../lut/UltraChipDirectGrayLuts.h"
#include "../lut/UltraChipLutBalance.h"
#include "FreeInkDisplay.h"
#if defined(BOARD_HAS_PSRAM)
#include <esp_heap_caps.h>
#endif

namespace freeink {
namespace {

// A plane upload is ~60 KB. Writing it one 100-byte row per SPI call spent a
// large share of each upload in per-call overhead, so rows (mirrored, plus the
// white padding for the non-visible gates) are batched into one write per
// chunk. Static: uploads are serialized by the display, and it keeps 4 KB off
// the task stack.
#if CONFIG_IDF_TARGET_ESP32S3
constexpr size_t STREAM_CHUNK_BYTES = 4000;
#else
// C3 (newer X4 UC8179 batches): keep the static footprint small on its tighter heap.
constexpr size_t STREAM_CHUNK_BYTES = 400;
#endif
uint8_t streamChunk[STREAM_CHUNK_BYTES];

template <typename FillRow>
void streamRows(EpdBus& bus, const uint16_t visibleRows, const uint16_t totalRows, const uint16_t wb,
                FillRow&& fillRow) {
  const uint16_t rowsPerChunk = static_cast<uint16_t>(STREAM_CHUNK_BYTES / wb);
  uint16_t pending = 0;
  for (uint16_t i = 0; i < totalRows; i++) {
    uint8_t* dst = streamChunk + static_cast<size_t>(pending) * wb;
    if (i < visibleRows) {
      fillRow(i, dst);
    } else {
      memset(dst, 0xFF, wb);
    }
    if (++pending == rowsPerChunk) {
      bus.rawWriteBytes(streamChunk, static_cast<uint16_t>(pending * wb));
      pending = 0;
    }
  }
  if (pending) bus.rawWriteBytes(streamChunk, static_cast<uint16_t>(pending * wb));
}
// UC8179 command set (UC8179 datasheet + OEM UC8179_800x480 stream, via Ghidra).
constexpr uint8_t CMD_PANEL_SETTING = 0x00;       // PSR
constexpr uint8_t CMD_POWER_OFF = 0x02;           // POF
constexpr uint8_t CMD_PFS = 0x03;                 // PFS (power-off sequence)
constexpr uint8_t CMD_POWER_ON = 0x04;            // PON
constexpr uint8_t CMD_BOOSTER_SOFT_START = 0x06;  // BTST
constexpr uint8_t CMD_DEEP_SLEEP = 0x07;          // DSLP (check code 0xA5)
constexpr uint8_t CMD_DTM1 = 0x10;                // OLD plane in KW mode
constexpr uint8_t CMD_DTM2 = 0x13;                // NEW plane in KW mode
constexpr uint8_t CMD_DISPLAY_REFRESH = 0x12;     // DRF
constexpr uint8_t CMD_PLL_CONTROL = 0x30;         // PLL
constexpr uint8_t CMD_PARTIAL_IN = 0x91;          // PTIN (partial refresh in)
constexpr uint8_t CMD_PARTIAL_OUT = 0x92;         // PTOUT (partial refresh out)
constexpr uint8_t CMD_VCOM_DATA_INTERVAL = 0x50;  // CDI
constexpr uint8_t CMD_RESOLUTION = 0x61;          // TRES
constexpr uint8_t CMD_GATE_SOURCE_START = 0x65;   // GSST (4 data bytes)
constexpr uint8_t CMD_CCSET = 0xE0;               // CCSET (cascade/output enable)
constexpr uint8_t CMD_GATE_SCAN = 0xE1;           // gate-scan selection
constexpr uint8_t CMD_POWER_SAVE = 0xE3;          // PWS (VCOM/source line periods)
constexpr uint8_t CMD_TSC = 0x40;                 // TSC: sense and read the on-chip temperature
constexpr uint8_t CMD_TSSET = 0xE5;               // TSSET (forced temperature; frame-rate lever)

constexpr uint8_t CDI_INTERVAL = 0x07;  // CDI byte1, constant
constexpr uint8_t PLL_40_HZ = 0x05;
constexpr uint8_t PLL_50_HZ = 0x06;

constexpr uint8_t GRAY_LUT_LEN = 42;  // 0x2A data bytes, command sent separately

// Upload sets in register order R20h..R24h, gated by UltraChipLutBalance.h.
// Direct gray rows are VCOM, black, light, dark, white; registers take VCOM,
// white, light, dark, black.
constexpr uint8_t kDirectGrayOrder[lutbalance::kRows] = {0, 4, 2, 3, 1};
constexpr lutbalance::LutSet kDirectGraySet = lutbalance::fromRows(kUltraChipDirectGray, kDirectGrayOrder);

// Smooth gray (setSmoothGray): the same set with the white (WW) and black (KK)
// level bytes grounded, frame counts and repeats kept. The B/W base of the page
// is on the panel, so black/white pixels hold and only gray pixels swing (no
// full-screen flash). The dark-gray (WK) row is grounded too: dark pixels are
// black in the base and stay black, so only light gray is drawn (bolder text,
// user pick 10:21 9/30). ponytail: holding keeps what the OTP Fast base left
// (same ghosting as a B/W page turn); the app turns it off on image pages.
constexpr lutbalance::LutSet makeDirectGrayHold() {
  lutbalance::LutSet s = kDirectGraySet;
  for (size_t g = 0; g < lutbalance::kRowBytes; g += lutbalance::kGroupBytes) {
    s.row[lutbalance::Ww][g] = 0;
    s.row[lutbalance::Wk][g] = 0;
    s.row[lutbalance::Kk][g] = 0;
  }
  return s;
}
constexpr lutbalance::LutSet kDirectGrayHoldSet = makeDirectGrayHold();

// Balanced DU register LUT: changing pixels get `frames` away from the target
// (invisible: they are already there), then `frames` to it, so every row nets
// zero. VCOM, WW and KK hold: unchanged pixels are not driven. Row:
// [levels (A in bits 7:6; 01 VDH -> black, 10 VDL -> white), TP_A..TP_D, RP].
constexpr lutbalance::LutSet makeDuLuts(const uint8_t frames) {
  constexpr uint8_t selectors[lutbalance::kRows] = {0x00, 0x00, 0x60, 0x90, 0x00};  // VCOM, WW, KW, WK, KK
  lutbalance::LutSet s{};
  for (uint8_t r = 0; r < lutbalance::kRows; ++r) {
    s.row[r][0] = selectors[r];
    s.row[r][1] = frames;
    s.row[r][2] = frames;
    s.row[r][5] = 0x01;
  }
  return s;
}

// Smooth gray paint: held pixels (WW, KK) are otherwise never driven, so held
// blacks faded (status bar, user 10:24 9/30) and held whites dirtied. They get
// a balanced re-drive that ends on their own color, n frames away then n back,
// at the end of the paint so it lands with the other pixels' final push. Every
// row stays 2 x frames long: no added time. ponytail: n=8 is a guess (3 did not
// stop the fade, a full 24+24 blinked); lower it if the re-drive shows.
constexpr uint8_t kHeldRedriveFrames = 8;
constexpr lutbalance::LutSet makeDuRedriveLuts(const uint8_t frames) {
  lutbalance::LutSet s = makeDuLuts(frames);
  const uint8_t n = frames < kHeldRedriveFrames ? frames : kHeldRedriveFrames;
  s.row[lutbalance::Kk][0] = 0x09;  // A, B ground; C 10 VDL (white), D 01 VDH (black)
  s.row[lutbalance::Ww][0] = 0x06;  // A, B ground; C 01 VDH (black), D 10 VDL (white)
  for (const uint8_t r : {lutbalance::Ww, lutbalance::Kk}) {
    s.row[r][1] = static_cast<uint8_t>(frames - n);
    s.row[r][2] = static_cast<uint8_t>(frames - n);
    s.row[r][3] = n;
    s.row[r][4] = n;
  }
  return s;
}

// The only LUT register writer in this driver: takes gated sets only.
void writeLutSet(EpdBus& bus, const lutbalance::CheckedLuts& luts) {
  for (uint8_t r = 0; r < lutbalance::kRows; ++r) {
    bus.cmd(static_cast<uint8_t>(0x20 + r));
    bus.data(luts.row(r), GRAY_LUT_LEN);
  }
}

// Register LUTs (PSR REG=1) drive with the PWR/VCOM_DC registers; OTP waveforms
// carry their own voltages and VCOM per temperature range (UC8179c datasheet,
// LUT format in OTP). After a reset these registers hold VDH/VDL +-14 V and
// VCOM -0.10 V (R01h/R82h defaults), so every register-LUT refresh loads the
// OEM gray_full packet's +-15 V and `vcom` (Uc8179Driver::vcomDc).
void writeRegisterLutPower(EpdBus& bus, const uint8_t vcom) {
  const auto* config = kUc8179DirectGrayConfig;
  bus.cmd(0x01);
  bus.data(0x17);
  bus.data(static_cast<uint8_t>(config[0] & 7));
  bus.data(static_cast<uint8_t>(config[1] & 0x3F));
  bus.data(static_cast<uint8_t>(config[2] & 0x3F));
  bus.data(static_cast<uint8_t>(config[3] & 0x3F));
  bus.cmd(0x82);
  bus.data(vcom);
}

// EXPERIMENT (test/kbd-uc8179): state set by the app between refreshes.
Uc8179KbdExperiment gKbdExp;
bool gKbdExpOn = false;
bool gHalfNext = false;
bool gDuScrubNext = false;  // next Fast refresh: full-panel DU scrub (see displayStart)
uint8_t gHalfScrubFrames = 0;  // next Half refresh: DU scrub with these LUT frames (0 = GC Half)
Uc8179KbdTiming gKbdTiming;
unsigned long gUploadStartMs = 0;
unsigned long gDrfStartMs = 0;
bool gExpActive = false;  // this refresh runs the experiment
bool gExpPll = false;     // this refresh changed the PLL

#if FREEINK_UC8179_PANEL_TEMP
// Last on-chip temperature sample. The TSC conversion holds BUSY ~106 ms and
// the read re-attaches SPI, so it runs while the panel idles (before the idle
// booster-off, at most once per PANEL_TEMP_PERIOD_MS). A panel that never idles
// that long still samples after a refresh once per PANEL_TEMP_REFRESH_PERIOD_MS.
constexpr unsigned long PANEL_TEMP_PERIOD_MS = 60000;
constexpr unsigned long PANEL_TEMP_REFRESH_PERIOD_MS = 300000;
// Below this panel temperature the forced 30 C TSSET of full refreshes is
// replaced by the measured value (the OTP then picks its longer cold waveform)
// and Half-as-scrub gets a third more LUT frames. Samples older than
// PANEL_TEMP_MAX_AGE_MS are ignored.
constexpr int8_t PANEL_COLD_C = 15;
constexpr unsigned long PANEL_TEMP_MAX_AGE_MS = 600000;
int8_t gPanelTempC = 0;
unsigned long gPanelTempMs = 0;
unsigned long gPanelTempTryMs = 0;
bool gPanelTempTried = false;
bool gPanelTempValid = false;
#endif
}  // namespace

void setUc8179KbdExperiment(const Uc8179KbdExperiment* experiment) {
  gKbdExpOn = experiment != nullptr;
  if (experiment) gKbdExp = *experiment;
}

void requestUc8179HalfNext() { gHalfNext = true; }

void requestUc8179DuScrubNext() { gDuScrubNext = true; }

void requestUc8179HalfAsDuScrubNext(const uint8_t frames) { gHalfScrubFrames = frames; }

Uc8179KbdTiming uc8179KbdTiming() { return gKbdTiming; }

#if FREEINK_UC8179_PANEL_TEMP
bool uc8179PanelTemperature(int8_t& celsius, uint32_t& ageMs) {
  if (!gPanelTempValid) return false;
  celsius = gPanelTempC;
  ageMs = static_cast<uint32_t>(millis() - gPanelTempMs);
  return true;
}
#endif

namespace {
// True with the temperature when a recent sample says the panel is cold.
bool coldPanel(int8_t& celsius) {
#if FREEINK_UC8179_PANEL_TEMP
  if (!gPanelTempValid || millis() - gPanelTempMs > PANEL_TEMP_MAX_AGE_MS) return false;
  celsius = gPanelTempC;
  return gPanelTempC < PANEL_COLD_C;
#else
  (void)celsius;
  return false;
#endif
}

// Register-LUT DU frames get a third more on a cold panel (slower particles).
uint8_t coldScaledFrames(const uint8_t frames) {
  int8_t celsius = 0;
  return coldPanel(celsius) ? static_cast<uint8_t>(frames + frames / 3) : frames;
}
}  // namespace

const Uc8179Config& uc8179DefaultConfig() {
  static const Uc8179Config cfg = {
      0x3F,                      // psr0: 0x3B + SHL for FreeInk's framebuffer orientation;
                                 // refresh re-asserts psr0 & 0xDF = 0x1F (OTP + SHL)
      0x0A,                      // psr1
      0x20,                      // pfs (0x03 power-off sequence)
      {0x25, 0x25, 0x3C, 0x25},  // btst (0x06 booster soft-start)
      0x02,                      // gateScan (0xE1)
      0x02,                      // ccset (0xE0)
      0x1E,                      // tsset (0xE5) full refresh (forced-temperature value)
      0x5A,                      // tssetFast (0xE5) fast refresh — REQUIRED: this is the
                                 // frame-rate lever that makes the partial shorter (per RE)
      0x29,                      // cdiActive (0x50, during refresh)
      0xA9,                      // cdiIdle (0x50, restored after)
      600,                       // tresHeight — panel addressed 800x600 (480 visible)
      0x22,                      // powerSave (0xE3): VCOM 2 lines, source 2 * 660 ns
  };
  return cfg;
}

// Visible geometry comes from the active BoardProfile (X4 / X4 Pro, 800x480).
Uc8179Driver::Uc8179Driver(const Uc8179Config& cfg)
    : _cfg(cfg),
      _w(BoardConfig::ACTIVE.displayWidth),
      _h(BoardConfig::ACTIVE.displayHeight),
      _wb(BoardConfig::ACTIVE.displayWidth / 8),
      _tresH(cfg.tresHeight),
      _bufferSize(static_cast<uint32_t>(BoardConfig::ACTIVE.displayWidth / 8) * BoardConfig::ACTIVE.displayHeight) {}

uint32_t Uc8179Driver::spiHz() const {
  // UC8179 serial write timing is rated to 20 MHz. Where the SD card is on
  // native SDMMC (X4 Pro) nothing else shares this bus and the plane uploads
  // dominate an AA page turn, so run at the rated clock. Newer C3 X4 batches
  // also carry a UC8179 but share the bus with the SPI SD card: keep the board
  // default there.
  if (BoardConfig::ACTIVE.sdmmc.busWidth != 0) return 20000000;
  return BoardConfig::ACTIVE.displaySpiHz != 0 ? BoardConfig::ACTIVE.displaySpiHz : 16000000;
}

PanelGeometry Uc8179Driver::geometry() const { return {_w, _h, _wb, _bufferSize}; }

// The OEM init (FUN_4214dff8): PSR, TRES (800x600), GSST, PFS, BTST, E1. No plane
// fill, no CDI/VCOM here — those are (re)asserted per refresh. OTP waveforms
// (PSR REG bit cleared at refresh), so no LUT upload.
void Uc8179Driver::initController(EpdBus& bus) {
  bus.cmd(CMD_PANEL_SETTING);
  bus.data(_cfg.psr0);
  bus.data(_cfg.psr1);

  // TRES: HRES (16-bit BE) then VRES (16-bit BE). Width from the visible geometry
  // (800 -> 0x03,0x20), height is the addressed gate count (600 -> 0x02,0x58).
  bus.cmd(CMD_RESOLUTION);
  bus.data(static_cast<uint8_t>((_w >> 8) & 0xFF));
  bus.data(static_cast<uint8_t>(_w & 0xFF));
  bus.data(static_cast<uint8_t>((_tresH >> 8) & 0xFF));
  bus.data(static_cast<uint8_t>(_tresH & 0xFF));

  // GSST is a 4-byte register (S_START, banks, G_START x2); the vendor reference
  // writes all four zero bytes.
  bus.cmd(CMD_GATE_SOURCE_START);
  bus.data(0x00);
  bus.data(0x00);
  bus.data(0x00);
  bus.data(0x00);

  bus.cmd(CMD_PFS);
  bus.data(_cfg.pfs);

  bus.cmd(CMD_BOOSTER_SOFT_START);
  bus.data(_cfg.btst[0]);
  bus.data(_cfg.btst[1]);
  bus.data(_cfg.btst[2]);
  bus.data(_cfg.btst[3]);

  bus.cmd(CMD_GATE_SCAN);
  bus.data(_cfg.gateScan);

  // GxEPD2 added this UC8179 setting specifically for dithered-bitmap
  // stability. Keep it configurable because the X4 Pro uses different glass
  // and a 600-gate scan; zero lets a board preserve its OTP/default behavior.
  if (_cfg.powerSave != 0) {
    bus.cmd(CMD_POWER_SAVE);
    bus.data(_cfg.powerSave);
  }

  _isScreenOn = false;
  _bwPlanesSynced = false;
  _grayBaseValid = false;
  _absoluteGrayPlanes = false;
}

// Switching to direct gray with the panel still powered skips the POF, reset
// and the PON the gray refresh would then need (~210 ms per direct-gray image
// in the .67 serial log). The gray power/PLL registers below then load while
// the charge pumps run. Bench-check the gray levels of a direct-gray image;
// false restores the OEM power cycle.
constexpr bool kDirectGrayKeepPower = true;

void Uc8179Driver::configureDirectGrayscale(EpdBus& bus) {
  if (_directGrayConfigured) return;
  const bool keepPower = kDirectGrayKeepPower && _isScreenOn;
  if (keepPower) {
    bus.waitBusy(" 8179_direct_setup_ready");  // no register writes mid-refresh
  } else {
    if (_isScreenOn) {
      bus.cmd(CMD_POWER_OFF);
      bus.waitBusy(" 8179_direct_setup_POF");
    }
    bus.reset(50);
  }
  initController(bus);
  _isScreenOn = keepPower;  // initController() assumes a freshly reset controller
  // OEM gray_full packet loader (FUN_4214d79c), with the panel's PSR/SHL.
  const auto* config = kUc8179DirectGrayConfig;
  bus.cmd(0x52);
  bus.data(static_cast<uint8_t>((config[0] & 8) | (config[1] >> 6)));
  bus.cmd(0x30);
  bus.data(static_cast<uint8_t>(config[0] >> 4));
  writeRegisterLutPower(bus, vcomDc());
  bus.cmd(0x2A);
  bus.data(static_cast<uint8_t>(config[2] & 0xC0));
  bus.data(config[4]);
  bus.cmdData2(CMD_VCOM_DATA_INTERVAL, _cfg.cdiActive, CDI_INTERVAL);
  _directGrayConfigured = true;
}

void Uc8179Driver::restoreBwConfiguration(EpdBus& bus) {
  if (!_directGrayConfigured) return;
  if (_isScreenOn) {
    bus.cmd(CMD_POWER_OFF);
    bus.waitBusy(" 8179_direct_exit_POF");
  }
  // Reset the full-gray power/PLL registers before the OTP or overlay path.
  bus.reset(50);
  initController(bus);
  _directGrayConfigured = false;
  _needFullClear = true;
  _oldPlaneValid = false;
}

// The panel's VCOM per temperature range is in its OTP (bank0, TR headers at
// 0x049 + n*0xF7, +5 = VCOM_DC; boundaries TB0..TB10 at 0x001, 0x7F ends the
// list; datasheet p.47/49/51). .67 reads -1.80 V at room temperature and
// -2.40 V at 15 C and below, not the gray packet's fixed -2.00 V (log
// 20260930T171300Z-8aa3091b-otpread L6-15). ROTP (RA2h, p.37) only reads.
// Read once at begin(); ~2 KB bit-banged, about 40 ms.
void Uc8179Driver::readOtpVcom(EpdBus& bus) {
  constexpr uint32_t kLen = 0x49 + 11 * 0xF7 + 6;
  auto* otp = static_cast<uint8_t*>(heap_caps_malloc(kLen, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (otp == nullptr) {
    LOG_ERR("EPD", "8179 OTP VCOM: no buffer, using the gray packet's");
    return;
  }
  bus.waitBusy(" 8179_otp_ready");
  bool ok = bus.cmdReadStream(0xA2, otp, kLen) && otp[0] == 0xA5;
  unsigned trs = 0;
  for (; ok && trs < 11 && otp[1 + trs] != 0x7F; ++trs) {
    ok = trs == 0 || static_cast<int8_t>(otp[1 + trs]) > static_cast<int8_t>(otp[trs]);
  }
  for (unsigned n = 0; ok && n <= trs; ++n) {
    _otpVcom[n] = otp[0x49 + n * 0xF7 + 5];
    ok = _otpVcom[n] <= 0x4F;  // -4.05 V, the table's end (p.35)
  }
  memcpy(_otpTb, otp + 1, sizeof(_otpTb));
  free(otp);
  _otpTrs = ok ? static_cast<uint8_t>(trs + 1) : 0;
  if (!ok) {
    LOG_ERR("EPD", "8179 OTP VCOM: unreadable, using the gray packet's");
    return;
  }
  char line[96];
  int n = snprintf(line, sizeof(line), "8179 OTP VCOM (TB C: VCOM_DC):");
  for (unsigned t = 0; t < _otpTrs && n < static_cast<int>(sizeof(line)) - 12; ++t) {
    n += snprintf(line + n, sizeof(line) - n, " %d:%02X", t + 1 < _otpTrs ? static_cast<int8_t>(_otpTb[t]) : 127,
                  _otpVcom[t]);
  }
  LOG_INF("EPD", "%s", line);
}

// OTP VCOM for the panel's last measured temperature (25 C before a sample),
// the gray packet's when the OTP was unreadable.
uint8_t Uc8179Driver::vcomDc() const {
  if (_otpTrs == 0) return kUc8179DirectGrayConfig[5];
  int celsius = 25;
#if FREEINK_UC8179_PANEL_TEMP
  if (gPanelTempValid && millis() - gPanelTempMs <= PANEL_TEMP_MAX_AGE_MS) celsius = gPanelTempC;
#endif
  unsigned tr = 0;
  while (tr + 1 < _otpTrs && celsius > static_cast<int8_t>(_otpTb[tr])) ++tr;
  return _otpVcom[tr];
}

void Uc8179Driver::begin(EpdBus& bus) {
  _oldPlaneStale = false;
  _directGrayOnPanel = false;
  _directGrayConfigured = false;
  _directGrayPass = false;
  _directGrayPlanes = 0;
  _needFullClear = true;
  _oldPlaneValid = false;
#if defined(BOARD_HAS_PSRAM)
  if (_grayBase == nullptr) {
    _grayBase = static_cast<uint8_t*>(heap_caps_malloc(_bufferSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  }
  if (_grayMask == nullptr) {
    _grayMask = static_cast<uint8_t*>(heap_caps_malloc(_bufferSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  }
#endif
  _panelGrayValid = false;
  _bwBaseShown = false;
  bus.reset(50);
  initController(bus);
  if (_otpTrs == 0) readOtpVcom(bus);
}

void Uc8179Driver::display(EpdBus& bus, const uint8_t* fb, const uint8_t* prev, RefreshMode mode, bool turnOff) {
  syncStaleOldPlane(bus);
  displayStart(bus, fb, prev, mode, turnOff);
  displayFinish(bus, fb);
}

// Direct gray drives every pixel absolutely from any prior state, so a B/W
// base under the next gray pass over direct gray is only a second flash (the
// balanced exit paint). Keep the gray on the panel, the controller in its
// direct-gray config, and hand the base to the plane conversion alone. If the
// gray pass is then cancelled, the next refresh still runs the exit paint.
bool Uc8179Driver::skipBaseOverDirectGray(const uint8_t* fb, RefreshMode fallback) {
  // Smooth gray needs the B/W base on the panel, so it keeps the base refresh.
  if (_smoothGray || !_directGrayOnPanel || fallback != RefreshMode::Fast || _grayBase == nullptr) return false;
  memcpy(_grayBase, fb, _bufferSize);
  _grayBaseValid = true;
  _panelGrayValid = false;  // _grayBase no longer holds the displayed gray page's base
  _bwBaseShown = false;
  _absoluteInput = false;
  _directGrayPass = false;
  _directGrayPlanes = 0;
  _absoluteGrayPlanes = false;
  LOG_DBG("EPD", "8179: gray base kept on direct gray, no B/W refresh");
  return true;
}

void Uc8179Driver::displayGrayscaleBase(EpdBus& bus, const uint8_t* fb, RefreshMode fallback, bool turnOff) {
  syncStaleOldPlane(bus);
  if (!fb) return;
  if (skipBaseOverDirectGray(fb, fallback)) return;
  _paintForGrayBase = true;
  display(bus, fb, nullptr, fallback, turnOff);
  _paintForGrayBase = false;
}

bool Uc8179Driver::displayGrayscaleBaseStart(EpdBus& bus, const uint8_t* fb, RefreshMode fallback, bool turnOff) {
  syncStaleOldPlane(bus);
  if (!fb) return false;
  if (skipBaseOverDirectGray(fb, fallback)) return false;
  // Same Overlay setup as beginGrayscale().
  _absoluteInput = false;
  _directGrayPass = false;
  _directGrayPlanes = 0;
  // Same routing as displayGrayscaleBase(); displayStart() leaves its refresh pending.
  _paintForGrayBase = true;
  const bool pending = displayStart(bus, fb, nullptr, fallback, turnOff);
  _paintForGrayBase = false;
  return pending;
}

// Stream a framebuffer into RAM plane `ramCmd`, mirrored vertically via row
// reversal. SHL in PSR handles the horizontal panel direction for FreeInk's
// framebuffer convention. White padding fills the non-visible gates.
void Uc8179Driver::streamPlane(EpdBus& bus, uint8_t ramCmd, const uint8_t* fb, bool invert) {
  const uint16_t wb = _wb;
  const uint16_t h = _h;
  const uint32_t startUs = micros();
  bus.cmd(ramCmd);
  bus.beginTxn();
  streamRows(bus, _h, _tresH, _wb, [&](const uint16_t i, uint8_t* dst) {
    const uint8_t* src = fb + static_cast<uint32_t>(h - 1 - i) * wb;
    if (invert) {
      for (uint16_t x = 0; x < wb; x++) dst[x] = static_cast<uint8_t>(~src[x]);
    } else {
      memcpy(dst, src, wb);
    }
  });
  bus.endTxn();
  _spiUs += micros() - startUs;
  _spiPlanes++;
}

void Uc8179Driver::streamPlaneXor(EpdBus& bus, uint8_t ramCmd, const uint8_t* lhs, const uint8_t* rhs) {
  const uint16_t wb = _wb;
  const uint16_t h = _h;
  const uint32_t startUs = micros();
  bus.cmd(ramCmd);
  bus.beginTxn();
  streamRows(bus, _h, _tresH, _wb, [&](const uint16_t i, uint8_t* dst) {
    const uint32_t offset = static_cast<uint32_t>(h - 1 - i) * wb;
    for (uint16_t x = 0; x < wb; x++) dst[x] = static_cast<uint8_t>(lhs[offset + x] ^ rhs[offset + x]);
  });
  bus.endTxn();
  _spiUs += micros() - startUs;
  _spiPlanes++;
}

bool Uc8179Driver::displayStart(EpdBus& bus, const uint8_t* fb, const uint8_t* prev, RefreshMode mode, bool turnOff) {
  gUploadStartMs = millis();
  if (gHalfNext && mode == RefreshMode::Fast) {
    gHalfNext = false;
    mode = RefreshMode::Half;
    LOG_DBG("EPD", "8179_EXP: half cleanup refresh");
  }
  // DU scrub: a Fast refresh with the keyboard LUT whose OLD plane is the
  // target's complement, so every pixel runs KW or WK once (~250 ms instead of
  // the 1.5 s Half). Needs the T4 LUT; otherwise it is dropped.
  const bool duScrubRequested = gDuScrubNext && mode == RefreshMode::Fast;
  if (duScrubRequested) gDuScrubNext = false;
  // Half as DU scrub: the same complement drive, with its own register LUT so
  // it needs no experiment and no valid OLD plane (DTM1 is the complement).
  uint8_t halfScrubFrames = mode == RefreshMode::Half ? gHalfScrubFrames : 0;
  gHalfScrubFrames = 0;
  if (halfScrubFrames) {
    halfScrubFrames = coldScaledFrames(halfScrubFrames);
    mode = RefreshMode::Fast;
    LOG_DBG("EPD", "8179: Half as DU scrub, %u frames", static_cast<unsigned>(halfScrubFrames));
  }
  syncStaleOldPlane(bus);
  const bool paintDestination = _directGrayOnPanel;
  _directGrayOnPanel = false;
  // After the balanced DU paint the panel shows the B/W target, so a Fast
  // request is done: no OTP Full flash on top (menus, Home, the next page's base).
  const bool paintIsRefresh = paintDestination && mode == RefreshMode::Fast && !halfScrubFrames;
  restoreBwConfiguration(bus);
  _absoluteInput = false;
  _directGrayPass = false;
  _directGrayPlanes = 0;
  (void)prev;
  _bwPlanesSynced = false;
  _absoluteGrayPlanes = false;
  _grayBaseValid = false;
  // Stock derives its B/W base from absolute gray planes (plane0 & plane1).
  // CrossPoint instead displays that B/W base first and then reuses its single
  // framebuffer for transition masks. Preserve the base before returning from
  // this potentially asynchronous entry point so the masks can be converted to
  // stock's absolute selector encoding later.
  // Selective exit paint: after an overlay-path gray page, _grayBase holds its
  // B/W base and _grayMask its gray pixels. Pure B/W pixels are at their base,
  // so they get their true OLD (hold or a real transition); only gray pixels
  // get the target's complement. Built here, before _grayBase is overwritten.
  // Smooth gray never swings held pixels; they get the short re-drive in
  // makeDuRedriveLuts instead.
  const bool selectivePaint = paintDestination && _panelGrayValid && _grayBase != nullptr && _grayMask != nullptr &&
                              fb != nullptr;
  if (selectivePaint) {
    for (uint32_t i = 0; i < _bufferSize; i++) {
      _grayMask[i] = static_cast<uint8_t>((_grayBase[i] & ~_grayMask[i]) | (~fb[i] & _grayMask[i]));
    }
  }
  _panelGrayValid = false;
  if (_grayBase != nullptr && fb != nullptr) {
    memcpy(_grayBase, fb, _bufferSize);
    _grayBaseValid = true;
  }
  // Full and Half use the clearing OTP GC waveform; only an explicit Fast
  // request may use the differential DU partial (PTIN/PTOUT). Half keeps the
  // true previous frame in DTM1 so the OTP runs real transitions and holds. A
  // complement OLD plane would re-run K->W on every white pixel each Half, a
  // one-way drive unless the (unreadable) OTP GC rows net zero.
  //
  // GHOSTING FIX: the OLD plane (0x10) MUST hold the PREVIOUS displayed frame for
  // a partial, not a flat 0xFF. In KW mode the (old,new) pair selects the per-
  // pixel LUT; with old=0xFF only WW/WK fire (white-stays and white->black), so
  // KW (black->white) NEVER runs and last page's text is never erased = heavy
  // ghosting. Feeding the previous frame lets KW clear it. (0x10 is synced to the
  // just-displayed frame in displayFinish; a full refresh reseeds it to white.)
  // Half is the explicit GC clean.
  const bool scrub = (mode == RefreshMode::Half);
  const bool fast = ((mode == RefreshMode::Fast) && !scrub && !_needFullClear && _oldPlaneValid) || halfScrubFrames;

  if (paintDestination) {
    // The panel holds direct gray, so no OLD plane is true: drive every pixel to
    // its B/W target with the complement pair on the balanced DU LUT (every row
    // nets zero), not OTP Fast, whose one-way K->W/W->K rows can't be checked.
    // 6 frames per phase left overlays drawn over direct gray visibly gray
    // (log 20260930T043445Z-ae927686: drawer at 41661), and 12 still left the
    // drawer's new black text a bit gray, which each later OTP Fast then darkened
    // (log 20260930T092637Z-c7ee2000-aa-darken L1722-2019). The OEM gray set
    // ends with ~24 frames to white and ~38 to black. ponytail: 24 per phase
    // before a gray pass; 36 in smooth gray (Softfast) when the paint is the
    // final B/W screen (menus and the reader panels over a gray page, user pick
    // 10:21: Softfast only).
    // Every row of the paint LUT nets zero (Absolute), so a per-pixel mix of
    // true and complement OLD stays balanced.
    if (selectivePaint) {
      streamPlane(bus, CMD_DTM1, _grayMask);
      LOG_DBG("EPD", "8179: selective exit paint (gray pixels + changes only)");
    } else {
      streamPlane(bus, CMD_DTM1, fb, true);
    }
    streamPlane(bus, CMD_DTM2, fb);
    _scrubLutFrames = coldScaledFrames(_smoothGray && !_paintForGrayBase ? 36 : 24);
    _complementOldPlane = true;
    startBwRefresh(bus, true);
    _scrubLutFrames = 0;
    _complementOldPlane = false;
    bus.waitRefreshComplete(" 8179_BW_TARGET_DRF");
    bus.cmd(CMD_PARTIAL_OUT);
    if (paintIsRefresh) {
      bus.cmdData2(CMD_VCOM_DATA_INTERVAL, _cfg.cdiIdle, CDI_INTERVAL);
      streamPlane(bus, CMD_DTM1, fb);
      _oldPlaneValid = true;
      _bwPlanesSynced = true;
      _needFullClear = false;
      _bwBaseShown = true;
      if (turnOff) {
        bus.cmd(CMD_POWER_OFF);
        bus.waitBusy(" 8179_POF");
        _isScreenOn = false;
      }
      return true;
    }
  }

  // A Half-as-scrub keeps the experiment's windows, PLL and resync skip out.
  gExpActive = gKbdExpOn && fast && !paintDestination && !halfScrubFrames;
  const bool duScrub =
      (duScrubRequested && gExpActive && (gKbdExp.flags & Uc8179KbdExperiment::KbdLut)) || halfScrubFrames;
  if (duScrub) LOG_DBG("EPD", "8179_EXP: DU scrub refresh");
  // NEW plane (0x13) = new frame.
  streamPlane(bus, CMD_DTM2, fb);
  if (duScrub) streamPlane(bus, CMD_DTM1, fb, /*invert=*/true);
  // Half with a valid OLD plane keeps it (true transitions). Full, and Half on
  // an unknown panel state, keep the absolute-from-white behavior.
  if (!fast && !(scrub && _oldPlaneValid)) bus.fillPlane(CMD_DTM1, 0xFF, _tresH, _wb);
  // (Ordinary Fast: OLD still holds the previous frame from displayFinish.)

  // A smooth gray base over a B/W panel (the last gray pass was cancelled)
  // takes the balanced DU LUT on the true OLD plane, like the paint, not OTP
  // Fast, whose one-way rows can't be gated and leave new text gray under the
  // held gray pass. Same time as a regular smooth turn's paint.
  const bool smoothBase = _smoothGray && _paintForGrayBase && fast && !duScrub && !gExpActive;
  _scrubLutFrames = smoothBase ? coldScaledFrames(24) : halfScrubFrames;
  _complementOldPlane = duScrub;
  startBwRefresh(bus, fast);
  _scrubLutFrames = 0;
  _complementOldPlane = false;
  _pendingPartial = fast;
  _pendingTurnOff = turnOff;
  _pendingRefresh = true;
  _bwBaseShown = true;  // fb in B/W once this refresh completes (gray passes wait for it)
  return true;
}

void Uc8179Driver::logSpiBeforeDrf(const char* kind) {
  LOG_DBG("EPD", "SPI %s: %u planes %lu ms", kind, static_cast<unsigned>(_spiPlanes),
          static_cast<unsigned long>(_spiUs / 1000));
  _spiUs = 0;
  _spiPlanes = 0;
}

void Uc8179Driver::startBwRefresh(EpdBus& bus, bool fast) {
  // --- Refresh setup (exact OEM order) -----------------------------------------
  bus.cmd(CMD_VCOM_DATA_INTERVAL);
  bus.data(_cfg.cdiActive);  // 0x29
  bus.data(CDI_INTERVAL);
  bus.cmd(CMD_CCSET);
  bus.data(_cfg.ccset);  // 0x02
  bus.cmd(CMD_TSSET);
  // fast 0x5A (frame lever) / full 0x1E, or the measured temperature on a cold
  // panel so the OTP full waveform matches it.
  int8_t celsius = 0;
  const bool coldFull = !fast && coldPanel(celsius);
  bus.data(fast ? _cfg.tssetFast : (coldFull ? static_cast<uint8_t>(celsius < 0 ? 0 : celsius) : _cfg.tsset));
  const bool kbdLut = fast && ((gExpActive && (gKbdExp.flags & Uc8179KbdExperiment::KbdLut)) || _scrubLutFrames);
  bus.cmd(CMD_PANEL_SETTING);
  // EXPERIMENT T4: REG set -> register LUTs below instead of OTP.
  bus.data(static_cast<uint8_t>(kbdLut ? _cfg.psr0 : (_cfg.psr0 & 0xDF)));  // 0x1F: REG cleared -> OTP + SHL
  bus.data(_cfg.psr1);
  if (kbdLut) {
    const uint8_t frames = _scrubLutFrames ? _scrubLutFrames : (gKbdExp.lutFrames ? gKbdExp.lutFrames : 3);
    // A DU scrub loads DTM1 with the target's complement, so the OLD plane is
    // not the pixel's real state and the set must balance per row (Absolute).
    lutbalance::LutSet storage;
    // Smooth gray paints and bases re-drive held pixels (WW, KK); a full DU
    // scrub complements every pixel, so those rows are unused there.
    writeRegisterLutPower(bus, vcomDc());
    writeLutSet(bus,
                _smoothGray && _scrubLutFrames
                    ? lutbalance::checkedGenerator<makeDuRedriveLuts, lutbalance::Policy::Absolute>(frames, storage)
                : _complementOldPlane
                    ? lutbalance::checkedGenerator<makeDuLuts, lutbalance::Policy::Absolute>(frames, storage)
                    : lutbalance::checkedGenerator<makeDuLuts, lutbalance::Policy::Transition>(frames, storage));
  }
  gExpPll = fast && gExpActive && gKbdExp.pll != 0;
  if (gExpPll) {
    bus.cmd(CMD_PLL_CONTROL);
    bus.data(gKbdExp.pll);
  }
  if (fast) {
    // Fast-only: PFS/gate-scan re-assert. Full omits these; without them the OTP
    // waveform runs at the full frame count (same duration + garbled).
    bus.cmd(CMD_PFS);
    bus.data(_cfg.pfs);  // 0x03 <- 0x20
    bus.cmd(CMD_GATE_SCAN);
    bus.data(_cfg.gateScan);  // 0xE1 <- 0x02
  }

  if (!_isScreenOn) {
    bus.cmd(CMD_POWER_ON);
    bus.waitBusy(" 8179_PON");
    _isScreenOn = true;
  }

  if (fast) bus.cmd(CMD_PARTIAL_IN);  // PTIN — whole-panel partial (no 0x90 window)
  gKbdTiming.drfRows = static_cast<uint16_t>(_h);
  logSpiBeforeDrf(fast ? "fast" : "full");
  gDrfStartMs = millis();
  gKbdTiming.uploadMs = static_cast<uint32_t>(gDrfStartMs - gUploadStartMs);
  bus.cmd(CMD_DISPLAY_REFRESH);
  // Confirm the waveform started (BUSY dropped) before returning, so
  // displayFinish() only rides out the completion edge.
  {
    const int8_t busyPin = bus.pins().busy;
    const unsigned long t0 = millis();
    while (digitalRead(busyPin) == HIGH && millis() - t0 < 50) delay(1);
  }
}

#if FREEINK_UC8179_PANEL_TEMP
void Uc8179Driver::samplePanelTemperature(EpdBus& bus, const unsigned long periodMs) {
  const unsigned long now = millis();
  if (gPanelTempTried && now - gPanelTempTryMs < periodMs) return;
  gPanelTempTried = true;
  gPanelTempTryMs = now;
  // TSE (R41h) stays at its power-on default (internal sensor, no offset), so
  // the first data byte is TS[7:0]: signed whole degrees C (datasheet R40h).
  // CCSET TSFIX=1 makes the chip report the forced TSSET value instead of the
  // sensor (RE0h), so clear it for the read only. Gray refreshes do not rewrite
  // CCSET, so it is restored before returning.
  bus.cmd(CMD_CCSET);
  bus.data(static_cast<uint8_t>(_cfg.ccset & ~0x02));
  bus.cmd(CMD_TSC);
  bus.waitBusy(" 8179_TSC");
  uint8_t raw = 0;
  const bool read = bus.readData(&raw, 1);
  bus.cmd(CMD_CCSET);
  bus.data(_cfg.ccset);
  if (!read) {
    LOG_DBG("EPD", "8179 TSC read skipped (shared SPI bus)");
    return;
  }
  gPanelTempC = static_cast<int8_t>(raw);
  gPanelTempMs = now;
  gPanelTempValid = true;
  LOG_DBG("EPD", "8179 TSC raw 0x%02X = %d C", raw, static_cast<int>(gPanelTempC));
}
#endif

void Uc8179Driver::displayFinish(EpdBus& bus, const uint8_t* fb) {
  if (!_pendingRefresh) return;
  _pendingRefresh = false;
  bus.waitRefreshComplete(" 8179_DRF");
  gKbdTiming.doneMs = millis();
  gKbdTiming.drfMs = static_cast<uint32_t>(gKbdTiming.doneMs - gDrfStartMs);
  gKbdTiming.count++;
  if (_pendingPartial) bus.cmd(CMD_PARTIAL_OUT);  // PTOUT closes the partial window
  if (gExpPll) {
    // Power-on default (50 Hz); the OTP path never sets it.
    bus.cmd(CMD_PLL_CONTROL);
    bus.data(PLL_50_HZ);
    gExpPll = false;
  }
  // Restore the idle CDI (border) after the refresh, as the OEM does.
  bus.cmd(CMD_VCOM_DATA_INTERVAL);
  bus.data(_cfg.cdiIdle);  // 0xA9
  bus.data(CDI_INTERVAL);

  // Sync the OLD plane (0x10) with the just-displayed frame so the NEXT partial
  // diffs against it (KW clears erased pixels -> no ghosting). This is the piece
  // that makes fast page turns clean.
  // Keyboard DU frames may leave it to CDI N2OCP (0x29), which copies the
  // full NEW plane to OLD after the refresh: verified on the X4 Pro panel by
  // the Goodies N2OCP probe (log 20260930T094413Z-187d9a2c-n2ocp.txt). Only
  // after a full-frame balanced DU refresh; every other refresh re-streams.
  const bool skipResync = gExpActive && _pendingPartial && (gKbdExp.flags & Uc8179KbdExperiment::KbdLut) &&
                          (gKbdExp.flags & Uc8179KbdExperiment::SkipOldResync);
  gExpActive = false;
  const unsigned long syncStart = millis();
  if (!skipResync) streamPlane(bus, CMD_DTM1, fb);
  gKbdTiming.syncMs = skipResync ? 0 : static_cast<uint32_t>(millis() - syncStart);
  _oldPlaneValid = true;
  _bwPlanesSynced = true;
  _needFullClear = false;

#if FREEINK_UC8179_PANEL_TEMP
  // The TSC holds BUSY ~106 ms: never after a partial (a page turn or menu
  // move waits on it), only after the already-slow Full/Half.
  // ponytail: an interactive-only session samples only at those; add an idle
  // render-task sample if cold detection goes stale in practice.
  if (!_pendingPartial) samplePanelTemperature(bus, PANEL_TEMP_REFRESH_PERIOD_MS);
#endif

  if (_pendingTurnOff) {
    bus.cmd(CMD_POWER_OFF);
    bus.waitBusy(" 8179_POF");
    _isScreenOn = false;
  }
}

void Uc8179Driver::requestResync(uint8_t settlePasses) {
  _absoluteInput = false;
  _directGrayPass = false;
  _directGrayPlanes = 0;
  (void)settlePasses;
  _needFullClear = true;  // next refresh does a full flash to clear ghosting
}

void Uc8179Driver::skipInitialResync() { _needFullClear = false; }

// After a software restart the panel still shows the last frame, but begin()
// reset the controller. Loading that frame as the OLD plane lets the first
// refresh run the fast differential waveform instead of a full GC flash.
bool Uc8179Driver::seedDisplayedFrame(EpdBus& bus, const uint8_t* frame) {
  if (_directGrayConfigured) return false;
  bus.waitBusy(" 8179_seed");
  streamPlane(bus, CMD_DTM1, frame);
  _oldPlaneValid = true;
  _needFullClear = false;
  _bwPlanesSynced = false;
  return true;
}

bool Uc8179Driver::powerOffIdle(EpdBus& bus) {
  if (!_isScreenOn) return false;
#if FREEINK_UC8179_PANEL_TEMP
  samplePanelTemperature(bus, PANEL_TEMP_PERIOD_MS);  // idle: nobody waits on its BUSY
#endif
  bus.cmd(CMD_POWER_OFF);  // POF; startBwRefresh() and the gray paths re-send PON
  bus.waitBusy(" 8179_IDLE_POF");
  _isScreenOn = false;
  return true;
}

bool Uc8179Driver::powerOnIdle(EpdBus& bus) {
  // The direct-gray register set resets the controller (and powers off) on the
  // next B/W refresh, so an early PON there would be wasted.
  if (_isScreenOn || _directGrayConfigured) return false;
  bus.cmd(CMD_POWER_ON);
  bus.waitBusy(" 8179_EARLY_PON");
  _isScreenOn = true;
  return true;
}

void Uc8179Driver::deepSleep(EpdBus& bus) {
  syncStaleOldPlane(bus);
  _directGrayOnPanel = false;
  _panelGrayValid = false;
  _bwBaseShown = false;
  _absoluteInput = false;
  _directGrayPass = false;
  _directGrayPlanes = 0;
  _grayBaseValid = false;
  _absoluteGrayPlanes = false;
  if (_isScreenOn) {
    bus.cmd(CMD_POWER_OFF);
    bus.waitBusy(" 8179 power-down");
    _isScreenOn = false;
  }
  bus.cmd(CMD_DEEP_SLEEP);
  bus.data(0xA5);
}

// --- 4-level grayscale (anti-aliasing) --------------------------------------
// Load the two bitplanes (oriented + padded like the B/W path) into controller
// RAM; displayGray() then runs the custom-LUT grayscale waveform. CrossPoint's
// masks are converted below to Factory.bin's absolute plane0/plane1 encoding;
// the resulting (DTM1,DTM2) pair selects the WW/BW/WB/BB LUT per pixel.
void Uc8179Driver::syncStaleOldPlane(EpdBus& bus) {
  if (!_oldPlaneStale) return;
  _oldPlaneStale = false;
  bus.waitBusy(" 8179_old_sync");
  if (_grayBaseValid && _grayBase != nullptr) {
    streamPlane(bus, CMD_DTM1, _grayBase);  // DTM2 already holds the same base
    _bwPlanesSynced = true;
  } else {
    _oldPlaneValid = false;
    _needFullClear = true;
  }
}

void Uc8179Driver::preconditionGrayscale(EpdBus& bus, uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
  // Intentionally no extra pass: the direct-gray waveform drives every pixel
  // absolutely, so there is nothing to condition.
  (void)bus;
  (void)x;
  (void)y;
  (void)w;
  (void)h;
}

void Uc8179Driver::beginGrayscale(EpdBus& bus, const uint8_t* fb, GrayscaleMode mode, RefreshMode fallback,
                                  bool turnOff) {
  syncStaleOldPlane(bus);
  if (mode == GrayscaleMode::Direct) {
    configureDirectGrayscale(bus);
    _absoluteInput = true;
    _directGrayPass = true;
    _bwBaseShown = false;  // no B/W base: the full-swing set sets every pixel
    // Canonical VCOM/black/light/dark/white rows, mapped to absolute selectors.
    writeLutSet(bus, lutbalance::checkedTable<kDirectGraySet, lutbalance::Policy::Absolute>());
    _directGrayPlanes = 0;
    _grayBaseValid = false;
    _absoluteGrayPlanes = false;
    _oldPlaneValid = false;
    _bwPlanesSynced = false;
    _needFullClear = true;
      return;
  }
  _absoluteInput = false;
  _directGrayPass = false;
  _directGrayPlanes = 0;
  displayGrayscaleBase(bus, fb, fallback, turnOff);
  _absoluteInput = mode == GrayscaleMode::Absolute;
}

void Uc8179Driver::copyGrayscaleLsb(EpdBus& bus, const uint8_t* lsb) {
  if (!lsb) return;
  _oldPlaneStale = false;  // every branch below overwrites DTM1
  if (_absoluteInput) {
    bus.waitBusy(" absolute plane");
    streamPlane(bus, CMD_DTM1, lsb, false);
    if (_directGrayPass) _directGrayPlanes |= 1;
    _bwPlanesSynced = false;
    return;
  }
  bus.waitBusy(" 8179_gray_lsb");  // prior base refresh must finish before RAM writes
  _absoluteGrayPlanes = false;
  if (_grayBaseValid) {
    // Factory firmware feeds the AA LUT absolute selectors, written here as
    // (plane0/DTM1, plane1/DTM2):
    //   black=(0,0), dark=(1,0), light=(0,1), white=(1,1).
    // CrossPoint instead supplies (maskLsb, maskMsb):
    //   black/white=(0,0), dark=(1,1), light=(0,1),
    // while its B/W base is 0 for every non-white pixel and 1 for white.
    // Folding the base into its LSB mask produces stock plane0:
    //   plane0 = base | maskLsb.
    for (uint32_t i = 0; i < _bufferSize; i++) {
      _grayBase[i] = static_cast<uint8_t>(_grayBase[i] | lsb[i]);
    }
    if (_grayMask != nullptr) memcpy(_grayMask, lsb, _bufferSize);  // copyGrayscaleMsb ORs in msb
    streamPlane(bus, CMD_DTM1, _grayBase);
    _absoluteGrayPlanes = true;
  } else {
    streamPlane(bus, CMD_DTM1, lsb);  // compatibility fallback without a base snapshot
  }
  _grayBaseValid = false;  // _grayBase now holds absolute plane0, not the B/W base
  _bwPlanesSynced = false;
}

void Uc8179Driver::copyGrayscaleMsb(EpdBus& bus, const uint8_t* msb) {
  syncStaleOldPlane(bus);
  if (!msb) return;
  if (_absoluteInput) {
    bus.waitBusy(" absolute plane");
    streamPlane(bus, CMD_DTM2, msb, false);
    if (_directGrayPass) _directGrayPlanes |= 2;
    _bwPlanesSynced = false;
    return;
  }
  bus.waitBusy(" 8179_gray_msb");
  if (_absoluteGrayPlanes) {
    // With plane0=(base|maskLsb), stock plane1 is plane0 XOR maskMsb:
    // black 0^0=0, dark 1^1=0, light 0^1=1, white 1^0=1.
    streamPlaneXor(bus, CMD_DTM2, _grayBase, msb);
    // Gray pixels: msb (== plane0 ^ plane1, the light/dark rows) plus any lsb
    // bit, so an lsb-only pixel (not a valid mask pair) is never trusted as B/W.
    if (_grayMask != nullptr) {
      for (uint32_t i = 0; i < _bufferSize; i++) _grayMask[i] = static_cast<uint8_t>(_grayMask[i] | msb[i]);
    }
    // The stock gray_aa routine restores BOTH controller planes to its B/W base
    // after the gray activation. Recover that base now while plane0 and the MSB
    // mask are still available: base = plane0 & plane1.
    for (uint32_t i = 0; i < _bufferSize; i++) {
      _grayBase[i] = static_cast<uint8_t>(_grayBase[i] & (_grayBase[i] ^ msb[i]));
    }
    _grayBaseValid = true;
  } else {
    streamPlane(bus, CMD_DTM2, msb);  // compatibility fallback
  }
}

void Uc8179Driver::displayGray(EpdBus& bus, const uint8_t* fb, bool turnOff, const unsigned char* lut,
                               bool factoryMode) {
  syncStaleOldPlane(bus);
  (void)fb;
  (void)lut;  // waveform comes from kDirectGraySet
  (void)factoryMode;

  // Every gray page runs the balanced direct-gray waveform (every row nets
  // zero) instead of the stock AA set, whose rows push one way. The overlay
  // path already built the same absolute selectors (DTM1 plane0, DTM2 plane1)
  // the direct rows are mapped to, so only the waveform changes.
  // Overlay planes derive from the B/W base (black/white pixels == base), so
  // with that base on the panel smooth gray may hold them. Direct gray always
  // swings black/white too: the base is an OTP Fast transition, and holding
  // its result kept the old page as ghosts in the image's white/black areas
  // and left first-page text gray (log 20260930T082014Z-2557c0fd L4067-4077).
  const bool overlayPlanes = !_directGrayPass && _absoluteGrayPlanes;
  const bool holdBw = overlayPlanes && _bwBaseShown;
  if (!_directGrayPass) {
    bus.waitBusy(" 8179_gray_ready");
    if (!_absoluteGrayPlanes && !_absoluteInput) {
      // Raw overlay masks (no base snapshot) are not absolute selectors: keep
      // the B/W base on screen and resync from scratch.
      LOG_DBG("EPD", "8179: gray pass skipped, no absolute planes");
      _absoluteInput = false;
      _needFullClear = true;
      _oldPlaneValid = false;
      _bwPlanesSynced = false;
      return;
    }
    // Power on first so the setup keeps power and never resets the controller,
    // which would lose the planes already in DTM1/DTM2.
    if (!_isScreenOn) {
      bus.cmd(CMD_POWER_ON);
      bus.waitBusy(" 8179_gray_PON");
      _isScreenOn = true;
    }
    configureDirectGrayscale(bus);
    if (holdBw && _smoothGray) {
      writeLutSet(bus, lutbalance::checkedTable<kDirectGrayHoldSet, lutbalance::Policy::Absolute>());
      LOG_DBG("EPD", "8179: smooth gray, B/W pixels hold");
    } else {
      writeLutSet(bus, lutbalance::checkedTable<kDirectGraySet, lutbalance::Policy::Absolute>());
    }
    _directGrayPass = true;
    _directGrayPlanes = 3;
  }
  if (_directGrayPlanes != 3) return;
  bus.waitBusy(" 8179_direct_ready");
  if (!_isScreenOn) {
    bus.cmd(CMD_POWER_ON);
    bus.waitBusy(" 8179_direct_PON");
    _isScreenOn = true;
  }
  logSpiBeforeDrf("direct_gray");
  bus.cmd(CMD_DISPLAY_REFRESH);
  bus.waitBusy(" 8179_DIRECT_GRAY_DRF");
  _directGrayOnPanel = true;
  // _grayBase (B/W base) + _grayMask (gray pixels) now describe the panel.
  _panelGrayValid = overlayPlanes && _grayBase != nullptr && _grayMask != nullptr;
  _bwBaseShown = false;
  _directGrayPass = false;
  _directGrayPlanes = 0;
  _absoluteInput = false;
  _needFullClear = true;
  _oldPlaneValid = false;
  _grayBaseValid = false;
  _absoluteGrayPlanes = false;
  if (turnOff) {
    bus.cmd(CMD_POWER_OFF);
    bus.waitBusy(" 8179_direct_POF");
    _isScreenOn = false;
  }
}

void Uc8179Driver::cleanupGrayscaleBuffers(EpdBus& bus, const uint8_t* bw) {
  syncStaleOldPlane(bus);
  bus.waitBusy(" 8179_gray_cleanup");
  _absoluteInput = false;
  _directGrayPass = false;
  _directGrayPlanes = 0;
  _grayBaseValid = false;
  _absoluteGrayPlanes = false;
  if (_directGrayConfigured) {
    _needFullClear = true;
    _oldPlaneValid = false;
    _bwPlanesSynced = false;
    return;
  }
  if (!bw) {
    // No baseline provided — fall back to a full flash on the next B/W refresh.
    _needFullClear = true;
    _oldPlaneValid = false;
    return;
  }
  if (_bwPlanesSynced && _oldPlaneValid) return;
  // Compatibility fallback when no PSRAM base snapshot was available. Stock
  // restores both planes, not only DTM1.
  streamPlane(bus, CMD_DTM1, bw);
  streamPlane(bus, CMD_DTM2, bw);
  _oldPlaneValid = true;
  _bwPlanesSynced = true;
  // RAM restoration does not cancel a requested physical clean.
}

// Per-board config injection, same idiom as the other drivers: define
// `const Uc8179Config& yourConfig();` in namespace freeink and build with
// -DFREEINK_UC8179_CONFIG=yourConfig.
#ifdef FREEINK_UC8179_CONFIG
const Uc8179Config& FREEINK_UC8179_CONFIG();
static const Uc8179Config& uc8179ActiveConfig() { return FREEINK_UC8179_CONFIG(); }
#else
static const Uc8179Config& uc8179ActiveConfig() { return uc8179DefaultConfig(); }
#endif

PanelDriver& uc8179Driver() {
  static Uc8179Driver instance(uc8179ActiveConfig());
  return instance;
}

}  // namespace freeink
