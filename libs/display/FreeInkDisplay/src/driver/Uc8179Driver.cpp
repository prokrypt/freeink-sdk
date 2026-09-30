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
constexpr uint8_t CMD_PARTIAL_WINDOW = 0x90;      // PTL
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

// B/W-dependent grayscale (AA) waveform LUTs — stock's REAL grayscale set (the
// short 2-frame LUTs FUN_4214ebd0 actually uploads @app1 DROM 0x3c5d8994..),
// uploaded in custom-LUT mode (PSR REG=1). Unlike the full-gray packet, here
// the register command is sent SEPARATELY — blob byte0 is DATA, not the cmd.
// Each LUT is 42 (0x2A) data bytes; only the first ~12 are non-zero. Level
// select by (old=0x10/LSB, new=0x13/MSB): (0,0)=LUTKK black, (0,1)=LUTKW,
// (1,0)=LUTWK, (1,1)=LUTWW white. This is the byte-exact stock set;
// CrossPoint's overlay-mask representation is converted to these absolute
// selectors before upload rather than modifying the waveform.
constexpr uint8_t GRAY_LUT_LEN = 42;  // 0x2A data bytes, command sent separately
struct GrayLut {
  uint8_t cmd;
  uint8_t data[GRAY_LUT_LEN];
};
constexpr GrayLut kGrayLuts[5] = {
    {0x20, {0x00, 0x02, 0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}},  // LUTC / VCOM
    {0x21, {0x08, 0x02, 0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}},  // LUTWW (white)
    {0x22, {0x20, 0x02, 0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}},  // LUTKW
    {0x23, {0x20, 0x02, 0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}},  // LUTWK
    {0x24, {0x00, 0x02, 0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}},  // LUTKK (black)
};

// Separate dark gray from light gray for both text and image grayscale.
// UC8179 datasheet R23h: each group is [rail selectors, four frame counts,
// repeat count]. Shorten the VDL phase from two frames to one, moving that
// frame to the following GND phase so the group remains six frames long.
// Other rails, groups, and the light-gray/VCOM/black/white tables stay stock.
constexpr uint8_t kDarkGrayLut[GRAY_LUT_LEN] = {0x20, 0x02, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};

// OEM XTF_PRE_BW_MID conditioning waveform. Each row is command-prefixed:
// byte 0 selects LUT register 0x20..0x24 and the remaining 42 bytes are data.
// It runs over equal B/W planes immediately before the short AA waveform so
// gray and white particle states do not relax after the page stops updating.
constexpr uint8_t kGrayPreBwMid[5][43] = {
    {0x20, 0x00, 0x06, 0x01, 0x06, 0x06, 0x01, 0x00, 0x02, 0x04, 0x00, 0x00, 0x01},
    {0x21, 0x20, 0x06, 0x01, 0x06, 0x06, 0x01, 0x00, 0x02, 0x04, 0x00, 0x00, 0x01},
    {0x22, 0xAA, 0x06, 0x01, 0x06, 0x06, 0x01, 0xA0, 0x02, 0x04, 0x00, 0x00, 0x01},
    {0x23, 0x55, 0x06, 0x01, 0x06, 0x06, 0x01, 0x50, 0x02, 0x04, 0x00, 0x00, 0x01},
    {0x24, 0x00, 0x06, 0x01, 0x06, 0x06, 0x01, 0x10, 0x02, 0x04, 0x00, 0x00, 0x01},
};
// Upload sets in register order R20h..R24h, gated by UltraChipLutBalance.h.
// Stock AA set with R23h replaced by kDarkGrayLut.
constexpr lutbalance::LutSet makeGrayAaSet() {
  lutbalance::LutSet s{};
  for (uint8_t r = 0; r < lutbalance::kRows; ++r) {
    for (uint8_t i = 0; i < GRAY_LUT_LEN; ++i) {
      s.row[r][i] = r == lutbalance::Wk ? kDarkGrayLut[i] : kGrayLuts[r].data[i];
    }
  }
  return s;
}
constexpr lutbalance::LutSet makeGrayPreBwMidSet() {
  lutbalance::LutSet s{};
  for (uint8_t r = 0; r < lutbalance::kRows; ++r) {
    for (uint8_t i = 0; i < GRAY_LUT_LEN; ++i) s.row[r][i] = kGrayPreBwMid[r][i + 1];
  }
  return s;
}
constexpr bool cmdOrderIsR20ToR24() {
  for (uint8_t r = 0; r < lutbalance::kRows; ++r) {
    if (kGrayLuts[r].cmd != 0x20 + r || kGrayPreBwMid[r][0] != 0x20 + r) return false;
  }
  return true;
}
static_assert(cmdOrderIsR20ToR24(), "gray LUT rows must be in register order R20h..R24h");
constexpr lutbalance::LutSet kGrayAaSet = makeGrayAaSet();
constexpr lutbalance::LutSet kGrayPreBwMidSet = makeGrayPreBwMidSet();
// Direct gray rows are VCOM, black, light, dark, white; registers take VCOM,
// white, light, dark, black.
constexpr uint8_t kDirectGrayOrder[lutbalance::kRows] = {0, 4, 2, 3, 1};
constexpr lutbalance::LutSet kDirectGraySet = lutbalance::fromRows(kUltraChipDirectGray, kDirectGrayOrder);

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

// The only LUT register writer in this driver: takes gated sets only.
void writeLutSet(EpdBus& bus, const lutbalance::CheckedLuts& luts) {
  for (uint8_t r = 0; r < lutbalance::kRows; ++r) {
    bus.cmd(static_cast<uint8_t>(0x20 + r));
    bus.data(luts.row(r), GRAY_LUT_LEN);
  }
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
  _grayRefreshedOnce = false;
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
  bus.cmd(0x01);
  bus.data(0x17);
  bus.data(static_cast<uint8_t>(config[0] & 7));
  bus.data(static_cast<uint8_t>(config[1] & 0x3F));
  bus.data(static_cast<uint8_t>(config[2] & 0x3F));
  bus.data(static_cast<uint8_t>(config[3] & 0x3F));
  bus.cmd(0x2A);
  bus.data(static_cast<uint8_t>(config[2] & 0xC0));
  bus.data(config[4]);
  bus.cmd(0x82);
  bus.data(config[5]);
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
  _redriveAfterGray = false;
}

void Uc8179Driver::begin(EpdBus& bus) {
  _oldPlaneStale = false;
  _directGrayOnPanel = false;
  _directGrayConfigured = false;
  _directGrayPass = false;
  _directGrayPlanes = 0;
  _needFullClear = true;
  _oldPlaneValid = false;
  _redriveAfterGray = false;
#if defined(BOARD_HAS_PSRAM)
  if (_grayBase == nullptr) {
    _grayBase = static_cast<uint8_t*>(heap_caps_malloc(_bufferSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  }
#endif
  bus.reset(50);
  initController(bus);
}

void Uc8179Driver::display(EpdBus& bus, const uint8_t* fb, const uint8_t* prev, RefreshMode mode, bool turnOff) {
  syncStaleOldPlane(bus);
  // CrossPoint's whole-plane text-AA path calls ordinary displayBuffer(FAST)
  // for its B/W base. After an AA page, route that base through stock's
  // non-flashing previous->current transition; promoting it to GC fixed the
  // charge but caused a full flash on every page.
  if (mode == RefreshMode::Fast && _redriveAfterGray && _grayRefreshedOnce && _oldPlaneValid && !_needFullClear) {
    transitionGrayscaleBase(bus, fb, turnOff);
    return;
  }
  displayStart(bus, fb, prev, mode, turnOff);
  displayFinish(bus, fb);
}

void Uc8179Driver::transitionGrayscaleBase(EpdBus& bus, const uint8_t* fb, bool turnOff) {
  if (!fb) return;
  const bool preconditionRunning = transitionGrayscaleBaseStart(bus, fb);
  transitionGrayscaleBaseFinish(bus, fb, turnOff, preconditionRunning, /*deferOldPlane=*/false);
}

bool Uc8179Driver::transitionGrayscaleBaseStart(EpdBus& bus, const uint8_t* fb) {
  syncStaleOldPlane(bus);
  _grayBaseValid = false;
  _absoluteGrayPlanes = false;
  if (_grayBase != nullptr) {
    memcpy(_grayBase, fb, _bufferSize);
    _grayBaseValid = true;
  }

  bus.waitBusy(" 8179_gray_base_ready");
  // DTM1 retains the preceding page's clean B/W base; DTM2 receives the new
  // base. XTF_PRE_BW_MID drives that real transition without the OTP GC flash.
  streamPlane(bus, CMD_DTM2, fb);
  _bwPlanesSynced = false;
  return startGrayscalePrecondition(bus);
}

void Uc8179Driver::transitionGrayscaleBaseFinish(EpdBus& bus, const uint8_t* fb, bool turnOff, bool preconditionRunning,
                                                 bool deferOldPlane) {
  if (preconditionRunning) finishGrayscalePrecondition(bus);

  // Keep the generic B/W baseline coherent in case no AA pass follows (Home or
  // a menu). An AA upload may immediately overwrite these planes; its cached
  // B/W snapshot above remains intact.
  // A deferred base defers this upload: the AA LSB upload that follows
  // overwrites DTM1 anyway, and _grayBase holds the frame for any other path.
  if (deferOldPlane && _grayBaseValid && _grayBase != nullptr) {
    _oldPlaneStale = true;
    _bwPlanesSynced = false;
  } else {
    streamPlane(bus, CMD_DTM1, fb);
    _bwPlanesSynced = true;
  }
  _oldPlaneValid = true;
  _redriveAfterGray = false;
  _needFullClear = false;

  if (turnOff && _isScreenOn) {
    bus.cmd(CMD_POWER_OFF);
    bus.waitBusy(" 8179_gray_base_POF");
    _isScreenOn = false;
  }
}

void Uc8179Driver::displayGrayscaleBase(EpdBus& bus, const uint8_t* fb, RefreshMode fallback, bool turnOff) {
  syncStaleOldPlane(bus);
  if (!fb) return;

  // Factory.bin's first gray_aa call paints its B/W base normally. Later calls
  // do NOT paint the new base and then condition two equal planes: they load the
  // retained previous base into DTM1, the new base into DTM2, and use
  // XTF_PRE_BW_MID as the page transition itself. Our former extra equal-plane
  // pass caused the visible gray muddling seen in hardware testing and did not
  // discharge the AA residue left by the preceding page.
  // Explicit Full/Half requests must remain real B/W clearing activations.
  // Only Fast may be replaced by the differential stock AA transition.
  // The XTF_PRE_BW_MID transition exists to discharge the residue of a
  // completed AA page. When the previous page never got its gray activation
  // (a quick turn cancelled it), the panel is plain B/W: use the ~110 ms
  // shorter DU base, as Factory.bin does for its first AA page.
  if (fallback != RefreshMode::Fast || !_redriveAfterGray || !_oldPlaneValid || _needFullClear) {
    display(bus, fb, nullptr, fallback, turnOff);
    return;
  }

  transitionGrayscaleBase(bus, fb, turnOff);
}

bool Uc8179Driver::displayGrayscaleBaseStart(EpdBus& bus, const uint8_t* fb, RefreshMode fallback, bool turnOff) {
  syncStaleOldPlane(bus);
  if (!fb) return false;
  // Same Overlay setup as beginGrayscale().
  _absoluteInput = false;
  _directGrayPass = false;
  _directGrayPlanes = 0;
  // Same routing as displayGrayscaleBase(), with the B/W fallback through the
  // async split. displayStart() always leaves its refresh pending.
  if (fallback != RefreshMode::Fast || !_redriveAfterGray || !_oldPlaneValid || _needFullClear) {
    return displayStart(bus, fb, nullptr, fallback, turnOff);
  }
  _pendingGrayPre = transitionGrayscaleBaseStart(bus, fb);
  _pendingGrayBase = true;
  _pendingTurnOff = turnOff;
  _pendingRefresh = true;
  return true;
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
    int8_t celsius = 0;
    if (coldPanel(celsius)) halfScrubFrames = static_cast<uint8_t>(halfScrubFrames + halfScrubFrames / 3);
    mode = RefreshMode::Fast;
    LOG_DBG("EPD", "8179: Half as DU scrub, %u frames", static_cast<unsigned>(halfScrubFrames));
  }
  syncStaleOldPlane(bus);
  const bool paintDestination = _directGrayOnPanel;
  _directGrayOnPanel = false;
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
  // Half is the explicit GC clean. Post-AA Fast paints are intercepted by
  // display() and use stock's non-flashing XTF_PRE_BW_MID transition instead.
  const bool scrub = (mode == RefreshMode::Half);
  const bool fast = ((mode == RefreshMode::Fast) && !scrub && !_needFullClear && _oldPlaneValid) || halfScrubFrames;

  if (paintDestination) {
    streamPlane(bus, CMD_DTM1, fb, true);
    streamPlane(bus, CMD_DTM2, fb);
    startBwRefresh(bus, true);
    bus.waitRefreshComplete(" 8179_BW_TARGET_DRF");
    bus.cmd(CMD_PARTIAL_OUT);
  }

  // A Half-as-scrub keeps the experiment's windows, PLL and resync skip out.
  gExpActive = gKbdExpOn && fast && !paintDestination && !halfScrubFrames;
  const bool duScrub =
      (duScrubRequested && gExpActive && (gKbdExp.flags & Uc8179KbdExperiment::KbdLut)) || halfScrubFrames;
  if (duScrub) LOG_DBG("EPD", "8179_EXP: DU scrub refresh");
  // NEW plane (0x13) = new frame.
  if (gExpActive && !duScrub && (gKbdExp.flags & Uc8179KbdExperiment::TwoWindow) && gKbdExp.windowCount > 0) {
    streamWindows(bus, fb);
  } else {
    streamPlane(bus, CMD_DTM2, fb);
  }
  if (duScrub) streamPlane(bus, CMD_DTM1, fb, /*invert=*/true);
  // Half with a valid OLD plane keeps it (true transitions). Full, and Half on
  // an unknown panel state, keep the absolute-from-white behavior.
  if (!fast && !(scrub && _oldPlaneValid)) bus.fillPlane(CMD_DTM1, 0xFF, _tresH, _wb);
  // (Ordinary Fast: OLD still holds the previous frame from displayFinish.)
  // A completed ordinary refresh supersedes any pending post-AA transition.
  _redriveAfterGray = false;

  _scrubLutFrames = halfScrubFrames;
  _complementOldPlane = duScrub;
  startBwRefresh(bus, fast);
  _scrubLutFrames = 0;
  _complementOldPlane = false;
  _pendingPartial = fast;
  _pendingTurnOff = turnOff;
  _pendingRefresh = true;
  return true;
}

void Uc8179Driver::logSpiBeforeDrf(const char* kind) {
  LOG_DBG("EPD", "SPI %s: %u planes %lu ms (%u windowed writes)", kind, static_cast<unsigned>(_spiPlanes),
          static_cast<unsigned long>(_spiUs / 1000), static_cast<unsigned>(_spiWindows));
  _spiUs = 0;
  _spiPlanes = 0;
  _spiWindows = 0;
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
    writeLutSet(bus, _complementOldPlane
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
  if (fast && gExpActive && (gKbdExp.flags & Uc8179KbdExperiment::TwoWindow) && gKbdExp.windowCount > 0) {
    // PTL persists across PTOUT, and plain PTIN refreshes whatever window it
    // holds: restore the whole panel after T3 windowed writes.
    writeFullPartialWindow(bus);
  }
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
  if (_pendingGrayBase) {
    _pendingGrayBase = false;
    transitionGrayscaleBaseFinish(bus, fb, _pendingTurnOff, _pendingGrayPre, /*deferOldPlane=*/true);
    _pendingGrayPre = false;
    return;
  }

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
  // EXPERIMENT T2: CDI 0x29 sets N2OCP, so the controller should already have
  // copied NEW to OLD after this refresh.
  const bool skipResync = gExpActive && _pendingPartial && (gKbdExp.flags & Uc8179KbdExperiment::SkipOldResync);
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

void Uc8179Driver::writePartialWindow(EpdBus& bus, uint16_t xStart, uint16_t xEnd, uint16_t yStart, uint16_t yEnd,
                                      bool scanAllGates) {
  const uint8_t window[9] = {static_cast<uint8_t>(xStart >> 8),
                             static_cast<uint8_t>(xStart & 0xF8),
                             static_cast<uint8_t>(xEnd >> 8),
                             static_cast<uint8_t>(xEnd | 0x07),
                             static_cast<uint8_t>(yStart >> 8),
                             static_cast<uint8_t>(yStart),
                             static_cast<uint8_t>(yEnd >> 8),
                             static_cast<uint8_t>(yEnd),
                             static_cast<uint8_t>(scanAllGates ? 0x01 : 0x00)};  // PT_SCAN
  bus.cmdData(CMD_PARTIAL_WINDOW, window, sizeof(window));
}

void Uc8179Driver::writeFullPartialWindow(EpdBus& bus) {
  writePartialWindow(bus, 0, static_cast<uint16_t>(_w - 1), 0, static_cast<uint16_t>(_h - 1));
}

// EXPERIMENT T3: write DTM2 only inside each window. RAM row r holds
// framebuffer row _h-1-r (see streamPlane); RAM x equals framebuffer x.
void Uc8179Driver::streamWindows(EpdBus& bus, const uint8_t* fb) {
  const uint32_t startUs = micros();
  bus.cmd(CMD_PARTIAL_IN);
  for (uint8_t i = 0; i < gKbdExp.windowCount && i < 2; i++) {
    const auto& win = gKbdExp.windows[i];
    const uint16_t x0 = static_cast<uint16_t>(win.x & ~7U);
    uint16_t x1 = static_cast<uint16_t>((win.x + win.w + 7U) & ~7U);
    if (x1 > _w) x1 = _w;
    uint16_t y1 = static_cast<uint16_t>(win.y + win.h);
    if (y1 > _h) y1 = _h;
    if (x1 <= x0 || y1 <= win.y) continue;
    const uint16_t ramTop = static_cast<uint16_t>(_h - y1);  // RAM row of framebuffer row y1-1
    const uint16_t ramBottom = static_cast<uint16_t>(_h - 1 - win.y);
    writePartialWindow(bus, x0, static_cast<uint16_t>(x1 - 1), ramTop, ramBottom);
    const uint16_t bytes = static_cast<uint16_t>((x1 - x0) / 8);
    const uint16_t rowsPerChunk = static_cast<uint16_t>(STREAM_CHUNK_BYTES / bytes);
    bus.cmd(CMD_DTM2);
    bus.beginTxn();
    uint16_t pending = 0;
    for (uint16_t r = ramTop; r <= ramBottom; r++) {
      const uint8_t* src = fb + static_cast<uint32_t>(_h - 1 - r) * _wb + x0 / 8;
      memcpy(streamChunk + static_cast<size_t>(pending) * bytes, src, bytes);
      if (++pending == rowsPerChunk) {
        bus.rawWriteBytes(streamChunk, static_cast<uint16_t>(pending * bytes));
        pending = 0;
      }
    }
    if (pending) bus.rawWriteBytes(streamChunk, static_cast<uint16_t>(pending * bytes));
    bus.endTxn();
    _spiWindows++;
  }
  _spiUs += micros() - startUs;
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

void Uc8179Driver::runGrayscalePrecondition(EpdBus& bus) {
  if (startGrayscalePrecondition(bus)) finishGrayscalePrecondition(bus);
}

bool Uc8179Driver::startGrayscalePrecondition(EpdBus& bus) {
  // Factory.bin skips XTF_PRE_BW_MID for its first AA page. Callers must have
  // retained the previous B/W base in DTM1 and loaded the new base into DTM2.
  if (!_oldPlaneValid || !_grayRefreshedOnce) return false;

  bus.waitBusy(" 8179_gray_pre_ready");
  bus.cmd(CMD_PARTIAL_IN);
  const uint16_t xEnd = static_cast<uint16_t>(_w - 1);
  const uint16_t yEnd = static_cast<uint16_t>(_h - 1);
  const uint8_t fullWindow[9] = {0x00, 0x00, static_cast<uint8_t>(xEnd >> 8), static_cast<uint8_t>(xEnd | 0x07),
                                 0x00, 0x00, static_cast<uint8_t>(yEnd >> 8), static_cast<uint8_t>(yEnd),
                                 0x01};
  bus.cmdData(CMD_PARTIAL_WINDOW, fullWindow, sizeof(fullWindow));
  bus.cmd(CMD_PANEL_SETTING);
  bus.data(_cfg.psr0);  // REG=1: run the external XTF_PRE_BW_MID tables
  bus.data(_cfg.psr1);
  bus.cmd(CMD_PFS);
  bus.data(_cfg.pfs);
  bus.cmd(CMD_GATE_SCAN);
  bus.data(_cfg.gateScan);
  bus.cmd(CMD_VCOM_DATA_INTERVAL);
  bus.data(_cfg.cdiActive);  // Factory.bin FUN_4214eab4 uses 0x29 for every pre-pass
  bus.data(CDI_INTERVAL);
  bus.cmd(CMD_CCSET);
  bus.data(_cfg.ccset);
  bus.cmd(CMD_TSSET);
  bus.data(_cfg.tssetFast);
  // Vendor exemption: byte-exact OEM XTF_PRE_BW_MID; nets WW -1, KK +4 frames.
  writeLutSet(bus, lutbalance::vendorTable<kGrayPreBwMidSet>());

  if (!_isScreenOn) {
    bus.cmd(CMD_POWER_ON);
    bus.waitBusy(" 8179_gray_pre_PON");
    _isScreenOn = true;
  }
  logSpiBeforeDrf("gray_pre");
  bus.cmd(CMD_DISPLAY_REFRESH);
  // Confirm the waveform started (BUSY dropped) before returning, as
  // startBwRefresh() does, so a deferred finish only rides out completion.
  {
    const int8_t busyPin = bus.pins().busy;
    const unsigned long t0 = millis();
    while (digitalRead(busyPin) == HIGH && millis() - t0 < 50) delay(1);
  }
  return true;
}

void Uc8179Driver::finishGrayscalePrecondition(EpdBus& bus) {
  bus.waitBusy(" 8179_gray_pre_DRF");
  bus.cmd(CMD_PARTIAL_OUT);
  bus.cmd(CMD_VCOM_DATA_INTERVAL);
  bus.data(_cfg.cdiIdle);
  bus.data(CDI_INTERVAL);
}

void Uc8179Driver::preconditionGrayscale(EpdBus& bus, uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
  // Intentionally no extra pass. The correct stock transition has to run while
  // DTM1 still contains the previous page and is therefore performed by
  // displayGrayscaleBase(). After a normal B/W activation both planes are the
  // current page; conditioning that equal pair only adds visible gray muddling.
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
    // Canonical VCOM/black/light/dark/white rows, mapped to absolute selectors.
    writeLutSet(bus, lutbalance::checkedTable<kDirectGraySet, lutbalance::Policy::Absolute>());
    _directGrayPlanes = 0;
    _grayBaseValid = false;
    _absoluteGrayPlanes = false;
    _oldPlaneValid = false;
    _bwPlanesSynced = false;
    _needFullClear = true;
    _redriveAfterGray = false;
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
  // fb = the reader's current frame; used to re-seed the B/W baseline below.
  (void)lut;  // waveform comes from the built-in gray LUT set (kGrayLuts)

  // The base refresh must be fully complete before we upload LUTs / stream — the
  // controller drops LUT/DTM/DRF writes while BUSY.
  if (_directGrayPass) {
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
    _directGrayPass = false;
    _directGrayPlanes = 0;
    _absoluteInput = false;
    _needFullClear = true;
    _oldPlaneValid = false;
    _redriveAfterGray = false;
    if (turnOff) {
      bus.cmd(CMD_POWER_OFF);
      bus.waitBusy(" 8179_direct_POF");
      _isScreenOn = false;
    }
    return;
  }
  bus.waitBusy(" 8179_gray_ready");
  _bwPlanesSynced = false;

  // Custom-LUT grayscale uses the stock gray_aa sequence (FUN_4214ec2c),
  // with a separate dark-gray table and FreeInk's SHL bit: PSR 0x3F (REG bit5=1
  // custom LUT; the B/W path masks to 0x1F/OTP) -> upload the 5 short LUTs
  // separately, 42 data bytes each) -> CDI 0x29/07 -> PON -> DRF. Unlike the
  // gray_full path, Factory.bin's gray_aa function sends no POF afterward. It
  // also sends no E0/E5/booster here; those belong to prebw/gray_full.
  bus.cmd(CMD_PANEL_SETTING);
  bus.data(_cfg.psr0);  // 0x3F: REG=1 (custom LUT) + KW + SHL
  bus.data(_cfg.psr1);
  // Vendor exemption: stock AA set (R23h shortened by kDarkGrayLut); every
  // gray row is a one-way VDL push: WW -1, KW -2, WK -1 frames.
  writeLutSet(bus, lutbalance::vendorTable<kGrayAaSet>());
  bus.cmd(CMD_VCOM_DATA_INTERVAL);
  // Factory.bin FUN_4214ec2c calls vtable +0x118 unconditionally; the UC8179
  // getter at 0x422988b0 returns 0x29. Unlike UC8279, it does not switch the AA
  // activation to the idle/hold CDI after the first page.
  bus.data(_cfg.cdiActive);
  bus.data(CDI_INTERVAL);
  _grayRefreshedOnce = true;

  // Absolute images start from a black/white base. Give the short gray AA LUTs
  // 25% longer to move their gray pixels toward white, then restore the stock
  // rate so ordinary black/white refresh timing is unchanged.
  const bool slowerImageWaveform = factoryMode && _absoluteInput;
  if (slowerImageWaveform) {
    bus.cmd(CMD_PLL_CONTROL);
    bus.data(PLL_40_HZ);
  }

  if (!_isScreenOn) {
    bus.cmd(CMD_POWER_ON);
    bus.waitBusy(" 8179_gray_PON");
    _isScreenOn = true;
  }
  logSpiBeforeDrf("gray");
  bus.cmd(CMD_DISPLAY_REFRESH);
  bus.waitBusy(" 8179_gray_split_DRF");
  if (slowerImageWaveform) {
    bus.cmd(CMD_PLL_CONTROL);
    bus.data(PLL_50_HZ);
  }
  // Deliberately remain powered. FUN_4214ec2c returns after DRF and RAM/base
  // bookkeeping without issuing command 0x02; deepSleep() still powers down.
  // Its bookkeeping writes the clean B/W base to BOTH DTM1 and DTM2. Besides
  // preserving the next transition's old frame, this prevents a stale gray
  // selector plane from being reused by a later refresh (especially sleep).
  if (_grayBaseValid) {
    streamPlane(bus, CMD_DTM1, _grayBase);
    streamPlane(bus, CMD_DTM2, _grayBase);
    _oldPlaneValid = true;
    _bwPlanesSynced = true;
    _needFullClear = false;
  }
  _grayBaseValid = false;
  _absoluteGrayPlanes = false;

  // `fb` is the MSB mask here, not the B/W frame; the recovered base above was
  // used for the RAM restore. Physically, AA still leaves intermediate charge
  // that a plain DU diff does not neutralize. Route the next Fast B/W base through
  // stock's non-flashing transition; an explicit Half remains the strong purge.
  (void)fb;
  _redriveAfterGray = true;
  _absoluteInput = false;
  _directGrayPass = false;
  _directGrayPlanes = 0;
  if (factoryMode && turnOff && _isScreenOn) {
    bus.cmd(CMD_POWER_OFF);
    bus.waitBusy(" 8179_absolute_POF");
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
