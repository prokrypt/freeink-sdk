#pragma once

// UC8179 panel driver — Xteink X4 / X4 Pro, newer production run (800x480 B/W).
// UltraChip UC8179 driven in KW mode (PSR KW/R=1): 1-bpp, DTM1 = OLD plane,
// DTM2 = NEW plane, differential refresh — the same KW paradigm and command set
// as the UC8279d X3 driver. It is a *separate* driver because the UC8179 needs
// an explicit PLL / booster / VCOM bring-up that the UC8279d (pure-OTP) omits:
// on OTP defaults alone the UC8179 never develops an image.
//
// Recovered from the X4 Pro OEM firmware (UC8179_800x480 init FUN_4214dff8 /
// full-update FUN_4214e584, via Ghidra). It runs the factory OTP waveforms
// (PSR REG=0) — the MTP holds temperature-compensated LUT sets — so no custom
// LUT upload is needed; only the power rails are programmed here. PENDING
// HARDWARE VALIDATION on a UC8179 (screenType=1 / hw_calib=2) X4 / X4 Pro unit.
//
// BUSY_N: low while busy (PON/DRF/POF all flag). Production waits one RTOS tick
// and then polls until BUSY_N is HIGH; it does not require observing a LOW edge.

#include <BoardConfig.h>  // FREEINK_UC8179_PANEL_TEMP

#include "PanelDriver.h"

namespace freeink {

struct Uc8179Config {
  // PSR (0x00) byte 0, as written at INIT. The OEM writes 0x3B here (REG bit5=1)
  // and re-asserts (psr0 & 0xDF)=0x1B (REG=0 -> OTP LUTs) just before every PON.
  uint8_t psr0;
  // PSR (0x00) byte 1.
  uint8_t psr1;
  // PFS power-off sequence (cmd 0x03). (PLL is 0x30 and stays panel-programmed.)
  uint8_t pfs;
  // Booster soft-start (cmd 0x06), 4 bytes.
  uint8_t btst[4];
  // Gate-scan selection (cmd 0xE1).
  uint8_t gateScan;
  // CCSET cascade/output enable (cmd 0xE0).
  uint8_t ccset;
  // TSSET forced temperature (cmd 0xE5) for a full refresh — selects the OTP
  // waveform's frame count/rate.
  uint8_t tsset;
  // TSSET (cmd 0xE5) for a fast/partial refresh (the OEM uses a different value).
  uint8_t tssetFast;
  // CDI (0x50) byte0 asserted during a refresh (before DRF); byte1 is 0x07.
  uint8_t cdiActive;
  // CDI (0x50) byte0 restored after the refresh completes; byte1 is 0x07.
  uint8_t cdiIdle;
  // TRES (0x61) gate count. The X4 Pro panel is addressed as 800x600 even though
  // only 480 rows are visible — the OTP waveform is tuned for the full 600-gate
  // scan, so the DTM transfer is padded to this height. (Visible height comes
  // from the board profile.)
  uint16_t tresHeight;
  // Power Save (cmd 0xE3). 0 disables the write; 0x22 is the UC8179 value used
  // by GxEPD2 to prevent artifacts with dithered images. Appended to preserve
  // the field order of existing aggregate board configurations.
  uint8_t powerSave;
};

const Uc8179Config& uc8179DefaultConfig();

class Uc8179Driver : public PanelDriver {
 public:
  explicit Uc8179Driver(const Uc8179Config& cfg = uc8179DefaultConfig());

  uint32_t spiHz() const override;
  BusyPolarity busyPolarity() const override { return BusyPolarity::UcIdleHigh; }
  PanelGeometry geometry() const override;

  void begin(EpdBus& bus) override;
  void deepSleep(EpdBus& bus) override;
  bool powerOffIdle(EpdBus& bus) override;
  bool powerOnIdle(EpdBus& bus) override;

  void display(EpdBus& bus, const uint8_t* fb, const uint8_t* prev, RefreshMode mode, bool turnOff) override;
  bool displayStart(EpdBus& bus, const uint8_t* fb, const uint8_t* prev, RefreshMode mode, bool turnOff) override;
  void displayFinish(EpdBus& bus, const uint8_t* fb) override;
  bool supportsAsyncDisplay() const override { return true; }

  void requestResync(uint8_t settlePasses) override;
  void skipInitialResync() override;
  bool seedDisplayedFrame(EpdBus& bus, const uint8_t* frame) override;
  bool grayOnPanel() const override { return _directGrayOnPanel; }

  // --- 4-level grayscale (anti-aliasing) ---
  // CrossPoint supplies two full 1bpp overlay masks. The driver combines them
  // with the displayed B/W base to recover Factory.bin's absolute 2-bit planes,
  // then sends plane0 -> DTM 0x10 and plane1 -> DTM 0x13. Full-buffer path only
  // (supportsStripGrayscale stays false; conversion needs the complete base).
  void displayGrayscaleBase(EpdBus& bus, const uint8_t* fb, RefreshMode fallback, bool turnOff) override;
  void preconditionGrayscale(EpdBus& bus, uint16_t x, uint16_t y, uint16_t w, uint16_t h) override;
  GrayscaleCapabilities grayscaleCapabilities(GrayscaleMode mode = GrayscaleMode::Overlay) const override {
    if (mode == GrayscaleMode::Absolute || mode == GrayscaleMode::Direct)
      return {GrayscaleEncoding::AbsolutePlanes,
              mode == GrayscaleMode::Direct ? GrayscaleBase::Combined : GrayscaleBase::Separate, false, false, false};
    if (mode != GrayscaleMode::Overlay) return {};
    return {GrayscaleEncoding::OverlayMasks, GrayscaleBase::Separate, false, false, false};
  }
  void beginGrayscale(EpdBus& bus, const uint8_t* fb, GrayscaleMode mode, RefreshMode fallback, bool turnOff) override;
  // Starts the XTF_PRE_BW_MID base transition (or the B/W fallback) and returns
  // while it runs; displayFinish() completes it. Lets the host render its AA
  // planes during the ~670 ms base waveform instead of after it.
  bool displayGrayscaleBaseStart(EpdBus& bus, const uint8_t* fb, RefreshMode fallback, bool turnOff) override;
  bool supportsDeferredGrayscaleBase() const override { return true; }
  void copyGrayscaleLsb(EpdBus& bus, const uint8_t* lsb) override;
  void copyGrayscaleMsb(EpdBus& bus, const uint8_t* msb) override;
  void displayGray(EpdBus& bus, const uint8_t* fb, bool turnOff, const unsigned char* lut, bool factoryMode) override;
  void cleanupGrayscaleBuffers(EpdBus& bus, const uint8_t* bw) override;

 private:
  void initController(EpdBus& bus);
  void startBwRefresh(EpdBus& bus, bool fast);
  // EXPERIMENT (test/kbd-uc8179): PTL helpers and windowed DTM2 upload.
  void writePartialWindow(EpdBus& bus, uint16_t xStart, uint16_t xEnd, uint16_t yStart, uint16_t yEnd,
                          bool scanAllGates = true);
  void writeFullPartialWindow(EpdBus& bus);
  void streamWindows(EpdBus& bus, const uint8_t* fb);
  void configureDirectGrayscale(EpdBus& bus);
  void restoreBwConfiguration(EpdBus& bus);
  // Stream a framebuffer into a RAM plane (ramCmd): reverse row order, use PSR
  // SHL for horizontal panel direction, then pad to the addressed gate count.
  // Used for both NEW plane (0x13) and OLD-plane sync (0x10).
  void streamPlane(EpdBus& bus, uint8_t ramCmd, const uint8_t* fb, bool invert = false);
  // Stream lhs XOR rhs with the same orientation and white gate padding. Used
  // to translate CrossPoint's MSB transition mask into stock absolute plane1.
  void streamPlaneXor(EpdBus& bus, uint8_t ramCmd, const uint8_t* lhs, const uint8_t* rhs);
  // Debug log: SPI time spent streaming planes since the last DRF, printed as
  // each refresh starts so a refresh splits into SPI upload vs waveform.
  void logSpiBeforeDrf(const char* kind);
#if FREEINK_UC8179_PANEL_TEMP
  // periodMs: minimum age of the last sample before another is taken.
  void samplePanelTemperature(EpdBus& bus, unsigned long periodMs);
#endif
  // Streams the displayed base (_grayBase) into DTM1 when a deferred base left
  // it stale. Every entry point that relies on DTM1 calls this first.
  void syncStaleOldPlane(EpdBus& bus);
  bool skipBaseOverDirectGray(const uint8_t* fb, RefreshMode fallback);

  const Uc8179Config& _cfg;

  uint16_t _w;      // visible width (800)
  uint16_t _h;      // visible height (480)
  uint16_t _wb;     // width in bytes (100)
  uint16_t _tresH;  // addressed gate count (600) — DTM padded to this
  uint32_t _bufferSize;

  // Stock Factory.bin uses absolute AA planes and derives its B/W base as
  // plane0 & plane1. CrossPoint supplies overlay masks after displaying the B/W
  // base separately, so preserve that base in X4 Pro PSRAM and fold it into the
  // masks. The resulting (plane0,plane1) selectors are black=(0,0), dark=(1,0),
  // light=(0,1), white=(1,1). The allocation temporarily holds absolute plane0,
  // then copyGrayscaleMsb() recovers the clean B/W base for stock's RAM restore.
  uint8_t* _grayBase = nullptr;
  bool _grayBaseValid = false;
  // Gray pixels (1 = light/dark) of the last overlay-path gray page, PSRAM.
  // With _grayBase it gives the panel's true state for the selective exit paint.
  uint8_t* _grayMask = nullptr;
  bool _panelGrayValid = false;  // _grayBase + _grayMask describe what the panel shows
  bool _bwBaseShown = false;     // the last B/W refresh showed the base the gray planes come from
  bool _absoluteGrayPlanes = false;
  bool _absoluteInput = false;
  bool _directGrayPass = false;
  bool _directGrayOnPanel = false;
  bool _directGrayConfigured = false;
  uint8_t _directGrayPlanes = 0;
  uint32_t _spiUs = 0;
  uint8_t _spiPlanes = 0;
  uint8_t _spiWindows = 0;  // T3 windowed DTM2 writes since the last SPI log

  bool _isScreenOn = false;
  // Register-LUT frames for this refresh's Half-as-DU-scrub (0 = none).
  uint8_t _scrubLutFrames = 0;
  bool _complementOldPlane = false;  // this refresh's DTM1 is the target's complement (DU scrub)
  // Force the first refresh after begin() to a full flash, so a partial update
  // never runs against an unknown on-screen state (e.g. a retained boot image).
  bool _needFullClear = true;
  // True once the OLD plane (0x10) holds a valid previous displayed frame, so a
  // differential partial or stock AA base transition has a real baseline.
  bool _oldPlaneValid = false;
  // True when both controller planes have been restored to the displayed B/W
  // base. False while an ordinary refresh or AA selector upload is in flight.
  bool _bwPlanesSynced = false;

  // Async split state (see Uc8279Driver for the contract).
  bool _pendingRefresh = false;
  bool _pendingTurnOff = false;
  bool _pendingPartial = false;  // this refresh used the PTIN/PTOUT partial path
  // A deferred base finished without re-sending DTM1: the AA LSB upload that
  // normally follows overwrites it immediately. DTM1 still holds the previous
  // page until syncStaleOldPlane() or that upload runs.
  bool _oldPlaneStale = false;
};

PanelDriver& uc8179Driver();

}  // namespace freeink
