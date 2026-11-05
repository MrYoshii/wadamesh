// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <helpers/ui/DisplayDriver.h>

#define ENABLE_GxEPD2_GFX 0
#include <GxEPD2_BW.h>
#include <CSE_CST328.h>
#include <LovyanGFX.hpp>
#include <SPI.h>
#include <Wire.h>

class TDeckProDisplay : public DisplayDriver {
public:
  using BusyHook = void (*)();

  TDeckProDisplay();
  bool begin();

  bool isOn() override { return _is_on; }
  bool isEink() override { return true; }
  void turnOn() override;
  void turnOff() override;
  void clear() override;
  void startFrame(ColorVal background = UIColor::window_bkg) override;
  void setTextSize(int size) override;
  void setColor(ColorVal color) override;
  void setCursor(int x, int y) override;
  void print(const char* text) override;
  void fillRect(int x, int y, int w, int h) override;
  void drawRect(int x, int y, int w, int h) override;
  void drawXbm(int x, int y, const uint8_t* bits, int w, int h) override;
  uint16_t getTextWidth(const char* text) override;
  void endFrame() override;

  void writePixelsRGB565(int x, int y, int w, int h, const uint16_t* pixels);
  static void prepareMapTileRGB565(uint16_t* pixels, int width, int height);
  void setDisplayRotation(uint8_t rotation);
  void setBrightness(uint8_t brightness);
#if defined(HAS_TDECK_MAX)
  // The three capacitive pads below the glass are CST3530 keys. Returns true
  // once per press (edge), with the key id (Meck: 0 = heart, 1 = bubble, 2 = send).
  bool takeFrontKeyPress(uint8_t& key_id);
#endif
  bool getTouchPoint(uint16_t& x, uint16_t& y);
  void serviceRefresh(bool force = false);
  void setBusyHook(BusyHook hook) { _busy_hook = hook; }

private:
#if defined(HAS_TDECK_MAX)
  // The GDEQ031T10 driver with its refresh split in two, so the UI keeps running
  // during the panel's ~0.65 s (partial) / ~1 s (full) update instead of standing
  // still in GxEPD2's busy wait. The commands are the driver's own _Update_Part /
  // _Update_Full sequence (GxEPD2 1.6.9); only the wait is left to the caller.
  class MaxPanel : public GxEPD2_310_GDEQ031T10 {
  public:
    using GxEPD2_310_GDEQ031T10::GxEPD2_310_GDEQ031T10;
    bool needsInitialFull() const { return _initial_refresh || _initial_write; }
    void startRefresh(bool full) {
      if (full) {
        _writeCommand(0xE0); _writeData(0x02);   // CCSET: TSFIX
        _writeCommand(0xE5); _writeData(0x5A);   // forced temperature: fast full update
        _writeCommand(0x50); _writeData(0x97);
      } else {
        _writeCommand(0x91);                     // partial in, the whole panel as the window
        _writeCommand(0x90);
        _writeData(0x00); _writeData((uint8_t)((WIDTH - 1) | 0x07));
        _writeData(0x00); _writeData(0x00);
        _writeData((uint8_t)((HEIGHT - 1) / 256)); _writeData((uint8_t)((HEIGHT - 1) % 256));
        _writeData(0x01);
        _writeCommand(0xE0); _writeData(0x02);
        _writeCommand(0xE5); _writeData(0x79);   // forced temperature: fast partial update
        _writeCommand(0x50); _writeData(0xD7);
      }
      if (!_power_is_on) { _writeCommand(0x04); _waitWhileBusy("_PowerOn", power_on_time); }
      _power_is_on = true;
      _writeCommand(0x12);                       // display refresh: returns at once
      delay(1);                                  // BUSY goes active within a moment
    }
    bool refreshBusy() const { return digitalRead(_busy) == _busy_level; }
    void finishRefresh(bool full) {
      if (!full) _writeCommand(0x92);            // partial out
      else       _initial_refresh = false;
      _init_display_done = false;                // as the driver does after every update
    }
  };
  using Panel = MaxPanel;
#else
  using Panel = GxEPD2_310_GDEQ031T10;
#endif
  using Eink = GxEPD2_BW<Panel, 40>;

  static void busyCallback(const void*);
  static bool isDark(uint16_t color);
  void resetTouch();
  bool probeCst3530();
  bool writeCst3530Command(uint32_t command);
  bool initCst3530();
  bool readCst3530(int16_t& x, int16_t& y);
  void canvasToMono();
  void requestRefresh(bool full = false);
  void writeBrightness(uint8_t brightness);

  static BusyHook _busy_hook;
  static constexpr int WIDTH = 240;
  static constexpr int HEIGHT = 320;
  static constexpr size_t MONO_BYTES = (size_t)WIDTH * HEIGHT / 8;

  lgfx::LGFX_Sprite _canvas;
  Eink _epd;
  CSE_CST328 _cst328;
  uint8_t* _mono = nullptr;
  uint8_t* _sent = nullptr;
  uint16_t _color = 0x0000;
  uint32_t _last_refresh_ms = 0;
  uint8_t _partial_refreshes = 0;
  uint8_t _brightness = 255;
  bool _refresh_pending = false;
  bool _full_refresh_pending = true;
  bool _sleeping = false;
  bool _touch_ready = false;
  bool _touch_is_cst3530 = false;
  bool _sent_valid = false;
  bool _is_on = false;
#if defined(HAS_TDECK_MAX)
  bool _epd_busy = false;            // an update is running on the panel (finishUpdate() ends it)
  bool _epd_busy_full = false;
  uint32_t _epd_start_ms = 0;
  uint32_t _last_request_ms = 0;     // the newest LVGL change, for the settle delay
  uint32_t _first_request_ms = 0;    // the oldest change not yet on the panel
  void finishUpdate(bool wait);
#endif
};