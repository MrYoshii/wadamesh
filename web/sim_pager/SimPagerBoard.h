// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// T-LoRa Pager simulator: the Pager board the UI's TLORA_PAGER branches talk to, on
// top of the web board. A pack at 76 %, no USB, no card, the radio of the web board.
#include "../platform/WebBoard.h"
#include "../../src/helpers/HardwareRtcClock.h"

#define PAGER_PIN_SD_CS 21
#ifndef PIN_I2S_MCLK
  #define PIN_I2S_MCLK 10
  #define PIN_I2S_BCK  11
  #define PIN_I2S_WS   18
  #define PIN_I2S_DOUT 45
  #define PIN_I2S_SDIN 17
#endif

class SPIClass;

class TLoraPagerBoard : public WebBoard {
public:
  enum class SdCardState : uint8_t { Absent, Present, Unknown };
  enum class ChargeState : uint8_t { NotCharging = 0, PreCharge = 1, FastCharge = 2, Done = 3, Unknown = 0xFF };
  uint16_t getBattMilliVolts() override { return 3950; }
  const char* getManufacturerName() const override { return "LilyGo T-LoRa Pager (sim)"; }
  int  getBattStateOfCharge() { return 76; }
  struct GaugeInfo {
    bool valid; int16_t current_ma, power_mw; uint16_t remaining_mah, full_mah, to_empty_min, to_full_min;
    int16_t temp_c10; uint16_t cycles, health_pct;
  };
  GaugeInfo getGaugeInfo() {   // a pack drawing ~42 mA with the screen on
    return { true, (int16_t)(-38 - (int)(millis() / 1000 % 9)), -152, 1131, 1488, 1615, 0xFFFF, 241, 12, 99 };
  }
  int  externalPowerPresent() { return 0; }
  ChargeState chargeState() { return ChargeState::NotCharging; }
  bool enterShipMode() { return false; }
  SdCardState sdCardState() { return SdCardState::Absent; }
  bool sdCardPresent() { return false; }
  bool setSdCardPower(bool) { return true; }
  bool resetSdCardPower() { return true; }
  void setAmpEnabled(bool) {}
  bool setGpsPower(bool) { return true; }
  bool gpsPowerIsOn() const { return true; }
};

extern HardwareRtcClock hw_rtc;
SPIClass* tloraPagerSharedSPI();
void tloraPagerGpsPowerDown();
