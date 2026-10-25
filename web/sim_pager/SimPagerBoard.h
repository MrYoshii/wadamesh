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
  int  externalPowerPresent() { return 0; }
  ChargeState chargeState() { return ChargeState::NotCharging; }
  bool chargerPresent() const { return true; }
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
