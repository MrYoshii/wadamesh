// SPDX-License-Identifier: GPL-3.0-or-later
//
// Board class for the LilyGo T-Deck Max (ESP32-S3). Same SoC, GDEQ031T10
// e-paper panel, SX1262, TCA8418 keyboard and CST328/CST3530 touch as the
// T-Deck Pro, so the Pro's display and touch drivers are reused unchanged.
// What differs is power and reset routing: on the Max the peripheral power
// enables, resets and switches go through an XL9555 I/O expander on the shared
// I2C bus instead of direct GPIOs, and battery reporting comes from the
// BQ27220 fuel gauge (the Pro's BQ25896 charger at 0x6B is not fitted; the Max
// carries an SY6970 at 0x6A, a register-compatible part that owns USB input,
// charging and the battery switch). Line map, boot state and reset timings follow
// the Meck firmware's working bring-up of this exact board.
#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <driver/rtc_io.h>
#include <helpers/ESP32Board.h>
#include <ExtensionIOXL9555.hpp>
#include <GaugeBQ27220.hpp>

#define TDMAX_XL9555_ADDR 0x20
#define TDMAX_SY6970_ADDR 0x6A   // charger / power path

// XL9555 lines. 0-7 are port 0, 8-15 are port 1.
#define TDMAX_XL_6609_EN     0   // HIGH: A7682E modem supply (SGM6609 boost)
#define TDMAX_XL_LORA_EN     1   // HIGH: SX1262 supply (the Pro drives GPIO 46 for this)
#define TDMAX_XL_GPS_EN      2   // HIGH: GPS supply (the Pro drives GPIO 39 for this)
#define TDMAX_XL_1V8_EN      3   // HIGH: BHI260AP 1.8 V supply
#define TDMAX_XL_LORA_SEL    4   // HIGH: internal antenna, LOW: external (SKY13453)
#define TDMAX_XL_MOTOR_EN    5   // HIGH: DRV2605 supply
#define TDMAX_XL_AMP_EN      6   // HIGH: NS4150B speaker amplifier
#define TDMAX_XL_TOUCH_RST   7   // LOW: touch controller held in reset (the Pro uses GPIO 38)
#define TDMAX_XL_PWRKEY      8   // A7682E POWERKEY
#define TDMAX_XL_KEY_RST     9   // LOW: keyboard controller held in reset
#define TDMAX_XL_AUDIO_SEL  10   // HIGH: modem audio out, LOW: ES8311 audio out

// Boot state, written to both output ports in one go before the lines are
// switched to outputs: LoRa on, 1.8 V on, internal antenna, touch and keyboard
// resets released, the vibration driver's supply on (the DRV2605 sits in standby
// between patterns, a few uA); modem and amplifier off; audio mux to the ES8311.
//
// GPS is deliberately OFF here. On the T-Deck Pro the GPS rail is GPIO 39 and
// the location provider drives it: off at boot, on when GPS is enabled, off
// again when it is disabled. The Max keeps that behaviour by routing the same
// calls to this expander line (see TDeckMaxGps.h).
#define TDMAX_XL_BOOT_PORT0 0xBA   // 1011 1010: T_RST=1 AMP=0 MOT=1 LSEL=1 1V8=1 GPS=0 LORA=1 6609=0
#define TDMAX_XL_BOOT_PORT1 0x02   // 0000 0010: ASEL=0 KRST=1 PKEY=0

// Gauge probe/refresh failed -- never let the UI divide by zero.
#define TDMAX_BATT_MILLIVOLTS_FALLBACK 3700

class TDeckMaxBoard : public ESP32Board {
public:
  // SY6970 REG0B CHRG_STAT, plus Unknown when the charger does not answer.
  // Same values and meaning as TLoraPagerBoard::ChargeState (BQ25896).
  enum class ChargeState : uint8_t { NotCharging = 0, PreCharge = 1, FastCharge = 2, Done = 3, Unknown = 0xFF };

  void begin();

  // ---- Battery (BQ27220 gauge) -----------------------------------------------
  // The same API as TLoraPagerBoard, so the UI's battery pages, charge settings
  // and low-battery guard serve both boards. One gauge read is cached for 2 s:
  // the status bar asks several times a second.
  uint16_t getBattMilliVolts() { refreshGauge(); return batt_mv_; }
  // The gauge's StateOfCharge(), %; -1 until it has answered with an in-range value.
  int getBattStateOfCharge() { refreshGauge(); return batt_soc_; }
  struct GaugeInfo {
    bool     valid;
    int16_t  current_ma;      // + charging, - drawing from the pack
    int16_t  power_mw;        // average power, same sign
    uint16_t remaining_mah;
    uint16_t full_mah;
    uint16_t to_empty_min;    // 0xFFFF = not discharging
    uint16_t to_full_min;     // 0xFFFF = not charging
    int16_t  temp_c10;        // pack temperature, 0.1 deg C
    uint16_t cycles;
    uint16_t health_pct;
  };
  GaugeInfo getGaugeInfo() { refreshGauge(); return gauge_info_; }
  // The fitted pack's capacity (LilyGo fits 1500 mAh). Stored in NVS and written
  // to the gauge as design and full-charge capacity; true when it reads it back.
  bool     setBatteryCapacity(uint16_t mah);
  uint16_t batteryCapacitySetting();
  uint16_t batteryDesignCapacity() const { return gauge.getDesignCapacity(); }

  // ---- SY6970 charger ----------------------------------------------------------
  // External power present: 1 yes, 0 no, -1 charger not answering (cached 1 s).
  int  externalPowerPresent();
  ChargeState chargeState();
  // Charging held off for the pack temperature: 1 too hot, -1 too cold, 0 allowed.
  int  chargeTempBlock() const { return charge_temp_block_; }
  // Battery care: stop charging at 80 %, resume below 75 %.
  void setChargeCare(bool on);
  bool chargeCareHold() const { return care_hold_; }
  // Fast-charge current (64 mA steps) and charge voltage (16 mV steps), and the
  // temperature guard's limits in 0.1 deg C (upper 0 = guard off).
  void setChargeLimits(uint16_t ma, uint16_t mv);
  void setChargeTempMax(int16_t c10);
  void setChargeTempMin(int16_t c10);
  // Real power-off: opens the charger's battery switch (BATFET_DIS). Only the
  // power key or plugging in USB switches the board back on. Refused (false)
  // while USB is connected; on success it does not return.
  bool enterShipMode();

  void enterDeepSleep(uint32_t secs, int pin_wake_btn) {
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

    const gpio_num_t dio = (gpio_num_t)P_LORA_DIO_1;
    rtc_gpio_init(dio);
    rtc_gpio_set_direction(dio, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pulldown_en(dio);
    esp_sleep_enable_ext1_wakeup(1ULL << P_LORA_DIO_1, ESP_EXT1_WAKEUP_ANY_HIGH);

    rtc_gpio_hold_en((gpio_num_t)P_LORA_NSS);
    if (pin_wake_btn >= 0) {
      const gpio_num_t wake = (gpio_num_t)pin_wake_btn;
      rtc_gpio_init(wake);
      rtc_gpio_set_direction(wake, RTC_GPIO_MODE_INPUT_ONLY);
      rtc_gpio_pullup_en(wake);
      rtc_gpio_pulldown_dis(wake);
      esp_sleep_enable_ext0_wakeup(wake, 0);
    }
    if (secs > 0) esp_sleep_enable_timer_wakeup((uint64_t)secs * 1000000ULL);
    esp_deep_sleep_start();
  }

  const char* getManufacturerName() const { return "LilyGo T-Deck Max"; }

  // True once the XL9555 answered on the bus and the boot state was applied.
  bool expanderReady() const { return expander_ready_; }

  // Drive one expander line. Returns false if the expander is unavailable or
  // the guard could not be taken; the line is then left as it was.
  bool xlWrite(uint8_t line, bool high);

  // Reset pulses, same shape as Meck's on this board: LOW 20 ms, HIGH, then
  // 50 ms for the controller to come out of reset.
  void touchReset();
  void keyboardReset();

  // GPS rail, called by the Max location provider around the core provider's
  // begin()/stop() so GPS on/off in the UI powers the module on and off.
  void gpsPowerOn();
  void gpsPowerOff();
  bool gpsPowerIsOn() const { return gps_on_; }

  // Speaker path, mirroring LilyGo's ES8311 examples (examples/ES8311/playWAV):
  // AUDIO_SEL LOW = ES8311 codec, HIGH = A7682E modem; AMP_EN HIGH enables the
  // NS4150B speaker amplifier. The amp is parked OFF at idle to save battery;
  // the sound code brackets every chime/playback with setAmpEnabled(true/false)
  // exactly like TLoraPagerBoard does, and re-asserts the ES8311 mux before
  // enabling the amp in case the modem path ever drives it HIGH.
  void setAmpEnabled(bool on);
  void selectEs8311Audio();

  ExtensionIOXL9555 io_expander;
  GaugeBQ27220 gauge;

private:
  void refreshGauge();
  void chargeThermalCheck(int16_t temp_c10);
  void chargeCareCheck(int soc);
  void chargeEnableApply();

  bool expander_ready_ = false;
  bool charger_ready_ = false;
  uint16_t batt_mv_ = TDMAX_BATT_MILLIVOLTS_FALLBACK;
  int      batt_soc_ = -1;
  GaugeInfo gauge_info_ = {};
  uint32_t batt_ms_ = 0;             // when the gauge was last read, 0 = never
  int8_t   ext_pwr_ = -1;            // externalPowerPresent() cache
  uint8_t  sys_stat_ = 0;            // REG0B as last read
  uint32_t chg_ms_ = 0;              // when the charger was last read, 0 = never
  int8_t   charge_temp_block_ = 0;
  int16_t  chg_tmax_c10_ = 450;
  int16_t  chg_tmin_c10_ = 0;
  bool     care_on_ = false;
  bool     care_hold_ = false;
  bool     chg_enabled_ = true;
  bool gps_on_ = false;
  SemaphoreHandle_t expander_mutex_ = nullptr;
};
