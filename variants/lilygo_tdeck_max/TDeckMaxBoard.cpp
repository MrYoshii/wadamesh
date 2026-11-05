// SPDX-License-Identifier: GPL-3.0-or-later
#include "TDeckMaxBoard.h"
#include <Preferences.h>
#include "PagerHaptics.h"   // DRV2605 vibration motor (variants/lilygo_tlora_pager, shared)
#define XPOWERS_CHIP_SY6970
#include <XPowersLib.h>   // PowersSY6970: the charger

// One board, one charger; kept out of the header so nothing else that includes
// TDeckMaxBoard.h (the GPS provider, the UI) needs XPowersLib on its include path.
static PowersSY6970 s_charger;

// The pack LilyGo fits (1500 mAh). The BQ27220 leaves the factory set up for
// 3000 mAh, and its state of charge is only as good as that figure. Same scheme
// as the Pager: NVS "maxhw/cap_mah" holds the user's choice, "cap_try" makes
// sure a gauge that refuses the write is not put through config-update mode on
// every boot.
#define TDMAX_GAUGE_DESIGN_MAH 1500
static uint16_t maxCapTarget(Preferences& p) {
  const uint16_t v = p.getUShort("cap_mah", TDMAX_GAUGE_DESIGN_MAH);
  return (v >= 300 && v <= 20000) ? v : TDMAX_GAUGE_DESIGN_MAH;
}
static void maxConfigureGauge(GaugeBQ27220& gauge) {
  if (!gauge.refresh()) return;
  Preferences p;
  if (!p.begin("maxhw", false)) return;
  const uint16_t want = maxCapTarget(p);
  if (gauge.getDesignCapacity() != want && p.getUShort("cap_try", 0) != want) {
    const bool ok = gauge.setNewCapacity(want, want);
    Serial.printf("[BOOT] T-Deck Max bq27220 capacity -> %u mAh %s\n", (unsigned)want, ok ? "ok" : "FAILED");
    p.putUShort("cap_try", want);
  }
  p.end();
}

// Boot order matters on this board and mirrors Meck's proven sequence:
//   1. I2C up (XL9555, BQ27220, TCA8418, CST328, ES8311, SY6970 all share it)
//   2. XL9555 up and its boot state applied -- until this runs the LoRa module
//      is unpowered and the touch and keyboard controllers are unreset
//   3. Touch reset pulse, then keyboard reset pulse
//   4. Fuel gauge
// display.begin() runs after this and probes the touch controller, so the
// touch reset must already have happened here. The Pro's own resetTouch()
// pulse is a no-op on the Max (PIN_TOUCH_RST is -1 in this env).
void TDeckMaxBoard::begin() {
  ESP32Board::begin();

  Wire.begin(PIN_BOARD_SDA, PIN_BOARD_SCL, 400000);
  pinMode(PIN_USER_BTN, INPUT_PULLUP);

  // Park every shared-SPI select/reset OUTPUT-HIGH before any bus traffic,
  // exactly as the Pro board does.
  const uint8_t selects[] = { PIN_TFT_CS, P_LORA_NSS, P_LORA_RESET, PIN_SD_CS };
  for (uint8_t pin : selects) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
  }

  if (!expander_mutex_) expander_mutex_ = xSemaphoreCreateMutex();
  expander_ready_ = io_expander.begin(Wire, TDMAX_XL9555_ADDR);
  // Not a nicety: LoRa power and both input-controller resets hang off this
  // chip. If it is missing the radio probe fails and touch/keyboard are dead.
  Serial.printf("[BOOT] T-Deck Max xl9555 %s\n", expander_ready_ ? "ok" : "PROBE FAILED");

  if (expander_ready_) {
    // Output latches first, then direction. The XL9555 powers up with every
    // line as an input and its output latches at 1, so writing direction
    // first would put the modem enable, motor and amplifier lines HIGH for
    // the few milliseconds until the levels were corrected.
    io_expander.writePort(ExtensionIOXL9555::PORT0, TDMAX_XL_BOOT_PORT0);
    io_expander.writePort(ExtensionIOXL9555::PORT1, TDMAX_XL_BOOT_PORT1);
    io_expander.configPort(ExtensionIOXL9555::PORT0, 0x00);
    io_expander.configPort(ExtensionIOXL9555::PORT1, 0x00);
    delay(10);   // LoRa rail settle before anything downstream probes the radio

    touchReset();
    keyboardReset();
  }
  gps_on_ = false;

  const bool gauge_ok = gauge.begin(Wire);
  Serial.printf("[BOOT] T-Deck Max bq27220 %s\n", gauge_ok ? "ok" : "PROBE FAILED");
  if (gauge_ok) maxConfigureGauge(gauge);

  // SY6970 charger. XPowersLib's init checks the chip id and switches the I2C
  // watchdog off, so what is set here stays set. Its default fast-charge
  // current is far too much for the 1500 mAh pack: start at the Pager's safe
  // figures (704 mA, under C/2, up to 4288 mV); the UI applies the stored
  // Settings > Battery > Charging choice once it has loaded the prefs.
  charger_ready_ = s_charger.init(Wire, PIN_BOARD_SDA, PIN_BOARD_SCL, TDMAX_SY6970_ADDR);
  if (charger_ready_) {
    s_charger.setChargeTargetVoltage(4288);
    s_charger.setChargerConstantCurr(704);
    s_charger.enableCharge();   // CHG_CONFIG survives an ESP reset (see the Pager)
    const int stat = s_charger.readRegister(0x0B);
    if (stat >= 0) sys_stat_ = (uint8_t)stat;
  }
  Serial.printf("[BOOT] T-Deck Max sy6970 %s\n", charger_ready_ ? "ok" : "PROBE FAILED");

  // DRV2605 vibration driver: its supply came up with the boot state above.
  if (expander_ready_) pagerHapticsBegin();

  // ES8311 codec presence check (address only -- no register writes yet, that
  // is the sound code's job on first use, same as the pager). Every rail this
  // chip depends on is already up; a missing ack here means the speaker path
  // cannot work and is worth knowing from the boot log alone.
  Wire.beginTransmission(0x18);
  Serial.printf("[BOOT] T-Deck Max es8311 %s\n", Wire.endTransmission() == 0 ? "ok" : "PROBE FAILED");

  if (esp_reset_reason() == ESP_RST_DEEPSLEEP) {
    if (esp_sleep_get_ext1_wakeup_status() & (1ULL << P_LORA_DIO_1))
      startup_reason = BD_STARTUP_RX_PACKET;
    rtc_gpio_hold_dis((gpio_num_t)P_LORA_NSS);
    rtc_gpio_deinit((gpio_num_t)P_LORA_DIO_1);
    rtc_gpio_deinit((gpio_num_t)PIN_USER_BTN);
  }
}

bool TDeckMaxBoard::xlWrite(uint8_t line, bool high) {
  if (!expander_ready_ || !expander_mutex_) return false;
  if (xSemaphoreTake(expander_mutex_, pdMS_TO_TICKS(100)) != pdTRUE) return false;
  io_expander.digitalWrite(line, high ? HIGH : LOW);
  xSemaphoreGive(expander_mutex_);
  return true;
}

void TDeckMaxBoard::touchReset() {
  if (!expander_ready_) return;
  xlWrite(TDMAX_XL_TOUCH_RST, false);
  delay(20);
  xlWrite(TDMAX_XL_TOUCH_RST, true);
  delay(50);
}

void TDeckMaxBoard::keyboardReset() {
  if (!expander_ready_) return;
  xlWrite(TDMAX_XL_KEY_RST, false);
  delay(20);
  xlWrite(TDMAX_XL_KEY_RST, true);
  delay(50);
}

void TDeckMaxBoard::gpsPowerOn() {
  if (xlWrite(TDMAX_XL_GPS_EN, true)) {
    gps_on_ = true;
    delay(100);   // let the module's supply settle before the UART is nudged
  }
}

void TDeckMaxBoard::gpsPowerOff() {
  if (xlWrite(TDMAX_XL_GPS_EN, false)) gps_on_ = false;
}

void TDeckMaxBoard::setAmpEnabled(bool on) {
  if (!expander_ready_) { Serial.println("[MAXAUDIO] amp toggle skipped: expander not ready"); return; }
  // Route the shared speaker to the ES8311 before the amp comes up, so a
  // modem-audio selection can never blast through the NS4150B.
  if (on) xlWrite(TDMAX_XL_AUDIO_SEL, false);
  const bool ok = xlWrite(TDMAX_XL_AMP_EN, on);
  Serial.printf("[MAXAUDIO] amp %s %s\n", on ? "ON" : "OFF", ok ? "ok" : "WRITE FAILED");
}

void TDeckMaxBoard::selectEs8311Audio() {
  xlWrite(TDMAX_XL_AUDIO_SEL, false);
}

// ---- Battery ---------------------------------------------------------------------
// The logic is the Pager's (TLoraPagerBoard.cpp), on the SY6970 instead of the
// BQ25896: both have the same REG0B status layout, the same 64 mA / 16 mV
// charge steps and the same BATFET_DIS ship-mode bit.
void TDeckMaxBoard::refreshGauge() {
  const uint32_t now = millis();
  if (batt_ms_ && now - batt_ms_ < 2000) return;
  batt_ms_ = now ? now : 1;
  if (!gauge.refresh()) {
    gauge_info_.valid = false;
    return;
  }
  const uint16_t mv = gauge.getVoltage();
  if (mv >= 2500 && mv <= 4600) batt_mv_ = mv;
  const uint16_t soc = gauge.getStateOfCharge();
  if (soc <= 100) batt_soc_ = (int)soc;
  GaugeInfo g;
  g.valid         = true;
  g.current_ma    = gauge.getCurrent();
  g.power_mw      = gauge.getAveragePower();
  g.remaining_mah = gauge.getRemainingCapacity();
  g.full_mah      = gauge.getFullChargeCapacity();
  g.to_empty_min  = gauge.getTimeToEmpty();
  g.to_full_min   = gauge.getTimeToFull();
  g.temp_c10      = (int16_t)(gauge.getTemperature() * 10.0f);
  g.cycles        = gauge.getCycleCount();
  g.health_pct    = gauge.getStateOfHealth();
  gauge_info_ = g;
  chargeThermalCheck(g.temp_c10);
  chargeCareCheck(batt_soc_);
}

// Charging only inside the pack temperature window (default 0-45 deg C), with
// 3 deg C hysteresis. Only charging is gated; on USB the board keeps running.
#define TDMAX_CHG_TEMP_HYST_C10 30
void TDeckMaxBoard::chargeThermalCheck(int16_t t) {
  if (!charger_ready_) return;
  int8_t want = charge_temp_block_;
  if (chg_tmax_c10_ <= 0) {
    want = 0;
  } else {
    if (t < -400 || t > 1000) return;
    if (charge_temp_block_ == 0) {
      if (t > chg_tmax_c10_)      want = 1;
      else if (t < chg_tmin_c10_) want = -1;
    } else if (t <= chg_tmax_c10_ - TDMAX_CHG_TEMP_HYST_C10 &&
               t >= chg_tmin_c10_ + TDMAX_CHG_TEMP_HYST_C10) {
      want = 0;
    }
  }
  if (want == charge_temp_block_) return;
  charge_temp_block_ = want;
  Serial.printf("[POWER] temperature guard: %s (pack %d.%d C)\n",
                want > 0 ? "too hot" : want < 0 ? "too cold" : "ok",
                t / 10, (t < 0 ? -t : t) % 10);
  chargeEnableApply();
}

// Battery care: stop at 80 %, start again below 75 %.
#define TDMAX_CARE_STOP_PCT   80
#define TDMAX_CARE_RESUME_PCT 75
void TDeckMaxBoard::chargeCareCheck(int soc) {
  bool want = care_hold_;
  if (!care_on_)                 want = false;
  else if (soc < 0 || soc > 100) return;
  else if (!care_hold_ && soc >= TDMAX_CARE_STOP_PCT)  want = true;
  else if (care_hold_ && soc < TDMAX_CARE_RESUME_PCT)  want = false;
  if (want == care_hold_) return;
  care_hold_ = want;
  Serial.printf("[POWER] battery care: %s at %d %%\n", want ? "holding" : "charging", soc);
  chargeEnableApply();
}

void TDeckMaxBoard::chargeEnableApply() {
  if (!charger_ready_) return;
  const bool on = charge_temp_block_ == 0 && !care_hold_;
  if (on == chg_enabled_) return;
  chg_enabled_ = on;
  if (on) s_charger.enableCharge();
  else    s_charger.disableCharge();
  chg_ms_ = 0;
  Serial.printf("[POWER] charging %s\n", on ? "on" : "off");
}

void TDeckMaxBoard::setChargeCare(bool on) {
  care_on_ = on;
  chargeCareCheck(batt_soc_);
}

bool TDeckMaxBoard::setBatteryCapacity(uint16_t mah) {
  if (mah < 300 || mah > 20000) return false;
  Preferences p;
  if (!p.begin("maxhw", false)) return false;
  p.putUShort("cap_mah", mah);
  bool ok = false;
  if (gauge.refresh()) {
    if (gauge.getDesignCapacity() == mah) {
      ok = true;
    } else {
      gauge.setNewCapacity(mah, mah);
      p.putUShort("cap_try", mah);
      ok = gauge.refresh() && gauge.getDesignCapacity() == mah;
    }
  }
  p.end();
  batt_ms_ = 0;
  Serial.printf("[POWER] battery capacity %u mAh %s\n", (unsigned)mah, ok ? "ok" : "NOT TAKEN");
  return ok;
}

uint16_t TDeckMaxBoard::batteryCapacitySetting() {
  Preferences p;
  if (!p.begin("maxhw", true)) return TDMAX_GAUGE_DESIGN_MAH;
  const uint16_t v = maxCapTarget(p);
  p.end();
  return v;
}

void TDeckMaxBoard::setChargeLimits(uint16_t ma, uint16_t mv) {
  if (!charger_ready_) return;
  ma = (uint16_t)(ma / 64 * 64);
  mv = (uint16_t)(mv / 16 * 16);
  if (ma < 64) ma = 64;
  if (mv < 3840) mv = 3840;
  s_charger.setChargerConstantCurr(ma);
  s_charger.setChargeTargetVoltage(mv);
  Serial.printf("[POWER] charge limits %u mA, %u mV\n", (unsigned)ma, (unsigned)mv);
}

void TDeckMaxBoard::setChargeTempMin(int16_t c10) {
  chg_tmin_c10_ = c10 > 0 ? c10 : 0;
  chargeThermalCheck(gauge_info_.valid ? gauge_info_.temp_c10 : (int16_t)250);
}

void TDeckMaxBoard::setChargeTempMax(int16_t c10) {
  chg_tmax_c10_ = c10 > 0 ? c10 : 0;
  chargeThermalCheck(gauge_info_.valid ? gauge_info_.temp_c10 : (int16_t)250);
}

// REG0B: VBUS_STAT[7:5] (000 none, 111 OTG), CHRG_STAT[4:3], PG_STAT[2].
int TDeckMaxBoard::externalPowerPresent() {
  if (!charger_ready_) return -1;
  const uint32_t now = millis();
  if (!chg_ms_ || now - chg_ms_ >= 1000) {
    chg_ms_ = now ? now : 1;
    const int r = s_charger.readRegister(0x0B);
    if (r >= 0) {
      const uint8_t stat = (uint8_t)r;
      sys_stat_ = stat;
      const uint8_t vbus = (stat >> 5) & 0x07;
      const bool pg = (stat & 0x04) != 0;
      ext_pwr_ = (pg || (vbus != 0 && vbus != 7)) ? 1 : 0;
    }
  }
  return ext_pwr_;
}

TDeckMaxBoard::ChargeState TDeckMaxBoard::chargeState() {
  if (externalPowerPresent() < 0) return ChargeState::Unknown;
  return (ChargeState)((sys_stat_ >> 3) & 0x03);
}

// REG09: BATFET_DIS bit 5, BATFET_DLY bit 3 (0 = open at once). The touch
// controller is put in reset first, as LilyGo's own power-off does: with the
// rails dropping it can otherwise come back in an undefined state.
bool TDeckMaxBoard::enterShipMode() {
  if (!charger_ready_) return false;
  chg_ms_ = 0;
  if (externalPowerPresent() != 0) return false;
  Serial.println("[POWER] ship mode");
  Serial.flush();
  xlWrite(TDMAX_XL_TOUCH_RST, false);
  delay(20);
  const int r09 = s_charger.readRegister(0x09);
  if (r09 >= 0) s_charger.writeRegister(0x09, (uint8_t)(r09 & ~0x08));
  s_charger.shutdown();
  delay(2000);
  xlWrite(TDMAX_XL_TOUCH_RST, true);   // still running: undo, the caller falls back
  return false;
}
