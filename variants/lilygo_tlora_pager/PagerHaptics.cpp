// SPDX-License-Identifier: GPL-3.0-or-later
#include "PagerHaptics.h"

#include <Arduino.h>
#include <Wire.h>

// DRV2605 register map (TI DRV2605 datasheet, SLOS825).
#define DRV_ADDR          0x5A
#define DRV_REG_STATUS    0x00   // DEVICE_ID in [7:5]
#define DRV_REG_MODE      0x01   // bit6 STANDBY, [2:0] mode (0 = internal trigger)
#define DRV_REG_RTPIN     0x02
#define DRV_REG_LIBRARY   0x03
#define DRV_REG_WAVESEQ1  0x04   // ..0x0B: eight sequencer slots, 0 ends the sequence
#define DRV_REG_GO        0x0C   // bit0 GO, cleared by the chip when the sequence ends
#define DRV_REG_OVERDRIVE 0x0D
#define DRV_REG_SUSTAINP  0x0E
#define DRV_REG_SUSTAINN  0x0F
#define DRV_REG_BREAK     0x10
#define DRV_REG_AUDIOMAX  0x13
#define DRV_REG_FEEDBACK  0x1A   // bit7 N_ERM_LRA (0 = ERM)
#define DRV_REG_CONTROL3  0x1D   // bit5 ERM_OPEN_LOOP

#define DRV_MODE_STANDBY  0x40
#define DRV_WAIT(ms10)    (uint8_t)(0x80 | (ms10))   // sequencer slot: pause of ms10 * 10 ms

static bool     s_present = false;
static bool     s_awake = false;       // out of standby, a pattern may be running
static uint32_t s_check_ms = 0;        // when to look at GO next

static bool drvWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(DRV_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static int drvRead(uint8_t reg) {
  Wire.beginTransmission(DRV_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom((uint8_t)DRV_ADDR, (uint8_t)1) != 1) return -1;
  return Wire.read();
}

void pagerHapticsBegin() {
  const int st = drvRead(DRV_REG_STATUS);
  const int id = st < 0 ? -1 : (st >> 5) & 0x07;
  // 3 = DRV2605, 4 = DRV2604, 6 = DRV2604L, 7 = DRV2605L
  s_present = (id == 3 || id == 4 || id == 6 || id == 7);
  if (s_present) {
    s_present = drvWrite(DRV_REG_MODE, 0x00)                       // out of standby
             && drvWrite(DRV_REG_RTPIN, 0x00)
             && drvWrite(DRV_REG_LIBRARY, 0x01)                     // ERM library A
             && drvWrite(DRV_REG_WAVESEQ1, 0x00)
             && drvWrite(DRV_REG_OVERDRIVE, 0x00)
             && drvWrite(DRV_REG_SUSTAINP, 0x00)
             && drvWrite(DRV_REG_SUSTAINN, 0x00)
             && drvWrite(DRV_REG_BREAK, 0x00)
             && drvWrite(DRV_REG_AUDIOMAX, 0x64);
    if (s_present) {
      const int fb = drvRead(DRV_REG_FEEDBACK);
      const int c3 = drvRead(DRV_REG_CONTROL3);
      s_present = fb >= 0 && c3 >= 0
               && drvWrite(DRV_REG_FEEDBACK, (uint8_t)(fb & 0x7F))   // ERM, not LRA
               && drvWrite(DRV_REG_CONTROL3, (uint8_t)(c3 | 0x20))   // open loop
               && drvWrite(DRV_REG_MODE, DRV_MODE_STANDBY);
    }
  }
  s_awake = false;
  Serial.printf("[BOOT] drv2605 %s\n", s_present ? "ok" : "PROBE FAILED");
}

bool pagerHapticsPresent() { return s_present; }

void pagerHapticsPlay(PagerHaptic kind) {
  if (!s_present) return;
  // Library A effect ids: 1 strong click, 14 strong buzz. The key/knob click is
  // effect 1 too: the sharp ticks (24-26) are too short for this ERM motor to
  // spin up, and were not felt at all on hardware.
  static const uint8_t k_msg[]  = { 14, 0 };
  static const uint8_t k_dm[]   = { 14, DRV_WAIT(12), 14, 0 };
  static const uint8_t k_men[]  = { 1, DRV_WAIT(8), 1, DRV_WAIT(8), 14, 0 };
  static const uint8_t k_tick[] = { 1, 0 };
  const uint8_t* seq = k_msg;
  size_t n = sizeof k_msg;
  switch (kind) {
    case PagerHaptic::Message: seq = k_msg;  n = sizeof k_msg;  break;
    case PagerHaptic::Direct:  seq = k_dm;   n = sizeof k_dm;   break;
    case PagerHaptic::Mention: seq = k_men;  n = sizeof k_men;  break;
    case PagerHaptic::Tick:    seq = k_tick; n = sizeof k_tick; break;
  }
  drvWrite(DRV_REG_MODE, 0x00);                        // wake, internal trigger
  for (size_t i = 0; i < n && i < 8; ++i) drvWrite(DRV_REG_WAVESEQ1 + i, seq[i]);
  drvWrite(DRV_REG_GO, 0x01);
  s_awake = true;
  s_check_ms = millis() + 250;
}

void pagerHapticsIdleTick() {
  if (!s_awake) return;
  const uint32_t now = millis();
  if ((int32_t)(now - s_check_ms) < 0) return;
  const int go = drvRead(DRV_REG_GO);
  if (go >= 0 && (go & 0x01)) { s_check_ms = now + 100; return; }   // still playing
  drvWrite(DRV_REG_MODE, DRV_MODE_STANDBY);
  s_awake = false;
}
