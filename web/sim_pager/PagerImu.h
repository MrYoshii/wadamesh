// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stdlib.h>
// T-LoRa Pager simulator: lying flat on a desk (WADA_SIM_FACEDOWN=1: face down).
static inline bool pagerImuRead(float* x, float* y, float* z) {
  static const bool down = getenv("WADA_SIM_FACEDOWN") != nullptr;
  *x = 0; *y = 0; *z = down ? -1.0f : 1.0f;
  return true;
}
static inline void pagerImuIdleTick() {}
