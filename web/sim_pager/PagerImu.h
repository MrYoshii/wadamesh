// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// T-LoRa Pager simulator: lying flat on a desk.
static inline bool pagerImuRead(float* x, float* y, float* z) { *x = 0; *y = 0; *z = 1.0f; return true; }
static inline void pagerImuIdleTick() {}
