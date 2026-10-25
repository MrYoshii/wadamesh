#pragma once
// T-LoRa Pager simulator: the vibration motor prints what it would play.
#include <stdint.h>
#include <stdio.h>
enum class PagerHaptic : uint8_t { Message, Direct, Mention, Tick };
static inline void pagerHapticsBegin() {}
static inline bool pagerHapticsPresent() { return true; }
static inline void pagerHapticsPlay(PagerHaptic k) {
  static const char* n[] = { "message", "direct", "mention", "tick" };
  if (k != PagerHaptic::Tick) printf("[sim] vibrate: %s\n", n[(int)k]);
}
static inline void pagerHapticsIdleTick() {}
