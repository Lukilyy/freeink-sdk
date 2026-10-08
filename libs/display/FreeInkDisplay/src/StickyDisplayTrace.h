#pragma once

#include <cstdint>

extern "C" void freeink_board_sticky_display_trace(const char* phase, uint8_t value, bool hasValue, bool error)
    __attribute__((weak));

inline void traceStickyDisplay(const char* phase, uint8_t value = 0, bool hasValue = false, bool error = false) {
  if (freeink_board_sticky_display_trace) freeink_board_sticky_display_trace(phase, value, hasValue, error);
}
