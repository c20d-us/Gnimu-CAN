// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace gnimu {

// Volts as whole volts and hundredths, rounded, for "%lu.%02lu": the console
// prints integers only (no float format, see Report.h). Below 0 reads 0.
struct CentiVolts {
  unsigned long whole;
  unsigned long hundredths;
};

inline CentiVolts centiVolts(float v) {
  const unsigned long c = v <= 0.0f ? 0UL : static_cast<unsigned long>(v * 100.0f + 0.5f);
  return {c / 100, c % 100};
}

namespace detail {

// Digit count of `v` in base 10 (at least 1, for 0 itself).
inline size_t decimalWidth(unsigned long v) {
  size_t w = 1;
  while (v >= 10) {
    v /= 10;
    ++w;
  }
  return w;
}

// Characters `s` occupies on screen: UTF-8 continuation bytes (10xxxxxx)
// don't count, so "max \xCE\xBCs" is 6 wide, not 7. Right for single-width
// symbols like \xCE\xBC and \xC2\xB0; wide emoji would count as 1 but draw as 2, so
// keep them out of padded cells.
inline size_t displayWidth(const char *s) {
  size_t w = 0;
  for (; *s; s++)
    if ((static_cast<unsigned char>(*s) & 0xC0) != 0x80)
      ++w;
  return w;
}

// Bounded string builder: every append checks remaining capacity first, so a
// caller's buffer is truncated, never overrun. Shared by Report.h and
// StatusLine.h rather than each keeping its own copy.
struct Out {
  char *buf;
  size_t cap;
  size_t len;
  void put(const char *fmt, ...) __attribute__((format(printf, 2, 3))) {
    if (len + 1 >= cap)
      return;
    va_list ap;
    va_start(ap, fmt);
    const int n = vsnprintf(buf + len, cap - len, fmt, ap);
    va_end(ap);
    if (n > 0)
      len += (static_cast<size_t>(n) < cap - len) ? static_cast<size_t>(n)
                                                  : cap - len - 1;
  }
  // Appends `n` copies of `c` directly, with the same never-overrun contract
  // as put() - used for padding and rule lines without relying on printf's
  // '*' dynamic-width specifier, which isn't guaranteed present on a
  // newlib-nano build (the likely libc on the nRF52 target).
  void fill(char c, size_t n) {
    for (size_t i = 0; i < n && len + 1 < cap; i++)
      buf[len++] = c;
    if (cap)
      buf[len] = '\0';
  }
  void spaces(size_t n) { fill(' ', n); }
  // Left-justified text cell, padded with trailing spaces to width `w`.
  void left(const char *s, size_t w) {
    put("%s", s);
    const size_t n = displayWidth(s);
    if (n < w)
      spaces(w - n);
  }
  // Right-justified text or number cell, padded with leading spaces to width
  // `w`.
  void right(const char *s, size_t w) {
    const size_t n = displayWidth(s);
    if (n < w)
      spaces(w - n);
    put("%s", s);
  }
  void right(unsigned long v, size_t w) {
    const size_t n = decimalWidth(v);
    if (n < w)
      spaces(w - n);
    put("%lu", v);
  }
};

} // namespace detail
} // namespace gnimu
