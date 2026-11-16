// flash.h — keep constant strings in FLASH on the UNO (RAM is only 2 KB).
// Plain C (avr-libc, not Arduino). On the PC the same code reads normal memory.
// A string list is "A\0B\0C\0" (one literal, implicit final \0 ends the last item).
#pragma once
#include <stdint.h>
#include <string.h>

#ifdef __AVR__
#include <avr/pgmspace.h>
#define FLASH        PROGMEM
#define FBYTE(p)     ((char)pgm_read_byte(p))
#define FFLOAT(p)    pgm_read_float(p)
#define FMEMCPY(d, s, n) memcpy_P(d, s, n)
#else
#define FLASH
#define PSTR(s)      (s)
#define FBYTE(p)     (*(const char *)(p))
#define FFLOAT(p)    (*(p))
#define FMEMCPY(d, s, n) memcpy(d, s, n)
#endif

// pointer (in flash) to the i-th item of a string list
static inline const char *flash_nth(const char *list, uint8_t i) {
  while (i--) while (FBYTE(list++)) {}
  return list;
}

// index of 'word' (RAM) in a flash string list of n items, or -1
static inline int8_t flash_find(const char *list, uint8_t n, const char *word) {
  for (uint8_t i = 0; i < n; i++) {
    const char *f = list; const char *w = word;
    while (FBYTE(f) && FBYTE(f) == *w) { f++; w++; }
    if (!FBYTE(f) && !*w) return (int8_t)i;
    while (FBYTE(list++)) {}
  }
  return -1;
}
