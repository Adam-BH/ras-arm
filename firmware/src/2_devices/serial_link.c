// LAYER 2 — DEVICES · serial link
#include "serial_link.h"
#include "../1_hal/hal.h"

static char    buf[LINE_MAX];
static uint8_t n;
static bool    ready;

char *serial_link_poll(void) {
  if (ready) { ready = false; n = 0; }       // the previous line was consumed
  int ch;
  while ((ch = hal_serial_read()) >= 0) {
    if (ch == '\r') continue;
    if (ch == '\n') { buf[n] = 0; ready = true; return buf; }
    if (n < LINE_MAX - 1) buf[n++] = (char)ch;   // too long: extra chars dropped
  }
  return 0;
}

void serial_link_send(const char *text) { hal_serial_write(text); }
