// LAYER 2 — DEVICES · serial link (the USB cable to the PC / teach UI)
// Turns the raw byte stream into complete text lines, and sends text back.
#pragma once
#include "../config.h"

#define LINE_MAX 72                       // longest command: "MOVEJ -100.0 -20.0 150.0 -90.0 -100.0 -90.0 20000"

char *serial_link_poll(void);             // a full line ("...\n" stripped) or NULL; valid until the next poll
void  serial_link_send(const char *text);
