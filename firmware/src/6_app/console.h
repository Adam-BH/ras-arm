// LAYER 6 — APP · console (serial protocol + telemetry). Full spec: docs/serial_protocol.md
// Line-based ASCII, 115200 baud. Every command gets exactly one reply:
//   OK <CMD> [data]   or   ERR <CMD> <reason>
// Moves also send "DONE <CMD>" when they finish. Events: "EV <what> [arg] [n]".
// Telemetry (20 Hz, integers only):
//   T,<ms>,<state>,<j1..j6 ×10>,<grip ×10>,<x>,<y>,<z>,<ir>,<colour>,<err>,<lim1><lim2><lim3>,<estop>,<layout>
//   layout = station table version (+1 per ST COMMIT): re-read ST LIST when it changes
#pragma once
#include "../config.h"

void console_update(uint32_t now);
// "EV <what> [arg] [n]": what/arg are FLASH strings (PSTR), arg may be NULL, n < 0 = none
void console_event(const char *what, const char *arg, int32_t n);
