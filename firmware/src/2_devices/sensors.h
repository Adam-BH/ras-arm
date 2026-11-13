// LAYER 2 — DEVICES · sensors
// Reads every sensor and cleans the signal (debounce, NC logic, I²C protocol).
// Answers simple questions: "is switch 2 pressed?", "is there a cube?", "what colour values?"
#pragma once
#include "../config.h"

void sensors_init(void);
void sensors_update(uint32_t now);      // call every loop: debouncing

// limit switches (homing) — wired NC: pressed OR broken wire = true (fail-safe)
bool limit_pressed(uint8_t axis);

// E-stop / motor power sense
bool estop_active(void);

// E18 IR at the cradle — true when the beam is blocked for IR_STABLE_MS
bool ir_object(void);

// TCS34725 colour sensor (non-blocking: start, wait ready, get)
bool   colour_init(void);
void   colour_start(void);
bool   colour_ready(void);
rgbc_t colour_get(void);
