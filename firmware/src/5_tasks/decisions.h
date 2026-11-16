// LAYER 5 — TASKS · decisions   ★ YOUR LOGIC LIVES HERE ★
// Every "what should the robot decide?" rule in ONE small file: colour logic,
// grip strategy (jiggle/wiggle), what counts as a successful pick, retries, re-homing.
// Tasks and the FSM ask these functions; change the rules here without touching the sequences.
// Pure logic: no hardware, no I/O — fully unit-tested on the PC.
#pragma once
#include "../config.h"

// ── colour ──
colour_t decide_colour(const rgbc_t *avg);                       // RED / BLUE / EMPTY / UNKNOWN
void     decide_colour_calibrate(colour_t c, const rgbc_t *avg); // CAL RED|BLUE|EMPTY
void     decide_colour_confirmed(colour_t c, const rgbc_t *avg); // remember the last good cube
void    *decide_cal_data(uint16_t *len);                         // calibration block (for EEPROM)

typedef enum { ON_UNKNOWN_ERROR, ON_UNKNOWN_RETRY_THEN_ERROR } unknown_policy_t;
unknown_policy_t decide_on_unknown(void);
uint8_t  decide_colour_tries(void);                              // reads before giving up
int8_t   decide_unknown_target(int8_t reject_bin);               // after the tries: bin index, or -1 = ERROR

// ── gripping ──
uint8_t decide_max_pick_attempts(void);
void    decide_grip_offset(uint8_t attempt, float *dx, float *dy, float *droll); // jiggle pattern
float   decide_grip_wiggle_deg(uint8_t attempt);                // J6 wiggle while closing (0 = off)
bool    decide_pick_ok(bool ir_still_blocked, colour_t cradle_now);

// ── housekeeping ──
bool decide_rehome_now(uint16_t cycles_done);                   // drift check every N cycles

// ── runtime tuning: SET <key> <value> ──
#define DECIDE_KEYS "confident_ratio\0same_as_last\0empty_ratio\0use_last_trick\0grip_wiggle\0jiggle_mm\0jiggle_deg\0pick_retries\0rehome_every"
#define DECIDE_NKEYS 9
bool  decide_set(const char *key, float value);                 // false = unknown key
float decide_get(uint8_t i);
void  decide_reset(void);                                       // defaults (called by tasks_init)
