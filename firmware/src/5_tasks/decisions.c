// LAYER 5 — TASKS · decisions   ★ YOUR LOGIC LIVES HERE ★
//
// THE RULES (defaults in config.h, runtime-tunable with SET <key> <value>):
//
//  R1 EMPTY      clear channel < empty_clear × empty_ratio → nothing on the sensor.
//                empty_clear comes from CAL EMPTY.
//  R2 COLOUR     normalise by brightness (r/c, g/c, b/c) so the light level doesn't matter,
//                then take the nearest calibrated colour (CAL RED / CAL BLUE) — but only if it
//                is clearly closer: d_best < confident_ratio × d_other.
//  R3 2-COLOUR   only red and blue exist. If R2 is unsure and use_last_trick = 1, compare with
//                the last CONFIRMED cube: within same_as_last → same colour, else the other one.
//  R4 UNKNOWN    retry the reading, COLOUR_TRIES times in total, then ERROR.
//  R5 PICK OK    after a pick, the IR beam must be clear AND the colour sensor must read EMPTY.
//                Both sensors must agree.
//  R6 RETRIES    up to pick_retries extra picks after a failed VERIFY, then ERROR.
//  R7 JIGGLE     retry n shifts the grip point: 1 → (+jiggle_mm, 0, +jiggle_deg),
//                2 → (−jiggle_mm, 0, −jiggle_deg), 3 → (0, +jiggle_mm), 4 → (0, −jiggle_mm).
//  R8 WIGGLE     on retries the wrist (J6) wiggles ±grip_wiggle while the fingers close.
//  R9 RE-HOME    every rehome_every cycles, home again to catch lost steps (0 = never).
//
#include "decisions.h"
#include "../flash.h"
#include <math.h>

enum { K_CONFIDENT, K_SAME, K_EMPTY_RATIO, K_TRICK, K_WIGGLE, K_JIGGLE_MM, K_JIGGLE_DEG, K_RETRIES, K_REHOME };
static float P[DECIDE_NKEYS];

typedef struct { float r, g, b; } chroma_t;
static struct { chroma_t red, blue; uint16_t empty_clear; } cal;   // persisted by tasks (EEPROM)
static chroma_t last_good;
static colour_t last_colour = COL_UNKNOWN;

void decide_reset(void) {
  const float d[DECIDE_NKEYS] = {DEC_CONFIDENT_RATIO, DEC_SAME_AS_LAST, DEC_EMPTY_RATIO, DEC_USE_LAST_TRICK,
                                 DEC_GRIP_WIGGLE_DEG, DEC_JIGGLE_MM, DEC_JIGGLE_DEG, PICK_RETRIES, REHOME_EVERY};
  for (uint8_t i = 0; i < DECIDE_NKEYS; i++) P[i] = d[i];
  cal.red  = (chroma_t){DEC_REF_RED_R,  DEC_REF_RED_G,  DEC_REF_RED_B};
  cal.blue = (chroma_t){DEC_REF_BLUE_R, DEC_REF_BLUE_G, DEC_REF_BLUE_B};
  cal.empty_clear = DEC_EMPTY_CLEAR;
  last_colour = COL_UNKNOWN;
}

bool decide_set(const char *key, float v) {
  int8_t i = flash_find(PSTR(DECIDE_KEYS), DECIDE_NKEYS, key);
  if (i < 0) return false;
  P[i] = v;
  return true;
}
float decide_get(uint8_t i) { return P[i]; }
void *decide_cal_data(uint16_t *len) { *len = sizeof cal; return &cal; }

// ════════════════════════ COLOUR ════════════════════════
static chroma_t to_chroma(const rgbc_t *v) {
  float c = v->c ? (float)v->c : 1.0f;
  chroma_t k = {v->r / c, v->g / c, v->b / c};
  return k;
}
static float dist(chroma_t a, chroma_t b) {
  return sqrtf((a.r - b.r) * (a.r - b.r) + (a.g - b.g) * (a.g - b.g) + (a.b - b.b) * (a.b - b.b));
}

colour_t decide_colour(const rgbc_t *avg) {
  if (avg->c < cal.empty_clear * P[K_EMPTY_RATIO]) return COL_EMPTY;            // R1
  chroma_t k = to_chroma(avg);
  float dr = dist(k, cal.red), db = dist(k, cal.blue);
  if (dr < db * P[K_CONFIDENT]) return COL_RED;                                 // R2
  if (db < dr * P[K_CONFIDENT]) return COL_BLUE;
  if (P[K_TRICK] != 0 && (last_colour == COL_RED || last_colour == COL_BLUE)) { // R3
    colour_t other = (last_colour == COL_RED) ? COL_BLUE : COL_RED;
    return dist(k, last_good) < P[K_SAME] ? last_colour : other;
  }
  return COL_UNKNOWN;
}

void decide_colour_calibrate(colour_t c, const rgbc_t *avg) {
  if (c == COL_EMPTY) cal.empty_clear = avg->c;
  if (c == COL_RED)   cal.red  = to_chroma(avg);
  if (c == COL_BLUE)  cal.blue = to_chroma(avg);
}

void decide_colour_confirmed(colour_t c, const rgbc_t *avg) { last_colour = c; last_good = to_chroma(avg); }

unknown_policy_t decide_on_unknown(void)   { return ON_UNKNOWN_RETRY_THEN_ERROR; }      // R4
uint8_t          decide_colour_tries(void) { return COLOUR_TRIES; }

// ════════════════════════ GRIPPING ════════════════════════
uint8_t decide_max_pick_attempts(void) { return 1 + (uint8_t)P[K_RETRIES]; } // R6

void decide_grip_offset(uint8_t attempt, float *dx, float *dy, float *droll) {                 // R7
  float m = P[K_JIGGLE_MM], d = P[K_JIGGLE_DEG];
  *dx = *dy = *droll = 0;
  switch (attempt % 5) {
    case 1: *dx =  m; *droll =  d; break;
    case 2: *dx = -m; *droll = -d; break;
    case 3: *dy =  m; break;
    case 4: *dy = -m; break;
    default: break;
  }
}

float decide_grip_wiggle_deg(uint8_t attempt) { return attempt == 0 ? 0.0f : P[K_WIGGLE]; }  // R8

bool decide_pick_ok(bool ir_still_blocked, colour_t cradle_now) {                            // R5
  return !ir_still_blocked && cradle_now == COL_EMPTY;
}

// ════════════════════════ HOUSEKEEPING ════════════════════════
bool decide_rehome_now(uint16_t cycles_done) {                                               // R9
  uint16_t every = (uint16_t)P[K_REHOME];
  return every > 0 && cycles_done > 0 && cycles_done % every == 0;
}
