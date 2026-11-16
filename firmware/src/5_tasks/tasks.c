// LAYER 5 — TASKS · robot skills (homing, colour, pick, place, return, manual moves, teach)
#include "tasks.h"
#include "decisions.h"
#include "../2_devices/sensors.h"
#include "../2_devices/storage.h"
#include "../flash.h"

static bool colour_ok;

void tasks_init(void) {
  sensors_init();
  colour_ok = colour_init();
  motion_init();                                      // joints, steppers ON, servos neutral
  stations_init();                                    // workcell layout: EEPROM or CAD defaults
  decide_reset();
  uint16_t n; void *cal = decide_cal_data(&n);
  storage_load(EE_ADDR_CAL, EE_MAGIC_CAL, cal, n);     // bad CRC → defaults stay
}

static int8_t pv = -1;                                // ST GO preview: station to line down onto
void tasks_update(uint32_t now) {
  sensors_update(now); motion_update(now);
  if (pv >= 0 && !motion_busy()) { pose_t g = stations_grip((uint8_t)pv); pv = -1; motion_line(&g, PREVIEW_MM_S); }
}
bool task_servos_ok(void)       { return motion_servos_ok(); }
bool task_colour_ok(void)       { return colour_ok; }

static float absf(float v) { return v < 0 ? -v : v; }

// ════════════════════════ HOMING ════════════════════════
// Order J2 → J3 → J1. Each joint: (leave the switch if already pressed) → FAST to the switch →
// BACK OFF → SLOW to the switch → counter := switch angle. Then go to HOME.
// Every phase has a travel cap, so a broken wire (switch reads "pressed" forever — NC wiring)
// or a dead switch (never pressed) ends in TASK_FAILED instead of driving into a hard stop.
typedef enum { H_START, H_LEAVE, H_FAST, H_BACKOFF, H_SLOW, H_NEXT, H_GO_HOME } hphase_t;
static const uint8_t HOME_ORDER[N_STEPPERS] = {J2, J3, J1};
static hphase_t hp; static uint8_t hi; static uint32_t ht0; static float ha0;
static const char *h_why;

static void hgo(hphase_t p, uint8_t j, float dps, uint32_t now) {
  hp = p; ht0 = now; ha0 = motion_joint_angle(j);
  motion_run_toward_switch(j, dps);
}
static task_status_t hfail(const char *why) { motion_stop(); h_why = why; return TASK_FAILED; }

void        task_homing_start(void) { hp = H_START; hi = 0; h_why = PSTR("none"); }
const char *task_homing_why(void)   { return h_why; }
uint8_t     task_homing_joint(void) { return HOME_ORDER[hi < N_STEPPERS ? hi : N_STEPPERS - 1]; }

task_status_t task_homing_update(uint32_t now) {
  uint8_t j = task_homing_joint();
  float travel = absf(motion_joint_angle(j) - ha0);
  if (hp != H_START && hp != H_NEXT && hp != H_GO_HOME && now - ht0 > HOMING_PHASE_TIMEOUT_MS)
    return hfail(PSTR("timeout"));
  switch (hp) {
    case H_START:
      if (limit_pressed(j)) hgo(H_LEAVE, j, -HOMING_FAST_DPS, now);
      else                  hgo(H_FAST,  j,  HOMING_FAST_DPS, now);
      break;
    case H_LEAVE:                                   // started ON the switch: get off it first
      if (!limit_pressed(j))              hgo(H_FAST, j, HOMING_FAST_DPS, now);
      else if (travel > HOMING_LEAVE_MAX) return hfail(PSTR("broken_wire"));
      break;
    case H_FAST:
      if (limit_pressed(j)) hgo(H_BACKOFF, j, -HOMING_FAST_DPS, now);
      else if (travel > JOINT_MAX[j] - JOINT_MIN[j] + HOMING_SEARCH_MARGIN) return hfail(PSTR("not_found"));
      break;
    case H_BACKOFF:
      if (travel >= HOMING_BACKOFF && !limit_pressed(j)) hgo(H_SLOW, j, HOMING_SLOW_DPS, now);
      else if (travel > HOMING_LEAVE_MAX)                return hfail(PSTR("stuck"));
      break;
    case H_SLOW:
      if (limit_pressed(j))              { motion_set_homed(j); hp = H_NEXT; }   // the click we trust
      else if (travel > HOMING_SLOW_MAX) return hfail(PSTR("not_found"));
      break;
    case H_NEXT:
      if (++hi < N_STEPPERS) { hp = H_START; break; }
      { pose_t home = stations_grip(STATION_HOME);
        if (!motion_to(&home, MOVE_HOME_MS)) return hfail(PSTR("home_pose")); }
      hp = H_GO_HOME;
      break;
    case H_GO_HOME:
      if (!motion_busy()) return TASK_DONE;
      break;
  }
  return TASK_RUNNING;
}

// ════════════════════════ READ COLOUR (average of COLOUR_SAMPLES) ════════════════════════
static uint8_t cn; static uint32_t sr, sg, sb, sc, ct0;
void task_colour_start(void) { cn = 0; sr = sg = sb = sc = 0; colour_start(); ct0 = 0; }

task_status_t task_colour_update(uint32_t now, colour_t *out, rgbc_t *avg) {
  if (ct0 == 0) ct0 = now ? now : 1;
  if (!colour_ready()) {
    if (now - ct0 < COLOUR_TIMEOUT_MS) return TASK_RUNNING;
    avg->r = avg->g = avg->b = avg->c = 0; *out = COL_UNKNOWN;   // sensor not answering
    return TASK_DONE;
  }
  rgbc_t v = colour_get(); sr += v.r; sg += v.g; sb += v.b; sc += v.c;
  ct0 = now ? now : 1;
  if (++cn < COLOUR_SAMPLES) { colour_start(); return TASK_RUNNING; }
  avg->r = sr / cn; avg->g = sg / cn; avg->b = sb / cn; avg->c = sc / cn;
  *out = decide_colour(avg);
  return TASK_DONE;
}

// ════════════════════════ PICK ════════════════════════
// above (open) → straight line down → close (+ wiggle on retries) → straight line up
typedef enum { P_ABOVE, P_DOWN, P_CLOSE, P_WIGGLE, P_UP, P_END } pphase_t;
static pphase_t pp; static pose_t p_grip, p_above; static uint8_t p_attempt; static float p_j6;

void task_pick_start(uint8_t attempt) {
  float dx, dy, dr; decide_grip_offset(attempt, &dx, &dy, &dr);
  p_attempt = attempt;
  uint8_t i = (uint8_t)stations_find_type(STN_PICK, 0);   // always exists (protected)
  p_grip = stations_grip(i); p_grip.x += dx; p_grip.y += dy; p_grip.roll += dr;
  p_above = p_grip; p_above.z = stations_above(i).z;
  motion_grip(GRIP_OPEN_MM, GRIP_MOVE_MS);
  pp = motion_to(&p_above, MOVE_ABOVE_MS) ? P_ABOVE : P_END;
}

task_status_t task_pick_update(uint32_t now) {
  (void)now;
  if (pp == P_END) return TASK_FAILED;              // above pose unreachable
  if (motion_busy()) return TASK_RUNNING;
  float w = decide_grip_wiggle_deg(p_attempt);
  switch (pp) {
    case P_ABOVE: if (!motion_line(&p_grip, PICK_DOWN_MM_S)) return TASK_FAILED; pp = P_DOWN; break;
    case P_DOWN:  motion_grip(GRIP_CLOSED_MM, GRIP_MOVE_MS); pp = P_CLOSE;
                  p_j6 = motion_joint_angle(J6);
                  if (w > 0) { motion_joint(J6, p_j6 + w, WIGGLE_MS); pp = P_WIGGLE; }
                  break;
    case P_WIGGLE: motion_joint(J6, p_j6, WIGGLE_MS); pp = P_CLOSE; break;   // back to centre
    case P_CLOSE: if (!motion_line(&p_above, PICK_UP_MM_S)) return TASK_FAILED; pp = P_UP; break;
    case P_UP:    return TASK_DONE;
    case P_END:   return TASK_FAILED;
  }
  return TASK_RUNNING;
}

// ════════════════════════ PLACE ════════════════════════
// transit above the bin → straight line down → open → straight line up
typedef enum { L_TRANSIT, L_DOWN, L_OPEN, L_UP, L_END } lphase_t;
static lphase_t lp; static pose_t l_drop, l_above;

void task_place_start(colour_t c) {
  int8_t i = stations_bin_for(c);                      // colour → bin map (ST MAP)
  if (i < 0) { lp = L_END; return; }
  l_drop = stations_grip((uint8_t)i); l_above = stations_above((uint8_t)i);
  lp = motion_to(&l_above, MOVE_TRANSIT_MS) ? L_TRANSIT : L_END;
}

task_status_t task_place_update(uint32_t now) {
  (void)now;
  if (lp == L_END) return TASK_FAILED;
  if (motion_busy()) return TASK_RUNNING;
  switch (lp) {
    case L_TRANSIT: if (!motion_line(&l_drop, PLACE_DOWN_MM_S)) return TASK_FAILED; lp = L_DOWN; break;
    case L_DOWN:    motion_grip(GRIP_OPEN_MM, GRIP_MOVE_MS); lp = L_OPEN; break;
    case L_OPEN:    if (!motion_line(&l_above, PLACE_UP_MM_S)) return TASK_FAILED; lp = L_UP; break;
    case L_UP:      return TASK_DONE;
    case L_END:     return TASK_FAILED;
  }
  return TASK_RUNNING;
}

// ════════════════════════ RETURN ════════════════════════
static bool r_ok;
void task_return_start(void) { pose_t h = stations_grip(STATION_HOME); r_ok = motion_to(&h, MOVE_HOME_MS); }
task_status_t task_return_update(uint32_t now) {
  (void)now;
  if (!r_ok) return TASK_FAILED;
  return motion_busy() ? TASK_RUNNING : TASK_DONE;
}

// ════════════════════════ MANUAL MOVES ════════════════════════
bool task_move(const pose_t *p, uint32_t ms)   { return motion_to(p, ms); }
bool task_line(const pose_t *p, float mm_s)    { return motion_line(p, mm_s); }
bool task_movej(const angles_t *a, uint32_t ms) { return motion_joints(a, ms); }
bool task_jog(float dx, float dy, float dz, float dr) { return motion_jog(dx, dy, dz, dr); }
bool task_jogj(uint8_t j, float delta)         { return motion_jog_joint(j, delta); }
void task_grip(float mm)                       { motion_grip(mm, GRIP_MOVE_MS); }
bool task_busy(void)                           { return motion_busy() || pv >= 0; }
void task_stop(void)                           { pv = -1; motion_stop(); }
void task_where(pose_t *out)                   { motion_where(out); }

// ════════════════════════ TEACH ════════════════════════
st_err_t task_teach_save(uint8_t i) {                  // SAVE <name>: here, validated, committed at once
  pose_t p; motion_where(&p);
  st_err_t e = stations_set(i, &p, stations_get(i)->above_dz);
  if (!e) stations_commit();
  return e;
}
bool task_teach_go(uint8_t i)  { pose_t p = stations_grip(i); return motion_to(&p, MOVE_HOME_MS); }
bool task_preview_start(uint8_t i) {                   // ST GO: above the station, then a slow line down
  pose_t a = stations_above(i);
  if (!motion_to(&a, MOVE_HOME_MS)) return false;
  pv = (int8_t)i;
  return true;
}

void task_calibrate_finish(colour_t c, const rgbc_t *avg) {
  decide_colour_calibrate(c, avg);
  uint16_t n; void *cal = decide_cal_data(&n);
  storage_save(EE_ADDR_CAL, EE_MAGIC_CAL, cal, n);
}
