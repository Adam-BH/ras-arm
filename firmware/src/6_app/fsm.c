// LAYER 6 — APP · state machine (the 11 states of the design)
#include "fsm.h"
#include "console.h"
#include "../5_tasks/tasks.h"
#include "../5_tasks/decisions.h"
#include "../2_devices/sensors.h"      // status reads only (IR, E-stop)
#include "../2_devices/actuators.h"    // status LED (non-motion I/O owned by the app)
#include "../flash.h"

static state_t     st = ST_BOOT;
static fsm_error_t err = E_NONE;
static bool        running;            // RUN = loop cycles, STOP = finish current cube then idle
static bool        homed;
static bool        manual;             // a manual move (MOVE, LINE, JOG, GO…) is running
static uint8_t     attempt, colour_tries;
static uint16_t    cycles;
static colour_t    cube_colour = COL_UNKNOWN, last_colour = COL_UNKNOWN, cal_target = COL_UNKNOWN;
static rgbc_t      cube_rgbc;
static uint32_t    t_enter;

state_t     fsm_state(void)       { return st; }
fsm_error_t fsm_error(void)       { return err; }
bool        fsm_homed(void)       { return homed; }
bool        fsm_running(void)     { return running; }
uint16_t    fsm_cycles(void)      { return cycles; }
colour_t    fsm_last_colour(void) { return last_colour; }
bool        fsm_busy(void)        { return manual || st == ST_HOMING || cal_target != COL_UNKNOWN; }

// entering a state = start its task once
static void enter(state_t s, uint32_t now) {
  st = s; t_enter = now;
  switch (s) {
    case ST_HOMING:      homed = false; task_homing_start(); break;
    case ST_READ_COLOUR: task_colour_start(); break;
    case ST_PICK:        task_pick_start(attempt); break;
    case ST_VERIFY:      task_colour_start(); break;          // re-read the cradle: empty now?
    case ST_PLACE:       task_place_start(cube_colour); break;
    case ST_RETURN:      task_return_start(); break;
    case ST_ERROR:
    case ST_ESTOP:       task_stop(); running = false; manual = false; cal_target = COL_UNKNOWN; break;
    default: break;
  }
}
static void fail(fsm_error_t e, uint32_t now) {
  err = e; enter(ST_ERROR, now);
  console_event(PSTR("ERROR"), flash_nth(PSTR(ERROR_NAMES), e), -1);
}

void fsm_init(void) {
  tasks_init();                     // sensors, joints, steppers ON, servos neutral, taught poses
  st = ST_BOOT; err = E_NONE; running = homed = manual = false;
  cycles = 0; cal_target = cube_colour = last_colour = COL_UNKNOWN;
  console_event(PSTR("BOOT"), 0, -1);
  if (!task_servos_ok()) console_event(PSTR("WARN"), PSTR("no_servo_driver"), -1);
  if (!task_colour_ok()) console_event(PSTR("WARN"), PSTR("no_colour_sensor"), -1);
}

static void led(uint32_t now) {
  uint8_t pattern = st == ST_ESTOP ? LED_SOLID : st == ST_ERROR ? (uint8_t)err : 0;
  status_led_update(now, pattern);
}

void fsm_tick(uint32_t now) {
  tasks_update(now);
  led(now);

  // ⚡ E-stop first, from ANY state
  if (estop_active()) {
    if (st != ST_ESTOP) { err = E_ESTOP; homed = false; enter(ST_ESTOP, now); console_event(PSTR("ESTOP"), 0, -1); }
    return;
  }

  if (manual && !task_busy()) manual = false;

  colour_t c; rgbc_t avg; task_status_t r;
  switch (st) {
    case ST_BOOT:
      if (now - t_enter > BOOT_SETTLE_MS) enter(ST_HOMING, now);   // let servos settle
      break;

    case ST_HOMING:
      r = task_homing_update(now);
      if (r == TASK_DONE) { homed = true; console_event(PSTR("HOMED"), 0, -1); enter(ST_IDLE, now); }
      if (r == TASK_FAILED) {
        console_event(PSTR("HOME_FAIL"), task_homing_why(), task_homing_joint() + 1);
        fail(E_HOME_FAILED, now);
      }
      break;

    case ST_IDLE:
      if (running && !manual && ir_object()) { colour_tries = 0; attempt = 0; enter(ST_READ_COLOUR, now); }
      break;

    case ST_READ_COLOUR:
      if (task_colour_update(now, &c, &avg) != TASK_DONE) break;
      last_colour = c;
      if ((c == COL_RED || c == COL_BLUE) && stations_bin_for(c) >= 0) cube_colour = c;
      else if (++colour_tries < decide_colour_tries() && decide_on_unknown() != ON_UNKNOWN_ERROR) { task_colour_start(); break; }
      else if (decide_unknown_target(stations_bin_for(COL_UNKNOWN)) >= 0) cube_colour = COL_UNKNOWN;   // → reject bin
      else { fail(E_COLOUR_UNKNOWN, now); break; }
      cube_rgbc = avg;
      console_event(PSTR("COLOUR"), flash_nth(PSTR(COLOUR_NAMES), cube_colour), -1);
      enter(ST_PICK, now);
      break;

    case ST_PICK:
      r = task_pick_update(now);
      if (r == TASK_DONE)   enter(ST_VERIFY, now);
      if (r == TASK_FAILED) fail(E_UNREACHABLE, now);
      break;

    case ST_VERIFY:
      if (task_colour_update(now, &c, &avg) != TASK_DONE) break;
      last_colour = c;
      if (decide_pick_ok(ir_object(), c)) {
        decide_colour_confirmed(cube_colour, &cube_rgbc);
        console_event(PSTR("PICKED"), 0, attempt);
        enter(ST_PLACE, now);
      }
      else if (++attempt < decide_max_pick_attempts()) {       // jiggle retry
        console_event(PSTR("PICK_RETRY"), 0, attempt);
        enter(ST_PICK, now);
      }
      else fail(E_PICK_FAILED, now);
      break;

    case ST_PLACE:
      r = task_place_update(now);
      if (r == TASK_DONE)   { console_event(PSTR("PLACED"), flash_nth(PSTR(COLOUR_NAMES), cube_colour), -1); enter(ST_RETURN, now); }
      if (r == TASK_FAILED) fail(E_UNREACHABLE, now);
      break;

    case ST_RETURN:
      r = task_return_update(now);
      if (r == TASK_FAILED) { fail(E_UNREACHABLE, now); break; }
      if (r != TASK_DONE) break;
      cycles++;
      console_event(PSTR("CYCLE"), 0, (int32_t)cycles);
      if (decide_rehome_now(cycles)) { console_event(PSTR("REHOME"), 0, -1); enter(ST_HOMING, now); }  // drift check
      else enter(ST_IDLE, now);
      break;

    case ST_TEACH:
      if (cal_target != COL_UNKNOWN && task_colour_update(now, &c, &avg) == TASK_DONE) {
        task_calibrate_finish(cal_target, &avg);
        console_event(PSTR("CAL"), flash_nth(PSTR(COLOUR_NAMES), cal_target), -1);
        cal_target = COL_UNKNOWN;
      }
      break;

    case ST_ERROR:
    case ST_ESTOP:
    case ST_COUNT:
      break;                                                   // wait for RESET
  }
}

fsm_reply_t fsm_command(cmd_t cmd, uint32_t now) {
  switch (cmd) {
    case CMD_HOME:
      if (st != ST_IDLE && st != ST_TEACH && st != ST_ERROR) return R_STATE;
      if (estop_active()) return R_ESTOP;
      if (manual || running) return manual ? R_BUSY : R_RUNNING;
      err = E_NONE; enter(ST_HOMING, now); return R_OK;
    case CMD_RUN:
      if (st != ST_IDLE) return R_STATE;
      if (manual) return R_BUSY;
      running = true; return R_OK;
    case CMD_STOP:
      running = false;
      if (manual) { task_stop(); manual = false; }
      return R_OK;
    case CMD_TEACH:
      if (st != ST_IDLE) return R_STATE;
      if (manual) return R_BUSY;
      running = false; enter(ST_TEACH, now); return R_OK;
    case CMD_DONE:
      if (st != ST_TEACH) return R_STATE;
      cal_target = COL_UNKNOWN; enter(ST_IDLE, now);
      if (stations_dirty()) console_event(PSTR("WARN"), PSTR("uncommitted_layout"), -1);   // stays in RAM until reset
      return R_OK;
    case CMD_RESET:
      if (st != ST_ERROR && st != ST_ESTOP) return R_STATE;
      if (estop_active()) return R_ESTOP;
      err = E_NONE; enter(ST_HOMING, now); return R_OK;
  }
  return R_STATE;
}

// manual moves: IDLE (not running) or TEACH, one at a time
static fsm_reply_t can_move(bool teach_only) {
  if (st != ST_TEACH && (teach_only || st != ST_IDLE)) return R_STATE;
  if (running) return R_RUNNING;
  if (fsm_busy()) return R_BUSY;
  return R_OK;
}
static fsm_reply_t started(bool ok, fsm_reply_t why) { if (!ok) return why; manual = true; return R_OK; }

#define GUARD(teach_only) do { fsm_reply_t g = can_move(teach_only); if (g != R_OK) return g; } while (0)
fsm_reply_t fsm_move(const pose_t *p, uint32_t ms)    { GUARD(false); return started(task_move(p, ms), R_UNREACHABLE); }
fsm_reply_t fsm_line(const pose_t *p, float mm_s)     { GUARD(false); return started(task_line(p, mm_s), R_UNREACHABLE); }
fsm_reply_t fsm_movej(const angles_t *a, uint32_t ms) { GUARD(false); return started(task_movej(a, ms), R_LIMIT); }
fsm_reply_t fsm_grip(float mm)                        { GUARD(false); task_grip(mm); return started(true, R_OK); }
fsm_reply_t fsm_jog(float dx, float dy, float dz, float dr) { GUARD(true); return started(task_jog(dx, dy, dz, dr), R_UNREACHABLE); }
fsm_reply_t fsm_jogj(uint8_t j, float d)              { GUARD(true); return started(task_jogj(j, d), R_LIMIT); }
fsm_reply_t fsm_go(uint8_t i)                         { GUARD(true); return started(task_teach_go(i), R_UNREACHABLE); }
fsm_reply_t fsm_preview(uint8_t i)                    { GUARD(true); return started(task_preview_start(i), R_UNREACHABLE); }
fsm_reply_t fsm_save(uint8_t i)                       { GUARD(true); st_err_t e = task_teach_save(i); return e ? (fsm_reply_t)(R_ST + e) : R_OK; }
bool        fsm_teaching(void)                        { return st == ST_TEACH; }
fsm_reply_t fsm_calibrate(colour_t c)                 { GUARD(true); cal_target = c; task_colour_start(); return R_OK; }
fsm_reply_t fsm_can_tune(void)                        { return (st == ST_IDLE || st == ST_TEACH) ? R_OK : R_STATE; }
