// LAYER 6 — APP · console (serial protocol + telemetry)
#include "console.h"
#include "fsm.h"
#include "../5_tasks/tasks.h"
#include "../5_tasks/decisions.h"
#include "../2_devices/serial_link.h"
#include "../2_devices/sensors.h"     // status reads only
#include "../3_joints/joints.h"       // status reads only (angles for telemetry)
#include "../flash.h"
#include <string.h>
#include <stdlib.h>

#define COMMANDS "PING\0STATUS\0HOME\0RUN\0STOP\0RESET\0TEACH\0DONE\0MOVE\0LINE\0MOVEJ\0JOG\0JOGJ\0GRIP\0SAVE\0GO\0POSES\0CAL\0SET"
enum { C_PING, C_STATUS, C_HOME, C_RUN, C_STOP, C_RESET, C_TEACH, C_DONE, C_MOVE, C_LINE, C_MOVEJ,
       C_JOG, C_JOGJ, C_GRIP, C_SAVE, C_GO, C_POSES, C_CAL, C_SET, C_COUNT };
#define CAL_NAMES "EMPTY\0RED\0BLUE"
#define MAX_ARGS 8
#define OUT_MAX  104

static uint32_t t_tel;
static int8_t   pending_done = -1;        // command waiting for its "DONE <CMD>"

// ── output line builder: one serial write per line ──
static char    out[OUT_MAX];
static uint8_t on;
static void o_c(char c)          { if (on < OUT_MAX - 2) out[on++] = c; }
static void o_s(const char *s)   { while (*s) o_c(*s++); }
static void o_p(const char *p)   { char c; while ((c = FBYTE(p++))) o_c(c); }   // flash string
static void o_i(int32_t v) {
  char d[11]; uint8_t k = 0; uint32_t u = v < 0 ? (uint32_t)-v : (uint32_t)v;
  if (v < 0) o_c('-');
  do { d[k++] = (char)('0' + u % 10); u /= 10; } while (u);
  while (k) o_c(d[--k]);
}
static int32_t rmm(float v) { return (int32_t)(v + (v < 0 ? -0.5f : 0.5f)); }          // rounded
static int32_t r10(float v) { return rmm(v * 10.0f); }                                   // ×10, rounded
static void o_f1(float v) {                                   // one decimal, no printf("%f")
  int32_t t = r10(v);
  if (t < 0) { o_c('-'); t = -t; }
  o_i(t / 10); o_c('.'); o_c((char)('0' + t % 10));
}
static void o_send(void) { out[on++] = '\n'; out[on] = 0; serial_link_send(out); on = 0; }

void console_event(const char *what, const char *arg, int32_t n) {
  on = 0; o_p(PSTR("EV ")); o_p(what);
  if (arg) { o_c(' '); o_p(arg); }
  if (n >= 0) { o_c(' '); o_i(n); }
  o_send();
}

static void ok(const char *cmd)  { on = 0; o_p(PSTR("OK ")); o_s(cmd); }   // caller may add data, then o_send
static void err_reply(const char *cmd, const char *reason_P) {
  on = 0; o_p(PSTR("ERR ")); o_s(cmd); o_c(' '); o_p(reason_P); o_send();
}

// strict number parsing: the whole token must be a number
static bool num(const char *s, float *v) {
  char *end; *v = (float)strtod(s, &end);
  return end != s && *end == 0;
}

static void upcase(char *s) { for (; *s; s++) if (*s >= 'a' && *s <= 'z') *s -= 'a' - 'A'; }

static void handle(char *line, uint32_t now) {
  char *tok[MAX_ARGS + 1]; uint8_t n = 0;
  for (char *t = strtok(line, " \t"); t && n <= MAX_ARGS; t = strtok(0, " \t")) tok[n++] = t;
  if (n == 0) return;                                          // empty line: not a command
  char *cmd = tok[0]; upcase(cmd);
  uint8_t argc = n - 1;
  int8_t c = flash_find(PSTR(COMMANDS), C_COUNT, cmd);
  if (c < 0) { err_reply(cmd, PSTR("unknown")); return; }

  // numeric arguments (pose / colour / key names are parsed separately)
  float a[MAX_ARGS] = {0};
  bool numeric = c == C_MOVE || c == C_LINE || c == C_MOVEJ || c == C_JOG || c == C_JOGJ || c == C_GRIP;
  if (numeric) for (uint8_t i = 0; i < argc; i++) if (!num(tok[i + 1], &a[i])) { err_reply(cmd, PSTR("args")); return; }

  fsm_reply_t r = R_OK;
  bool is_move = false;
  switch (c) {
    case C_PING:
      ok(cmd); o_p(PSTR(" ras-arm " FW_VERSION)); o_send(); return;
    case C_STATUS:
      ok(cmd); o_c(' '); o_p(flash_nth(PSTR(STATE_NAMES), fsm_state()));
      o_c(' '); o_p(flash_nth(PSTR(ERROR_NAMES), fsm_error()));
      o_p(PSTR(" homed=")); o_i(fsm_homed()); o_p(PSTR(" running=")); o_i(fsm_running());
      o_send(); return;
    case C_POSES:
      for (uint8_t i = 0; i < POSE_COUNT; i++) {
        pose_t p = task_pose((pose_id_t)i);
        on = 0; o_p(PSTR("POSE ")); o_p(flash_nth(PSTR(POSE_NAMES), i));
        o_c(' '); o_f1(p.x); o_c(' '); o_f1(p.y); o_c(' '); o_f1(p.z); o_c(' '); o_f1(p.roll); o_send();
      }
      break;
    case C_HOME:  r = fsm_command(CMD_HOME, now); is_move = true; break;
    case C_RUN:   r = fsm_command(CMD_RUN, now); break;
    case C_STOP:  r = fsm_command(CMD_STOP, now); break;
    case C_RESET: r = fsm_command(CMD_RESET, now); break;
    case C_TEACH: r = fsm_command(CMD_TEACH, now); break;
    case C_DONE:  r = fsm_command(CMD_DONE, now); break;
    case C_MOVE: case C_LINE: {
      if (argc < 3 || argc > (c == C_MOVE ? 5 : 4)) { err_reply(cmd, PSTR("args")); return; }
      pose_t p = {a[0], a[1], a[2], c == C_MOVE && argc >= 4 ? a[3] : 0};
      r = c == C_MOVE ? fsm_move(&p, argc == 5 ? (uint32_t)a[4] : 0) : fsm_line(&p, argc == 4 ? a[3] : LINE_DEFAULT_MM_S);
      is_move = true; break; }
    case C_MOVEJ: {
      if (argc < 6 || argc > 7) { err_reply(cmd, PSTR("args")); return; }
      angles_t q; for (uint8_t j = 0; j < JG; j++) q.j[j] = a[j];
      q.j[JG] = 0;
      r = fsm_movej(&q, argc == 7 ? (uint32_t)a[6] : 0); is_move = true; break; }
    case C_JOG:
      if (argc < 3 || argc > 4) { err_reply(cmd, PSTR("args")); return; }
      r = fsm_jog(a[0], a[1], a[2], argc == 4 ? a[3] : 0); is_move = true; break;
    case C_JOGJ:                                               // j = 1..6, 7 = gripper
      if (argc != 2 || a[0] < 1 || a[0] > NJ || a[0] != (int)a[0]) { err_reply(cmd, PSTR("args")); return; }
      r = fsm_jogj((uint8_t)a[0] - 1, a[1]); is_move = true; break;
    case C_GRIP:
      if (argc != 1) { err_reply(cmd, PSTR("args")); return; }
      r = fsm_grip(a[0]); is_move = true; break;
    case C_SAVE: case C_GO: {
      if (argc != 1) { err_reply(cmd, PSTR("args")); return; }
      upcase(tok[1]);
      int8_t id = flash_find(PSTR(POSE_NAMES), POSE_COUNT, tok[1]);
      if (id < 0) { err_reply(cmd, PSTR("pose")); return; }
      r = c == C_SAVE ? fsm_save((pose_id_t)id) : fsm_go((pose_id_t)id);
      is_move = c == C_GO; break; }
    case C_CAL: {
      if (argc != 1) { err_reply(cmd, PSTR("args")); return; }
      upcase(tok[1]);
      int8_t col = flash_find(PSTR(CAL_NAMES), 3, tok[1]);     // index = colour_t
      if (col < 0) { err_reply(cmd, PSTR("args")); return; }
      r = fsm_calibrate((colour_t)col); is_move = true; break; }
    case C_SET: {
      float v;
      if (argc != 2 || !num(tok[2], &v)) { err_reply(cmd, PSTR("args")); return; }
      r = fsm_can_tune();
      if (r == R_OK && !decide_set(tok[1], v)) { err_reply(cmd, PSTR("key")); return; }
      break; }
  }
  if (r != R_OK) { err_reply(cmd, flash_nth(PSTR(REPLY_REASONS), r)); return; }
  ok(cmd); o_send();
  if (is_move) pending_done = c;
}

static void telemetry(uint32_t now) {
  pose_t p; task_where(&p);
  on = 0; o_p(PSTR("T,")); o_i((int32_t)now); o_c(','); o_p(flash_nth(PSTR(STATE_NAMES), fsm_state()));
  for (uint8_t j = 0; j < NJ; j++) { o_c(','); o_i(r10(joint_angle(j))); }
  o_c(','); o_i(rmm(p.x)); o_c(','); o_i(rmm(p.y)); o_c(','); o_i(rmm(p.z));
  o_c(','); o_i(ir_object()); o_c(','); o_i(fsm_last_colour()); o_c(','); o_i(fsm_error()); o_c(',');
  for (uint8_t i = 0; i < N_STEPPERS; i++) o_c(limit_pressed(i) ? '1' : '0');
  o_c(','); o_i(estop_active());
  o_send();
}

void console_update(uint32_t now) {
  char *line = serial_link_poll();
  if (line) handle(line, now);
  if (pending_done >= 0 && !fsm_busy()) {                      // the move finished (or was stopped)
    state_t s = fsm_state();
    if (s != ST_ERROR && s != ST_ESTOP) { on = 0; o_p(PSTR("DONE ")); o_p(flash_nth(PSTR(COMMANDS), pending_done)); o_send(); }
    pending_done = -1;
  }
  if (now - t_tel >= TELEMETRY_MS) { t_tel = now; telemetry(now); }
}
