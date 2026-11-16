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

#define COMMANDS "PING\0STATUS\0HOME\0RUN\0STOP\0RESET\0TEACH\0DONE\0MOVE\0LINE\0MOVEJ\0JOG\0JOGJ\0GRIP\0SAVE\0GO\0POSES\0CAL\0SET\0ST"
enum { C_PING, C_STATUS, C_HOME, C_RUN, C_STOP, C_RESET, C_TEACH, C_DONE, C_MOVE, C_LINE, C_MOVEJ,
       C_JOG, C_JOGJ, C_GRIP, C_SAVE, C_GO, C_POSES, C_CAL, C_SET, C_ST, C_COUNT };
#define ST_SUBS "LIST\0ADD\0SET\0DEL\0MAP\0GO\0COMMIT\0REVERT\0FACTORY"
enum { S_LIST, S_ADD, S_SET, S_DEL, S_MAP, S_GO, S_COMMIT, S_REVERT, S_FACTORY, S_COUNT };
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

static void o_t(int16_t tenths) { o_c(','); o_i((tenths + (tenths < 0 ? -5 : 5)) / 10); }   // 0.1 → integer

// ST LIST: "ST,<i>,<name>,<type>,<x>,<y>,<z>,<roll>,<above_dz>" per station, "MAP,<colour>,<name|->" per colour
static void st_list(void) {
  for (uint8_t i = 0; i < stations_count(); i++) {
    const station_t *s = stations_get(i);
    on = 0; o_p(PSTR("ST,")); o_i(i); o_c(','); o_s(s->name); o_c(','); o_p(flash_nth(PSTR(STATION_TYPES), s->type));
    o_t(s->x); o_t(s->y); o_t(s->z); o_t(s->roll); o_c(','); o_i(s->above_dz); o_send();
  }
  for (uint8_t c = COL_RED; c < N_COLOURS; c++) {
    int8_t b = stations_bin_for((colour_t)c);
    on = 0; o_p(PSTR("MAP,")); o_p(flash_nth(PSTR(COLOUR_NAMES), c)); o_c(',');
    if (b < 0) o_c('-'); else o_s(stations_get((uint8_t)b)->name);
    o_send();
  }
}

// ST <sub> …: layout edits (TEACH only, RAM until ST COMMIT). Returns a station reason (0 = OK),
// or 0xFF = bad arguments. *r = FSM reply for ST GO (a move).
#define ST_ARGS 0xFF
static uint8_t st_command(char **tok, uint8_t argc, fsm_reply_t *r) {
  for (uint8_t k = 1; k <= argc; k++) upcase(tok[k]);
  int8_t sc = flash_find(PSTR(ST_SUBS), S_COUNT, tok[1]);
  if (sc < 0) return ST_ARGS;
  if (sc == S_LIST) { st_list(); return STE_OK; }
  if (!fsm_teaching()) return STE_NOT_TEACH;
  int8_t i = argc >= 2 ? stations_find(tok[2]) : -1;
  switch (sc) {
    case S_ADD: {                                              // ST ADD <name> <type>: here
      int8_t t = argc == 3 ? flash_find(PSTR(STATION_TYPES), 3, tok[3]) : -1;
      if (t < 0) return ST_ARGS;
      pose_t p; task_where(&p);
      return stations_add(tok[2], (uint8_t)t, &p, STATION_ABOVE_DZ); }
    case S_SET: {                                              // ST SET <name> x y z roll [above_dz [type]]
      float v[5];
      if (argc < 6 || argc > 8) return ST_ARGS;
      for (uint8_t k = 0; k < 5 && k + 3 <= argc; k++) if (!num(tok[k + 3], &v[k])) return ST_ARGS;
      pose_t p = {v[0], v[1], v[2], v[3]};
      int16_t dz = argc >= 7 ? (int16_t)v[4] : i >= 0 ? stations_get((uint8_t)i)->above_dz : STATION_ABOVE_DZ;
      if (i >= 0) return stations_set((uint8_t)i, &p, dz);      // move (type unchanged)
      int8_t t = argc == 8 ? flash_find(PSTR(STATION_TYPES), 3, tok[8]) : STN_BIN;
      if (t < 0) return ST_ARGS;
      return stations_add(tok[2], (uint8_t)t, &p, dz); }        // create
    case S_DEL:
      if (argc != 2) return ST_ARGS;
      return i < 0 ? STE_NAME : stations_delete((uint8_t)i);
    case S_MAP: {                                              // ST MAP <RED|BLUE|UNKNOWN> <name|->
      if (argc != 3) return ST_ARGS;
      int8_t c = flash_find(PSTR(COLOUR_NAMES), N_COLOURS, tok[2]);
      int8_t b = strcmp(tok[3], "-") ? stations_find(tok[3]) : -1;
      if (c <= COL_EMPTY || (b < 0 && strcmp(tok[3], "-"))) return STE_NAME;
      return stations_map((colour_t)c, b); }
    case S_GO:
      if (argc != 2) return ST_ARGS;
      if (i < 0) return STE_NAME;
      *r = fsm_preview((uint8_t)i);
      return STE_OK;
    case S_COMMIT:  stations_commit();  return STE_OK;
    case S_REVERT:  stations_revert();  return STE_OK;
    case S_FACTORY: stations_factory(); return STE_OK;         // still needs ST COMMIT
  }
  return ST_ARGS;
}

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
      for (uint8_t i = 0; i < stations_count(); i++) {
        pose_t p = stations_grip(i);
        on = 0; o_p(PSTR("POSE ")); o_s(stations_get(i)->name);
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
      int8_t id = stations_find(tok[1]);
      if (id < 0) { err_reply(cmd, PSTR("pose")); return; }
      r = c == C_SAVE ? fsm_save((uint8_t)id) : fsm_go((uint8_t)id);
      is_move = c == C_GO; break; }
    case C_CAL: {
      if (argc != 1) { err_reply(cmd, PSTR("args")); return; }
      upcase(tok[1]);
      int8_t col = flash_find(PSTR(COLOUR_NAMES), 3, tok[1]);  // EMPTY RED BLUE, index = colour_t
      if (col < 0) { err_reply(cmd, PSTR("args")); return; }
      r = fsm_calibrate((colour_t)col); is_move = true; break; }
    case C_SET: {
      float v;
      if (argc != 2 || !num(tok[2], &v)) { err_reply(cmd, PSTR("args")); return; }
      r = fsm_can_tune();
      if (r == R_OK && !decide_set(tok[1], v)) { err_reply(cmd, PSTR("key")); return; }
      break; }
    case C_ST: {
      uint8_t e = argc ? st_command(tok, argc, &r) : ST_ARGS;
      if (e == ST_ARGS) { err_reply(cmd, PSTR("args")); return; }
      if (e) { err_reply(cmd, flash_nth(PSTR(ST_ERR_NAMES), e)); return; }
      is_move = !strcmp(tok[1], "GO");                         // st_command upcased it
      break; }
  }
  if (r != R_OK) { err_reply(cmd, r >= R_ST ? flash_nth(PSTR(ST_ERR_NAMES), r - R_ST) : flash_nth(PSTR(REPLY_REASONS), r)); return; }
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
  o_c(','); o_i(stations_layout_version());
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
