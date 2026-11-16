// test/test_arm/test_main.c — runs on the PC: pio test -e native
// Pure math tests + whole-robot scenarios on the fake hardware (hal_fake.c).
#include <unity.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "../../src/4_motion/kinematics.h"
#include "../../src/4_motion/motion.h"
#include "../../src/4_motion/stations.h"
#include "../../src/3_joints/joints.h"
#include "../../src/5_tasks/decisions.h"
#include "../../src/6_app/fsm.h"
#include "../../src/6_app/console.h"
#include "../../src/2_devices/actuators.h"
#include "../../src/2_devices/storage.h"
#include "ik_vectors.h"

extern uint32_t fake_now; extern bool fake_power_ok, fake_ir_blocked; extern rgbc_t fake_rgbc;
extern bool fake_limit_broken[], fake_limit_dead[];
extern char fake_log[]; extern char fake_last_tel[];
void fake_reset(float, float, float); void fake_tick_1ms(void); void fake_type(const char *);
float fake_true_deg(int); void fake_clear_log(void); void fake_wipe_eeprom(void); uint8_t *fake_eeprom(void);

static const rgbc_t RED = {620, 180, 150, 950}, BLUE = {140, 260, 560, 900}, EMPTY = {60, 65, 70, 200};
static const float SPD_ALL[3] = {SPD_J1, SPD_J2, SPD_J3};

void setUp(void) { decide_reset(); }
void tearDown(void) {}

// ── helpers ──
static void tick(void) { fake_tick_1ms(); fsm_tick(fake_now); console_update(fake_now); }
static void run_ms(uint32_t ms) { while (ms--) tick(); }
static bool run_until(state_t until, uint32_t ms) {         // stop early when 'until' is reached
  for (uint32_t t = 0; t < ms; t++) { tick(); if (fsm_state() == until) return true; }
  return false;
}
static bool run_until_log(const char *needle, uint32_t ms) {
  for (uint32_t t = 0; t < ms; t++) { tick(); if (strstr(fake_log, needle)) return true; }
  return false;
}
static bool boot(float j1, float j2, float j3) {
  fake_reset(j1, j2, j3); fake_wipe_eeprom(); fake_rgbc = EMPTY;
  fsm_init();
  return run_until(ST_IDLE, 60000);
}
static const char *cmd(const char *line) {                  // type a command, let the console answer
  char b[96]; snprintf(b, sizeof b, "%s\n", line);
  fake_clear_log(); fake_type(b); run_ms(3);
  return fake_log;
}
static int count_replies(const char *log, const char *name) {   // lines "OK <name>…" / "ERR <name>…"
  int n = 0; char ok[32], er[32];
  snprintf(ok, sizeof ok, "OK %s", name); snprintf(er, sizeof er, "ERR %s", name);
  for (const char *l = log; *l; ) {
    const char *e = strchr(l, '\n'); size_t len = e ? (size_t)(e - l) : strlen(l);
    size_t lo = strlen(ok), le = strlen(er);
    if ((len >= lo && !strncmp(l, ok, lo) && (len == lo || l[lo] == ' ')) ||
        (len >= le && !strncmp(l, er, le) && (len == le || l[le] == ' '))) n++;
    l += len + (e ? 1 : 0);
  }
  return n;
}
static pose_t station_pose(const char *name) { int8_t i = stations_find(name); TEST_ASSERT_TRUE(i >= 0); return stations_grip((uint8_t)i); }
static void assert_counters_match_motors(void) {
  for (int i = 0; i < 3; i++) TEST_ASSERT_FLOAT_WITHIN(0.2f, fake_true_deg(i), stepper_pos(i) / SPD_ALL[i]);
}

// ════════════════════ kinematics ════════════════════
void test_ik_fk_roundtrip(void) {
  int solved = 0;
  for (float x = 120; x <= 260; x += 20)
    for (float z = 20; z <= 160; z += 20)
      for (float yaw = -80; yaw <= 80; yaw += 40) {
        pose_t p = {x * cosf(yaw * 0.0174533f), x * sinf(yaw * 0.0174533f), z, 10}, back; angles_t a = {{0}};
        if (ik_solve(&p, &a) == IK_UNREACHABLE) continue;
        fk_solve(&a, &back); solved++;
        TEST_ASSERT_FLOAT_WITHIN(0.2f, p.x, back.x);
        TEST_ASSERT_FLOAT_WITHIN(0.2f, p.y, back.y);
        TEST_ASSERT_FLOAT_WITHIN(0.2f, p.z, back.z);
        TEST_ASSERT_FLOAT_WITHIN(0.05f, p.roll, back.roll);
      }
  TEST_ASSERT_GREATER_THAN(100, solved);
}
void test_ik_gripper_vertical(void) {
  pose_t p = {200, 50, 60, 0}; angles_t a = {{0}};
  TEST_ASSERT_EQUAL(IK_OK, ik_solve(&p, &a));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 180.0f, a.j[J2] + a.j[J3] + a.j[J5]);   // tool points straight down
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, a.j[J4]);
}
void test_ik_rejects_unreachable(void) {
  pose_t far = {600, 0, 50, 0}, near = {5, 0, 300, 0}; angles_t a = {{0}};
  TEST_ASSERT_EQUAL(IK_UNREACHABLE, ik_solve(&far, &a));
  TEST_ASSERT_NOT_EQUAL(IK_OK, ik_solve(&near, &a));
  pose_t behind = {-200, -10, 50, 0};                       // J1 ≈ −177°: outside −100…100
  TEST_ASSERT_EQUAL(IK_LIMIT, ik_solve(&behind, &a));
}
void test_ik_stations_reachable(void) {
  pose_t st[] = {{220, 0, CELL_GRIP_Z, 0}, {0, 220, CELL_DROP_Z, 0}, {0, -220, CELL_DROP_Z, 0},
                 {CELL_HOME_X, 0, CELL_HOME_Z, 0}, {220, 0, SAFE_Z, 0}, {0, 220, SAFE_Z + PLACE_ABOVE_DZ, 0},
                 {0, -220, SAFE_Z + PLACE_ABOVE_DZ, 0}};
  angles_t a = {{0}};
  for (unsigned i = 0; i < sizeof st / sizeof st[0]; i++) TEST_ASSERT_EQUAL_MESSAGE(IK_OK, ik_solve(&st[i], &a), "station unreachable");
}
void test_ik_golden_vectors(void) {                         // same vectors the sim's TS IK is tested on
  for (unsigned i = 0; i < IK_N_VECTORS; i++) {
    const float *v = IK_VECTORS[i];
    pose_t p = {v[0], v[1], v[2], v[3]}; angles_t a = {{0}};
    ik_result_t r = ik_solve(&p, &a);
    TEST_ASSERT_EQUAL_MESSAGE((int)v[4], (int)r, "IK status differs from reference");
    if (r == IK_UNREACHABLE) continue;
    for (int j = 0; j < 6; j++) TEST_ASSERT_FLOAT_WITHIN(0.05f, v[5 + j], a.j[j]);
    if (r != IK_OK) continue;
    pose_t back; fk_solve(&a, &back);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, p.x, back.x); TEST_ASSERT_FLOAT_WITHIN(0.1f, p.y, back.y);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, p.z, back.z);
  }
}

// ════════════════════ joints ════════════════════
void test_joint_conversions(void) {
  TEST_ASSERT_EQUAL_INT32(800, joint_deg_to_steps(J1, 90));            // 8.89 steps/deg
  TEST_ASSERT_EQUAL_INT32(-1266, joint_deg_to_steps(J2, -10));         // 126.6 (v2: 5.18 × 88/16, 1/8)
  TEST_ASSERT_EQUAL_INT32(2889, joint_deg_to_steps(J3, 100));          // 28.9 (v2: 104/16, 1/8)
  TEST_ASSERT_EQUAL_UINT16(1500, joint_deg_to_us(J4, 0));              // centre
  TEST_ASSERT_EQUAL_UINT16(1833, joint_deg_to_us(J5, 45));             // 270° servo: 7.41 µs/deg
  TEST_ASSERT_EQUAL_UINT16(2000, joint_deg_to_us(J6, 45));             // 180° servo: 11.1 µs/deg
  TEST_ASSERT_EQUAL_UINT16(1000, joint_deg_to_us(JG, 0));              // gripper closed
  TEST_ASSERT_EQUAL_UINT16(2000, joint_deg_to_us(JG, 50));             // gripper fully open
  TEST_ASSERT_EQUAL_UINT16(joint_deg_to_us(J4, 90), joint_deg_to_us(J4, 170));   // clamped
  TEST_ASSERT_EQUAL_FLOAT(110.0f, joint_clamp(J2, 200));
  TEST_ASSERT_EQUAL_FLOAT(-20.0f, joint_clamp(J2, -50));
  angles_t ok = {{0, 30, 90, 0, 60, 0, 99}}, bad = ok;                 // grip 99 is clamped, not judged
  TEST_ASSERT_TRUE(joint_within_limits(&ok));
  bad.j[J3] = 10; TEST_ASSERT_FALSE(joint_within_limits(&bad));
}

// ════════════════════ decisions ════════════════════
void test_colour_decisions(void) {
  TEST_ASSERT_EQUAL(COL_RED,   decide_colour(&RED));
  TEST_ASSERT_EQUAL(COL_BLUE,  decide_colour(&BLUE));
  TEST_ASSERT_EQUAL(COL_EMPTY, decide_colour(&EMPTY));
  rgbc_t purple = {380, 210, 360, 920};                    // ambiguous: in the middle
  TEST_ASSERT_EQUAL(COL_UNKNOWN, decide_colour(&purple));  // no history yet → unknown
  decide_colour_confirmed(COL_RED, &purple);               // ...but the last cube looked exactly like this
  TEST_ASSERT_EQUAL(COL_RED, decide_colour(&purple));      // 2-colour trick: same → same colour
  rgbc_t purple2 = {300, 210, 440, 920};                   // ambiguous, far from the last red one
  TEST_ASSERT_EQUAL(COL_BLUE, decide_colour(&purple2));    // → must be the other colour
  TEST_ASSERT_TRUE(decide_set("use_last_trick", 0));
  TEST_ASSERT_EQUAL(COL_UNKNOWN, decide_colour(&purple));  // trick off
}
void test_colour_calibration(void) {
  rgbc_t dark_red = {300, 120, 110, 600};                  // different lighting, re-calibrated
  decide_colour_calibrate(COL_RED, &dark_red);
  TEST_ASSERT_EQUAL(COL_RED, decide_colour(&dark_red));
  rgbc_t bright_empty = {200, 200, 200, 700};
  decide_colour_calibrate(COL_EMPTY, &bright_empty);       // ambient brighter now
  TEST_ASSERT_EQUAL(COL_EMPTY, decide_colour(&bright_empty));
}
void test_jiggle_pattern(void) {
  float dx, dy, dr;
  decide_grip_offset(0, &dx, &dy, &dr); TEST_ASSERT_EQUAL_FLOAT(0, dx); TEST_ASSERT_EQUAL_FLOAT(0, dy); TEST_ASSERT_EQUAL_FLOAT(0, dr);
  decide_grip_offset(1, &dx, &dy, &dr); TEST_ASSERT_EQUAL_FLOAT(DEC_JIGGLE_MM, dx);  TEST_ASSERT_EQUAL_FLOAT(DEC_JIGGLE_DEG, dr);
  decide_grip_offset(2, &dx, &dy, &dr); TEST_ASSERT_EQUAL_FLOAT(-DEC_JIGGLE_MM, dx); TEST_ASSERT_EQUAL_FLOAT(-DEC_JIGGLE_DEG, dr);
  decide_grip_offset(3, &dx, &dy, &dr); TEST_ASSERT_EQUAL_FLOAT(DEC_JIGGLE_MM, dy);
  TEST_ASSERT_EQUAL_FLOAT(0, decide_grip_wiggle_deg(0));
  TEST_ASSERT_EQUAL_FLOAT(DEC_GRIP_WIGGLE_DEG, decide_grip_wiggle_deg(1));
  TEST_ASSERT_EQUAL(1 + PICK_RETRIES, decide_max_pick_attempts());
  TEST_ASSERT_TRUE(decide_pick_ok(false, COL_EMPTY));
  TEST_ASSERT_FALSE(decide_pick_ok(true, COL_EMPTY));      // beam still blocked
  TEST_ASSERT_FALSE(decide_pick_ok(false, COL_RED));       // cube still on the sensor
}
void test_rehome_rule_and_set(void) {
  TEST_ASSERT_FALSE(decide_rehome_now(0)); TEST_ASSERT_FALSE(decide_rehome_now(9));
  TEST_ASSERT_TRUE(decide_rehome_now(10)); TEST_ASSERT_TRUE(decide_rehome_now(20));
  TEST_ASSERT_TRUE(decide_set("rehome_every", 3));  TEST_ASSERT_TRUE(decide_rehome_now(3));
  TEST_ASSERT_TRUE(decide_set("rehome_every", 0));  TEST_ASSERT_FALSE(decide_rehome_now(10));
  TEST_ASSERT_FALSE(decide_set("no_such_key", 1));
}

// ════════════════════ storage ════════════════════
void test_eeprom_crc(void) {
  fake_wipe_eeprom();
  pose_t p[2] = {{1, 2, 3, 4}, {5, 6, 7, 8}}, q[2] = {{0}};
  TEST_ASSERT_FALSE(storage_load(EE_ADDR_STATIONS, EE_MAGIC_STATIONS, q, sizeof q));      // blank chip
  storage_save(EE_ADDR_STATIONS, EE_MAGIC_STATIONS, p, sizeof p);
  TEST_ASSERT_TRUE(storage_load(EE_ADDR_STATIONS, EE_MAGIC_STATIONS, q, sizeof q));
  TEST_ASSERT_EQUAL_FLOAT(7, q[1].z);
  fake_eeprom()[EE_ADDR_STATIONS + 5] ^= 0x40;                // flip one bit
  pose_t r[2] = {{9, 9, 9, 9}, {9, 9, 9, 9}};
  TEST_ASSERT_FALSE(storage_load(EE_ADDR_STATIONS, EE_MAGIC_STATIONS, r, sizeof r));
  TEST_ASSERT_EQUAL_FLOAT(9, r[0].x);                      // untouched → caller keeps CAD defaults
}

// ════════════════════ stations (workcell as data) ════════════════════
static st_err_t val(float x, float y, float z, float roll, uint8_t type) {
  pose_t p = {x, y, z, roll}; return stations_validate(&p, STATION_ABOVE_DZ, type, -1);
}
void test_stations_defaults_valid(void) {
  fake_wipe_eeprom(); stations_init();
  TEST_ASSERT_EQUAL(4, stations_count());
  TEST_ASSERT_FALSE(stations_dirty());
  TEST_ASSERT_EQUAL(0, stations_find("HOME")); TEST_ASSERT_EQUAL(1, stations_find_type(STN_PICK, 0));
  TEST_ASSERT_EQUAL(-1, stations_find_type(STN_PICK, 1));
  for (uint8_t i = 0; i < stations_count(); i++) {          // every CAD station passes with the v2 lengths
    pose_t p = stations_grip(i);
    TEST_ASSERT_EQUAL_MESSAGE(STE_OK, stations_validate(&p, stations_get(i)->above_dz, stations_get(i)->type, (int8_t)i), stations_get(i)->name);
  }
  TEST_ASSERT_EQUAL(stations_find("BIN_A"), stations_bin_for(COL_RED));
  TEST_ASSERT_EQUAL(stations_find("BIN_B"), stations_bin_for(COL_BLUE));
  TEST_ASSERT_EQUAL(-1, stations_bin_for(COL_UNKNOWN));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -90.0f, station_pose("BIN_B").roll);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, CELL_GRIP_Z + STATION_ABOVE_DZ > SAFE_Z ? CELL_GRIP_Z + STATION_ABOVE_DZ : SAFE_Z,
                           stations_above(1).z);            // approach never below SAFE_Z
}
void test_stations_validation_codes(void) {
  fake_wipe_eeprom(); stations_init();
  TEST_ASSERT_EQUAL(STE_OK,          val(150, 150, 80, 45, STN_BIN));   // a 3rd bin between A and the cradle
  TEST_ASSERT_EQUAL(STE_UNREACHABLE, val(600, 0, 50, 0, STN_PARK));
  TEST_ASSERT_EQUAL(STE_SINGULAR,    val(329.8f, 0, 10, 0, STN_PARK));   // arm almost straight: cos J3 > 0.995
  TEST_ASSERT_EQUAL(STE_LIMIT,       val(-200, -10, 50, 0, STN_PARK));   // J1 ≈ −177°
  TEST_ASSERT_EQUAL(STE_LIMIT,       val(-20, 220, 80, 95, STN_BIN));    // J1 95°: inside the joint, outside the working range
  TEST_ASSERT_EQUAL(STE_LIMIT,       val(200, 0, 80, 120, STN_BIN));     // J6 = roll − J1 = 120° > 90°
  TEST_ASSERT_EQUAL(STE_KEEPOUT,     val(220, 100, 5, 0, STN_BIN));      // through the table
  TEST_ASSERT_EQUAL(STE_KEEPOUT,     val(60, 0, 100, 0, STN_PARK));      // inside the base column
  TEST_ASSERT_EQUAL(STE_KEEPOUT,     val(0, 160, 50, 90, STN_BIN));      // drop below the bin wall
  TEST_ASSERT_EQUAL(STE_TOO_CLOSE,   val(220, 40, 80, 0, STN_BIN));      // on top of the cradle
  TEST_ASSERT_EQUAL(STE_OK,          val(220, 40, 80, 0, STN_PARK));     // park poses have no footprint
}
void test_stations_table_edits(void) {
  fake_wipe_eeprom(); stations_init();
  pose_t p = {150, 150, 80, 45}, q = {150, -150, 80, -45};
  TEST_ASSERT_EQUAL(STE_OK, stations_add("REJECT", STN_BIN, &p, STATION_ABOVE_DZ));
  TEST_ASSERT_TRUE(stations_dirty());
  TEST_ASSERT_EQUAL(4, stations_find("REJECT"));
  TEST_ASSERT_EQUAL(STE_NAME, stations_add("REJECT", STN_BIN, &q, STATION_ABOVE_DZ));    // duplicate
  TEST_ASSERT_EQUAL(STE_NAME, stations_add("bad name", STN_BIN, &q, STATION_ABOVE_DZ));
  TEST_ASSERT_EQUAL(STE_NAME, stations_add("TOOLONGNAME", STN_BIN, &q, STATION_ABOVE_DZ));
  TEST_ASSERT_EQUAL(STE_FULL, stations_add("CUBE2", STN_PICK, &q, STATION_ABOVE_DZ));     // one pick station
  TEST_ASSERT_EQUAL(STE_TOO_CLOSE, stations_add("NEAR", STN_BIN, &p, STATION_ABOVE_DZ));  // on REJECT
  TEST_ASSERT_EQUAL(5, stations_count());                                                 // refused = not added
  TEST_ASSERT_EQUAL(STE_OK, stations_set(4, &q, 40));                                     // move it
  TEST_ASSERT_FLOAT_WITHIN(0.05f, -150, stations_grip(4).y);
  TEST_ASSERT_EQUAL(40, stations_get(4)->above_dz);
  pose_t far = {600, 0, 80, 0};
  TEST_ASSERT_EQUAL(STE_UNREACHABLE, stations_set(4, &far, 40));
  TEST_ASSERT_FLOAT_WITHIN(0.05f, -150, stations_grip(4).y);                              // refused = unchanged
  TEST_ASSERT_EQUAL(STE_PROTECTED, stations_delete(0));                                   // HOME
  TEST_ASSERT_EQUAL(STE_PROTECTED, stations_delete(1));                                   // the pick station
  TEST_ASSERT_EQUAL(STE_OK, stations_map(COL_UNKNOWN, 4));
  TEST_ASSERT_EQUAL(STE_NAME, stations_map(COL_RED, 0));                                  // HOME is not a bin
  TEST_ASSERT_EQUAL(STE_OK, stations_delete(2));                                          // BIN_A: map follows
  TEST_ASSERT_EQUAL(-1, stations_bin_for(COL_RED));
  TEST_ASSERT_EQUAL(2, stations_bin_for(COL_BLUE));                                       // BIN_B moved 3 → 2
  TEST_ASSERT_EQUAL(3, stations_bin_for(COL_UNKNOWN));                                    // REJECT moved 4 → 3
  static const pose_t more[] = {{0, 220, 78, 90}, {150, 150, 80, 45}, {230, 90, 80, 21}, {230, -90, 80, -21}};
  static const char *const names[] = {"B1", "B2", "B3", "B4"};
  for (int k = 0; k < 4; k++) TEST_ASSERT_EQUAL_MESSAGE(STE_OK, stations_add(names[k], STN_BIN, &more[k], STATION_ABOVE_DZ), names[k]);
  TEST_ASSERT_EQUAL(MAX_STATIONS, stations_count());
  pose_t b = {-60, 200, 80, 90};
  TEST_ASSERT_EQUAL(STE_FULL, stations_add("MORE", STN_BIN, &b, STATION_ABOVE_DZ));
}
void test_stations_eeprom(void) {
  fake_wipe_eeprom(); stations_init();
  pose_t p = {150, 150, 80, 45};
  TEST_ASSERT_EQUAL(STE_OK, stations_add("REJECT", STN_BIN, &p, 50));
  TEST_ASSERT_EQUAL(STE_OK, stations_map(COL_UNKNOWN, 4));
  stations_revert();                                        // not committed → gone
  TEST_ASSERT_EQUAL(4, stations_count());
  TEST_ASSERT_EQUAL(STE_OK, stations_add("REJECT", STN_BIN, &p, 50));
  TEST_ASSERT_EQUAL(STE_OK, stations_map(COL_UNKNOWN, 4));
  uint8_t v = stations_layout_version();
  stations_commit();
  TEST_ASSERT_EQUAL(v + 1, stations_layout_version());
  stations_factory(); stations_init();                      // "reboot"
  TEST_ASSERT_EQUAL(5, stations_count());
  TEST_ASSERT_EQUAL(4, stations_bin_for(COL_UNKNOWN));
  TEST_ASSERT_EQUAL(50, stations_get(4)->above_dz);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 150, stations_grip(4).y);
  TEST_ASSERT_EQUAL(v + 1, stations_layout_version());
  uint8_t buf[256]; uint16_t len = 1;                       // old record version (valid CRC) → CAD defaults
  while (len < sizeof buf && !storage_load(EE_ADDR_STATIONS, EE_MAGIC_STATIONS, buf, len)) len++;   // = record size
  TEST_ASSERT_LESS_THAN(sizeof buf, len);
  buf[0] = STATIONS_REC_VERSION + 1;
  storage_save(EE_ADDR_STATIONS, EE_MAGIC_STATIONS, buf, len);
  stations_init();
  TEST_ASSERT_EQUAL(4, stations_count()); TEST_ASSERT_EQUAL(-1, stations_find("REJECT"));
  stations_add("REJECT", STN_BIN, &p, 50); stations_commit();
  fake_eeprom()[EE_ADDR_STATIONS + 9] ^= 0x10;              // corrupt CRC → CAD defaults
  stations_init();
  TEST_ASSERT_EQUAL(4, stations_count()); TEST_ASSERT_FALSE(stations_dirty());
}

// ════════════════════ homing ════════════════════
void test_homing_unknown_start(void) {
  TEST_ASSERT_TRUE_MESSAGE(boot(20, 40, 90), "homing never finished");
  assert_counters_match_motors();
  TEST_ASSERT_TRUE(fsm_homed());
  pose_t p; motion_where(&p);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, CELL_HOME_X, p.x); TEST_ASSERT_FLOAT_WITHIN(1.0f, CELL_HOME_Z, p.z);
}
void test_homing_start_on_switch(void) {
  TEST_ASSERT_TRUE_MESSAGE(boot(-101, -21, 151), "homing from on-switch never finished");   // all 3 pressed
  assert_counters_match_motors();
}
void test_homing_broken_wire(void) {
  fake_reset(20, 40, 90); fake_wipe_eeprom(); fake_limit_broken[J2] = true;
  float start = fake_true_deg(J2);
  fake_clear_log(); fsm_init();
  float lowest = start;                                    // J2's switch is toward −
  for (int t = 0; t < 20000 && fsm_state() != ST_ERROR; t++) { tick(); if (fake_true_deg(J2) < lowest) lowest = fake_true_deg(J2); }
  TEST_ASSERT_EQUAL(ST_ERROR, fsm_state());
  TEST_ASSERT_EQUAL(E_HOME_FAILED, fsm_error());
  TEST_ASSERT_FLOAT_WITHIN(0.01f, start, lowest);          // never drove toward the hard stop
  TEST_ASSERT_LESS_OR_EQUAL_FLOAT(start + HOMING_LEAVE_MAX + 1, fake_true_deg(J2));
  TEST_ASSERT_NOT_NULL(strstr(fake_log, "EV HOME_FAIL broken_wire 2"));
}
void test_homing_switch_never_found(void) {
  fake_reset(20, 40, 90); fake_wipe_eeprom(); fake_limit_dead[J1] = true;
  fake_clear_log(); fsm_init();
  TEST_ASSERT_TRUE(run_until(ST_ERROR, 30000));
  TEST_ASSERT_EQUAL(E_HOME_FAILED, fsm_error());
  TEST_ASSERT_NOT_NULL(strstr(fake_log, "EV HOME_FAIL not_found 1"));
  TEST_ASSERT_FALSE(fsm_homed());
}

// ════════════════════ whole robot on fake hardware ════════════════════
void test_cycle_with_one_failed_pick(void) {
  TEST_ASSERT_TRUE(boot(20, 40, 90));
  fake_rgbc = RED; fake_ir_blocked = true;                 // a red cube arrives
  fake_clear_log();
  fake_type("RUN\n");
  TEST_ASSERT_TRUE(run_until(ST_PICK, 5000));
  TEST_ASSERT_TRUE(run_until(ST_VERIFY, 20000));
  TEST_ASSERT_TRUE_MESSAGE(run_until(ST_PICK, 2000), "should retry the pick");   // cube still there
  TEST_ASSERT_TRUE(run_until(ST_VERIFY, 20000));
  fake_ir_blocked = false; fake_rgbc = EMPTY;              // this time the cube left the cradle
  TEST_ASSERT_TRUE(run_until(ST_PLACE, 2000));
  TEST_ASSERT_TRUE(run_until(ST_IDLE, 30000));
  TEST_ASSERT_EQUAL(1, fsm_cycles());
  TEST_ASSERT_NOT_NULL(strstr(fake_log, "OK RUN"));
  TEST_ASSERT_NOT_NULL(strstr(fake_log, "EV COLOUR RED"));
  TEST_ASSERT_NOT_NULL(strstr(fake_log, "EV PICK_RETRY 1"));
  TEST_ASSERT_NOT_NULL(strstr(fake_log, "EV PLACED RED"));
  TEST_ASSERT_NOT_NULL(strstr(fake_log, "EV CYCLE 1"));
  pose_t p; motion_where(&p);                              // back home
  TEST_ASSERT_FLOAT_WITHIN(1.0f, CELL_HOME_X, p.x);
  cmd("STOP");
  TEST_ASSERT_FALSE(fsm_running());
}
void test_pick_fails_three_times_is_error(void) {
  TEST_ASSERT_TRUE(boot(0, 30, 90));
  fake_rgbc = BLUE; fake_ir_blocked = true;                // cube never leaves
  cmd("RUN");
  TEST_ASSERT_TRUE(run_until(ST_ERROR, 90000));
  TEST_ASSERT_EQUAL(E_PICK_FAILED, fsm_error());
  TEST_ASSERT_FALSE(fsm_running());
}
void test_unknown_colour_is_error(void) {
  TEST_ASSERT_TRUE(boot(0, 30, 90));
  rgbc_t purple = {380, 210, 360, 920};
  fake_rgbc = purple; fake_ir_blocked = true;
  cmd("RUN");
  TEST_ASSERT_TRUE(run_until(ST_ERROR, 5000));
  TEST_ASSERT_EQUAL(E_COLOUR_UNKNOWN, fsm_error());
}
void test_teach_over_serial(void) {
  TEST_ASSERT_TRUE(boot(20, 40, 90));
  TEST_ASSERT_NOT_NULL(strstr(cmd("MOVE 200 0 70"), "OK MOVE"));   // near the table: a pose CUBE can really have
  TEST_ASSERT_TRUE(run_until_log("DONE MOVE", 5000));              // (its approach, 60 mm higher, must be reachable)
  TEST_ASSERT_EQUAL(1, count_replies(cmd("TEACH"), "TEACH"));
  TEST_ASSERT_EQUAL(ST_TEACH, fsm_state());
  pose_t before; motion_where(&before);
  TEST_ASSERT_NOT_NULL(strstr(cmd("JOG 0 0 -10"), "OK JOG"));
  TEST_ASSERT_TRUE(run_until_log("DONE JOG", 3000));
  TEST_ASSERT_NOT_NULL(strstr(cmd("JOG 0 0 -10"), "OK JOG"));
  TEST_ASSERT_NOT_NULL(strstr(cmd("JOG 0 0 1"), "ERR JOG busy"));   // one move at a time
  TEST_ASSERT_TRUE(run_until_log("DONE JOG", 3000));
  TEST_ASSERT_NOT_NULL(strstr(cmd("SAVE CUBE"), "OK SAVE"));
  TEST_ASSERT_FLOAT_WITHIN(1.0f, before.z - 20, station_pose("CUBE").z);
  TEST_ASSERT_NOT_NULL(strstr(cmd("JOGJ 1 15"), "OK JOGJ"));
  TEST_ASSERT_TRUE(run_until_log("DONE JOGJ", 3000));
  TEST_ASSERT_FLOAT_WITHIN(0.2f, 15.0f, joint_angle(J1));
  TEST_ASSERT_NOT_NULL(strstr(cmd("GO CUBE"), "OK GO"));
  TEST_ASSERT_TRUE(run_until_log("DONE GO", 5000));
  pose_t at; motion_where(&at);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, before.z - 20, at.z);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, before.y, at.y);
  const char *poses = cmd("POSES");
  TEST_ASSERT_NOT_NULL(strstr(poses, "POSE HOME 160.0 0.0 170.0 0.0"));
  TEST_ASSERT_NOT_NULL(strstr(poses, "POSE CUBE "));
  TEST_ASSERT_NOT_NULL(strstr(poses, "OK POSES"));
  fake_rgbc = RED;                                         // colour calibration with a cube on the sensor
  TEST_ASSERT_NOT_NULL(strstr(cmd("CAL RED"), "OK CAL"));
  TEST_ASSERT_TRUE(run_until_log("DONE CAL", 1000));
  TEST_ASSERT_NOT_NULL(strstr(fake_log, "EV CAL RED"));
  TEST_ASSERT_EQUAL(1, count_replies(cmd("DONE"), "DONE"));
  TEST_ASSERT_EQUAL(ST_IDLE, fsm_state());
  // taught pose + calibration survive a reboot (EEPROM, CRC)
  float z_cube = station_pose("CUBE").z;
  fake_reset(0, 30, 90); fsm_init();
  TEST_ASSERT_FLOAT_WITHIN(0.01f, z_cube, station_pose("CUBE").z);
}
void test_move_replies(void) {
  TEST_ASSERT_TRUE(boot(20, 40, 90));
  const char *r = cmd("MOVE 200 30 100");
  TEST_ASSERT_EQUAL(1, count_replies(r, "MOVE")); TEST_ASSERT_NOT_NULL(strstr(r, "OK MOVE"));
  TEST_ASSERT_TRUE(run_until_log("DONE MOVE", 5000));
  pose_t p; motion_where(&p);
  TEST_ASSERT_FLOAT_WITHIN(0.6f, 200, p.x); TEST_ASSERT_FLOAT_WITHIN(0.6f, 30, p.y); TEST_ASSERT_FLOAT_WITHIN(0.6f, 100, p.z);
  TEST_ASSERT_NOT_NULL(strstr(cmd("MOVE 600 0 0"), "ERR MOVE unreachable"));
  TEST_ASSERT_NOT_NULL(strstr(cmd("MOVE 200 x 0"), "ERR MOVE args"));
  TEST_ASSERT_NOT_NULL(strstr(cmd("MOVE 200"), "ERR MOVE args"));
  TEST_ASSERT_NOT_NULL(strstr(cmd("LINE 200 30 60 40"), "OK LINE"));
  TEST_ASSERT_TRUE(run_until_log("DONE LINE", 5000));
  motion_where(&p); TEST_ASSERT_FLOAT_WITHIN(0.6f, 60, p.z);
  TEST_ASSERT_NOT_NULL(strstr(cmd("MOVEJ 0 30 90 0 60 0"), "OK MOVEJ"));
  TEST_ASSERT_TRUE(run_until_log("DONE MOVEJ", 5000));
  TEST_ASSERT_FLOAT_WITHIN(0.2f, 30, joint_angle(J2));
  TEST_ASSERT_NOT_NULL(strstr(cmd("MOVEJ 0 30 5 0 60 0"), "ERR MOVEJ limit"));    // J3 min is 20
  TEST_ASSERT_NOT_NULL(strstr(cmd("GRIP 30"), "OK GRIP"));
  TEST_ASSERT_TRUE(run_until_log("DONE GRIP", 2000));
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 30, joint_angle(JG));
  TEST_ASSERT_NOT_NULL(strstr(cmd("JOG 0 0 1"), "ERR JOG state"));                // TEACH only
  cmd("RUN");
  TEST_ASSERT_NOT_NULL(strstr(cmd("MOVE 200 0 100"), "ERR MOVE running"));
  cmd("STOP");
}
void test_estop_from_pick(void) {
  TEST_ASSERT_TRUE(boot(20, 40, 90));
  fake_rgbc = RED; fake_ir_blocked = true;
  cmd("RUN");
  TEST_ASSERT_TRUE(run_until(ST_PICK, 5000));
  run_ms(300);
  fake_power_ok = false;                                   // E-stop pressed: motor rail cut
  TEST_ASSERT_TRUE(run_until(ST_ESTOP, 50));
  TEST_ASSERT_FALSE(fsm_homed());
  TEST_ASSERT_FALSE(fsm_running());
  TEST_ASSERT_NOT_NULL(strstr(cmd("RESET"), "ERR RESET estop"));   // power still off
  TEST_ASSERT_NOT_NULL(strstr(cmd("MOVE 200 0 100"), "ERR MOVE state"));
  fake_power_ok = true; run_ms(100);
  TEST_ASSERT_EQUAL(ST_ESTOP, fsm_state());                // stays until RESET
  TEST_ASSERT_NOT_NULL(strstr(cmd("RESET"), "OK RESET"));
  TEST_ASSERT_EQUAL(ST_HOMING, fsm_state());
  fake_ir_blocked = false;
  TEST_ASSERT_TRUE(run_until(ST_IDLE, 60000));
  assert_counters_match_motors();
}

// ════════════════════ configurable workcell over serial ════════════════════
static int tel_field(int k) {                              // k-th field of the latest telemetry line
  char line[128]; strcpy(line, fake_last_tel);
  char *t = strtok(line, ",\n"); for (int i = 0; i < k && t; i++) t = strtok(0, ",\n");
  return t ? atoi(t) : -999;
}
void test_st_protocol(void) {
  TEST_ASSERT_TRUE(boot(20, 40, 90));
  const char *l = cmd("ST LIST");                          // allowed in any state
  TEST_ASSERT_NOT_NULL(strstr(l, "ST,0,HOME,PARK,160,0,170,0,60\n"));
  TEST_ASSERT_NOT_NULL(strstr(l, "ST,1,CUBE,PICK,220,0,24,0,60\n"));
  TEST_ASSERT_NOT_NULL(strstr(l, "ST,3,BIN_B,BIN,0,-220,78,-90,60\n"));
  TEST_ASSERT_NOT_NULL(strstr(l, "MAP,RED,BIN_A\nMAP,BLUE,BIN_B\nMAP,UNKNOWN,-\nOK ST"));
  TEST_ASSERT_NOT_NULL(strstr(cmd("ST SET REJECT 150 150 80 45"), "ERR ST NOT_TEACH"));
  TEST_ASSERT_NOT_NULL(strstr(cmd("ST"), "ERR ST args"));
  cmd("TEACH");
  TEST_ASSERT_NOT_NULL(strstr(cmd("ST SET REJECT 150 150 80 45"), "OK ST"));       // create (BIN)
  TEST_ASSERT_NOT_NULL(strstr(cmd("ST SET REJECT 150 150 85 45 50"), "OK ST"));    // move + above_dz
  TEST_ASSERT_NOT_NULL(strstr(cmd("ST LIST"), "ST,4,REJECT,BIN,150,150,85,45,50"));
  TEST_ASSERT_NOT_NULL(strstr(cmd("ST SET FAR 600 0 80 0"), "ERR ST UNREACHABLE"));
  TEST_ASSERT_NOT_NULL(strstr(cmd("ST SET P2 250 0 150 0 60 PARK"), "OK ST"));
  TEST_ASSERT_NOT_NULL(strstr(cmd("ST SET X 1 2 3 4 5 ROBOT"), "ERR ST args"));
  TEST_ASSERT_NOT_NULL(strstr(cmd("ST SET X 1 two 3 4"), "ERR ST args"));
  TEST_ASSERT_NOT_NULL(strstr(cmd("ST DEL HOME"), "ERR ST PROTECTED"));
  TEST_ASSERT_NOT_NULL(strstr(cmd("ST DEL CUBE"), "ERR ST PROTECTED"));
  TEST_ASSERT_NOT_NULL(strstr(cmd("ST DEL P2"), "OK ST"));
  TEST_ASSERT_NOT_NULL(strstr(cmd("ST DEL P2"), "ERR ST NAME"));
  TEST_ASSERT_NOT_NULL(strstr(cmd("st map unknown reject"), "OK ST"));             // case-insensitive
  TEST_ASSERT_NOT_NULL(strstr(cmd("ST MAP GREEN REJECT"), "ERR ST NAME"));
  TEST_ASSERT_NOT_NULL(strstr(cmd("ST MAP RED HOME"), "ERR ST NAME"));             // not a bin
  TEST_ASSERT_NOT_NULL(strstr(cmd("ST GO REJECT"), "OK ST"));                       // preview: above, then down
  TEST_ASSERT_TRUE(run_until_log("DONE ST", 15000));
  pose_t p; motion_where(&p);
  TEST_ASSERT_FLOAT_WITHIN(0.6f, 150, p.x); TEST_ASSERT_FLOAT_WITHIN(0.6f, 150, p.y); TEST_ASSERT_FLOAT_WITHIN(0.6f, 85, p.z);
  TEST_ASSERT_NOT_NULL(strstr(cmd("ST ADD HERE BIN"), "ERR ST TOO_CLOSE"));        // teach on the arm: right on REJECT
  TEST_ASSERT_NOT_NULL(strstr(cmd("ST ADD HERE PARK"), "OK ST"));
  TEST_ASSERT_NOT_NULL(strstr(cmd("DONE"), "EV WARN uncommitted_layout"));
  cmd("TEACH");
  TEST_ASSERT_NOT_NULL(strstr(cmd("ST COMMIT"), "OK ST"));
  run_ms(60); TEST_ASSERT_EQUAL(1, tel_field(18));         // the sim sees the new layout version
  TEST_ASSERT_NULL(strstr(cmd("DONE"), "WARN"));
  cmd("TEACH");
  cmd("ST FACTORY");
  TEST_ASSERT_NULL(strstr(cmd("ST LIST"), "REJECT"));
  cmd("ST REVERT");                                        // back to the committed layout
  TEST_ASSERT_NOT_NULL(strstr(cmd("ST LIST"), "MAP,UNKNOWN,REJECT"));
  fake_reset(0, 30, 90); fsm_init();                       // survives a reboot
  TEST_ASSERT_EQUAL(6, stations_count());
}
static bool run_until_return(float *y) {                   // run a cycle; tip y when the drop finished
  if (!run_until(ST_VERIFY, 30000)) return false;
  fake_ir_blocked = false; fake_rgbc = EMPTY;              // the cube left the cradle
  if (!run_until(ST_RETURN, 30000)) return false;
  pose_t p; motion_where(&p); *y = p.y;
  return run_until(ST_IDLE, 30000);
}
void test_swapped_bins(void) {
  TEST_ASSERT_TRUE(boot(20, 40, 90));
  cmd("TEACH");
  TEST_ASSERT_NOT_NULL(strstr(cmd("ST MAP RED BIN_B"), "OK ST"));
  TEST_ASSERT_NOT_NULL(strstr(cmd("ST MAP BLUE BIN_A"), "OK ST"));
  cmd("ST COMMIT"); cmd("DONE");
  fake_rgbc = RED; fake_ir_blocked = true; fake_clear_log();
  cmd("RUN");
  float y = 0; TEST_ASSERT_TRUE(run_until_return(&y));
  TEST_ASSERT_LESS_THAN_FLOAT(-200.0f, y);                 // red went to BIN_B (−y)
  TEST_ASSERT_NOT_NULL(strstr(fake_log, "EV PLACED RED"));
  cmd("STOP");
}
void test_reject_bin(void) {
  TEST_ASSERT_TRUE(boot(20, 40, 90));
  cmd("TEACH");
  TEST_ASSERT_NOT_NULL(strstr(cmd("ST SET REJECT 150 150 80 45"), "OK ST"));
  TEST_ASSERT_NOT_NULL(strstr(cmd("ST MAP UNKNOWN REJECT"), "OK ST"));
  cmd("ST COMMIT"); cmd("DONE");
  rgbc_t purple = {380, 210, 360, 920};                    // neither red nor blue
  fake_rgbc = purple; fake_ir_blocked = true; fake_clear_log();
  cmd("RUN");
  float y = 0; TEST_ASSERT_TRUE(run_until_return(&y));
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 150, y);                  // dropped into REJECT
  TEST_ASSERT_NOT_NULL(strstr(fake_log, "EV COLOUR UNKNOWN"));
  TEST_ASSERT_NOT_NULL(strstr(fake_log, "EV PLACED UNKNOWN"));
  TEST_ASSERT_EQUAL(E_NONE, fsm_error());
  cmd("STOP");
}

// ════════════════════ protocol ════════════════════
void test_every_command_one_reply(void) {
  TEST_ASSERT_TRUE(boot(0, 30, 90));
  static const char *const lines[][2] = {                 // {line, name in the reply}
    {"PING", "PING"}, {"STATUS", "STATUS"}, {"POSES", "POSES"}, {"FOO 1 2", "FOO"}, {"ping", "PING"},
    {"MOVE 200 0 100", "MOVE"}, {"MOVE 200 0 100", "MOVE"} /* busy */, {"STOP", "STOP"},
    {"LINE 200 0 90", "LINE"}, {"STOP", "STOP"}, {"MOVEJ 0 30 90 0 60 0 500", "MOVEJ"}, {"STOP", "STOP"},
    {"MOVEJ 1 2", "MOVEJ"}, {"GRIP 40", "GRIP"}, {"STOP", "STOP"}, {"GRIP", "GRIP"},
    {"JOG 1 0 0", "JOG"}, {"JOGJ 1 5", "JOGJ"}, {"SAVE CUBE", "SAVE"}, {"GO CUBE", "GO"}, {"CAL RED", "CAL"},
    {"SET grip_wiggle 4", "SET"}, {"SET nope 1", "SET"}, {"SET grip_wiggle", "SET"},
    {"DONE", "DONE"}, {"RESET", "RESET"}, {"TEACH", "TEACH"},
    {"JOG 1 0 0", "JOG"}, {"STOP", "STOP"}, {"JOGJ 9 5", "JOGJ"}, {"SAVE NOWHERE", "SAVE"}, {"CAL GREEN", "CAL"},
    {"SAVE BIN_A", "SAVE"}, {"RUN", "RUN"}, {"TEACH", "TEACH"}, {"DONE", "DONE"}, {"RUN", "RUN"}, {"STOP", "STOP"},
    {"ST LIST", "ST"}, {"ST", "ST"}, {"ST FOO", "ST"}, {"ST DEL HOME", "ST"},
    {"HOME", "HOME"}, {"HOME", "HOME"} /* already homing */,
  };
  for (unsigned i = 0; i < sizeof lines / sizeof lines[0]; i++) {
    const char *log = cmd(lines[i][0]);
    char msg[64]; snprintf(msg, sizeof msg, "command '%s'", lines[i][0]);
    TEST_ASSERT_EQUAL_MESSAGE(1, count_replies(log, lines[i][1]), msg);
  }
  TEST_ASSERT_NOT_NULL(strstr(cmd("PING"), "OK PING ras-arm " FW_VERSION));
  TEST_ASSERT_TRUE(run_until(ST_IDLE, 60000));
  TEST_ASSERT_NOT_NULL(strstr(cmd("STATUS"), "OK STATUS IDLE NONE homed=1 running=0"));
  TEST_ASSERT_NOT_NULL(strstr(cmd(""), "") );              // empty line: no reply, no crash
  TEST_ASSERT_EQUAL(0, (int)strlen(fake_log));
}
void test_telemetry_format(void) {
  TEST_ASSERT_TRUE(boot(0, 30, 90));
  fake_ir_blocked = true; run_ms(300);
  char line[128]; strcpy(line, fake_last_tel);
  TEST_ASSERT_EQUAL_STRING_LEN("T,", line, 2);
  TEST_ASSERT_EQUAL_CHAR('\n', line[strlen(line) - 1]);
  char *f[32]; int n = 0;
  for (char *t = strtok(line, ",\n"); t && n < 32; t = strtok(0, ",\n")) f[n++] = t;
  TEST_ASSERT_EQUAL(19, n);                                // T ms state j1..j6 grip x y z ir col err lim estop layout
  TEST_ASSERT_EQUAL_STRING("IDLE", f[2]);
  TEST_ASSERT_INT_WITHIN(5, (int)(GRIP_OPEN_MM * 10), atoi(f[9]));
  TEST_ASSERT_INT_WITHIN(1, (int)CELL_HOME_X, atoi(f[10]));
  TEST_ASSERT_INT_WITHIN(1, (int)CELL_HOME_Z, atoi(f[12]));
  TEST_ASSERT_EQUAL_STRING("1", f[13]);                    // IR blocked
  TEST_ASSERT_EQUAL_STRING("0", f[15]);                    // no error
  TEST_ASSERT_EQUAL(3, (int)strlen(f[16]));                // three limit switch digits
  TEST_ASSERT_EQUAL_STRING("0", f[17]);                    // power OK
  TEST_ASSERT_EQUAL_STRING("0", f[18]);                    // layout version: blank EEPROM, never committed
  for (int i = 3; i < 16; i++) { char *e; strtol(f[i], &e, 10); TEST_ASSERT_EQUAL_CHAR(0, *e); }   // integers only
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_ik_fk_roundtrip);
  RUN_TEST(test_ik_gripper_vertical);
  RUN_TEST(test_ik_rejects_unreachable);
  RUN_TEST(test_ik_stations_reachable);
  RUN_TEST(test_ik_golden_vectors);
  RUN_TEST(test_joint_conversions);
  RUN_TEST(test_colour_decisions);
  RUN_TEST(test_colour_calibration);
  RUN_TEST(test_jiggle_pattern);
  RUN_TEST(test_rehome_rule_and_set);
  RUN_TEST(test_eeprom_crc);
  RUN_TEST(test_stations_defaults_valid);
  RUN_TEST(test_stations_validation_codes);
  RUN_TEST(test_stations_table_edits);
  RUN_TEST(test_stations_eeprom);
  RUN_TEST(test_homing_unknown_start);
  RUN_TEST(test_homing_start_on_switch);
  RUN_TEST(test_homing_broken_wire);
  RUN_TEST(test_homing_switch_never_found);
  RUN_TEST(test_cycle_with_one_failed_pick);
  RUN_TEST(test_pick_fails_three_times_is_error);
  RUN_TEST(test_unknown_colour_is_error);
  RUN_TEST(test_teach_over_serial);
  RUN_TEST(test_move_replies);
  RUN_TEST(test_estop_from_pick);
  RUN_TEST(test_every_command_one_reply);
  RUN_TEST(test_telemetry_format);
  RUN_TEST(test_st_protocol);
  RUN_TEST(test_swapped_bins);
  RUN_TEST(test_reject_bin);
  return UNITY_END();
}
