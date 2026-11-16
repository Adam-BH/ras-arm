// config.h — every number of the robot in ONE place + the shared types.
// When the CAD or the wiring changes, only this file changes.
// The sim mirrors the geometry/limits/workcell in sim/src/core/config.ts — keep both in sync.
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define FW_VERSION "1.0.0"

// ───────────── Pins (Arduino UNO + CNC Shield V3) ─────────────
#define PIN_STEP_J1   2     // shield X  (PD2)
#define PIN_STEP_J2   3     // shield Y  (PD3)
#define PIN_STEP_J3   4     // shield Z  (PD4)
#define PIN_DIR_J1    5     // PD5
#define PIN_DIR_J2    6     // PD6
#define PIN_DIR_J3    7     // PD7
#define PIN_EN        8     // all drivers, active LOW
#define PIN_LIMIT_J1  9     // X-  (switches wired NC: pressed OR broken wire = HIGH)
#define PIN_LIMIT_J2  10    // Y-
#ifndef LIMIT_Z_ON_D12      // some CNC shield clones route Z- to D12: build with -DLIMIT_Z_ON_D12
#define PIN_LIMIT_J3  11    // Z-
#else
#define PIN_LIMIT_J3  12
#endif
#define PIN_ESTOP     14    // A0  HIGH = motor rail OK (47k/10k divider from 24 V)
#define PIN_TCS_LED   15    // A1  TCS34725 LED enable (optional)
#define PIN_BUZZER    16    // A2  spare / buzzer
#define PIN_IR        17    // A3  E18-D80NK, INPUT_PULLUP, LOW = object
#define PIN_LED       13    // status LED (blink codes)
// A4/A5 = I²C SDA/SCL, D0/D1 = USB serial

#define SERIAL_BAUD   115200UL
#define I2C_CLOCK_HZ  400000UL
#define I2C_TIMEOUT_US 3000UL
#define I2C_PCA9685   0x40  // servo driver
#define I2C_TCS34725  0x29  // colour sensor
#define WATCHDOG_MS   250   // kicked every loop()

// PCA9685 channels
#define SERVO_CH_J4   0
#define SERVO_CH_J5   1
#define SERVO_CH_J6   2
#define SERVO_CH_GRIP 3
#define PCA9685_OSC_HZ   25000000UL
#define SERVO_FRAME_HZ   50          // 20 ms servo period
#define SERVO_UPDATE_MS  20          // servo ramps are sent at the frame rate (I²C is slow)

// ───────────── Geometry (mm) ─────────────
#define L1          165.0f  // shoulder → elbow   (v2 design)
#define L2          165.0f  // elbow → wrist      (v2 design)
#define L3          110.0f  // wrist → gripper tip (kept vertical)
#define H_SHOULDER  120.0f  // shoulder axis above the table
#define IK_MAX_COS3 0.995f  // reject (nearly) straight elbow
#define IK_MIN_COS3 -1.0f

// ───────────── Joints ─────────────
enum { J1, J2, J3, J4, J5, J6, JG, NJ };   // JG = gripper (value in mm, not deg)
#define N_STEPPERS 3                        // J1..J3 are steppers, J4..JG servos
#define N_SERVOS   (NJ - N_STEPPERS)

// steps per joint degree = 200 steps × microstep × ratio / 360  (microstep = shield jumpers)
#define MICROSTEP_J1  16
#define MICROSTEP_J2  8           // TEAM TO CONFIRM: v2 report assumes 16× on J2/J3
#define MICROSTEP_J3  8
#define RATIO_J2_GEARBOX 5.18f             // PG518 planetary
#define RATIO_J2_BELT   (88.0f / 16.0f)    // 16T → 88T = 5.5  (v2)
#define RATIO_J3_BELT   (104.0f / 16.0f)   // 16T → 104T = 6.5 (v2)
#define SPD_J1  (200.0f * MICROSTEP_J1 / 360.0f)                                     //   8.89 direct drive
#define SPD_J2  (200.0f * MICROSTEP_J2 * RATIO_J2_GEARBOX * RATIO_J2_BELT / 360.0f)  // 126.6
#define SPD_J3  (200.0f * MICROSTEP_J3 * RATIO_J3_BELT / 360.0f)                     //  28.9
#define BASE_MAX_ACC_DPS2 143.0f  // J1 acceleration cap, 2.5 rad/s² (v2 report: base torque budget)
// flip a motor's direction here if it turns the wrong way (1 or -1)
#define DIR_SIGN_J1  1
#define DIR_SIGN_J2  1
#define DIR_SIGN_J3  1

//                                  J1    J2    J3    J4    J5    J6   grip
static const float JOINT_MIN[NJ]  = {-100, -20,   20,  -90, -100,  -90,   0};
static const float JOINT_MAX[NJ]  = { 100, 110,  150,   90,  100,   90,  50};
static const float JOINT_MAX_DPS[NJ] = {90,  60,   90,  180,  180,  180, 100}; // speed limit (deg/s, grip mm/s)
#define J1_WORK_MIN  -90.0f        // the cell only uses −90…90 (bins at ±90)
#define J1_WORK_MAX   90.0f

// homing: where each switch clicks and which way to drive to reach it. Order J2 → J3 → J1.
static const float  HOME_SWITCH_DEG[N_STEPPERS] = {-100, -20, 150};
static const int8_t HOME_DIR[N_STEPPERS]        = {-1, -1, +1};
#define HOMING_FAST_DPS   30.0f
#define HOMING_SLOW_DPS   4.0f
#define HOMING_BACKOFF    5.0f     // deg, back off after the fast touch
#define HOMING_LEAVE_MAX  15.0f    // deg; started "pressed" and still pressed after this → broken wire
#define HOMING_SEARCH_MARGIN 10.0f // deg beyond the joint range before "switch never found"
#define HOMING_SLOW_MAX   (HOMING_BACKOFF + 3.0f)
#define HOMING_PHASE_TIMEOUT_MS 15000UL

// servo calibration: pulse = CENTER_US + value × US_PER_UNIT (sign = direction).
// value = deg for J4..J6 (0 = centre), mm opening for the gripper (0 = closed).
// DS3218MG comes as 180° or 270°: set DS3218_RANGE_DEG to the version you have.
#define DS3218_RANGE_DEG 270.0f
#define MG996_RANGE_DEG  180.0f
#define SERVO_SPAN_US    2000.0f   // 500…2500 µs end to end
//                                       J4                               J5                                J6                              grip
static const float SERVO_CENTER_US[N_SERVOS]   = {1500,                           1500,                            1500,                           1000};
static const float SERVO_US_PER_UNIT[N_SERVOS] = {SERVO_SPAN_US/DS3218_RANGE_DEG, SERVO_SPAN_US/DS3218_RANGE_DEG,  SERVO_SPAN_US/MG996_RANGE_DEG,  20.0f};
#define SERVO_US_LO  500
#define SERVO_US_HI  2500

// ───────────── Motion ─────────────
#define STEP_ISR_HZ      20000   // Timer1 stepper interrupt
#define STEP_ISR_HZ_OVER_MS (STEP_ISR_HZ / 1000)
#define RAMP_FRACTION    0.25f   // trapezoid: 25 % accel, 50 % cruise, 25 % decel
#define RAMP_MIN_RATE    0.05f   // never crawl to a stall before the last step
#define MIN_MOVE_MS      50
#define LINE_STEP_MM     10.0f   // straight lines = IK every 10 mm
#define LINE_MAX_WP      40
#define LINE_DEFAULT_MM_S 50.0f
#define JOG_MS           300
#define SAFE_Z           110.0f  // transit height of the tip
#define PLACE_ABOVE_DZ   20.0f   // (pysim sorter only; the firmware uses each station's above_dz)
#define MOVE_HOME_MS     1500
#define MOVE_TRANSIT_MS  1600
#define MOVE_ABOVE_MS    1200
#define PICK_DOWN_MM_S   40.0f
#define PICK_UP_MM_S     60.0f
#define PLACE_DOWN_MM_S  60.0f
#define PLACE_UP_MM_S    80.0f
#define GRIP_MOVE_MS     300
#define WIGGLE_MS        150

// ───────────── Workcell (mm, world frame: x → cradle, y left, z up) ─────────────
// CAD default layout: used on first boot / bad EEPROM and by ST FACTORY (see 4_motion/stations.c).
#define CELL_HOME_X   160.0f
#define CELL_HOME_Z   170.0f
#define CELL_CRADLE_X 220.0f
#define CELL_GRIP_Z    24.0f
#define CELL_BIN_Y    220.0f     // BIN_A at +y (red), BIN_B at −y (blue)
#define CELL_DROP_Z    78.0f
#define CELL_BIN_ROLL  90.0f     // bins: fingers tangent to the base (J6 = roll − J1 = 0)

// station table (taught or sent by the sim, validated here, stored in EEPROM)
#define MAX_STATIONS      8
#define STATION_NAME_LEN  8      // incl. the final \0 → names up to 7 chars, A-Z 0-9 _
#define STATION_ABOVE_DZ  60     // default approach height above the grip pose (mm)
#define Z_MIN_TABLE       10.0f  // keep-out: tip never below this (mm)
#define R_MIN_BASE        90.0f  // keep-out: base column radius + margin (mm)
#define MIN_STATION_GAP   80.0f  // pick/bin centres at least this far apart in x,y (mm)  TEAM TO CONFIRM
#define BIN_MIN_DROP_Z    70.0f  // drop height ≥ bin wall (60 mm) + margin
#define PREVIEW_MM_S      20.0f  // ST GO: slow line down onto the station

// ───────────── Cycle ─────────────
#define BOOT_SETTLE_MS   500     // servos reach neutral before homing
#define IR_STABLE_MS     200
#define IR_CLEAR_MS      20
#define LIMIT_DEBOUNCE_MS 5
#define ESTOP_ON_MS      2
#define ESTOP_OFF_MS     50
#define COLOUR_SAMPLES   5
#define COLOUR_TRIES     3       // unknown colour this many times → ERROR
#define COLOUR_INTEG_MS  26      // one TCS34725 integration (24 ms) + margin
#define TCS_ATIME_VAL    0xF6    // 24 ms
#define TCS_GAIN_VAL     0x01    // ×4
#define PICK_RETRIES     2
#define GRIP_OPEN_MM     45.0f
#define GRIP_CLOSED_MM   26.0f
#define REHOME_EVERY     10      // cycles between drift-check homings
#define COLOUR_TIMEOUT_MS 200    // a sample that never becomes ready (I²C fault) → UNKNOWN
#define TELEMETRY_MS     50      // 20 Hz
#define LED_BLINK_MS     200     // one blink = 200 ms on + 200 ms off
#define LED_PAUSE_MS     1200    // pause between blink groups
#define LED_IDLE_MS      1000    // heartbeat half period when healthy

// ───────────── Decisions (defaults; rules in 5_tasks/decisions.c, SET <key> <value> at runtime) ─────────────
#define DEC_CONFIDENT_RATIO 0.67f   // nearest colour must be < 67 % of the distance to the other
#define DEC_SAME_AS_LAST    0.08f   // chroma distance that counts as "same as the last cube"
#define DEC_EMPTY_RATIO     1.3f    // clear < empty_clear × 1.3 → nothing on the sensor
#define DEC_USE_LAST_TRICK  1.0f    // 1 = resolve ambiguous colours with the 2-colour trick
#define DEC_GRIP_WIGGLE_DEG 5.0f    // J6 wiggle while closing on a retry (0 = off)
#define DEC_JIGGLE_MM       3.0f    // grip point shift on retries
#define DEC_JIGGLE_DEG      4.0f    // roll shift on retries
#define DEC_EMPTY_CLEAR     250     // clear channel with nothing on the sensor (CAL EMPTY replaces it)
#define DEC_REF_RED_R  0.62f        // chroma (r/c, g/c, b/c) of a red cube  (CAL RED replaces it)
#define DEC_REF_RED_G  0.20f
#define DEC_REF_RED_B  0.16f
#define DEC_REF_BLUE_R 0.16f        // chroma of a blue cube (CAL BLUE replaces it)
#define DEC_REF_BLUE_G 0.29f
#define DEC_REF_BLUE_B 0.60f

// ───────────── EEPROM layout (1 KB) ─────────────
// [0..127] = old fixed-pose record (v1, ignored now)
#define EE_ADDR_CAL       128
#define EE_ADDR_STATIONS  256    // station table + colour map, ~165 bytes
#define EE_MAGIC_CAL      0xC0C1
#define EE_MAGIC_STATIONS 0x57A7
#define STATIONS_REC_VERSION 1   // bump when station_t changes: old records → CAD defaults

// ───────────── Shared types ─────────────
typedef struct { float x, y, z, roll; } pose_t;    // gripper tip, mm / deg
typedef struct { float j[NJ]; } angles_t;          // degrees (grip: mm)
typedef struct { uint16_t r, g, b, c; } rgbc_t;    // raw colour reading
typedef enum { COL_EMPTY, COL_RED, COL_BLUE, COL_UNKNOWN, N_COLOURS } colour_t;
