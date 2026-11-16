// LAYER 5 — TASKS · robot skills
// Each task is a small non-blocking sequence: start() once, then update() every loop
// until it returns TASK_DONE or TASK_FAILED. The decisions come from decisions.h.
#pragma once
#include "../config.h"
#include "../4_motion/motion.h"
#include "../4_motion/stations.h"

typedef enum { TASK_RUNNING, TASK_DONE, TASK_FAILED } task_status_t;

void tasks_init(void);                     // sensors, colour sensor, motion, stations, decisions + calibration
void tasks_update(uint32_t now);           // sensors debounce + motion, every loop
bool task_servos_ok(void);                 // PCA9685 answered
bool task_colour_ok(void);                 // TCS34725 answered

void task_homing_start(void);              task_status_t task_homing_update(uint32_t now);
const char *task_homing_why(void);         // flash string: why homing failed
uint8_t     task_homing_joint(void);       // which joint was homing (0-based)
void task_colour_start(void);              task_status_t task_colour_update(uint32_t now, colour_t *out, rgbc_t *avg);
void task_pick_start(uint8_t attempt);     task_status_t task_pick_update(uint32_t now);
void task_place_start(colour_t c);         task_status_t task_place_update(uint32_t now);
void task_return_start(void);              task_status_t task_return_update(uint32_t now);

// manual moves (console → FSM → here). false = unreachable / out of limits
bool task_move(const pose_t *p, uint32_t ms);
bool task_line(const pose_t *p, float mm_s);
bool task_movej(const angles_t *a, uint32_t ms);
bool task_jog(float dx, float dy, float dz, float droll);
bool task_jogj(uint8_t j, float delta);
void task_grip(float mm);
bool task_busy(void);
void task_stop(void);
void task_where(pose_t *out);

// teach mode
st_err_t task_teach_save(uint8_t station);   // current tip → station, validated + committed
bool     task_teach_go(uint8_t station);     // straight to the grip pose
bool     task_preview_start(uint8_t station); // above, then slow line down (task_busy until done)
void   task_calibrate_finish(colour_t c, const rgbc_t *avg);   // store + persist (EEPROM)
