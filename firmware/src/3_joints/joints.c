// LAYER 3 — JOINTS
#include "joints.h"
#include "../2_devices/actuators.h"
#include <math.h>

static const float SPD[N_STEPPERS] = {SPD_J1, SPD_J2, SPD_J3};

// servo joints: we remember where we sent them and ramp toward the target (index = j − N_STEPPERS)
static float    srv_now[N_SERVOS], srv_from[N_SERVOS], srv_to[N_SERVOS];
static uint32_t srv_t0, srv_T, srv_last_write;
static bool     srv_moving;
static uint32_t last_ms;

static float absf(float v) { return v < 0 ? -v : v; }

bool  joint_is_stepper(uint8_t j)        { return j < N_STEPPERS; }
float joint_clamp(uint8_t j, float deg)  { return deg < JOINT_MIN[j] ? JOINT_MIN[j] : deg > JOINT_MAX[j] ? JOINT_MAX[j] : deg; }

int32_t joint_deg_to_steps(uint8_t j, float deg) {
  float s = deg * SPD[j];
  return (int32_t)(s < 0 ? s - 0.5f : s + 0.5f);           // round to nearest step
}

uint16_t joint_deg_to_us(uint8_t j, float deg) {            // linear calibration per servo
  uint8_t s = j - N_STEPPERS;
  float us = SERVO_CENTER_US[s] + joint_clamp(j, deg) * SERVO_US_PER_UNIT[s];
  if (us < SERVO_US_LO) us = SERVO_US_LO;
  if (us > SERVO_US_HI) us = SERVO_US_HI;
  return (uint16_t)(us + 0.5f);
}

static void servos_write_all(void) {
  for (uint8_t s = 0; s < N_SERVOS; s++) servo_write_us(SERVO_CH_J4 + s, joint_deg_to_us(N_STEPPERS + s, srv_now[s]));
}

bool joints_init(void) {
  steppers_init();
  bool ok = servos_init();
  for (uint8_t s = 0; s < N_SERVOS; s++)                    // servos to neutral, gripper open
    srv_now[s] = srv_to[s] = (N_STEPPERS + s == JG) ? GRIP_OPEN_MM : 0.0f;
  servos_write_all();
  steppers_enable(true);
  return ok;
}

bool joint_within_limits(const angles_t *a) {
  for (uint8_t j = 0; j < JG; j++)
    if (a->j[j] < JOINT_MIN[j] - 0.01f || a->j[j] > JOINT_MAX[j] + 0.01f) return false;
  return true;
}

void joints_move(const angles_t *t, uint32_t ms) {
  // never faster than each joint's speed limit (trapezoid average = peak × (1 − RAMP))
  for (uint8_t j = 0; j < NJ; j++) {
    float need = absf(joint_clamp(j, t->j[j]) - joint_angle(j)) / JOINT_MAX_DPS[j] / (1.0f - RAMP_FRACTION) * 1000.0f;
    if (need > ms) ms = (uint32_t)need + 1;
  }
  // base: trapezoid accel = D / (f(1−f)T²) ≤ BASE_MAX_ACC_DPS2  →  T ≥ √(D / (f(1−f)·A))
  float need = sqrtf(absf(joint_clamp(J1, t->j[J1]) - joint_angle(J1)) /
                     (RAMP_FRACTION * (1.0f - RAMP_FRACTION) * BASE_MAX_ACC_DPS2)) * 1000.0f;
  if (need > ms) ms = (uint32_t)need + 1;
  int32_t target[N_STEPPERS];
  for (uint8_t j = 0; j < N_STEPPERS; j++) target[j] = joint_deg_to_steps(j, joint_clamp(j, t->j[j]));
  steppers_move_to(target, ms);
  for (uint8_t s = 0; s < N_SERVOS; s++) { srv_from[s] = srv_now[s]; srv_to[s] = joint_clamp(N_STEPPERS + s, t->j[N_STEPPERS + s]); }
  srv_t0 = last_ms; srv_T = ms ? ms : 1; srv_moving = true;
}

void joint_move(uint8_t j, float deg, uint32_t ms) {
  angles_t a;
  for (uint8_t k = 0; k < NJ; k++) a.j[k] = joint_target(k);   // others keep their targets
  a.j[j] = deg;
  joints_move(&a, ms);
}

float joint_angle(uint8_t j)  { return joint_is_stepper(j) ? stepper_pos(j) / SPD[j] : srv_now[j - N_STEPPERS]; }
float joint_target(uint8_t j) { return joint_is_stepper(j) ? joint_angle(j) : srv_to[j - N_STEPPERS]; }

void joints_get(angles_t *out) { for (uint8_t j = 0; j < NJ; j++) out->j[j] = joint_angle(j); }
bool joints_busy(void)         { return steppers_busy() || srv_moving; }
void joints_stop(void) {
  steppers_stop(); srv_moving = false;
  for (uint8_t s = 0; s < N_SERVOS; s++) srv_to[s] = srv_now[s];
}

void joint_run_toward_switch(uint8_t j, float dps) {
  int8_t dir = dps >= 0 ? HOME_DIR[j] : -HOME_DIR[j];
  stepper_run(j, dir, absf(dps) * SPD[j]);
}

void joint_set_homed(uint8_t j) {
  steppers_stop();
  stepper_set_pos(j, joint_deg_to_steps(j, HOME_SWITCH_DEG[j]));
}

void joints_update(uint32_t now) {
  last_ms = now;
  steppers_update(now);
  if (!srv_moving) return;
  float k = (float)(now - srv_t0) / srv_T;
  if (k >= 1.0f) { k = 1.0f; srv_moving = false; }
  float e = k < 0.5f ? 2 * k * k : 1 - 2 * (1 - k) * (1 - k);   // smooth start/stop
  for (uint8_t s = 0; s < N_SERVOS; s++) srv_now[s] = srv_from[s] + (srv_to[s] - srv_from[s]) * e;
  if (srv_moving && now - srv_last_write < SERVO_UPDATE_MS) return;   // I²C at the servo frame rate
  srv_last_write = now;
  servos_write_all();
}
