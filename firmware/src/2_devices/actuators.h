// LAYER 2 — DEVICES · actuators
// Talks to the motor hardware in its own units: STEPS for steppers, MICROSECONDS for servos.
// Knows nothing about degrees, joints or cubes. Also drives the status LED.
#pragma once
#include "../config.h"

// ── 3 steppers (J1, J2, J3) through TMC2209 drivers ──
void    steppers_init(void);
void    steppers_enable(bool on);
void    steppers_move_to(const int32_t target[N_STEPPERS], uint32_t duration_ms); // all arrive together, trapezoid
void    stepper_run(uint8_t axis, int8_t dir, float steps_per_s);                 // constant speed (homing)
void    steppers_stop(void);
bool    steppers_busy(void);
int32_t stepper_pos(uint8_t axis);
void    stepper_set_pos(uint8_t axis, int32_t steps);                              // after homing
void    steppers_update(uint32_t now);                                             // speed ramp, every loop

// ── 4 servos (J4, J5, J6, gripper) through the PCA9685 ──
bool servos_init(void);                     // false = PCA9685 not answering
void servo_write_us(uint8_t channel, uint16_t us);

// ── status LED: 0 = heartbeat, 1..9 = blink that many times then pause, LED_SOLID = on ──
#define LED_SOLID 0xFF
void status_led_update(uint32_t now, uint8_t pattern);
