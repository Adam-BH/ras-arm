// LAYER 3 — JOINTS
// One interface for all 7 joints, in DEGREES (gripper: mm). Above this layer nobody knows
// (or cares) whether J2 is a stepper and J5 a servo. Converts degrees ↔ steps / µs, applies limits.
#pragma once
#include "../config.h"

bool  joints_init(void);                                    // false = servo driver missing
void  joints_update(uint32_t now);                          // stepper ramp + servo ramps

void  joints_move(const angles_t *target, uint32_t duration_ms); // all 7 arrive together (clamped,
                                                                 // slowed down to the speed limits)
void  joint_move(uint8_t j, float deg, uint32_t duration_ms);    // just one
float joint_angle(uint8_t j);                                    // current angle (deg / mm)
float joint_target(uint8_t j);                                   // where it is going
void  joints_get(angles_t *out);
bool  joints_busy(void);
void  joints_stop(void);

bool  joint_is_stepper(uint8_t j);
void  joint_run_toward_switch(uint8_t j, float deg_per_s);  // homing helper (+ = toward switch)
void  joint_set_homed(uint8_t j);                            // "you are exactly at the switch angle"
bool  joint_within_limits(const angles_t *a);                // J1..J6 (the gripper is clamped)

// conversions (exposed for the tests)
float    joint_clamp(uint8_t j, float deg);
int32_t  joint_deg_to_steps(uint8_t j, float deg);
uint16_t joint_deg_to_us(uint8_t j, float deg);
