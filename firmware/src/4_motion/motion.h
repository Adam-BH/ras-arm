// LAYER 4 — MOTION · moves & saved poses
// "Go to this position" in mm. Uses kinematics to get angles, joints to move.
#pragma once
#include "kinematics.h"

typedef enum { POSE_HOME, POSE_CUBE, POSE_BIN_A, POSE_BIN_B, POSE_COUNT } pose_id_t;
#define POSE_NAMES "HOME\0CUBE\0BIN_A\0BIN_B"   // flash string list, same order

void   motion_init(void);                                   // joints + taught poses
bool   motion_servos_ok(void);                              // PCA9685 answered at boot
void   motion_update(uint32_t now);

bool   motion_to(const pose_t *p, uint32_t duration_ms);    // IK, all joints together; false = unreachable
bool   motion_line(const pose_t *p, float mm_per_s);        // straight line, whole line checked first
bool   motion_joints(const angles_t *a, uint32_t duration_ms); // joint space J1..J6 (grip kept); false = limits
bool   motion_jog(float dx, float dy, float dz, float droll);  // relative Cartesian
bool   motion_jog_joint(uint8_t j, float delta);            // relative, one joint; false = limit
void   motion_grip(float opening_mm, uint32_t duration_ms);
void   motion_joint(uint8_t j, float deg, uint32_t duration_ms); // one joint (wrist wiggle)
bool   motion_busy(void);
void   motion_stop(void);
void   motion_where(pose_t *out);                           // current tip position (FK)

// homing helpers (pass-through to the joints layer)
void   motion_run_toward_switch(uint8_t j, float deg_per_s);
void   motion_set_homed(uint8_t j);
float  motion_joint_angle(uint8_t j);

// taught positions (EEPROM with CRC, defaults from the CAD layout)
pose_t pose_get(pose_id_t id);
void   pose_save(pose_id_t id, const pose_t *p);
bool   poses_load(void);                                    // false = bad CRC → CAD defaults kept
void   poses_defaults(void);
