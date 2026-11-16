// LAYER 4 — MOTION · kinematics (pure math, no hardware, fully unit-tested)
// IK: "tip at (x, y, z)" → joint angles.  FK: joint angles → tip position.
// Gripper kept vertical ⇒ base yaw + a 2-link planar arm (law of cosines).
#pragma once
#include "../config.h"

typedef enum { IK_OK, IK_UNREACHABLE, IK_LIMIT } ik_result_t;

ik_result_t ik_solve(const pose_t *tip, angles_t *out);   // keeps gripper value untouched
void        fk_solve(const angles_t *a, pose_t *tip);
float       ik_cos3(const pose_t *tip);                   // cos J3: > IK_MAX_COS3 and ≤ 1 = (near) singular
