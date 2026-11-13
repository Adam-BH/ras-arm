// LAYER 4 — MOTION · kinematics
// Conventions: x forward (cradle), y left, z up, origin = base axis on the table.
// J1 yaw (0 = cradle), J2 from vertical (+ = forward), J3 bend relative to the upper arm.
#include "kinematics.h"
#include "../3_joints/joints.h"
#include <math.h>

#define PI_F   3.14159265f
#define DEG(r) ((r) * (180.0f / PI_F))
#define RAD(d) ((d) * (PI_F / 180.0f))

ik_result_t ik_solve(const pose_t *p, angles_t *q) {
  float r  = sqrtf(p->x * p->x + p->y * p->y);       // horizontal reach
  float zw = p->z + L3 - H_SHOULDER;                  // wrist height above the shoulder
  float c3 = (r * r + zw * zw - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
  if (c3 < IK_MIN_COS3 || c3 > IK_MAX_COS3) return IK_UNREACHABLE; // stay away from a straight arm

  float j3 = acosf(c3);                               // elbow-up solution
  float j2 = atan2f(r, zw) - atan2f(L2 * sinf(j3), L1 + L2 * cosf(j3));

  q->j[J1] = DEG(atan2f(p->y, p->x));
  q->j[J2] = DEG(j2);
  q->j[J3] = DEG(j3);
  q->j[J4] = 0.0f;                                    // forearm roll neutral when picking
  q->j[J5] = 180.0f - q->j[J2] - q->j[J3];            // keeps the gripper vertical
  q->j[J6] = p->roll - q->j[J1];                      // keeps fingers aligned with the cube
  return joint_within_limits(q) ? IK_OK : IK_LIMIT;  // J1..J6; the gripper is never touched
}

void fk_solve(const angles_t *a, pose_t *t) {
  float s2 = RAD(a->j[J2]), s23 = RAD(a->j[J2] + a->j[J3]);
  float r  = L1 * sinf(s2) + L2 * sinf(s23);          // wrist reach
  float zw = L1 * cosf(s2) + L2 * cosf(s23);          // wrist height above shoulder
  float y1 = RAD(a->j[J1]);
  t->x = r * cosf(y1);
  t->y = r * sinf(y1);
  t->z = zw + H_SHOULDER - L3;                        // tip is L3 below the wrist
  t->roll = a->j[J6] + a->j[J1];
}
