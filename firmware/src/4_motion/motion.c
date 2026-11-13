// LAYER 4 — MOTION · moves & saved poses
#include "motion.h"
#include "../3_joints/joints.h"
#include "../2_devices/storage.h"
#include <math.h>

static pose_t poses[POSE_COUNT];
static bool   servos_ok;

void poses_defaults(void) {                     // from the CAD workcell layout
  const pose_t d[POSE_COUNT] = {
    {CELL_HOME_X,   0,            CELL_HOME_Z, 0},
    {CELL_CRADLE_X, 0,            CELL_GRIP_Z, 0},
    {0,             CELL_BIN_Y,   CELL_DROP_Z, 0},   // bin A, +90°, red
    {0,            -CELL_BIN_Y,   CELL_DROP_Z, 0},   // bin B, −90°, blue
  };
  for (uint8_t i = 0; i < POSE_COUNT; i++) poses[i] = d[i];
}

// ── straight lines = IK waypoints computed on the fly (no array: RAM is only 2 KB) ──
// ponytail: each 10 mm segment is its own trapezoid (tiny slow-down at each waypoint);
// blend segments in the stepper ramp if lines ever look jerky.
static pose_t   wp_from, wp_to;
static uint8_t  wp_n, wp_i;
static uint32_t wp_ms;
static pose_t lerp_pose(uint8_t k) {
  float t = (float)k / wp_n; pose_t p;
  p.x = wp_from.x + (wp_to.x - wp_from.x) * t;  p.y = wp_from.y + (wp_to.y - wp_from.y) * t;
  p.z = wp_from.z + (wp_to.z - wp_from.z) * t;  p.roll = wp_from.roll + (wp_to.roll - wp_from.roll) * t;
  return p;
}

bool poses_load(void)                     { return storage_load(EE_ADDR_POSES, EE_MAGIC_POSES, poses, sizeof poses); }
void pose_save(pose_id_t id, const pose_t *p) {
  poses[id] = *p;
  storage_save(EE_ADDR_POSES, EE_MAGIC_POSES, poses, sizeof poses);
}
pose_t pose_get(pose_id_t id) { return poses[id]; }

void motion_init(void) { wp_n = wp_i = 0; servos_ok = joints_init(); poses_defaults(); poses_load(); }
bool motion_servos_ok(void) { return servos_ok; }

void motion_where(pose_t *out) { angles_t a; joints_get(&a); fk_solve(&a, out); }

static void targets(angles_t *a) { for (uint8_t j = 0; j < NJ; j++) a->j[j] = joint_target(j); }

bool motion_to(const pose_t *p, uint32_t ms) {
  angles_t a; targets(&a);                          // keeps the gripper's target
  if (ik_solve(p, &a) != IK_OK) return false;       // refuse before moving
  wp_n = wp_i = 0;
  joints_move(&a, ms);
  return true;
}

bool motion_joints(const angles_t *q, uint32_t ms) {
  angles_t a = *q; a.j[JG] = joint_target(JG);
  if (!joint_within_limits(&a)) return false;
  wp_n = wp_i = 0;
  joints_move(&a, ms);
  return true;
}

bool motion_line(const pose_t *to, float mm_s) {
  pose_t from; motion_where(&from);
  float d = sqrtf((to->x - from.x) * (to->x - from.x) + (to->y - from.y) * (to->y - from.y) +
                  (to->z - from.z) * (to->z - from.z));
  float nf = ceilf(d / LINE_STEP_MM);
  uint8_t n = nf < 1 ? 1 : nf > LINE_MAX_WP ? LINE_MAX_WP : (uint8_t)nf;
  pose_t save_from = wp_from, save_to = wp_to; uint8_t save_n = wp_n;
  wp_from = from; wp_to = *to; wp_n = n;
  angles_t check; targets(&check);
  for (uint8_t k = 1; k <= n; k++) {                // validate the whole line before moving
    pose_t p = lerp_pose(k);
    if (ik_solve(&p, &check) != IK_OK) { wp_from = save_from; wp_to = save_to; wp_n = save_n; return false; }
  }
  if (mm_s <= 0) mm_s = LINE_DEFAULT_MM_S;
  wp_i = 0; wp_ms = (uint32_t)(1000.0f * d / n / mm_s);   // motion_update feeds the waypoints
  return true;
}

bool motion_jog(float dx, float dy, float dz, float droll) {
  pose_t p; motion_where(&p);
  p.x += dx; p.y += dy; p.z += dz; p.roll += droll;
  return motion_to(&p, JOG_MS);
}

bool motion_jog_joint(uint8_t j, float delta) {
  angles_t a; targets(&a);
  a.j[j] += delta;
  if (j == JG) { motion_grip(a.j[JG], GRIP_MOVE_MS); return true; }
  return motion_joints(&a, JOG_MS);
}

void motion_grip(float mm, uint32_t ms)              { joint_move(JG, mm, ms); }
void motion_joint(uint8_t j, float deg, uint32_t ms) { joint_move(j, deg, ms); }
bool motion_busy(void)                               { return joints_busy() || wp_i < wp_n; }
void motion_stop(void)                               { wp_n = wp_i = 0; joints_stop(); }

void  motion_run_toward_switch(uint8_t j, float dps) { wp_n = wp_i = 0; joint_run_toward_switch(j, dps); }
void  motion_set_homed(uint8_t j)                    { joint_set_homed(j); }
float motion_joint_angle(uint8_t j)                  { return joint_angle(j); }

void motion_update(uint32_t now) {
  joints_update(now);
  if (wp_i < wp_n && !joints_busy()) {             // feed the next waypoint
    angles_t a; targets(&a);
    pose_t p = lerp_pose(++wp_i);
    ik_solve(&p, &a);                               // already validated
    joints_move(&a, wp_ms);
  }
}
