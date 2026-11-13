// Port of firmware/src/4_motion/kinematics.c — same conventions (docs/kinematics.md):
// x forward (cradle), y left, z up, origin = base axis on the table.
// J1 yaw (0 = cradle), J2 from vertical (+ forward), J3 relative to the upper arm (0 = straight),
// J5 = 180 − J2 − J3 keeps the gripper vertical, J6 = roll − J1, J4 = 0.
import { L1, L2, L3, H_SHOULDER, IK_MAX_COS3, IK_MIN_COS3, JOINT_MIN, JOINT_MAX } from './config';

export type Pose = { x: number; y: number; z: number; roll: number };
export type IkStatus = 'ok' | 'unreachable' | 'limit';
export const IK_STATUS_CODE: Record<IkStatus, number> = { ok: 0, unreachable: 1, limit: 2 };

const DEG = 180 / Math.PI, RAD = Math.PI / 180;

export function withinLimits(q: number[]): boolean {          // J1..J6 (the gripper is clamped)
  for (let j = 0; j < 6; j++) if (q[j] < JOINT_MIN[j] - 0.01 || q[j] > JOINT_MAX[j] + 0.01) return false;
  return true;
}

/** returns joint angles J1..J6 (deg) and the status, like ik_solve() */
export function ik(p: Pose): { status: IkStatus; q: number[] } {
  const r = Math.hypot(p.x, p.y);
  const zw = p.z + L3 - H_SHOULDER;                           // wrist height above the shoulder
  const c3 = (r * r + zw * zw - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  if (c3 < IK_MIN_COS3 || c3 > IK_MAX_COS3) return { status: 'unreachable', q: [0, 0, 0, 0, 0, 0] };
  const j3 = Math.acos(c3);                                    // elbow-up solution
  const j2 = Math.atan2(r, zw) - Math.atan2(L2 * Math.sin(j3), L1 + L2 * Math.cos(j3));
  const j1d = Math.atan2(p.y, p.x) * DEG, j2d = j2 * DEG, j3d = j3 * DEG;
  const q = [j1d, j2d, j3d, 0, 180 - j2d - j3d, p.roll - j1d];
  return { status: withinLimits(q) ? 'ok' : 'limit', q };
}

export function fk(q: number[]): Pose {
  const s2 = q[1] * RAD, s23 = (q[1] + q[2]) * RAD;
  const r = L1 * Math.sin(s2) + L2 * Math.sin(s23);
  const zw = L1 * Math.cos(s2) + L2 * Math.cos(s23);
  const y1 = q[0] * RAD;
  return { x: r * Math.cos(y1), y: r * Math.sin(y1), z: zw + H_SHOULDER - L3, roll: q[5] + q[0] };
}
