"""Port of firmware/src/4_motion/kinematics.c (same as sim/src/core/kinematics.ts), docs/kinematics.md.

x forward (cradle), y left, z up, origin = base axis on the table. Degrees and mm.
J1 yaw (0 = cradle), J2 from vertical (+ forward), J3 relative to the upper arm (0 = straight),
J5 = 180 − J2 − J3 keeps the gripper vertical, J6 = roll − J1, J4 = 0.
Tested against the golden vectors of tools/ik_reference.py (tests/test_kinematics.py).
"""
from __future__ import annotations

import math

from .config import FW, Pose

OK, UNREACHABLE, LIMIT = "ok", "unreachable", "limit"


def within_limits(q) -> bool:
    """J1..J6 inside JOINT_MIN / JOINT_MAX (the gripper is clamped, never judged)"""
    return all(FW.JOINT_MIN[j] - 0.01 <= q[j] <= FW.JOINT_MAX[j] + 0.01 for j in range(6))


def ik(p: Pose) -> tuple[str, list[float]]:
    r = math.hypot(p.x, p.y)
    zw = p.z + FW.L3 - FW.H_SHOULDER                            # wrist height above the shoulder axis
    c3 = (r * r + zw * zw - FW.L1 ** 2 - FW.L2 ** 2) / (2 * FW.L1 * FW.L2)
    if c3 < FW.IK_MIN_COS3 or c3 > FW.IK_MAX_COS3:
        return UNREACHABLE, [0.0] * 6
    j3 = math.acos(c3)                                          # elbow-up solution
    j2 = math.atan2(r, zw) - math.atan2(FW.L2 * math.sin(j3), FW.L1 + FW.L2 * math.cos(j3))
    j1d, j2d, j3d = math.degrees(math.atan2(p.y, p.x)), math.degrees(j2), math.degrees(j3)
    q = [j1d, j2d, j3d, 0.0, 180.0 - j2d - j3d, p.roll - j1d]
    return (OK if within_limits(q) else LIMIT), q


def fk(q) -> Pose:
    s2, s23 = math.radians(q[1]), math.radians(q[1] + q[2])
    r = FW.L1 * math.sin(s2) + FW.L2 * math.sin(s23)
    zw = FW.L1 * math.cos(s2) + FW.L2 * math.cos(s23)
    y1 = math.radians(q[0])
    return Pose(r * math.cos(y1), r * math.sin(y1), zw + FW.H_SHOULDER - FW.L3, q[5] + q[0])
