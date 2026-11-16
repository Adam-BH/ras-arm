"""Port of firmware/src/4_motion/kinematics.c (same as sim/src/core/kinematics.ts), docs/kinematics.md.

x forward (cradle), y left, z up, origin = base axis on the table. Degrees and mm.
J1 yaw (0 = cradle), J2 from vertical (+ forward), J3 relative to the upper arm (0 = straight),
J5 = 180 − J2 − J3 keeps the gripper vertical, J6 = roll − J1, J4 = 0.
The geometry is the active profile's (config.FW); tests also run it on config.h untouched (FW_RAW) against
the golden vectors of tools/ik_reference.py, and on the design book against its Table 4.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .config import FW, Pose

OK, UNREACHABLE, LIMIT = "ok", "unreachable", "limit"


@dataclass(frozen=True)
class Geometry:
    L1: float
    L2: float
    L3: float
    H: float
    cos3_max: float
    cos3_min: float
    jmin: tuple
    jmax: tuple

    @classmethod
    def of(cls, ns) -> "Geometry":
        return cls(ns.L1, ns.L2, ns.L3, ns.H_SHOULDER, ns.IK_MAX_COS3, ns.IK_MIN_COS3,
                   tuple(ns.JOINT_MIN), tuple(ns.JOINT_MAX))


GEOM = Geometry.of(FW)


def within_limits(q, g: Geometry = GEOM) -> bool:
    """J1..J6 inside JOINT_MIN / JOINT_MAX (the gripper is clamped, never judged)"""
    return all(g.jmin[j] - 0.01 <= q[j] <= g.jmax[j] + 0.01 for j in range(6))


def ik(p: Pose, g: Geometry = GEOM) -> tuple[str, list[float]]:
    r = math.hypot(p.x, p.y)
    zw = p.z + g.L3 - g.H                                       # wrist height above the shoulder axis
    c3 = (r * r + zw * zw - g.L1 ** 2 - g.L2 ** 2) / (2 * g.L1 * g.L2)
    if c3 < g.cos3_min or c3 > g.cos3_max:
        return UNREACHABLE, [0.0] * 6
    j3 = math.acos(c3)                                          # elbow-up solution
    j2 = math.atan2(r, zw) - math.atan2(g.L2 * math.sin(j3), g.L1 + g.L2 * math.cos(j3))
    j1d, j2d, j3d = math.degrees(math.atan2(p.y, p.x)), math.degrees(j2), math.degrees(j3)
    q = [j1d, j2d, j3d, 0.0, 180.0 - j2d - j3d, p.roll - j1d]
    return (OK if within_limits(q, g) else LIMIT), q


def fk(q, g: Geometry = GEOM) -> Pose:
    s2, s23 = math.radians(q[1]), math.radians(q[1] + q[2])
    r = g.L1 * math.sin(s2) + g.L2 * math.sin(s23)
    zw = g.L1 * math.cos(s2) + g.L2 * math.cos(s23)
    y1 = math.radians(q[0])
    return Pose(r * math.cos(y1), r * math.sin(y1), zw + g.H - g.L3, q[5] + q[0])
