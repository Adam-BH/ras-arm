"""The arm's URDF, generated from the active profile (config.FW: lengths, limits) and SIM (shapes, masses).

The design book's arm (RAS_Arm_Design_Book.pdf, Fig. 4 and Sec. 2.4, masses from Table 3):
base housing (J1 motor underneath) → turntable → 300 mm tower in two parts → shoulder fork with the J2 motor
+ PG518 on the tower top (62 mm from the shoulder axis, belt 16T → 88T) and the J3 motor on the shoulder axis
(belt 16T → 104T along the upper arm) → upper arm 32 × 42 box, Ø48 bosses → forearm 26 × 34 box with the J4
servo, forearm roll at 140 mm on 2 × 608ZZ → J5 servo at the wrist → wrist link → J6 servo → gripper (MG996R,
gears A/B, parallel fingers with V-groove pads).

Same chain, names, signs and zero pose as the ROS 2 twin (feature/ros2-twin): a joint position is the
firmware angle in rad, a finger is the opening / 2 in m. Zero pose = everything straight up, facing +x.

    J1 yaw (z) → turret → J2 pitch (y) at H → upper arm L1 → J3 pitch (y) → forearm, J4 roll (z) at d4
       → J5 pitch (y) at L2 → wrist → J6 roll (−z) at L3 − gripper → tool → tool_tip at L3 + 2 fingers (±y)

The stepper joints get a hard stop SIM.hard_stop_margin past their range, so a homing switch (which
clicks at the end of the range) is reached before the mechanics stop the joint.
"""
from __future__ import annotations

import math
from pathlib import Path

from .config import FW, SIM

MATERIALS = {
    "printed": (0.91, 0.86, 0.79), "printed_dark": (0.80, 0.74, 0.66), "stepper": (0.25, 0.26, 0.28),
    "servo": (0.10, 0.10, 0.12), "pulley": (0.93, 0.72, 0.62), "belt": (0.08, 0.08, 0.08),
    "finger": (0.80, 0.82, 0.85), "pad": (0.30, 0.30, 0.32), "metal": (0.62, 0.64, 0.68),
}   # colours stay out of the vision HSV classes (no saturated red / blue on the arm)
MM = 0.001


def _f(v: float) -> str:
    return f"{v:.6g}"


def _xyz(*v) -> str:
    return " ".join(_f(c) for c in v)


def _box(size, xyz=(0, 0, 0), rpy=(0, 0, 0)):
    """size and xyz in mm"""
    return (f'<origin xyz="{_xyz(*(c * MM for c in xyz))}" rpy="{_xyz(*rpy)}"/>'
            f'<geometry><box size="{_xyz(*(c * MM for c in size))}"/></geometry>')


def _cyl(d, length, xyz=(0, 0, 0), axis="z"):
    """diameter, length and xyz in mm; axis x, y or z"""
    rpy = {"z": (0, 0, 0), "y": (math.pi / 2, 0, 0), "x": (0, math.pi / 2, 0)}[axis]
    return (f'<origin xyz="{_xyz(*(c * MM for c in xyz))}" rpy="{_xyz(*rpy)}"/>'
            f'<geometry><cylinder radius="{_f(d / 2 * MM)}" length="{_f(length * MM)}"/></geometry>')


def _inertial(m, size, com):
    """box-shaped inertia of mass m (kg), size and com in mm"""
    x, y, z = (c * MM for c in size)
    return (f'<inertial><origin xyz="{_xyz(*(c * MM for c in com))}"/><mass value="{_f(m)}"/>'
            f'<inertia ixx="{_f(m * (y * y + z * z) / 12)}" iyy="{_f(m * (x * x + z * z) / 12)}" '
            f'izz="{_f(m * (x * x + y * y) / 12)}" ixy="0" ixz="0" iyz="0"/></inertial>')


def _link(name, visuals, collisions, inertial):
    vis = "".join(f'<visual>{g}<material name="{mat}"/></visual>' for g, mat in visuals)
    col = "".join(f"<collision>{g}</collision>" for g in collisions)
    return f'<link name="{name}">{vis}{col}{inertial}</link>'


def _revolute(name, parent, child, xyz_mm, axis, j):
    lo, hi = FW.JOINT_MIN[j], FW.JOINT_MAX[j]
    if j < FW.N_STEPPERS:
        lo, hi = lo - SIM.hard_stop_margin, hi + SIM.hard_stop_margin
    vel = math.radians(max(FW.JOINT_MAX_DPS[j], SIM.servo_dps if j >= FW.N_STEPPERS else 0) * 1.5)
    return (f'<joint name="{name}" type="revolute"><parent link="{parent}"/><child link="{child}"/>'
            f'<origin xyz="{_xyz(*(c * MM for c in xyz_mm))}"/><axis xyz="{axis}"/>'
            f'<limit lower="{_f(math.radians(lo))}" upper="{_f(math.radians(hi))}" velocity="{_f(vel)}" '
            f'effort="{_f(SIM.effort[name])}"/><dynamics damping="0.02" friction="0"/></joint>')


def _belt(c1, r1, c2, r2, y, width=6.0, thick=1.5):
    """the two straight strands of a belt between pulleys at c1, c2 = (x, z) in mm, in the plane y = const"""
    dx, dz = c2[0] - c1[0], c2[1] - c1[1]
    d = math.hypot(dx, dz)
    base, phi = math.atan2(dz, dx), math.asin((r2 - r1) / d)
    out = []
    for s in (1, -1):                                   # external tangents, one on each side
        a = base + s * (math.pi / 2 + phi)
        p1 = (c1[0] + r1 * math.cos(a), c1[1] + r1 * math.sin(a))
        p2 = (c2[0] + r2 * math.cos(a), c2[1] + r2 * math.sin(a))
        length = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        pitch = math.atan2(p2[0] - p1[0], p2[1] - p1[1])   # rotation about y that turns +z onto the strand
        mid = ((p1[0] + p2[0]) / 2, y, (p1[1] + p2[1]) / 2)
        out.append((_box((thick, width, length), mid, (0, pitch, 0)), "belt"))
    return out


def arm_urdf() -> str:
    H, L1, L2 = FW.H_SHOULDER, FW.L1, FW.L2
    d4, d6, G = SIM.d4, SIM.d6, SIM.gripper_len
    B = SIM.boss_d
    ua_x, ua_y = SIM.upper_arm_section
    fa_x, fa_y = SIM.forearm_section
    p88, p104 = SIM.pulley_d
    hx, hy, hz = SIM.base_housing
    tt_d, tt_h = SIM.turntable
    FL, FB = SIM.finger_len, SIM.finger_beyond
    side = ua_y / 2 + 18                                  # y of the belt planes, beside the forks

    parts = ['<?xml version="1.0"?>',
             "<!-- GENERATED by pysim/ras_sim/robot_urdf.py (design book layout, lengths from the active profile) -->",
             '<robot name="ras_arm">']
    parts += [f'<material name="{k}"><color rgba="{_xyz(*v)} 1"/></material>' for k, v in MATERIALS.items()]

    # ── base: housing with the J1 motor underneath (P01) ──
    housing = _box((hx, hy, hz), (0, 0, hz / 2))
    parts.append(_link("base_link", [(housing, "printed_dark")], [housing],
                       _inertial(SIM.m_base, (hx, hy, hz), (0, 0, hz / 2))))
    parts.append(_revolute("J1", "base_link", "turret", (0, 0, 0), "0 0 1", FW.J1))

    # ── turret: turntable (P02), tower in two parts (P03), shoulder fork, J2 motor + PG518, J3 motor ──
    t0, t1 = hz + tt_h, H - 30
    tm = (t0 + t1) / 2
    lower, upper = _box((80, 70, tm - t0), (0, 0, (t0 + tm) / 2)), _box((58, 50, t1 - tm), (0, 0, (tm + t1) / 2))
    mx, mz = SIM.j2_motor_offset[0], H - SIM.j2_motor_offset[1]           # J2 motor shaft, 62 mm from the shoulder
    vis = [(_cyl(tt_d, tt_h, (0, 0, hz + tt_h / 2)), "printed"), (lower, "printed_dark"), (upper, "printed_dark")]
    for s in (1, -1):                                                     # fork cheeks around the shoulder boss
        vis.append((_box((B + 12, 6, 64), (0, s * (ua_y / 2 + 6), H - 12)), "printed_dark"))
    vis += [(_cyl(10, 10, (mx, side, mz), "y"), "metal"),                 # 16T on the PG518 output
            (_cyl(36, 32, (mx, side + 24, mz), "y"), "stepper"),          # PG518 gearbox
            (_box((42, 40, 42), (mx, side + 60, mz)), "stepper"),         # J2 17HS4401S
            (_box((50, 6, 50), (mx, side + 4, mz)), "printed_dark"),      # motor plate on the tower top
            (_cyl(10, 10, (0, -side, H), "y"), "metal"),                  # 16T on the J3 motor, on the J2 axis
            (_box((42, 40, 42), (0, -side - 26, H)), "stepper"),          # J3 17HS4401 on the shoulder axis
            ]
    vis += _belt((mx, mz), 5.0, (0, H), p88 / 2, side)                    # J2 belt: both pulleys fixed here
    parts.append(_link("turret", vis, [lower, upper, _cyl(tt_d, tt_h, (0, 0, hz + tt_h / 2))],
                       _inertial(SIM.m_turret, (100, 100, H), (0, 0, 0.55 * H))))

    # ── J2 shoulder → upper arm (P06) with the 88T pulley; the J3 belt runs along it ──
    parts.append(_revolute("J2", "turret", "upper_arm", (0, 0, H), "0 1 0", FW.J2))
    arm = _box((ua_x, ua_y, L1 - B), (0, 0, L1 / 2))
    vis = [(_cyl(B, ua_y + 4, axis="y"), "printed"), (arm, "printed"),
           (_cyl(p88, 8, (0, side, 0), "y"), "pulley")]
    for s in (1, -1):                                                     # elbow fork
        vis.append((_box((B, 5, B + 8), (0, s * (fa_y / 2 + 5), L1)), "printed"))
    vis += _belt((0, 0), 5.0, (0, L1), p104 / 2, -side)                   # J3 belt: 16T on the J2 axis → 104T
    m_ua = SIM.rho_upper * L1 * MM + 0.05                                 # ρ in kg/m, lengths in mm
    parts.append(_link("upper_arm", vis, [arm], _inertial(m_ua, (ua_x, ua_y, L1), (0, 0, 0.45 * L1))))

    # ── J3 elbow → forearm root (P07): elbow boss, 104T pulley, J4 servo, J4 bearing housing ──
    parts.append(_revolute("J3", "upper_arm", "forearm", (0, 0, L1), "0 1 0", FW.J3))
    root = _box((fa_x, fa_y, d4 - B / 2 - 20), (0, 0, (B / 2 + d4 - 20) / 2))
    vis = [(_cyl(B, fa_y + 4, axis="y"), "printed"), (_cyl(p104, 8, (0, -side, 0), "y"), "pulley"), (root, "printed"),
           (_box((20, 40, 40), (0, 0, d4 - 42)), "servo"),                 # J4 DS3218MG
           (_cyl(40, 20, (0, 0, d4 - 10)), "printed")]                     # 2 × 608ZZ housing
    m_fa = SIM.m_elbow + SIM.rho_fore * d4 * MM + SIM.m_ds3218
    com = (SIM.rho_fore * d4 * MM * d4 / 2 + SIM.m_ds3218 * (d4 - 42)) / m_fa
    parts.append(_link("forearm", vis, [root], _inertial(m_fa, (fa_x, fa_y, d4), (0, 0, com))))

    # ── J4 forearm roll → forearm tip (P08 hub + P09) with the J5 servo at the wrist ──
    parts.append(_revolute("J4", "forearm", "forearm_roll", (0, 0, d4), "0 0 1", FW.J4))
    tip_len = L2 - d4
    tip = _box((fa_x, fa_y, max(tip_len - 32, 4)), (0, 0, 12 + max(tip_len - 32, 4) / 2))
    vis = [(_cyl(36, 12, (0, 0, 6)), "printed"), (tip, "printed"),
           (_box((40, 20, 40), (0, -(fa_y / 2 + 10), tip_len)), "servo"),  # J5 DS3218MG, shaft on the J5 axis
           (_box((40, 4, 40), (0, fa_y / 2 + 2, tip_len)), "printed")]     # other cheek of the wrist fork
    m_fr = SIM.rho_fore * tip_len * MM + 0.02 + SIM.m_ds3218
    parts.append(_link("forearm_roll", vis, [tip],
                       _inertial(m_fr, (fa_x, fa_y + 20, tip_len), (0, -6, 0.7 * tip_len))))

    # ── J5 wrist pitch → wrist link (P10), J6 servo at its end ──
    parts.append(_revolute("J5", "forearm_roll", "wrist", (0, 0, tip_len), "0 1 0", FW.J5))
    link_len = max(d6 - 54, 4)
    wl = _box((26, 26, link_len), (0, 0, 16 + link_len / 2))
    vis = [(_cyl(32, fa_y + 4, axis="y"), "printed"), (wl, "printed"),
           (_box((20, 40, 38), (0, 0, d6 - 19)), "servo")]                 # J6 MG996R, shaft along the tool
    m_w = 0.04 + SIM.m_mg996r
    parts.append(_link("wrist", vis, [wl], _inertial(m_w, (26, 40, d6), (0, 0, 0.7 * d6))))

    # ── J6 wrist roll → gripper (P11, P12): flange, base, gripper MG996R, gears A / B ──
    # J6 axis −z: with the gripper pointing down, + J6 turns the fingers + in world yaw (yaw = J1 + J6 = roll)
    parts.append(_revolute("J6", "wrist", "tool", (0, 0, d6), "0 0 -1", FW.J6))
    g0 = G - FL                                                            # finger roots
    base = _box((30, 76, 6), (0, 0, g0 - 3))
    servo = _box((20, 40, 38), (0, 0, g0 - 6 - 19))
    vis = [(_cyl(36, 4, (0, 0, 2)), "metal"), (_box((26, 26, max(g0 - 50, 2)), (0, 0, 4 + max(g0 - 50, 2) / 2)), "printed"),
           (base, "printed"), (servo, "servo")]
    for s in (1, -1):                                                      # gears A (servo) and B, 1:1
        vis.append((_cyl(24, 6, (0, s * 12, g0 + 6), "x"), "pulley"))
    m_t = SIM.m_gripper - SIM.m_mg996r - 2 * SIM.finger_mass
    parts.append(_link("tool", vis, [base, servo], _inertial(m_t, (30, 76, G), (0, 0, 0.5 * G))))

    # the firmware's "tip" (x, y, z, roll in MOVE / telemetry): between the fingertips
    parts.append('<joint name="tool_tip_joint" type="fixed"><parent link="tool"/><child link="tool_tip"/>'
                 f'<origin xyz="0 0 {_f(G * MM)}"/></joint>')
    parts.append('<link name="tool_tip"><inertial><mass value="0.001"/>'
                 '<inertia ixx="1e-9" iyy="1e-9" izz="1e-9" ixy="0" ixz="0" iyz="0"/></inertial></link>')

    # ── fingers (P13): parallel jaws, V-groove pads on the inner faces ──
    fz = FB - FL / 2
    for name, sign in (("left", 1), ("right", -1)):
        geo = _box((14, 8, FL), (0, sign * 4, fz))
        pad = _box((10, 1.5, FL * 0.6), (0, sign * 0.75, FB - FL * 0.3))
        parts.append(_link(f"finger_{name}", [(geo, "finger"), (pad, "pad")], [geo],
                           _inertial(SIM.finger_mass, (14, 8, FL), (0, sign * 4, fz))))
        parts.append(f'<joint name="grip_{name}" type="prismatic"><parent link="tool"/><child link="finger_{name}"/>'
                     f'<origin xyz="0 0 {_f(G * MM)}"/><axis xyz="0 {sign} 0"/>'
                     f'<limit lower="0" upper="{_f(FW.JOINT_MAX[FW.JG] / 2 * MM)}" velocity="0.2" '
                     f'effort="{_f(SIM.finger_force)}"/><dynamics damping="0.5" friction="0"/></joint>')
    parts.append("</robot>")
    return "\n".join(parts) + "\n"


def write_arm_urdf(path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(arm_urdf(), encoding="utf-8")
    return path
