"""Every robot number comes from firmware/src/config.h, parsed at import time (no copy to keep in sync).

    from ras_sim.config import FW, SIM
    FW.L1, FW.JOINT_MIN[2], FW.GRIP_CLOSED_MM ...      # firmware values, firmware units (mm, deg, ms)
    SIM.cube_mass ...                                   # simulation-only values (physics, camera, faults)

The parser understands what config.h uses: #define NAME <arithmetic>, static const arrays,
enums and #ifdef / #ifndef / #else / #endif.
"""
from __future__ import annotations

import ast
import operator
import os
import re
from dataclasses import dataclass, field
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
CONFIG_H = REPO_ROOT / "firmware" / "src" / "config.h"

_OPS = {ast.Add: operator.add, ast.Sub: operator.sub, ast.Mult: operator.mul, ast.Div: operator.truediv,
        ast.USub: operator.neg, ast.UAdd: operator.pos, ast.Mod: operator.mod,
        ast.LShift: operator.lshift, ast.RShift: operator.rshift, ast.BitOr: operator.or_}


def _c_to_py(expr: str) -> str:
    expr = re.sub(r"\b(0x[0-9A-Fa-f]+|\d+)(UL|LU|U|L)\b", r"\1", expr)          # 115200UL → 115200
    expr = re.sub(r"(\d+\.\d*|\.\d+|\d+)([eE][-+]?\d+)?f\b", r"\1\2", expr)      # 180.0f → 180.0
    return expr.strip()


def _eval(expr: str, names: dict):
    def ev(node):
        if isinstance(node, ast.Expression):
            return ev(node.body)
        if isinstance(node, ast.Constant) and isinstance(node.value, (int, float, str)):
            return node.value
        if isinstance(node, ast.Name):
            return names[node.id]
        if isinstance(node, ast.BinOp) and type(node.op) in _OPS:
            return _OPS[type(node.op)](ev(node.left), ev(node.right))
        if isinstance(node, ast.UnaryOp) and type(node.op) in _OPS:
            return _OPS[type(node.op)](ev(node.operand))
        raise ValueError(f"unsupported C expression: {expr!r}")
    return ev(ast.parse(_c_to_py(expr), mode="eval"))


def parse_config_h(path: Path = CONFIG_H, defined: set[str] | None = None) -> dict:
    """returns {NAME: value} for every #define, const array (as list) and enum member"""
    defined = set(defined or ())
    text = path.read_text(encoding="utf-8")
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    lines = [re.sub(r"//.*", "", ln) for ln in text.splitlines()]

    out: dict = {}
    active = [True]                       # #if stack
    body = []                             # lines that survive the preprocessor
    for ln in lines:
        s = ln.strip()
        if m := re.match(r"#\s*ifdef\s+(\w+)", s):
            active.append(active[-1] and (m[1] in defined or m[1] in out)); continue
        if m := re.match(r"#\s*ifndef\s+(\w+)", s):
            active.append(active[-1] and not (m[1] in defined or m[1] in out)); continue
        if re.match(r"#\s*else\b", s):
            active[-1] = active[-2] and not active[-1]; continue
        if re.match(r"#\s*endif\b", s):
            active.pop(); continue
        if not active[-1]:
            continue
        if m := re.match(r"#\s*define\s+(\w+)\s+(.+)$", s):
            try:
                out[m[1]] = _eval(m[2], out)
            except (ValueError, KeyError, SyntaxError):
                pass                      # not a number (e.g. a type): not needed by the sim
            continue
        body.append(ln)

    src = "\n".join(body)
    for m in re.finditer(r"enum\s*\{([^}]*)\}", src):
        k = 0
        for name in (n.strip() for n in m[1].split(",")):
            if not name:
                continue
            if "=" in name:
                name, val = (t.strip() for t in name.split("=", 1))
                k = int(_eval(val, out))
            out[name] = k
            k += 1
    for m in re.finditer(r"static\s+const\s+\w+\s+(\w+)\s*\[[^\]]*\]\s*=\s*\{([^}]*)\}\s*;", src, flags=re.S):
        out[m[1]] = [_eval(v, out) for v in m[2].split(",") if v.strip()]
    return out


class _Namespace:
    def __init__(self, values: dict):
        self.__dict__.update(values)

    def __repr__(self):
        return f"<config.h: {len(self.__dict__)} values>"


# The design book (RAS_Arm_Design_Book.pdf, 16 Nov 2026) lists, in appendix A.1, the firmware constants
# still to update in config.h before flashing. Profile "design" (the default) applies them on top of
# config.h, so the twin runs the firmware as it will be; profile "firmware" uses config.h untouched.
DESIGN_BOOK = {
    "L1": 165.0, "L2": 190.0,                          # Table 3 (adopted design, 355 mm reach)
    "L3": 200.0,                                       # T: wrist pitch → fingertip = 100 link + 100 gripper
    "H_SHOULDER": 300.0,                               # base tower
    "SPD_J2": 200.0 * 8 * 5.18 * 5.5 / 360.0,          # 126.62: PG518 × belt 16T→88T, 1/8 (Table 4)
    "SPD_J3": 200.0 * 8 * 6.5 / 360.0,                 #  28.89: belt 16T→104T, 1/8
    "SERVO_CH_J4": 3, "SERVO_CH_J5": 0, "SERVO_CH_J6": 1, "SERVO_CH_GRIP": 2,   # rev F (Table 11)
    "CELL_GRIP_Z": 37.5, "CELL_DROP_Z": 88.0,          # Table 4 stations (HOME and SAFE unchanged)
}
PROFILES = ("design", "firmware")
PROFILE = os.environ.get("RAS_SIM_PROFILE", "design")
if PROFILE not in PROFILES:
    raise ValueError(f"RAS_SIM_PROFILE must be one of {PROFILES}, not {PROFILE!r}")

FW_RAW = _Namespace(parse_config_h())                  # config.h exactly as it is
FW = _Namespace({**FW_RAW.__dict__, **(DESIGN_BOOK if PROFILE == "design" else {})})
FW.SERVO_CH = [FW.SERVO_CH_J4, FW.SERVO_CH_J5, FW.SERVO_CH_J6, FW.SERVO_CH_GRIP]


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    z: float
    roll: float = 0.0


# the four stations (docs/kinematics.md), from config.h
CELL = {
    "HOME": Pose(FW.CELL_HOME_X, 0.0, FW.CELL_HOME_Z),
    "CUBE": Pose(FW.CELL_CRADLE_X, 0.0, FW.CELL_GRIP_Z),
    "BIN_A": Pose(0.0, FW.CELL_BIN_Y, FW.CELL_DROP_Z),       # red
    "BIN_B": Pose(0.0, -FW.CELL_BIN_Y, FW.CELL_DROP_Z),      # blue
}


@dataclass
class SimParams:
    """Simulation only: nothing here exists in the firmware. mm, deg, ms, kg, N unless the name says.
    Defaults = the design book (RAS_Arm_Design_Book.pdf): Table 3 masses, Fig. 4 layout, Sec. 2.4 sections."""
    dt_ms: float = 1.0                     # physics step (1 kHz, small cubes need it)
    tick_ms: int = 5                       # firmware loop period (5 physics steps)
    solver_iterations: int = 60            # 30…150 give the same results in the tests; 60 keeps a margin

    # the object and the cell
    cube_size: float = 35.0                # design object: 35 mm PLA cube (Sec. 1.3)
    cube_mass: float = 0.021               # 21 g
    cube_friction: float = 0.5             # × finger pads 0.8 = µ 0.4, PLA on PLA (Sec. 3.2.1)
    cube_rgba: dict = field(default_factory=lambda: {"RED": (0.85, 0.13, 0.12, 1), "BLUE": (0.12, 0.32, 0.86, 1)})
    plate: tuple = (60.0, 0.0, 560.0, 640.0, 10.0)        # baseplate: centre x, y and size x, y, thickness
    cradle_top: float = 20.0               # the cube sits on the V-cradle (P14), 20 mm above the table
    cradle_block: tuple = (70.0, 70.0)     # x, y
    cradle_fence: tuple = (4.0, 8.0)       # thickness, height of the walls that centre the cube
    cradle_clear: float = 2.0              # free play around the cube in the cradle, per side
    bin_size: float = 110.0                # ≥ twice the object (Sec. 2.6)
    bin_wall: float = 4.0
    bin_height: float = 60.0

    # the arm: layout from Fig. 4, sections from Sec. 2.4
    base_housing: tuple = (140.0, 140.0, 45.0)            # P01: J1 motor underneath
    turntable: tuple = (130.0, 6.0)                       # P02: diameter, thickness (4 × 608ZZ)
    upper_arm_section: tuple = (42.0, 32.0)               # P06: 32 × 42 box (42 in the bending plane)
    forearm_section: tuple = (34.0, 26.0)                 # P07 / P09: 26 × 34 box
    boss_d: float = 48.0                                  # joint bosses around the 608ZZ
    d4: float = 140.0                                     # elbow → J4 forearm roll
    gripper_len: float = 100.0                            # J6 → fingertip (so J5 → J6 = L3 − 100)
    pulley_d: tuple = (56.0, 66.0)                        # P04 shoulder 88T, P05 elbow 104T
    j2_motor_offset: tuple = (-40.0, 47.0)                # J2 motor on the tower top: 62 mm from the shoulder
    # masses (Table 3): ρ1 0.743 kg/m, ρ2 0.546 kg/m, elbow 97 g, DS3218MG 60 g, gripper + 2 MG996R 180 g
    rho_upper: float = 0.743
    rho_fore: float = 0.546
    m_elbow: float = 0.097
    m_ds3218: float = 0.060
    m_mg996r: float = 0.055
    m_gripper: float = 0.180               # gripper + J6 and gripper MG996R
    m_turret: float = 1.30                 # turntable, tower, J2 motor + PG518, J3 motor
    m_base: float = 1.5
    finger_mass: float = 0.010
    finger_force: float = 15.0             # N per finger: 1.84 N needed (Sec. 3.2.1), MG996R stalls on the cube
    finger_len: float = 50.0
    finger_beyond: float = 0.0             # the firmware's tip IS the fingertip (T = wrist pitch → fingertip)
    finger_pad: float = 0.0                # mm the pads sit inside the servo's nominal opening
    finger_friction: float = 0.8           # V-groove pads
    hard_stop_margin: float = 3.0          # deg past each homing switch before the mechanical stop
    self_collision: bool = True            # links (except a joint's two sides) collide: a limp arm lands on itself
    servo_dps: float = 400.0               # DS3218 / MG996R no-load speed (the firmware ramps slower)
    grip_mm_s: float = 150.0               # finger opening speed of the gripper servo + linkage
    stepper_rated: float = 0.49            # 17HS4401 holding torque, N·m (Table 5)
    stepper_detent: float = 0.02           # unpowered detent torque, N·m at the motor
    servo_rated: tuple = (1.96, 1.96, 1.08)                # J4, J5 DS3218MG; J6 MG996R (Table 5)

    # sensors
    ir_beam_z: float = 30.0                # E18 beside the cradle, horizontal, at the cube's lower half
    colour_ambient: tuple = (60.0, 65.0, 70.0)            # RGB counts with nothing on the sensor
    colour_led_gain: float = 700.0         # counts for a white surface touching the sensor
    colour_falloff_mm: float = 8.0         # reflected light ∝ 1 / (1 + (d / falloff)²)
    colour_noise: float = 0.02             # 2 % per channel and sample

    # overhead camera (same placement as the web twin's vision camera, feature/vision-sort)
    camera: dict = field(default_factory=lambda: {"x": 205.0, "y": 0.0, "z": 500.0, "fov_deg": 33.4,
                                                  "width": 400, "height": 300})

    # ── derived from the transmissions in FW (so both profiles stay consistent) ──
    def ratio(self, j: int) -> float:
        """stepper joint j: motor turns per joint turn, from the steps per degree"""
        spd, micro = (FW.SPD_J1, FW.SPD_J2, FW.SPD_J3)[j], (16, 8, 8)[j]
        return spd * 360.0 / (200.0 * micro)

    @property
    def effort(self) -> dict:
        """rated torque at each joint, N·m (Table 5): stepper × ratio × efficiency (0.9 planetary, 0.95 belt)"""
        eff = (1.0, 0.9 * 0.95, 0.95)
        out = {f"J{j + 1}": self.stepper_rated * self.ratio(j) * eff[j] for j in range(3)}
        out.update({f"J{j + 4}": t for j, t in enumerate(self.servo_rated)})
        return out

    @property
    def unpowered_hold(self) -> tuple:
        """what still holds each joint with the motor rail cut: detent × ratio + friction (N·m)"""
        steppers = tuple(self.stepper_detent * self.ratio(j) + f for j, f in enumerate((0.0, 0.05, 0.02)))
        return steppers + (0.05, 0.05, 0.03)

    @property
    def d6(self) -> float:
        """J5 wrist pitch → J6 wrist roll along the tool"""
        return FW.L3 - self.gripper_len


# the old cell, for profile "firmware" (config.h as it is: 25 mm cube on the table, 110 mm tool)
FIRMWARE_CELL = dict(cube_size=25.0, cube_mass=0.012, cube_friction=0.9, cradle_top=0.0, cradle_fence=(4.0, 16.0),
                     d4=100.0, gripper_len=70.0, finger_len=44.0, finger_beyond=7.0, finger_pad=1.0,
                     finger_friction=1.2, ir_beam_z=12.5,
                     self_collision=False)               # the design shapes do not fit the short old arm

SIM = SimParams() if PROFILE == "design" else SimParams(**FIRMWARE_CELL)
