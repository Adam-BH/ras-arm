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


FW = _Namespace(parse_config_h())


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
    """Simulation only: nothing here exists in the firmware. mm, deg, ms, kg, N unless the name says."""
    dt_ms: float = 1.0                     # physics step (1 kHz, small cubes need it)
    tick_ms: int = 5                       # firmware loop period (5 physics steps)
    solver_iterations: int = 60            # 30…150 give the same results in the tests; 60 keeps a margin

    # the cube and the cell (same sizes as the web twin and the Gazebo cell)
    cube_size: float = 25.0
    cube_mass: float = 0.012               # 25 mm PLA cube, ~20 % infill
    cube_friction: float = 0.9
    cube_rgba: dict = field(default_factory=lambda: {"RED": (0.85, 0.13, 0.12, 1), "BLUE": (0.12, 0.32, 0.86, 1)})
    plate: tuple = (60.0, 0.0, 560.0, 640.0, 10.0)        # centre x, y and size x, y, thickness
    cradle_wall: tuple = (40.0, 8.0, 16.0)                # x, y, z of each U wall, centred at y = ±20
    bin_size: float = 110.0
    bin_wall: float = 4.0
    bin_height: float = 60.0

    # the arm (masses and torques: ros2/ras_arm_description/config/hardware.yaml on feature/ros2-twin)
    mass: dict = field(default_factory=lambda: {"base_link": 1.5, "turret": 1.1, "upper_arm": 0.25, "forearm": 0.12,
                                                "forearm_roll": 0.15, "wrist": 0.09, "tool": 0.12, "finger": 0.01})
    effort: dict = field(default_factory=lambda: {"J1": 0.40, "J2": 6.2, "J3": 2.0, "J4": 2.1, "J5": 2.1, "J6": 1.08})
    finger_force: float = 12.0             # N per finger: MG996R through the linkage, stalls on the cube
    finger_len: float = 44.0
    finger_beyond: float = 7.0             # fingertips reach this far below the firmware's tip
    finger_pad: float = 1.0                # pads sit 1 mm inside the servo's nominal opening, so
                                           # GRIP_CLOSED_MM (26) presses the 25 mm cube like the real
                                           # servo stalling on it (the Gazebo twin squeezes the same way)
    finger_friction: float = 1.2
    hard_stop_margin: float = 3.0          # deg past each homing switch before the mechanical stop
    servo_dps: float = 400.0               # DS3218 / MG996R no-load speed (the firmware ramps slower)
    grip_mm_s: float = 150.0               # finger opening speed of the gripper servo + linkage
    # E-stop cuts the motor rail: what still holds each joint (detent / gearbox friction, N·m)
    unpowered_hold: tuple = (0.02, 0.35, 0.12, 0.05, 0.05, 0.03)

    # sensors
    ir_beam_z: float = 12.5                # E18 beam across the cradle pocket, mm above the table
    colour_ambient: tuple = (60.0, 65.0, 70.0)            # RGB counts with nothing on the sensor
    colour_led_gain: float = 700.0         # counts for a white surface touching the sensor
    colour_falloff_mm: float = 8.0         # reflected light ∝ 1 / (1 + (d / falloff)²)
    colour_noise: float = 0.02             # 2 % per channel and sample

    # overhead camera (same placement as the web twin's vision camera, feature/vision-sort)
    camera: dict = field(default_factory=lambda: {"x": 205.0, "y": 0.0, "z": 500.0, "fov_deg": 33.4,
                                                  "width": 400, "height": 300})


SIM = SimParams()
