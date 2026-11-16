"""Keyframe sequencer ("slides"), like the ICRA arm's: a list of poses the arm visits one after the other.

A slide is either joint angles (J1..J6, degrees, firmware convention: docs/kinematics.md) sent as MOVEJ,
or a tip pose (x, y, z mm, roll deg) sent as MOVE and solved by the firmware's own IK. The gripper
command runs BEFORE the arm moves in that slide (put "open" on the approach, "close" on the grasp).

    Slide(pose=(220, 0, 24, 0), gripper="keep", duration=1.2, label="down to the cube")
    Slide(joints=[90, 35.4, 88.8, 0, 55.8, -90], gripper="close", duration=2.0, label="over bin A")

Edit DEMO below, or pass a JSON file: [{"pose": [x, y, z, roll], "gripper": "open", "duration": 1.5}, …]
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

from .app import CommandError, Simulation, cmd, wait
from .config import FW
from .protocol import fmt


@dataclass
class Slide:
    pose: tuple | None = None         # (x, y, z, roll) mm / deg, through the firmware IK
    joints: list | None = None        # [J1..J6] deg
    gripper: str = "keep"             # "open" | "close" | "keep" | a number (mm)
    duration: float = 2.0             # s to reach this slide (the firmware may take longer: speed limits)
    label: str = "slide"
    pause_after: float = 0.3          # s to hold once there

    def __post_init__(self):
        if (self.pose is None) == (self.joints is None):
            raise ValueError(f"slide '{self.label}': give exactly one of pose or joints")
        if self.joints is not None and len(self.joints) != 6:
            raise ValueError(f"slide '{self.label}': joints needs 6 values (J1..J6)")
        if self.pose is not None and len(self.pose) not in (3, 4):
            raise ValueError(f"slide '{self.label}': pose is (x, y, z) or (x, y, z, roll)")
        if not (self.gripper in ("open", "close", "keep") or isinstance(self.gripper, (int, float))):
            raise ValueError(f"slide '{self.label}': gripper is open, close, keep or a width in mm")


C = FW.CELL_CRADLE_X
DEMO = [   # a pick from the cradle into bin A, then a wave, by hand
    Slide(pose=(160, 0, 170, 0), gripper="open", duration=1.5, label="home, gripper open"),
    Slide(pose=(C, 0, FW.SAFE_Z, 0), duration=1.5, label="above the cradle"),
    Slide(pose=(C, 0, FW.CELL_GRIP_Z, 0), duration=1.6, label="down to the cube"),
    Slide(pose=(C, 0, FW.CELL_GRIP_Z, 0), gripper="close", duration=0.3, label="grasp", pause_after=0.3),
    Slide(pose=(C, 0, FW.SAFE_Z, 0), duration=1.4, label="lift"),
    Slide(joints=[90, 25, 85, 0, 70, -90], duration=2.0, label="swing over bin A (joint move)"),
    Slide(pose=(0, FW.CELL_BIN_Y, FW.CELL_DROP_Z, 0), duration=1.2, label="lower into bin A"),
    Slide(pose=(0, FW.CELL_BIN_Y, FW.CELL_DROP_Z, 0), gripper="open", duration=0.3, label="release", pause_after=0.4),
    Slide(joints=[45, 10, 60, 0, 30, 0], duration=1.8, label="wave: up"),
    Slide(joints=[45, 10, 60, 60, 30, 0], duration=0.6, label="wave: roll J4 +60"),
    Slide(joints=[45, 10, 60, -60, 30, 0], duration=0.9, label="wave: roll J4 -60"),
    Slide(pose=(160, 0, 170, 0), duration=1.8, label="home"),
]


def load_slides(path: str | Path) -> list[Slide]:
    return [Slide(**d) for d in json.loads(Path(path).read_text(encoding="utf-8"))]


def play(sim: Simulation, slides: list[Slide], on_slide=None):
    st = sim.send("STATUS")
    if not st.text.startswith(("IDLE ", "TEACH ")) or "running=1" in st.text:
        raise CommandError(f"slides need IDLE or TEACH, not running ({st.text})")
    for i, s in enumerate(slides, 1):
        if on_slide:
            on_slide(i, len(slides), s)
        if s.gripper != "keep":
            mm = {"open": FW.GRIP_OPEN_MM, "close": FW.GRIP_CLOSED_MM}.get(s.gripper, s.gripper)
            yield from cmd(sim, fmt.grip(float(mm)))
        ms = s.duration * 1000
        if s.joints is not None:
            yield from cmd(sim, fmt.movej(s.joints, ms))
        else:
            x, y, z, *r = s.pose
            yield from cmd(sim, fmt.move(x, y, z, r[0] if r else 0.0, ms))
        yield from wait(sim, s.pause_after * 1000)
    return len(slides)
