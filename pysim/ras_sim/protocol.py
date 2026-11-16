"""The serial protocol (docs/serial_protocol.md), same as sim/src/link/protocol.ts."""
from __future__ import annotations

import re
from dataclasses import dataclass

STATES = ["BOOT", "HOMING", "IDLE", "READ_COLOUR", "PICK", "VERIFY", "PLACE", "RETURN", "TEACH", "ERROR", "ESTOP"]
ERRORS = ["NONE", "HOME_FAILED", "COLOUR_UNKNOWN", "PICK_FAILED", "UNREACHABLE", "ESTOP"]
COLOURS = ["EMPTY", "RED", "BLUE", "UNKNOWN"]
POSE_NAMES = ["HOME", "CUBE", "BIN_A", "BIN_B"]
COMMANDS = ["PING", "STATUS", "HOME", "RUN", "STOP", "RESET", "TEACH", "DONE", "MOVE", "LINE", "MOVEJ",
            "JOG", "JOGJ", "GRIP", "SAVE", "GO", "POSES", "CAL", "SET"]


@dataclass
class Telemetry:
    ms: int
    state: str
    joints: list[float]          # J1..J6 deg, [6] = grip mm
    tip: tuple[float, float, float]
    ir: bool
    colour: str
    err: int
    limits: tuple[bool, bool, bool]
    estop: bool

    @property
    def err_name(self) -> str:
        return ERRORS[self.err] if 0 <= self.err < len(ERRORS) else f"E{self.err}"


def parse_telemetry(line: str) -> Telemetry | None:
    """T,<ms>,<state>,<j1..j6 ×10>,<grip ×10>,<x>,<y>,<z>,<ir>,<colour>,<err>,<lim1><lim2><lim3>,<estop>"""
    f = line.strip().split(",")
    if f[0] != "T" or len(f) != 18:
        return None
    if not all(re.fullmatch(r"-?\d+", f[k]) for k in (1, *range(3, 16), 17)) or not re.fullmatch(r"[01]{3}", f[16]):
        return None
    n = lambda k: int(f[k])  # noqa: E731
    return Telemetry(n(1), f[2], [n(k) / 10 for k in range(3, 10)], (n(10), n(11), n(12)), f[13] == "1",
                     COLOURS[n(14)] if 0 <= n(14) < 4 else "UNKNOWN", n(15),
                     tuple(c == "1" for c in f[16]), f[17] == "1")


def classify(line: str) -> str:
    if line.startswith("T,"):
        return "telemetry"
    if line.startswith(("OK ", "ERR ")):
        return "reply"
    if line.startswith("DONE "):
        return "done"
    if line.startswith("EV "):
        return "event"
    return "data"


@dataclass
class Reply:
    ok: bool
    cmd: str
    text: str


def parse_reply(line: str) -> Reply | None:
    m = re.match(r"^(OK|ERR) (\S+)(?: (.*))?$", line.strip())
    return Reply(m[1] == "OK", m[2], m[3] or "") if m else None


def command_name(cmd: str) -> str:
    t = cmd.strip().split()
    return t[0].upper() if t else ""


def _n(v: float) -> str:
    return f"{round(v * 10) / 10:g}"


class fmt:
    @staticmethod
    def move(x, y, z, roll=0.0, ms=None):
        return f"MOVE {_n(x)} {_n(y)} {_n(z)} {_n(roll)}" + (f" {round(ms)}" if ms else "")

    @staticmethod
    def line(x, y, z, mm_s=None):
        return f"LINE {_n(x)} {_n(y)} {_n(z)}" + (f" {_n(mm_s)}" if mm_s else "")

    @staticmethod
    def movej(q, ms=None):
        return "MOVEJ " + " ".join(_n(v) for v in q[:6]) + (f" {round(ms)}" if ms else "")

    @staticmethod
    def grip(mm):
        return f"GRIP {_n(mm)}"
