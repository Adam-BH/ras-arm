"""A Python twin of the firmware: same protocol, same 11-state machine, same decisions, same motion
profiles. Port of firmware/src/{3_joints … 6_app}.

There is no scripted world. The twin drives a HAL (world.World)
and reads back only what the board reads (limit switches, IR beam, colour sensor, E-stop sense);
whether a grasp holds, where a dropped cube lands or how far the arm sags on an E-stop is physics.
"""
from __future__ import annotations

import math
from typing import Callable

from .config import CELL, FW, Pose
from .kinematics import OK, fk, ik, within_limits
from .protocol import COLOURS, COMMANDS, ERRORS, POSE_NAMES

TICK_MS = 5
RAMP = FW.RAMP_FRACTION
HOMING = {"fast": FW.HOMING_FAST_DPS, "slow": FW.HOMING_SLOW_DPS, "backoff": FW.HOMING_BACKOFF,
          "leave_max": FW.HOMING_LEAVE_MAX, "margin": FW.HOMING_SEARCH_MARGIN, "slow_max": FW.HOMING_SLOW_MAX}
ORDER = [1, 2, 0]                                   # homing order J2 → J3 → J1
REASON = {"state": "state", "busy": "busy", "running": "running", "unreachable": "unreachable",
          "limit": "limit", "estop": "estop"}


def clamp(j: int, v: float) -> float:
    return min(FW.JOINT_MAX[j], max(FW.JOINT_MIN[j], v))


def trapezoid(k: float) -> float:
    """position profile: 25 % accel / 50 % cruise / 25 % decel (same as the stepper ramp)"""
    v = 1 / (1 - RAMP)
    if k <= 0:
        return 0.0
    if k >= 1:
        return 1.0
    if k < RAMP:
        return 0.5 * v * k * k / RAMP
    if k < 1 - RAMP:
        return 0.5 * v * RAMP + v * (k - RAMP)
    return 1 - 0.5 * v * (1 - k) * (1 - k) / RAMP


def _num(s: str) -> float | None:
    try:
        v = float(s)
    except ValueError:
        return None
    return v if math.isfinite(v) else None


class FirmwareTwin:
    def __init__(self, hal, out: Callable[[str], None]):
        self.hal = hal
        self.out = out
        self.now = 0
        # what the board believes at power-up: step counters at 0, servos neutral, gripper open
        self.q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, FW.GRIP_OPEN_MM]
        self.mv = None                              # {from, to, t0, T}
        self.run = None                             # (joint, deg/s): homing drives
        self.wp: list[Pose] = []
        self.wp_ms = 0.0
        self.params = {"confident_ratio": FW.DEC_CONFIDENT_RATIO, "same_as_last": FW.DEC_SAME_AS_LAST,
                       "empty_ratio": FW.DEC_EMPTY_RATIO, "use_last_trick": FW.DEC_USE_LAST_TRICK,
                       "grip_wiggle": FW.DEC_GRIP_WIGGLE_DEG, "jiggle_mm": FW.DEC_JIGGLE_MM,
                       "jiggle_deg": FW.DEC_JIGGLE_DEG, "pick_retries": FW.PICK_RETRIES,
                       "rehome_every": FW.REHOME_EVERY}
        self.cal = {"red": [FW.DEC_REF_RED_R, FW.DEC_REF_RED_G, FW.DEC_REF_RED_B],
                    "blue": [FW.DEC_REF_BLUE_R, FW.DEC_REF_BLUE_G, FW.DEC_REF_BLUE_B],
                    "empty_clear": FW.DEC_EMPTY_CLEAR}
        self.last_good = [0.0, 0.0, 0.0]
        self.last_colour = "UNKNOWN"
        self.poses = dict(CELL)
        self.ir_raw_since = 0
        self.ir_stable = False
        self.col = {"n": 0, "sum": [0, 0, 0, 0], "t": 0}
        self.h = {"phase": "start", "i": 0, "a0": 0.0, "t0": 0, "why": "none"}
        self.seq = {"phase": "", "grip": self.poses["CUBE"], "above": self.poses["CUBE"], "j6": 0.0, "ok": True}
        self.state = "BOOT"
        self.err = 0
        self.running = self.homed = self.manual = False
        self.cal_target = None
        self.attempt = self.colour_tries = self.cycles = 0
        self.cube_colour = "UNKNOWN"
        self.cube_rgbc = (0, 0, 0, 0)
        self.t_enter = 0
        self.last_reading = "UNKNOWN"
        self.pending_done = None
        self.t_tel = 0
        self.boot()

    # ───────────── motion (3_joints + 4_motion) ─────────────
    def _start_move(self, target, ms=0.0):
        to = [clamp(j, v) for j, v in enumerate(target)]
        T = ms
        for j in range(7):
            T = max(T, abs(to[j] - self.q[j]) / FW.JOINT_MAX_DPS[j] / (1 - RAMP) * 1000)
        self.run = None
        self.mv = {"from": self.q[:], "to": to, "t0": self.now, "T": max(T, FW.MIN_MOVE_MS)}

    def targets(self):
        return self.mv["to"][:] if self.mv else self.q[:]

    def busy(self) -> bool:
        return bool(self.mv or self.run or self.wp)

    def stop(self):
        self.mv, self.run, self.wp = None, None, []

    def where(self) -> Pose:
        return fk(self.q)

    def move_to(self, p: Pose, ms=0.0) -> bool:
        st, q = ik(p)
        if st != OK:
            return False
        t = self.targets()
        self.wp = []
        self._start_move(q + [t[6]], ms)
        return True

    def move_joints(self, q6, ms=0.0) -> bool:
        if not within_limits(q6):
            return False
        self.wp = []
        self._start_move(list(q6[:6]) + [self.targets()[6]], ms)
        return True

    def move_line(self, to: Pose, mm_s=FW.LINE_DEFAULT_MM_S) -> bool:
        a = self.where()
        d = math.dist((a.x, a.y, a.z), (to.x, to.y, to.z))
        n = min(FW.LINE_MAX_WP, max(1, math.ceil(d / FW.LINE_STEP_MM)))
        pts = []
        for k in range(1, n + 1):
            t = k / n
            pt = Pose(a.x + (to.x - a.x) * t, a.y + (to.y - a.y) * t, a.z + (to.z - a.z) * t,
                      a.roll + (to.roll - a.roll) * t)
            if ik(pt)[0] != OK:                       # the whole line is checked before moving
                return False
            pts.append(pt)
        self.wp = pts
        self.wp_ms = 1000 * d / n / (mm_s if mm_s > 0 else FW.LINE_DEFAULT_MM_S)
        return True

    def grip(self, mm, ms=FW.GRIP_MOVE_MS):
        t = self.targets()
        t[6] = mm
        self._start_move(t, ms)

    def joint(self, j, v, ms):
        t = self.targets()
        t[j] = v
        self._start_move(t, ms)

    def _update_motion(self, dt):
        if self.run:
            j, v = self.run
            self.q[j] += v * dt / 1000
        if self.mv:
            k = (self.now - self.mv["t0"]) / self.mv["T"]
            s = trapezoid(k)
            for j in range(7):
                self.q[j] = self.mv["from"][j] + (self.mv["to"][j] - self.mv["from"][j]) * s
            if k >= 1:
                self.mv = None
        if not self.mv and not self.run and self.wp:
            pt = self.wp.pop(0)
            self._start_move(ik(pt)[1] + [self.targets()[6]], self.wp_ms)

    # ───────────── sensors (2_devices) ─────────────
    def limit(self, j) -> bool:
        return self.hal.limit(j)

    def limits(self):
        return [self.limit(0), self.limit(1), self.limit(2)]

    def estop(self) -> bool:
        return not self.hal.power_ok()

    def _update_sensors(self):
        raw = self.hal.ir_raw()
        if raw != self.ir_stable:
            if not self.ir_raw_since:
                self.ir_raw_since = self.now
            if self.now - self.ir_raw_since >= (FW.IR_STABLE_MS if raw else FW.IR_CLEAR_MS):
                self.ir_stable, self.ir_raw_since = raw, 0
        else:
            self.ir_raw_since = 0

    def ir(self) -> bool:
        return self.ir_stable

    # ───────────── decisions (5_tasks/decisions.c) ─────────────
    @staticmethod
    def chroma(v):
        c = v[3] or 1
        return [v[0] / c, v[1] / c, v[2] / c]

    def decide_colour(self, v) -> str:
        if v[3] < self.cal["empty_clear"] * self.params["empty_ratio"]:
            return "EMPTY"
        k = self.chroma(v)
        dr, db = math.dist(k, self.cal["red"]), math.dist(k, self.cal["blue"])
        if dr < db * self.params["confident_ratio"]:
            return "RED"
        if db < dr * self.params["confident_ratio"]:
            return "BLUE"
        if self.params["use_last_trick"] and self.last_colour in ("RED", "BLUE"):
            if math.dist(k, self.last_good) < self.params["same_as_last"]:
                return self.last_colour
            return "BLUE" if self.last_colour == "RED" else "RED"
        return "UNKNOWN"

    def grip_offset(self, a):
        m, g = self.params["jiggle_mm"], self.params["jiggle_deg"]
        return [(0, 0, 0), (m, 0, g), (-m, 0, -g), (0, m, 0), (0, -m, 0)][a % 5]

    # ───────────── tasks (5_tasks/tasks.c) ─────────────
    def colour_start(self):
        self.col = {"n": 0, "sum": [0, 0, 0, 0], "t": self.now}

    def colour_done(self):
        """COLOUR_SAMPLES integrations of COLOUR_INTEG_MS, averaged → (colour, rgbc) or None"""
        c = self.col
        if self.now - c["t"] < FW.COLOUR_INTEG_MS:
            return None
        v = self.hal.colour_sample()
        c["sum"] = [s + x for s, x in zip(c["sum"], v)]
        c["n"] += 1
        c["t"] = self.now
        if c["n"] < FW.COLOUR_SAMPLES:
            return None
        avg = tuple(s // c["n"] for s in c["sum"])
        return self.decide_colour(avg), avg

    def _hgo(self, phase, j, dps):
        self.h.update(phase=phase, t0=self.now, a0=self.q[j])
        self.mv, self.wp = None, []
        self.run = (j, (FW.HOME_DIR[j] if dps >= 0 else -FW.HOME_DIR[j]) * abs(dps))

    def _homing_update(self) -> str:
        h = self.h
        j = ORDER[min(h["i"], 2)]
        travel = abs(self.q[j] - h["a0"])

        def fail(why):
            self.stop()
            h["why"] = why
            return "failed"

        if h["phase"] not in ("start", "next", "gohome") and self.now - h["t0"] > FW.HOMING_PHASE_TIMEOUT_MS:
            return fail("timeout")
        ph = h["phase"]
        if ph == "start":
            if self.limit(j):
                self._hgo("leave", j, -HOMING["fast"])
            else:
                self._hgo("fast", j, HOMING["fast"])
        elif ph == "leave":
            if not self.limit(j):
                self._hgo("fast", j, HOMING["fast"])
            elif travel > HOMING["leave_max"]:
                return fail("broken_wire")
        elif ph == "fast":
            if self.limit(j):
                self._hgo("backoff", j, -HOMING["fast"])
            elif travel > FW.JOINT_MAX[j] - FW.JOINT_MIN[j] + HOMING["margin"]:
                return fail("not_found")
        elif ph == "backoff":
            if travel >= HOMING["backoff"] and not self.limit(j):
                self._hgo("slow", j, HOMING["slow"])
            elif travel > HOMING["leave_max"]:
                return fail("stuck")
        elif ph == "slow":
            if self.limit(j):
                self.run = None
                self.hal.rezero(j, self.q[j], FW.HOME_SWITCH_DEG[j])      # stepper_set_pos()
                self.q[j] = float(FW.HOME_SWITCH_DEG[j])
                h["phase"] = "next"
            elif travel > HOMING["slow_max"]:
                return fail("not_found")
        elif ph == "next":
            h["i"] += 1
            if h["i"] < 3:
                h["phase"] = "start"
            elif not self.move_to(self.poses["HOME"], FW.MOVE_HOME_MS):
                return fail("home_pose")
            else:
                h["phase"] = "gohome"
        elif ph == "gohome" and not self.busy():
            return "done"
        return "running"

    def _pick_start(self, attempt):
        dx, dy, dr = self.grip_offset(attempt)
        c = self.poses["CUBE"]
        g = Pose(c.x + dx, c.y + dy, c.z, c.roll + dr)
        self.seq = {"phase": "above", "grip": g, "above": Pose(g.x, g.y, FW.SAFE_Z, g.roll), "j6": 0.0, "ok": True}
        self.grip(FW.GRIP_OPEN_MM)
        self.seq["ok"] = self.move_to(self.seq["above"], FW.MOVE_ABOVE_MS)

    def _pick_update(self) -> str:
        s = self.seq
        if not s["ok"]:
            return "failed"
        if self.busy():
            return "running"
        w = 0 if self.attempt == 0 else self.params["grip_wiggle"]
        ph = s["phase"]
        if ph == "above":
            if not self.move_line(s["grip"], FW.PICK_DOWN_MM_S):
                return "failed"
            s["phase"] = "down"
        elif ph == "down":
            self.grip(FW.GRIP_CLOSED_MM)
            s["phase"], s["j6"] = "close", self.q[5]
            if w > 0:
                self.joint(5, s["j6"] + w, FW.WIGGLE_MS)
                s["phase"] = "wiggle"
        elif ph == "wiggle":
            self.joint(5, s["j6"], FW.WIGGLE_MS)
            s["phase"] = "close"
        elif ph == "close":
            if not self.move_line(s["above"], FW.PICK_UP_MM_S):
                return "failed"
            s["phase"] = "up"
        elif ph == "up":
            return "done"
        return "running"

    def _place_start(self, colour):
        drop = self.poses["BIN_A"] if colour == "RED" else self.poses["BIN_B"]
        self.seq = {"phase": "transit", "grip": drop, "above": Pose(drop.x, drop.y, FW.SAFE_Z + FW.PLACE_ABOVE_DZ, drop.roll),
                    "j6": 0.0, "ok": True}
        self.seq["ok"] = self.move_to(self.seq["above"], FW.MOVE_TRANSIT_MS)

    def _place_update(self) -> str:
        s = self.seq
        if not s["ok"]:
            return "failed"
        if self.busy():
            return "running"
        ph = s["phase"]
        if ph == "transit":
            if not self.move_line(s["grip"], FW.PLACE_DOWN_MM_S):
                return "failed"
            s["phase"] = "down"
        elif ph == "down":
            self.grip(FW.GRIP_OPEN_MM, 250)
            s["phase"] = "open"
        elif ph == "open":
            if not self.move_line(s["above"], FW.PLACE_UP_MM_S):
                return "failed"
            s["phase"] = "up"
        elif ph == "up":
            return "done"
        return "running"

    # ───────────── state machine (6_app/fsm.c) ─────────────
    def ev(self, what, arg=None, n=None):
        self.out(f"EV {what}" + (f" {arg}" if arg else "") + (f" {n}" if n is not None else ""))

    def enter(self, s):
        self.state, self.t_enter = s, self.now
        if s == "HOMING":
            self.homed = False
            self.h = {"phase": "start", "i": 0, "a0": 0.0, "t0": self.now, "why": "none"}
        if s in ("READ_COLOUR", "VERIFY"):
            self.colour_start()
        if s == "PICK":
            self._pick_start(self.attempt)
        if s == "PLACE":
            self._place_start(self.cube_colour)
        if s == "RETURN":
            self.seq["ok"] = self.move_to(self.poses["HOME"], FW.MOVE_HOME_MS)
        if s in ("ERROR", "ESTOP"):
            self.stop()
            self.running = self.manual = False
            self.cal_target = None

    def fail(self, e):
        self.err = e
        self.enter("ERROR")
        self.ev("ERROR", ERRORS[e])

    def fsm_busy(self) -> bool:
        return self.manual or self.state == "HOMING" or self.cal_target is not None

    def boot(self):
        self.state, self.t_enter, self.err = "BOOT", self.now, 0
        self.running = self.homed = self.manual = False
        self.ev("BOOT")

    def _fsm_tick(self):
        if self.estop():
            if self.state != "ESTOP":
                self.err, self.homed = 5, False
                self.enter("ESTOP")
                self.ev("ESTOP")
            return
        if self.manual and not self.busy():
            self.manual = False
        s = self.state
        if s == "BOOT":
            if self.now - self.t_enter > FW.BOOT_SETTLE_MS:
                self.enter("HOMING")
        elif s == "HOMING":
            r = self._homing_update()
            if r == "done":
                self.homed = True
                self.ev("HOMED")
                self.enter("IDLE")
            elif r == "failed":
                self.ev("HOME_FAIL", self.h["why"], ORDER[min(self.h["i"], 2)] + 1)
                self.fail(1)
        elif s == "IDLE":
            if self.running and not self.manual and self.ir():
                self.colour_tries = self.attempt = 0
                self.enter("READ_COLOUR")
        elif s == "READ_COLOUR":
            c = self.colour_done()
            if c:
                self.last_reading = c[0]
                if c[0] in ("RED", "BLUE"):
                    self.cube_colour, self.cube_rgbc = c
                    self.ev("COLOUR", c[0])
                    self.enter("PICK")
                else:
                    self.colour_tries += 1
                    if self.colour_tries < FW.COLOUR_TRIES:
                        self.colour_start()
                    else:
                        self.fail(2)
        elif s == "PICK":
            r = self._pick_update()
            if r == "done":
                self.enter("VERIFY")
            elif r == "failed":
                self.fail(4)
        elif s == "VERIFY":
            c = self.colour_done()
            if c:
                self.last_reading = c[0]
                if not self.ir() and c[0] == "EMPTY":
                    self.last_colour, self.last_good = self.cube_colour, self.chroma(self.cube_rgbc)
                    self.ev("PICKED", None, self.attempt)
                    self.enter("PLACE")
                else:
                    self.attempt += 1
                    if self.attempt < 1 + self.params["pick_retries"]:
                        self.ev("PICK_RETRY", None, self.attempt)
                        self.enter("PICK")
                    else:
                        self.fail(3)
        elif s == "PLACE":
            r = self._place_update()
            if r == "done":
                self.ev("PLACED", self.cube_colour)
                self.enter("RETURN")
            elif r == "failed":
                self.fail(4)
        elif s == "RETURN":
            if not self.seq["ok"]:
                self.fail(4)
            elif not self.busy():
                self.cycles += 1
                self.ev("CYCLE", None, self.cycles)
                every = self.params["rehome_every"]
                if every > 0 and self.cycles % int(every) == 0:
                    self.ev("REHOME")
                    self.enter("HOMING")
                else:
                    self.enter("IDLE")
        elif s == "TEACH" and self.cal_target:
            c = self.colour_done()
            if c:
                k = self.chroma(c[1])
                if self.cal_target == "EMPTY":
                    self.cal["empty_clear"] = c[1][3]
                elif self.cal_target == "RED":
                    self.cal["red"] = k
                elif self.cal_target == "BLUE":
                    self.cal["blue"] = k
                self.ev("CAL", self.cal_target)
                self.cal_target = None

    # ───────────── console (6_app/console.c) ─────────────
    def input(self, raw: str):
        tok = raw.strip().split()
        if not tok:
            return
        cmd, args = tok[0].upper(), tok[1:]
        argc = len(args)
        ok_line = lambda data="": self.out(f"OK {cmd}" + (f" {data}" if data else ""))  # noqa: E731
        err_line = lambda why: self.out(f"ERR {cmd} {why}")  # noqa: E731
        if cmd not in COMMANDS:
            return err_line("unknown")
        a = [_num(x) for x in args]
        if cmd in ("MOVE", "LINE", "MOVEJ", "JOG", "JOGJ", "GRIP") and any(v is None for v in a):
            return err_line("args")

        def can_move(teach_only):
            if self.state != "TEACH" and (teach_only or self.state != "IDLE"):
                return "state"
            if self.running:
                return "running"
            if self.fsm_busy():
                return "busy"
            return None

        def started(ok, why):
            if not ok:
                return why
            self.manual = True
            return None

        why, is_move = None, False
        if cmd == "PING":
            return ok_line(f"ras-arm {FW.FW_VERSION}")
        if cmd == "STATUS":
            return ok_line(f"{self.state} {ERRORS[self.err]} homed={int(self.homed)} running={int(self.running)}")
        if cmd == "POSES":
            for n in POSE_NAMES:
                pp = self.poses[n]
                self.out(f"POSE {n} {pp.x:.1f} {pp.y:.1f} {pp.z:.1f} {pp.roll:.1f}")
        elif cmd == "HOME":
            is_move = True
            if self.state not in ("IDLE", "TEACH", "ERROR"):
                why = "state"
            elif self.estop():
                why = "estop"
            elif self.manual:
                why = "busy"
            elif self.running:
                why = "running"
            else:
                self.err = 0
                self.enter("HOMING")
        elif cmd == "RUN":
            if self.state != "IDLE":
                why = "state"
            elif self.manual:
                why = "busy"
            else:
                self.running = True
        elif cmd == "STOP":
            self.running = False
            if self.manual:
                self.stop()
                self.manual = False
        elif cmd == "TEACH":
            if self.state != "IDLE":
                why = "state"
            elif self.manual:
                why = "busy"
            else:
                self.running = False
                self.enter("TEACH")
        elif cmd == "DONE":
            if self.state != "TEACH":
                why = "state"
            else:
                self.cal_target = None
                self.enter("IDLE")
        elif cmd == "RESET":
            if self.state not in ("ERROR", "ESTOP"):
                why = "state"
            elif self.estop():
                why = "estop"
            else:
                self.err = 0
                self.enter("HOMING")
        elif cmd in ("MOVE", "LINE"):
            if argc < 3 or argc > (5 if cmd == "MOVE" else 4):
                return err_line("args")
            pose = Pose(a[0], a[1], a[2], a[3] if cmd == "MOVE" and argc >= 4 else 0.0)
            why = can_move(False)
            if not why:
                ok = (self.move_to(pose, a[4] if argc == 5 else 0) if cmd == "MOVE"
                      else self.move_line(pose, a[3] if argc == 4 else FW.LINE_DEFAULT_MM_S))
                why = started(ok, "unreachable")
            is_move = True
        elif cmd == "MOVEJ":
            if argc < 6 or argc > 7:
                return err_line("args")
            why = can_move(False) or started(self.move_joints(a[:6], a[6] if argc == 7 else 0), "limit")
            is_move = True
        elif cmd == "JOG":
            if argc < 3 or argc > 4:
                return err_line("args")
            why = can_move(True)
            if not why:
                pp = self.where()
                why = started(self.move_to(Pose(pp.x + a[0], pp.y + a[1], pp.z + a[2],
                                                pp.roll + (a[3] if argc == 4 else 0)), FW.JOG_MS), "unreachable")
            is_move = True
        elif cmd == "JOGJ":
            if argc != 2 or not float(a[0]).is_integer() or not 1 <= a[0] <= 7:
                return err_line("args")
            why = can_move(True)
            if not why:
                j = int(a[0]) - 1
                t = self.targets()
                t[j] += a[1]
                if j == 6:
                    self.grip(t[6])
                    why = started(True, None)
                else:
                    why = started(self.move_joints(t, FW.JOG_MS), "limit")
            is_move = True
        elif cmd == "GRIP":
            if argc != 1:
                return err_line("args")
            why = can_move(False)
            if not why:
                self.grip(a[0])
                why = started(True, None)
            is_move = True
        elif cmd in ("SAVE", "GO"):
            if argc != 1:
                return err_line("args")
            name = args[0].upper()
            if name not in POSE_NAMES:
                return err_line("pose")
            why = can_move(True)
            if not why and cmd == "SAVE":
                self.poses[name] = self.where()
            if not why and cmd == "GO":
                why = started(self.move_to(self.poses[name], FW.MOVE_HOME_MS), "unreachable")
                is_move = True
        elif cmd == "CAL":
            if argc != 1:
                return err_line("args")
            c = args[0].upper()
            if c not in ("EMPTY", "RED", "BLUE"):
                return err_line("args")
            why = can_move(True)
            if not why:
                self.cal_target = c
                self.colour_start()
            is_move = True
        elif cmd == "SET":
            if argc != 2 or _num(args[1]) is None:
                return err_line("args")
            if self.state not in ("IDLE", "TEACH"):
                why = "state"
            elif args[0] not in self.params:
                return err_line("key")
            else:
                self.params[args[0]] = _num(args[1])
        if why:
            return err_line(REASON[why])
        ok_line()
        if is_move:
            self.pending_done = cmd

    def telemetry_line(self) -> str:
        pp = self.where()
        i = lambda v: str(round(v))  # noqa: E731
        return ",".join(["T", i(self.now), self.state, *(i(v * 10) for v in self.q), i(pp.x), i(pp.y), i(pp.z),
                         str(int(self.ir())), str(COLOURS.index(self.last_reading)), str(self.err),
                         "".join(str(int(b)) for b in self.limits()), str(int(self.estop()))])

    def tick(self):
        """one 5 ms firmware loop; the caller then drives the HAL and steps the physics"""
        self.now += TICK_MS
        if self.hal.power_ok():
            self._update_motion(TICK_MS)
        self._update_sensors()
        self._fsm_tick()
        if self.pending_done and not self.fsm_busy():
            if self.state not in ("ERROR", "ESTOP"):
                self.out(f"DONE {self.pending_done}")
            self.pending_done = None
        if self.now - self.t_tel >= FW.TELEMETRY_MS:
            self.t_tel = self.now
            self.out(self.telemetry_line())
