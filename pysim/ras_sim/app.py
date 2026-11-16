"""The simulation: physics world + firmware twin + host scripts, on one deterministic clock.

Every 5 ms: the firmware twin runs one loop, its joint commands go to the motors, physics advances
5 × 1 ms, and each host script gets one turn. Host scripts are generators that talk to the firmware
only through the serial protocol (send a line, wait for its OK / DONE), exactly like a serial monitor or a
Python script on the real USB port would.

    sim = Simulation()
    sim.run_until(lambda: sim.fw.state == "IDLE")
    sim.world.place_in_cradle("RED")
    sim.send("RUN")
    sim.run_until(lambda: sim.world.bin_counts()["A"] == 1, timeout_ms=30000)
"""
from __future__ import annotations

import time
import traceback
from dataclasses import dataclass, field
from typing import Callable, Generator, Iterable

from .config import FW
from .firmware import TICK_MS, FirmwareTwin
from .protocol import Reply, Telemetry, classify, command_name, parse_reply, parse_telemetry
from .world import World

MOTION_COMMANDS = {"HOME", "MOVE", "LINE", "MOVEJ", "JOG", "JOGJ", "GRIP", "GO", "CAL"}
BOOTLOADER_MS = 1600          # UNO reset → optiboot waits for an upload → the sketch starts (≈ 2 s on the bench)
Script = Generator[None, None, object]


class CommandError(RuntimeError):
    pass


@dataclass
class LogLine:
    ms: int
    line: str


@dataclass
class _Task:
    name: str
    gen: Script
    done: bool = False
    result: object = None
    error: BaseException | None = None


@dataclass
class Simulation:
    gui: bool = False
    seed: int = 0
    # where the arm really is at power-up (the firmware believes 0, 0, 0): homing has to find out
    power_up_pose: tuple = (20.0, 40.0, 90.0, 0.0, 0.0, 0.0)
    echo: Callable[[int, str], None] | None = None        # gets (ms, line) for every non-telemetry line
    realtime: bool = False
    speed: float = 1.0
    log: list[LogLine] = field(default_factory=list)
    tel: Telemetry | None = None

    def __post_init__(self):
        self.world = World(gui=self.gui, seed=self.seed)
        self.world.set_true_pose(self.power_up_pose, FW.GRIP_OPEN_MM)
        self._reply: Reply | None = None
        self.tasks: list[_Task] = []
        self.tel_sinks: list[Callable[[str], None]] = []
        self.line_sinks: list[Callable[[str], None]] = []    # every line the board sends (the serial port)
        self.t_ms = 0                                         # simulation clock (the board's millis() restarts)
        self.booting_until: int | None = None                 # set while the UNO is in reset / bootloader
        self.resets = 0
        self.on_tick: list[Callable[[], None]] = []
        self.fw = FirmwareTwin(self.world, self._on_line)
        believed = self.fw.q
        for j in range(FW.N_STEPPERS):
            self.world.offset[j] = self.power_up_pose[j] - believed[j]
        self._wall0, self._sim0 = time.perf_counter(), 0

    # ───────────── lines from the firmware ─────────────
    def _on_line(self, line: str):
        for sink in self.line_sinks:
            sink(line)
        kind = classify(line)
        if kind == "telemetry":
            self.tel = parse_telemetry(line)
            for sink in self.tel_sinks:
                sink(line)
            return
        if kind == "reply":
            self._reply = parse_reply(line)
        ms = self.t_ms
        self.log.append(LogLine(ms, line))
        if self.echo:
            self.echo(ms, line)

    @property
    def now(self) -> int:
        return self.t_ms

    @property
    def booting(self) -> bool:
        return self.booting_until is not None

    def send(self, cmd: str) -> Reply:
        """one protocol line → its OK / ERR reply (synchronous, like the firmware's console)"""
        if self.booting:                                   # the bootloader drops what the host sends
            return Reply(False, command_name(cmd), "board resetting")
        self._reply = None
        self.fw.input(cmd)
        return self._reply or Reply(True, command_name(cmd), "")

    def reboot(self, bootloader_ms: int = BOOTLOADER_MS):
        """reset the UNO, like a host opening the USB port (DTR) or the reset button: the board goes silent,
        EN is pulled up (steppers off, the arm may sag), the PCA9685 keeps the servos where they were; then
        the sketch starts again from scratch (step counters 0, millis() 0, EV BOOT, homing)"""
        self.booting_until = self.t_ms + bootloader_ms
        self.world.drivers_enabled = False
        self.resets += 1

    def _finish_boot(self):
        self.booting_until = None
        self.fw = FirmwareTwin(self.world, self._on_line)
        self.world.drivers_enabled = True

    # ───────────── the clock ─────────────
    def tick(self):
        self.t_ms += TICK_MS
        if self.booting:
            if self.t_ms >= self.booting_until:
                self._finish_boot()
        else:
            self.fw.tick()
        self.world.drive(self.fw.q)
        self.world.step()                                   # 5 ms of physics in 1 ms substeps
        for t in self.tasks:
            if t.done:
                continue
            try:
                next(t.gen)
            except StopIteration as e:
                t.done, t.result = True, e.value
            except Exception as e:  # noqa: BLE001 - a failing script must not stop the simulation
                t.done, t.error = True, e
                if self.echo:
                    self.echo(self.now, f"# script {t.name} failed: {e}")
                    if not isinstance(e, CommandError):
                        self.echo(self.now, traceback.format_exc())
        self.tasks = [t for t in self.tasks if not t.done]      # callers keep their own _Task handle
        for cb in self.on_tick:
            cb()
        if self.realtime:
            self._pace()

    def _pace(self):
        target = self._wall0 + (self.now - self._sim0) / 1000 / max(self.speed, 1e-3)
        lag = target - time.perf_counter()
        if lag > 0:
            time.sleep(lag)
        elif lag < -0.25:                                   # cannot keep up: drop the debt
            self.set_speed(self.speed)

    def set_speed(self, speed: float):
        self.speed = speed
        self._wall0, self._sim0 = time.perf_counter(), self.now

    def run_for(self, ms: float):
        end = self.now + ms
        while self.now < end:
            self.tick()

    def run_until(self, pred: Callable[[], bool], timeout_ms: float = 60000) -> bool:
        end = self.now + timeout_ms
        while self.now < end:
            if pred():
                return True
            self.tick()
        return pred()

    def start(self, gen: Script, name: str = "script") -> _Task:
        t = _Task(name, gen)
        self.tasks.append(t)
        return t

    def busy_scripts(self) -> list[str]:
        return [t.name for t in self.tasks if not t.done]

    def run_script(self, gen: Script, name: str = "script", timeout_ms: float = 600000):
        """run one host script to its end (headless use and tests); raises what the script raised"""
        t = self.start(gen, name)
        if not self.run_until(lambda: t.done, timeout_ms):
            raise TimeoutError(f"{name} still running after {timeout_ms / 1000:.0f} s")
        if t.error:
            raise t.error
        return t.result

    def events(self, since: int = 0) -> list[str]:
        return [ln.line for ln in self.log[since:] if ln.line.startswith("EV ")]

    def close(self):
        self.world.close()


# ───────────── host-side helpers (use with `yield from`) ─────────────
def wait(sim: Simulation, ms: float) -> Script:
    end = sim.now + ms
    while sim.now < end:
        yield


def wait_until(sim: Simulation, pred: Callable[[], bool], timeout_ms: float = 60000, what: str = "condition") -> Script:
    end = sim.now + timeout_ms
    while not pred():
        if sim.now > end:
            raise TimeoutError(f"timed out waiting for {what}")
        yield


def cmd(sim: Simulation, line: str, timeout_ms: float = 30000) -> Script:
    """send a command; for motion commands wait for its DONE. Raises CommandError on ERR / EV ERROR / EV ESTOP."""
    mark = len(sim.log)
    r = sim.send(line)
    if not r.ok:
        raise CommandError(f"{line} → ERR {r.text}")
    name = command_name(line)
    if name not in MOTION_COMMANDS:
        return r
    end = sim.now + timeout_ms
    while True:
        for ln in sim.log[mark:]:
            if ln.line == f"DONE {name}":
                return r
            if ln.line.startswith(("EV ERROR", "EV ESTOP")):
                raise CommandError(f"{line} → {ln.line}")
        mark = len(sim.log)
        if sim.now > end:
            raise CommandError(f"{line} → no DONE after {timeout_ms / 1000:.0f} s")
        yield


def commands(sim: Simulation, lines: Iterable[str]) -> Script:
    for ln in lines:
        yield from cmd(sim, ln)


def idle_and_homed(sim: Simulation) -> bool:
    return sim.fw.state == "IDLE" and sim.fw.homed and not sim.fw.manual
