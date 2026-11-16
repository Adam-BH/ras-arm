"""Interactive front end: PyBullet window with buttons and a live status overlay, a protocol console on the
terminal, and an OpenCV window for the overhead camera.

Buttons sit in PyBullet's parameter panel (right side of the window; click = press). The terminal takes
firmware commands (PING, STATUS, MOVE 200 0 100, TEACH, JOG 0 0 -5 …) and sim commands starting with '!'
(type !help).
"""
from __future__ import annotations

import math
import queue
import sys
import threading
import time

import cv2
import numpy as np
import pybullet as p

from .app import Simulation
from .config import FW
from .feeder import feeder
from .protocol import ERRORS
from .slides import DEMO, play
from .sorter import SortReport, scatter, vision_sort
from .vision import OverheadCamera, annotate, detect, mask_view

HELP = """sim commands (firmware commands go straight to the twin):
  !red / !blue         put a cube in the cradle          !scatter [n]   n loose cubes on the table
  !feed [n]            RUN + feed n random cubes          !sort          vision sort the table
  !slides              play the keyframe demo             !clear         remove every cube
  !fail                the next grasp slips               !estop         toggle the motor rail
  !lose <j> <deg>      stepper j (1-3) skips deg          !break <j>     toggle switch j's wire (1-3)
  !snap                camera snapshot (+ mask) window    !speed <x>     sim speed (1 = real time)
  !bins                count the cubes in the bins        !reset         reset the board (like a USB reconnect)
  !quit"""


class Gui:
    def __init__(self, sim: Simulation, camera_window: bool = True, bridge=None):
        self.sim, self.w = sim, sim.world
        self.cam = OverheadCamera(self.w.cid)
        self.camera_window = camera_window
        self.cid = self.w.cid
        self.buttons: dict[int, tuple[str, float, callable]] = {}
        self.inbox: queue.Queue[str] = queue.Queue()
        self.last_event = ""
        self.status = ""
        self.sort_rep: SortReport | None = None
        self.quit = False
        self.bridge = bridge
        self._text_ids: list[int] = []
        self._last_overlay = 0.0
        self._last_cam = 0.0
        self._frame_img = None
        self._make_buttons()
        self.speed_slider = p.addUserDebugParameter("sim speed (x real time)", 0.1, 4.0, sim.speed,
                                                    physicsClientId=self.cid)
        self._speed = sim.speed
        self._draw_static()
        threading.Thread(target=self._read_stdin, daemon=True).start()

    # ───────────── buttons ─────────────
    def _make_buttons(self):
        add = [
            ("HOME", lambda: self._send("HOME")),
            ("RUN (cradle cycle)", lambda: self._send("RUN")),
            ("STOP", lambda: self._send("STOP")),
            ("RESET (after error / E-stop)", lambda: self._send("RESET")),
            ("TEACH / DONE", lambda: self._send("DONE" if self.sim.fw.state == "TEACH" else "TEACH")),
            ("+ red cube in cradle", lambda: self._cradle("RED")),
            ("+ blue cube in cradle", lambda: self._cradle("BLUE")),
            ("Feeder: RUN + 6 cubes", lambda: self._feed(6)),
            ("Scatter 5 cubes on the table", lambda: self._scatter(5)),
            ("Vision sort the table", self._sort),
            ("Play keyframe slides", self._slides),
            ("Camera snapshot", self._snap),
            ("Clear all cubes", self.w.clear_cubes),
            ("FAULT: next grasp slips", self._fail),
            ("FAULT: E-stop (toggle rail)", self._estop),
            ("FAULT: J1 skips 6 deg", lambda: self._lose(0, 6.0)),
            ("FAULT: J2 switch wire (toggle)", lambda: self._break(1)),
            ("Board reset (like a USB reconnect)", self.sim.reboot),
        ]
        for label, fn in add:
            bid = p.addUserDebugParameter(label, 1, 0, 0, physicsClientId=self.cid)
            self.buttons[bid] = (label, p.readUserDebugParameter(bid, physicsClientId=self.cid), fn)

    def _poll_buttons(self):
        for bid, (label, last, fn) in list(self.buttons.items()):
            v = p.readUserDebugParameter(bid, physicsClientId=self.cid)
            if v != last:
                self.buttons[bid] = (label, v, fn)
                self._say(f"[button] {label}")
                fn()
        s = p.readUserDebugParameter(self.speed_slider, physicsClientId=self.cid)
        if abs(s - self._speed) > 0.01:
            self._speed = s
            self.sim.set_speed(s)

    # ───────────── actions ─────────────
    def _say(self, msg: str):
        print(f"{self.sim.now / 1000:8.2f}s  {msg}", flush=True)

    def _send(self, line: str):
        self.sim.send(line)

    def _cradle(self, colour):
        if self.w.place_in_cradle(colour) is None:
            self._say("# the cradle is not empty")

    def _feed(self, n):
        if self.sim.fw.state != "IDLE":
            return self._say("# feeder: wait for IDLE")
        self._send("RUN")
        self.sim.start(feeder(self.sim, n, seed=None), "feeder")

    def _scatter(self, n):
        placed = scatter(self.sim, n, seed=None)
        self._say(f"# {len(placed)} cubes scattered")

    def _sort(self):
        if "sort" in self.sim.busy_scripts():
            return self._say("# already sorting")
        self.sort_rep = SortReport()
        self.sim.start(vision_sort(self.sim, self.cam, self.sort_rep, on_look=self._on_look), "sort")

    def _slides(self):
        self.sim.start(play(self.sim, DEMO, on_slide=lambda i, n, s: self._say(f"# slide {i}/{n}: {s.label}")),
                       "slides")

    def _on_look(self, rep: SortReport):
        if self.camera_window and rep.frame is not None:
            self._frame_img = annotate(rep.frame, self.cam, rep.seen, f"vision sort: {rep.text}")

    def _snap(self):
        f = self.cam.capture()
        dets = detect(f, self.cam)
        for d in dets:
            self._say(f"# saw {d.colour} at {d.x:.1f}, {d.y:.1f} yaw {d.yaw:+.1f}" + (f" [{d.reason}]" if d.reason else ""))
        self._frame_img = np.hstack([annotate(f, self.cam, dets, "snapshot"), mask_view(f)])
        if not self.camera_window:
            cv2.imwrite("camera_snapshot.png", self._frame_img)
            self._say("# saved camera_snapshot.png")

    def _fail(self):
        self.w.fail_next_pick()

    def _estop(self):
        self.w.set_power(not self.w.power)
        self._say(f"# motor rail {'ON' if self.w.power else 'OFF (E-stop)'}")

    def _lose(self, j, deg):
        self.w.lose_steps(j, deg)

    def _break(self, j):
        self.w.break_switch(j, not self.w.broken_switch[j])
        self._say(f"# J{j + 1} switch wire {'cut' if self.w.broken_switch[j] else 'repaired'}")

    # ───────────── console ─────────────
    def _read_stdin(self):
        for line in sys.stdin:
            self.inbox.put(line.strip())

    def _console(self):
        while not self.inbox.empty():
            line = self.inbox.get()
            if not line:
                continue
            if not line.startswith("!"):
                self.sim.send(line)
                continue
            cmd, *args = line[1:].split()
            cmd = cmd.lower()
            try:
                if cmd == "help":
                    print(HELP)
                elif cmd in ("red", "blue"):
                    self._cradle(cmd.upper())
                elif cmd == "scatter":
                    self._scatter(int(args[0]) if args else 5)
                elif cmd == "feed":
                    self._feed(int(args[0]) if args else 6)
                elif cmd == "sort":
                    self._sort()
                elif cmd == "slides":
                    self._slides()
                elif cmd == "clear":
                    self.w.clear_cubes()
                elif cmd == "fail":
                    self._fail()
                elif cmd == "estop":
                    self._estop()
                elif cmd == "lose":
                    self._lose(int(args[0]) - 1, float(args[1]))
                elif cmd == "break":
                    self._break(int(args[0]) - 1)
                elif cmd == "snap":
                    self._snap()
                elif cmd == "speed":
                    self.sim.set_speed(float(args[0]))
                elif cmd == "reset":
                    self.sim.reboot()
                elif cmd == "bins":
                    self._say(f"# bins {self.w.bin_counts()}")
                elif cmd == "quit":
                    self.quit = True
                else:
                    print("unknown sim command, try !help")
            except (IndexError, ValueError):
                print("bad arguments, try !help")

    # ───────────── overlay ─────────────
    def _draw_static(self):
        """the camera's field of view on the table, and labels"""
        c, e = self.cam, self.cam.eye
        along_x = e[2] * math.tan(math.radians(c.fov / 2))      # image up = robot +x (vertical FOV)
        along_y = along_x * c.w / c.h
        corners = [[e[0] + sx * along_x, e[1] + sy * along_y, 0.001] for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1))]
        for k in range(4):
            p.addUserDebugLine(corners[k], corners[(k + 1) % 4], [0.4, 0.7, 1.0], 1, physicsClientId=self.cid)
            p.addUserDebugLine(e, corners[k], [0.4, 0.7, 1.0], 0.5, physicsClientId=self.cid)
        p.addUserDebugText("Bin A (red)", [0, FW.CELL_BIN_Y / 1000, 0.09], [1, 0.6, 0.6], 1.1, physicsClientId=self.cid)
        p.addUserDebugText("Bin B (blue)", [0, -FW.CELL_BIN_Y / 1000, 0.09], [0.6, 0.7, 1], 1.1, physicsClientId=self.cid)
        p.addUserDebugText("cradle", [FW.CELL_CRADLE_X / 1000, 0.05, 0.03], [0.85, 0.85, 0.85], 1.0, physicsClientId=self.cid)

    def _overlay(self):
        fw = self.sim.fw
        bins = self.w.bin_counts()
        tip = fw.where()
        lines = [
            ("UNO RESETTING (bootloader)" if self.sim.booting else
             f"{fw.state}  err {ERRORS[fw.err]}  homed {int(fw.homed)}  running {int(fw.running)}  cycles {fw.cycles}"),
            f"tip {tip.x:.0f} {tip.y:.0f} {tip.z:.0f} roll {tip.roll:.0f}   grip {fw.q[6]:.0f} mm"
            f"   IR {int(fw.ir())}  colour {fw.last_reading}",
            f"bins  A {bins['A']}  B {bins['B']}" + (f"  (wrong: {bins['A_wrong'] + bins['B_wrong']})"
                                                     if bins['A_wrong'] + bins['B_wrong'] else "")
            + f"   switches {''.join(str(int(b)) for b in fw.limits())}   rail {'ON' if self.w.power else 'OFF'}",
            f"last: {self.last_event}",
        ]
        if self.sort_rep and "sort" in self.sim.busy_scripts():
            lines.append(f"vision sort: {self.sort_rep.text}")
        if self.bridge:
            lines.append(f"{self.bridge.describe()}  (in {self.bridge.lines_in} / out {self.bridge.lines_out} lines)")
        base = [-0.30, 0.0, 0.66]
        for k, txt in enumerate(lines):
            kw = {"replaceItemUniqueId": self._text_ids[k]} if k < len(self._text_ids) else {}
            tid = p.addUserDebugText(txt, [base[0], base[1], base[2] - 0.028 * k], [0.95, 0.95, 0.95], 1.15,
                                     physicsClientId=self.cid, **kw)
            if k >= len(self._text_ids):
                self._text_ids.append(tid)
        for k in range(len(lines), len(self._text_ids)):
            p.addUserDebugText("", [0, 0, 0], replaceItemUniqueId=self._text_ids[k], physicsClientId=self.cid)

    # ───────────── main loop ─────────────
    def on_line(self, ms: int, line: str):
        if line.startswith(("EV ", "ERR ", "#")):
            self.last_event = line
        self._say(line)

    def loop(self):
        print(HELP, flush=True)
        self.sim.set_speed(self.sim.speed)
        while not self.quit and p.isConnected(self.cid):
            self.sim.tick()
            now = time.perf_counter()
            if now - self._last_overlay > 0.1:
                self._last_overlay = now
                try:
                    self._poll_buttons()
                    self._console()
                    self._overlay()
                except p.error:
                    break
            if self.camera_window and self._frame_img is not None and now - self._last_cam > 0.1:
                self._last_cam = now
                cv2.imshow("ras-arm overhead camera", self._frame_img)
                if cv2.waitKey(1) & 0xFF == 27:
                    self._frame_img = None
                    cv2.destroyAllWindows()
        cv2.destroyAllWindows()
