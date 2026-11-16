"""The physical workcell in PyBullet: the arm (from robot_urdf), the cell, the cubes and the sensors.

The world is the firmware's HAL: the firmware twin (firmware.py) only sees what the board sees.
- Steppers are open loop. The firmware counts steps (its joint angle q); the motor sits at
  q + offset. offset is unknown at power-up (the arm is wherever it was left), changes when steps
  are lost, and is re-learned by homing (firmware calls rezero() when a switch clicks).
- Servos are absolute: they go where they are told, no offset.
- E-stop cuts the motor rail: every joint goes limp (only detent / gearbox friction holds it) and the
  steppers re-energise wherever the arm sagged to.
- Sensors: NC limit switches (from the real joint angle), the E18 IR beam (a ray across the cradle
  pocket), the TCS34725 under the cradle (LED light reflected by whatever cube sits on it).
Units: the PyBullet world is in m / rad; this class speaks mm / deg like the firmware.
"""
from __future__ import annotations

import math
import random
import tempfile
from pathlib import Path

import pybullet as p

from .config import FW, SIM
from .robot_urdf import write_arm_urdf

MM = 0.001
D2R = math.pi / 180
STEPS_PER_DEG = [FW.SPD_J1, FW.SPD_J2, FW.SPD_J3]
ARM_JOINTS = ["J1", "J2", "J3", "J4", "J5", "J6"]


class World:
    def __init__(self, gui: bool = False, seed: int = 0, urdf_dir: Path | None = None):
        self.gui = gui
        self.rng = random.Random(seed)
        self.cid = p.connect(p.GUI if gui else p.DIRECT)
        # one step() = one firmware tick (5 ms) = 5 plain 1 ms physics steps. Not Bullet's numSubSteps: with it,
        # force-limited motors hold far more than their limit (a limp arm stops sagging on an E-stop)
        self.dt = SIM.tick_ms / 1000
        self._substeps = max(1, round(SIM.tick_ms / SIM.dt_ms))
        p.resetSimulation(physicsClientId=self.cid)
        p.setTimeStep(SIM.dt_ms / 1000, physicsClientId=self.cid)
        p.setGravity(0, 0, -9.81, physicsClientId=self.cid)
        p.setPhysicsEngineParameter(numSolverIterations=SIM.solver_iterations, enableConeFriction=1,
                                    physicsClientId=self.cid)
        self._q_cache: list[float] | None = None
        if gui:
            for flag in (p.COV_ENABLE_RGB_BUFFER_PREVIEW, p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,
                         p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW):
                p.configureDebugVisualizer(flag, 0, physicsClientId=self.cid)
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1, physicsClientId=self.cid)
            p.resetDebugVisualizerCamera(1.35, 50, -24, [0.08, 0.0, 0.17], physicsClientId=self.cid)

        # the floor under the bench: neutral grey (PyBullet's stock plane is blue, which the camera would see)
        floor_z = -SIM.plate[4] * MM
        self.plane = p.createMultiBody(
            0, p.createCollisionShape(p.GEOM_PLANE, physicsClientId=self.cid),
            p.createVisualShape(p.GEOM_BOX, halfExtents=[3, 3, 0.001], rgbaColor=[0.42, 0.42, 0.40, 1],
                                physicsClientId=self.cid),
            [0, 0, floor_z], physicsClientId=self.cid)
        self.cell: list[int] = []
        self._build_cell()

        urdf_dir = urdf_dir or Path(tempfile.gettempdir()) / "ras_sim"
        urdf = write_arm_urdf(urdf_dir / "ras_arm.urdf")
        self.arm = p.loadURDF(str(urdf), [0, 0, 0], useFixedBase=True,
                              flags=p.URDF_USE_INERTIA_FROM_FILE | ((p.URDF_USE_SELF_COLLISION
                              | p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT) if SIM.self_collision else 0),
                              physicsClientId=self.cid)
        self.jidx: dict[str, int] = {}
        for i in range(p.getNumJoints(self.arm, physicsClientId=self.cid)):
            self.jidx[p.getJointInfo(self.arm, i, physicsClientId=self.cid)[1].decode()] = i
        self.arm_idx = [self.jidx[n] for n in ARM_JOINTS]
        self.finger_idx = [self.jidx["grip_left"], self.jidx["grip_right"]]
        self.tip_link = self.jidx["tool_tip_joint"]
        for i in self.finger_idx:
            p.changeDynamics(self.arm, i, lateralFriction=SIM.finger_friction, spinningFriction=0.002,
                             physicsClientId=self.cid)

        self.offset = [0.0] * 6            # motor position − firmware angle (deg); servos stay 0
        self.power = True
        self.drivers_enabled = True        # EN low; False while the UNO is in reset (10 k pull-ups, rev F)
        self._steppers_live = True         # what drive() last applied
        self.broken_switch = [False, False, False]
        self.cubes: dict[int, str] = {}    # body id → "RED" / "BLUE"
        self._slip_next = False
        self._slipping = False
        self._ir_line = None
        self._ir_drawn: bool | None = None
        self._servo: list[float] | None = None     # where each servo (J4, J5, J6, grip) has slewed to
        self.set_true_pose([0, 0, 0, 0, 0, 0], FW.GRIP_OPEN_MM)

    # ───────────── the cell ─────────────
    def _box(self, size_mm, pos_mm, rgba, collide=True, yaw=0.0) -> int:
        half = [s * MM / 2 for s in size_mm]
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half, physicsClientId=self.cid) if collide else -1
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=rgba, physicsClientId=self.cid)
        body = p.createMultiBody(0, col, vis, [c * MM for c in pos_mm], p.getQuaternionFromEuler([0, 0, yaw]),
                                 physicsClientId=self.cid)
        self.cell.append(body)
        return body

    def _build_cell(self):
        px, py, sx, sy, th = SIM.plate
        self._box((sx, sy, th), (px, py, -th / 2), (0.16, 0.18, 0.22, 1))
        # the V-cradle (P14): a block the cube sits on, walls that centre it, the TCS34725 in its floor
        cx, top = FW.CELL_CRADLE_X, SIM.cradle_top
        grey = (0.36, 0.40, 0.47, 1)
        if top > 0:
            self._box((*SIM.cradle_block, top), (cx, 0, top / 2), grey)
        t, fh = SIM.cradle_fence
        inner = SIM.cube_size / 2 + SIM.cradle_clear                          # walls this far from the centre
        for s in (-1, 1):
            self._box((2 * inner + 2 * t, t, fh), (cx, s * (inner + t / 2), top + fh / 2), grey)
        self._box((t, 2 * inner, fh), (cx + inner + t / 2, 0, top + fh / 2), grey)   # back stop, far side
        self._box((14, 14, 0.6), (cx, 0, top + 0.3), (0.61, 0.35, 0.71, 1), collide=False)   # TCS34725, flush
        self._box((14, 10, 18), (cx, -(inner + t + 7), SIM.ir_beam_z), (0.10, 0.10, 0.12, 1))  # E18 beside it
        B, W, Hb = SIM.bin_size, SIM.bin_wall, SIM.bin_height
        for y, rgba in ((FW.CELL_BIN_Y, (0.62, 0.17, 0.17, 0.9)), (-FW.CELL_BIN_Y, (0.15, 0.25, 0.55, 0.9))):
            self._box((B, W, Hb), (0, y + B / 2, Hb / 2), rgba)
            self._box((B, W, Hb), (0, y - B / 2, Hb / 2), rgba)
            self._box((W, B, Hb), (B / 2, y, Hb / 2), rgba)
            self._box((W, B, Hb), (-B / 2, y, Hb / 2), rgba)

    # ───────────── cubes ─────────────
    def spawn_cube(self, colour: str, x: float, y: float, yaw_deg: float = 0.0, z: float | None = None) -> int:
        h = SIM.cube_size / 2
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[h * MM] * 3, physicsClientId=self.cid)
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[h * MM] * 3, rgbaColor=SIM.cube_rgba[colour],
                                  physicsClientId=self.cid)
        body = p.createMultiBody(SIM.cube_mass, col, vis, [x * MM, y * MM, (h + 0.2 if z is None else z) * MM],
                                 p.getQuaternionFromEuler([0, 0, yaw_deg * D2R]), physicsClientId=self.cid)
        # resting cubes sleep (a pile in a bin would otherwise cost 50× in contact solving); any contact wakes them
        p.changeDynamics(body, -1, lateralFriction=SIM.cube_friction, spinningFriction=0.002, rollingFriction=0,
                         restitution=0.0, activationState=p.ACTIVATION_STATE_ENABLE_SLEEPING,
                         physicsClientId=self.cid)
        self.cubes[body] = colour
        return body

    def place_in_cradle(self, colour: str) -> int | None:
        if self.cradle_cube() is not None:
            return None
        return self.spawn_cube(colour, FW.CELL_CRADLE_X, 0.0, z=SIM.cradle_top + SIM.cube_size / 2 + 0.3)

    def remove_cube(self, body: int):
        if body in self.cubes:
            p.removeBody(body, physicsClientId=self.cid)
            del self.cubes[body]

    def clear_cubes(self):
        for b in list(self.cubes):
            self.remove_cube(b)

    def cube_pose(self, body: int) -> tuple[float, float, float, float]:
        """x, y, z (mm, centre) and yaw (deg)"""
        pos, orn = p.getBasePositionAndOrientation(body, physicsClientId=self.cid)
        return pos[0] / MM, pos[1] / MM, pos[2] / MM, math.degrees(p.getEulerFromQuaternion(orn)[2])

    def where(self, body: int) -> str:
        """cradle | bin_A | bin_B | gripper | table | floor"""
        x, y, z, _ = self.cube_pose(body)
        h = SIM.cube_size / 2
        if z < -2:
            return "floor"
        on_cradle = abs(x - FW.CELL_CRADLE_X) < SIM.cradle_block[0] / 2 and abs(y) < SIM.cradle_block[1] / 2
        rest = h + (SIM.cradle_top if on_cradle else 0.0)                     # centre height when resting
        if z > rest + 5 and self._touching_fingers(body):
            return "gripper"
        inner = SIM.bin_size / 2 - SIM.bin_wall
        for name, by in (("bin_A", FW.CELL_BIN_Y), ("bin_B", -FW.CELL_BIN_Y)):
            if abs(x) < inner and abs(y - by) < inner:   # any height: cubes dropped on one spot stack up
                return name
        if z < SIM.cradle_top + h + 5 and abs(x - FW.CELL_CRADLE_X) < 12 and abs(y) < 12:
            return "cradle"
        return "table"

    def _touching_fingers(self, body) -> bool:
        return any(p.getContactPoints(self.arm, body, i, -1, physicsClientId=self.cid) for i in self.finger_idx)

    def cradle_cube(self) -> int | None:
        return next((b for b in self.cubes if self.where(b) == "cradle"), None)

    def bin_counts(self) -> dict[str, int]:
        out = {"A": 0, "B": 0, "A_wrong": 0, "B_wrong": 0}
        for b, colour in self.cubes.items():
            w = self.where(b)
            if w in ("bin_A", "bin_B"):
                k = w[-1]
                out[k] += 1
                if (k == "A") != (colour == "RED"):
                    out[k + "_wrong"] += 1
        return out

    # ───────────── the arm ─────────────
    def set_true_pose(self, q_deg, grip_mm):
        """teleport the arm (power-up pose, tests); the firmware is not told"""
        for i, v in zip(self.arm_idx, q_deg):
            p.resetJointState(self.arm, i, v * D2R, physicsClientId=self.cid)
        for i in self.finger_idx:
            p.resetJointState(self.arm, i, self._finger_target(grip_mm), physicsClientId=self.cid)
        self._q_cache = None
        self._servo = None

    def true_q(self) -> list[float]:
        """the real joint angles (deg), read once per physics step"""
        if self._q_cache is None:
            self._q_cache = [s[0] / D2R for s in p.getJointStates(self.arm, self.arm_idx, physicsClientId=self.cid)]
        return self._q_cache

    def true_grip(self) -> float:
        s = sum(p.getJointState(self.arm, i, physicsClientId=self.cid)[0] for i in self.finger_idx) / 2
        return 2 * (s / MM + SIM.finger_pad)

    def tip(self) -> tuple[float, float, float]:
        pos = p.getLinkState(self.arm, self.tip_link, computeForwardKinematics=True, physicsClientId=self.cid)[4]
        return pos[0] / MM, pos[1] / MM, pos[2] / MM

    @staticmethod
    def _finger_target(grip_mm: float) -> float:
        return max(0.0, grip_mm / 2 - SIM.finger_pad) * MM

    def drive(self, q_fw):
        """apply the firmware's joint commands (deg, [6] = grip mm) for the next physics steps.
        With drivers_enabled False (the UNO in reset: EN pulled up) the steppers are off and the servos keep
        the last pulses the PCA9685 was given."""
        live = self.power and self.drivers_enabled
        if live and not self._steppers_live:           # rail back / EN low again: steppers hold where the arm is
            tq = self.true_q()
            for j in range(FW.N_STEPPERS):
                self.offset[j] = tq[j] - q_fw[j]
        self._steppers_live = live
        idx = self.arm_idx + self.finger_idx
        if not self.power:                              # per joint: the array call does not honour these forces
            for i, force in zip(idx, list(SIM.unpowered_hold) + [0.5, 0.5]):
                p.setJointMotorControl2(self.arm, i, p.VELOCITY_CONTROL, targetVelocity=0, force=force,
                                        physicsClientId=self.cid)
            self._servo = None                          # servos re-read their position at power-up
            return
        if not live:            # drivers disabled: the steppers go limp, the servos keep the last pulses (q_fw frozen)
            for i, force in zip(self.arm_idx[:3], SIM.unpowered_hold[:3]):
                p.setJointMotorControl2(self.arm, i, p.VELOCITY_CONTROL, targetVelocity=0, force=force,
                                        physicsClientId=self.cid)
        if self._servo is None:
            tq = self.true_q()
            self._servo = [tq[3], tq[4], tq[5], self.true_grip()]
        for k, (target, rate) in enumerate(zip(q_fw[3:7], [SIM.servo_dps] * 3 + [SIM.grip_mm_s])):
            step = rate * self.dt                       # a servo slews toward its command at its own top speed
            self._servo[k] += max(-step, min(step, target - self._servo[k]))
        targets = []
        for j in range(6):
            t = (q_fw[j] + self.offset[j]) if j < FW.N_STEPPERS else self._servo[j - 3]
            if j < FW.N_STEPPERS:                       # the motor moves in whole (micro)steps
                t = round(t * STEPS_PER_DEG[j]) / STEPS_PER_DEG[j]
            targets.append(t * D2R)
        self._update_slip(q_fw[6])
        f = self._finger_target(self._servo[3])
        forces = [SIM.effort[n] for n in ARM_JOINTS] + [SIM.finger_force] * 2
        first = 0 if live else FW.N_STEPPERS            # steppers off: only the servos are driven
        p.setJointMotorControlArray(self.arm, idx[first:], p.POSITION_CONTROL, targetPositions=(targets + [f, f])[first:],
                                    forces=forces[first:], positionGains=[0.3] * (8 - first),
                                    velocityGains=[1.0] * (8 - first), physicsClientId=self.cid)

    def rezero(self, j: int, q_old: float, q_new: float):
        """the firmware overwrote stepper j's step counter (homing): the motor does not move"""
        self.offset[j] += q_old - q_new

    def step(self, n: int = 1):
        """advance n firmware ticks (SIM.tick_ms each)"""
        for _ in range(n * self._substeps):
            p.stepSimulation(physicsClientId=self.cid)
        self._q_cache = None

    # ───────────── sensors (what the board reads) ─────────────
    def power_ok(self) -> bool:
        return self.power

    def limit(self, j: int) -> bool:
        """NC switch: pressed OR broken wire reads 'pressed'"""
        if self.broken_switch[j]:
            return True
        a = self.true_q()[j]
        sw = FW.HOME_SWITCH_DEG[j]
        return a <= sw if FW.HOME_DIR[j] < 0 else a >= sw

    def ir_raw(self) -> bool:
        """E18 beam across the cradle, between the centring walls: anything in it blocks it"""
        inner = SIM.cube_size / 2 + SIM.cradle_clear
        cx, z, half = FW.CELL_CRADLE_X * MM, SIM.ir_beam_z * MM, (inner - 0.5) * MM
        hit = p.rayTest([cx, -half, z], [cx, half, z], physicsClientId=self.cid)[0]
        blocked = hit[0] >= 0
        if self.gui and blocked != self._ir_drawn:              # redraw only on change
            self._ir_drawn = blocked
            colour = [0.2, 0.9, 0.3] if blocked else [0.9, 0.2, 0.2]
            kw = {"replaceItemUniqueId": self._ir_line} if self._ir_line is not None else {}
            start = -(inner + SIM.cradle_fence[0] + 7) * MM
            self._ir_line = p.addUserDebugLine([cx, start, z], [cx, half, z], colour, 2,
                                               physicsClientId=self.cid, **kw)
        return blocked

    def colour_sample(self) -> tuple[int, int, int, int]:
        """one TCS34725 integration (raw r, g, b, clear): LED light bounced off the cube above the sensor,
        which sits in the cradle floor"""
        h = SIM.cube_size / 2
        refl, d = (0.0, 0.0, 0.0), 1e9
        for body, colour in self.cubes.items():
            x, y, z, _ = self.cube_pose(body)
            gap = z - h - SIM.cradle_top                       # cube bottom above the sensor
            if abs(x - FW.CELL_CRADLE_X) < h and abs(y) < h and gap < d:
                d, refl = max(0.0, gap), SIM.cube_rgba[colour][:3]
        f = 1.0 / (1.0 + (d / SIM.colour_falloff_mm) ** 2) if d < 1e8 else 0.0
        rgb = [max(0.0, (a + SIM.colour_led_gain * k * f) * (1 + self.rng.gauss(0, SIM.colour_noise)))
               for a, k in zip(SIM.colour_ambient, refl)]
        return int(rgb[0]), int(rgb[1]), int(rgb[2]), int(sum(rgb))

    # ───────────── faults ─────────────
    def fail_next_pick(self):
        """the next grasp slips: frictionless pads until the gripper opens again"""
        self._slip_next = True

    def _update_slip(self, grip_cmd: float):
        if self._slip_next and grip_cmd < FW.GRIP_CLOSED_MM + 2 and not self._slipping:
            self._slip_next, self._slipping = False, True
            for i in self.finger_idx:
                p.changeDynamics(self.arm, i, lateralFriction=0.0, physicsClientId=self.cid)
        elif self._slipping and grip_cmd > FW.GRIP_CLOSED_MM + 8:
            self._slipping = False
            for i in self.finger_idx:
                p.changeDynamics(self.arm, i, lateralFriction=SIM.finger_friction, physicsClientId=self.cid)

    def lose_steps(self, j: int, deg: float):
        """the motor skips: the arm is deg away from where the firmware thinks it is"""
        self.offset[j] += deg

    def break_switch(self, j: int, broken: bool = True):
        self.broken_switch[j] = broken

    def set_power(self, ok: bool):
        self.power = ok

    def close(self):
        if p.isConnected(self.cid):
            p.disconnect(self.cid)
