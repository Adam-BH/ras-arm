"""Vision sort, host side: look → pick the nearest cube → look again to check it left the table →
drop it in its colour's bin → repeat until the camera sees no reachable cube.

Only the normal protocol (STATUS, MOVE, GRIP): no firmware change, and the same loop would drive the real
arm with a calibrated webcam.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from .app import CommandError, Simulation, cmd, wait
from .config import FW, SIM, Pose
from .kinematics import OK, ik
from .protocol import fmt
from .vision import Detection, OverheadCamera, detect

VIEW_POSE = Pose(40, -200, 170, 0)          # arm parked out of the picture for every snapshot
RETRIES = 2                                  # re-picks after a slip (the firmware's pick_retries)
VERIFY_MM = 15.0                             # still seen this close to where it was → the pick slipped
JIGGLE = [(0, 0, 0), (3, 0, 4), (-3, 0, -4), (0, 3, 0), (0, -3, 0)]   # dx, dy mm, droll deg (decisions R7)
# drop spots around the taught bin pose (mm): the fingers are above the 60 mm walls at drop height, the cube
# lands at least 13 mm inside them
DROP_SLOTS = [(-20, -20), (20, 20), (-20, 20), (20, -20)]
MS = {"view": 1400, "above": 1200, "descend": 700, "settle": 150, "transit": 1600, "drop": 700}


def grasp_roll(x: float, y: float, yaw: float, grip_z: float) -> float | None:
    """fingers square to the cube's faces: roll = yaw + k·90°, the smallest J6 turn that the IK accepts
    at grip height and at SAFE_Z"""
    best = None
    for k in range(-2, 3):
        roll = yaw + 90 * k
        st1, q1 = ik(Pose(x, y, grip_z, roll))
        st2, _ = ik(Pose(x, y, FW.SAFE_Z, roll))
        if st1 == OK and st2 == OK and (best is None or abs(q1[5]) < best[1]):
            best = (roll, abs(q1[5]))
    return best[0] if best else None


@dataclass
class SortReport:
    sorted: int = 0
    bins: dict = field(default_factory=lambda: {"A": 0, "B": 0})
    slips: int = 0
    skipped: int = 0
    out_of_reach: int = 0
    looks: int = 0
    seen: list = field(default_factory=list)       # last snapshot (rejected blobs included)
    frame: object = None
    text: str = ""


def vision_sort(sim: Simulation, cam: OverheadCamera, report: SortReport | None = None, max_picks: int = 16,
                on_look=None):
    rep = report or SortReport()
    st = sim.send("STATUS")
    if not st.text.startswith("IDLE ") or "homed=1" not in st.text or "running=1" in st.text:
        raise CommandError(f"the arm must be IDLE, homed and not running ({st.text})")
    # grip height = the cube's centre + where the taught CUBE pose holds a cube relative to its centre
    # (0 in the design book: fingertips at the centre). The camera measures each cube's top.
    offset = sim.fw.poses["CUBE"].z - (SIM.cradle_top + SIM.cube_size / 2)

    def grip_z(d: Detection) -> float:
        return d.z_top - SIM.cube_size / 2 + offset
    skip: list[Detection] = []

    def look():
        yield from cmd(sim, fmt.move(*_xyzr(VIEW_POSE), MS["view"]))
        yield from wait(sim, MS["settle"])
        rep.frame = cam.capture()
        rep.seen = detect(rep.frame, cam)
        rep.looks += 1
        if on_look:
            on_look(rep)
        return [d for d in rep.seen if d.ok]

    def pick(d: Detection, roll: float, gz: float):
        for attempt in range(1 + RETRIES):
            dx, dy, dr = JIGGLE[attempt]
            x, y, r = d.x + dx, d.y + dy, roll + dr
            rep.text = f"pick {d.colour} at {d.x:.0f}, {d.y:.0f} (try {attempt + 1})"
            yield from cmd(sim, fmt.grip(FW.GRIP_OPEN_MM))
            yield from cmd(sim, fmt.move(x, y, FW.SAFE_Z, r, MS["above"]))
            yield from cmd(sim, fmt.move(x, y, gz + 30, r, MS["descend"]))       # two short joint moves:
            yield from cmd(sim, fmt.move(x, y, gz, r, MS["descend"]))            # the tip stays near vertical
            yield from cmd(sim, fmt.grip(FW.GRIP_CLOSED_MM))
            yield from wait(sim, 150)
            yield from cmd(sim, fmt.move(x, y, FW.SAFE_Z, r, MS["above"]))
            now = yield from look()
            if not any(s.colour == d.colour and math.hypot(s.x - d.x, s.y - d.y) < VERIFY_MM for s in now):
                return True
            rep.slips += 1
        return False

    def place(colour: str):
        k = "A" if colour == "RED" else "B"
        drop = sim.fw.poses["BIN_" + k]
        dx, dy = DROP_SLOTS[rep.bins[k] % len(DROP_SLOTS)]   # spread the drops: one spot builds a tower
        x, y = drop.x + dx, drop.y + dy
        roll = math.degrees(math.atan2(y, x))            # J6 = 0: the cube's yaw does not matter in a bin
        if any(ik(Pose(x, y, z, roll))[0] != OK for z in (drop.z, FW.SAFE_Z + FW.PLACE_ABOVE_DZ)):
            x, y, roll = drop.x, drop.y, drop.roll
        yield from cmd(sim, fmt.move(x, y, FW.SAFE_Z + FW.PLACE_ABOVE_DZ, roll, MS["transit"]))
        yield from cmd(sim, fmt.move(x, y, drop.z, roll, MS["drop"]))
        yield from cmd(sim, fmt.grip(FW.GRIP_OPEN_MM))
        yield from cmd(sim, fmt.move(x, y, FW.SAFE_Z + FW.PLACE_ABOVE_DZ, roll, MS["drop"]))

    for _ in range(max_picks):
        rep.text = "looking"
        cubes = yield from look()
        cubes = [d for d in cubes if not any(s.colour == d.colour and math.hypot(s.x - d.x, s.y - d.y) < VERIFY_MM
                                             for s in skip)]
        todo = [(d, grasp_roll(d.x, d.y, d.yaw, grip_z(d))) for d in cubes]
        rep.out_of_reach = sum(1 for _, r in todo if r is None)
        todo = sorted(((d, r) for d, r in todo if r is not None), key=lambda t: math.hypot(t[0].x, t[0].y))
        if not todo:
            yield from cmd(sim, fmt.grip(FW.GRIP_OPEN_MM))
            rep.text = "table clear" + (f" ({rep.out_of_reach} out of reach)" if rep.out_of_reach else "")
            return rep
        d, roll = todo[0]
        if (yield from pick(d, roll, grip_z(d))):
            yield from place(d.colour)
            rep.sorted += 1
            rep.bins["A" if d.colour == "RED" else "B"] += 1
        else:
            skip.append(d)
            rep.skipped += 1
    rep.text = f"gave up after {max_picks} picks"
    return rep


def _xyzr(p: Pose):
    return p.x, p.y, p.z, p.roll


def scatter(sim: Simulation, n: int, seed: int | None = None, colours=("RED", "BLUE")):
    """n loose cubes in the pick zone, at least 60 mm apart, away from the cradle, random yaw, all
    within the arm's reach (with the fingers square to the faces)"""
    import random
    rng = random.Random(seed)
    placed: list[tuple[float, float]] = []
    tries = 0
    while len(placed) < n and tries < 5000:
        tries += 1
        x, y, yaw = rng.uniform(135, 275), rng.uniform(-145, 145), rng.uniform(-45, 45)
        if math.hypot(x - FW.CELL_CRADLE_X, y) < SIM.cradle_block[0] / 2 + SIM.cube_size + 5 or                 any(math.hypot(x - a, y - b) < max(60, 2 * SIM.cube_size + 5) for a, b in placed):
            continue
        if grasp_roll(x, y, yaw, FW.CELL_GRIP_Z - SIM.cradle_top) is None:      # a cube on the table
            continue
        placed.append((x, y))
        sim.world.spawn_cube(rng.choice(colours), x, y, yaw)
    return placed
