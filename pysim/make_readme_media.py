"""Regenerate the README's pictures from the simulation (docs/images/).

    pip install pillow            # only for the GIF; not needed to run the sim
    python make_readme_media.py

Runs the real thing headless: the firmware twin homes, then sorts a red and a blue cube through the RUN
cycle (IR beam → colour → pick → verify → place → return), and the overhead camera looks at loose cubes.
Every picture is a PyBullet render taken at that moment, labelled with the state the twin was in.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np
import pybullet as p

sys.path.insert(0, str(Path(__file__).resolve().parent))

from ras_sim.app import Simulation  # noqa: E402
from ras_sim.sorter import SortReport, scatter, vision_sort  # noqa: E402
from ras_sim.vision import OverheadCamera, annotate, mask_view  # noqa: E402

OUT = Path(__file__).resolve().parents[1] / "docs" / "images"
VIEWS = {   # yaw, pitch, distance (m), target (m)
    "cell": (25, -29, 1.25, (0.06, 0.02, 0.18)),
    "cradle": (40, -18, 0.50, (0.20, 0.0, 0.10)),
    "bin_A": (120, -35, 0.85, (0.0, 0.20, 0.12)),
    "bin_B": (-60, -35, 0.85, (0.05, -0.20, 0.12)),
}


def render(sim, view, size=(960, 640)):
    yaw, pitch, dist, target = VIEWS[view]
    cid = sim.world.cid
    vm = p.computeViewMatrixFromYawPitchRoll(target, dist, yaw, pitch, 0, 2, physicsClientId=cid)
    pm = p.computeProjectionMatrixFOV(40, size[0] / size[1], 0.05, 5, physicsClientId=cid)
    img = p.getCameraImage(size[0], size[1], vm, pm, renderer=p.ER_TINY_RENDERER, lightDirection=[0.5, 0.4, 1.2],
                           shadow=1, physicsClientId=cid)
    rgb = np.reshape(np.asarray(img[2], np.uint8), (size[1], size[0], 4))[:, :, :3]
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)


def label(img, title, sub="", s=None):
    """a dark banner with the firmware state, top left (s: text scale, default from the width)"""
    h = 52 if sub else 34
    s = s or img.shape[1] / 960
    out = img.copy()
    cv2.rectangle(out, (0, 0), (img.shape[1], int(h * s)), (30, 30, 34), -1)
    out = cv2.addWeighted(out, 0.82, img, 0.18, 0)
    cv2.putText(out, title, (int(12 * s), int(24 * s)), cv2.FONT_HERSHEY_SIMPLEX, 0.7 * s, (255, 255, 255), 2, cv2.LINE_AA)
    if sub:
        cv2.putText(out, sub, (int(12 * s), int(44 * s)), cv2.FONT_HERSHEY_SIMPLEX, 0.48 * s, (210, 210, 210), 1, cv2.LINE_AA)
    return out


def save(name, img):
    OUT.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(OUT / name), img)
    print("wrote", OUT / name)


def main():
    sim = Simulation(seed=3)
    sim.run_until(lambda: sim.fw.state == "IDLE", 30000)
    save("01_initial_pose.png", label(render(sim, "cell"), "IDLE: homed, waiting for a cube",
                                      "the firmware twin homed every stepper on its switch (J2, J3, J1) from an unknown pose"))
    raw = []                                            # (frame, state + phase, info) every 100 ms
    tick = {"next": 0}

    def grab():
        if sim.now >= tick["next"]:
            tick["next"] = sim.now + 100
            fw = sim.fw
            phase = fw.seq["phase"] if fw.state in ("PICK", "PLACE") else ""
            raw.append((render(sim, "cell", (640, 427)), f"{fw.state} {phase}".strip(),
                        f"t = {sim.now / 1000:4.1f} s   grip {fw.q[6]:.0f} mm   IR {int(fw.ir())}   colour {fw.last_reading}"))
    sim.on_tick.append(grab)

    # ── a red cube through the whole RUN cycle ──
    sim.world.place_in_cradle("RED")
    tick["next"] = sim.now
    sim.send("RUN")
    sim.run_until(lambda: sim.fw.state == "READ_COLOUR", 5000)
    sim.run_for(60)
    r, g, b, c = sim.world.colour_sample()
    save("02_detecting.png", label(render(sim, "cradle"), "READ_COLOUR: a cube is in the cradle",
                                   f"E18 IR beam blocked; TCS34725 under the cube reads r {r}  g {g}  b {b}  clear {c}"))
    sim.run_until(lambda: sim.fw.state == "PICK", 5000)
    colour = [ln.line for ln in sim.log if ln.line.startswith("EV COLOUR")][-1].split()[-1]
    sim.run_until(lambda: sim.fw.state == "PICK" and sim.fw.seq["phase"] == "up", 15000)
    sim.run_for(400)
    save("04_picking.png", label(render(sim, "cradle"), f"PICK: colour {colour}, the fingers close and lift",
                                 "above -> straight down -> close (force-limited, the cube is held by friction) -> straight up"))
    sim.run_until(lambda: sim.fw.state == "PLACE" and sim.fw.seq["phase"] in ("open", "up"), 20000)
    sim.run_for(250)
    save("05_place_bin_a.png", label(render(sim, "bin_A"), "PLACE: red cube released into bin A",
                                     "transit above the bin -> straight down to the drop height -> open"))
    sim.run_until(lambda: sim.fw.cycles >= 1 and sim.fw.state == "IDLE", 20000)
    sim.on_tick.remove(grab)

    # ── a blue cube into bin B ──
    sim.world.place_in_cradle("BLUE")
    sim.run_until(lambda: sim.fw.state == "PLACE" and sim.fw.seq["phase"] in ("open", "up"), 30000)
    sim.run_for(250)
    save("06_place_bin_b.png", label(render(sim, "bin_B"), "PLACE: blue cube released into bin B",
                                     "same cycle, the colour sensor sent it to the other bin"))
    sim.run_until(lambda: sim.fw.cycles >= 2 and sim.fw.state == "IDLE", 20000)
    sim.send("STOP")

    # ── the full cycle: GIF and key frames ──
    try:
        from PIL import Image
        frames = [Image.fromarray(cv2.cvtColor(label(f, t, i, 0.85), cv2.COLOR_BGR2RGB))
                  .convert("P", palette=Image.ADAPTIVE, colors=128) for f, t, i in raw]
        frames[0].save(OUT / "sorting_cycle.gif", save_all=True, append_images=frames[1:], duration=50, loop=0,
                       optimize=True)
        print("wrote", OUT / "sorting_cycle.gif", f"({len(frames)} frames, 2x speed)")
    except ImportError:
        print("pillow not installed: no GIF (pip install pillow)")
    def first(tag, skip=0):                             # the first frame of a state (+ phase), plus skip
        k = next(n for n, (_, t, _) in enumerate(raw) if t == tag)
        return raw[min(k + skip, len(raw) - 1)]
    steps = [("IDLE", 0, "1  IDLE: waiting"), ("READ_COLOUR", 1, "2  READ_COLOUR: IR + colour"),
             ("PICK close", 2, "3  PICK: fingers close"), ("VERIFY", 1, "4  VERIFY: cradle empty?"),
             ("PLACE open", 2, "5  PLACE: release in the bin"), ("RETURN", 8, "6  RETURN: back home")]
    tiles = [label(cv2.resize(first(tag, k)[0], (480, 320)), title, s=0.75) for tag, k, title in steps]
    save("07_full_cycle.png", np.vstack([np.hstack(tiles[:3]), np.hstack(tiles[3:6])]))

    # ── the overhead camera finding loose cubes ──
    scatter(sim, 4, seed=9)
    sim.run_for(200)
    cam, rep, shots = OverheadCamera(sim.world.cid), SortReport(), []
    sim.start(vision_sort(sim, cam, rep, on_look=lambda r: shots.append((r.frame, r.seen)) if not shots else None),
              "sort")
    sim.run_until(lambda: shots, 20000)
    frame, seen = shots[0]
    save("03_vision_detection.png", np.hstack([annotate(frame, cam, seen, "overhead camera: blobs -> top face -> mm"),
                                               mask_view(frame)]))
    sim.close()


if __name__ == "__main__":
    main()
