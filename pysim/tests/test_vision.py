"""Overhead camera + OpenCV accuracy, the vision sort loop and the keyframe player."""
import math
import random

from ras_sim.config import Pose
from ras_sim.kinematics import ik
from ras_sim.slides import DEMO, play
from ras_sim.sorter import SortReport, grasp_roll, scatter, vision_sort
from ras_sim.vision import OverheadCamera, detect
from ras_sim.world import World


def test_detection_accuracy():
    w = World()
    cam = OverheadCamera(w.cid)
    w.set_true_pose(ik(Pose(40, -200, 170, 0))[1], 45)              # arm parked out of the picture
    rng = random.Random(5)
    errs, yaw_errs = [], []
    for _ in range(12):
        w.clear_cubes()
        truth = []
        while len(truth) < 4:
            x, y = rng.uniform(135, 275), rng.uniform(-145, 145)
            if all(math.hypot(x - a, y - b) > 60 for a, b, *_ in truth) and math.hypot(x - 220, y) > 50:
                c, yaw = rng.choice(["RED", "BLUE"]), rng.uniform(-45, 45)
                truth.append((x, y, c, yaw))
                w.spawn_cube(c, x, y, yaw)
        w.step(200)
        dets = [d for d in detect(cam.capture(), cam) if d.ok]
        assert len(dets) == 4
        for x, y, c, yaw in truth:
            d = min((d for d in dets if d.colour == c), key=lambda d: math.hypot(d.x - x, d.y - y))
            errs.append(math.hypot(d.x - x, d.y - y))
            yaw_errs.append(abs((d.yaw - yaw + 45) % 90 - 45))
    w.close()
    assert max(errs) < 2.0 and max(yaw_errs) < 3.0, (max(errs), max(yaw_errs))


def test_cradle_walls_and_bins_are_not_cubes():
    w = World()
    cam = OverheadCamera(w.cid)
    w.set_true_pose(ik(Pose(40, -200, 170, 0))[1], 45)
    w.step(50)
    assert not [d for d in detect(cam.capture(), cam) if d.ok]
    w.close()


def test_grasp_roll_is_square_to_the_cube():
    r = grasp_roll(200, 60, 30, 24)
    assert r is not None and (r - 30) % 90 == 0
    assert grasp_roll(330, 150, 0, 24) is None                       # out of reach


def test_vision_sort_clears_the_table(sim):
    scatter(sim, 5, seed=2)
    sim.run_for(200)
    sim.world.fail_next_pick()
    rep = SortReport()
    sim.run_script(vision_sort(sim, OverheadCamera(sim.world.cid), rep), "sort")
    bins = sim.world.bin_counts()
    assert rep.text == "table clear" and rep.sorted == 5 and rep.slips == 1
    assert bins["A"] + bins["B"] == 5 and bins["A_wrong"] + bins["B_wrong"] == 0


def test_keyframe_demo_moves_the_cube_to_bin_a(sim):
    b = sim.world.place_in_cradle("RED")
    assert sim.run_script(play(sim, DEMO), "slides") == len(DEMO)
    assert sim.world.where(b) == "bin_A"
