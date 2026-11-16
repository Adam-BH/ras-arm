"""The model matches the firmware: config.h parse, kinematics vs the golden vectors, URDF vs firmware FK."""
import json
import math
import random

import pybullet as p
import pytest

from ras_sim.config import CELL, FW, REPO_ROOT, Pose
from ras_sim.kinematics import LIMIT, OK, UNREACHABLE, fk, ik
from ras_sim.world import World


def test_config_h_parsed():
    assert (FW.L1, FW.L2, FW.L3, FW.H_SHOULDER) == (180, 140, 110, 120)
    assert FW.JOINT_MIN == [-100, -20, 20, -90, -100, -90, 0]
    assert FW.JOINT_MAX == [100, 110, 150, 90, 100, 90, 50]
    assert FW.HOME_SWITCH_DEG == [-100, -20, 150] and FW.HOME_DIR == [-1, -1, 1]
    assert FW.SPD_J2 == pytest.approx(69.07, abs=0.01)
    assert FW.HOMING_SLOW_MAX == FW.HOMING_BACKOFF + 3
    assert FW.PIN_LIMIT_J3 == 11                                   # the #ifndef branch
    assert (FW.J1, FW.JG, FW.NJ) == (0, 6, 7)
    assert FW.FW_VERSION == "1.0.0"
    assert CELL["BIN_B"] == Pose(0, -220, 78)


def test_kinematics_matches_golden_vectors():
    """tools/ik_reference.py vectors: the same ones the C firmware and the TS twin are tested against"""
    vecs = json.loads((REPO_ROOT / "sim/src/core/__tests__/ik_vectors.json").read_text())
    assert len(vecs) > 500
    status = {0: OK, 1: UNREACHABLE, 2: LIMIT}
    for v in vecs:
        st, q = ik(Pose(*v["pose"]))
        assert st == status[v["status"]], v
        if st == UNREACHABLE:
            continue
        assert max(abs(a - b) for a, b in zip(q, v["joints"])) < 0.05, v
        tip = fk(q)
        assert math.dist((tip.x, tip.y, tip.z), v["tip"][:3]) < 0.1, v


@pytest.fixture(scope="module")
def world():
    w = World()
    yield w
    w.close()


def test_urdf_tip_is_the_firmware_tip(world):
    rng = random.Random(1)
    n = 0
    while n < 300:
        q = [rng.uniform(FW.JOINT_MIN[j], FW.JOINT_MAX[j]) for j in range(6)]
        q[3], q[4] = 0.0, 180 - q[1] - q[2]                         # the firmware's vertical gripper
        if not FW.JOINT_MIN[4] <= q[4] <= FW.JOINT_MAX[4]:
            continue
        n += 1
        world.set_true_pose(q, 45)
        f = fk(q)
        assert math.dist(world.tip(), (f.x, f.y, f.z)) < 0.01


def test_gripper_points_down_and_fingers_follow_roll(world):
    for pose in (CELL["CUBE"], Pose(150, 80, 24, 30), Pose(0, 220, 78, 0)):
        st, q = ik(pose)
        assert st == OK
        world.set_true_pose(q, 45)
        orn = p.getLinkState(world.arm, world.tip_link, computeForwardKinematics=True, physicsClientId=world.cid)[5]
        m = p.getMatrixFromQuaternion(orn)
        assert (m[2], m[5], m[8]) == pytest.approx((0, 0, -1), abs=1e-6)            # tool z = world down
        assert math.degrees(math.atan2(m[4], m[1])) == pytest.approx(pose.roll + 90, abs=1e-4)  # fingers ⟂ roll


def test_hard_stops_sit_past_the_homing_switches(world):
    world.set_true_pose([0, 0, 90, 0, 0, 0], 45)
    for _ in range(600):                                             # 3 s
        world.drive([0, -60, 90, 0, 0, 0, 45])
        world.step()
    j2 = world.true_q()[1]
    assert j2 < FW.HOME_SWITCH_DEG[1] and j2 == pytest.approx(FW.HOME_SWITCH_DEG[1] - 3, abs=0.2)
    assert world.limit(1)
