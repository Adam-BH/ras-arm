"""The model matches the firmware and the design book: config.h parse, kinematics vs the golden vectors
and vs the design book's Table 4, URDF vs firmware FK."""
import json
import math
import random

import pybullet as p
import pytest

from ras_sim.config import CELL, DESIGN_BOOK, FW, FW_RAW, PROFILE, REPO_ROOT, SIM, Pose
from ras_sim.kinematics import LIMIT, OK, UNREACHABLE, Geometry, fk, ik
from ras_sim.world import World


def test_config_h_parsed():
    """config.h as it is in the repo"""
    assert (FW_RAW.L1, FW_RAW.L2, FW_RAW.L3, FW_RAW.H_SHOULDER) == (180, 140, 110, 120)
    assert FW_RAW.JOINT_MIN == [-100, -20, 20, -90, -100, -90, 0]
    assert FW_RAW.JOINT_MAX == [100, 110, 150, 90, 100, 90, 50]
    assert FW_RAW.HOME_SWITCH_DEG == [-100, -20, 150] and FW_RAW.HOME_DIR == [-1, -1, 1]
    assert FW_RAW.SPD_J2 == pytest.approx(69.07, abs=0.01)
    assert FW_RAW.HOMING_SLOW_MAX == FW_RAW.HOMING_BACKOFF + 3
    assert FW_RAW.PIN_LIMIT_J3 == 11                               # the #ifndef branch
    assert (FW_RAW.J1, FW_RAW.JG, FW_RAW.NJ) == (0, 6, 7)
    assert FW_RAW.FW_VERSION == "1.0.0"


def test_design_book_profile():
    """the design book's appendix A.1 constants, applied on top of config.h (default profile)"""
    assert PROFILE == "design"
    assert (FW.L1, FW.L2, FW.L3, FW.H_SHOULDER) == (165, 190, 200, 300)
    assert FW.SPD_J2 == pytest.approx(126.62, abs=0.01) and FW.SPD_J3 == pytest.approx(28.89, abs=0.01)
    assert FW.SERVO_CH == [3, 0, 1, 2]                             # rev F: J4, J5, J6, gripper
    assert CELL["CUBE"] == Pose(220, 0, 37.5) and CELL["BIN_B"] == Pose(0, -220, 88)
    assert FW.JOINT_MIN == FW_RAW.JOINT_MIN and FW.PIN_LIMIT_J3 == FW_RAW.PIN_LIMIT_J3   # the rest is config.h
    assert set(DESIGN_BOOK) <= set(FW.__dict__)
    # Table 5 rated torques at the joints: usable = half of them
    assert SIM.effort["J2"] / 2 == pytest.approx(5.97, abs=0.01) and SIM.effort["J3"] / 2 == pytest.approx(1.51, abs=0.01)
    assert (round(SIM.ratio(1), 2), round(SIM.ratio(2), 2)) == (28.49, 6.5)


def test_kinematics_matches_golden_vectors():
    """tools/ik_reference.py vectors (config.h geometry): the same ones the C firmware and the TS twin are
    tested against, so the Python port computes exactly what they compute"""
    g = Geometry.of(FW_RAW)
    vecs = json.loads((REPO_ROOT / "sim/src/core/__tests__/ik_vectors.json").read_text())
    assert len(vecs) > 500
    status = {0: OK, 1: UNREACHABLE, 2: LIMIT}
    for v in vecs:
        st, q = ik(Pose(*v["pose"]), g)
        assert st == status[v["status"]], v
        if st == UNREACHABLE:
            continue
        assert max(abs(a - b) for a, b in zip(q, v["joints"])) < 0.05, v
        tip = fk(q, g)
        assert math.dist((tip.x, tip.y, tip.z), v["tip"][:3]) < 0.1, v


@pytest.mark.parametrize("r, z, q2, q3, q5", [      # design book, Table 4
    (160, 170, -1.6, 121.6, 60.0),                  # HOME
    (220, 110, 30.4, 103.7, 45.9),                  # SAFE (above cradle / bin)
    (220, 37.5, 51.0, 100.1, 28.9),                 # GRIP (cube centre)
    (220, 88, 36.2, 103.6, 40.2),                   # DROP (above bin floor)
])
def test_design_book_stations(r, z, q2, q3, q5):
    st, q = ik(Pose(r, 0, z, 0))
    assert st == OK
    assert q[1] == pytest.approx(q2, abs=0.06) and q[2] == pytest.approx(q3, abs=0.06) and q[4] == pytest.approx(q5, abs=0.06)
    for name in ("BIN_A", "BIN_B"):
        assert ik(CELL[name])[0] == OK


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
    for pose in (CELL["CUBE"], Pose(180, 60, 40, 30), CELL["BIN_A"]):
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
