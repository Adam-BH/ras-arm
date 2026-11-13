"""The twin's URDF against the firmware: limits from config.h, and forward kinematics through the URDF
joint tree landing on the same tip as the firmware's golden IK vectors (sim/src/core/__tests__/ik_vectors.json,
from tools/ik_reference.py — the vectors the C firmware and the TypeScript twin are tested against)."""
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import xacro
import yaml

PKG = Path(__file__).resolve().parents[1]
REPO = Path(__file__).resolve().parents[3]
VECTORS = REPO / 'sim/src/core/__tests__/ik_vectors.json'
ARM = yaml.safe_load((PKG / 'config/arm.yaml').read_text())
JOINTS = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']


def expand(name, **args):
    return ET.fromstring(xacro.process_file(str(PKG / 'urdf' / name), mappings=args).toxml())


def rot(axis, a):
    x, y, z = axis / np.linalg.norm(axis)
    c, s, C = math.cos(a), math.sin(a), 1 - math.cos(a)
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def rpy(r, p, y):
    return rot(np.array([0, 0, 1.0]), y) @ rot(np.array([0, 1.0, 0]), p) @ rot(np.array([1.0, 0, 0]), r)


class Tree:
    def __init__(self, robot):
        self.by_child = {}
        for j in robot.iter('joint'):
            o = j.find('origin')
            xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]
            r = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
            a = j.find('axis')
            self.by_child[j.find('child').get('link')] = dict(
                name=j.get('name'), type=j.get('type'), parent=j.find('parent').get('link'),
                xyz=np.array(xyz), R=rpy(*r), axis=np.array([float(v) for v in (a.get('xyz') if a is not None else '1 0 0').split()]))

    def fk(self, link, q):
        """4 × 4 pose of link in the root frame, q = {joint: position}"""
        chain = []
        while link in self.by_child:
            chain.append(self.by_child[link])
            link = chain[-1]['parent']
        T = np.eye(4)
        for j in reversed(chain):
            A = np.eye(4)
            A[:3, :3], A[:3, 3] = j['R'], j['xyz']
            M = np.eye(4)
            if j['type'] in ('revolute', 'continuous'):
                M[:3, :3] = rot(j['axis'], q.get(j['name'], 0.0))
            elif j['type'] == 'prismatic':
                M[:3, 3] = j['axis'] * q.get(j['name'], 0.0)
            T = T @ A @ M
        return T


@pytest.fixture(scope='module')
def robot():
    return expand('ras_arm.urdf.xacro')


def test_joint_limits_are_config_h(robot):
    for j in robot.iter('joint'):
        name = j.get('name')
        if name in JOINTS:
            lim = j.find('limit')
            assert float(lim.get('lower')) == pytest.approx(ARM['joints'][name]['lower'])
            assert float(lim.get('upper')) == pytest.approx(ARM['joints'][name]['upper'])
            assert float(lim.get('velocity')) == pytest.approx(ARM['joints'][name]['velocity'])
    names = [j.get('name') for j in robot.iter('joint') if j.get('type') != 'fixed']
    assert names == JOINTS + ['grip_left', 'grip_right']
    links = {l.get('name') for l in robot.iter('link')}
    assert not links & {j.get('name') for j in robot.iter('joint')}              # SDF: one namespace


def test_zero_pose_is_straight_up(robot):
    g = ARM['geometry']
    tip = Tree(robot).fk('tool_tip', {})
    assert tip[:3, 3] == pytest.approx([0, 0, g['h_shoulder'] + g['l1'] + g['l2'] + g['l3']], abs=1e-9)


@pytest.mark.skipif(not VECTORS.exists(), reason='needs the repo checkout (sim/ golden vectors)')
def test_fk_matches_firmware_golden_vectors(robot):
    tree, n = Tree(robot), 0
    for v in json.loads(VECTORS.read_text()):
        if v['status'] != 0:
            continue
        q = {name: math.radians(a) for name, a in zip(JOINTS, v['joints'])}
        T = tree.fk('tool_tip', q)
        x, y, z, roll = v['tip']
        assert T[:3, 3] * 1000 == pytest.approx([x, y, z], abs=1e-3), v['pose']   # mm
        # the vectors' angles are rounded to 1e-5°: direction to ~2e-7, yaw to 1e-4°
        assert T[:3, 2] == pytest.approx([0, 0, -1], abs=1e-6)                    # gripper straight down
        yaw = math.degrees(math.atan2(T[1, 1], T[0, 1])) - 90                      # finger axis = tool y
        assert (yaw - roll + 180) % 360 - 180 == pytest.approx(0, abs=1e-4), v['pose']
        n += 1
    assert n > 200


def test_fingers_open_to_the_grip_value(robot):
    tree, half = Tree(robot), ARM['gripper']['open']
    q = {'grip_left': half, 'grip_right': half}
    gap = tree.fk('finger_left', q)[:3, 3] - tree.fk('finger_right', q)[:3, 3]
    assert np.linalg.norm(gap) == pytest.approx(2 * half)                          # = GRIP_OPEN_MM
    mimic = [j for j in robot.iter('joint') if j.get('name') == 'grip_right'][0].find('mimic')
    assert mimic.get('joint') == 'grip_left'


def test_sim_adds_ros2_control_starting_at_home():
    robot = expand('ras_arm.urdf.xacro', sim='true', controllers='/tmp/c.yaml')
    rc = robot.find('ros2_control')
    assert rc.find('hardware/plugin').text == 'gz_ros2_control/GazeboSimSystem'
    init = {j.get('name'): j.find("state_interface[@name='position']/param") for j in rc.iter('joint')}
    for k, name in enumerate(JOINTS):
        assert float(init[name].text) == pytest.approx(ARM['cell']['home_joints'][k])
    assert rc.find("joint[@name='grip_right']/command_interface") is None          # mimic: state only
    assert robot.find('gazebo/plugin/parameters').text == '/tmp/c.yaml'


def read_stl(path):
    data = path.read_bytes()
    n = int.from_bytes(data[80:84], 'little')
    rec = np.frombuffer(data[84:84 + 50 * n], dtype=[('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    return rec['v'].reshape(-1, 3).astype(float)


def test_cad_meshes_sit_on_their_links(robot):
    """for each imported mesh: drawn where the CAD had it in the zero pose, overlapping its link's
    simple collision shape, mass properties sane (no-op until the CAD is imported)"""
    meshes = yaml.safe_load((PKG / 'config/cad_meshes.yaml').read_text())['links'] or {}
    links = {l.get('name'): l for l in robot.iter('link')}
    tree = Tree(robot)
    q0 = yaml.safe_load((PKG / 'config/cad.yaml').read_text())['offsets']['export_opening'] / 2
    for name, e in meshes.items():
        vis = links[name].find('visual')
        assert vis.find('geometry/mesh').get('filename').endswith(e['mesh'])
        p = read_stl(PKG / 'meshes' / e['mesh']) / 1000                             # CAD frame, m
        T = tree.fk(name, {'grip_left': q0, 'grip_right': q0})                     # link in the exported pose
        off = np.array([float(v) for v in vis.find('origin').get('xyz').split()])
        drawn = p + off + T[:3, 3]                                                 # where RViz / Gazebo draw it
        assert drawn == pytest.approx(p, abs=1e-9), name
        col = links[name].find('collision')
        c = np.array([float(v) for v in col.find('origin').get('xyz', '0 0 0').split()]) + T[:3, 3]
        lo, hi = drawn.min(0), drawn.max(0)
        assert np.all(lo - 0.02 <= c) and np.all(c <= hi + 0.02), f'{name}: mesh away from its link'
        assert e['mass'] > 0 and min(e['inertia'][:3]) > 0, name


def test_cell_from_config_h():
    cell = expand('ras_cell.urdf.xacro')
    links = {l.get('name'): l for l in cell.iter('link')}
    xyz = lambda n: [float(v) for v in links[n].find('visual/origin').get('xyz').split()]
    assert xyz('cradle_l')[0] == pytest.approx(ARM['cell']['cradle_x'])
    assert xyz('bin_a_e')[1] == pytest.approx(ARM['cell']['bin_y'])
    assert xyz('bin_b_e')[1] == pytest.approx(-ARM['cell']['bin_y'])
