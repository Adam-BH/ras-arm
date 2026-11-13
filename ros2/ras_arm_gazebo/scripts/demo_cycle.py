#!/usr/bin/env python3
"""demo_cycle.py — the firmware's RUN cycle on the Gazebo twin, with the firmware's IK, speed limits and
timings (config.h via arm.yaml): open → above the cradle → straight down → close → straight up →
above the bin → straight down → open → up → home.

ros2 run ras_arm_gazebo demo_cycle.py [--bin A|B] [--cycles N] [--check]
--check: afterwards, ask Gazebo where the cradle cube is; exit 1 unless it landed in the bin.
"""
import argparse
import math
import os
import subprocess

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from trajectory_msgs.msg import JointTrajectoryPoint

with open(os.path.join(get_package_share_directory('ras_arm_description'), 'config', 'arm.yaml')) as f:
    ARM = yaml.safe_load(f)
G, CELL, MOTION, GRIP = ARM['geometry'], ARM['cell'], ARM['motion'], ARM['gripper']
JOINTS = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
# GRIP_CLOSED_MM (26) is 1 mm wider than the cube: the real servo closes on it and stalls. The twin's
# fingers are exact, so they are told to go 2 mm further per side and stop on the cube, pressing with
# their effort limit (hardware.yaml) like the stalled servo.
SQUEEZE = 0.002


def ik(x, y, z, roll=0.0):
    """kinematics.c in SI units: tip (m, rad) → joints (rad), gripper vertical"""
    r, zw = math.hypot(x, y), z + G['l3'] - G['h_shoulder']
    c3 = (r * r + zw * zw - G['l1'] ** 2 - G['l2'] ** 2) / (2 * G['l1'] * G['l2'])
    if not -1.0 <= c3 <= 0.995:
        raise ValueError(f'unreachable: {x:.3f} {y:.3f} {z:.3f}')
    j3 = math.acos(c3)
    j2 = math.atan2(r, zw) - math.atan2(G['l2'] * math.sin(j3), G['l1'] + G['l2'] * math.cos(j3))
    j1 = math.atan2(y, x)
    q = [j1, j2, j3, 0.0, math.pi - j2 - j3, roll - j1]
    for k, (name, v) in enumerate(zip(JOINTS, q)):
        lim = ARM['joints'][name]
        if not lim['lower'] - 1e-4 <= v <= lim['upper'] + 1e-4:      # the firmware's 0.01° slack, then clamp
            raise ValueError(f'{name} out of range at {x:.3f} {y:.3f} {z:.3f}')
        q[k] = min(max(v, lim['lower']), lim['upper'])
    return q


def stamp(t):
    return Duration(sec=int(t), nanosec=int((t % 1) * 1e9))


class Demo(Node):
    def __init__(self):
        super().__init__('ras_arm_demo', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.arm = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.hand = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        self.q = list(CELL['home_joints'])
        self.tip = list(CELL['home'])

    def follow(self, client, label, joints, points):
        """points: [(positions, t)] → one trajectory, wait for the result"""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joints
        goal.trajectory.points = [JointTrajectoryPoint(positions=q, time_from_start=stamp(t)) for q, t in points]
        goal.trajectory.points[-1].velocities = [0.0] * len(joints)
        sent = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, sent)
        if not sent.result().accepted:
            raise RuntimeError(f'{label}: goal rejected')
        done = sent.result().get_result_async()
        rclpy.spin_until_future_complete(self, done)
        if done.result().result.error_code != 0:
            raise RuntimeError(f'{label}: {done.result().result.error_string}')

    def run(self, label, points):
        self.get_logger().info(f'{label:<14} → {" ".join(f"{v * 1000:.0f}" for v in self.tip)} mm   {points[-1][1]:.2f} s')
        self.follow(self.arm, label, JOINTS, points)
        self.q = points[-1][0]

    def move(self, label, x, y, z, min_s):
        """MOVE: joint space, the slowest joint at its speed limit, stretched by the trapezoid ramp"""
        q = ik(x, y, z)
        t = max(abs(b - a) / ARM['joints'][n]['velocity'] for n, a, b in zip(JOINTS, self.q, q)) / (1 - MOTION['ramp'])
        self.tip = [x, y, z]
        self.run(label, [(q, max(t, min_s))])

    def line(self, label, x, y, z, speed):
        """LINE: IK every line_step along the straight segment, like motion_line()"""
        x0, y0, z0 = self.tip
        d = math.dist((x0, y0, z0), (x, y, z))
        n = max(1, math.ceil(d / MOTION['line_step']))
        pts = [(ik(x0 + (x - x0) * k / n, y0 + (y - y0) * k / n, z0 + (z - z0) * k / n), d * k / n / speed) for k in range(1, n + 1)]
        self.tip = [x, y, z]
        self.run(label, pts)

    def gripper(self, label, half):
        """GRIP: the opening in GRIP_MOVE_MS, both fingers (grip_right mimics grip_left)"""
        self.get_logger().info(f'{label:<14} → {half * 2000:.0f} mm')
        self.follow(self.hand, label, ['grip_left'], [([half], MOTION['move_s']['grip'])])

    def cycle(self, bin_name):
        cx, safe, s = CELL['cradle_x'], CELL['safe_z'], MOTION['line_m_s']
        by = CELL['bin_y'] if bin_name == 'A' else -CELL['bin_y']
        above_bin = safe + CELL['place_above_dz']
        self.gripper('open', GRIP['open'])
        self.move('above cradle', cx, 0.0, safe, MOTION['move_s']['above'])
        self.line('down', cx, 0.0, CELL['grip_z'], s['pick_down'])
        self.gripper('close', GRIP['closed'] - SQUEEZE)
        self.line('up', cx, 0.0, safe, s['pick_up'])
        self.move(f'above bin {bin_name}', 0.0, by, above_bin, MOTION['move_s']['transit'])
        self.line('down', 0.0, by, CELL['drop_z'], s['place_down'])
        self.gripper('open', GRIP['open'])
        self.line('up', 0.0, by, above_bin, s['place_up'])
        self.move('home', CELL['home'][0], 0.0, CELL['home'][2], MOTION['move_s']['home'])


def cube_position(name='cube_red_cradle'):
    """x, y, z (m) of a model, from the Gazebo command line"""
    out = subprocess.run(['gz', 'model', '-m', name, '-p'], capture_output=True, text=True, timeout=10).stdout.splitlines()
    i = next(k for k, line in enumerate(out) if 'XYZ' in line)
    return [float(v) for v in out[i + 1].strip(' []').split()]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--bin', choices=['A', 'B'], default='A')
    ap.add_argument('--cycles', type=int, default=1)
    ap.add_argument('--check', action='store_true')
    args, _ = ap.parse_known_args()
    rclpy.init()
    node = Demo()
    if not (node.arm.wait_for_server(timeout_sec=30.0) and node.hand.wait_for_server(timeout_sec=10.0)):
        raise SystemExit('no /arm_controller or /gripper_controller: is sim.launch.py running?')
    try:
        for _ in range(args.cycles):
            node.cycle(args.bin)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if args.check:
        x, y, z = cube_position()
        by = CELL['bin_y'] if args.bin == 'A' else -CELL['bin_y']
        ok = math.hypot(x, y - by) < 0.045 and z < 0.05              # inside the 110 mm bin, on its floor
        print(f'cradle cube at {x * 1000:.0f}, {y * 1000:.0f}, {z * 1000:.0f} mm: {"in" if ok else "NOT in"} bin {args.bin}')
        raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
