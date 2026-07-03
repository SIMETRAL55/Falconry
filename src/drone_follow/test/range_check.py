#!/usr/bin/env python3
"""Milestone 3 exit check: deprojected range_m vs true sim distance.

Parks the car (teleport via gz set_pose), levels the gimbal, locks the
car, and compares range_m against gz ground truth EVENT-DRIVEN (on each
visible TargetState, truth sampled at most once per second) — the car is
static so timing skew is nil.

BACKGROUND (found via a headless GUI screenshot debugging session — see
project memory / commit history): the drone rests on the ground disarmed,
so its camera (spawn default: 45 deg down) needs to be closer to LEVEL to
see a target at roughly the same elevation (drone/car both near the track
surface). Confirmed visually: pitch=-0.15 rad centers the car well.
A one-shot `gz topic -t ... -p ...` CLI publish is UNRELIABLE — it races
subscriber discovery and is frequently dropped silently. This script uses
a real rclpy publisher instead (kept alive, publishes periodically),
which is also how follower_node drives the gimbal in production — the
one-shot pitfall never affected the actual flight code, only ad hoc CLI
debugging.

Reports raw error vs car-CENTER distance and error vs the car's near
SURFACE (depth cameras measure the surface; a hatchback's half-length is
roughly 1.8-2.2 m, so the center sits that much behind what the camera
sees looking at the rear of the car).
"""

import math
import re
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32

from drone_follow_msgs.msg import TargetState

RUN_S = 120.0
CAR = 'hatchback_blue_1'
DRONE = 'x500_depth_0'
# ~6 m ahead of the drone spawn (268.08,-128.22, yaw -0.7), visually
# confirmed in the sim GUI to be a clean, unoccluded view of the car's
# rear from the drone's resting position.
CAR_POSE = (272.67, -132.09, 2.76, 0.0)   # x, y, z, yaw
GIMBAL_PITCH = -0.15   # rad; visually confirmed to center this specific target
GIMBAL_YAW = 0.0
SURFACE_OFFSET = 2.0    # m, approx car half-length toward camera (rear view)


def gz_pos(name):
    try:
        out = subprocess.run(
            ['gz', 'topic', '-e', '-t', '/world/default/pose/info', '-n', '1'],
            capture_output=True, text=True, timeout=10).stdout
        m = re.search(r'name: "%s".*?position \{(.*?)\}' % re.escape(name), out, re.S)
        v = dict(re.findall(r'([xyz]): ([-\d.e]+)', m.group(1)))
        return (float(v.get('x', 0)), float(v.get('y', 0)), float(v.get('z', 0)))
    except Exception:
        return None


def park_car():
    x, y, z, yaw = CAR_POSE
    req = (f'name: "{CAR}", position: {{x: {x}, y: {y}, z: {z}}}, '
           f'orientation: {{x: 0, y: 0, z: {math.sin(yaw/2):.6f}, '
           f'w: {math.cos(yaw/2):.6f}}}')
    subprocess.run(['gz', 'service', '-s', '/world/default/set_pose',
                    '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '2000', '--req', req], capture_output=True)


class RangeCheck(Node):
    def __init__(self):
        super().__init__('range_check')
        self.tgt = None
        self.create_subscription(TargetState, '/target/state', self.on_t, 10)
        self.lock_pub = self.create_publisher(Int32, '/target/lock_id', 10)
        self.gp_pub = self.create_publisher(Float64, '/gimbal/cmd_pitch', 10)
        self.gy_pub = self.create_publisher(Float64, '/gimbal/cmd_yaw', 10)

    def on_t(self, m):
        self.tgt = m

    def drive_gimbal(self):
        self.gp_pub.publish(Float64(data=GIMBAL_PITCH))
        self.gy_pub.publish(Float64(data=GIMBAL_YAW))


def main():
    park_car()
    rclpy.init()
    n = RangeCheck()
    t0 = time.monotonic()
    errs_center, errs_surface = [], []
    last_truth = 0.0
    while time.monotonic() - t0 < RUN_S:
        n.drive_gimbal()   # keep publishing — no follower_node is running here
        rclpy.spin_once(n, timeout_sec=0.1)
        now = time.monotonic()
        if n.tgt is None:
            continue
        if n.tgt.track_id < 0 and int((now - t0) * 2) % 4 == 0:
            n.lock_pub.publish(Int32(data=-2))
        if (n.tgt.target_visible and math.isfinite(n.tgt.range_m)
                and now - last_truth > 1.0):
            last_truth = now
            drone = gz_pos(DRONE)
            car = gz_pos(CAR)
            if drone and car:
                truth = math.dist(drone, car)
                e_c = abs(n.tgt.range_m - truth) / truth
                e_s = abs(n.tgt.range_m - (truth - SURFACE_OFFSET)) / max(0.1, truth - SURFACE_OFFSET)
                errs_center.append(e_c)
                errs_surface.append(e_s)
                print(f'[{now-t0:5.1f}s] range_m={n.tgt.range_m:6.2f} '
                      f'truth_center={truth:6.2f} err_center={e_c*100:5.1f}% '
                      f'err_surface={e_s*100:5.1f}%')
            n.tgt = None   # one truth sample per fresh measurement

    print('=' * 60)
    if errs_surface:
        med_c = sorted(errs_center)[len(errs_center) // 2]
        med_s = sorted(errs_surface)[len(errs_surface) // 2]
        ok = med_s <= 0.10 or med_c <= 0.10
        print(f'n={len(errs_surface)}  median err vs center={med_c*100:.1f}%  '
              f'vs near-surface={med_s*100:.1f}%  -> {"PASS" if ok else "FAIL"} (~10%)')
        sys.exit(0 if ok else 1)
    print('FAIL: no valid range samples (target never visible with finite range_m)')
    sys.exit(2)


if __name__ == '__main__':
    main()
