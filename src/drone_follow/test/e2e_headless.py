#!/usr/bin/env python3
"""End-to-end headless evaluation of the follow stack against the live sim.

Prereqs (already running): PX4 SITL headless + XRCE agent + ros_gz bridge
(/camera, /camera_info, /depth_camera, gimbal), perception_node
(display:=false), follower_node (auto_arm:=true), move_car.py.

What it verifies (goal items 2-3, minus the eyes-on parts):
  A. YOLO detects + ByteTrack tracks the car; /target/lock_id -2 locks it.
  B. range_m vs ground-truth camera->car distance from gz pose/info (~10%).
  C. PX4 stays armed + in OFFBOARD (nav_state 14) the whole run.
  D. Drone moves toward the car / holds standoff while locked (FOLLOW).
  E. Occlusion is survivable in principle: lock persists across short
     detection gaps (REACQUIRE re-match) — reported, not hard-asserted.

Prints a PASS/FAIL summary; exit 0 only if A-C pass.
"""

import math
import re
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import Int32

from px4_msgs.msg import VehicleStatus
from drone_follow_msgs.msg import TargetState

RUN_S = 150.0
NAV_STATE_OFFBOARD = 14
ARMING_ARMED = 2


def gz_model_pos(name: str):
    try:
        out = subprocess.run(
            ['gz', 'topic', '-e', '-t', '/world/default/pose/info', '-n', '1'],
            capture_output=True, text=True, timeout=10).stdout
        m = re.search(r'name: "%s".*?position \{(.*?)\}' % re.escape(name), out, re.S)
        if not m:
            return None
        v = dict(re.findall(r'([xyz]): ([-\d.e]+)', m.group(1)))
        return (float(v.get('x', 0)), float(v.get('y', 0)), float(v.get('z', 0)))
    except Exception:
        return None


class Eval(Node):
    def __init__(self):
        super().__init__('e2e_eval')
        self.tgt = None
        self.status = None
        self.n_tgt = 0
        px4_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(TargetState, '/target/state', self.on_t, 10)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status',
                                 self.on_s, px4_qos)
        self.lock_pub = self.create_publisher(Int32, '/target/lock_id', 10)

    def on_t(self, m):
        self.tgt = m
        self.n_tgt += 1

    def on_s(self, m):
        self.status = m


def main():
    rclpy.init()
    n = Eval()
    t0 = time.monotonic()
    locked_ever = False
    visible_samples = 0
    range_errs = []       # (range_m, truth, err_frac)
    offboard_samples = 0
    armed_samples = 0
    status_samples = 0
    dists = []            # (t, drone-car horizontal distance)
    gaps_survived = 0
    last_visible = None
    lock_id_seen = set()

    next_truth = 0.0
    while time.monotonic() - t0 < RUN_S:
        rclpy.spin_once(n, timeout_sec=0.1)
        now = time.monotonic() - t0

        # keep requesting a lock until we have one
        if n.tgt is not None and n.tgt.track_id < 0 and int(now * 2) % 4 == 0:
            n.lock_pub.publish(Int32(data=-2))

        if n.tgt is not None and n.tgt.track_id >= 0:
            locked_ever = True
            lock_id_seen.add(n.tgt.track_id)
            if n.tgt.target_visible:
                if last_visible is False:
                    gaps_survived += 1
                last_visible = True
                visible_samples += 1
            else:
                last_visible = False

        if n.status is not None:
            status_samples += 1
            if n.status.nav_state == NAV_STATE_OFFBOARD:
                offboard_samples += 1
            if n.status.arming_state == ARMING_ARMED:
                armed_samples += 1
            n.status = None

        if now >= next_truth:
            next_truth = now + 3.0
            drone = gz_model_pos('x500_depth_0')
            car = gz_model_pos('hatchback_blue_1')
            if drone and car:
                d3 = math.dist(drone, car)
                d2 = math.hypot(drone[0] - car[0], drone[1] - car[1])
                dists.append((now, d2))
                if (n.tgt is not None and n.tgt.target_visible
                        and math.isfinite(n.tgt.range_m)):
                    err = abs(n.tgt.range_m - d3) / d3
                    range_errs.append((n.tgt.range_m, d3, err))
                    print(f'[{now:5.1f}s] range_m={n.tgt.range_m:6.2f} '
                          f'truth={d3:6.2f} err={err*100:5.1f}% '
                          f'horiz={d2:6.2f} id={n.tgt.track_id}')
                else:
                    vis = n.tgt.target_visible if n.tgt else None
                    print(f'[{now:5.1f}s] horiz={d2:6.2f} visible={vis} '
                          f'tgt_msgs={n.n_tgt}')

    # ---------------- verdicts ----------------
    ok_a = locked_ever and visible_samples > 10
    med_err = sorted(e for *_, e in range_errs)[len(range_errs) // 2] if range_errs else None
    ok_b = med_err is not None and med_err <= 0.15  # margin over 10%: car center vs surface
    ok_c = (status_samples > 0
            and offboard_samples / max(1, status_samples) > 0.9
            and armed_samples / max(1, status_samples) > 0.9)
    approach = dists[0][1] - min(d for _, d in dists) if len(dists) > 3 else 0.0

    print('=' * 60)
    print(f'A detect+lock+visible : {"PASS" if ok_a else "FAIL"} '
          f'(locked={locked_ever}, visible_samples={visible_samples}, ids={sorted(lock_id_seen)})')
    print(f'B range vs truth      : {"PASS" if ok_b else "FAIL"} '
          f'(median err={None if med_err is None else round(med_err*100,1)}%, n={len(range_errs)})')
    print(f'C armed+offboard held : {"PASS" if ok_c else "FAIL"} '
          f'({offboard_samples}/{status_samples} offboard, {armed_samples}/{status_samples} armed)')
    print(f'D follow behaviour    : closed distance by {approach:.1f} m '
          f'(start {dists[0][1]:.1f} -> min {min(d for _, d in dists):.1f})'
          if dists else 'D follow behaviour    : no distance data')
    print(f'E gaps survived       : {gaps_survived} (visibility gaps re-entered FOLLOW)')
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if (ok_a and ok_b and ok_c) else 1)


if __name__ == '__main__':
    main()
