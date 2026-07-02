#!/usr/bin/env python3
"""Headless integration check for follower_node (no sim, no GUI).

Runs against a LIVE follower_node (start it first, e.g.:
    ros2 run drone_follow follower_node &
    python3 heartbeat_check.py
).

Feeds it fake odometry + camera_info + TargetState through four phases:
  SEARCH (no target) -> FOLLOW (locked, visible) ->
  REACQUIRE/HOLD (target msgs keep coming but not visible) -> recovery
and asserts the offboard heartbeat (OffboardControlMode + TrajectorySetpoint)
never drops below 19 Hz in ANY phase — the PX4 drop-out rule from design §5.

Exit code 0 = pass. This does NOT replace sim verification; it only proves
the control loop is never gated by perception.
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import CameraInfo

from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleOdometry)
from drone_follow_msgs.msg import TargetState

MIN_HZ = 19.0
PHASE_S = 3.0


class Harness(Node):

    def __init__(self):
        super().__init__('heartbeat_check')
        self.n_ocm = 0
        self.n_sp = 0
        px4_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(OffboardControlMode,
                                 '/fmu/in/offboard_control_mode',
                                 self.on_ocm, 10)
        self.create_subscription(TrajectorySetpoint,
                                 '/fmu/in/trajectory_setpoint',
                                 self.on_sp, 10)
        self.pub_odom = self.create_publisher(
            VehicleOdometry, '/fmu/out/vehicle_odometry', px4_qos)
        self.pub_tgt = self.create_publisher(TargetState, '/target/state', 10)
        self.pub_info = self.create_publisher(CameraInfo, '/camera_info', 10)

    def on_ocm(self, _):
        self.n_ocm += 1

    def on_sp(self, _):
        self.n_sp += 1

    def feed_odom(self):
        m = VehicleOdometry()
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        m.position = [0.0, 0.0, -6.0]     # NED: 6 m altitude
        m.q = [1.0, 0.0, 0.0, 0.0]        # facing North
        self.pub_odom.publish(m)

    def feed_info(self):
        m = CameraInfo()
        m.width, m.height = 640, 480
        m.k = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
        self.pub_info.publish(m)

    def feed_target(self, visible: bool):
        m = TargetState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.track_id = 7
        m.target_visible = visible
        if visible:
            m.u, m.v = 360.0, 260.0       # slightly right/below center
            m.bbox_w, m.bbox_h = 80.0, 60.0
            m.range_m = 12.0
            m.position_cam.x, m.position_cam.y, m.position_cam.z = 0.96, 0.48, 12.0
        else:
            m.range_m = float('nan')
            m.position_cam.x = m.position_cam.y = m.position_cam.z = float('nan')
        self.pub_tgt.publish(m)


def run_phase(node: Harness, name: str, seconds: float,
              target_mode: str) -> float:
    """target_mode: 'none' | 'visible' | 'lost' (msgs with visible=False)."""
    node.n_ocm = node.n_sp = 0
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        node.feed_odom()
        node.feed_info()
        if target_mode == 'visible':
            node.feed_target(True)
        elif target_mode == 'lost':
            node.feed_target(False)
        rclpy.spin_once(node, timeout_sec=0.02)
    hz_ocm = node.n_ocm / seconds
    hz_sp = node.n_sp / seconds
    ok = hz_ocm >= MIN_HZ and hz_sp >= MIN_HZ
    print(f'[{name:10s}] OffboardControlMode {hz_ocm:5.1f} Hz | '
          f'TrajectorySetpoint {hz_sp:5.1f} Hz | '
          f'{"OK" if ok else "FAIL (< %.0f Hz)" % MIN_HZ}')
    return min(hz_ocm, hz_sp)


def main():
    rclpy.init()
    node = Harness()
    # Give the follower time to discover us (DDS matching).
    t0 = time.monotonic()
    while time.monotonic() - t0 < 5.0 and node.n_ocm == 0:
        node.feed_odom()
        rclpy.spin_once(node, timeout_sec=0.05)
    if node.n_ocm == 0:
        print('FAIL: no OffboardControlMode received — is follower_node running?')
        sys.exit(2)

    results = [
        run_phase(node, 'SEARCH', PHASE_S, 'none'),
        run_phase(node, 'FOLLOW', PHASE_S, 'visible'),
        # visible=False msgs accumulate the lost counter -> REACQUIRE -> HOLD
        run_phase(node, 'LOST/REACQ', PHASE_S, 'lost'),
        run_phase(node, 'HOLD', PHASE_S, 'lost'),
        run_phase(node, 'RE-FOLLOW', PHASE_S, 'visible'),
    ]
    node.destroy_node()
    rclpy.shutdown()
    if all(r >= MIN_HZ for r in results):
        print('PASS: heartbeat >= %.0f Hz in every state' % MIN_HZ)
        sys.exit(0)
    print('FAIL: heartbeat dropped below %.0f Hz in at least one state' % MIN_HZ)
    sys.exit(1)


if __name__ == '__main__':
    main()
