#!/usr/bin/env python3
"""Empirically converge the parked car onto the camera's optical axis, then
measure range_m vs gz ground truth. Self-correcting: uses OBSERVED pixel
error from YOLO on each iteration rather than hand-derived sign conventions
for pitch/yaw/frame composition, which proved error-prone to derive by hand.

Requires: PX4 (disarmed, resting at spawn), bridge, perception_node
(display:=false) already running. follower_node must NOT be running (it
would fight the direct /gimbal/cmd_* commands and vehicle stays disarmed
anyway, so it wouldn't publish valid offboard setpoints either).
"""

import json
import math
import re
import subprocess
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
from ultralytics import YOLO

from drone_follow_msgs.msg import TargetState

CAR = 'hatchback_blue_1'
DRONE = 'x500_depth_0'


def gz_pos(name):
    out = subprocess.run(
        ['gz', 'topic', '-e', '-t', '/world/default/pose/info', '-n', '1'],
        capture_output=True, text=True, timeout=10).stdout
    m = re.search(r'name: "%s".*?position \{(.*?)\}' % re.escape(name), out, re.S)
    v = dict(re.findall(r'([xyz]): ([-\d.e]+)', m.group(1)))
    return np.array([float(v['x']), float(v['y']), float(v['z'])])


def set_car_pos(p):
    req = (f'name: "{CAR}", position: {{x: {p[0]:.4f}, y: {p[1]:.4f}, z: {p[2]:.4f}}}, '
           f'orientation: {{x:0,y:0,z:0,w:1}}')
    subprocess.run(['gz', 'service', '-s', '/world/default/set_pose',
                    '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '2000', '--req', req], capture_output=True)


class Grabber(Node):
    def __init__(self):
        super().__init__('converge_check')
        self.br = CvBridge()
        self.frame = None
        self.K = None
        self.create_subscription(Image, '/camera', self.on_rgb, 10)
        self.create_subscription(CameraInfo, '/camera_info', self.on_info, 10)
        self.tgt = None
        self.create_subscription(TargetState, '/target/state', self.on_tgt, 10)
        self.lock_pub = self.create_publisher(Int32, '/target/lock_id', 10)

    def on_rgb(self, m):
        self.frame = self.br.imgmsg_to_cv2(m, 'bgr8')

    def on_info(self, m):
        self.K = np.array(m.k).reshape(3, 3)

    def on_tgt(self, m):
        self.tgt = m

    def spin_for(self, secs):
        t0 = time.monotonic()
        while time.monotonic() - t0 < secs:
            rclpy.spin_once(self, timeout_sec=0.1)


def detect_car(model, frame):
    res = model.predict(frame, classes=[2], verbose=False)[0]
    if len(res.boxes) == 0:
        return None
    # largest box
    boxes = res.boxes.xywh.cpu().numpy()
    areas = boxes[:, 2] * boxes[:, 3]
    i = int(np.argmax(areas))
    return boxes[i]  # xc, yc, w, h


def main():
    rclpy.init()
    n = Grabber()
    model = YOLO('yolov8n.pt')

    drone_pos = gz_pos(DRONE)
    # initial guess: 6 m ahead along measured body yaw (reuse earlier estimate),
    # roughly level with the drone (near its own resting altitude).
    yaw = -0.7
    car = drone_pos + np.array([6.0 * math.cos(yaw), 6.0 * math.sin(yaw), 0.0])
    car[2] = drone_pos[2] - 0.3   # slightly below camera height

    # ensure gimbal is level and not fought by follower
    subprocess.run(['gz', 'topic', '-t', '/gimbal/cmd_yaw', '-m', 'gz.msgs.Double', '-p', 'data: 0.0'])
    subprocess.run(['gz', 'topic', '-t', '/gimbal/cmd_pitch', '-m', 'gz.msgs.Double', '-p', 'data: 0.0'])
    n.spin_for(3)

    last_err = None
    for it in range(8):
        set_car_pos(car)
        n.frame = None
        n.spin_for(3)
        if n.frame is None or n.K is None:
            print(f'iter {it}: no frame/info yet')
            continue
        det = detect_car(model, n.frame)
        h, w = n.frame.shape[:2]
        cx, cy = n.K[0, 2], n.K[1, 2]
        fx, fy = n.K[0, 0], n.K[1, 1]
        if det is None:
            print(f'iter {it}: car={car.round(2)} -> NOT DETECTED, widening search')
            # try moving car further from camera along the ray + centering vertically
            car[2] = drone_pos[2] - 0.3 + 0.3 * ((it % 3) - 1)  # jitter z
            continue
        xc, yc, bw, bh = det
        e_u = (xc - cx) / w
        e_v = (yc - cy) / h
        dist = math.dist(drone_pos, car)
        print(f'iter {it}: car={car.round(2)} dist={dist:.2f} bbox_center=({xc:.0f},{yc:.0f}) '
              f'err_u={e_u:+.3f} err_v={e_v:+.3f} bbox=({bw:.0f}x{bh:.0f})')
        if abs(e_u) < 0.15 and abs(e_v) < 0.15:
            print('CONVERGED')
            break
        # Convert pixel error to a lateral/vertical world correction at current range.
        # Empirical gain with adaptive sign flip if error grows.
        step_lat = e_u * dist * (fx and (w / fx) or 1.0) * 0.5
        step_vert = e_v * dist * (fy and (h / fy) or 1.0) * 0.5
        # lateral = perpendicular to bearing (yaw+90deg); vertical = world Z
        perp = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
        car = car + perp * step_lat
        car[2] = car[2] - step_vert
        last_err = abs(e_u) + abs(e_v)

    # final: lock and measure
    n.spin_for(2)
    n.lock_pub.publish(Int32(data=-2))
    n.spin_for(3)
    if n.tgt is None or not n.tgt.target_visible or not math.isfinite(n.tgt.range_m):
        print('FAIL: could not lock/measure after convergence')
        sys.exit(1)
    truth = math.dist(gz_pos(DRONE), gz_pos(CAR))
    err = abs(n.tgt.range_m - truth) / truth
    print(f'range_m={n.tgt.range_m:.2f} truth={truth:.2f} err={err*100:.1f}%')
    print('PASS' if err <= 0.15 else 'FAIL', '(~10% target, 15% margin for car-center-vs-surface)')
    with open('/tmp/converge_result.json', 'w') as f:
        json.dump({'range_m': n.tgt.range_m, 'truth': truth, 'err': err}, f)
    sys.exit(0 if err <= 0.15 else 1)


if __name__ == '__main__':
    main()
