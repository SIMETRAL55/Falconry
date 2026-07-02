"""perception_node — YOLOv8 + ByteTrack, click-to-lock, depth deprojection.

Design: docs/drone_follow_design.md §4 (Milestones 1-3, + REACQUIRE re-match
support for §6). Runs at camera frame rate (variable CV latency); the 20 Hz
control loop lives in follower_node and is never gated by this node.

Frames: TargetState.position_cam is in the CAMERA OPTICAL frame
(x right, y down, z forward). All world/NED/ENU conversion happens in
follower_node, which owns the vehicle pose.
"""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int32
from ultralytics import YOLO

from drone_follow_msgs.msg import TargetState

from .geometry import deproject, median_depth

WINDOW = 'drone_follow: perception'


class PerceptionNode(Node):

    def __init__(self):
        super().__init__('perception_node')
        # Topic names are parameters: gz-bridged names vary by PX4 version.
        # Confirm against `gz topic -l` / `ros2 topic list` on the live sim.
        self.declare_parameter('rgb_topic', '/camera')
        self.declare_parameter('depth_topic', '/depth_camera')
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('model', 'yolov8m.pt')
        self.declare_parameter('target_classes', [2])  # COCO: 2 = car
        # REACQUIRE gate: re-lock a same-class track whose center is within
        # this pixel radius of the last-known target center.
        self.declare_parameter('reacquire_gate_px', 150.0)  # UNTUNED default — needs sim tuning
        self.declare_parameter('reacquire_timeout_s', 5.0)  # UNTUNED default — needs sim tuning
        # display:=false runs headless (no window, no click). Lock targets via
        # the /target/lock_id topic instead (also works with the window up).
        self.declare_parameter('display', True)

        self.rgb_topic = self.get_parameter('rgb_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.info_topic = self.get_parameter('camera_info_topic').value
        self.classes = list(self.get_parameter('target_classes').value)
        self.gate_px = float(self.get_parameter('reacquire_gate_px').value)
        self.reacq_timeout = float(self.get_parameter('reacquire_timeout_s').value)

        self.bridge = CvBridge()
        self.model = YOLO(self.get_parameter('model').value)

        self.locked_id = -1
        self.last_uv = None          # last-known pixel center of the locked target
        self.lost_since = None       # wall time when the locked id vanished
        self.K = None                # 3x3 intrinsics from CameraInfo
        self.depth = None            # latest depth image (float meters)
        self.tracks = {}             # id -> (xc, yc, w, h)

        self.display = bool(self.get_parameter('display').value)

        self.create_subscription(Image, self.rgb_topic, self.on_rgb, 10)
        self.create_subscription(Image, self.depth_topic, self.on_depth, 10)
        self.create_subscription(CameraInfo, self.info_topic, self.on_info, 10)
        # Programmatic lock command: id >= 0 locks that track, -1 clears,
        # -2 locks the largest visible bbox (headless convenience).
        self.create_subscription(Int32, '/target/lock_id', self.on_lock_cmd, 10)
        self.pub = self.create_publisher(TargetState, '/target/state', 10)

        if self.display:
            cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
            cv2.setMouseCallback(WINDOW, self.on_click)
        self.get_logger().info(
            f'perception up: rgb={self.rgb_topic} depth={self.depth_topic} '
            f'info={self.info_topic} classes={self.classes}')

    # ---------------- camera geometry inputs ----------------

    def on_info(self, msg: CameraInfo):
        self.K = np.array(msg.k).reshape(3, 3)

    def on_depth(self, msg: Image):
        # gz depth bridges as 32FC1 in meters; "passthrough" keeps it float.
        self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    # ---------------- operator selection ----------------

    def on_click(self, event, x, y, *_):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        for tid, (xc, yc, w, h) in self.tracks.items():
            if abs(x - xc) < w / 2 and abs(y - yc) < h / 2:
                self.set_lock(tid, 'click')
                return
        self.get_logger().info('click missed all tracked boxes')

    def on_lock_cmd(self, msg: Int32):
        if msg.data == -1:
            self.locked_id = -1
            self.last_uv = None
            self.lost_since = None
            self.get_logger().info('lock cleared (command)')
        elif msg.data == -2 and self.tracks:
            biggest = max(self.tracks, key=lambda t: self.tracks[t][2] * self.tracks[t][3])
            self.set_lock(biggest, 'command:largest')
        elif msg.data >= 0:
            self.set_lock(int(msg.data), 'command')

    def handle_key(self, key: int):
        """Keyboard fallback: [ / ] cycle ids, c clears the lock."""
        if key == ord('c'):
            self.locked_id = -1
            self.last_uv = None
            self.lost_since = None
        elif key in (ord('['), ord(']')) and self.tracks:
            ids = sorted(self.tracks.keys())
            if self.locked_id in ids:
                i = ids.index(self.locked_id) + (1 if key == ord(']') else -1)
                self.set_lock(ids[i % len(ids)], 'cycle')
            else:
                self.set_lock(ids[0], 'cycle')

    def set_lock(self, tid: int, how: str):
        self.locked_id = tid
        self.lost_since = None
        if tid in self.tracks:
            xc, yc, *_ = self.tracks[tid]
            self.last_uv = (xc, yc)
        self.get_logger().info(f'locked track id {tid} ({how})')

    # ---------------- main CV callback ----------------

    def on_rgb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        res = self.model.track(frame, persist=True, tracker='bytetrack.yaml',
                               classes=self.classes, verbose=False)[0]

        self.tracks.clear()
        if res.boxes.id is not None:
            for box, tid in zip(res.boxes.xywh.cpu().numpy(),
                                res.boxes.id.int().cpu().tolist()):
                xc, yc, w, h = (float(v) for v in box)
                self.tracks[tid] = (xc, yc, w, h)

        st = TargetState()
        st.header = msg.header
        st.track_id = self.locked_id
        st.target_visible = False
        st.range_m = float('nan')
        st.position_cam.x = st.position_cam.y = st.position_cam.z = float('nan')
        # velocity_world stays 0: estimated in follower_node, which owns the
        # vehicle pose needed for the camera->world transform (design §4 note).

        if self.locked_id >= 0:
            if self.locked_id in self.tracks:
                self.lost_since = None
                xc, yc, w, h = self.tracks[self.locked_id]
                self.last_uv = (xc, yc)
                st.target_visible = True
                st.u, st.v, st.bbox_w, st.bbox_h = xc, yc, w, h
                self.fill_3d(st, xc, yc, w, h)
            else:
                self.try_reacquire(msg)
                if self.locked_id in self.tracks:   # re-lock succeeded this frame
                    xc, yc, w, h = self.tracks[self.locked_id]
                    st.track_id = self.locked_id
                    st.target_visible = True
                    st.u, st.v, st.bbox_w, st.bbox_h = xc, yc, w, h
                    self.fill_3d(st, xc, yc, w, h)

        self.pub.publish(st)
        if self.display:
            self.draw(frame)
            self.handle_key(cv2.waitKey(1) & 0xFF)

    def try_reacquire(self, msg: Image):
        """ByteTrack reassigns ids after occlusion, so id-lock alone fails
        (design §6). Re-match by CLASS (tracker already class-filtered) +
        proximity to last-known pixel position, within a timeout."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.lost_since is None:
            self.lost_since = now
            return
        if now - self.lost_since > self.reacq_timeout:
            self.get_logger().warn(
                f'reacquire timed out after {self.reacq_timeout}s — lock cleared')
            self.locked_id = -1
            self.lost_since = None
            return
        if self.last_uv is None or not self.tracks:
            return
        lu, lv = self.last_uv
        best, best_d = None, self.gate_px
        for tid, (xc, yc, _w, _h) in self.tracks.items():
            d = math.hypot(xc - lu, yc - lv)
            if d < best_d:
                best, best_d = tid, d
        if best is not None:
            self.get_logger().info(
                f'reacquired: old id {self.locked_id} -> new id {best} ({best_d:.0f}px)')
            self.set_lock(best, 'reacquire')

    # ---------------- deprojection (Milestone 3) ----------------

    def fill_3d(self, st: TargetState, u: float, v: float, w: float, h: float):
        """Pixel + depth -> camera-optical-frame 3D (design §4).

        Depth is sampled as the MEDIAN over the central region of the bbox
        (central half in each dimension, min 7x7 px) — single-pixel depth is
        noisy and frequently lands on a hole."""
        if self.depth is None or self.K is None:
            return
        Z = median_depth(self.depth, u, v, w, h)
        if not math.isfinite(Z):
            return
        st.range_m = Z
        # Camera optical frame: x right, y down, z forward.
        p = deproject(u, v, Z, self.K)
        st.position_cam.x, st.position_cam.y, st.position_cam.z = p

    # ---------------- display ----------------

    def draw(self, frame):
        for tid, (xc, yc, w, h) in self.tracks.items():
            p1 = (int(xc - w / 2), int(yc - h / 2))
            p2 = (int(xc + w / 2), int(yc + h / 2))
            locked = tid == self.locked_id
            color = (0, 0, 255) if locked else (0, 255, 0)
            cv2.rectangle(frame, p1, p2, color, 2 if locked else 1)
            cv2.putText(frame, f'id:{tid}', (p1[0], p1[1] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        status = f'locked: {self.locked_id}' if self.locked_id >= 0 \
            else 'click a box to lock ([ ] cycle, c clear)'
        cv2.putText(frame, status, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.imshow(WINDOW, frame)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
