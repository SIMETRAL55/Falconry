"""Pure geometry helpers — no ROS imports, unit-testable headlessly.

Frame chain (see follower_node docstring for the full frame table):
  camera optical (x right, y down, z fwd)
    -> camera link FLU (fwd = z_o, left = -x_o, up = -y_o)
    -> body FLU via gimbal Rz(yaw) @ Ry(pitch)   [joint order per setup_gimbal.py]
    -> body FRD (x, -y, -z)
    -> world NED via vehicle attitude quaternion (FRD->NED) + position.
"""

import math

import numpy as np


def median_depth(depth: np.ndarray, u: float, v: float,
                 w: float, h: float) -> float:
    """Median depth over the CENTRAL region of the bbox (half-size each dim,
    min 7x7 px), ignoring NaN/inf/non-positive holes. NaN if nothing valid."""
    H, W = depth.shape[:2]
    hw = max(3.0, w / 4.0)
    hh = max(3.0, h / 4.0)
    x0, x1 = max(0, int(u - hw)), min(W, int(u + hw) + 1)
    y0, y1 = max(0, int(v - hh)), min(H, int(v + hh) + 1)
    patch = np.asarray(depth[y0:y1, x0:x1], dtype=np.float32)
    patch = patch[np.isfinite(patch) & (patch > 0.0)]
    if patch.size == 0:
        return float('nan')
    return float(np.median(patch))


def rgb_to_depth_px(u: float, v: float, k_rgb: np.ndarray,
                    depth_w: int, depth_h: int,
                    depth_hfov: float) -> tuple:
    """Map an RGB pixel to the corresponding depth-image pixel.

    The RGB and depth sensors are CO-LOCATED (same SDF pose) but differ in
    resolution AND field of view (OakD-Lite: RGB 1920x1080 hfov 1.204,
    depth 640x480 hfov 1.274) — indexing the depth image with RGB pixel
    coordinates is wrong. Same optical ray: x/z = (u-cx)/fx in both cameras.
    Assumes square pixels on the depth sensor (fy_d = fx_d)."""
    fx_d = (depth_w / 2.0) / math.tan(depth_hfov / 2.0)
    cx_d, cy_d = depth_w / 2.0, depth_h / 2.0
    fx_r, fy_r = k_rgb[0, 0], k_rgb[1, 1]
    cx_r, cy_r = k_rgb[0, 2], k_rgb[1, 2]
    u_d = cx_d + fx_d * (u - cx_r) / fx_r
    v_d = cy_d + fx_d * (v - cy_r) / fy_r
    scale = fx_d / fx_r    # bbox size scale factor RGB px -> depth px
    return u_d, v_d, scale


def deproject(u: float, v: float, z: float, k: np.ndarray) -> np.ndarray:
    """Pixel + depth -> camera OPTICAL frame 3D (x right, y down, z forward)."""
    fx, fy = k[0, 0], k[1, 1]
    cx, cy = k[0, 2], k[1, 2]
    return np.array([(u - cx) * z / fx, (v - cy) * z / fy, z])


def optical_to_frd(p_opt: np.ndarray, gimbal_yaw: float,
                   gimbal_pitch: float) -> np.ndarray:
    """Camera optical -> body FRD, through the gimbal joints.

    gimbal_pitch + = camera down; gimbal_yaw + = camera left (gz FLU frame,
    per setup_gimbal.py joint axes)."""
    # optical -> camera-link FLU
    p_link = np.array([p_opt[2], -p_opt[0], -p_opt[1]])
    cy_, sy_ = math.cos(gimbal_yaw), math.sin(gimbal_yaw)
    cp_, sp_ = math.cos(gimbal_pitch), math.sin(gimbal_pitch)
    ry = np.array([[cp_, 0, sp_], [0, 1, 0], [-sp_, 0, cp_]])
    rz = np.array([[cy_, -sy_, 0], [sy_, cy_, 0], [0, 0, 1]])
    p_body_flu = rz @ ry @ p_link
    # FLU -> FRD: negate y (left->right) and z (up->down)
    return np.array([p_body_flu[0], -p_body_flu[1], -p_body_flu[2]])


def quat_to_rot(w: float, x: float, y: float, z: float) -> np.ndarray:
    """Rotation matrix from unit quaternion [w,x,y,z] (body FRD -> world NED)."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def frd_to_ned(p_frd: np.ndarray, q_wxyz, pos_ned) -> np.ndarray:
    """Body FRD vector -> world NED point, via vehicle attitude + position."""
    r = quat_to_rot(q_wxyz[0], q_wxyz[1], q_wxyz[2], q_wxyz[3])
    return np.asarray(pos_ned, dtype=float) + r @ p_frd


def yaw_from_quat(w: float, x: float, y: float, z: float) -> float:
    """NED heading from quaternion: 0 = North, + = clockwise (toward East)."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
