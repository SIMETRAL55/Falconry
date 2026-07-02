"""Headless unit tests for the frame-conversion math.

These pin down the NED/ENU/FRD/FLU/optical sign conventions — the #1
expected bug class (CLAUDE.md). Every expectation below is derived by hand
from the frame definitions, not from the implementation.
"""

import math

import numpy as np
import pytest

from drone_follow.geometry import (deproject, frd_to_ned, median_depth,
                                   optical_to_frd, quat_to_rot, yaw_from_quat)

K = np.array([[500.0, 0.0, 320.0],
              [0.0, 500.0, 240.0],
              [0.0, 0.0, 1.0]])

Q_IDENT = (1.0, 0.0, 0.0, 0.0)


# ---------------- deprojection ----------------

def test_deproject_center_pixel_is_on_axis():
    p = deproject(320.0, 240.0, 10.0, K)
    assert p == pytest.approx([0.0, 0.0, 10.0])


def test_deproject_right_of_center_is_positive_x():
    # optical x = right
    p = deproject(420.0, 240.0, 5.0, K)
    assert p[0] == pytest.approx((420 - 320) * 5.0 / 500.0)  # = 1.0 m right
    assert p[0] > 0 and p[1] == pytest.approx(0.0)


def test_deproject_below_center_is_positive_y():
    # optical y = down; image v grows downward
    p = deproject(320.0, 340.0, 5.0, K)
    assert p[1] == pytest.approx(1.0)


# ---------------- median depth ----------------

def test_median_depth_ignores_holes():
    d = np.full((100, 100), 8.0, dtype=np.float32)
    d[48:53, 48:53] = 0.0          # dropout hole at the very center
    d[50, 50] = float('nan')       # and a NaN
    z = median_depth(d, 50, 50, 40, 40)
    assert z == pytest.approx(8.0)


def test_median_depth_all_invalid_is_nan():
    d = np.zeros((50, 50), dtype=np.float32)
    assert math.isnan(median_depth(d, 25, 25, 10, 10))


def test_median_depth_clips_at_image_border():
    d = np.full((50, 50), 3.0, dtype=np.float32)
    assert median_depth(d, 0, 0, 20, 20) == pytest.approx(3.0)


# ---------------- optical -> FRD through the gimbal ----------------

def test_gimbal_zero_target_ahead():
    # Gimbal level & centered: optical z (forward) is body FRD +x.
    p = optical_to_frd(np.array([0.0, 0.0, 10.0]), 0.0, 0.0)
    assert p == pytest.approx([10.0, 0.0, 0.0])


def test_gimbal_pitch_90_down_target_below():
    # Camera pointing straight down: optical forward = body FRD +z (down).
    p = optical_to_frd(np.array([0.0, 0.0, 10.0]), 0.0, math.pi / 2)
    assert p == pytest.approx([0.0, 0.0, 10.0], abs=1e-9)


def test_gimbal_yaw_left_target_left():
    # Gimbal yaw + = camera left (FLU); left = -y in FRD.
    p = optical_to_frd(np.array([0.0, 0.0, 10.0]), math.pi / 2, 0.0)
    assert p == pytest.approx([0.0, -10.0, 0.0], abs=1e-9)


def test_optical_right_is_frd_right():
    # Object 1 m to the RIGHT in the image, gimbal centered -> +y in FRD.
    p = optical_to_frd(np.array([1.0, 0.0, 10.0]), 0.0, 0.0)
    assert p == pytest.approx([10.0, 1.0, 0.0])


def test_optical_down_is_frd_down():
    # Object 1 m BELOW optical axis, gimbal centered -> +z in FRD (down).
    p = optical_to_frd(np.array([0.0, 1.0, 10.0]), 0.0, 0.0)
    assert p == pytest.approx([10.0, 0.0, 1.0])


# ---------------- FRD -> NED ----------------

def test_identity_attitude_frd_equals_ned_offset():
    p = frd_to_ned(np.array([5.0, 0.0, 0.0]), Q_IDENT, [10.0, 20.0, -6.0])
    assert p == pytest.approx([15.0, 20.0, -6.0])


def test_yaw_east_rotates_forward_to_east():
    # Vehicle yawed 90 deg (NED + = clockwise = facing East):
    # body forward -> world East (+y in NED).
    q = (math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4))
    p = frd_to_ned(np.array([5.0, 0.0, 0.0]), q, [0.0, 0.0, 0.0])
    assert p == pytest.approx([0.0, 5.0, 0.0], abs=1e-9)


def test_yaw_from_quat_matches_rotation():
    for yaw in (-2.0, -0.5, 0.0, 1.0, 3.0):
        q = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
        got = yaw_from_quat(*q)
        assert math.atan2(math.sin(yaw), math.cos(yaw)) == pytest.approx(got)


def test_quat_to_rot_is_orthonormal():
    q = np.array([0.9, 0.1, -0.2, 0.3])
    q = q / np.linalg.norm(q)
    r = quat_to_rot(*q)
    assert np.allclose(r @ r.T, np.eye(3), atol=1e-12)
    assert np.linalg.det(r) == pytest.approx(1.0)


# ---------------- end-to-end chain ----------------

def test_full_chain_camera_center_hit():
    """Drone at (0,0,-6) NED facing North, gimbal pitched 45 deg down,
    target centered in image at 8.49 m range -> target on the ground
    ~6 m ahead (45 deg geometry: forward = altitude = 6)."""
    rng = 6.0 * math.sqrt(2.0)
    p_opt = deproject(320.0, 240.0, rng, K)
    p_frd = optical_to_frd(p_opt, 0.0, math.pi / 4)
    p_ned = frd_to_ned(p_frd, Q_IDENT, [0.0, 0.0, -6.0])
    assert p_ned == pytest.approx([6.0, 0.0, 0.0], abs=1e-6)
