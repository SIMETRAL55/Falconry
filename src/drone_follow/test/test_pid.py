"""Headless unit tests for the PID helper."""

import pytest

from drone_follow.pid import PID, clamp


def test_clamp():
    assert clamp(5.0, -1.0, 1.0) == 1.0
    assert clamp(-5.0, -1.0, 1.0) == -1.0
    assert clamp(0.3, -1.0, 1.0) == 0.3


def test_proportional():
    pid = PID(kp=2.0, out_min=-10, out_max=10)
    assert pid.step(1.5, 0.05) == pytest.approx(3.0)


def test_output_clamped():
    pid = PID(kp=100.0, out_min=-1.0, out_max=1.0)
    assert pid.step(5.0, 0.05) == 1.0
    assert pid.step(-5.0, 0.05) == -1.0


def test_integrator_stops_when_saturated():
    pid = PID(kp=0.0, ki=10.0, out_min=-1.0, out_max=1.0)
    for _ in range(1000):
        pid.step(1.0, 0.05)
    # windup capped: once we saturate, the integrator must stop growing,
    # so recovery after the error flips sign is fast.
    steps_to_recover = 0
    while pid.step(-1.0, 0.05) > 0.0:
        steps_to_recover += 1
        assert steps_to_recover < 50, 'integrator wound up while saturated'


def test_reset():
    pid = PID(kp=1.0, ki=1.0, kd=1.0, out_min=-10, out_max=10)
    pid.step(1.0, 0.05)
    pid.reset()
    # after reset, integrator is empty and the derivative term must not
    # fire on the first step (no previous error)
    assert pid.step(1.0, 0.05) == pytest.approx(1.0)
