"""Minimal PID with output clamping and conditional anti-windup."""


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class PID:
    def __init__(self, kp: float, ki: float = 0.0, kd: float = 0.0,
                 out_min: float = -1.0, out_max: float = 1.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self._i = 0.0
        self._prev_e = None

    def reset(self) -> None:
        self._i = 0.0
        self._prev_e = None

    def step(self, e: float, dt: float) -> float:
        d = 0.0 if self._prev_e is None or dt <= 0.0 else (e - self._prev_e) / dt
        self._prev_e = e
        u_unsat = self.kp * e + self._i + self.kd * d
        # Conditional anti-windup: don't integrate further INTO saturation,
        # but always allow integrating back OUT of it.
        into_hi = u_unsat >= self.out_max and e > 0.0
        into_lo = u_unsat <= self.out_min and e < 0.0
        if not into_hi and not into_lo:
            self._i += self.ki * e * dt
        return clamp(u_unsat, self.out_min, self.out_max)
