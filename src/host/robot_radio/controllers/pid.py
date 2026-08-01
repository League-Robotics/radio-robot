"""Discrete PID controller and angle utilities."""

import math


class PID:
    """Discrete PID controller with output clamping and integral windup guard."""

    # Anti-windup: the raw integral accumulator is clamped so that
    # ki * integral never exceeds this magnitude, regardless of out_min/
    # out_max -- see update()'s own clamp line below.
    _INTEGRAL_CLAMP = 50.0
    # Floor applied to ki ONLY inside the clamp's own denominator (never to
    # the controller's real ki) -- avoids a division by a zero/near-zero ki
    # blowing the clamp bound up toward +/-inf.
    _MIN_KI_FOR_CLAMP = 0.001

    def __init__(self, kp, ki, kd, out_min=-100, out_max=100):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.integral = 0.0
        self.prev_error = None
        self.prev_time = None

    def update(self, error, now):
        if self.prev_time is None:
            self.prev_time = now
            self.prev_error = error
            return max(self.out_min, min(self.out_max, self.kp * error))

        dt = now - self.prev_time
        if dt <= 0:
            return max(self.out_min, min(self.out_max, self.kp * error))

        self.integral += error * dt
        clamp = self._INTEGRAL_CLAMP / max(self.ki, self._MIN_KI_FOR_CLAMP)
        self.integral = max(-clamp, min(clamp, self.integral))

        derivative = (error - self.prev_error) / dt

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = max(self.out_min, min(self.out_max, output))

        self.prev_error = error
        self.prev_time = now
        return output

    def reset(self):
        self.integral = 0.0
        self.prev_error = None
        self.prev_time = None


def normalize_angle(a):
    """Normalize angle to (-pi, pi]."""
    while a > math.pi:
        a -= 2 * math.pi
    while a <= -math.pi:
        a += 2 * math.pi
    return a
