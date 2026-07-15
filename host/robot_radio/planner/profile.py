"""robot_radio.planner.profile -- pure trapezoidal profile generator.

Given a signed straight-line distance OR a signed in-place turn angle, plus
velocity/acceleration limits, generates a deterministic, ordered sequence of
``ProfileSetpoint`` twists: ``(elapsed, v_x, omega)``.

Shape
-----
Classic trapezoid: accelerate at ``a_max`` up to ``v_max``, cruise at exactly
``v_max``, decelerate at ``a_max`` back to exactly zero. When the input
distance/angle is too short for the accelerate/decelerate legs to ever reach
``v_max``, the cruise leg collapses to zero duration and the shape becomes a
triangle (accelerate straight into decelerate) -- a distinct shape from the
trapezoid, not a truncated/incorrect trapezoid.

Sign convention
----------------
The distance/angle sign is the travel direction (reverse/CW is negative) and
is preserved through EVERY setpoint in the returned sequence, including the
terminal one (which always lands at exactly zero velocity, never a
sign-reversal "creep back" -- binding requirement #7 of
``host-planner-design-lessons-from-drive-v2-review.md``). Callers must never
apply a direction-blind (``fabs``-style) predicate to any setpoint in the
sequence -- binding requirement #1 of the same issue.

Non-goals
---------
- No jerk limiting (no ``j_max`` term) -- explicitly out of scope this
  sprint (``architecture-update.md`` Step 1 finding 3).
- No opinion on sampling cadence beyond the ``cadence`` parameter -- the
  caller (ticket 005's ``StreamingExecutor``) owns that decision.
- No I/O, no wall-clock read, no robot/sim dependency -- this module is
  pure and has zero outward edges in the architecture's dependency graph.
- No latency modeling -- the ~130ms actuation-lag parameter (binding
  requirement #8) is consumed by ticket 005's ``planner/model.py`` and
  ``executor.py``, not by this generator.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class ProfileLimits:
    """Velocity/acceleration ceilings shared by both profile shapes.

    Both fields describe a MAGNITUDE (always positive) -- the profile
    functions apply the travel direction from the signed distance/angle
    argument, not from these limits.
    """

    v_max: float  # [mm/s] straight profiles, [rad/s] turn profiles; > 0
    a_max: float  # [mm/s^2] straight profiles, [rad/s^2] turn profiles; > 0


@dataclass(frozen=True)
class ProfileSetpoint:
    """One sampled twist setpoint of a generated profile."""

    elapsed: float  # [s] time since profile start; monotonically non-decreasing
    v_x: float  # [mm/s] signed forward velocity; 0 for turn profiles
    omega: float  # [rad/s] signed angular velocity; 0 for straight profiles


DEFAULT_CADENCE = 0.05  # [s] fallback sampling interval; callers may override


def profile_for_distance(
    distance: float,
    limits: ProfileLimits,
    cadence: float = DEFAULT_CADENCE,
) -> list[ProfileSetpoint]:
    """Trapezoidal straight-line profile.

    Args:
        distance: signed travel distance -- positive forward, negative
            reverse. Zero is rejected as degenerate, not a silent no-op.
        limits: v_max/a_max ceilings (magnitudes).
        cadence: sampling interval between setpoints.

    Returns:
        Ordered ``ProfileSetpoint`` sequence with ``v_x`` carrying the
        profile and ``omega`` held at exactly 0.0 throughout.

    Raises:
        ValueError: distance/limits/cadence fail validation (non-finite,
            non-positive limit, or zero distance).
    """
    samples = _scalar_trapezoidal_profile(distance, limits, cadence)
    return [ProfileSetpoint(elapsed=t, v_x=v, omega=0.0) for t, v in samples]


def profile_for_turn(
    angle: float,
    limits: ProfileLimits,
    cadence: float = DEFAULT_CADENCE,
) -> list[ProfileSetpoint]:
    """Trapezoidal in-place-turn profile.

    Args:
        angle: signed turn angle -- positive CCW, negative CW (matches the
            project's standard heading-angle sign convention). Zero is
            rejected as degenerate, not a silent no-op.
        limits: v_max/a_max ceilings (magnitudes), interpreted here as
            omega_max/alpha_max.
        cadence: sampling interval between setpoints.

    Returns:
        Ordered ``ProfileSetpoint`` sequence with ``omega`` carrying the
        profile and ``v_x`` held at exactly 0.0 throughout.

    Raises:
        ValueError: angle/limits/cadence fail validation (non-finite,
            non-positive limit, or zero angle).
    """
    samples = _scalar_trapezoidal_profile(angle, limits, cadence)
    return [ProfileSetpoint(elapsed=t, v_x=0.0, omega=v) for t, v in samples]


def _validate_limits(limits: ProfileLimits) -> None:
    if not math.isfinite(limits.v_max) or limits.v_max <= 0:
        raise ValueError(f"limits.v_max must be finite and positive, got {limits.v_max!r}")
    if not math.isfinite(limits.a_max) or limits.a_max <= 0:
        raise ValueError(f"limits.a_max must be finite and positive, got {limits.a_max!r}")


def _validate_scalar(value: float, name: str) -> None:
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite, got {value!r}")
    if value == 0.0:
        raise ValueError(f"{name} must be non-zero -- a zero-{name} profile is degenerate")


def _validate_cadence(cadence: float) -> None:
    if not math.isfinite(cadence) or cadence <= 0:
        raise ValueError(f"cadence must be finite and positive, got {cadence!r}")


def _scalar_trapezoidal_profile(
    distance: float,
    limits: ProfileLimits,
    cadence: float,
) -> list[tuple[float, float]]:
    """Shared accelerate/cruise/decelerate timing math.

    Works identically whether ``distance`` is a linear distance (mm) or an
    angle (rad) -- the shape math only cares about the scalar magnitude and
    the v_max/a_max ceilings. Returns ``(elapsed, signed_velocity)`` pairs;
    ``profile_for_distance``/``profile_for_turn`` map the SAME sequence onto
    ``v_x``/``omega`` respectively, so the shape logic is never duplicated.
    """
    _validate_scalar(distance, "distance")
    _validate_limits(limits)
    _validate_cadence(cadence)

    direction = 1.0 if distance > 0 else -1.0
    magnitude = abs(distance)
    v_max = limits.v_max
    a_max = limits.a_max

    # Distance covered while accelerating (== decelerating) from 0 to v_max.
    t_acc_to_v_max = v_max / a_max
    d_acc_to_v_max = 0.5 * a_max * t_acc_to_v_max**2

    if 2 * d_acc_to_v_max <= magnitude:
        # Trapezoid: cruise leg reached.
        t_acc = t_acc_to_v_max
        v_peak = v_max
        d_cruise = magnitude - 2 * d_acc_to_v_max
        t_cruise = d_cruise / v_max
    else:
        # Triangle: cruise never reached: solve v_peak from
        # magnitude == v_peak^2 / a_max (equal accel/decel legs).
        v_peak = math.sqrt(a_max * magnitude)
        t_acc = v_peak / a_max
        t_cruise = 0.0

    t_decel_start = t_acc + t_cruise
    total_time = t_decel_start + t_acc

    def velocity_at(t: float) -> float:
        if t <= t_acc:
            return a_max * t
        if t <= t_decel_start:
            return v_peak
        return max(v_peak - a_max * (t - t_decel_start), 0.0)

    setpoints: list[tuple[float, float]] = []
    t = 0.0
    # Small epsilon avoids emitting a spurious near-duplicate sample when
    # total_time happens to land almost exactly on a cadence multiple.
    while t < total_time - 1e-9:
        setpoints.append((t, direction * velocity_at(t)))
        t += cadence
    # Terminal setpoint always lands at EXACTLY zero velocity -- never a
    # sign-reversal creep-back (binding requirement #7).
    setpoints.append((total_time, 0.0))
    return setpoints
