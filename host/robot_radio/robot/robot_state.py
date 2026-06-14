"""RobotState — composite motion-state dataclass for the Nezha robot.

Carries the full TLM payload — pose, encoders, twist, line sensor, color,
optional world pose, and a host-side timestamp — as one coherent frozen
snapshot.  Built by ``Nezha._apply_tlm`` from each incoming TLM frame.

Unit conventions:
    pose.x / pose.y  : millimetres (body-frame OTOS position, matching TLM)
    pose.heading     : radians (CCW-positive, standard maths convention)
    encoders         : (left_mm, right_mm) cumulative encoder totals in mm
    twist            : (v_mmps, omega_mradps) fused body-frame velocity
    line             : (g1, g2, g3, g4) raw line-sensor ADC counts
    color            : (r, g, b, c) raw colour-sensor ADC counts
    world_pose       : (x_cm, y_cm, heading_rad) optional camera-calibrated
                       world position; None until set by update_world_pose()
    v                : mm/s (body-frame forward speed, EKF-fused)
    omega            : rad/s (yaw rate, CCW-positive, EKF-fused)
    accel            : (ax_mmps2, ay_mmps2) body-frame acceleration, or None
    stamp            : time.monotonic() seconds at the host when built
"""

from __future__ import annotations

from dataclasses import dataclass

from robot_radio.nav.pose import Pose


@dataclass(frozen=True)
class RobotState:
    """Composite frozen snapshot of the full TLM payload for one frame.

    All fields that were absent from the incoming TLM frame retain the value
    they held in the *previous* state (partial-frame handling).

    Parameters
    ----------
    pose:
        Robot OTOS position and heading.  ``x`` and ``y`` are in millimetres
        (matching the TLM wire format); ``heading`` is in radians
        (CCW-positive).
    encoders:
        Cumulative encoder totals as ``(left_mm, right_mm)`` in mm, or
        ``None`` if no encoder frame has been received yet.
    twist:
        Fused body-frame velocity as ``(v_mmps, omega_mradps)``, or ``None``
        if the ``twist=`` field was absent from all frames so far.
    line:
        Raw line-sensor ADC counts as ``(g1, g2, g3, g4)``, or ``None``.
    color:
        Raw colour-sensor ADC counts as ``(r, g, b, c)``, or ``None``.
    world_pose:
        Camera-calibrated world position as ``(x_cm, y_cm, heading_rad)``,
        or ``None`` until explicitly set via ``Nezha.update_world_pose()``.
    v:
        Body-frame forward speed in mm/s (EKF-fused).
    omega:
        Yaw rate in rad/s, CCW-positive (EKF-fused).
    accel:
        Body-frame linear acceleration as ``(ax_mmps2, ay_mmps2)``, or
        ``None`` when the frame carries no accel data.
    stamp:
        ``time.monotonic()`` seconds at the host when this state was built.
    """

    pose: Pose
    v: float
    omega: float
    accel: tuple[float, float] | None
    stamp: float
    # New fields — all default to None so callers that construct RobotState
    # explicitly (e.g. tests) need not supply them.
    encoders: tuple[int, int] | None = None
    twist: tuple[int, int] | None = None
    line: tuple[int, int, int, int] | None = None
    color: tuple[int, int, int, int] | None = None
    world_pose: tuple[float, float, float] | None = None
    # Raw OTOS (optical odometry sensor) pose as ``(x_mm, y_mm, yaw_rad)``,
    # pre-fusion, from the TLM ``otos=`` field — distinct from ``pose`` (the
    # encoder/EKF-fused pose).  ``None`` until an ``otos=`` field is seen (the
    # firmware omits it when the OTOS read is stale/invalid, e.g. lifted).
    otos_pose: tuple[float, float, float] | None = None
