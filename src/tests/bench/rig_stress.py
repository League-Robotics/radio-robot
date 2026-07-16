"""src/tests/bench/rig_stress.py -- quality metrics + stress battery for the HITL
drivetrain (rig_drive). Measures the QUALITY of the drive output: velocity-trace
discontinuities, commanded-acceleration over-limit, heading overshoot/error, and
distance accuracy cross-checked against the line-sensor SECONDARY encoder (the
printed pattern on drum wheel-1, 256.1 mm/revolution, n0 = once-per-rev index).
"""
from __future__ import annotations

import math

import numpy as np

import rig_drive

DRUM_MM_PER_REV = rig_drive.DRUM_MM_PER_REV


def _a(rows, k):
    return np.array([(r.get(k) if r.get(k) is not None else np.nan) for r in rows], float)


def secondary_distance(rows) -> tuple[float, int]:
    """Independent motor-1 (drum) travel from the line index: count n0 rising
    edges (once per revolution) and interpolate. Returns (distance_mm, revs)."""
    pos = _a(rows, "pos_l")
    n0 = _a(rows, "n0")
    hi = n0 > 500
    valid = ~np.isnan(n0)
    edges = []
    for i in range(1, len(rows)):
        if valid[i] and valid[i - 1] and hi[i] and not hi[i - 1] and not np.isnan(pos[i]):
            edges.append(pos[i])
    revs = len(edges)
    return revs * DRUM_MM_PER_REV, revs


def quality(rows, target_deg=0.0, target_arc=0.0, wheel_accel_limit=200.0,
            disc_thresh=120.0):
    """Compute quality metrics for one drive trace.
      wheel_accel_limit [mm/s^2]  -- profile ceiling to flag over-acceleration.
      disc_thresh [mm/s]          -- commanded wheel-velocity step flagged as a
                                     discontinuity (a jump no smooth profile makes).
    """
    t = _a(rows, "t")
    dt = np.diff(t)
    dt = np.where(dt <= 0, 1e-3, dt)
    cmd_l, cmd_r = _a(rows, "cmd_l"), _a(rows, "cmd_r")
    vel_l, vel_r = _a(rows, "vel_l"), _a(rows, "vel_r")
    heading = _a(rows, "heading_enc")

    # commanded wheel acceleration (the tracker's own output profile)
    acc_l = np.abs(np.diff(cmd_l) / dt)
    acc_r = np.abs(np.diff(cmd_r) / dt)
    max_cmd_accel = float(np.nanmax(np.concatenate([acc_l, acc_r])))
    over_accel = int(np.nansum(acc_l > wheel_accel_limit) + np.nansum(acc_r > wheel_accel_limit))

    # commanded-velocity discontinuities (single-tick jumps beyond disc_thresh)
    step_l = np.abs(np.diff(cmd_l))
    step_r = np.abs(np.diff(cmd_r))
    discontinuities = int(np.nansum(step_l > disc_thresh) + np.nansum(step_r > disc_thresh))
    max_cmd_step = float(np.nanmax(np.concatenate([step_l, step_r])))

    # wheel tracking: measured vs commanded (includes actuation lag)
    trk = np.concatenate([(vel_l - cmd_l), (vel_r - cmd_r)])
    track_rms = float(np.sqrt(np.nanmean(trk ** 2)))

    # heading overshoot + final error (turn)
    final_heading = float(heading[-1])
    if target_deg > 0:
        overshoot = float(max(0.0, np.nanmax(heading) - target_deg))
    elif target_deg < 0:
        overshoot = float(max(0.0, target_deg - np.nanmin(heading)))
    else:
        overshoot = 0.0
    heading_err = final_heading - target_deg

    # distance accuracy: encoder (motor-1 travel) vs secondary line encoder vs commanded
    pos_l = _a(rows, "pos_l")
    enc_dist = float(np.nanmax(pos_l) - np.nanmin(pos_l))
    sec_dist, revs = secondary_distance(rows)

    return {
        "n": len(rows),
        "max_cmd_accel": max_cmd_accel, "over_accel": over_accel,
        "discontinuities": discontinuities, "max_cmd_step": max_cmd_step,
        "track_rms": track_rms,
        "final_heading": final_heading, "heading_err": heading_err, "overshoot": overshoot,
        "enc_dist": enc_dist, "sec_dist": sec_dist, "sec_revs": revs,
        "target_deg": target_deg, "target_arc": target_arc,
    }


def wheel_accel_limit_for(limits) -> float:
    """[mm/s^2] wheel-level accel ceiling implied by the rotational+linear
    profile (whichever a pivot/move would hit): max wheel accel = rot_accel *
    trackwidth/2 + linear_accel."""
    return limits.rotational.accel * rig_drive.TRACKWIDTH / 2.0 + limits.linear.accel
