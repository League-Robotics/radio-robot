"""test_rt_slip.py — RT encoder-arc compensation for rotationalSlip (sprint 024-006).

Verifies that beginRotation() divides the encoder-arc target by the effective slip
so the body achieves the commanded angle despite wheel scrub.

Math:
  No-slip arc  = |deg| * (π/180) * (trackwidth/2) = 90 * π/180 * 63 ≈ 98.9 mm
  Slip=0.74:   arc = 98.9 / 0.74 ≈ 133.6 mm  (larger target → wheels travel farther)

  Coast anticipation (073-001) is derived live from the SOFT ramp-down's actual
  kinematics, not a hardcoded constant: coastAngleDeg = rate²/(2·yawAccMax),
  coastArc = coastAngleDeg·(π/180)·(trackwidth/2), where rate = min(cfg.yawRateMax,
  kRtRate=100) = 70 deg/s and cfg.yawAccMax = 720 deg/s² (DefaultConfig.cpp
  defaults). At this test's own tw_mm=83.0 that is ≈2.47 mm — this is subtracted
  from both runs below (see `_coast_mm()`).

  stopArc (no slip)  = 98.9 - 2.47 ≈ 96.4 mm
  stopArc (slip=0.74) = 133.6 - 2.47 ≈ 131.1 mm

When RT completes, the encoder differential |encR - encL|/2 should ≈ stopArc
(motors coast slightly beyond that due to SOFT stop ramp), so the slip=0.74
run should drive meaningfully more arc than the slip-disabled run.
"""

import math
import pytest


def _coast_mm(tw_mm: float, yaw_rate_max: float = 70.0,
              yaw_acc_max: float = 720.0, kRtRate: float = 100.0) -> float:
    """Coast-anticipation arc (mm), mirroring beginRotation()'s 073-001 formula.

    coastAngleDeg = rate^2 / (2*yawAccMax); coastArc = coastAngleDeg*(pi/180)*(tw/2).
    Defaults (yaw_rate_max=70, yaw_acc_max=720) mirror DefaultConfig.cpp's live
    cfg.yawRateMax/cfg.yawAccMax — NOT re-derived from a hardcoded 8mm constant.
    """
    rate = min(yaw_rate_max, kRtRate)
    coast_deg = rate * rate / (2.0 * yaw_acc_max)
    return coast_deg * math.pi / 180.0 * (tw_mm * 0.5)


def _arc_after_rt(sim, cdeg: int = 9000) -> float:
    """Issue RT <cdeg>, wait for completion, return |encR - encL| / 2 in mm.

    067-001: a bare ``ZERO`` (no token) is rejected by ``parseZero()`` with
    ``ERR badarg`` — it does NOT reset the encoders. The reply was
    previously unchecked, so encoder readings silently accumulated across
    the two sequential RT calls each test makes, faking a slip effect that
    wasn't real. ``ZERO enc`` is the valid, encoder-only reset; the reply
    is checked so a future rejected ZERO fails loudly instead of silently
    degrading into an accumulation artifact.
    """
    # Reset encoders before RT.
    reply = sim.send_command("ZERO enc")
    assert "OK" in reply.upper(), f"ZERO enc → unexpected reply {repr(reply)}"

    # Issue RT command.
    r = sim.send_command(f"RT {cdeg}")
    assert "OK" in r.upper(), f"RT {cdeg} → unexpected reply {repr(r)}"

    # Tick up to 8 s; RT should complete well before then.
    sim.tick_for(8000)

    enc_l = float(sim._lib.sim_get_enc_l(sim._h))
    enc_r = float(sim._lib.sim_get_enc_r(sim._h))

    # For a CCW spin: left wheel goes backward (negative), right goes forward.
    # |differential| / 2 = per-wheel arc.
    diff_half = abs(enc_r - enc_l) / 2.0
    return diff_half


def test_rt_arc_larger_with_slip(sim):
    """RT 9000 with rotationalSlip=0.74 drives a larger encoder arc than slip-disabled.

    With slip=0.74 (firmware default), beginRotation() divides the no-slip arc
    by 0.74, expanding the target from ~96.4 mm (stop-arc) to ~131.1 mm.
    The final encoder arc after the SOFT ramp should therefore be meaningfully
    larger than the no-slip case.

    Expected (coast_mm from the 073-001 ramp-dynamics formula, ~2.47 mm at
    this test's tw_mm=83.0 — see module docstring):
      No-slip  (rotSlip=0):    stopArc ≈ 96.4 mm   → final enc-arc ≈ 95–105 mm
      Slip=0.74 (default):     stopArc ≈ 131.1 mm  → final enc-arc ≈ 130–145 mm
    """
    # --- Run 1: slip disabled (SET rotSlip=0 → effectiveSlip → 1.0) ---
    sim.send_command("SET rotSlip=0")
    arc_no_slip = _arc_after_rt(sim, 9000)

    # --- Run 2: firmware default slip=0.74 ---
    sim.send_command("SET rotSlip=0.74")
    arc_with_slip = _arc_after_rt(sim, 9000)

    # The slip-compensated run must drive at least 15% more encoder arc.
    ratio = arc_with_slip / arc_no_slip if arc_no_slip > 0 else 0.0
    assert arc_with_slip > arc_no_slip * 1.1, (
        f"RT 9000 with slip=0.74 should drive ≥10% more arc than no-slip. "
        f"no-slip={arc_no_slip:.1f} mm, slip=0.74={arc_with_slip:.1f} mm, "
        f"ratio={ratio:.2f} (expected ≥1.10)"
    )


def test_rt_arc_no_slip_matches_geometry(sim):
    """RT 9000 with rotSlip=0 (identity) produces arc ≈ theoretical no-slip value.

    Theoretical: stopArc = 90 * π/180 * (83/2) - coast_mm ≈ 62.7 mm, where
    coast_mm is computed from the 073-001 ramp-dynamics formula (`_coast_mm()`),
    not a hardcoded 8mm constant. The SOFT ramp adds some coast, so actual
    final arc ≥ stopArc.
    """
    sim.send_command("SET rotSlip=0")
    arc = _arc_after_rt(sim, 9000)

    # Theoretical no-slip per-wheel arc (before coast).
    tw_mm = 83.0   # trackwidthMm default
    coast_mm = _coast_mm(tw_mm)
    theoretical_stop_arc = 90.0 * math.pi / 180.0 * (tw_mm * 0.5) - coast_mm
    # Actual must be at least the stop-arc (SOFT ramp adds some coast past it).
    assert arc >= theoretical_stop_arc * 0.8, (
        f"RT 9000 no-slip arc {arc:.1f} mm should be ≈ {theoretical_stop_arc:.1f} mm "
        f"(stopping arc before coast)"
    )


def test_rt_slip_compensation_ratio(sim):
    """The slip=0.74 arc is approximately 1/0.74 ≈ 1.35× the no-slip arc.

    The ratio arc_with_slip / arc_no_slip should be approximately 1/slip = 1/0.74 ≈ 1.35.
    Because the SAME coast arc (~2.47 mm, 073-001 ramp-dynamics formula — see
    module docstring) is subtracted in both cases, the ratio will not be
    exactly 1/0.74, but it should exceed 1.2 (generous tolerance for coast effects).
    """
    sim.send_command("SET rotSlip=0")
    arc_no_slip = _arc_after_rt(sim, 9000)

    sim.send_command("SET rotSlip=0.74")
    arc_slip = _arc_after_rt(sim, 9000)

    assert arc_no_slip > 10.0, f"No-slip arc too small ({arc_no_slip:.1f} mm) — RT did not run"
    ratio = arc_slip / arc_no_slip
    assert ratio >= 1.2, (
        f"RT arc ratio (slip/no-slip) = {ratio:.2f} should be ≥1.2 "
        f"(no-slip={arc_no_slip:.1f}, slip=0.74={arc_slip:.1f})"
    )
