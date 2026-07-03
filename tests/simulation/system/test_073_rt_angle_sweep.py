"""test_073_rt_angle_sweep.py — RT angle-sweep regression (ticket 073-004).

Headline acceptance test for sprint 073. A fresh, ZERO-configuration ``Sim()``
(default ``RobotConfig``, no injected sim error, no explicit ``SET``/``SIMSET``
override beyond the ``sim`` fixture's own watchdog-timeout extension — see
``tests/conftest.py``, unrelated to RT/slip/scrub) issuing a bare
``RT <cdeg>`` should land within a documented, tight tolerance of the
commanded angle, measured against the plant's TRUE pose
(``sim.get_true_pose()``), never the encoder/OTOS estimate.

This exercises Tickets 001 and 002 TOGETHER, not independently:

- Ticket 001 alone (coast anticipation derived from live ramp-down dynamics)
  would still leave the ~+8.7% slip-driven over-rotation at every angle — a
  fresh ``Sim()``'s plant did not yet scrub by ``rotationalSlip`` at all, so
  ``beginRotation()``'s slip-inflated arc target was executed in full.
- Ticket 002 alone (seeding the plant's body-rotational scrub from
  ``cfg.rotationalSlip`` at ``SimHandle`` construction) would still leave the
  ~3.3° constant coast-anticipation gap at every angle — the stale,
  hand-tuned ``kRtCoastArc = 8.0mm`` constant assumed a 100°/s cruise the
  actual ``yawRateMax = 70°/s`` cap never reaches.

Only the combination lands inside the tolerance below across the full
45°-300° sweep.

Measured (this ticket, same build as the rest of sprint 073's tickets;
``true`` and ``diff`` are the plant's true heading and its signed,
wrap-normalized difference from the commanded angle):

    45°  (cdeg=4500):  true=46.10°   diff=+1.10°
    90°  (cdeg=9000):  true=91.01°   diff=+1.01°
    180° (cdeg=18000): true=180.59°  diff=+0.59°  (heading wraps at ±180°)
    300° (cdeg=30000): true=300.93°  diff=+0.93°  (heading wraps at ±180°)

Bound: ``_TOL_DEG = 1.25`` degrees. This documents "lands within ~1°" (the
sprint's own headline claim) precisely: it absorbs the measured worst case
(45°, +1.10°) with a small margin for tick-granularity/floating-point
variance across build platforms, while staying an order of magnitude tighter
than the pre-073 tests' 3-5° tolerances (which existed specifically to
absorb the two now-fixed defects above — see
``tests/simulation/system/test_069_rt_90deg_body_scrub.py``'s
``_NEAR_90_TOL_DEG = 5.0``, pre-existing and intentionally left wide for a
different, still-open residual, per that file's own module docstring).
"""
from __future__ import annotations

import math

import pytest

# See module docstring: measured worst case across the sweep is +1.10° (45°).
# 1.25° gives headroom for platform/tick-granularity variance without
# reopening the old, multi-degree coast/slip-driven tolerances this sprint's
# tickets 001+002 exist to eliminate.
_TOL_DEG = 1.25


def _angle_diff_deg(true_deg: float, commanded_deg: float) -> float:
    """Signed angular difference (true - commanded), wrapped to (-180, 180].

    Needed because ``sim.get_true_pose()``'s heading (and PhysicsWorld's own
    accumulator) wraps to (-pi, pi] — a straight subtraction is wrong for the
    180°/300° cases (e.g. true=-179.4° vs commanded=180.0° is a ~0.6° miss,
    not a ~359.4° one).
    """
    return (true_deg - commanded_deg + 180.0) % 360.0 - 180.0


@pytest.mark.parametrize("cdeg", [4500, 9000, 18000, 30000], ids=["45deg", "90deg", "180deg", "300deg"])
def test_rt_lands_within_tolerance_clean_sim(sim, cdeg):
    """A fresh, zero-configuration Sim()'s ``RT <cdeg>`` lands within
    ``_TOL_DEG`` of the commanded angle, measured from plant ground truth.

    The ``sim`` fixture (``tests/conftest.py``) constructs a brand-new
    ``Sim()`` per test — a fresh default ``RobotConfig``, no ``SET
    rotSlip=...``, no ``SIMSET bodyRotScrub=...``/``bodyLinScrub=...`` — the
    exact "clean sim, neutral profile" scenario the sprint issue names.
    """
    commanded_deg = cdeg / 100.0

    reply = sim.send_command(f"RT {cdeg}")
    assert "OK" in reply.upper(), f"RT {cdeg} -> unexpected reply {reply!r}"

    sim.tick_for(8000)

    _, _, true_h_rad = sim.get_true_pose()
    true_deg = math.degrees(true_h_rad)
    diff = _angle_diff_deg(true_deg, commanded_deg)

    assert abs(diff) < _TOL_DEG, (
        f"RT {cdeg} ({commanded_deg}° commanded) landed at {true_deg:.2f}° true "
        f"(diff={diff:+.2f}°); expected within {_TOL_DEG}° of commanded "
        f"for a clean, zero-configuration Sim() (sprint 073 combined coast+scrub fix)"
    )
