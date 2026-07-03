"""
test_069_rt_90deg_body_scrub.py — ticket 069-002 headline acceptance points.

``PlannerBegin.cpp::beginRotation()`` computes the RT command's per-wheel
encoder-arc target as ``arc = |Δθ|·(tw/2)/effectiveSlip(cfg.rotationalSlip)``
— INFLATING the commanded arc so the body still reaches the requested angle
despite the REAL chassis's rotational scrub (``rotationalSlip`` defaults to
0.92, see ``data/robots/*.json``). Before this ticket, ``PhysicsWorld``'s
plant had no way to actually scrub by that factor (its only rotation-scaling
channel, ``_rotationalSlip``/``setSlip()``, is a test-infra encoder-defect
knob every current test drives to ``<= 0``, i.e. no plant scrub) — so a
zero-error sim plant received the inflated command and executed all of it,
over-rotating (RT 9000 → ~94-96° true instead of 90°, depending on tick
granularity; architecture-update.md's own measurement was ~95.2°).

This test drives ticket 002's new, independent body-rotational/linear scrub
fields (``PhysicsWorld::setBodyRotationalScrub``/``setBodyLinearScrub``)
through the ticket 003 ``SIMSET bodyRotScrub=…``/``SIMSET bodyLinScrub=…``
wire-command surface (rebased from the direct ``sim_set_body_rot_scrub()``/
``sim_set_body_lin_scrub()`` ctypes hooks ticket 002 used ahead of ``SIMSET``
landing — those ctypes forwards are NOT deleted; they remain a valid,
alternate entry point, see architecture-update.md Migration Concerns).  RT is
already a real wire command, so this exercises the full command-dispatch
pipeline end to end, and asserts the plant's TRUE pose
(``sim.get_true_pose()``, not the encoder/OTOS estimates) lands close to the
commanded 90° once the body-rotational scrub is configured to match
``rotSlip``.

Tolerance note: RT's own stop-arc coast constant (``PlannerBegin.cpp``'s
``kRtCoastArcMm=8mm``, commented "~7.3° SOFT-ramp coast at 100°/s
(sim-tuned)") assumes a spin rate of ``kRtRateDps=100`` °/s, but the actual
spin rate is capped to ``min(cfg.yawRateMax, kRtRateDps)`` — DefaultConfig's
``yawRateMax`` is 70°/s, below that assumption — leaving a small (~2-4°),
constant, slip/scrub-independent residual in every RT 9000 run. That
residual is a pre-existing ``PlannerBegin.cpp`` coast-tuning artifact, not
something ticket 069-002 is scoped to fix (the ticket, and
architecture-update.md, explicitly say not to touch
``Planner::beginRotation()``) — so the tolerances below are set wide enough
to absorb it while still tightly proving this ticket's actual mathematical
claim: that ``bodyRotScrub == rotationalSlip`` exactly cancels the arc
inflation, reproducing the identity (no-inflation, no-scrub) run's result.
"""
from __future__ import annotations

import math


def _true_heading_deg_after_rt(
    sim,
    rot_slip: float,
    body_rot_scrub: float | None = None,
    body_lin_scrub: float | None = None,
    cdeg: int = 9000,
) -> float:
    """Reset ground truth + encoders, configure slip/scrub, run RT <cdeg>,
    return the plant's TRUE heading in degrees.

    ``body_rot_scrub``/``body_lin_scrub`` of ``None`` leaves ``PhysicsWorld``'s
    own default (1.0 = no-op) untouched — used for the "identity" acceptance
    point so the test proves the DEFAULT is a no-op, not just that passing
    1.0 explicitly is a no-op. When calling this helper more than once
    against the SAME ``sim`` (to compare scenarios), pass explicit values for
    every scrub field every time — the underlying ``PhysicsWorld`` is a
    single persistent object for the life of the ``Sim``, so a scrub set by
    an earlier scenario is NOT reset by ``ZERO enc``/``set_true_pose``.

    Ticket 003: body_rot_scrub/body_lin_scrub are applied via ``SIMSET
    bodyRotScrub=…``/``SIMSET bodyLinScrub=…`` sent through the normal
    command-dispatch pipeline, not the ticket-002 ctypes hooks directly.
    """
    sim.set_true_pose(0.0, 0.0, 0.0)
    reply = sim.send_command("ZERO enc")
    assert "OK" in reply.upper(), f"ZERO enc → unexpected reply {reply!r}"

    reply = sim.send_command(f"SET rotSlip={rot_slip}")
    assert "OK" in reply.upper(), f"SET rotSlip={rot_slip} → unexpected reply {reply!r}"

    if body_rot_scrub is not None:
        reply = sim.send_command(f"SIMSET bodyRotScrub={body_rot_scrub}")
        assert "OK" in reply.upper(), (
            f"SIMSET bodyRotScrub={body_rot_scrub} → unexpected reply {reply!r}"
        )
    if body_lin_scrub is not None:
        reply = sim.send_command(f"SIMSET bodyLinScrub={body_lin_scrub}")
        assert "OK" in reply.upper(), (
            f"SIMSET bodyLinScrub={body_lin_scrub} → unexpected reply {reply!r}"
        )

    reply = sim.send_command(f"RT {cdeg}")
    assert "OK" in reply.upper(), f"RT {cdeg} → unexpected reply {reply!r}"

    sim.tick_for(8000)

    _, _, true_h = sim.get_true_pose()
    return math.degrees(true_h)


# Wide enough to absorb PlannerBegin.cpp's pre-existing, out-of-scope RT
# coast-tuning residual (see module docstring); still far tighter than the
# ~95°-vs-90° gap this ticket closes.
_NEAR_90_TOL_DEG = 5.0


def test_rt_90deg_with_body_scrub_matching_rot_slip(sim):
    """Headline acceptance point 1: bodyRotScrub=rotSlip closes the RT 9000
    over-rotation gap.

    ``SET rotSlip=0.92`` (``RobotConfig.rotationalSlip``'s default) inflates
    ``PlannerBegin.cpp``'s commanded arc by ``1/0.92``;
    ``sim_set_body_rot_scrub(0.92)`` makes the plant genuinely scrub by that
    same factor, cancelling the inflation. True pose should land close to
    the commanded 90°, not the ~94-96° a zero-scrub plant produces for the
    same command (see the comparison test below).
    """
    true_h_deg = _true_heading_deg_after_rt(sim, rot_slip=0.92, body_rot_scrub=0.92)
    assert abs(true_h_deg - 90.0) < _NEAR_90_TOL_DEG, (
        f"RT 9000 with rotSlip=0.92 + bodyRotScrub=0.92 should land near 90° "
        f"true (closing the ~95° over-rotation gap); got {true_h_deg:.2f}°"
    )


def test_rt_90deg_identity_no_scrub(sim):
    """Headline acceptance point 2: rotSlip=1.0 (identity) with both new
    scrub fields left at their default (1.0, no-op) lands RT 9000 near 90°
    true — the plant is never asked to scrub at all, so this pins the
    "no correction needed" baseline the scrub math must reproduce when
    correction IS needed (point 1, above).
    """
    true_h_deg = _true_heading_deg_after_rt(sim, rot_slip=1.0)
    assert abs(true_h_deg - 90.0) < _NEAR_90_TOL_DEG, (
        f"RT 9000 with rotSlip=1.0 (identity) and default (no-op) scrub "
        f"fields should land near 90° true; got {true_h_deg:.2f}°"
    )


def test_rt_scrub_cancellation_matches_identity_not_uncorrected_baseline(sim):
    """The core mathematical claim: bodyRotScrub=rotSlip's cancellation
    reproduces the IDENTITY run's result (both land at the same true
    rotation, modulo PlannerBegin.cpp's shared coast-tuning residual — see
    module docstring), and is clearly different from the UNCORRECTED
    (rotSlip=0.92, scrub left at its 1.0 default) baseline that over-rotates.

    This assertion is the most robust to the shared RT coast-tuning artifact
    (present, with the same sign/magnitude, in both the corrected and
    identity runs) and most directly pins this ticket's own code change:
    without a genuine, independent body-rotational scrub (this ticket's
    PhysicsWorld addition), the "corrected" run would be identical to the
    uncorrected baseline (nothing to cancel the inflation with) instead of
    matching the identity run.
    """
    uncorrected_deg = _true_heading_deg_after_rt(sim, rot_slip=0.92, body_rot_scrub=1.0)
    corrected_deg = _true_heading_deg_after_rt(sim, rot_slip=0.92, body_rot_scrub=0.92)
    identity_deg = _true_heading_deg_after_rt(sim, rot_slip=1.0, body_rot_scrub=1.0)

    assert abs(corrected_deg - identity_deg) < 3.0, (
        f"scrub-corrected run ({corrected_deg:.2f}°) should closely match "
        f"the identity (no-inflation, no-scrub) run ({identity_deg:.2f}°) — "
        f"both share only PlannerBegin.cpp's coast-tuning residual, not any "
        f"slip-driven inflation"
    )
    assert abs(corrected_deg - uncorrected_deg) > 3.0, (
        f"scrub-corrected run ({corrected_deg:.2f}°) should be clearly "
        f"different from the uncorrected baseline ({uncorrected_deg:.2f}°) — "
        f"otherwise the new scrub fields are having no effect"
    )
