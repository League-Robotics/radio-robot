"""Off-hardware acceptance proof for ticket 084-003 (SUC-002): registers
``R``/``TURN``/``RT`` as top-level wire verbs, extending ticket 084-002's
``S``/``T``/``D``/``STOP`` family (``source/commands/motion_commands.cpp``)
with the constant-curvature arc (``R``), absolute-heading turn-in-place
(``TURN``), and relative turn-in-place (``RT``) goal kinds.

Like ``test_motion_commands.py``, this drives ``libfirmware_host`` through
the full wire dispatch (``Sim.command()``) -- ``CommandProcessor`` ->
``source/commands/motion_commands.cpp`` -> ``MotionLoopState``'s outbox ->
``dev_loop.cpp``'s drain step -> ``Subsystems::Planner`` -> the simulated
plant.

``TURN``/``RT`` are the FIRST verbs to close against ``Planner::tick()``'s
``fusedPose`` argument (``TURN``) and the per-wheel encoder arc (``RT``) --
both self-terminating, unlike ``R`` (open-loop, matches ``S``'s family:
runs until stopped or a ``stop=`` clause fires). Per the ticket's own
testing note, ``TURN``/``RT`` accuracy shows small over-rotation from the
SMOOTH stop style's ramp-down coast (no coast-anticipation this ticket --
see ``handleRT``'s doc comment) -- tolerances below are set from the
measured plant behavior, documented at each assertion.
"""

import math

import pytest


def _wrap_pi(angle: float) -> float:   # [rad]
    """Wrap an angle into (-pi, pi] -- 086-002 helper. 180 deg is the
    heading representation's own branch cut: a converged heading an epsilon
    PAST it reads back as slightly UNDER -180 deg, so a raw ``h - expected``
    difference against a target of exactly +180 deg can blow up to ~2*pi
    even though the true angular distance is tiny. Only
    ``test_turn_reaches_absolute_heading_from_nonzero_start`` targets
    exactly +-180 deg (the sole target sitting on the branch cut); every
    other assertion in this file targets a heading far enough from it that
    the raw difference was never at risk, so this helper is applied there
    only rather than rewriting every assertion in the file."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@pytest.mark.xfail(
    reason=(
        "087-007: the real cyclic executive's synchronous-update discipline "
        "(architecture-update-r1.md Decision 6) adds a uniform one-tick-per-"
        "hop latency to the Planner->Drivetrain->Hardware command path "
        "(Planner's output -> bb.driveIn, drained by Drivetrain next pass; "
        "Drivetrain's output -> bb.motorIn[], drained by Hardware the pass "
        "after that -- Decision 2's per-port unpack), versus ticket 006's "
        "transitional same-pass feed-forward. RT's terminal rotation "
        "overshoot (086-004's own hard-won, precisely-measured 96.3669deg / "
        "+6.37deg-over-90 bound) is now a deterministic, bit-exact "
        "99.30046deg / +9.30deg-over-90 -- a genuine terminal-decel-"
        "anticipation control-accuracy regression caused by the added "
        "latency, not a test-tolerance nuisance. Widening the tolerance "
        "here would silently mask exactly the class of regression 086-004 "
        "fought to characterize precisely. Control/anticipation retuning "
        "for the added latency is ticket 009's scope (preserve-serial-"
        "silence-safety-watchdog-in-greenfield-loop.md's sibling design "
        "issue's Open Question 1) -- this xfail must be lifted once 009 "
        "retunes Planner's STOP_ROTATION anticipation (or the tolerance is "
        "re-measured and re-tightened) against the new latency."
    ),
    strict=True,
)
def test_rt_rotates_about_90_degrees_and_emits_done_rot(sim):
    reply = sim.command("RT 9000")
    assert reply.strip() == "OK rt rot=9000"

    sim.tick_for(3000)

    _x, _y, h = sim.true_pose()
    # Measured plant behavior (2026-07-05, pre-086): the ROTATION stop fires
    # once the per-wheel arc target is reached (a ">=" threshold, no
    # early-fire margin), then the SMOOTH stop style ramps yaw rate to zero
    # -- coasting a further ~4-5 degrees past the target before fully
    # stopping (no coast-anticipation this ticket -- see handleRT's doc
    # comment).
    #
    # 086-004 retune: re-measured against 086-002 (motor-loop reverse-spin
    # fix) + 086-003 (Planner terminal-decel anticipation, extended to
    # STOP_ROTATION too) -- this converges to a fully deterministic
    # 96.3669 deg (+6.37 deg over 90), bit-exact across repeated runs (this
    # is a pure sim tick loop, no wall-clock jitter). Still the same
    # already-documented, deliberately-open-loop RT coast characteristic
    # (086-003's ROTATION anticipation is a documented approximation, per
    # that ticket's own completion notes, and does not close this residual
    # to near-zero the way it closed D 200 200 500's) -- not a regression.
    # tests/sim/system/test_tour_geometry.py's own per-leg tolerance
    # (+-8 deg, set from the widest RT leg observed across Tour 1/2) uses
    # the same measured behavior. +-7 degrees here keeps ~1.1x headroom over
    # the exact 6.37 deg measured, tightened from the old +-10 deg (which
    # was never measured this precisely).
    expected = math.pi / 2.0
    assert abs(h - expected) < math.radians(7.0), (
        f"expected heading near {expected:.4f} rad (90 deg), got {h:.4f} rad "
        f"({math.degrees(h):.2f} deg)"
    )

    evts = sim.get_async_evts()
    assert "EVT done RT reason=rot" in evts


@pytest.mark.xfail(
    reason=(
        "087-007: symmetric with test_rt_rotates_about_90_degrees_and_"
        "emits_done_rot's own xfail above -- the added Decision-6/2 "
        "per-hop latency shifts this leg's deterministic overshoot from "
        "-96.3669deg to -99.30046deg (-9.30deg over -90, vs the tightened "
        "+-7deg bound). See that test's xfail reason for the full "
        "explanation; lifted once ticket 009 retunes for the added latency."
    ),
    strict=True,
)
def test_rt_negative_relangle_rotates_the_opposite_direction(sim):
    reply = sim.command("RT -9000")
    assert reply.strip() == "OK rt rot=-9000"

    sim.tick_for(3000)

    _x, _y, h = sim.true_pose()
    # 086-004 retune: mirrors test_rt_rotates_about_90_degrees_and_emits_
    # done_rot's own retune above -- measured -96.3669 deg (-6.37 deg over
    # -90), bit-exact and symmetric with the positive-angle case. Same +-7
    # deg tightened bound (was +-10 deg).
    expected = -math.pi / 2.0
    assert abs(h - expected) < math.radians(7.0), (
        f"expected heading near {expected:.4f} rad (-90 deg), got {h:.4f} rad "
        f"({math.degrees(h):.2f} deg)"
    )

    evts = sim.get_async_evts()
    assert "EVT done RT reason=rot" in evts


def test_rt_stop_clause_time_fires_before_built_in_rotation_stop(sim):
    reply = sim.command("RT 9000 stop=t:100")
    assert reply.strip() == "OK rt rot=9000"

    sim.tick_for(2000)
    evts = sim.get_async_evts()
    assert "EVT done RT reason=time" in evts


def test_rt_range_validation(sim):
    assert sim.command("RT 200000").strip() == "ERR range relAngle"
    assert sim.command("RT -200000").strip() == "ERR range relAngle"


def test_turn_reaches_absolute_heading_from_zero(sim):
    reply = sim.command("TURN 9000")
    assert reply.strip() == "OK turn heading=9000 eps=300"

    sim.tick_for(3000)

    _x, _y, h = sim.true_pose()
    # Measured plant behavior (2026-07-05): the HEADING stop's eps window
    # means the stop can fire slightly BEFORE the exact target is reached,
    # then the SMOOTH ramp-down coasts a bit further -- the two effects
    # partially cancel (see handleTURN's doc comment); +-8 degrees covers
    # the residual.
    expected = math.pi / 2.0
    assert abs(h - expected) < math.radians(8.0), (
        f"expected heading near {expected:.4f} rad (90 deg), got {h:.4f} rad "
        f"({math.degrees(h):.2f} deg)"
    )

    evts = sim.get_async_evts()
    assert "EVT done TURN reason=heading" in evts


def test_turn_reaches_absolute_heading_from_nonzero_start(sim):
    # Establish a nonzero starting heading via RT first (a different verb's
    # own closed loop), then TURN to a fresh absolute target -- proves TURN's
    # shortest-path delta is computed from the CURRENT fused heading, not
    # baked in at some fixed zero start.
    sim.command("RT 9000")
    sim.tick_for(3000)
    sim.get_async_evts()   # drain RT's own completion event

    reply = sim.command("TURN 18000")
    assert reply.strip() == "OK turn heading=18000 eps=300"

    sim.tick_for(3000)
    _x, _y, h = sim.true_pose()
    expected = math.pi   # 180 degrees -- exactly on the heading wrap's own branch cut
    # Measured plant behavior (2026-07-05, pre-086-002): converged to ~169.6
    # deg (~10.4 deg short) -- the HEADING stop fired and the ramp correctly
    # converged (~181.7 deg, matching the target + the usual coast), but the
    # wheels then crossed through zero velocity, and the (then-unfixed)
    # Hal::Motor zero-crossing reset-guard/dwell interaction with
    # Hal::MotorVelocityPid's integrator-freeze deadband (086 issue's own
    # root cause) produced a further ~600-800ms settle with small
    # opposite-sign wheel speeds, backing off part of the rotation before
    # finally settling.
    #
    # 086-002 update: that interaction is now fixed (velocity_pid.cpp's
    # deadband-entry integrator reset) -- this converges to ~182.9 deg now
    # (~2.9 deg over), a tighter residual than before, but on the FAR side
    # of the +-180 deg branch cut: a raw ``h - expected`` blows up to ~2*pi
    # even though the true angular distance is small, so the comparison
    # below wraps the difference into (-pi, pi] first (see this file's
    # ``_wrap_pi`` helper).
    #
    # 086-004 retune: with 086-003 (Planner terminal-decel anticipation for
    # STOP_HEADING, this TURN's own stop kind) landed on top of 086-002, this
    # converges even tighter -- a fully deterministic 1.95 deg wrapped
    # residual (bit-exact across repeated runs, plain sim tick loop). The
    # +-13 deg bound (never actually re-measured before this ticket, per the
    # comment above) is tightened here to +-5 deg -- ~2.6x headroom over the
    # 1.95 deg measured, per architecture-update.md's "Impact on Existing
    # Components" table entry earmarking this exact test for 086-004.
    diff = _wrap_pi(h - expected)
    assert abs(diff) < math.radians(5.0), (
        f"expected heading near {expected:.4f} rad (180 deg), got {h:.4f} rad "
        f"({math.degrees(h):.2f} deg), wrapped diff {math.degrees(diff):.2f} deg"
    )

    evts = sim.get_async_evts()
    assert "EVT done TURN reason=heading" in evts


def test_turn_takes_the_shortest_path_around_the_wrap(sim):
    # From ~170 degrees, TURN -17000 (-170 degrees) is only a 20-degree CCW
    # step across the +-180 wrap -- NOT a 340-degree trip the other way.
    # Ticking for a duration well short of the 340-degree trip's nominal
    # time (but ample for the 20-degree one) empirically proves the
    # shortest-path sign resolution in handleTURN.
    sim.command("RT 17000")
    sim.tick_for(3000)
    sim.get_async_evts()

    reply = sim.command("TURN -17000")
    assert reply.strip() == "OK turn heading=-17000 eps=300"

    sim.tick_for(1000)   # ample for ~20 deg at ~70 deg/s; nowhere near 340 deg
    evts = sim.get_async_evts()
    assert "EVT done TURN reason=heading" in evts

    _x, _y, h = sim.true_pose()
    expected = -17000 * (math.pi / 18000.0)
    # wrap into (-pi, pi] before comparing, matching the firmware's own
    # wrapAngle() convention.
    err = math.atan2(math.sin(h - expected), math.cos(h - expected))
    assert abs(err) < math.radians(10.0), (
        f"expected heading near {expected:.4f} rad (-170 deg), got {h:.4f} rad "
        f"({math.degrees(h):.2f} deg)"
    )


def test_turn_stop_clause_time_fires_before_built_in_heading_stop(sim):
    reply = sim.command("TURN 9000 stop=t:100")
    assert reply.strip() == "OK turn heading=9000 eps=300"

    sim.tick_for(2000)
    evts = sim.get_async_evts()
    assert "EVT done TURN reason=time" in evts


def test_turn_range_validation(sim):
    assert sim.command("TURN 20000").strip() == "ERR range heading"
    assert sim.command("TURN -20000").strip() == "ERR range heading"
    assert sim.command("TURN 9000 eps=5").strip() == "ERR range eps"
    assert sim.command("TURN 9000 eps=2000").strip() == "ERR range eps"


def test_turn_stop_clause_malformed_rejected_with_badarg(sim):
    reply = sim.command("TURN 9000 stop=xyz:1")
    assert reply.strip().startswith("ERR badarg")

    x, y, h = sim.true_pose()
    assert (x, y, h) == (0.0, 0.0, 0.0)


def test_r_bare_runs_open_ended_until_stop(sim):
    reply = sim.command("R 150 500")
    assert reply.strip() == "OK arc speed=150 radius=500"

    sim.tick_for(2000)
    assert "EVT done R" not in sim.get_async_evts()

    x_before_stop, _y, _h = sim.true_pose()
    assert x_before_stop != 0.0, "R should have been driving the robot"

    reply = sim.command("STOP")
    assert reply.strip() == "OK stop"
    # No EVT at all -- STOP is a silent, immediate halt.
    assert sim.get_async_evts() == ""


def test_r_realized_curvature_matches_speed_over_radius(sim):
    reply = sim.command("R 200 400 stop=t:2000")
    assert reply.strip() == "OK arc speed=200 radius=400"

    sim.tick_for(2500)
    evts = sim.get_async_evts()
    assert "EVT done R reason=time" in evts

    _x, _y, h = sim.true_pose()
    # omega = speed/radius = 200/400 = 0.5 rad/s; the stop=t:2000 clause
    # fires ~2 s after the goal starts (ramp-up/down at 20 rad/s^2 is a
    # negligible ~25 ms either side), so total heading change should be
    # close to omega * 2.0 s = 1.0 rad. Measured plant behavior (2026-07-05):
    # 1.198 rad -- the same wheel-zero-crossing settle behavior documented in
    # test_turn_reaches_absolute_heading_from_nonzero_start's comment above
    # applies here too (both wheels ramp through zero when stop=t: fires).
    expected = 0.5 * 2.0
    assert abs(h - expected) < math.radians(14.0), (
        f"expected heading change near {expected:.4f} rad, got {h:.4f} rad"
    )


def test_r_stop_clause_distance_from_docs_example(sim):
    # docs/protocol-v2.md section 10's own documented example.
    reply = sim.command("R 200 500 stop=d:400")
    assert reply.strip() == "OK arc speed=200 radius=500"

    sim.tick_for(3000)
    evts = sim.get_async_evts()
    assert "EVT done R reason=dist" in evts


def test_r_range_validation(sim):
    assert sim.command("R 1500 500").strip() == "ERR range speed"
    assert sim.command("R -1500 500").strip() == "ERR range speed"
    assert sim.command("R 200 20000").strip() == "ERR range radius"
    assert sim.command("R 200 -20000").strip() == "ERR range radius"
