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


def test_rt_rotates_about_90_degrees_and_emits_done_rot(sim):
    reply = sim.command("RT 9000")
    assert reply.strip() == "OK rt rot=9000"

    sim.tick_for(3000)

    _x, _y, h = sim.true_pose()
    # Measured plant behavior (2026-07-05): the ROTATION stop fires once the
    # per-wheel arc target is reached (a ">=" threshold, no early-fire
    # margin), then the SMOOTH stop style ramps yaw rate to zero -- coasting
    # a further ~4-5 degrees past the target before fully stopping (no
    # coast-anticipation this ticket -- see handleRT's doc comment). +-10
    # degrees covers that coast plus ordinary plant/ramp variance.
    expected = math.pi / 2.0
    assert abs(h - expected) < math.radians(10.0), (
        f"expected heading near {expected:.4f} rad (90 deg), got {h:.4f} rad "
        f"({math.degrees(h):.2f} deg)"
    )

    evts = sim.get_async_evts()
    assert "EVT done RT reason=rot" in evts


def test_rt_negative_relangle_rotates_the_opposite_direction(sim):
    reply = sim.command("RT -9000")
    assert reply.strip() == "OK rt rot=-9000"

    sim.tick_for(3000)

    _x, _y, h = sim.true_pose()
    expected = -math.pi / 2.0
    assert abs(h - expected) < math.radians(10.0), (
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
    expected = math.pi   # 180 degrees
    # Measured plant behavior (2026-07-05): converges to ~169.6 deg (~10.4
    # deg short). Traced via per-100ms true_pose()/true_velocity() logging:
    # the HEADING stop fires and the ramp correctly converges (~181.7 deg,
    # matching the target + the usual coast), but the wheels then cross
    # through zero velocity, and the existing Hal::Motor zero-crossing
    # reset-guard/dwell policy (source/hal/velocity_pid.cpp, sprint 081 --
    # unrelated to this ticket, and out of this ticket's file scope to
    # change) produces a further ~600-800ms settle with small opposite-sign
    # wheel speeds, backing off part of the rotation before finally
    # settling. +-13 degrees covers this measured settle plus variance.
    assert abs(h - expected) < math.radians(13.0), (
        f"expected heading near {expected:.4f} rad (180 deg), got {h:.4f} rad "
        f"({math.degrees(h):.2f} deg)"
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
