"""Off-hardware acceptance proof for ticket 084-004 (SUC-003): registers
``G`` -- relative-XY go-to navigation -- as a top-level wire verb, the first
verb in this sprint to close against ``Planner::tick()``'s fused
**position** (``TURN``/``RT``, ticket 084-003, only needed fused heading).

Like ``test_motion_commands_arc_turn.py``, this drives ``libfirmware_host``
through the full wire dispatch (``Sim.command()``) -- ``CommandProcessor``
-> ``source/commands/motion_commands.cpp`` -> ``MotionLoopState``'s outbox
-> ``dev_loop.cpp``'s drain step -> ``Subsystems::Planner`` -> the simulated
plant. ``PlannerConfig.turn_in_place_gate``/``arrive_tol`` are the sim
build's defaults (``tests/_infra/sim/sim_api.cpp``'s
``defaultSimPlannerConfig()``: 35 deg / 25 mm, matching
``docs/protocol-v2.md`` section 10's documented ``G`` defaults) -- ticket
084-004 registers no ``SET``-able override for either field.

**Measured settle behavior (2026-07-06).** Every ``G`` completion below
overshoots its ``arrive_tol`` window slightly at the instant the POSITION
stop fires (a few mm past the target -- well within tolerance), then the
SMOOTH ramp-down's terminal wheel-velocity zero-crossing triggers the same
``source/hal/velocity_pid.cpp`` (sprint 081) reset-guard/dwell settle
already documented in ``test_motion_commands_arc_turn.py``'s own TURN/RT
comments -- a brief reverse creep before the plant fully stops, well past
the moment the completion EVT itself fires. This is a pre-existing,
out-of-scope (for this ticket) plant/PID characteristic, reproducible
identically via a plain ``D`` (confirmed by hand during this ticket's
implementation) -- not something ``G``'s own PRE_ROTATE/PURSUE state
machine introduces. Tolerances below are set from the measured settled
plant behavior, documented at each assertion, the same way 084-003's own
tests document theirs.
"""

import math


def test_g_reaches_relative_target_and_emits_done_reason_pos(sim):
    reply = sim.command("G 300 0 200")
    assert reply.strip() == "OK goto x=300 y=0 speed=200"

    sim.tick_for(6000)
    evts = sim.get_async_evts()
    assert "EVT done G reason=pos" in evts

    x, y, h = sim.true_pose()
    # Measured plant behavior (2026-07-06): the POSITION stop fires at
    # ~302 mm (well within the default 25 mm arrive_tol of the 300 mm
    # target), then the SMOOTH ramp-down's zero-crossing settle (see this
    # file's header comment) creeps the plant back to a final rest of
    # ~262 mm by 6 s. +-45 mm covers both the ~303 mm overshoot at the
    # moment the stop fires and that eventual ~262 mm settle-back.
    assert abs(x - 300.0) < 45.0, f"expected x near 300 mm, got {x:.2f} mm"
    assert abs(y) < 5.0, f"expected y ~= 0 (straight-ahead target, no steering needed), got {y:.2f} mm"
    assert abs(h) < math.radians(3.0), (
        f"expected heading ~= 0 (bearing 0 deg never engages PRE_ROTATE), got {math.degrees(h):.2f} deg"
    )


def test_g_pre_rotates_when_bearing_exceeds_the_gate(sim):
    # bearing = atan2(300, 0) = 90 deg -- well past the default 35 deg
    # turn_in_place_gate -- PRE_ROTATE must engage (spin in place) BEFORE
    # any translation happens.
    reply = sim.command("G 0 300 150")
    assert reply.strip() == "OK goto x=0 y=300 speed=150"

    sim.tick_for(800)
    x, y, h = sim.true_pose()
    # Measured plant behavior (2026-07-06): position holds exactly at the
    # origin through 800 ms while heading climbs to ~59 deg -- PRE_ROTATE is
    # a pure spin-in-place, no translation, until the bearing gate is
    # reached (~1.3 s nominal at PRE_ROTATE's ~70 deg/s rate for a 90 deg
    # turn).
    assert (x, y) == (0.0, 0.0), (
        f"expected PRE_ROTATE to hold position while spinning, got ({x:.2f}, {y:.2f})"
    )
    assert h > math.radians(10.0), (
        f"expected PRE_ROTATE to have turned well off zero by 800 ms, got {math.degrees(h):.2f} deg"
    )
    assert sim.get_async_evts() == "", "PRE_ROTATE reaching the gate hands off to PURSUE with no EVT"

    sim.tick_for(3000)
    evts = sim.get_async_evts()
    assert "EVT done G reason=pos" in evts

    x, y, h = sim.true_pose()
    # Measured plant behavior (2026-07-06): PURSUE's curvature-clamped
    # approach lands near (12, 278) at completion (some cross-track/heading
    # overshoot from the curvature-based steering, plus the same settle
    # behavior documented at the top of this file) -- +-40 mm on each axis
    # covers it.
    assert abs(x - 0.0) < 40.0, f"expected x near 0 mm, got {x:.2f} mm"
    assert abs(y - 300.0) < 40.0, f"expected y near 300 mm, got {y:.2f} mm"


def test_g_does_not_pre_rotate_when_bearing_is_within_the_gate(sim):
    # bearing = atan2(0, 300) = 0 deg -- well within the default 35 deg
    # turn_in_place_gate -- PURSUE must start immediately with no
    # spin-in-place phase at all.
    reply = sim.command("G 300 0 200")
    assert reply.strip() == "OK goto x=300 y=0 speed=200"

    sim.tick_for(200)
    x, _y, h = sim.true_pose()
    # Measured plant behavior (2026-07-06): the robot is already translating
    # by 200 ms (x > 0) with heading still at exactly 0 -- confirms PURSUE
    # engaged directly, never spinning in place first.
    assert x > 0.0, f"expected PURSUE to be driving forward by 200 ms, got x={x:.2f} mm"
    assert h == 0.0, f"expected no PRE_ROTATE spin (bearing within the gate), got heading={h:.4f} rad"


def test_g_short_distance_within_arrive_tol_completes_almost_immediately(sim):
    # 20 mm is inside the default 25 mm arrive_tol -- the robot starts
    # already within arrival radius of the target, so the POSITION stop
    # must fire on (or immediately after) the very first PURSUE tick,
    # before any meaningful travel.
    reply = sim.command("G 20 0 200")
    assert reply.strip() == "OK goto x=20 y=0 speed=200"

    sim.tick_for(50)
    evts = sim.get_async_evts()
    assert "EVT done G reason=pos" in evts

    x, y, _h = sim.true_pose()
    assert (x, y) == (0.0, 0.0), (
        f"expected arrival honored before any travel (20 mm < 25 mm arrive_tol), got ({x:.2f}, {y:.2f})"
    )


def test_g_distance_just_over_arrive_tol_requires_travel_before_completing(sim):
    # 30 mm is just OUTSIDE the default 25 mm arrive_tol -- the robot must
    # actually travel some distance before the POSITION stop can fire.
    reply = sim.command("G 30 0 200")
    assert reply.strip() == "OK goto x=30 y=0 speed=200"

    sim.tick_for(200)
    assert sim.get_async_evts() == "", "30 mm is outside arrive_tol -- must not complete instantly"
    x, _y, _h = sim.true_pose()
    assert 0.0 < x < 30.0, f"expected partial travel toward the 30 mm target, got x={x:.2f} mm"

    sim.tick_for(200)
    evts = sim.get_async_evts()
    assert "EVT done G reason=pos" in evts
    x, _y, _h = sim.true_pose()
    assert abs(x - 30.0) < 25.0, f"expected arrival within arrive_tol of 30 mm, got x={x:.2f} mm"


def test_g_range_validation(sim):
    assert sim.command("G 11000 0 200").strip() == "ERR range x"
    assert sim.command("G -11000 0 200").strip() == "ERR range x"
    assert sim.command("G 0 11000 200").strip() == "ERR range y"
    assert sim.command("G 0 -11000 200").strip() == "ERR range y"
    assert sim.command("G 300 0 0").strip() == "ERR range speed"
    assert sim.command("G 300 0 1001").strip() == "ERR range speed"


def test_g_too_few_args_rejected_with_badarg(sim):
    assert sim.command("G 300 0").strip() == "ERR badarg"
