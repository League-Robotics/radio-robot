"""Off-hardware acceptance proof for ticket 094-005's own end-to-end
acceptance criterion: a `Motion::Segment` posted directly to `bb.segmentIn`
executes and settles through the FULL composition root (`SimHandle` ->
`Rt::MainLoop::tick()` -> `Subsystems::Drivetrain::tick()` ->
`Motion::SegmentExecutor`), proving the loop reorder (`hardware_.
serviceBus()` -> `drivetrain_.tick()` -> commit, main_loop.cpp) plus the
`bb.segmentIn` blackboard wiring work together correctly -- ahead of
094-006's wire `MOVE` verb, which does not exist yet.

Posts via `sim.post_segment()` (`tests/_infra/sim/firmware.py`, wrapping
this ticket's test-only `sim_post_segment()` C ABI entry point,
`tests/_infra/sim/sim_api.cpp`) -- a direct `bb.segmentIn.post(...)`
producer, exactly what 094-005's own acceptance criteria call for when
094-006 has not yet landed.

Tolerances are intentionally wide (not pinned to the commanded distance):
the REAL simulated plant (PID + inertia) coasts some distance past the
encoder-based STOP_DISTANCE threshold before the presolved decel-to-zero
fully arrests it -- an accepted, pre-existing plant/PID settling
characteristic (see tests/sim/unit/drivetrain_harness.cpp's own scenarios
for the same tolerance rationale, empirically measured against the same
plant). This test's own job is to prove the LOOP-LEVEL wiring, not to
re-pin the executor's/plant's numeric accuracy (094-001's/094-004's own
scope).
"""
from __future__ import annotations

import pytest


def test_segment_posted_directly_to_blackboard_executes_and_settles(sim):
    """A straight (no-pivot) Motion::Segment posted via sim.post_segment()
    drives both wheels forward and settles near the commanded distance,
    proving Rt::MainLoop's reordered tick() (serviceBus -> drivetrain.tick
    -> commit) and bb.segmentIn actually wire a segment through to the
    real (simulated) hardware end to end."""
    accepted = sim.post_segment(distance=120.0, direction=0.0, final_heading=0.0,
                                speed_max=250.0, accel_max=800.0)
    assert accepted, "sim.post_segment() was accepted by bb.segmentIn (not already full)"

    # Give the loop a few passes to drain segmentIn into Drivetrain's ring
    # and start executing, then let it run to a generous settle window.
    sim.tick_for(6000)

    enc_l, enc_r = sim.enc()
    vel_l, vel_r = sim.vel()

    # Both wheels moved forward roughly the commanded distance (wide
    # tolerance -- see this file's own docstring).
    assert enc_l == pytest.approx(120.0, abs=60.0)
    assert enc_r == pytest.approx(120.0, abs=60.0)
    assert enc_l > 20.0, "the segment actually drove the left wheel forward, not a no-op"
    assert enc_r > 20.0, "the segment actually drove the right wheel forward, not a no-op"

    # Settled back to ~0 -- the segment converged and popped, nothing left
    # re-driving the wheels.
    assert vel_l == pytest.approx(0.0, abs=10.0)
    assert vel_r == pytest.approx(0.0, abs=10.0)


def test_segment_ring_full_post_segment_reports_not_accepted(sim):
    """Posting more segments than bb.segmentIn's 8-slot capacity in a
    single burst (no intervening sim_tick() to drain any of them into
    Drivetrain's own ring) surfaces WorkQueue<T,8>::post()'s own
    true/false "accepted" contract through sim_post_segment() -- proving
    the test-only entry point genuinely reflects the blackboard queue's
    real capacity, not a stub that always reports success."""
    results = [
        sim.post_segment(distance=10.0, direction=0.0, final_heading=0.0)
        for _ in range(9)
    ]
    assert results[:8] == [True] * 8, "the first 8 posts (segmentIn's own capacity) are all accepted"
    assert results[8] is False, "the 9th post is rejected -- segmentIn is at its 8-slot cap"
