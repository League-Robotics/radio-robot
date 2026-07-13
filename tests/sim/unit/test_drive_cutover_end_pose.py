"""tests/sim/unit/test_drive_cutover_end_pose.py -- ticket 100-007 (THE
CUTOVER) tier-1 acceptance: end-pose accuracy for the FOUR shapes the
ticket's own verification section names -- straight, arc, pivot, and a
chained-3 (decomposed legacy MOVE) -- exercised through the REAL
Rt::MainLoop + the new source/drive/ stack (via `sim.command_on()`'s binary
`segment` arm, the same wire path a real client uses), checked against
`sim.true_pose()` (Hal::PhysicsWorld's own ground truth, never the fused/
encoder estimate -- this test isolates the adapter+tracker+plant loop from
any PoseEstimator/EKF concern).

Each scenario's ideal end pose is computed independently, via plain
constant-curvature-arc geometry (never re-derived from source/drive/'s own
implementation -- the "transcribe/verify, don't trust by inspection"
discipline every other translator test in this tree already follows):
for one arc primitive (arcLength, deltaHeading) starting at world pose
(0, 0, 0):
    kappa = deltaHeading / arcLength   (undefined/unused for a pivot)
    pivot   (arcLength == 0):     end = (0, 0, deltaHeading)
    straight (deltaHeading == 0): end = (arcLength, 0, 0)
    arc     (both nonzero):       R = arcLength / deltaHeading
                                   end = (R*sin(deltaHeading),
                                          R*(1 - cos(deltaHeading)),
                                          deltaHeading)
A CHAIN of primitives composes by re-anchoring each phase's own local
(dx, dy, dh) at the PREVIOUS phase's ending world heading -- the same
composition test_tour_closure.py's own `_ideal_tour_poses()` already
established and this file's own `_compose_ideal_chain()` mirrors.

Tolerances are generous (tens of mm / a few degrees) -- this test proves
the adapter+tracker+plant loop converges to the RIGHT PLACE, not a tight
bench-accuracy bound (that is M11's job, tests/bench/arc_sweep.py, against
the real plant).
"""
from __future__ import annotations

import math

import pytest

from _binary_envelope import send_segment
from robot_radio.robot import legacy_translate


def _ideal_end_pose(arc_length: float, delta_heading: float,
                     start: tuple[float, float, float] = (0.0, 0.0, 0.0),
                     ) -> tuple[float, float, float]:
    """Ideal world-frame end pose after ONE constant-curvature primitive,
    composed onto `start` -- see this module's own header comment for the
    geometry."""
    x0, y0, h0 = start
    if abs(delta_heading) < 1e-9:
        # Straight (or a degenerate zero-motion primitive): no rotation.
        dx, dy = arc_length, 0.0
    elif abs(arc_length) < 1e-9:
        # Pivot: no translation.
        dx, dy = 0.0, 0.0
    else:
        radius = arc_length / delta_heading
        dx = radius * math.sin(delta_heading)
        dy = radius * (1.0 - math.cos(delta_heading))
    # Rotate the LOCAL (dx, dy) into the world frame at the start heading,
    # then translate.
    x = x0 + dx * math.cos(h0) - dy * math.sin(h0)
    y = y0 + dx * math.sin(h0) + dy * math.cos(h0)
    h = h0 + delta_heading
    return x, y, h


def _compose_ideal_chain(segs) -> tuple[float, float, float]:
    """Fold _ideal_end_pose() over a sequence of (arc_length, delta_heading)
    primitives, each re-anchored at the previous phase's own ending pose."""
    pose = (0.0, 0.0, 0.0)
    for seg in segs:
        pose = _ideal_end_pose(seg.arc_length, seg.delta_heading, start=pose)
    return pose


def _send_and_settle(sim, segs, seconds: float = 10.0, step: int = 24) -> None:
    """Send every primitive in `segs`, in order (each ADMITTED before the
    next is sent, mirroring legacy_verbs's own multi-envelope discipline),
    then tick until well past natural completion."""
    for seg in segs:
        reply = send_segment(sim, seg)
        assert reply.WhichOneof("body") == "ok", reply
    for _ in range(int(seconds * 1000 / step)):
        sim.tick_for(step)


# ---------------------------------------------------------------------------
# Straight / arc / pivot -- single primitive each.
# ---------------------------------------------------------------------------


def test_straight_segment_end_pose_matches_true_pose(sim):
    seg = legacy_translate.segment_for_seg(arc_length=400.0)
    _send_and_settle(sim, [seg])

    ideal_x, ideal_y, ideal_h = _ideal_end_pose(400.0, 0.0)
    x, y, h = sim.true_pose()
    assert x == pytest.approx(ideal_x, abs=20.0)
    assert y == pytest.approx(ideal_y, abs=20.0)
    assert h == pytest.approx(ideal_h, abs=math.radians(3.0))


def test_pivot_segment_end_pose_matches_true_pose(sim):
    seg = legacy_translate.segment_for_seg(delta_heading=math.radians(90.0))
    _send_and_settle(sim, [seg])

    ideal_x, ideal_y, ideal_h = _ideal_end_pose(0.0, math.radians(90.0))
    x, y, h = sim.true_pose()
    assert x == pytest.approx(ideal_x, abs=20.0)
    assert y == pytest.approx(ideal_y, abs=20.0)
    assert h == pytest.approx(ideal_h, abs=math.radians(3.0))


def test_arc_segment_end_pose_matches_true_pose(sim):
    """A genuine constant-curvature arc (both arc_length and delta_heading
    nonzero) -- kappa = delta_heading/arc_length != 0, exercising the
    tracker's own curvature-preserving IK/saturate cascade, not just the
    straight/pivot degenerate cases above."""
    arc_length = 500.0
    delta_heading = math.radians(60.0)
    seg = legacy_translate.segment_for_seg(arc_length=arc_length, delta_heading=delta_heading)
    _send_and_settle(sim, [seg])

    ideal_x, ideal_y, ideal_h = _ideal_end_pose(arc_length, delta_heading)
    x, y, h = sim.true_pose()
    assert x == pytest.approx(ideal_x, abs=30.0)
    assert y == pytest.approx(ideal_y, abs=30.0)
    assert h == pytest.approx(ideal_h, abs=math.radians(4.0))


# ---------------------------------------------------------------------------
# Chained-3 -- a decomposed legacy MOVE (primitives_for_move()'s own <=3-
# phase pivot/straight/pivot decomposition), all three phases nonzero.
# ---------------------------------------------------------------------------


def test_chained_move_decomposition_end_pose_matches_true_pose(sim):
    """`MOVE 300 <90deg> <-90deg>` -- direction != 0 (leading pivot fires),
    distance != 0 (straight fires), final_heading != direction (trailing
    pivot fires, delta = -90 - 90 = -180deg) -- all THREE
    primitives_for_move() phases present, sent in order and admitted
    against each other's own predicted ChainTail (bb.chainTail, advanced
    synchronously by BinaryChannel's wire admission -- commands/
    binary_channel.cpp's admitSegment())."""
    segs = legacy_translate.primitives_for_move(300.0, 9000.0, -9000.0)
    assert len(segs) == 3, "leading pivot + straight + trailing pivot, all nonzero"
    _send_and_settle(sim, segs, seconds=14.0)

    ideal_x, ideal_y, ideal_h = _compose_ideal_chain(segs)
    x, y, h = sim.true_pose()
    assert x == pytest.approx(ideal_x, abs=35.0)
    assert y == pytest.approx(ideal_y, abs=35.0)
    # Wrap the heading comparison to (-pi, pi] before comparing -- a chain
    # ending near +/-180deg can legitimately read either sign.
    dh = (h - ideal_h + math.pi) % (2 * math.pi) - math.pi
    assert dh == pytest.approx(0.0, abs=math.radians(5.0))


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
