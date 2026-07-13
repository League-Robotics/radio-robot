"""tests/sim/unit/test_plan_dump.py -- ticket 100-009 AC: `PlanDumpRequest`
(`CommandEnvelope.cmd` arm 18) against a MULTI-SEGMENT ring returns one
correlated `PlanRecord` (`ReplyEnvelope.body` arm `plan`) per dumpable ring
entry, sharing `corr_id` -- BinaryChannel::handlePlanDump()'s real
implementation, replacing the `ERR_UNIMPLEMENTED` stub ticket 100-001 left
in place.

To get MULTIPLE entries queued in the wafer adapter's own `ring_` at once
(rather than the single active plan a normal one-`segment`-then-tick
sequence produces), every segment in a batch is admitted via
``send_no_tick()`` (wire admission runs synchronously at ROUTE time,
independent of any tick -- see binary_channel.cpp's own admitSegment() doc
comment) BEFORE a single ``tick_for()`` call: that ONE tick drains
`bb.segmentIn` IN FULL into `ring_` (`Subsystems::Drivetrain::tick()`'s own
step 2), then pops exactly the FIRST entry into the active `plan_`
(`startNextPlan()`), leaving every other admitted segment still queued in
`ring_` -- exactly the "multi-segment ring" this ticket's AC names.
"""
from __future__ import annotations

from _binary_envelope import send_multi, send_no_tick
from robot_radio.robot.pb2 import envelope_pb2 as pb_envelope
from robot_radio.robot.pb2 import motion_pb2 as pb_motion


def _primitive_arc(arc_length: float, delta_heading: float = 0.0,
                    exit_speed: float = 0.0) -> "pb_motion.MotionSegment":
    return pb_motion.MotionSegment(arc_length=arc_length, delta_heading=delta_heading,
                                    exit_speed=exit_speed, primitive=True)


def _queue_segments(sim, segs) -> None:
    """Admit every segment in `segs`, in order, WITHOUT ticking in between
    (see this module's own header comment) -- each is a fresh wire
    admission (ACK expected), all landing in bb.segmentIn before anything
    drains it."""
    for seg in segs:
        reply = send_no_tick(sim, pb_envelope.CommandEnvelope(corr_id=1, segment=seg))
        assert reply.WhichOneof("body") == "ok", reply


def test_plan_dump_multi_segment_ring_one_record_per_entry(sim):
    # Three small, non-overlapping straight-ish arcs -- each admits cleanly
    # in a chain (entry/exit speed both 0.0 throughout, so admit()'s own
    # no-sign-reversal/inner-wheel-floor checks never trigger).
    segs = [_primitive_arc(200.0, 0.1), _primitive_arc(150.0, -0.05), _primitive_arc(100.0, 0.0)]
    _queue_segments(sim, segs)

    # ONE tick: drains bb.segmentIn (3 entries) into ring_, then pops the
    # FIRST into the active plan_ -- ring_ now holds the other 2.
    sim.tick_for(24)

    replies = send_multi(sim, pb_envelope.CommandEnvelope(corr_id=42,
                                                            plan_dump=pb_envelope.PlanDumpRequest()))

    assert len(replies) == len(segs), (
        f"expected one PlanRecord per ring entry ({len(segs)}), got {len(replies)}"
    )
    for i, reply in enumerate(replies):
        assert reply.corr_id == 42, f"record {i}: corr_id not echoed"
        assert reply.WhichOneof("body") == "plan", f"record {i}: not a PlanRecord reply: {reply}"
        record = reply.plan
        assert record.duration > 0.0, f"record {i}: non-positive duration"
        # v_eff is the folded body/yaw-rate ceiling -- always positive for a
        # nonzero primitive (arc_length or delta_heading != 0).
        assert record.v_eff > 0.0, f"record {i}: non-positive v_eff"
        assert record.replan_count == 0, (
            f"record {i}: replan_count should be 0 -- none of these plans have run yet"
        )

    # The active (first) entry's own record should roughly match what a
    # normal single-segment admission produces: anchor at the world origin
    # (fresh boot), goal displaced along the commanded arc's own geometry.
    active = replies[0].plan
    assert abs(active.anchor.x) < 1e-3
    assert abs(active.anchor.y) < 1e-3
    assert abs(active.anchor.h) < 1e-3


def test_plan_dump_empty_ring_replies_single_ack(sim):
    """No active plan, nothing queued -- a PlanDumpRequest still gets
    exactly one reply back (an Ack{q:0}), never zero replies (a pipelined
    client correlating on corr_id must always see something)."""
    replies = send_multi(sim, pb_envelope.CommandEnvelope(corr_id=7,
                                                            plan_dump=pb_envelope.PlanDumpRequest()))
    assert len(replies) == 1
    assert replies[0].corr_id == 7
    assert replies[0].WhichOneof("body") == "ok"
    assert replies[0].ok.q == 0


def test_plan_dump_single_active_plan_no_queue(sim):
    """The common case -- one segment sent and ticked to become the active
    plan (never queued) -- dumps exactly one PlanRecord."""
    reply = send_no_tick(sim, pb_envelope.CommandEnvelope(corr_id=1, segment=_primitive_arc(300.0)))
    assert reply.WhichOneof("body") == "ok"
    sim.tick_for(24)

    replies = send_multi(sim, pb_envelope.CommandEnvelope(corr_id=99,
                                                            plan_dump=pb_envelope.PlanDumpRequest()))
    assert len(replies) == 1
    assert replies[0].WhichOneof("body") == "plan"
    assert replies[0].plan.exit_speed == 0.0
