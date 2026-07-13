"""Tier-0 replay harness test (ticket 100-006 AC): given a recorded
TrackRecord.in_ sequence, replay.replay_track_records() reproduces the
recorded output bit-exact.

No real bench/field TrackRecord artifact exists yet (those come from later
tickets: 100-007's cutover, 100-010's fault-matrix, 100-011/012's bench/field
runs) -- this test generates its OWN "recording" by running one closed-loop
simulation (plant.py's Plant, same as test_drive_closed_loop.py) and
collecting the sequence of TrackRecords step() actually emitted, then proves
replay.replay_track_records() -- fed ONLY the recorded `.in_` fields, through
a FRESH StepState() on a FRESH Plan handle for the SAME solved plan --
reproduces every recorded StepOutput bit-exact. This is a genuine proof of
the replay mechanism's correctness/self-consistency: it does not (and does
not need to) depend on any tier-1+ artifact existing yet.
"""
from __future__ import annotations

import pytest
from _common import make_limits, TRACKWIDTH
from drive import Drive, Goal, PlanRequest, Pose, Status, StepInput, StepState, Verdict
from plant import Plant, PlantConfig
from replay import replay_track_records


def _record_closed_loop_run(plan, plant: Plant, *, dt: float, max_seconds: float):
    """Drive `plan` against `plant` in closed loop, collecting every tick's
    full StepOutput (whose .record IS the TrackRecord -- the replay
    payload). Stops at the first terminal Status (inclusive)."""
    state = StepState()
    t = 0.0
    steps = int(max_seconds / dt)
    outputs = []
    for _ in range(steps):
        body, left, right = plant.measured()
        step_input = StepInput(t=t, measured=body, left=left, right=right)
        out, state = plan.step(step_input, state)
        outputs.append(out)
        if out.status in (Status.DONE_STOP, Status.DONE_HANDOFF, Status.ABORT_TIMEOUT,
                          Status.ABORT_REPLAN_LIMIT):
            break
        plant.step(out.command.left, out.command.right, dt)
        t += dt
    return outputs


def test_replay_reproduces_recorded_output_bit_exact(build_drive_lib):
    limits = make_limits()
    with Drive(limits, TRACKWIDTH) as drive:
        request = PlanRequest(goal=Goal(arc_length=500.0, delta_heading=0.15, exit_speed=0.0),
                               start=Pose())

        # First plan handle: generate the "recording" against a real plant.
        recorded_result = drive.plan(request)
        assert recorded_result.verdict == Verdict.OK, recorded_result.verdict
        with recorded_result.plan as recording_plan:
            plant = Plant(PlantConfig(motor_lag=130.0, stiction=0.0, enc_staleness=0.0,
                                       quantization=0.0, slip=0.0, trackwidth=TRACKWIDTH))
            recorded_outputs = _record_closed_loop_run(recording_plan, plant, dt=0.02,
                                                         max_seconds=10.0)

        assert len(recorded_outputs) > 50, "recording too short to be a meaningful replay proof"
        assert recorded_outputs[-1].status in (Status.DONE_STOP, Status.DONE_HANDOFF), (
            "the recorded run should reach a clean completion, not time out mid-recording"
        )

        recorded_records = [out.record for out in recorded_outputs]

        # Second, INDEPENDENT plan handle for the SAME PlanRequest (a fresh
        # Drive::MotionPlan, re-solved from scratch) -- the replay must
        # reproduce the recording using only the .in_ sequence, never the
        # original plan handle or the original Plant.
        replay_result = drive.plan(request)
        assert replay_result.verdict == Verdict.OK, replay_result.verdict
        with replay_result.plan as replay_plan:
            replayed_outputs = replay_track_records(replay_plan, recorded_records)

        assert len(replayed_outputs) == len(recorded_outputs)
        for i, (recorded, replayed) in enumerate(zip(recorded_outputs, replayed_outputs)):
            assert replayed.status == recorded.status, f"tick {i}: status mismatch"
            assert replayed.command.left == recorded.command.left, f"tick {i}: command.left mismatch"
            assert replayed.command.right == recorded.command.right, f"tick {i}: command.right mismatch"
            assert replayed.record.e_along == recorded.record.e_along, f"tick {i}: record.e_along mismatch"
            assert replayed.record.e_cross == recorded.record.e_cross, f"tick {i}: record.e_cross mismatch"
            assert replayed.record.wheel_left == recorded.record.wheel_left, f"tick {i}: record.wheel_left mismatch"
            assert replayed.record.wheel_right == recorded.record.wheel_right, f"tick {i}: record.wheel_right mismatch"
            assert replayed == recorded, f"tick {i}: full StepOutput not bit-exact"


if __name__ == "__main__":
    import sys

    sys.exit(pytest.main([__file__, "-v"]))
