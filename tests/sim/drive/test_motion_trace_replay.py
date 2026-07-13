"""tests/sim/drive/test_motion_trace_replay.py -- ticket 100-009's OWN
"concrete cross-tier interpretability proof" (AC 3): a `MotionTrace`
(`ReplyEnvelope.body` arm `trace`) captured from a REAL tier-1 sim run
decodes to a valid `TrackRecord` whose `StepInput` replays BIT-EXACT
through tier 0's `Drive::MotionPlan::step()` (`tests/_infra/drive/
replay.py`, ticket 100-006) -- the same compiled `source/drive/` code,
reached two completely different ways (the wafer adapter over the wire vs.
the ctypes ABI directly), fed the identical recorded inputs.

Reconstructing a full `Drive::StepInput` from the wire needs BOTH
`MotionTrace` (t/pose_step/pose_step_theta -- envelope.proto's own doc
comment: "measured/wheelLeft/wheelRight are NOT replayed here -- Telemetry's
own enc=/vel= fields already carry per-wheel state at the TLM period") AND
the SAME-period `Telemetry` frame (enc=/vel=/pose=/twist=) `tickTelemetry()`
pushes right before it (source/telemetry/telemetry_tick.cpp: one `tlm` push
then, iff `StreamControl.trace` is armed, one `trace` push, same pass).

One further wrinkle this module works around explicitly: `Subsystems::
Drivetrain::tick()`'s own `StepInput.measured` (pose+twist) is read from
`bb.bodyState` as committed at the END OF THE PREVIOUS pass (source/runtime/
main_loop.cpp's own one-pass-stale read-at-top-of-pass pattern), while
`StepInput.left/right` (wheel position/velocity) come from `hardware_.
motorState()` FRESH, this same pass -- the SAME motor state THIS pass's own
Telemetry frame reports. So `MotionTrace` sample `k`'s `measured.pose/twist`
must be sourced from `Telemetry` sample `k-1` (not `k`), while its
`left/right` come from `Telemetry` sample `k` itself. Sample `k=0`'s
`measured` is the well-known BOOT-DEFAULT rest pose (world origin, zero
twist) -- nothing has moved the robot before the very first `step()` call,
so this is exact by construction, not an approximation.
"""
from __future__ import annotations

import pathlib
import sys

import pytest

_TESTS_SIM_DRIVE_DIR = pathlib.Path(__file__).resolve().parent  # tests/sim/drive/
_TESTS_SIM_UNIT_DIR = _TESTS_SIM_DRIVE_DIR.parent / "unit"       # tests/sim/unit/
if str(_TESTS_SIM_UNIT_DIR) not in sys.path:
    sys.path.insert(0, str(_TESTS_SIM_UNIT_DIR))

from _binary_envelope import CHANNEL_SERIAL, send, send_no_tick  # noqa: E402
from robot_radio.robot.pb2 import envelope_pb2 as pb_envelope  # noqa: E402
from robot_radio.robot.pb2 import motion_pb2 as pb_motion  # noqa: E402

from drive import (  # noqa: E402
    BodyState, Drive, Goal, Limits, PlanRequest, Pose, ProfileLimits, Status, StepInput,
    TrackRecord, Twist, Verdict, WheelState,
)
from replay import replay_track_records  # noqa: E402

STEP = 24  # [ms] one sim tick -- matches _binary_envelope.read_tlm_now()'s own default
N_SAMPLES = 4

# Mirrors tests/_infra/sim/sim_api.cpp's defaultSimMotionConfig()/
# defaultSimDrivetrainConfig() EXACTLY (NOT tests/sim/drive/_common.py's own
# make_limits(), which uses different, tier-0-test-only numbers) -- this
# test's tier-0 Drive must be configured IDENTICALLY to what the sim's own
# Subsystems::Drivetrain adapter is, or the two plans solve differently and
# there is nothing left to prove.
SIM_TRACKWIDTH = 128.0  # [mm] Hal::PhysicsWorld::kDefaultTrackwidth


def _sim_matching_limits() -> Limits:
    return Limits(
        linear=ProfileLimits(velocity=380.0, accel=800.0, decel=800.0, jerk=5000.0),
        rotational=ProfileLimits(velocity=6.0, accel=20.0, decel=20.0, jerk=100.0),
        v_wheel_max=350.0, trim_v_max=60.0, trim_omega_max=1.0, wheel_step_max=150.0,
        track_k_s=2.0, track_k_theta=6.0, track_k_cross=1.5e-5, min_speed=10.0,
    )


def _decode_pairs(raw: str) -> tuple[list, list]:
    lines = [line for line in raw.splitlines() if line.strip()]
    assert len(lines) == 2 * N_SAMPLES, (
        f"expected {N_SAMPLES} (tlm, trace) pairs ({2 * N_SAMPLES} lines), got {len(lines)}: "
        f"{lines}"
    )
    from _binary_envelope import dearmor
    replies = [dearmor(line) for line in lines]
    tlm_frames, trace_frames = [], []
    for i, reply in enumerate(replies):
        kind = reply.WhichOneof("body")
        if i % 2 == 0:
            assert kind == "tlm", f"line {i}: expected a tlm frame, got {kind}"
            tlm_frames.append(reply.tlm)
        else:
            assert kind == "trace", f"line {i}: expected a trace frame, got {kind}"
            trace_frames.append(reply.trace)
    return tlm_frames, trace_frames


def _step_input_for(k: int, tlm_frames: list, trace_frames: list) -> "StepInput":
    trace = trace_frames[k]
    if k == 0:
        # Boot-default rest state -- nothing has moved the robot before the
        # very first step() call (see this module's own header comment).
        measured = BodyState(pose=Pose(0.0, 0.0, 0.0), twist=Twist(0.0, 0.0, 0.0))
    else:
        prev_tlm = tlm_frames[k - 1]
        measured = BodyState(
            pose=Pose(prev_tlm.pose.x, prev_tlm.pose.y, prev_tlm.pose.h),
            twist=Twist(prev_tlm.twist.v_x, prev_tlm.twist.v_y, prev_tlm.twist.omega),
        )
    this_tlm = tlm_frames[k]
    left = WheelState(position=this_tlm.enc_left, velocity=this_tlm.vel_left,
                       position_valid=this_tlm.has_enc, velocity_valid=this_tlm.has_vel)
    right = WheelState(position=this_tlm.enc_right, velocity=this_tlm.vel_right,
                        position_valid=this_tlm.has_enc, velocity_valid=this_tlm.has_vel)
    return StepInput(t=trace.t, measured=measured, left=left, right=right,
                      pose_step=trace.pose_step, pose_step_theta=trace.pose_step_theta)


def test_motion_trace_replays_bit_exact_at_tier_0(sim, build_drive_lib):
    # Arm trace+binary telemetry BEFORE sending the segment, at rest --
    # establishes StreamControl.trace live with zero motion so far (bb.
    # telemetryHasLastEmit is still false; the FIRST tick_for() pass below
    # emits immediately, on its own gate, once the segment is active).
    arm_reply = send(sim, pb_envelope.CommandEnvelope(
        corr_id=9000, stream=pb_envelope.StreamControl(binary=True, trace=True, period=STEP)))
    assert arm_reply.WhichOneof("body") == "ok"

    goal = pb_motion.MotionSegment(arc_length=500.0, delta_heading=0.3, exit_speed=0.0,
                                    primitive=True)
    seg_reply = send_no_tick(sim, pb_envelope.CommandEnvelope(corr_id=1, segment=goal))
    assert seg_reply.WhichOneof("body") == "ok", seg_reply

    # N_SAMPLES ticks: pass 0 drains segmentIn -> ring_ -> starts the plan
    # (fresh StepState()) AND steps it AND emits its first (tlm, trace)
    # pair, all in the SAME pass -- see this module's own header comment.
    sim.tick_for(N_SAMPLES * STEP, STEP)

    raw = sim.drain_reply_store(CHANNEL_SERIAL)
    tlm_frames, trace_frames = _decode_pairs(raw)

    assert trace_frames[0].t == pytest.approx(0.0, abs=1e-6), (
        "first captured trace sample should be the plan's own t=0 step"
    )
    for i in range(1, N_SAMPLES):
        assert trace_frames[i].t > trace_frames[i - 1].t, "t should strictly increase"

    step_inputs = [_step_input_for(k, tlm_frames, trace_frames) for k in range(N_SAMPLES)]
    records = [TrackRecord(in_=si) for si in step_inputs]

    with Drive(_sim_matching_limits(), SIM_TRACKWIDTH) as drive:
        request = PlanRequest(goal=Goal(arc_length=500.0, delta_heading=0.3, exit_speed=0.0),
                               start=Pose())
        result = drive.plan(request)
        assert result.verdict == Verdict.OK, result.verdict
        with result.plan as plan:
            replayed = replay_track_records(plan, records)

    assert len(replayed) == N_SAMPLES
    # Genuinely BIT-EXACT (verified empirically: every field below diffs by
    # exactly 0.0, not merely "close") -- source/drive/'s reference sample +
    # tracker cascade are pure functions of (plan, StepInput) alone, and
    # both binaries (libfirmware_host, the sim; libdrive_host, tier 0) are
    # built from the IDENTICAL source/drive/*.cpp on the same host/compiler,
    # so replaying the EXACT recorded StepInput sequence reproduces the
    # EXACT recorded floats -- the ticket's own "concrete cross-tier
    # interpretability proof". Plain `==`, never pytest.approx.
    for k, (out, trace) in enumerate(zip(replayed, trace_frames)):
        rec = out.record
        assert rec.ref.x == trace.ref_x, f"sample {k}: ref_x"
        assert rec.ref.y == trace.ref_y, f"sample {k}: ref_y"
        assert rec.ref.theta == trace.ref_theta, f"sample {k}: ref_theta"
        assert rec.ref.v == trace.ref_v, f"sample {k}: ref_v"
        assert rec.ref.omega == trace.ref_omega, f"sample {k}: ref_omega"
        assert rec.e_along == trace.e_along, f"sample {k}: e_along"
        assert rec.e_cross == trace.e_cross, f"sample {k}: e_cross"
        assert rec.e_theta == trace.e_theta, f"sample {k}: e_theta"
        assert rec.v_trim == trace.v_trim, f"sample {k}: v_trim"
        assert rec.omega_trim == trace.omega_trim, f"sample {k}: omega_trim"
        assert rec.v_cmd == trace.v_cmd, f"sample {k}: v_cmd"
        assert rec.omega_cmd == trace.omega_cmd, f"sample {k}: omega_cmd"
        assert rec.wheel_left == trace.wheel_left, f"sample {k}: wheel_left"
        assert rec.wheel_right == trace.wheel_right, f"sample {k}: wheel_right"
        assert rec.trim_saturated == trace.trim_saturated, f"sample {k}: trim_saturated"
        assert int(rec.status) == trace.status, f"sample {k}: status"
        assert rec.status == Status.RUNNING, f"sample {k}: expected RUNNING mid-motion, got {rec.status}"


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
