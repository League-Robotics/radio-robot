"""tests/sim/unit/test_pose_fix_reset_zero.py -- 099-004 (SUC-002/SUC-006):
`CommandEnvelope.cmd.pose_fix`'s binary `reset`/`zero_encoders` arm,
exercised end to end via `sim_command_on()`:
`BinaryChannel::handlePose()` -> `bb.poseResetIn` ->
`Subsystems::PoseEstimator::tick()` -> `bb.fusedPose` (+ `bb.otosSetPoseIn`
-> `Rt::MainLoop`'s odometer drain) -> read back over the binary `stream`
arm's periodic TLM `pose=` field (`_binary_envelope.read_tlm_now()`).

`PoseFix` (`protos/drivetrain.proto`) retypes the formerly-declared-only
`CommandEnvelope.cmd.pose` arm (was `SetPose`) to `pose_fix` -- see
`architecture-update.md` D5-D8, Decision 1. This ticket implements ONLY the
`reset`/`zero_encoders` branches (both reuse `PoseEstimator`'s existing,
already-tested `setPose()`/`resetEncoderBaseline()` dispatch through the
UNCHANGED `bb.poseResetIn` queue). The genuine delayed-fix branch (neither
flag set) still replies `Error{ERR_UNIMPLEMENTED, field=7}` -- ticket
099-008 makes it live, reusing this same wire arm/message.

Covers ticket 099-004's own sim acceptance criterion:
  - `reset=true` re-anchors `pose=` to the commanded x/y/h.
  - `zero_encoders=true` alone does not move `pose=` or `otos=` -- the two
    are independent pipelines (`PoseEstimator`'s own encoder-delta baseline
    vs. the odometer's own ground-truth sample). `otos=`/`otosconn=` are
    live and present in sim as of ticket 099-002 (`bb.otosPresent` is now
    seeded `true` at boot -- `Hal::SimOdometer` has no physical chip to
    ever fail to detect, so `Hal::Odometer::present()`'s convenience
    default applies unmodified); before 099-002, `bb.otosPresent` was never
    seeded at all and `otos=` stayed permanently absent from TLM.
  - Both flags set in one message: both branches run (`pose=` re-anchors to
    the commanded x/y/h; the encoder-delta baseline also resyncs, verified
    indirectly by confirming the request is accepted with a single `OK`).
  - A stale/garbage neither-flag request (a genuine delayed camera fix,
    099-008's job) replies `Error{ERR_UNIMPLEMENTED, field=7}` -- unchanged
    behavior for the not-yet-live variant.
"""
from __future__ import annotations

import pathlib
import sys

import pytest

# tests/sim/unit/test_pose_fix_reset_zero.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_HOST_DIR = _REPO_ROOT / "host"
if str(_HOST_DIR) not in sys.path:
    sys.path.insert(0, str(_HOST_DIR))

from robot_radio.robot.pb2 import drivetrain_pb2 as pb_drivetrain  # noqa: E402
from robot_radio.robot.pb2 import envelope_pb2 as pb_envelope  # noqa: E402

# tests/sim/conftest.py's build_lib fixture already inserts this path; guard
# against a double-insert if this module is imported before that fixture runs
# (same guard _binary_envelope.py/test_binary_channel.py both use).
_SIM_INFRA_DIR = _REPO_ROOT / "tests" / "_infra" / "sim"
if str(_SIM_INFRA_DIR) not in sys.path:
    sys.path.insert(0, str(_SIM_INFRA_DIR))

from _binary_envelope import read_tlm_now, send  # noqa: E402


def _pose_fix(sim, corr_id, **kwargs):
    return send(sim, pb_envelope.CommandEnvelope(
        corr_id=corr_id, pose_fix=pb_drivetrain.PoseFix(**kwargs)))


def test_pose_fix_reset_reanchors_pose(sim):
    """reset=true re-anchors pose= to the commanded x/y/h within one tick --
    the SI-equivalent hard re-anchor, live this ticket."""
    baseline = read_tlm_now(sim)
    assert baseline.pose.x == pytest.approx(0.0)
    assert baseline.pose.y == pytest.approx(0.0)
    assert baseline.pose.h == pytest.approx(0.0)

    reply = _pose_fix(sim, 1, reset=True, x=842.5, y=-317.0, h=0.75)
    assert reply.WhichOneof("body") == "ok"
    assert reply.corr_id == 1

    frame = read_tlm_now(sim)
    assert frame.pose.x == pytest.approx(842.5, abs=0.5)
    assert frame.pose.y == pytest.approx(-317.0, abs=0.5)
    assert frame.pose.h == pytest.approx(0.75, abs=1e-3)


def test_pose_fix_zero_encoders_does_not_move_pose_or_otos(sim):
    """zero_encoders=true (ZERO-equivalent) resyncs the encoder-delta
    baseline only -- pose= (the believed/fused pose) must NOT jump, and
    otos= (the odometer's own independently-sampled ground-truth pose,
    committed every pass by Rt::MainLoop::commit() since ticket 099-002)
    must be completely unaffected by this command either way -- it is a
    different pipeline, not merely "absent" (099-002 seeds bb.otosPresent
    true at boot in sim -- Hal::SimOdometer has no physical chip to ever
    fail to detect -- so otos=/otosconn= are live in TLM from this ticket
    onward, unlike before it, when bb.otosPresent was never seeded at all
    and these fields stayed permanently absent)."""
    # Re-anchor to a known, nonzero pose first so "does not move" is a real
    # proof, not a trivial zero-stays-zero check.
    reply = _pose_fix(sim, 10, reset=True, x=150.0, y=60.0, h=0.2)
    assert reply.WhichOneof("body") == "ok"

    before = read_tlm_now(sim)
    assert before.pose.x == pytest.approx(150.0, abs=0.5)
    assert before.pose.y == pytest.approx(60.0, abs=0.5)
    assert before.pose.h == pytest.approx(0.2, abs=1e-3)
    assert before.has_otos == True  # noqa: E712 -- explicit tri-state check
    assert before.otos_connected == True  # noqa: E712

    reply = _pose_fix(sim, 11, zero_encoders=True)
    assert reply.WhichOneof("body") == "ok"
    assert reply.corr_id == 11

    after = read_tlm_now(sim)
    assert after.pose.x == pytest.approx(before.pose.x, abs=1e-3)
    assert after.pose.y == pytest.approx(before.pose.y, abs=1e-3)
    assert after.pose.h == pytest.approx(before.pose.h, abs=1e-3)
    # otos= tracks the SimOdometer's own ground-truth ODOMETER sample -- a
    # pipeline entirely independent of PoseEstimator's encoder-delta
    # baseline reset, so it must be unaffected by zero_encoders too.
    assert after.has_otos == before.has_otos == True  # noqa: E712 -- explicit tri-state check
    assert after.otos_connected == before.otos_connected == True  # noqa: E712
    assert after.otos.x == pytest.approx(before.otos.x, abs=1e-3)
    assert after.otos.y == pytest.approx(before.otos.y, abs=1e-3)
    assert after.otos.h == pytest.approx(before.otos.h, abs=1e-3)


def test_pose_fix_both_flags_set_both_branches_run(sim):
    """reset=true AND zero_encoders=true in ONE message: both branches run
    (architecture-update.md D5-D8's own "both may be set -- both run")."""
    reply = _pose_fix(sim, 20, reset=True, zero_encoders=True, x=-40.0, y=500.0, h=-1.1)
    assert reply.WhichOneof("body") == "ok"
    assert reply.corr_id == 20

    frame = read_tlm_now(sim)
    assert frame.pose.x == pytest.approx(-40.0, abs=0.5)
    assert frame.pose.y == pytest.approx(500.0, abs=0.5)
    assert frame.pose.h == pytest.approx(-1.1, abs=1e-3)


def test_pose_fix_neither_flag_replies_err_unimplemented(sim):
    """A stale/garbage neither-flag request (a genuine timestamped delayed
    camera fix -- 099-008's job, not this ticket's) replies
    Error{ERR_UNIMPLEMENTED, field=7} -- unchanged behavior for the
    not-yet-live variant, matching the pre-099-004 declared-only stub."""
    reply = _pose_fix(sim, 30, x=1.0, y=2.0, h=3.0, t=12345)
    assert reply.WhichOneof("body") == "err"
    assert reply.err.code == pb_envelope.ERR_UNIMPLEMENTED
    assert reply.err.field == 7   # CommandEnvelope.cmd.pose_fix's own field number
    assert reply.corr_id == 30


def test_pose_fix_default_empty_request_replies_err_unimplemented(sim):
    """A completely empty PoseFix{} (every field at its proto3 zero default,
    the simplest possible "neither flag set" request) hits the SAME
    not-yet-live branch -- confirms the false/false default itself (not
    just an explicit x/y/h/t payload) routes to ERR_UNIMPLEMENTED."""
    reply = _pose_fix(sim, 31)
    assert reply.WhichOneof("body") == "err"
    assert reply.err.code == pb_envelope.ERR_UNIMPLEMENTED
    assert reply.err.field == 7


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
