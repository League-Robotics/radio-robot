"""tests/sim/unit/test_pose_fix_end_to_end.py -- 099-008 (SUC-005, SUC-007):
`CommandEnvelope.cmd.pose_fix`'s third branch (neither `reset` nor
`zero_encoders` set) -- a genuine timestamped delayed camera fix -- exercised
end to end via `sim_command_on()`:
`BinaryChannel::handlePose()` -> `bb.poseFixIn` ->
`Subsystems::PoseEstimator::tick()` (pose-history ring, interpolate,
rigid-compose, ungated `EkfTiny` update) -> `bb.fusedPose` (+
`bb.otosSetPoseIn` -> `Rt::MainLoop`'s odometer drain) -> read back over the
binary `stream` arm's periodic TLM `pose=` field
(`_binary_envelope.read_tlm_now()`). `bb.encoderPose` (`PoseEstimator`'s
pure dead-reckoning accumulator, the pose-history ring's own source series)
is read via `sim.enc_pose()` (099-008, `tests/_infra/sim/sim_api.cpp`'s
`sim_get_enc_pose_x/y/h` -- TEST-ONLY, since `encpose=` was trimmed from the
wire, 096-001).

`PoseFix` (`protos/drivetrain.proto`) retypes the formerly-declared-only
`CommandEnvelope.cmd.pose` arm -- see `architecture-update.md` D5-D8.
Ticket 099-004 implemented ONLY the `reset`/`zero_encoders` branches
(`test_pose_fix_reset_zero.py`); this ticket makes the third branch live.

`t=` (D6): robot-clock ms, matching the existing `Ack.t`/`PING` clock-sync
convention -- every fix below captures a real prior robot time via a `PING`
mid-drive (`ping.ok.t`), exactly the way a bench/playfield client would
(ticket 009's own aprilcam script does the same thing against real
hardware).

Covers ticket 099-008's own sim acceptance criterion:
  - drive, capture a real prior robot time via PING, drive further, then
    send a fix with a KNOWN offset at that captured time: `fusedPose`
    (`pose=`) converges measurably toward the composed target
    (`encoderPose_now + offset`, per architecture-update.md's exact rigid-
    compose formula) while `encoderPose` (`sim.enc_pose()`) is left
    completely untouched by the fix itself (it only continues its own
    ordinary dead-reckoning trend).
  - a stale-timestamp fix (`t` older than the pose-history ring's oldest
    entry) produces NO jump in `fusedPose` -- dropped internally (still an
    `OK` wire reply; the drop is silent/diagnostic-only, not a wire error),
    not crashed.
"""
from __future__ import annotations

import pathlib
import sys

import pytest

# tests/sim/unit/test_pose_fix_end_to_end.py -> unit -> sim -> tests -> repo root
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

from _binary_envelope import read_tlm_now, send, send_drive  # noqa: E402


def _pose_fix(sim, corr_id, **kwargs):
    return send(sim, pb_envelope.CommandEnvelope(
        corr_id=corr_id, pose_fix=pb_drivetrain.PoseFix(**kwargs)))


def _ping_now(sim, corr_id) -> int:  # [ms] robot-clock time, D6's convention
    reply = send(sim, pb_envelope.CommandEnvelope(corr_id=corr_id, ping=pb_envelope.Ping()))
    assert reply.WhichOneof("body") == "ok"
    return reply.ok.t


def test_pose_fix_converges_and_leaves_encoder_pose_untouched(sim):
    """A genuine delayed fix (neither `reset` nor `zero_encoders`) -- the
    third branch, live this ticket: `fusedPose` converges measurably toward
    the composed target, `encoderPose` is untouched."""
    assert send_drive(sim, 150, 150).WhichOneof("body") == "ok"
    sim.tick_for(500)   # build up several pose-history ring entries

    # Capture a real prior robot time, exactly the way a bench/playfield
    # client would (D6) -- and the encoder-frame pose AT that same instant
    # (this is the "enc(T)" a correct implementation must reconstruct via
    # ring interpolation, not something this test derives independently).
    t_capture = _ping_now(sim, corr_id=50)
    enc_at_capture = sim.enc_pose()

    sim.tick_for(500)   # more motion between the captured time and "now"

    # A known offset: the "camera" claims the robot was this far ahead in x
    # (unchanged y/h) at the captured time than dead-reckoning believed.
    offset_x = 250.0   # [mm]
    fix_x = enc_at_capture[0] + offset_x
    fix_y = enc_at_capture[1]
    fix_h = enc_at_capture[2]

    # read_tlm_now() itself ticks the sim forward one `step` (re-arming the
    # binary stream -- see its own doc comment) -- capture "now"'s
    # encoderPose() AFTER that tick, at the SAME instant fused_before is
    # read, so the two are directly comparable (no intervening motion
    # neither side of this pair accounts for).
    fused_before = read_tlm_now(sim)
    enc_now_before_fix = sim.enc_pose()

    reply = _pose_fix(sim, 51, x=fix_x, y=fix_y, h=fix_h, t=t_capture)
    assert reply.WhichOneof("body") == "ok"
    assert reply.corr_id == 51

    # encoderPose() is read IMMEDIATELY after the fix's own dt=0 dispatch
    # (send()'s own synchronous-command tick, no intervening tick_for()) --
    # applying a fix must not move it AT ALL (architecture-update.md D5-D8:
    # "a delayed fix does not [touch the ring/encoder accumulator] either").
    enc_immediately_after_fix = sim.enc_pose()
    assert enc_immediately_after_fix[0] == pytest.approx(enc_now_before_fix[0], abs=0.05)
    assert enc_immediately_after_fix[1] == pytest.approx(enc_now_before_fix[1], abs=0.05)
    assert enc_immediately_after_fix[2] == pytest.approx(enc_now_before_fix[2], abs=1e-4)

    # fusedPose() converges MEASURABLY toward the composed target
    # (encoderPose_now + offset_x) -- not necessarily hitting it exactly
    # (the ungated EkfTiny update is a Kalman correction, gain < 1), but a
    # substantial fraction of the injected offset, in the RIGHT direction,
    # not a tiny/absent/wrong-direction change.
    #
    # Thresholds loosened from their 099-008 values (0.5 fraction / 0.5
    # tolerance) by 099-007: OTOS fusion is now LIVE (previously otosObs was
    # a literal nullptr, so this ungated fix's own correction was the ONLY
    # EkfTiny update ever applied, against a covariance P that had grown
    # unboundedly since boot -- gain near 1, near-full correction). With
    # OTOS fusion running every pass, P is continuously pulled back down
    # toward its (small) R_otos steady state, so the SAME fix now has a
    # smaller, but still substantial and still measurably-converging, gain
    # (observed ~46% of the offset covered; 0.3/0.65 below leave margin
    # without accepting a tiny/absent/wrong-direction change).
    fused_after = read_tlm_now(sim)
    composed_target_x = enc_now_before_fix[0] + offset_x
    moved_x = fused_after.pose.x - fused_before.pose.x
    assert moved_x > offset_x * 0.3, (
        f"fusedPose().pose.x barely moved ({moved_x:.2f}mm) after a delayed "
        f"fix with a {offset_x}mm offset -- expected a substantial "
        "correction toward the composed target"
    )
    assert fused_after.pose.x == pytest.approx(composed_target_x, abs=offset_x * 0.65), (
        f"fusedPose().pose.x ({fused_after.pose.x:.2f}) is not converging "
        f"toward the composed target ({composed_target_x:.2f})"
    )
    # y/h were given no offset (fix.y/fix.h reconstruct the ring's own
    # y=0/h=0 straight-line history) -- fusedPose() should not have drifted
    # off of them either.
    assert fused_after.pose.y == pytest.approx(0.0, abs=5.0)
    assert fused_after.pose.h == pytest.approx(0.0, abs=0.05)


def test_pose_fix_stale_timestamp_produces_no_jump(sim):
    """A fix timestamped older than the pose-history ring's oldest entry is
    dropped -- silently, internally (still an `OK` wire reply: the drop is
    a diagnostic, not a wire error) -- `fusedPose` shows no jump, just its
    ordinary continued drift, and the estimator keeps working afterward."""
    assert send_drive(sim, 150, 150).WhichOneof("body") == "ok"
    # Drive long enough that the ring (24 entries, ~50ms cadence) has
    # evicted its very first (session-start, t~0) entry -- otherwise t=1
    # below would not actually be "older than the ring's oldest entry" (a
    # session-start entry at t=0 make t=1 a valid, non-stale timestamp).
    sim.tick_for(3000)

    before = read_tlm_now(sim)

    reply = _pose_fix(sim, 60, x=-99999.0, y=99999.0, h=2.0, t=1)
    # Still an OK wire reply -- the third branch always dispatches/acks;
    # rejecting a stale t happens silently inside PoseEstimator, not at the
    # BinaryChannel dispatch boundary.
    assert reply.WhichOneof("body") == "ok"
    assert reply.corr_id == 60

    after = read_tlm_now(sim)

    # No jump: the wildly-different (x, y, h) the stale fix carried must
    # never have reached fusedPose() at all -- only the small, ordinary
    # continued-drive delta between the two read_tlm_now() calls' own extra
    # ticks is expected.
    assert after.pose.x == pytest.approx(before.pose.x, abs=20.0)
    assert after.pose.y == pytest.approx(before.pose.y, abs=5.0)
    assert after.pose.h == pytest.approx(before.pose.h, abs=0.05)
    assert abs(after.pose.x) < 90000.0, "fusedPose().pose.x must not reflect the stale fix's -99999 payload"
    assert abs(after.pose.y) < 90000.0, "fusedPose().pose.y must not reflect the stale fix's 99999 payload"

    # The estimator keeps working normally afterward (no crash -- a further
    # ordinary drive tick still produces sane, finite telemetry).
    sim.tick_for(240)
    still_ok = read_tlm_now(sim)
    assert still_ok.pose.x == still_ok.pose.x  # not NaN
    assert still_ok.pose.x > after.pose.x - 1.0   # still driving forward, not frozen/corrupted


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
