"""095-007: BinaryChannel + `*` discriminator + CommandProcessor/CommandRouter
wiring, exercised end to end over the sim's own command channel
(`sim.command()`/`sim.command_on()`/`sim.route_no_tick()`), mirroring
`test_bare_loop_move_and_tlm.py`'s established pattern (a fixture-provided
`sim`, plain assertions against replies and `sim.vel()`/`sim.tick_for()`)
extended for the BINARY plane: every request here is a `*B<base64>`-armored
`msg::CommandEnvelope`, built with the host's `pb2` bindings (the same
reference codec ticket 006's differential suite already proved correct)
and hand-armored/dearmored in this file (mirrors `send_envelope()`'s own
eventual host-side shape, ticket 002 -- but this file talks to the sim's
raw `sim.command()` line channel directly, not `serial_conn.py`).

Covers every one of ticket 007's acceptance criteria:
  - `drive` posts the exact decoded `msg::DrivetrainCommand`, unmodified,
    to `bb.driveIn` (behavioral: wheels spin).
  - `segment`/`replace` translate all 13 `MotionSegment` fields into a
    `Motion::Segment`, verified INDIVIDUALLY via `sim.peek_segment_in()`/
    `sim.peek_replace_in()` (non-destructive Blackboard reads, `sim_api.cpp`
    095-007 test-support additions) BEFORE any tick drains the queue.
  - `stop` posts `msg::DrivetrainCommand{NEUTRAL=BRAKE}` to `bb.driveIn`.
  - `ping`/`echo`/`id` reply inline (no Blackboard post), matching their
    text counterparts' information content.
  - `config`/`pose`/`otos`/`get`/`stream` reply `Error{ERR_UNIMPLEMENTED}`.
  - Malformed/out-of-range input yields a typed `Error{code, field}`.
  - A mixed text+binary session in ONE test proves dual-stack coexistence.
"""
from __future__ import annotations

import base64
import pathlib
import sys

import pytest

# tests/sim/unit/test_binary_channel.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_HOST_DIR = _REPO_ROOT / "host"
if str(_HOST_DIR) not in sys.path:
    sys.path.insert(0, str(_HOST_DIR))

from robot_radio.robot.pb2 import common_pb2 as pb_common  # noqa: E402
from robot_radio.robot.pb2 import drivetrain_pb2 as pb_drivetrain  # noqa: E402
from robot_radio.robot.pb2 import envelope_pb2 as pb_envelope  # noqa: E402
from robot_radio.robot.pb2 import motion_pb2 as pb_motion  # noqa: E402
from robot_radio.robot.pb2 import odometer_pb2 as pb_odometer  # noqa: E402

pytestmark = pytest.mark.filterwarnings("ignore")

# tests/sim/conftest.py's build_lib fixture already inserts this path; guard
# against a double-insert if this module is imported before that fixture runs.
_SIM_INFRA_DIR = _REPO_ROOT / "tests" / "_infra" / "sim"
if str(_SIM_INFRA_DIR) not in sys.path:
    sys.path.insert(0, str(_SIM_INFRA_DIR))
from firmware import CHANNEL_SERIAL  # noqa: E402


# ---------------------------------------------------------------------------
# Armor / dearmor helpers -- `*B<base64>`, standard alphabet (wire_runtime.h's
# own pinned-alphabet note).
# ---------------------------------------------------------------------------


def armor(envelope: "pb_envelope.CommandEnvelope") -> str:
    return "*B" + base64.b64encode(envelope.SerializeToString()).decode("ascii")


def dearmor(line: str) -> "pb_envelope.ReplyEnvelope":
    line = line.strip()
    assert line.startswith("*B"), f"expected an armored binary reply, got: {line!r}"
    raw = base64.b64decode(line[2:])
    reply = pb_envelope.ReplyEnvelope()
    reply.ParseFromString(raw)
    return reply


def send(sim, envelope: "pb_envelope.CommandEnvelope", channel: int = CHANNEL_SERIAL) -> "pb_envelope.ReplyEnvelope":
    """Send one binary command through the sim's dt=0 synchronous channel
    (ticks once, like every text `sim.command()` call already does)."""
    return dearmor(sim.command_on(armor(envelope), channel))


def send_no_tick(sim, envelope: "pb_envelope.CommandEnvelope") -> "pb_envelope.ReplyEnvelope":
    """Route one binary command WITHOUT the trailing tick -- lets the test
    peek bb.segmentIn/bb.replaceIn's raw posted payload before Drivetrain::
    tick() drains it (sim_route_no_tick(), 095-007 test-support addition)."""
    return dearmor(sim.route_no_tick(armor(envelope)))


_SEGMENT_SHAPE = dict(
    distance=456.5, direction=0.11, final_heading=0.22, speed_max=700.0, accel_max=1500.0,
    jerk_max=25000.0, yaw_rate_max=4.0, yaw_accel_max=30.0, yaw_jerk_max=80.0,
    time=123.0, v=-88.5, omega=1.75, stream=True,
)


def _assert_segment_matches_shape(peeked: dict, shape: dict) -> None:
    # peek_segment_in()/peek_replace_in() key their dict by MotionSegment's
    # own proto field spelling (final_heading, speed_max, ...) -- see
    # firmware.py's _SEGMENT_FIELDS -- so this compares 1:1 against `shape`
    # (also keyed that way) with no name translation needed at the test
    # level; BinaryChannel's OWN translation (proto snake_case ->
    # Motion::Segment's camelCase) already happened C++-side before
    # sim_peek_segment_in() ever ran.
    for key, expected in shape.items():
        if key == "stream":
            assert peeked["stream"] == expected
        else:
            assert peeked[key] == pytest.approx(expected, rel=1e-5), \
                f"{key}: got {peeked[key]}, expected {expected}"


# ===========================================================================
# drive -- posts the exact decoded msg::DrivetrainCommand, unmodified.
# ===========================================================================


def test_binary_drive_wheels_spins_the_wheels(sim):
    wheels = pb_drivetrain.WheelTargets(w=[
        pb_common.WheelTarget(speed=150.0), pb_common.WheelTarget(speed=150.0),
    ])
    env = pb_envelope.CommandEnvelope(corr_id=1, drive=pb_drivetrain.DrivetrainCommand(wheels=wheels))
    reply = send(sim, env)
    assert reply.WhichOneof("body") == "ok"
    assert reply.corr_id == 1

    sim.tick_for(1000)
    vel_l, vel_r = sim.vel()
    assert vel_l > 50.0 and vel_r > 50.0, "binary drive{wheels} never reached the plant"


def test_binary_stop_posts_neutral_brake(sim):
    """Byte-identical to handleStop()'s own NEUTRAL{BRAKE} construction --
    behaviorally: a spinning direct-mode drive settles to zero after a
    binary `stop`, matching test_s_and_stop_still_work_unchanged_over_the_wire's
    text-plane assertion shape."""
    wheels = pb_drivetrain.WheelTargets(w=[
        pb_common.WheelTarget(speed=150.0), pb_common.WheelTarget(speed=150.0),
    ])
    reply = send(sim, pb_envelope.CommandEnvelope(corr_id=1, drive=pb_drivetrain.DrivetrainCommand(wheels=wheels)))
    assert reply.WhichOneof("body") == "ok"
    sim.tick_for(1000)
    vel_l, vel_r = sim.vel()
    assert vel_l > 50.0 and vel_r > 50.0

    reply = send(sim, pb_envelope.CommandEnvelope(corr_id=2, stop=pb_envelope.Stop()))
    assert reply.WhichOneof("body") == "ok"
    assert reply.corr_id == 2
    sim.tick_for(1000)
    vel_l, vel_r = sim.vel()
    assert vel_l == pytest.approx(0.0, abs=10.0)
    assert vel_r == pytest.approx(0.0, abs=10.0)


# ===========================================================================
# segment / replace -- all 13 MotionSegment fields, translated individually.
# ===========================================================================


def test_binary_segment_translates_all_13_fields(sim):
    seg = pb_motion.MotionSegment(**_SEGMENT_SHAPE)
    reply = send_no_tick(sim, pb_envelope.CommandEnvelope(corr_id=3, segment=seg))
    assert reply.WhichOneof("body") == "ok"
    assert reply.corr_id == 3
    # q = bb.segmentIn.size() (1, just posted) + bb.drivetrain.queue (0,
    # nothing drained yet on a fresh sim) -- same formula handleMove()'s own
    # ack uses (motion_commands.cpp).
    assert reply.ok.q == 1
    assert reply.ok.rem == pytest.approx(0.0)

    peeked = sim.peek_segment_in(0)
    assert peeked is not None, "segment never reached bb.segmentIn"
    _assert_segment_matches_shape(peeked, _SEGMENT_SHAPE)


def test_binary_replace_translates_all_13_fields(sim):
    seg = pb_motion.MotionSegment(**_SEGMENT_SHAPE)
    reply = send_no_tick(sim, pb_envelope.CommandEnvelope(corr_id=4, replace=seg))
    assert reply.WhichOneof("body") == "ok"
    assert reply.corr_id == 4

    peeked = sim.peek_replace_in()
    assert peeked is not None, "segment never reached bb.replaceIn"
    _assert_segment_matches_shape(peeked, _SEGMENT_SHAPE)


def test_binary_segment_drives_the_plant(sim):
    """Lighter behavioral companion to the field-level proof above -- a
    binary MOVE-equivalent segment actually reaches the executor and drives
    the wheels, the same end-to-end proof test_bare_loop_move_and_tlm.py's
    text-plane MOVE tests already establish."""
    seg = pb_motion.MotionSegment(distance=300.0, direction=0.0, final_heading=0.0, speed_max=0.0,
                                  accel_max=0.0, jerk_max=0.0, yaw_rate_max=0.0, yaw_accel_max=0.0,
                                  yaw_jerk_max=0.0, time=0.0, v=0.0, omega=0.0, stream=False)
    reply = send(sim, pb_envelope.CommandEnvelope(corr_id=5, segment=seg))
    assert reply.WhichOneof("body") == "ok"

    max_v = 0.0
    for _ in range(150):
        sim.tick_for(24)
        vel_l, vel_r = sim.vel()
        max_v = max(max_v, (vel_l + vel_r) / 2.0)
    assert max_v > 50.0, "binary segment never genuinely drove"


def test_binary_segment_full_queue_replies_err_full(sim):
    """bb.segmentIn is an Rt::WorkQueue<Motion::Segment, 8> -- mirrors
    handleMove()'s own `ERR full` text behavior once the queue is at
    capacity. route_no_tick() never drains it, so 8 posts fill it exactly."""
    seg = pb_motion.MotionSegment(distance=10.0)
    for i in range(8):
        reply = send_no_tick(sim, pb_envelope.CommandEnvelope(corr_id=100 + i, segment=seg))
        assert reply.WhichOneof("body") == "ok", f"post {i} unexpectedly rejected"

    reply = send_no_tick(sim, pb_envelope.CommandEnvelope(corr_id=109, segment=seg))
    assert reply.WhichOneof("body") == "err"
    assert reply.err.code == pb_envelope.ERR_FULL


# ===========================================================================
# ping / echo / id -- inline replies, no Blackboard post.
# ===========================================================================


def test_binary_ping_replies_with_robot_clock_timestamp(sim):
    sim.tick_for(240)   # advance the host fake clock to a known value
    reply = send(sim, pb_envelope.CommandEnvelope(corr_id=6, ping=pb_envelope.Ping()))
    assert reply.WhichOneof("body") == "ok"
    assert reply.corr_id == 6
    # Ack.t (095-007 schema-gap closure) -- clock-sync parity with text
    # PING's own `OK pong t=<ms>` reply; Types::systemClockNow() at
    # route-time equals the sim's own tracked `now`.
    assert reply.ok.t == 240


@pytest.mark.parametrize("payload", [b"", b"\x00", b"hello binary", bytes(range(64))],
                         ids=["empty", "nul", "ascii", "max64"])
def test_binary_echo_replies_with_payload_verbatim(sim, payload):
    reply = send(sim, pb_envelope.CommandEnvelope(corr_id=7, echo=pb_envelope.Echo(payload=payload)))
    assert reply.WhichOneof("body") == "echo"
    assert reply.corr_id == 7
    assert reply.echo.payload == payload


def test_binary_id_replies_with_device_identity(sim):
    reply = send(sim, pb_envelope.CommandEnvelope(corr_id=8, id=pb_envelope.DeviceId()))
    assert reply.WhichOneof("body") == "id"
    assert reply.corr_id == 8
    # deviceIdentity() under HOST_BUILD -- system_commands.cpp's own fixed
    # placeholder, the SAME pair handleId()'s text ID reply uses.
    assert reply.id.model == "NEZHA2"
    assert reply.id.name == "HOST-SIM"
    assert reply.id.serial == 0
    assert reply.id.proto_version == 2


# ===========================================================================
# Declared-only arms -- ERR_UNIMPLEMENTED, never a crash, never silent.
# ===========================================================================


@pytest.mark.parametrize("kwargs,expected_field", [
    (dict(config=pb_envelope.ConfigDelta()), 6),
    (dict(pose=pb_drivetrain.SetPose()), 7),
    (dict(otos=pb_odometer.OdometerCommand()), 8),
    (dict(get=pb_envelope.ConfigGet(target=1)), 11),
    (dict(stream=pb_envelope.StreamControl()), 12),
], ids=["config", "pose", "otos", "get", "stream"])
def test_binary_declared_only_arms_reply_err_unimplemented(sim, kwargs, expected_field):
    env = pb_envelope.CommandEnvelope(corr_id=9, **kwargs)
    reply = send(sim, env)
    assert reply.WhichOneof("body") == "err"
    assert reply.err.code == pb_envelope.ERR_UNIMPLEMENTED
    assert reply.err.field == expected_field


# ===========================================================================
# Malformed / out-of-range input -- typed Error{code, field}, never a crash.
# ===========================================================================


def test_binary_malformed_base64_replies_err_decode(sim):
    reply = dearmor(sim.command("*B!!!not-valid-base64!!!"))
    assert reply.WhichOneof("body") == "err"
    assert reply.err.code == pb_envelope.ERR_DECODE


def test_binary_missing_armor_prefix_replies_err_decode(sim):
    """`*` alone (no `B`) still reaches BinaryChannel::handle() (only
    line[0]=='*' gates the dispatch in CommandProcessor::process()) -- must
    be rejected cleanly, not assumed to always be well-formed."""
    reply = dearmor(sim.command("*XnotArmored"))
    assert reply.WhichOneof("body") == "err"
    assert reply.err.code == pb_envelope.ERR_DECODE


def test_binary_out_of_range_segment_field_replies_err_range(sim):
    seg = pb_motion.MotionSegment(distance=99999.0)   # abs_max=10000
    reply = send(sim, pb_envelope.CommandEnvelope(corr_id=10, segment=seg))
    assert reply.WhichOneof("body") == "err"
    assert reply.err.code == pb_envelope.ERR_RANGE
    assert reply.err.field == 1   # MotionSegment.distance's own field number


def test_binary_empty_envelope_replies_err_unknown(sim):
    """A well-formed-but-empty CommandEnvelope (corr_id only, no oneof arm
    set) decodes cleanly (cmd_kind == NONE) -- BinaryChannel's default case
    must still reply, never silently drop."""
    reply = send(sim, pb_envelope.CommandEnvelope(corr_id=11))
    assert reply.WhichOneof("body") == "err"
    assert reply.err.code == pb_envelope.ERR_UNKNOWN


# ===========================================================================
# Dual-stack coexistence -- text and binary in ONE session.
# ===========================================================================


def test_mixed_text_and_binary_session(sim):
    """Proves the text plane and the binary plane share the SAME
    CommandRouter/CommandProcessor/Blackboard instance correctly within one
    session -- not just that each plane works alone (every other test in
    this file only ever sends binary; test_bare_loop_move_and_tlm.py/
    test_bare_loop_commands.py only ever send text)."""
    # Text S starts direct-mode driving.
    assert sim.command("S 150 150").strip() == "OK drive l=150 r=150"
    sim.tick_for(500)
    vel_l, vel_r = sim.vel()
    assert vel_l > 50.0 and vel_r > 50.0

    # Binary stop brakes the SAME drivetrain instance.
    reply = send(sim, pb_envelope.CommandEnvelope(corr_id=20, stop=pb_envelope.Stop()))
    assert reply.WhichOneof("body") == "ok"
    sim.tick_for(1000)
    vel_l, vel_r = sim.vel()
    assert vel_l == pytest.approx(0.0, abs=10.0)
    assert vel_r == pytest.approx(0.0, abs=10.0)

    # Binary drive restarts it.
    wheels = pb_drivetrain.WheelTargets(w=[
        pb_common.WheelTarget(speed=150.0), pb_common.WheelTarget(speed=150.0),
    ])
    reply = send(sim, pb_envelope.CommandEnvelope(corr_id=21, drive=pb_drivetrain.DrivetrainCommand(wheels=wheels)))
    assert reply.WhichOneof("body") == "ok"
    sim.tick_for(1000)
    vel_l, vel_r = sim.vel()
    assert vel_l > 50.0 and vel_r > 50.0

    # Text STOP brakes what binary drive started.
    assert sim.command("STOP").strip() == "OK stop"
    sim.tick_for(1000)
    vel_l, vel_r = sim.vel()
    assert vel_l == pytest.approx(0.0, abs=10.0)
    assert vel_r == pytest.approx(0.0, abs=10.0)

    # Both liveness verbs work side by side.
    text_ping = sim.command("PING").strip()
    assert text_ping.startswith("OK pong t=")
    bin_ping = send(sim, pb_envelope.CommandEnvelope(corr_id=22, ping=pb_envelope.Ping()))
    assert bin_ping.WhichOneof("body") == "ok"


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
