"""tests/unit/test_protocol_binary_client.py — 096-007 (M6 Host Config/
Telemetry Client).

Covers this ticket's two additions to ``host/robot_radio/robot/protocol.py``,
none of which need live hardware:

1. ``TLMFrame.from_pb2()`` — an alternate constructor adapting a binary-plane
   ``pb2.Telemetry`` message onto the SAME ``TLMFrame`` dataclass shape
   ``parse_tlm()`` already produces from a text STREAM/SNAP line. Tested by
   comparing ``from_pb2(telemetry)`` against ``parse_tlm(<the matching text
   line>)`` field-for-field, for every field the two wire formats share, and
   confirming the fields they do NOT share stay at this dataclass's own
   default.

2. ``NezhaProtocol.set_config_binary()``/``get_config_binary()`` — binary
   config set/get built on ``SerialConnection.send_envelope()``. Round-
   tripped end to end (``NezhaProtocol`` -> ``SerialConnection.
   send_envelope()`` -> a synthetic loopback transport -> the real reader
   thread -> the corr-id reply queue -> back to ``NezhaProtocol``) against a
   fake transport that never touches a real serial port — the same
   no-hardware ``_LoopbackSerial`` pattern
   ``tests/unit/test_serial_conn_binary_plane.py`` (095-002) already
   established, extended here to synthesize an ``Ack``/``ConfigSnapshot``
   reply keyed off the request's own oneof arm (mirroring BinaryChannel's
   CONFIG/GET arms, ``source/commands/binary_channel.cpp``, closely enough
   to exercise the full envelope round trip). ``pb2`` (``host/robot_radio/
   robot/pb2/``) is itself the "host-side codec" ticket 096-006's own
   differential-test docstring names — see
   ``tests/sim/unit/test_wire_differential.py``'s file header — so building
   reference ``ConfigDelta``/``CommandEnvelope`` messages directly via pb2
   here (rather than shelling out to the compiled differential harness,
   which exists to check the FIRMWARE codec against pb2, not this purely
   host-side client) is exactly what this ticket's own Testing section asks
   for: "build a ConfigDelta, round-trip through pb2 serialize/parse".

Collected under ``tests/unit/`` (host-side unit/tooling check, not
sim/bench/playfield-scoped — see ``tests/CLAUDE.md``); ``pyproject.toml``'s
``testpaths`` includes ``tests/unit`` so ``uv run python -m pytest`` collects
it.
"""

from __future__ import annotations

import base64
import queue

import pytest

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.pb2 import common_pb2, config_pb2, envelope_pb2, planner_pb2, telemetry_pb2
from robot_radio.robot.protocol import NezhaProtocol, TLMFrame, parse_tlm

# ---------------------------------------------------------------------------
# 1. TLMFrame.from_pb2()
# ---------------------------------------------------------------------------

# Fields both wire formats carry -- compared directly, field-for-field,
# between from_pb2(telemetry) and parse_tlm(<the matching text line>).
_SHARED_FIELDS = ("t", "mode", "seq", "enc", "vel", "cmd_vel", "pose", "otos", "twist")


def test_from_pb2_matches_text_parse_for_every_shared_field():
    telemetry = telemetry_pb2.Telemetry(
        now=12345,
        mode=planner_pb2.DISTANCE,
        seq=7,
        has_enc=True, enc_left=100.0, enc_right=-50.0,
        has_vel=True, vel_left=200.0, vel_right=-199.0,
        has_cmd_vel=True, cmd_vel_left=210.0, cmd_vel_right=-190.0,
        has_pose=True, pose=common_pb2.Pose2D(x=350.0, y=-12.0, h=1.0),
        has_otos=True, otos=common_pb2.Pose2D(x=1.0, y=2.0, h=0.5), otos_connected=True,
        has_twist=True, twist=common_pb2.BodyTwist3(v_x=150.0, v_y=0.0, omega=0.3),
    )

    from_pb2_frame = TLMFrame.from_pb2(telemetry)

    # The text plane's pose=/otos= tokens already carry PRE-CONVERTED
    # centidegree ints (buildTlmFrame() does the radians->cdeg conversion
    # firmware-side before formatting the line) -- so the "matching text
    # line" is built using the SAME centidegree ints from_pb2() itself
    # computed, canonicalizing both sides through the identical transform
    # before comparing (this project's established differential-test
    # posture -- tests/sim/unit/_wire_diff_driver.py's own f32()/float_eq()
    # precedent for exactly this kind of cross-format agreement check).
    pose_h_cdeg = from_pb2_frame.pose[2]
    otos_h_cdeg = from_pb2_frame.otos[2]
    line = (
        f"TLM t=12345 mode=D seq=7 enc=100,-50 vel=200,-199 cmd=210,-190 "
        f"pose=350,-12,{pose_h_cdeg} otos=1,2,{otos_h_cdeg} twist=150,300"
    )
    text_frame = parse_tlm(line)
    assert text_frame is not None

    for name in _SHARED_FIELDS:
        assert getattr(from_pb2_frame, name) == getattr(text_frame, name), name

    # Fields telemetry.proto/TLMFrame do NOT share stay at this dataclass's
    # own default -- see from_pb2()'s own doc comment for why each is
    # unshared.
    for name in ("wedge", "encpose", "otos_health", "ekf_rej", "line", "color"):
        assert getattr(from_pb2_frame, name) is None, name


def test_from_pb2_absent_optional_fields_stay_none():
    """No has_* flag set (beyond the always-present now/mode/seq) -> every
    gated field stays None, matching parse_tlm() on a line with no
    matching key=value token."""
    telemetry = telemetry_pb2.Telemetry(now=1, mode=planner_pb2.IDLE, seq=0)

    frame = TLMFrame.from_pb2(telemetry)

    assert frame.t == 1
    assert frame.mode == "I"
    assert frame.seq == 0
    assert frame.enc is None
    assert frame.vel is None
    assert frame.cmd_vel is None
    assert frame.pose is None
    assert frame.otos is None
    assert frame.twist is None

    text_frame = parse_tlm("TLM t=1 mode=I seq=0")
    assert text_frame is not None
    for name in _SHARED_FIELDS:
        assert getattr(frame, name) == getattr(text_frame, name), name


@pytest.mark.parametrize(
    ("mode_value", "expected_char"),
    [
        (planner_pb2.IDLE, "I"),
        (planner_pb2.STREAMING, "S"),
        (planner_pb2.TIMED, "T"),
        (planner_pb2.DISTANCE, "D"),
        (planner_pb2.GO_TO, "G"),
        (planner_pb2.VELOCITY, "I"),  # modeChar()'s own `default: return 'I';` case
    ],
)
def test_from_pb2_mode_mapping_matches_modechar(mode_value, expected_char):
    telemetry = telemetry_pb2.Telemetry(now=0, mode=mode_value, seq=0)
    frame = TLMFrame.from_pb2(telemetry)
    assert frame.mode == expected_char


def test_from_pb2_drops_bench_diagnostic_fields_with_no_tlmframe_slot():
    """acc_/active/conn_/glitch_/ts_ -- telemetry.proto's OTHER curated text
    surface (the one-shot TLM verb's "OK tlm ..." reply, handleTlm()) --
    have no TLMFrame field at all; from_pb2() must not invent one."""
    telemetry = telemetry_pb2.Telemetry(
        now=1, mode=planner_pb2.IDLE, seq=0,
        acc_left=1.0, acc_right=2.0, active=True,
        conn_left=True, conn_right=False,
        glitch_left=3, glitch_right=4, ts_left=5, ts_right=6,
    )

    frame = TLMFrame.from_pb2(telemetry)

    for attr in ("acc_left", "acc_right", "active", "conn_left", "conn_right",
                 "glitch_left", "glitch_right", "ts_left", "ts_right"):
        assert not hasattr(frame, attr), attr


# ---------------------------------------------------------------------------
# 2. NezhaProtocol.set_config_binary() / get_config_binary()
# ---------------------------------------------------------------------------


class _ConfigLoopbackSerial:
    """Mock transport for the binary config set/get round-trip tests.

    On write() of a `*B<base64>` CommandEnvelope, decodes it, records it
    (``sent_envelopes``), and synthesizes a reply keyed off the request's
    OWN oneof arm -- an ``Ack`` for a ``config`` request, a
    ``ConfigSnapshot`` for a ``get`` request (looked up in
    ``snapshot_by_target``, defaulting to an empty snapshot echoing the
    requested target) -- mirroring BinaryChannel's CONFIG/GET arms
    (source/commands/binary_channel.cpp) closely enough to exercise
    NezhaProtocol's full envelope round trip with no real serial port.
    """

    is_open = True

    def __init__(self, snapshot_by_target: dict[int, "envelope_pb2.ConfigSnapshot"] | None = None):
        self._pending: queue.Queue = queue.Queue()
        self.sent_envelopes: list[envelope_pb2.CommandEnvelope] = []
        self._snapshot_by_target = snapshot_by_target or {}

    def write(self, data: bytes) -> int:
        text = data.decode("ascii").strip()
        if text.startswith("*B"):
            raw = base64.b64decode(text[2:])
            cmd = envelope_pb2.CommandEnvelope.FromString(raw)
            self.sent_envelopes.append(cmd)

            reply = envelope_pb2.ReplyEnvelope(corr_id=cmd.corr_id)
            which = cmd.WhichOneof("cmd")
            if which == "config":
                reply.ok.q = 1
                reply.ok.rem = 0.0
                reply.ok.t = 999
            elif which == "get":
                snap = self._snapshot_by_target.get(cmd.get.target)
                if snap is not None:
                    reply.cfg.CopyFrom(snap)
                else:
                    reply.cfg.target = cmd.get.target
            armored = "*B" + base64.b64encode(reply.SerializeToString()).decode("ascii")
            self._pending.put((armored + "\n").encode("ascii"))
        return len(data)

    def flush(self) -> None:
        pass

    def readline(self) -> bytes:
        try:
            return self._pending.get(timeout=0.2)
        except queue.Empty:
            return b""


class _NoReplySerial:
    """Mock transport that never replies -- exercises the timeout path."""

    is_open = True

    def write(self, data: bytes) -> int:
        return len(data)

    def flush(self) -> None:
        pass

    def readline(self) -> bytes:
        return b""


class _ErrReplySerial:
    """Mock transport that always replies with an Error -- exercises the
    "reply arrived but wasn't the expected body arm" path."""

    is_open = True

    def __init__(self) -> None:
        self._pending: queue.Queue = queue.Queue()

    def write(self, data: bytes) -> int:
        text = data.decode("ascii").strip()
        if text.startswith("*B"):
            raw = base64.b64decode(text[2:])
            cmd = envelope_pb2.CommandEnvelope.FromString(raw)
            reply = envelope_pb2.ReplyEnvelope(corr_id=cmd.corr_id)
            reply.err.code = envelope_pb2.ERR_UNKNOWN
            reply.err.field = 1
            armored = "*B" + base64.b64encode(reply.SerializeToString()).decode("ascii")
            self._pending.put((armored + "\n").encode("ascii"))
        return len(data)

    def flush(self) -> None:
        pass

    def readline(self) -> bytes:
        try:
            return self._pending.get(timeout=0.2)
        except queue.Empty:
            return b""


def test_set_config_binary_round_trips_ack_and_builds_correct_envelope():
    fake = _ConfigLoopbackSerial()
    conn = SerialConnection()
    conn._ser = fake
    conn._start_reader()
    try:
        proto = NezhaProtocol(conn)
        delta = envelope_pb2.ConfigDelta(
            drivetrain=config_pb2.DrivetrainConfigPatch(trackwidth=128.0, rotational_slip=0.92))
        ack = proto.set_config_binary(delta)
    finally:
        conn._stop_reader()

    assert ack is not None
    assert ack.q == 1
    assert ack.t == 999

    assert len(fake.sent_envelopes) == 1
    sent = fake.sent_envelopes[0]
    assert sent.WhichOneof("cmd") == "config"
    assert sent.config.WhichOneof("patch") == "drivetrain"
    assert sent.config.drivetrain.trackwidth == pytest.approx(128.0)
    assert sent.config.drivetrain.rotational_slip == pytest.approx(0.92)

    # Byte-identical to an independently pb2-built reference envelope with
    # the SAME (send_envelope()-assigned) corr_id -- pb2 is the "host-side
    # codec" here (see this file's own header note).
    reference = envelope_pb2.CommandEnvelope(
        corr_id=sent.corr_id,
        config=envelope_pb2.ConfigDelta(
            drivetrain=config_pb2.DrivetrainConfigPatch(trackwidth=128.0, rotational_slip=0.92)))
    assert sent.SerializeToString() == reference.SerializeToString()


def test_set_config_binary_watchdog_arm():
    """ConfigDelta's bare-uint32 `watchdog` oneof arm (sTimeout) -- a
    different shape than the three message-typed Patch arms above."""
    fake = _ConfigLoopbackSerial()
    conn = SerialConnection()
    conn._ser = fake
    conn._start_reader()
    try:
        proto = NezhaProtocol(conn)
        ack = proto.set_config_binary(envelope_pb2.ConfigDelta(watchdog=5000))
    finally:
        conn._stop_reader()

    assert ack is not None
    sent = fake.sent_envelopes[0]
    assert sent.config.WhichOneof("patch") == "watchdog"
    assert sent.config.watchdog == 5000


def test_get_config_binary_round_trips_config_snapshot_and_builds_correct_envelope():
    canned_snapshot = envelope_pb2.ConfigSnapshot(
        target=config_pb2.CONFIG_MOTOR_LEFT,
        motor=config_pb2.MotorConfigPatch(side=config_pb2.LEFT, travel_calib=0.375, kp=1.5))
    fake = _ConfigLoopbackSerial(snapshot_by_target={config_pb2.CONFIG_MOTOR_LEFT: canned_snapshot})
    conn = SerialConnection()
    conn._ser = fake
    conn._start_reader()
    try:
        proto = NezhaProtocol(conn)
        snapshot = proto.get_config_binary(config_pb2.CONFIG_MOTOR_LEFT)
    finally:
        conn._stop_reader()

    assert snapshot is not None
    assert snapshot.target == config_pb2.CONFIG_MOTOR_LEFT
    assert snapshot.WhichOneof("patch") == "motor"
    assert snapshot.motor.side == config_pb2.LEFT
    assert snapshot.motor.travel_calib == pytest.approx(0.375)
    assert snapshot.motor.kp == pytest.approx(1.5)

    assert len(fake.sent_envelopes) == 1
    sent = fake.sent_envelopes[0]
    assert sent.WhichOneof("cmd") == "get"
    assert sent.get.target == config_pb2.CONFIG_MOTOR_LEFT

    reference = envelope_pb2.CommandEnvelope(
        corr_id=sent.corr_id, get=envelope_pb2.ConfigGet(target=config_pb2.CONFIG_MOTOR_LEFT))
    assert sent.SerializeToString() == reference.SerializeToString()


def test_set_config_binary_returns_none_on_timeout():
    conn = SerialConnection()
    conn._ser = _NoReplySerial()
    conn._start_reader()
    try:
        proto = NezhaProtocol(conn)
        ack = proto.set_config_binary(
            envelope_pb2.ConfigDelta(planner=config_pb2.PlannerConfigPatch(min_speed=50.0)),
            read_timeout=50)
    finally:
        conn._stop_reader()

    assert ack is None


def test_get_config_binary_returns_none_on_error_reply():
    conn = SerialConnection()
    conn._ser = _ErrReplySerial()
    conn._start_reader()
    try:
        proto = NezhaProtocol(conn)
        snapshot = proto.get_config_binary(config_pb2.CONFIG_WATCHDOG, read_timeout=200)
    finally:
        conn._stop_reader()

    assert snapshot is None


def test_set_config_binary_not_connected_returns_none():
    conn = SerialConnection()  # _ser stays None -- never connected
    proto = NezhaProtocol(conn)

    ack = proto.set_config_binary(envelope_pb2.ConfigDelta(watchdog=1000), read_timeout=50)

    assert ack is None


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
