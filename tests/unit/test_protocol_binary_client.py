"""tests/unit/test_protocol_binary_client.py — 096-007 (M6 Host Config/
Telemetry Client) + 097-002/097-003 (M2/M3 NezhaProtocol conversion).

Covers this ticket's two additions to ``host/robot_radio/robot/protocol.py``,
none of which need live hardware:

1. ``TLMFrame.from_pb2()`` — an alternate constructor adapting a binary-plane
   ``pb2.Telemetry`` message onto the SAME ``TLMFrame`` dataclass shape the
   retired text-plane TLM parser used to produce from a text STREAM/SNAP
   line. Tested by comparing ``from_pb2(telemetry)`` against
   ``parse_historical_tlm_line(<the matching text line>)`` field-for-field
   (``parse_historical_tlm_line`` — ``robot_radio.robot._legacy_tlm_text`` —
   is a frozen, private copy of the module-level parser 097-003 deleted from
   ``protocol.py``, kept ONLY for this historical-parity check and the
   narrow set of non-``SerialConnection``/wire-schema-gap consumers that
   module's own docstring names; see that module for the full rationale),
   for every field the two wire formats share, and confirming the fields
   they do NOT share stay at this dataclass's own default.

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
import contextlib
import queue

import pytest

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.pb2 import common_pb2, config_pb2, envelope_pb2, planner_pb2, telemetry_pb2
from robot_radio.robot import protocol
from robot_radio.robot.protocol import NezhaProtocol, TLMFrame
from robot_radio.robot._legacy_tlm_text import parse_historical_tlm_line

# ---------------------------------------------------------------------------
# 1. TLMFrame.from_pb2()
# ---------------------------------------------------------------------------

# Fields both wire formats carry -- compared directly, field-for-field,
# between from_pb2(telemetry) and parse_historical_tlm_line(<the matching
# text line>).
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
    text_frame = parse_historical_tlm_line(line)
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
    gated field stays None, matching parse_historical_tlm_line() on a line with no
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

    text_frame = parse_historical_tlm_line("TLM t=1 mode=I seq=0")
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
    """acc_/conn_/glitch_/ts_ -- telemetry.proto's OTHER curated text
    surface (the one-shot TLM verb's "OK tlm ..." reply, handleTlm()) --
    have no TLMFrame field at all; from_pb2() must not invent one.

    ``active`` is the ONE exception (097, this ticket) -- see
    ``test_from_pb2_populates_active_for_segment_completion_detection``
    below and ``TLMFrame.from_pb2()``'s own docstring for why."""
    telemetry = telemetry_pb2.Telemetry(
        now=1, mode=planner_pb2.IDLE, seq=0,
        acc_left=1.0, acc_right=2.0, active=True,
        conn_left=True, conn_right=False,
        glitch_left=3, glitch_right=4, ts_left=5, ts_right=6,
    )

    frame = TLMFrame.from_pb2(telemetry)

    for attr in ("acc_left", "acc_right", "conn_left", "conn_right",
                 "glitch_left", "glitch_right", "ts_left", "ts_right"):
        assert not hasattr(frame, attr), attr


@pytest.mark.parametrize(("raw_active",), [(True,), (False,)])
def test_from_pb2_populates_active_for_segment_completion_detection(raw_active):
    """097: unlike every other bench-diagnostic field, ``active``
    (``bb.drivetrain.busy``) IS populated -- it is the reliable
    segment/replace-arm motion-complete signal (``mode``/``bb.planner.mode``
    never leaves IDLE for S/D/T/RT/R/TURN/G/MOVE/MOVER, all of which bypass
    the parked Planner -- see ``TLMFrame.from_pb2()``'s own docstring).
    ``__main__.py``'s ``_TourRunner._wait_for_idle`` polls this field."""
    telemetry = telemetry_pb2.Telemetry(
        now=1, mode=planner_pb2.IDLE, seq=0, active=raw_active,
    )
    frame = TLMFrame.from_pb2(telemetry)
    assert frame.active is raw_active


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
        # 097-002: raw armored lines actually written, for tests that assert
        # the literal wire bytes (not just the decoded envelope) -- see e.g.
        # test_set_config_single_drivetrain_key_sends_binary_and_returns_applied.
        self.raw_writes: list[bytes] = []
        self._snapshot_by_target = snapshot_by_target or {}

    def write(self, data: bytes) -> int:
        self.raw_writes.append(data)
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


# ---------------------------------------------------------------------------
# 3. NezhaProtocol's ten-method core conversion (097-002, M2)
#
# ping/echo/get_id/get_ver/stop/drive/timed/distance/get_config/set_config:
# each now builds a CommandEnvelope and round-trips it via
# SerialConnection.send_envelope(), same no-hardware pattern as section 2
# above. get_config()/set_config() reuse _ConfigLoopbackSerial (already
# established above); the other eight need a broader per-arm fake since they
# touch ping/echo/id/stop/drive/segment, not just config/get.
# ---------------------------------------------------------------------------

class _UniversalLoopbackSerial:
    """Generic mock transport for the ten-method conversion tests.

    Records every raw `*B<base64>` LINE actually written (``raw_writes`` --
    the literal bytes handed to write(), what this ticket's own acceptance
    criterion asks a test to assert against) and every decoded
    CommandEnvelope (``sent_envelopes``), then synthesizes a reply keyed off
    the request's own oneof arm -- mirroring BinaryChannel's per-arm handlers
    (source/commands/binary_channel.cpp: handlePing/handleEcho/handleId/
    handleStop/handleDrive/handleSegment) closely enough to exercise
    NezhaProtocol's full envelope round trip with no real serial port.
    """

    is_open = True

    def __init__(self, *, ping_t: int = 0,
                id_reply: "envelope_pb2.DeviceId | None" = None,
                ack_q: int = 0, ack_rem: float = 0.0):
        self._pending: queue.Queue = queue.Queue()
        self.raw_writes: list[bytes] = []
        self.sent_envelopes: list[envelope_pb2.CommandEnvelope] = []
        self._ping_t = ping_t
        self._id_reply = id_reply if id_reply is not None else envelope_pb2.DeviceId(
            model="NEZHA2", name="TESTBOT", serial=99,
            fw_version="v0.0.0-test", proto_version=3)
        self._ack_q = ack_q
        self._ack_rem = ack_rem

    def write(self, data: bytes) -> int:
        self.raw_writes.append(data)
        text = data.decode("ascii").strip()
        if not text.startswith("*B"):
            return len(data)
        raw = base64.b64decode(text[2:])
        cmd = envelope_pb2.CommandEnvelope.FromString(raw)
        self.sent_envelopes.append(cmd)

        reply = envelope_pb2.ReplyEnvelope(corr_id=cmd.corr_id)
        which = cmd.WhichOneof("cmd")
        if which == "ping":
            reply.ok.q = 0
            reply.ok.rem = 0.0
            reply.ok.t = self._ping_t
        elif which == "echo":
            reply.echo.payload = cmd.echo.payload
        elif which == "id":
            reply.id.CopyFrom(self._id_reply)
        elif which in ("stop", "drive", "segment", "replace"):
            reply.ok.q = self._ack_q
            reply.ok.rem = self._ack_rem
            reply.ok.t = 0
        else:
            reply.err.code = envelope_pb2.ERR_UNIMPLEMENTED
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


@contextlib.contextmanager
def _connected_proto(fake):
    """A NezhaProtocol wired to `fake` with the real reader thread running
    (torn down on exit) -- the shared setup every test below needs."""
    conn = SerialConnection()
    conn._ser = fake
    conn._start_reader()
    try:
        yield NezhaProtocol(conn)
    finally:
        conn._stop_reader()


def _assert_wire_bytes_match(fake, index: int, reference: "envelope_pb2.CommandEnvelope") -> None:
    """Assert the `index`-th raw line actually written to `fake` is the
    `*B<base64>` armoring of `reference` (corr_id already matched by the
    caller) -- the literal wire bytes, not just a decoded-field comparison."""
    expected = ("*B" + base64.b64encode(reference.SerializeToString()).decode("ascii") + "\n")
    assert fake.raw_writes[index] == expected.encode("ascii")


def test_ping_sends_binary_envelope_and_returns_t_and_rtt():
    fake = _UniversalLoopbackSerial(ping_t=54321)
    with _connected_proto(fake) as proto:
        result = proto.ping()

    assert result is not None
    t_robot, rtt = result
    assert t_robot == 54321
    assert rtt >= 0.0

    assert len(fake.sent_envelopes) == 1
    sent = fake.sent_envelopes[0]
    assert sent.WhichOneof("cmd") == "ping"
    reference = envelope_pb2.CommandEnvelope(corr_id=sent.corr_id, ping=envelope_pb2.Ping())
    assert sent.SerializeToString() == reference.SerializeToString()
    _assert_wire_bytes_match(fake, 0, reference)


def test_ping_returns_none_on_timeout():
    fake = _NoReplySerial()
    with _connected_proto(fake) as proto:
        result = proto.ping()
    assert result is None


def test_echo_sends_binary_envelope_and_returns_payload():
    fake = _UniversalLoopbackSerial()
    with _connected_proto(fake) as proto:
        result = proto.echo("hello robot")

    assert result == "hello robot"
    sent = fake.sent_envelopes[0]
    assert sent.WhichOneof("cmd") == "echo"
    assert sent.echo.payload == b"hello robot"
    reference = envelope_pb2.CommandEnvelope(
        corr_id=sent.corr_id, echo=envelope_pb2.Echo(payload=b"hello robot"))
    assert sent.SerializeToString() == reference.SerializeToString()
    _assert_wire_bytes_match(fake, 0, reference)


def test_get_id_sends_binary_envelope_and_returns_dict():
    id_reply = envelope_pb2.DeviceId(
        model="NEZHA2", name="GUTOV", serial=2121102,
        fw_version="v0.20260710.1", proto_version=3)
    fake = _UniversalLoopbackSerial(id_reply=id_reply)
    with _connected_proto(fake) as proto:
        result = proto.get_id()

    assert result == {
        "model": "NEZHA2",
        "name": "GUTOV",
        "serial": "2121102",
        "fw": "v0.20260710.1",
        "proto": "3",
    }
    sent = fake.sent_envelopes[0]
    assert sent.WhichOneof("cmd") == "id"
    reference = envelope_pb2.CommandEnvelope(corr_id=sent.corr_id, id=envelope_pb2.DeviceId())
    assert sent.SerializeToString() == reference.SerializeToString()
    _assert_wire_bytes_match(fake, 0, reference)


def test_get_ver_sends_the_same_id_arm_and_returns_fw_proto_subset():
    """097-002 acceptance criterion: get_ver()'s fw/proto keys come from the
    binary `id` arm's DeviceId.fw_version/.proto_version -- no independent
    binary `ver` arm exists."""
    id_reply = envelope_pb2.DeviceId(
        model="NEZHA2", name="GUTOV", serial=1,
        fw_version="v0.20260710.1", proto_version=3)
    fake = _UniversalLoopbackSerial(id_reply=id_reply)
    with _connected_proto(fake) as proto:
        result = proto.get_ver()

    assert result == {"fw": "v0.20260710.1", "proto": "3"}
    sent = fake.sent_envelopes[0]
    assert sent.WhichOneof("cmd") == "id"  # NOT a separate "ver" arm


def test_stop_sends_binary_envelope():
    fake = _UniversalLoopbackSerial()
    with _connected_proto(fake) as proto:
        result = proto.stop()

    assert result is None
    sent = fake.sent_envelopes[0]
    assert sent.WhichOneof("cmd") == "stop"
    reference = envelope_pb2.CommandEnvelope(corr_id=sent.corr_id, stop=envelope_pb2.Stop())
    assert sent.SerializeToString() == reference.SerializeToString()
    _assert_wire_bytes_match(fake, 0, reference)


def test_drive_sends_binary_envelope_with_wheel_targets():
    fake = _UniversalLoopbackSerial()
    with _connected_proto(fake) as proto:
        result = proto.drive(200, -150)

    assert result is None
    sent = fake.sent_envelopes[0]
    assert sent.WhichOneof("cmd") == "drive"
    assert sent.drive.WhichOneof("control") == "wheels"
    assert len(sent.drive.wheels.w) == 2
    assert sent.drive.wheels.w[0].speed == pytest.approx(200.0)
    assert sent.drive.wheels.w[1].speed == pytest.approx(-150.0)

    reference = envelope_pb2.CommandEnvelope(corr_id=sent.corr_id)
    reference.drive.wheels.w.add(speed=200.0)
    reference.drive.wheels.w.add(speed=-150.0)
    assert sent.SerializeToString() == reference.SerializeToString()
    _assert_wire_bytes_match(fake, 0, reference)


def test_drive_with_stop_arg_sends_no_wire_traffic():
    """drive()'s `stop` kwarg has no binary wire home (093-001 already made
    S reject stop=/sensor= with no motor effect on the text plane) -- the
    binary implementation preserves "no motor effect" by sending nothing."""
    fake = _UniversalLoopbackSerial()
    with _connected_proto(fake) as proto:
        result = proto.drive(200, 200, stop=["stop=t:100"])

    assert result is None
    assert fake.sent_envelopes == []
    assert fake.raw_writes == []


def test_timed_sends_segment_envelope_matching_legacy_translate():
    fake = _UniversalLoopbackSerial(ack_q=2, ack_rem=50.0)
    with _connected_proto(fake) as proto:
        result = proto.timed(200, 200, 1000)

    # handleT() transcription (legacy_translate.segment_for_timed): v=200,
    # distance = 200 * (1000/1000) = 200.
    assert result == ["OK drive l=200 r=200 ms=1000 q=2 rem=50.0"]
    sent = fake.sent_envelopes[0]
    assert sent.WhichOneof("cmd") == "segment"
    assert sent.segment.distance == pytest.approx(200.0)
    reference = envelope_pb2.CommandEnvelope(corr_id=sent.corr_id)
    reference.segment.distance = 200.0
    assert sent.SerializeToString() == reference.SerializeToString()
    _assert_wire_bytes_match(fake, 0, reference)


def test_timed_returns_empty_list_on_timeout():
    fake = _NoReplySerial()
    with _connected_proto(fake) as proto:
        result = proto.timed(200, 200, 1000)
    assert result == []


def test_distance_sends_segment_envelope_matching_legacy_translate():
    fake = _UniversalLoopbackSerial(ack_q=1, ack_rem=0.0)
    with _connected_proto(fake) as proto:
        result = proto.distance(-200, -200, 500)

    # handleD() transcription (legacy_translate.segment_for_distance): v=-200
    # (< 0) -> sign=-1, distance = -1 * 500 = -500.
    assert result == ["OK drive l=-200 r=-200 mm=500 q=1 rem=0.0"]
    sent = fake.sent_envelopes[0]
    assert sent.WhichOneof("cmd") == "segment"
    assert sent.segment.distance == pytest.approx(-500.0)
    reference = envelope_pb2.CommandEnvelope(corr_id=sent.corr_id)
    reference.segment.distance = -500.0
    assert sent.SerializeToString() == reference.SerializeToString()
    _assert_wire_bytes_match(fake, 0, reference)


def test_distance_returns_empty_list_on_timeout():
    fake = _NoReplySerial()
    with _connected_proto(fake) as proto:
        result = proto.distance(200, 200, 500)
    assert result == []


# ---------------------------------------------------------------------------
# 4. NezhaProtocol.get_config()/.set_config() (097-002) -- thin wrappers over
# get_config_binary()/set_config_binary() (096-007); reuse
# _ConfigLoopbackSerial (section 2 above) rather than inventing a new fake.
# ---------------------------------------------------------------------------


def test_set_config_single_drivetrain_key_sends_binary_and_returns_applied():
    fake = _ConfigLoopbackSerial()
    with _connected_proto(fake) as proto:
        result = proto.set_config(tw=128)

    assert result == {"tw": "128"}
    assert len(fake.sent_envelopes) == 1
    sent = fake.sent_envelopes[0]
    assert sent.WhichOneof("cmd") == "config"
    assert sent.config.WhichOneof("patch") == "drivetrain"
    assert sent.config.drivetrain.trackwidth == pytest.approx(128.0)

    reference = envelope_pb2.CommandEnvelope(
        corr_id=sent.corr_id,
        config=envelope_pb2.ConfigDelta(
            drivetrain=config_pb2.DrivetrainConfigPatch(trackwidth=128.0)))
    assert sent.SerializeToString() == reference.SerializeToString()
    _assert_wire_bytes_match(fake, 0, reference)


def test_set_config_watchdog_key_sends_binary_and_returns_applied():
    fake = _ConfigLoopbackSerial()
    with _connected_proto(fake) as proto:
        result = proto.set_config(sTimeout=500)

    assert result == {"sTimeout": "500"}
    sent = fake.sent_envelopes[0]
    assert sent.config.WhichOneof("patch") == "watchdog"
    assert sent.config.watchdog == 500


def test_set_config_motor_pid_key_applies_once_on_left_envelope():
    """pid.* is applied to BOTH bound motors server-side from ONE patch
    (handleConfigMotor(), binary_channel.cpp) -- set_config() must send only
    ONE envelope for a pid.*-only call, not two."""
    fake = _ConfigLoopbackSerial()
    with _connected_proto(fake) as proto:
        result = proto.set_config(**{"pid.kp": 1.5})

    assert result == {"pid.kp": "1.5"}
    assert len(fake.sent_envelopes) == 1
    sent = fake.sent_envelopes[0]
    assert sent.config.WhichOneof("patch") == "motor"
    assert sent.config.motor.side == config_pb2.LEFT
    assert sent.config.motor.kp == pytest.approx(1.5)


def test_set_config_ml_and_mr_together_sends_two_motor_envelopes():
    """ml/mr both patch MotorConfigPatch.travel_calib, disambiguated by
    `side` -- a single MotorConfigPatch cannot carry both at once, so this
    needs two envelopes (unlike the pid.* case above)."""
    fake = _ConfigLoopbackSerial()
    with _connected_proto(fake) as proto:
        result = proto.set_config(ml=0.487, mr=0.481)

    assert result == {"ml": "0.487", "mr": "0.481"}
    assert len(fake.sent_envelopes) == 2
    left, right = fake.sent_envelopes
    assert left.config.motor.side == config_pb2.LEFT
    assert left.config.motor.travel_calib == pytest.approx(0.487)
    assert right.config.motor.side == config_pb2.RIGHT
    assert right.config.motor.travel_calib == pytest.approx(0.481)


def test_set_config_spans_multiple_targets_sends_one_envelope_per_target():
    fake = _ConfigLoopbackSerial()
    with _connected_proto(fake) as proto:
        result = proto.set_config(tw=128, sTimeout=500)

    assert result == {"tw": "128", "sTimeout": "500"}
    assert len(fake.sent_envelopes) == 2
    patches = {e.config.WhichOneof("patch") for e in fake.sent_envelopes}
    assert patches == {"drivetrain", "watchdog"}


def test_set_config_unknown_key_returns_none_with_no_wire_traffic():
    fake = _ConfigLoopbackSerial()
    with _connected_proto(fake) as proto:
        result = proto.set_config(bogusKey=1)

    assert result is None
    assert fake.sent_envelopes == []


def test_set_config_returns_none_when_a_target_times_out():
    fake = _NoReplySerial()
    with _connected_proto(fake) as proto:
        result = proto.set_config(tw=128)
    assert result is None


def test_get_config_single_key_sends_one_envelope_and_returns_value():
    canned = envelope_pb2.ConfigSnapshot(
        target=config_pb2.CONFIG_DRIVETRAIN,
        drivetrain=config_pb2.DrivetrainConfigPatch(trackwidth=128.0, rotational_slip=0.92))
    fake = _ConfigLoopbackSerial(snapshot_by_target={config_pb2.CONFIG_DRIVETRAIN: canned})
    with _connected_proto(fake) as proto:
        result = proto.get_config("tw")

    assert result == {"tw": "128"}
    assert len(fake.sent_envelopes) == 1
    sent = fake.sent_envelopes[0]
    assert sent.WhichOneof("cmd") == "get"
    assert sent.get.target == config_pb2.CONFIG_DRIVETRAIN
    reference = envelope_pb2.CommandEnvelope(
        corr_id=sent.corr_id, get=envelope_pb2.ConfigGet(target=config_pb2.CONFIG_DRIVETRAIN))
    assert sent.SerializeToString() == reference.SerializeToString()
    _assert_wire_bytes_match(fake, 0, reference)


def test_get_config_multiple_keys_across_targets_sends_one_envelope_per_target():
    dt_snap = envelope_pb2.ConfigSnapshot(
        target=config_pb2.CONFIG_DRIVETRAIN,
        drivetrain=config_pb2.DrivetrainConfigPatch(trackwidth=128.0))
    wd_snap = envelope_pb2.ConfigSnapshot(target=config_pb2.CONFIG_WATCHDOG, watchdog=750)
    fake = _ConfigLoopbackSerial(snapshot_by_target={
        config_pb2.CONFIG_DRIVETRAIN: dt_snap,
        config_pb2.CONFIG_WATCHDOG: wd_snap,
    })
    with _connected_proto(fake) as proto:
        result = proto.get_config("tw", "sTimeout")

    assert result == {"tw": "128", "sTimeout": "750"}
    assert len(fake.sent_envelopes) == 2


def test_get_config_no_keys_dumps_all_five_targets():
    dt_snap = envelope_pb2.ConfigSnapshot(
        target=config_pb2.CONFIG_DRIVETRAIN,
        drivetrain=config_pb2.DrivetrainConfigPatch(
            trackwidth=128.0, rotational_slip=0.92, ekf_q_xy=1.0, ekf_q_theta=2.0,
            ekf_r_otos_xy=3.0, ekf_r_otos_theta=4.0))
    left_snap = envelope_pb2.ConfigSnapshot(
        target=config_pb2.CONFIG_MOTOR_LEFT,
        motor=config_pb2.MotorConfigPatch(
            side=config_pb2.LEFT, travel_calib=0.487, kp=1.5, ki=0.1, kff=0.05,
            i_max=10.0, kaw=0.2))
    right_snap = envelope_pb2.ConfigSnapshot(
        target=config_pb2.CONFIG_MOTOR_RIGHT,
        motor=config_pb2.MotorConfigPatch(side=config_pb2.RIGHT, travel_calib=0.481))
    planner_snap = envelope_pb2.ConfigSnapshot(
        target=config_pb2.CONFIG_PLANNER,
        planner=config_pb2.PlannerConfigPatch(min_speed=50.0, heading_kp=6.0, heading_kd=0.25))
    wd_snap = envelope_pb2.ConfigSnapshot(target=config_pb2.CONFIG_WATCHDOG, watchdog=500)
    fake = _ConfigLoopbackSerial(snapshot_by_target={
        config_pb2.CONFIG_DRIVETRAIN: dt_snap,
        config_pb2.CONFIG_MOTOR_LEFT: left_snap,
        config_pb2.CONFIG_MOTOR_RIGHT: right_snap,
        config_pb2.CONFIG_PLANNER: planner_snap,
        config_pb2.CONFIG_WATCHDOG: wd_snap,
    })
    with _connected_proto(fake) as proto:
        result = proto.get_config()

    assert len(fake.sent_envelopes) == 5
    assert {e.get.target for e in fake.sent_envelopes} == {
        config_pb2.CONFIG_DRIVETRAIN, config_pb2.CONFIG_MOTOR_LEFT,
        config_pb2.CONFIG_MOTOR_RIGHT, config_pb2.CONFIG_PLANNER, config_pb2.CONFIG_WATCHDOG,
    }
    assert result == {
        "tw": "128",
        "ml": "0.487",
        "mr": "0.481",
        "pid.kp": "1.5",
        "pid.ki": "0.1",
        "pid.kff": "0.05",
        "pid.iMax": "10",
        "pid.kaw": "0.2",
        "rotSlip": "0.92",
        "ekfQxy": "1",
        "ekfQtheta": "2",
        "ekfROtosXy": "3",
        "ekfROtosTheta": "4",
        "minSpeed": "50",
        "headingKp": "6",
        "headingKd": "0.25",
        "sTimeout": "500",
    }


def test_get_config_unknown_key_returns_none_with_no_wire_traffic():
    fake = _ConfigLoopbackSerial()
    with _connected_proto(fake) as proto:
        result = proto.get_config("bogusKey")

    assert result is None
    assert fake.sent_envelopes == []


# ---------------------------------------------------------------------------
# 5. NezhaProtocol telemetry conversion (097-003, M3): stream()/snap()
#
# stream(): a direct 1:1 mapping onto CommandEnvelope{stream: StreamControl{
# period, binary: true}} -- reuses _UniversalLoopbackSerial (its "stream" arm
# falls through to its default ERR_UNIMPLEMENTED reply, which stream() never
# inspects -- it discards send_envelope()'s result entirely, matching
# stop()'s "block briefly for the Ack, but return nothing" posture).
#
# snap(): synthesized from the SAME binary stream arm (architecture-update.md
# (097) Decision 4) -- arm, wait for one push frame, disarm. _StreamLoopbackSerial
# below models the firmware side closely enough to exercise this: every
# stream request gets an Ack, and an ARMING request (period != 0) ALSO
# queues one unsolicited corr_id=0 ReplyEnvelope{tlm} right behind its Ack,
# mirroring tickTelemetry()'s next-pass emission once bb.telemetryPeriod is
# set (collapsed to "immediately after the Ack" -- this fake does not model
# firmware tick timing).
# ---------------------------------------------------------------------------


class _StreamLoopbackSerial:
    """Mock transport for stream()/snap() round-trip tests.

    On write() of a `*B<base64>` CommandEnvelope, decodes it, records it,
    and replies with an Ack. If `push_frame` is given and the request is an
    ARMING `stream` request (StreamControl.period != 0), also queues ONE
    unsolicited corr_id=0 ReplyEnvelope{tlm: push_frame} right after the Ack.
    """

    is_open = True

    def __init__(self, push_frame: "telemetry_pb2.Telemetry | None" = None):
        self._pending: queue.Queue = queue.Queue()
        self.sent_envelopes: list[envelope_pb2.CommandEnvelope] = []
        self._push_frame = push_frame

    def write(self, data: bytes) -> int:
        text = data.decode("ascii").strip()
        if not text.startswith("*B"):
            return len(data)
        raw = base64.b64decode(text[2:])
        cmd = envelope_pb2.CommandEnvelope.FromString(raw)
        self.sent_envelopes.append(cmd)

        reply = envelope_pb2.ReplyEnvelope(corr_id=cmd.corr_id)
        reply.ok.q = 0
        reply.ok.rem = 0.0
        armored = "*B" + base64.b64encode(reply.SerializeToString()).decode("ascii")
        self._pending.put((armored + "\n").encode("ascii"))

        if (self._push_frame is not None and cmd.WhichOneof("cmd") == "stream"
                and cmd.stream.period != 0):
            push = envelope_pb2.ReplyEnvelope(corr_id=0)
            push.tlm.CopyFrom(self._push_frame)
            push_armored = "*B" + base64.b64encode(push.SerializeToString()).decode("ascii")
            self._pending.put((push_armored + "\n").encode("ascii"))
        return len(data)

    def flush(self) -> None:
        pass

    def readline(self) -> bytes:
        try:
            return self._pending.get(timeout=0.2)
        except queue.Empty:
            return b""


def test_stream_sends_binary_envelope_with_period_and_binary_true():
    fake = _UniversalLoopbackSerial()
    with _connected_proto(fake) as proto:
        result = proto.stream(80)

    assert result is None
    sent = fake.sent_envelopes[0]
    assert sent.WhichOneof("cmd") == "stream"
    assert sent.stream.period == 80
    assert sent.stream.binary is True
    reference = envelope_pb2.CommandEnvelope(
        corr_id=sent.corr_id,
        stream=envelope_pb2.StreamControl(period=80, binary=True))
    assert sent.SerializeToString() == reference.SerializeToString()
    _assert_wire_bytes_match(fake, 0, reference)


def test_stream_zero_disarms_with_binary_true_still_set():
    fake = _UniversalLoopbackSerial()
    with _connected_proto(fake) as proto:
        proto.stream(0)

    sent = fake.sent_envelopes[0]
    assert sent.stream.period == 0
    assert sent.stream.binary is True


def test_snap_arms_waits_for_one_frame_disarms_and_returns_tlmframe():
    telemetry = telemetry_pb2.Telemetry(
        now=555, mode=planner_pb2.IDLE, seq=2,
        has_enc=True, enc_left=10.0, enc_right=-5.0,
    )
    fake = _StreamLoopbackSerial(push_frame=telemetry)
    with _connected_proto(fake) as proto:
        result = proto.snap()

    assert result is not None
    assert result.t == 555
    assert result.enc == (10, -5)

    # arm-wait-disarm: two stream envelopes, floor period then 0, both binary.
    stream_envelopes = [e for e in fake.sent_envelopes if e.WhichOneof("cmd") == "stream"]
    assert len(stream_envelopes) == 2
    assert stream_envelopes[0].stream.period == protocol._STREAM_FLOOR_MS
    assert stream_envelopes[0].stream.binary is True
    assert stream_envelopes[1].stream.period == 0
    assert stream_envelopes[1].stream.binary is True


def test_snap_returns_none_on_timeout_and_still_disarms():
    # No push_frame configured -- every stream request Acks, but no
    # unsolicited tlm frame is ever queued, so the wait times out.
    fake = _StreamLoopbackSerial(push_frame=None)
    with _connected_proto(fake) as proto:
        result = proto.snap()

    assert result is None
    stream_envelopes = [e for e in fake.sent_envelopes if e.WhichOneof("cmd") == "stream"]
    # Arm still happened, and disarm still ran (the `finally` clause) even
    # though no frame ever arrived.
    assert len(stream_envelopes) == 2
    assert stream_envelopes[0].stream.period == protocol._STREAM_FLOOR_MS
    assert stream_envelopes[1].stream.period == 0


def test_snap_drains_stale_frames_before_arming():
    """A stale frame already sitting in _binary_tlm_queue (e.g. left over
    from a previous stream()/snap() session) must not be returned -- snap()
    drains it first, per its own documented step 1."""
    telemetry = telemetry_pb2.Telemetry(now=1, mode=planner_pb2.IDLE, seq=0)
    fake = _StreamLoopbackSerial(push_frame=None)
    with _connected_proto(fake) as proto:
        stale = envelope_pb2.ReplyEnvelope(corr_id=0)
        stale.tlm.CopyFrom(telemetry)
        proto._conn._binary_tlm_queue.put_nowait(stale)

        result = proto.snap()

    # The stale frame was drained, and no fresh one was ever queued (this
    # fake never pushes one) -- snap() must time out, not return the stale
    # frame.
    assert result is None


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
