"""src/tests/unit/test_protocol_binary_client.py — 096-007 (M6 Host Config/
Telemetry Client) + 097-002/097-003 (M2/M3 NezhaProtocol conversion),
narrowed by 104-002 (Legacy translator and dead-verb deletion).

104-002 disposition: 103-001's schema prune reserved every
``CommandEnvelope``/``ReplyEnvelope`` oneof arm this file originally
exercised EXCEPT ``config``/``stop``/``twist`` (cmd) and ``ok``/``err``/
``tlm`` (body). Concretely:

- ``TLMFrame.from_pb2()`` tests (section 1) — KEPT, fixed: ``has_cmd_vel``/
  ``acc_left``/``acc_right``/``glitch_*``/``ts_*`` moved off the primary
  ``Telemetry`` message to ``TelemetrySecondary`` (103-001) — those
  kwargs/assertions are removed; the primary-frame decode itself is still
  live and still worth covering. 115-003 frame v2 folded ``conn_left``/
  ``conn_right`` (which DID stay on the primary message) into the new
  ``flags`` bit-string as NEW ``TLMFrame`` properties — see this file's
  ``test_from_pb2_conn_left_right_derived_from_flags``.
- ``set_config_binary()``/``get_config_binary()`` tests (section 2) —
  ``set_config_binary()`` targets the live ``config`` arm (KEPT unchanged).
  ``get_config_binary()`` targeted the now-reserved ``get`` arm — DELETED
  along with the method itself (no live config READ-back path exists on
  the P4 wire; see ``protocol.py``'s own module docstring).
- ping/echo/get_id/get_ver/stop/drive/timed/distance (section 3) — every
  target arm except ``stop`` is reserved; DELETED along with the deleted
  methods. ``stop()`` itself is live but this section's
  ``test_stop_sends_binary_envelope`` only existed to round out the
  ten-method sweep via a now-dead shared fixture (``_UniversalLoopbackSerial``,
  which default-constructs a now-nonexistent ``envelope_pb2.DeviceId``) —
  ``stop()`` already has thorough, fixture-independent coverage in
  ``src/tests/unit/test_twist_stop_ack_matcher.py`` (103-009/104-001), so this
  test is DELETED as redundant rather than repaired.
- get_config()/set_config() wrapper tests (section 4) — ``get_config()``
  depends on the now-dead ``get_config_binary()``; DELETED along with it.
  ``set_config()`` targets the live ``config`` arm via ``set_config_binary()``
  and is UNCHANGED/KEPT (still passes as written — no dead arm involved).
- stream()/snap() tests (section 5) — the binary ``stream`` arm (STREAM
  control) was reserved by 103-001: telemetry is now always-on, with no
  arm/disarm step at all. ``stream()``/``stream_fields()``/``snap()``/
  ``stream_drive()`` were deleted from ``protocol.py``; this whole section
  is DELETED along with them.

116-007 (MOVE protocol cutover) disposition: ``twist`` (field 19) and
``ConfigDelta.watchdog`` (field 4) are `reserved`, not reused — the
schema-level ``move``/``config``/``stop`` oneof coverage this file's
section 2 exercised for ``config``/``watchdog`` no longer applies to
``watchdog`` at all (``ConfigDelta(watchdog=...)`` now raises
``ValueError`` at the pb2 level, since the field is gone) —
``test_set_config_binary_watchdog_arm``/``test_set_config_watchdog_key_
sends_binary_and_returns_applied`` are DELETED, and the two collateral
watchdog-adjacent tests (``test_set_config_binary_not_connected_returns_
none``, ``test_set_config_spans_multiple_targets_sends_one_envelope_per_
target``) are rewritten onto a non-watchdog ``ConfigDelta``/kwarg pair.
``test_from_pb2_mode_mapping_matches_modechar``'s ``VELOCITY`` case
(section 1) is updated: ``_DRIVE_MODE_CHAR`` now maps it to its own ``"V"``
character instead of falling back to ``"I"`` (the 115-gate-noted decode
gap, ``docs/bench-checklists/sprint-115-gut-s1.md``).

Collected under ``src/tests/unit/`` (host-side unit/tooling check, not
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
from robot_radio.robot.pb2 import common_pb2, config_pb2, envelope_pb2, telemetry_pb2
from robot_radio.robot.protocol import NezhaProtocol, TLMFrame
from robot_radio.robot._legacy_tlm_text import parse_historical_tlm_line

# ---------------------------------------------------------------------------
# 1. TLMFrame.from_pb2() -- frame v2 (115-003): nested EncoderReading/
# OtosReading readings, one `flags` bit-string, single ack_corr/ack_err
# slot. See protocol.py's own TLMFrame docstring for the full field-by-field
# rationale; this section covers the SAME primary-frame decode coverage
# 104-002 established, updated for the new wire shape.
# ---------------------------------------------------------------------------

# Fields both wire formats carry -- compared directly, field-for-field,
# between from_pb2(telemetry) and parse_historical_tlm_line(<the matching
# text line>). `cmd_vel` stays excluded -- 103-001 moved
# has_cmd_vel/cmd_vel_left/cmd_vel_right off the primary Telemetry message
# onto TelemetrySecondary, so it is no longer a field the two formats share
# at this decode layer (see TLMFrame.from_pb2()'s own docstring).
_SHARED_FIELDS = ("t", "mode", "seq", "enc", "vel", "pose", "otos", "twist")

# flags bits used directly by this test file (mirrors protocol.py's own
# module-private _FLAG_* constants -- duplicated here rather than imported
# since they are private; see telemetry.proto's own bit-table comment for
# the authoritative numbering).
_FLAG_OTOS_PRESENT = 1 << 0
_FLAG_OTOS_CONNECTED = 1 << 1
_FLAG_ACTIVE = 1 << 2
_FLAG_CONN_LEFT = 1 << 3
_FLAG_CONN_RIGHT = 1 << 4


def test_from_pb2_matches_text_parse_for_every_shared_field():
    telemetry = telemetry_pb2.Telemetry(
        now=12345,
        mode=telemetry_pb2.DISTANCE,
        seq=7,
        flags=_FLAG_OTOS_PRESENT | _FLAG_OTOS_CONNECTED,
        enc_left=telemetry_pb2.EncoderReading(position=100.0, velocity=200.0, time=12000),
        enc_right=telemetry_pb2.EncoderReading(position=-50.0, velocity=-199.0, time=12000),
        pose=common_pb2.Pose2D(x=350.0, y=-12.0, h=1.0),
        otos=telemetry_pb2.OtosReading(x=1.0, y=2.0, heading=0.5, time=12000),
        twist=common_pb2.BodyTwist3(v_x=150.0, v_y=0.0, omega=0.3),
    )

    from_pb2_frame = TLMFrame.from_pb2(telemetry)

    # The text plane's pose=/otos= tokens already carry PRE-CONVERTED
    # centidegree ints (buildTlmFrame() does the radians->cdeg conversion
    # firmware-side before formatting the line) -- so the "matching text
    # line" is built using the SAME centidegree ints from_pb2() itself
    # computed, canonicalizing both sides through the identical transform
    # before comparing (this project's established differential-test
    # posture -- src/tests/sim/unit/_wire_diff_driver.py's own f32()/float_eq()
    # precedent for exactly this kind of cross-format agreement check).
    pose_h_cdeg = from_pb2_frame.pose[2]
    otos_h_cdeg = from_pb2_frame.otos[2]
    line = (
        f"TLM t=12345 mode=D seq=7 enc=100,-50 vel=200,-199 "
        f"pose=350,-12,{pose_h_cdeg} otos=1,2,{otos_h_cdeg} twist=150,300"
    )
    text_frame = parse_historical_tlm_line(line)
    assert text_frame is not None

    for name in _SHARED_FIELDS:
        assert getattr(from_pb2_frame, name) == getattr(text_frame, name), name

    # Fields telemetry.proto/TLMFrame do NOT share stay at this dataclass's
    # own default -- see from_pb2()'s own doc comment for why each is
    # unshared. line/color additionally stay None here because their flags
    # bits (13/14) were never set on this synthetic frame.
    for name in ("wedge", "encpose", "otos_health", "ekf_rej", "line", "color", "cmd_vel"):
        assert getattr(from_pb2_frame, name) is None, name

    # otos_reading (new, richer than the legacy `otos` 3-tuple) carries the
    # SAME burst, with its own time stamp.
    assert from_pb2_frame.otos_reading is not None
    assert from_pb2_frame.otos_reading.x == pytest.approx(1.0)
    assert from_pb2_frame.otos_reading.time == 12000
    assert from_pb2_frame.otos_connected is True


def test_from_pb2_bare_frame_decodes_zero_values_not_none():
    """A bare ``Telemetry(now=1, mode=IDLE, seq=0)`` (no ``flags`` bits set)
    -- frame v2's ``enc_left``/``enc_right``/``pose``/``twist`` are ALWAYS
    present on the wire (no presence gate any more), so they decode to
    their proto3 zero values, NOT ``None``. Only ``otos``/``otos_reading``/
    ``line``/``color`` (flags-gated) and the permanent-gap fields
    (``cmd_vel``/``wedge``/``encpose``/``otos_health``/``ekf_rej``) stay
    ``None`` -- the same "no matching key=value token" shape
    ``parse_historical_tlm_line()`` produces for a line with none of those
    tokens."""
    telemetry = telemetry_pb2.Telemetry(now=1, mode=telemetry_pb2.IDLE, seq=0)

    frame = TLMFrame.from_pb2(telemetry)

    assert frame.t == 1
    assert frame.mode == "I"
    assert frame.seq == 0
    assert frame.enc == (0, 0)
    assert frame.vel == (0, 0)
    assert frame.pose == (0, 0, 0)
    assert frame.twist == (0, 0)
    assert frame.cmd_vel is None
    assert frame.otos is None
    assert frame.otos_reading is None
    assert frame.line is None
    assert frame.color is None

    text_frame = parse_historical_tlm_line("TLM t=1 mode=I seq=0")
    assert text_frame is not None
    # The text plane's absent enc=/vel=/pose=/twist= tokens stay None on
    # ITS side (parse_historical_tlm_line is a frozen, unmodified reference
    # copy) -- the two formats deliberately diverge here (always-present vs
    # gated-by-token), so this is a value comparison for t/mode/seq only,
    # not the full _SHARED_FIELDS sweep test_from_pb2_matches_text_parse_
    # for_every_shared_field() above performs against a fully-populated line.
    for name in ("t", "mode", "seq"):
        assert getattr(frame, name) == getattr(text_frame, name), name


@pytest.mark.parametrize(
    ("mode_value", "expected_char"),
    [
        (telemetry_pb2.IDLE, "I"),
        (telemetry_pb2.STREAMING, "S"),
        (telemetry_pb2.TIMED, "T"),
        (telemetry_pb2.DISTANCE, "D"),
        (telemetry_pb2.GO_TO, "G"),
        (telemetry_pb2.VELOCITY, "V"),  # 116-007: own dedicated char, no longer falls back to "I"
    ],
)
def test_from_pb2_mode_mapping_matches_modechar(mode_value, expected_char):
    telemetry = telemetry_pb2.Telemetry(now=0, mode=mode_value, seq=0)
    frame = TLMFrame.from_pb2(telemetry)
    assert frame.mode == expected_char


def test_from_pb2_drops_bench_diagnostic_fields_with_no_tlmframe_slot():
    """``acc_``/``glitch_``/``ts_`` moved off the primary ``Telemetry``
    message entirely (103-001, to ``TelemetrySecondary``) -- the primary
    message no longer even declares these fields, so there is nothing left
    to construct here; this test proves they still have no ``TLMFrame``
    slot -- ``from_pb2()`` must not invent one."""
    telemetry = telemetry_pb2.Telemetry(now=1, mode=telemetry_pb2.IDLE, seq=0)

    frame = TLMFrame.from_pb2(telemetry)

    for attr in ("acc_left", "acc_right", "glitch_left", "glitch_right",
                 "ts_left", "ts_right"):
        assert not hasattr(frame, attr), attr


@pytest.mark.parametrize(("raw_active",), [(True,), (False,)])
def test_from_pb2_populates_active_for_segment_completion_detection(raw_active):
    """097: unlike most other status signals, ``active``
    (``bb.drivetrain.busy``, flags bit 2 -- folded into ``flags`` by
    115-003, previously a standalone bool field) IS populated -- it is the
    reliable motion-complete signal (``TLMFrame.from_pb2()``'s own
    docstring). ``__main__.py``'s ``_TourRunner._wait_for_idle`` polls this
    field."""
    telemetry = telemetry_pb2.Telemetry(
        now=1, mode=telemetry_pb2.IDLE, seq=0,
        flags=_FLAG_ACTIVE if raw_active else 0,
    )
    frame = TLMFrame.from_pb2(telemetry)
    assert frame.active is raw_active


@pytest.mark.parametrize(("raw_left", "raw_right"), [(True, False), (False, True)])
def test_from_pb2_conn_left_right_derived_from_flags(raw_left, raw_right):
    """115-003: ``conn_left``/``conn_right`` (per-motor bus connectivity,
    flags bits 3/4) are NEW ``TLMFrame`` properties derived from ``flags``
    -- unlike the pre-115 wire, which carried them as standalone bool
    fields with no ``TLMFrame`` slot at all (the property that predates
    this ticket asserted the OPPOSITE -- see this test's git history /
    ``test_from_pb2_drops_bench_diagnostic_fields_with_no_tlmframe_slot``
    above for the fields that are STILL absent)."""
    flags = (_FLAG_CONN_LEFT if raw_left else 0) | (_FLAG_CONN_RIGHT if raw_right else 0)
    telemetry = telemetry_pb2.Telemetry(now=1, mode=telemetry_pb2.IDLE, seq=0, flags=flags)

    frame = TLMFrame.from_pb2(telemetry)

    assert frame.conn_left is raw_left
    assert frame.conn_right is raw_right


# ---------------------------------------------------------------------------
# 2. NezhaProtocol.set_config_binary() -- the live half of 096-007's binary
# config client (get_config_binary() targeted the now-reserved `get` arm --
# deleted by 104-002, see this file's own header note).
# ---------------------------------------------------------------------------


class _ConfigLoopbackSerial:
    """Mock transport for the binary config set round-trip tests.

    On write() of a `*B<base64>` CommandEnvelope, decodes it, records it
    (``sent_envelopes``), and synthesizes an Ack reply -- mirroring
    BinaryChannel's CONFIG arm (src/firm/commands/binary_channel.cpp) closely
    enough to exercise NezhaProtocol's full envelope round trip with no
    real serial port.
    """

    is_open = True

    def __init__(self) -> None:
        self._pending: queue.Queue = queue.Queue()
        self.sent_envelopes: list[envelope_pb2.CommandEnvelope] = []
        # 097-002: raw armored lines actually written, for tests that assert
        # the literal wire bytes (not just the decoded envelope).
        self.raw_writes: list[bytes] = []

    def write(self, data: bytes) -> int:
        self.raw_writes.append(data)
        text = data.decode("ascii").strip()
        if text.startswith("*B"):
            raw = base64.b64decode(text[2:])
            cmd = envelope_pb2.CommandEnvelope.FromString(raw)
            self.sent_envelopes.append(cmd)

            reply = envelope_pb2.ReplyEnvelope(corr_id=cmd.corr_id)
            reply.ok.q = 1
            reply.ok.rem = 0.0
            reply.ok.t = 999
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


def test_set_config_binary_returns_none_on_timeout():
    conn = SerialConnection()
    conn._ser = _NoReplySerial()
    conn._start_reader()
    try:
        proto = NezhaProtocol(conn)
        ack = proto.set_config_binary(
            envelope_pb2.ConfigDelta(
                drivetrain=config_pb2.DrivetrainConfigPatch(trackwidth=128.0)),
            read_timeout=50)
    finally:
        conn._stop_reader()

    assert ack is None


def test_set_config_binary_not_connected_returns_none():
    conn = SerialConnection()  # _ser stays None -- never connected
    proto = NezhaProtocol(conn)

    ack = proto.set_config_binary(
        envelope_pb2.ConfigDelta(drivetrain=config_pb2.DrivetrainConfigPatch(trackwidth=128.0)),
        read_timeout=50)

    assert ack is None


# ---------------------------------------------------------------------------
# 3. NezhaProtocol.set_config() (097-002) -- thin wrapper over
# set_config_binary() (096-007); reuses _ConfigLoopbackSerial (section 2
# above). get_config()/get_config_binary() were DELETED by 104-002 (no live
# config READ-back arm) -- see this file's own header note.
# ---------------------------------------------------------------------------


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
    """tw= (drivetrain) and pid.kp= (motor) are DIFFERENT ConfigDelta.patch
    targets -- set_config() must fan them out into two round trips, one per
    target (unlike config()'s stricter single-target-per-call contract)."""
    fake = _ConfigLoopbackSerial()
    with _connected_proto(fake) as proto:
        result = proto.set_config(tw=128, **{"pid.kp": 1.5})

    assert result == {"tw": "128", "pid.kp": "1.5"}
    assert len(fake.sent_envelopes) == 2
    patches = {e.config.WhichOneof("patch") for e in fake.sent_envelopes}
    assert patches == {"drivetrain", "motor"}


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


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
