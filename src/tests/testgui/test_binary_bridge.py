"""src/tests/testgui/test_binary_bridge.py — 128-004: rewritten for
``binary_bridge.py``'s gutted shape (the dead ``_LEGACY_TRANSLATION_AVAILABLE``
gate and everything behind it are deleted; only ``OI``/``OL``/``OA``/``SET``/
``STREAM`` still have a live binary-plane translation, each a direct call to
a live ``NezhaProtocol`` method).

Qt-free — no QApplication, no sim lib, no PySide6 required. Exercises
``robot_radio.testgui.binary_bridge`` directly against a fake connection
double (mirrors ``src/tests/unit/test_bridge_routing.py``'s ``_FakeConn``
pattern for ``io/proxy.py``'s ProtocolBridge — this file is the TestGUI
bridge's own equivalent, a module that test file's own fixtures do not
reach).

What changed at 128-004
------------------------
``legacy_render``/``legacy_verbs`` (deleted wholesale by commit ``129cbcb3``,
104-002) are no longer imported or referenced at all -- there is no
``_LEGACY_TRANSLATION_AVAILABLE`` gate, no ``render``/``legacy_verbs`` module
attribute, and no ``_LEGACY_UNAVAILABLE_REPLY`` constant. Every verb without
a live direct-call translation (``S``/``T``/``D``/``R``/``TURN``/``RT``/
``G``, ``GET``, ``SNAP``, ``SI``/``ZERO``/``OZ``, ``GRIP``/``QLEN``, ``OV``/
``OP``/``OR``, and anything unrecognized) now returns a generic
``ERR unsupported <verb>`` reply built inline -- same "parsed, never sent,
replies ERR" outcome as before, just without a shared fixed-string constant
to assert equality against (tests below assert the ``ERR``/no-wire-call
SHAPE, not an exact string). ``STREAM`` is NEW as a live direct call
(``_handle_stream_patch()`` -> ``NezhaProtocol.tlmOn()``/``tlmOff()``,
per ticket 003's own note) -- it moves out of the "always unsupported"
parametrized test into its own section.

``envelope_pb2``'s own schema is unchanged by this ticket: ``ReplyEnvelope``'s
``body`` oneof is ``{ok, err, tlm}`` (``id``/``echo``/``helptext`` do not
exist as fields), and ``CommandEnvelope``'s ``cmd`` oneof is ``{config, stop,
move}`` plus ``wheels``/``estop`` (``drive``/``segment``/``replace``/the
interim ``twist`` arm are all gone). ``render_log_line()``'s rendering is
unconditional ``text_format`` now (128-004 deleted the dead
``if render is not None`` branch it never actually took), so the schema
lock-in test below still matters for the same reason it always did.

Run with::

    QT_QPA_PLATFORM=offscreen uv run pytest src/tests/testgui/test_binary_bridge.py -q

(``QT_QPA_PLATFORM`` is harmless here — nothing in this file touches Qt —
set for consistency with the rest of the ``tests/testgui`` suite's run
command.)

Collected under ``src/tests/testgui/`` per ``pyproject.toml``'s ``testpaths``
(107-004 re-added the directory — dropped at sprint 102 ticket 005).
"""

from __future__ import annotations

import pytest

from robot_radio.io.wire_codec import encode_frame

from robot_radio.robot.pb2 import config_pb2, envelope_pb2
from robot_radio.robot.protocol import NezhaProtocol
from robot_radio.testgui import binary_bridge

# _FakeConn.wait_for_ack() (below) returns a raw packed `int` ring entry
# directly (124-008, issue §B4: `corr_id<<4|err` -- `telemetry_pb2.AckEntry`
# is deleted, `Telemetry.acks` is `repeated uint32`) -- NOT the whole
# Telemetry frame it rode in on -- NezhaProtocol.wait_for_ack() adapts it
# via AckEntry.from_ring_entry(), which unpacks corr_id/err off it with no
# freshness gate needed (see that method's own docstring).


def _pack_ack(corr_id: int, err: int) -> int:
    """Mirrors telemetry.cpp's own pushAckRing() packed-word format EXACTLY
    (124-008, issue §B4): corr_id<<4 | err."""
    return (corr_id << 4) | err


# ---------------------------------------------------------------------------
# Fake connection double — SimConnection-shaped send_envelope() (returns the
# decoded ReplyEnvelope directly, not the SerialConnection dict wrapper; see
# NezhaProtocol._send_envelope()'s own docstring for why both shapes exist).
# ---------------------------------------------------------------------------


class _FakeConn:
    def __init__(self) -> None:
        self.envelope_calls: list[envelope_pb2.CommandEnvelope] = []
        self._reply_queue: list["envelope_pb2.ReplyEnvelope | None"] = []
        self._next_corr_id = 0
        # otos_config() uses send_envelope_fast() + wait_for_ack() (the
        # SAME fire-and-poll shape move_twist()/move_wheels()/stop()/
        # config() use), NOT send_envelope() -- see NezhaProtocol.otos_config()'s own
        # docstring. ack_result scripts wait_for_ack()'s return value,
        # mirroring test_protocol_config.py's _FakeFastConn.
        self.ack_result: "object | None" = None

    def queue_reply(self, reply: "envelope_pb2.ReplyEnvelope | None") -> None:
        self._reply_queue.append(reply)

    def send_envelope(self, envelope: envelope_pb2.CommandEnvelope,
                      read_timeout: int = 500) -> "envelope_pb2.ReplyEnvelope | None":
        self.envelope_calls.append(envelope)
        return self._reply_queue.pop(0) if self._reply_queue else None

    def send_envelope_fast(self, envelope: "envelope_pb2.CommandEnvelope") -> int:
        self._next_corr_id += 1
        envelope.corr_id = self._next_corr_id
        self.envelope_calls.append(envelope)
        return self._next_corr_id

    def wait_for_ack(self, corr_id: int, timeout: int = 500):
        return self.ack_result

    def drain_binary_tlm(self) -> list:
        return []


@pytest.fixture
def proto():
    conn = _FakeConn()
    return NezhaProtocol(conn), conn


# ---------------------------------------------------------------------------
# translate_command() — verbs with no live binary-plane translation: a
# generic "ERR unsupported <verb>" reply, no wire call, no ack wait.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("line", [
    "S 200 200",
    "D 200 200 500",
    "R 200 500",
    "TURN 9000 eps=300",
    "G 300 400 150",
    "GET rotSlip",
    "SNAP",
    "GRIP",
    "QLEN",
    "SI 0 0 0",
    "ZERO enc",
    "OZ",
    "OP",
    "BOGUSVERB 1 2 3",
])
def test_unsupported_verbs_get_a_generic_err_reply_no_wire_call(proto, line):
    """No verb-specific dispatch exists for any of these — every one gets a
    generic ``ERR unsupported <verb>`` reply and NOTHING is sent on the wire
    (see ``translate_command()``'s own docstring for the full unsupported
    list and why)."""
    nezha, conn = proto

    reply_line = binary_bridge.translate_command(nezha, line)

    verb = line.split()[0]
    assert reply_line.startswith(f"ERR unsupported {verb}")
    assert conn.envelope_calls == []


def test_empty_line_returns_empty_string_no_wire_call(proto):
    nezha, conn = proto

    for line in ("", "   ", "\t\n"):
        assert binary_bridge.translate_command(nezha, line) == ""
    assert conn.envelope_calls == []


# ---------------------------------------------------------------------------
# 109-004: OL/OA/OI direct-patch-send -- intercepted BEFORE the launch-
# unblock short-circuit above. Stakeholder bench fix (2026-07-22) adds SET
# to this same intercept (_handle_set_patch(), calling NezhaProtocol.
# set_config() directly) -- SET was found rejecting every push against real
# hardware (calibration push logged "'SET pid.kaw=0' rejected: ERR
# unavailable legacy verb translation removed", 1/12 values applied). These
# four verbs (OL/OA/OI/SET) are now the ONLY ones that still build and send
# a real envelope through translate_command().
# ---------------------------------------------------------------------------


def test_ol_sends_otos_config_patch_with_linear_scale(proto):
    nezha, conn = proto
    conn.ack_result = _pack_ack(1, 0)

    reply = binary_bridge.translate_command(nezha, "OL 1.05")

    assert reply == "OK ol"
    assert len(conn.envelope_calls) == 1
    sent = conn.envelope_calls[0]
    assert sent.WhichOneof("cmd") == "config"
    assert sent.config.WhichOneof("patch") == "otos"
    assert sent.config.otos.linear_scale == pytest.approx(1.05)


def test_oa_sends_otos_config_patch_with_angular_scale(proto):
    nezha, conn = proto
    conn.ack_result = _pack_ack(1, 0)

    reply = binary_bridge.translate_command(nezha, "OA -0.98")

    assert reply == "OK oa"
    sent = conn.envelope_calls[0]
    assert sent.config.otos.angular_scale == pytest.approx(-0.98)


def test_oi_sends_otos_config_patch_with_init_trigger(proto):
    nezha, conn = proto
    conn.ack_result = _pack_ack(1, 0)

    reply = binary_bridge.translate_command(nezha, "OI")

    assert reply == "OK oi"
    sent = conn.envelope_calls[0]
    assert sent.config.otos.init is True


def test_ol_with_no_scale_is_badarg_no_wire_call(proto):
    nezha, conn = proto

    reply = binary_bridge.translate_command(nezha, "OL")

    assert reply.startswith("ERR badarg")
    assert conn.envelope_calls == []


def test_oa_with_no_scale_is_badarg_no_wire_call(proto):
    nezha, conn = proto

    reply = binary_bridge.translate_command(nezha, "OA")

    assert reply.startswith("ERR badarg")
    assert conn.envelope_calls == []


def test_ol_with_non_numeric_scale_is_badarg_no_wire_call(proto):
    nezha, conn = proto

    reply = binary_bridge.translate_command(nezha, "OL notanumber")

    assert reply.startswith("ERR badarg")
    assert conn.envelope_calls == []


def test_ol_ack_timeout_renders_err(proto):
    nezha, conn = proto
    conn.ack_result = None  # no matching ack ever arrives

    reply = binary_bridge.translate_command(nezha, "OL 1.05")

    assert reply.startswith("ERR unknown")
    assert len(conn.envelope_calls) == 1  # the envelope was still sent


def test_ol_nak_ack_renders_err(proto):
    nezha, conn = proto
    conn.ack_result = _pack_ack(1, envelope_pb2.ERR_UNIMPLEMENTED)

    reply = binary_bridge.translate_command(nezha, "OL 1.05")

    assert reply.startswith("ERR nak")


def test_ov_op_or_still_render_unsupported_reply_unchanged(proto):
    """OV/OP/OR have no direct-patch-send equivalent -- they fall through to
    the SAME generic unsupported reply every other non-OL/OA/OI/SET/STREAM
    verb hits (no envelope sent)."""
    nezha, conn = proto

    for verb in ("OV 0 0 0", "OP", "OR"):
        reply = binary_bridge.translate_command(nezha, verb)
        assert reply.startswith(f"ERR unsupported {verb.split()[0]}")

    assert conn.envelope_calls == []


# ---------------------------------------------------------------------------
# STREAM direct call (128-004, per ticket 003's own note) -- calls
# NezhaProtocol.tlmOn()/tlmOff() directly. tlmOn()/tlmOff() are fire-and-
# forget send_fast() wrappers (no ack), so these tests assert on the fake
# connection's own send_fast()-observable call, not envelope_calls.
# ---------------------------------------------------------------------------


class _FakeConnStream(_FakeConn):
    """Extends ``_FakeConn`` with the ``send()``/``send_fast()`` cleartext
    surface ``NezhaProtocol.tlmOn()``/``tlmOff()`` actually call (a plain
    text-plane line, not a binary envelope)."""

    def __init__(self) -> None:
        super().__init__()
        self.fast_sends: list[str] = []

    def send_fast(self, cmd: str) -> None:
        self.fast_sends.append(cmd)


@pytest.fixture
def stream_proto():
    conn = _FakeConnStream()
    return NezhaProtocol(conn), conn


def test_stream_nonzero_period_calls_tlm_on(stream_proto):
    nezha, conn = stream_proto

    reply = binary_bridge.translate_command(nezha, "STREAM 50")

    assert reply == "OK stream period=50"
    assert conn.fast_sends == ["TLM:ON"]
    assert conn.envelope_calls == []


def test_stream_zero_period_calls_tlm_off(stream_proto):
    nezha, conn = stream_proto

    reply = binary_bridge.translate_command(nezha, "STREAM 0")

    assert reply == "OK stream period=0"
    assert conn.fast_sends == ["TLM:OFF"]


def test_stream_negative_period_calls_tlm_off(stream_proto):
    nezha, conn = stream_proto

    reply = binary_bridge.translate_command(nezha, "STREAM -5")

    assert reply == "OK stream period=0"
    assert conn.fast_sends == ["TLM:OFF"]


def test_stream_no_period_is_badarg_no_wire_call(stream_proto):
    nezha, conn = stream_proto

    reply = binary_bridge.translate_command(nezha, "STREAM")

    assert reply == "ERR badarg period"
    assert conn.fast_sends == []


def test_stream_non_numeric_period_is_badarg_no_wire_call(stream_proto):
    nezha, conn = stream_proto

    reply = binary_bridge.translate_command(nezha, "STREAM soon")

    assert reply == "ERR badarg period"
    assert conn.fast_sends == []


# ---------------------------------------------------------------------------
# SET direct-patch-send (stakeholder bench fix, 2026-07-22) -- calls
# NezhaProtocol.set_config(), which itself calls set_config_binary().
#
# set_config_binary() itself was ALSO fixed this session (protocol.py):
# it used to send via the BLOCKING _send_envelope()/conn.send_envelope()
# and look for a synchronous ReplyEnvelope{ok:...} -- which the current
# single-loop firmware never sends for ANY command (docs/protocol-v4.md
# sec 7.1). Confirmed on the real bench: with only binary_bridge.py's SET
# routing fixed (this file's own first round of tests), calibration push
# against real hardware still showed 9/12 keys "ERR badarg set failed" --
# set_config_binary() itself was timing out on every call. Fixed to send
# via send_envelope_fast() + wait_for_ack() (the ack-ring poll), matching
# move_twist()/stop()/config()/otos_config() -- so these tests now script
# conn.ack_result (the wait_for_ack() return value), the SAME pattern the
# OI/OL/OA tests above use, not conn.queue_reply() (send_envelope()'s own
# synchronous reply queue, no longer read by this call path at all).
# ---------------------------------------------------------------------------


def test_set_motor_pid_keys_send_one_left_side_motor_patch(proto):
    nezha, conn = proto
    conn.ack_result = _pack_ack(1, 0)

    reply = binary_bridge.translate_command(nezha, "SET pid.kp=1.5 pid.kaw=20")

    assert reply == "OK set pid.kp=1.5 pid.kaw=20"
    assert len(conn.envelope_calls) == 1
    sent = conn.envelope_calls[0]
    assert sent.WhichOneof("cmd") == "config"
    assert sent.config.WhichOneof("patch") == "motor"
    assert sent.config.motor.side == 0  # LEFT -- both PID keys land on ONE envelope
    assert sent.config.motor.kp == pytest.approx(1.5)
    assert sent.config.motor.kaw == pytest.approx(20)


def test_set_drivetrain_keys_send_drivetrain_patch(proto):
    nezha, conn = proto
    conn.ack_result = _pack_ack(1, 0)

    reply = binary_bridge.translate_command(nezha, "SET tw=128 rotSlip=0.92")

    assert reply == "OK set tw=128 rotSlip=0.92"
    assert len(conn.envelope_calls) == 1
    sent = conn.envelope_calls[0]
    assert sent.config.WhichOneof("patch") == "drivetrain"
    assert sent.config.drivetrain.trackwidth == pytest.approx(128)
    assert sent.config.drivetrain.rotational_slip == pytest.approx(0.92)


def test_set_ml_mr_send_two_separate_motor_side_patches(proto):
    nezha, conn = proto
    conn.ack_result = _pack_ack(1, 0)

    reply = binary_bridge.translate_command(nezha, "SET ml=0.523599 mr=0.523599")

    assert reply.startswith("OK set")
    assert len(conn.envelope_calls) == 2
    sides = {env.config.motor.side for env in conn.envelope_calls}
    assert sides == {config_pb2.LEFT, config_pb2.RIGHT}


def test_set_unknown_key_is_badkey_no_wire_call(proto):
    nezha, conn = proto

    reply = binary_bridge.translate_command(nezha, "SET bogusKey=1")

    assert reply == "ERR badkey bogusKey"
    assert conn.envelope_calls == []


def test_set_malformed_token_is_badarg_no_wire_call(proto):
    nezha, conn = proto

    reply = binary_bridge.translate_command(nezha, "SET notkeyvalue")

    assert reply.startswith("ERR badarg")
    assert conn.envelope_calls == []


def test_set_non_numeric_value_is_badarg_no_wire_call(proto):
    nezha, conn = proto

    reply = binary_bridge.translate_command(nezha, "SET tw=notanumber")

    assert reply.startswith("ERR badarg")
    assert conn.envelope_calls == []


def test_set_with_no_kv_pairs_is_badarg_no_wire_call(proto):
    nezha, conn = proto

    reply = binary_bridge.translate_command(nezha, "SET")

    assert reply == "ERR badarg no key=value pairs"
    assert conn.envelope_calls == []


def test_set_nak_ack_renders_set_failed(proto):
    nezha, conn = proto
    conn.ack_result = _pack_ack(1, envelope_pb2.ERR_BADARG)

    reply = binary_bridge.translate_command(nezha, "SET tw=128")

    assert reply == "ERR badarg set failed"
    assert len(conn.envelope_calls) == 1  # the envelope was still sent


def test_set_ack_timeout_renders_set_failed(proto):
    nezha, conn = proto
    conn.ack_result = None  # no matching completion ack ever arrives

    reply = binary_bridge.translate_command(nezha, "SET tw=128")

    assert reply == "ERR badarg set failed"
    assert len(conn.envelope_calls) == 1  # the envelope was still sent


# ---------------------------------------------------------------------------
# envelope_pb2 schema — locks in the shrunk oneofs render_log_line()'s
# id/echo/helptext branches now depend on being unreachable for (see module
# docstring, point 2).
# ---------------------------------------------------------------------------


def test_reply_oneof_no_longer_has_id_echo_helptext():
    fields = envelope_pb2.ReplyEnvelope.DESCRIPTOR.oneofs_by_name["body"].fields
    assert {f.name for f in fields} == {"ok", "err", "tlm"}


@pytest.mark.skip(reason="DEPRECATED-COMMAND-INGEST -- the cmd oneof gained "
                        "wheels(22)/estop(23); TestGUI is out of scope for that "
                        "change, so this assertion is quarantined for the deferred "
                        "big-bang test pass rather than re-derived here")
def test_command_oneof_no_longer_has_drive_segment_replace():
    fields = envelope_pb2.CommandEnvelope.DESCRIPTOR.oneofs_by_name["cmd"].fields
    # 109-003's `move` (CmdKind::MOVE) was itself DELETED (115-003, gut S1
    # motion-stack excision) -- field 20 is `reserved`, not an active oneof
    # arm any more (see envelope.proto's own CommandEnvelope header
    # comment). 116-001's MOVE-protocol cutover reintroduced a `Move`-named
    # arm at a FRESH number (21), never 20 -- and 116-001 also deleted the
    # interim `twist` arm (field 19, 103-001) it supersedes, so the live
    # `cmd` oneof is now `config`/`stop`/`move`.
    assert {f.name for f in fields} == {"config", "stop", "move"}


# ---------------------------------------------------------------------------
# render_log_line() — the serial/message monitor filter (Goal 4, 097).
# ---------------------------------------------------------------------------


def _armor(msg, command: bytes) -> bytes:
    """Build the FULL `<COMMAND>':'<COBS+CRC bytes>` wire line (124-005;
    was a bare COBS+CRC frame body 123-002/003, ``*B`` + base64 pre-123) --
    ``render_log_line()`` now dispatches ``bytes`` input by splitting off
    this SAME leading prefix itself (to scope ``decode_frame()``'s CRC
    check correctly), then still by Python TYPE at the top level (``bytes``
    for a binary line, ``str`` for a cleartext-plane line)."""
    return command + b":" + encode_frame(msg.SerializeToString(), command=command)


def test_tlm_reply_is_dropped_entirely():
    reply = envelope_pb2.ReplyEnvelope()
    reply.tlm.now = 12345
    assert binary_bridge.render_log_line(_armor(reply, b"TLM"), outbound=False) is None


def test_err_reply_falls_back_to_text_format_rendering():
    """``render`` is ``None`` (launch-unblock) — the ``err`` branch is
    unreachable (``if render is not None:`` guards it), so every reply,
    including ``err``, takes the ``text_format`` fallback: readable, not raw
    armor, but no longer the old ``legacy_render``-specific
    ``"ERR badarg #4"`` shape."""
    reply = envelope_pb2.ReplyEnvelope()
    reply.corr_id = 4
    reply.err.code = envelope_pb2.ERR_BADARG
    rendered = binary_bridge.render_log_line(_armor(reply, b"ERR"), outbound=False)
    assert rendered is not None
    assert not rendered.startswith("*B")
    assert "ERR_BADARG" in rendered
    assert "4" in rendered


def test_ok_reply_renders_readable_text_not_raw_armor():
    reply = envelope_pb2.ReplyEnvelope()
    reply.ok.q = 3
    reply.ok.rem = 45.0
    rendered = binary_bridge.render_log_line(_armor(reply, b"OK"), outbound=False)
    assert rendered is not None
    assert not rendered.startswith("*B")
    assert "3" in rendered


def test_outbound_command_renders_readable_text_not_raw_armor():
    """``CommandEnvelope`` never had a ``legacy_render`` equivalent at all
    (that module renders replies, not requests) — always ``text_format``,
    launch-unblock or not. Built from the ``move`` oneof arm's ``twist``
    velocity variant (``drive`` no longer exists, and the interim bare
    ``twist`` arm is itself gone since 116-001 — see
    ``test_command_oneof_no_longer_has_drive_segment_replace``)."""
    cmd = envelope_pb2.CommandEnvelope()
    cmd.corr_id = 9
    cmd.move.twist.v_x = 200
    cmd.move.twist.omega = -1.5
    rendered = binary_bridge.render_log_line(_armor(cmd, b"MOVE"), outbound=True)
    assert rendered is not None
    assert not rendered.startswith("*B")
    assert "200" in rendered


def test_text_plane_line_passes_through_unchanged():
    """123-002/003: dispatch is now by Python TYPE -- any ``str`` (a
    text-plane line) passes through unchanged, regardless of content."""
    line = "DEVICE:NEZHA2:robot:tovez:123"
    assert binary_bridge.render_log_line(line, outbound=True) == line
    assert binary_bridge.render_log_line(line, outbound=False) == line


def test_malformed_binary_frame_renders_marker_never_raises():
    """A ``bytes`` input that fails to COBS/CRC-decode renders a defensive
    malformed-marker string (never raises, never silently passes the raw
    bytes through as if they were text)."""
    garbage = b"not-a-valid-cobs-crc-frame-body"
    for outbound in (True, False):
        rendered = binary_bridge.render_log_line(garbage, outbound=outbound)
        assert rendered is not None
        assert "malformed" in rendered


# ---------------------------------------------------------------------------
# render_log_line() — empty-body-oneof frames render as malformed, not a
# bogus "corr_id: N" line (124-009: the ReplyEnvelope-vs-TelemetrySecondary
# disambiguation this section used to test is DELETED along with
# TelemetrySecondary itself, robot-state-blackboard-...md — there is no
# second shape left to retry a body-less parse against, so it is simply
# malformed now). Historical context (kept for why this bit of behavior
# exists at all): a bare TelemetrySecondary frame used to "successfully"
# parse as a ReplyEnvelope with an EMPTY body oneof (TelemetrySecondary's
# first field, `now`, and ReplyEnvelope's first field, `corr_id`, are both
# wire type 13/uint32), flooding the message monitor with bare "corr_id: N"
# lines at the secondary frame's own ~4 lines/s -- the reason
# render_log_line() checks WhichOneof("body") at all rather than trusting
# a successful parse alone.
# ---------------------------------------------------------------------------


def test_empty_body_oneof_frame_renders_as_malformed_not_a_bare_corr_id_line():
    """A frame that parses as a ReplyEnvelope with no body oneof set (the
    exact shape a stray corr_id-only frame produces) is malformed, not a
    silently-rendered bare "corr_id: N" line -- see this section's own
    historical-context comment for why that distinction matters."""
    reply = envelope_pb2.ReplyEnvelope()
    reply.corr_id = 123456
    # Deliberately NOT setting ok/err/tlm -- WhichOneof("body") stays None.

    rendered = binary_bridge.render_log_line(_armor(reply, b"TLM"), outbound=False)

    assert rendered is not None
    assert "malformed" in rendered


def test_reply_envelope_with_set_body_still_renders_not_dropped():
    """Regression guard for the check above: a REAL ReplyEnvelope (body
    oneof actually set) must still render normally, not get swept into the
    malformed-marker path."""
    reply = envelope_pb2.ReplyEnvelope()
    reply.corr_id = 7
    reply.ok.q = 1
    rendered = binary_bridge.render_log_line(_armor(reply, b"OK"), outbound=False)
    assert rendered is not None
    assert not rendered.startswith("*B")


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
