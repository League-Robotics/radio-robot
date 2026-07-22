"""src/tests/testgui/test_binary_bridge.py — 107-004: rewritten for the current
"launch-unblock" degraded-mode reality (was written for 097/100-007's fully
wired R/TURN/G translation, which no longer exists on this wire).

Qt-free — no QApplication, no sim lib, no PySide6 required. Exercises
``robot_radio.testgui.binary_bridge`` directly against a fake connection
double (mirrors ``src/tests/unit/test_bridge_routing.py``'s ``_FakeConn``
pattern for ``io/proxy.py``'s ProtocolBridge — this file is the TestGUI
bridge's own equivalent, a module that test file's own fixtures do not
reach).

What changed from the 097/100-007 version of this file
--------------------------------------------------------
This file (dropped from ``pyproject.toml``'s ``testpaths`` at sprint 102
ticket 005 along with the rest of ``src/tests/testgui/``) went stale against TWO
independent, later changes it never saw:

1. ``binary_bridge.py``'s own "107-003 launch-unblock" (this module's own
   header): ``legacy_render``/``legacy_verbs`` were deleted wholesale by
   commit ``129cbcb3`` (104-002) with no replacement. ``translate_command()``
   now short-circuits EVERY non-empty line to a single fixed
   ``_LEGACY_UNAVAILABLE_REPLY`` string — no envelope is ever built or sent,
   regardless of verb. The old ``R``/``TURN``/``G``-un-gating assertions
   (each expecting a real ``segment`` envelope on the wire) test dead code
   that can no longer be reached; ``GRIP``/``QLEN``/pose-reset/OTOS-device
   verbs used to render distinguishable ``"unsupported"``/``"nodev"`` codes,
   but now render the SAME fixed unavailable-reply string as everything
   else, since dispatch never gets far enough to distinguish them.
2. ``envelope_pb2``'s own schema shrank independently, underneath
   ``binary_bridge.py``: ``ReplyEnvelope``'s ``body`` oneof is down to
   exactly ``{ok, err, tlm}`` (``id``/``echo``/``helptext`` no longer
   exist as fields at all — constructing ``reply.id``/``reply.echo`` now
   raises ``AttributeError``), and ``CommandEnvelope``'s ``cmd`` oneof is
   down to ``{config, stop, move}`` (``drive``/``segment``/``replace`` are
   gone; 116-001's MOVE protocol cutover later swapped the interim
   ``twist`` arm this note originally described for ``move`` — see
   ``test_command_oneof_no_longer_has_drive_segment_replace``).
   ``render_log_line()``'s ``id``/``echo``/``helptext`` branches were
   already unreachable for a second, independent reason even before
   accounting for (1) above (``render`` being ``None``): those oneof arms
   cannot be constructed any more, so nothing can ever set ``which`` to
   those values. This file now builds replies only from the oneof arms that
   still exist.

Both facts are locked in below (``test_legacy_translation_is_unavailable``,
``test_reply_oneof_no_longer_has_id_echo_helptext``) so a future restoration
of either is a deliberate, visible test change — not a silent regression
nobody notices a second time (see
``clasi/issues/binary-bridge-segment-replace-arms-deleted.md``, referenced
by this module's own docstring, for the filed follow-up).

Run with::

    QT_QPA_PLATFORM=offscreen uv run pytest src/tests/testgui/test_binary_bridge.py -q

(``QT_QPA_PLATFORM`` is harmless here — nothing in this file touches Qt —
set for consistency with the rest of the ``tests/testgui`` suite's run
command.)

Collected under ``src/tests/testgui/`` per ``pyproject.toml``'s ``testpaths``
(107-004 re-added the directory — dropped at sprint 102 ticket 005).
"""

from __future__ import annotations

import base64

import pytest

from robot_radio.robot.pb2 import envelope_pb2, telemetry_pb2
from robot_radio.robot.protocol import NezhaProtocol
from robot_radio.testgui import binary_bridge

# flags bit 5 -- telemetry.proto Telemetry.flags (ack_fresh, 115-003 frame
# v2). _FakeConn.wait_for_ack() (below) returns a raw telemetry_pb2.Telemetry
# directly (NOT the deleted AckEntry pb2 message -- the depth-3 ack ring/
# AckEntry/AckStatus wire types are gone) -- NezhaProtocol.wait_for_ack()
# adapts it via AckEntry.from_telemetry(), which reads ack_corr/ack_err
# gated on this bit.
_ACK_FRESH_BIT = 1 << 5


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
# translate_command() — launch-unblock degraded mode (see module docstring):
# every non-empty verb short-circuits to the same fixed reply, no wire call.
# ---------------------------------------------------------------------------


def test_legacy_translation_is_unavailable():
    """Locks in the launch-unblock precondition this whole file tests against
    — if a future sprint restores ``legacy_render``/``legacy_verbs``, this
    assertion fails LOUDLY (not the silent-drift this file itself is a
    correction for), pointing straight at this file needing a rewrite back
    toward the pre-launch-unblock behavior."""
    assert binary_bridge._LEGACY_TRANSLATION_AVAILABLE is False
    assert binary_bridge.render is None
    assert binary_bridge.legacy_verbs is None


@pytest.mark.parametrize("line", [
    "S 200 200",
    "D 200 200 500",
    "R 200 500",
    "TURN 9000 eps=300",
    "G 300 400 150",
    "SET rotSlip=0",
    "GET rotSlip",
    "STREAM 50",
    "SNAP",
    "GRIP",
    "QLEN",
    "SI 0 0 0",
    "ZERO enc",
    "OZ",
    "OP",
    "BOGUSVERB 1 2 3",
])
def test_every_verb_short_circuits_to_the_fixed_unavailable_reply(proto, line):
    """No verb-specific dispatch survives the launch-unblock guard — every
    non-empty line, supported-looking or not, gets the SAME fixed reply and
    NOTHING is sent on the wire (parsing the line at all is itself
    ``legacy_verbs``' job — see ``translate_command()``'s own docstring)."""
    nezha, conn = proto

    reply_line = binary_bridge.translate_command(nezha, line)

    assert reply_line == binary_bridge._LEGACY_UNAVAILABLE_REPLY
    assert conn.envelope_calls == []


def test_empty_line_returns_empty_string_no_wire_call(proto):
    nezha, conn = proto

    for line in ("", "   ", "\t\n"):
        assert binary_bridge.translate_command(nezha, line) == ""
    assert conn.envelope_calls == []


# ---------------------------------------------------------------------------
# 109-004: OL/OA/OI direct-patch-send -- intercepted BEFORE the launch-
# unblock short-circuit above, so these three verbs are the ONLY ones that
# still build and send a real envelope through translate_command().
# ---------------------------------------------------------------------------


def test_ol_sends_otos_config_patch_with_linear_scale(proto):
    nezha, conn = proto
    conn.ack_result = telemetry_pb2.Telemetry(
        flags=_ACK_FRESH_BIT, ack_corr=1, ack_err=0)

    reply = binary_bridge.translate_command(nezha, "OL 1.05")

    assert reply == "OK ol"
    assert len(conn.envelope_calls) == 1
    sent = conn.envelope_calls[0]
    assert sent.WhichOneof("cmd") == "config"
    assert sent.config.WhichOneof("patch") == "otos"
    assert sent.config.otos.linear_scale == pytest.approx(1.05)


def test_oa_sends_otos_config_patch_with_angular_scale(proto):
    nezha, conn = proto
    conn.ack_result = telemetry_pb2.Telemetry(
        flags=_ACK_FRESH_BIT, ack_corr=1, ack_err=0)

    reply = binary_bridge.translate_command(nezha, "OA -0.98")

    assert reply == "OK oa"
    sent = conn.envelope_calls[0]
    assert sent.config.otos.angular_scale == pytest.approx(-0.98)


def test_oi_sends_otos_config_patch_with_init_trigger(proto):
    nezha, conn = proto
    conn.ack_result = telemetry_pb2.Telemetry(
        flags=_ACK_FRESH_BIT, ack_corr=1, ack_err=0)

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
    conn.ack_result = telemetry_pb2.Telemetry(
        flags=_ACK_FRESH_BIT, ack_corr=1, ack_err=envelope_pb2.ERR_UNIMPLEMENTED)

    reply = binary_bridge.translate_command(nezha, "OL 1.05")

    assert reply.startswith("ERR nak")


def test_ov_op_or_still_render_unavailable_reply_unchanged(proto):
    """OV/OP/OR have no direct-patch-send equivalent this ticket -- they
    fall through to the SAME launch-unblock short-circuit every other
    non-OL/OA/OI verb hits (no envelope sent)."""
    nezha, conn = proto

    for verb in ("OV 0 0 0", "OP", "OR"):
        reply = binary_bridge.translate_command(nezha, verb)
        assert reply == binary_bridge._LEGACY_UNAVAILABLE_REPLY

    assert conn.envelope_calls == []


# ---------------------------------------------------------------------------
# envelope_pb2 schema — locks in the shrunk oneofs render_log_line()'s
# id/echo/helptext branches now depend on being unreachable for (see module
# docstring, point 2).
# ---------------------------------------------------------------------------


def test_reply_oneof_no_longer_has_id_echo_helptext():
    fields = envelope_pb2.ReplyEnvelope.DESCRIPTOR.oneofs_by_name["body"].fields
    assert {f.name for f in fields} == {"ok", "err", "tlm"}


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


def _armor(msg) -> str:
    return "*B" + base64.b64encode(msg.SerializeToString()).decode("ascii")


def test_tlm_reply_is_dropped_entirely():
    reply = envelope_pb2.ReplyEnvelope()
    reply.tlm.now = 12345
    assert binary_bridge.render_log_line(_armor(reply), outbound=False) is None


def test_err_reply_falls_back_to_text_format_rendering():
    """``render`` is ``None`` (launch-unblock) — the ``err`` branch is
    unreachable (``if render is not None:`` guards it), so every reply,
    including ``err``, takes the ``text_format`` fallback: readable, not raw
    armor, but no longer the old ``legacy_render``-specific
    ``"ERR badarg #4"`` shape."""
    reply = envelope_pb2.ReplyEnvelope()
    reply.corr_id = 4
    reply.err.code = envelope_pb2.ERR_BADARG
    rendered = binary_bridge.render_log_line(_armor(reply), outbound=False)
    assert rendered is not None
    assert not rendered.startswith("*B")
    assert "ERR_BADARG" in rendered
    assert "4" in rendered


def test_ok_reply_renders_readable_text_not_raw_armor():
    reply = envelope_pb2.ReplyEnvelope()
    reply.ok.q = 3
    reply.ok.rem = 45.0
    rendered = binary_bridge.render_log_line(_armor(reply), outbound=False)
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
    rendered = binary_bridge.render_log_line(_armor(cmd), outbound=True)
    assert rendered is not None
    assert not rendered.startswith("*B")
    assert "200" in rendered


def test_non_armored_line_passes_through_unchanged():
    line = "DEVICE:NEZHA2:robot:tovez:123"
    assert binary_bridge.render_log_line(line, outbound=True) == line
    assert binary_bridge.render_log_line(line, outbound=False) == line


def test_malformed_armor_passes_through_unchanged_never_raises():
    garbage = "*Bnot-valid-base64!!!"
    assert binary_bridge.render_log_line(garbage, outbound=True) == garbage
    assert binary_bridge.render_log_line(garbage, outbound=False) == garbage


# ---------------------------------------------------------------------------
# render_log_line() — TelemetrySecondary disambiguation (emergency fix,
# stakeholder report: Tour 1 froze/died and the message monitor flooded at
# ~4 lines/s with bare "corr_id: N" lines). A bare TelemetrySecondary frame
# (its own armored *B line, NOT ReplyEnvelope-wrapped — telemetry.proto,
# 104-003) "successfully" parses as a ReplyEnvelope with an EMPTY body oneof:
# TelemetrySecondary's first field (`now`, its millisecond timestamp) and
# ReplyEnvelope's first field (`corr_id`) are both wire type 13 (uint32),
# so the bytes decode without error into a ReplyEnvelope carrying only
# corr_id set and no body arm. Fixed with the same structural
# disambiguation io/serial_conn.py's _handle_binary_reply() already uses:
# treat a ReplyEnvelope parse as real only when WhichOneof("body") is set;
# otherwise retry as TelemetrySecondary and drop the line on success (same
# policy as a primary `tlm` push frame).
# ---------------------------------------------------------------------------


def test_bare_telemetry_secondary_frame_is_dropped_not_misrendered():
    from robot_radio.robot.pb2 import telemetry_pb2

    secondary = telemetry_pb2.TelemetrySecondary()
    secondary.now = 123456
    secondary.has_cmd_vel = True
    secondary.cmd_vel_left = 150.0
    secondary.cmd_vel_right = 150.0
    secondary.acc_left = 0.5
    secondary.acc_right = 0.4
    secondary.glitch_left = 0
    secondary.glitch_right = 0
    secondary.ts_left = 12
    secondary.ts_right = 12

    rendered = binary_bridge.render_log_line(_armor(secondary), outbound=False)

    assert rendered is None


def test_reply_envelope_with_set_body_still_renders_not_dropped():
    """Regression guard for the fix above: a REAL ReplyEnvelope (body oneof
    actually set) must still render normally, not get swept into the new
    TelemetrySecondary fallback."""
    reply = envelope_pb2.ReplyEnvelope()
    reply.corr_id = 7
    reply.ok.q = 1
    rendered = binary_bridge.render_log_line(_armor(reply), outbound=False)
    assert rendered is not None
    assert not rendered.startswith("*B")


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
