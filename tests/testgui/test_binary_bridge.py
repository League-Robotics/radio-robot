"""tests/testgui/test_binary_bridge.py — 097 (this ticket): R/TURN/G
un-gating + the serial/message monitor's ``render_log_line`` filter.

Qt-free — no QApplication, no sim lib, no PySide6 required. Exercises
``robot_radio.testgui.binary_bridge`` directly against a fake connection
double (mirrors ``tests/unit/test_bridge_routing.py``'s ``_FakeConn``
pattern for ``io/proxy.py``'s ProtocolBridge — this file is the TestGUI
bridge's own equivalent, a module that test file's own fixtures do not
reach).

Run with::

    QT_QPA_PLATFORM=offscreen uv run pytest tests/testgui/test_binary_bridge.py -q

(``QT_QPA_PLATFORM`` is harmless here — nothing in this file touches Qt —
set for consistency with the rest of the ``tests/testgui`` suite's run
command.)
"""

from __future__ import annotations

import base64

import pytest

from robot_radio.robot.pb2 import envelope_pb2
from robot_radio.robot.protocol import NezhaProtocol
from robot_radio.testgui import binary_bridge


# ---------------------------------------------------------------------------
# Fake connection double — SimConnection-shaped send_envelope() (returns the
# decoded ReplyEnvelope directly, not the SerialConnection dict wrapper; see
# NezhaProtocol._send_envelope()'s own docstring for why both shapes exist).
# ---------------------------------------------------------------------------


class _FakeConn:
    def __init__(self) -> None:
        self.envelope_calls: list[envelope_pb2.CommandEnvelope] = []
        self._reply_queue: list["envelope_pb2.ReplyEnvelope | None"] = []

    def queue_reply(self, reply: "envelope_pb2.ReplyEnvelope | None") -> None:
        self._reply_queue.append(reply)

    def send_envelope(self, envelope: envelope_pb2.CommandEnvelope,
                      read_timeout: int = 500) -> "envelope_pb2.ReplyEnvelope | None":
        self.envelope_calls.append(envelope)
        return self._reply_queue.pop(0) if self._reply_queue else None

    def drain_binary_tlm(self) -> list:
        return []


def _ack_reply(q: int = 0, rem: float = 0.0, t: int = 0) -> envelope_pb2.ReplyEnvelope:
    reply = envelope_pb2.ReplyEnvelope()
    reply.ok.q = q
    reply.ok.rem = rem
    reply.ok.t = t
    return reply


@pytest.fixture
def proto():
    conn = _FakeConn()
    return NezhaProtocol(conn), conn


# ---------------------------------------------------------------------------
# R/TURN/G are un-gated (097) — no longer typed ERR "unsupported", no
# longer absent from BINARY_DISPATCH; each sends exactly one envelope of
# the expected oneof arm.
# ---------------------------------------------------------------------------


def test_r_translates_to_replace_arm_and_is_no_longer_unsupported(proto):
    nezha, conn = proto
    conn.queue_reply(_ack_reply())

    reply_line = binary_bridge.translate_command(nezha, "R 200 500")

    assert len(conn.envelope_calls) == 1
    assert conn.envelope_calls[0].WhichOneof("cmd") == "replace"
    assert "unsupported" not in reply_line
    assert reply_line.startswith("OK arc")


def test_turn_translates_to_segment_arm_and_is_no_longer_unsupported(proto):
    nezha, conn = proto
    conn.queue_reply(_ack_reply())

    reply_line = binary_bridge.translate_command(nezha, "TURN 9000 eps=300")

    assert len(conn.envelope_calls) == 1
    assert conn.envelope_calls[0].WhichOneof("cmd") == "segment"
    assert "unsupported" not in reply_line
    assert reply_line.startswith("OK turn")


def test_g_translates_to_segment_arm_and_is_no_longer_unsupported(proto):
    nezha, conn = proto
    conn.queue_reply(_ack_reply())

    reply_line = binary_bridge.translate_command(nezha, "G 300 400 150")

    assert len(conn.envelope_calls) == 1
    assert conn.envelope_calls[0].WhichOneof("cmd") == "segment"
    assert "unsupported" not in reply_line
    assert reply_line.startswith("OK goto")


def test_grip_and_qlen_remain_unsupported_no_wire_call(proto):
    """The two verbs that never had (and still don't have) a binary arm."""
    nezha, conn = proto

    for verb in ("GRIP", "QLEN"):
        reply_line = binary_bridge.translate_command(nezha, verb)
        assert "unsupported" in reply_line
    assert conn.envelope_calls == []


def test_pose_reset_and_otos_verbs_remain_gated_no_wire_call(proto):
    """SI/ZERO/OZ (pose reset) and OI/OL/OA/OV/OP/OR (OTOS chip) genuinely
    need sprint 098's fused pose / a real OTOS device — still gated."""
    nezha, conn = proto

    for verb, expected_code in (
        ("SI 0 0 0", "unsupported"),
        ("ZERO enc", "unsupported"),
        ("OZ", "unsupported"),
        ("OI", "nodev"),
        ("OP", "nodev"),
    ):
        reply_line = binary_bridge.translate_command(nezha, verb)
        assert expected_code in reply_line
    assert conn.envelope_calls == []


# ---------------------------------------------------------------------------
# render_log_line() — the serial/message monitor filter (Goal 4).
# ---------------------------------------------------------------------------


def _armor(msg) -> str:
    return "*B" + base64.b64encode(msg.SerializeToString()).decode("ascii")


def test_tlm_reply_is_dropped_entirely():
    reply = envelope_pb2.ReplyEnvelope()
    reply.tlm.now = 12345
    assert binary_bridge.render_log_line(_armor(reply), outbound=False) is None


def test_err_reply_renders_via_legacy_render():
    reply = envelope_pb2.ReplyEnvelope()
    reply.corr_id = 4
    reply.err.code = envelope_pb2.ERR_BADARG
    rendered = binary_bridge.render_log_line(_armor(reply), outbound=False)
    assert rendered == "ERR badarg #4"


def test_id_reply_renders_via_legacy_render():
    reply = envelope_pb2.ReplyEnvelope()
    reply.id.name = "tovez"
    reply.id.serial = 42
    reply.id.fw_version = "0.1"
    reply.id.proto_version = 3
    rendered = binary_bridge.render_log_line(_armor(reply), outbound=False)
    assert "tovez" in rendered
    assert "42" in rendered


def test_echo_reply_renders_via_legacy_render():
    reply = envelope_pb2.ReplyEnvelope()
    reply.echo.payload = b"hello"
    rendered = binary_bridge.render_log_line(_armor(reply), outbound=False)
    assert rendered == "OK echo hello"


def test_ok_reply_renders_readable_text_not_raw_armor():
    reply = envelope_pb2.ReplyEnvelope()
    reply.ok.q = 3
    reply.ok.rem = 45.0
    rendered = binary_bridge.render_log_line(_armor(reply), outbound=False)
    assert rendered is not None
    assert not rendered.startswith("*B")
    assert "3" in rendered


def test_outbound_command_renders_readable_text_not_raw_armor():
    cmd = envelope_pb2.CommandEnvelope()
    cmd.corr_id = 9
    cmd.drive.wheels.w.add(speed=200)
    cmd.drive.wheels.w.add(speed=-150)
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


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
