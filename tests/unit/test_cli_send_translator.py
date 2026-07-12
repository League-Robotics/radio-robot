"""tests/unit/test_cli_send_translator.py -- 097-004 (M5 rogo REPL
Translator).

Covers ``host/robot_radio/io/cli.py``'s ``cmd_send``/``_print_binary_reply``
extension: ``rogo send <verb> [args...]`` translates a text-v2 command line
into a binary ``CommandEnvelope`` for any verb with a proven binary
replacement (reusing ``legacy_translate.py``, M4, 097-002/004 -- this file
does not re-derive that translation, only checks ``cmd_send`` routes to it
correctly and builds the envelope M4 says to build), sends the five-verb
safety rump (PING/ID/HELLO/HELP/STOP) and any verb with no binary
replacement (R/TURN/G, DEV, ...) as plain text unchanged, and pretty-prints
a decoded reply under ``--decode``.

No real serial port/hardware anywhere here: ``cli._make_robot`` is
monkeypatched to return a lightweight fake ``SerialConnection`` double
(``_FakeConn``) that records exactly what ``cmd_send`` sent it (an
envelope, or a raw text line) -- this is a black-box test of ``cmd_send``'s
own dispatch/routing logic, at the same "no hardware" level the sibling
``test_legacy_translate.py``/``test_protocol_binary_client.py`` use for the
functions/methods one layer below this one.

Collected under ``tests/unit/`` (host-side unit/tooling check, not
sim/bench/playfield-scoped -- see ``tests/CLAUDE.md``); ``pyproject.toml``'s
``testpaths`` includes ``tests/unit`` so ``uv run python -m pytest`` collects
it.
"""

from __future__ import annotations

import argparse

import pytest

from robot_radio.io import cli
from robot_radio.robot import legacy_translate
from robot_radio.robot.pb2 import drivetrain_pb2, envelope_pb2


# ---------------------------------------------------------------------------
# Test double -- records exactly what cmd_send() sent, no real serial port.
# ---------------------------------------------------------------------------


class _FakeConn:
    """Stand-in for SerialConnection: records the one call cmd_send() makes
    (either .send() for plain text, or .send_envelope() for the binary
    plane) and returns a caller-supplied canned reply."""

    def __init__(self, envelope_reply=None, text_responses=None):
        self.envelope_sent: envelope_pb2.CommandEnvelope | None = None
        self.envelope_read_timeout: int | None = None
        self.text_sent: str | None = None
        self.text_read_timeout: int | None = None
        self.disconnected = False
        self._envelope_reply = envelope_reply
        self._text_responses = text_responses if text_responses is not None else []

    def send(self, message: str, read_timeout: int = 500) -> dict:
        self.text_sent = message
        self.text_read_timeout = read_timeout
        return {"sent": message, "mode": "direct", "responses": self._text_responses}

    def send_envelope(self, envelope: envelope_pb2.CommandEnvelope,
                      read_timeout: int = 500) -> dict:
        self.envelope_sent = envelope
        self.envelope_read_timeout = read_timeout
        return {"sent": envelope, "mode": "direct", "reply": self._envelope_reply}

    def disconnect(self) -> None:
        self.disconnected = True


def _run_send(monkeypatch, tokens: list[str], *, decode: bool = False,
              read_timeout: int = 500, envelope_reply=None,
              text_responses=None) -> _FakeConn:
    """Run cmd_send() against a fake connection, return the fake for
    assertion. Mirrors cli.main()'s own argparse Namespace shape for the
    `send` subcommand (message/read_timeout/decode -- the three attributes
    cmd_send() actually reads; _make_robot() itself is mocked away, so no
    other Namespace attribute is needed)."""
    fake_conn = _FakeConn(envelope_reply=envelope_reply, text_responses=text_responses)
    monkeypatch.setattr(cli, "_make_robot", lambda args: (object(), fake_conn, {}))
    args = argparse.Namespace(message=tokens, read_timeout=read_timeout, decode=decode)
    cli.cmd_send(args)
    return fake_conn


# ---------------------------------------------------------------------------
# _tokenize_send_line() -- CommandProcessor::parseTokens()/parseKV() mirror
# ---------------------------------------------------------------------------


def test_tokenize_splits_verb_positional_and_kv():
    verb, pos, kv = cli._tokenize_send_line("MOVE 500 9000 9000 v=300 w=4500 s=1")
    assert verb == "MOVE"
    assert pos == ["500", "9000", "9000"]
    assert kv == {"v": "300", "w": "4500", "s": "1"}


def test_tokenize_upper_cases_only_the_verb():
    verb, pos, kv = cli._tokenize_send_line("s 200 200")
    assert verb == "S"
    assert pos == ["200", "200"]


def test_tokenize_empty_line():
    assert cli._tokenize_send_line("") == ("", [], {})
    assert cli._tokenize_send_line("   ") == ("", [], {})


# ---------------------------------------------------------------------------
# Binary-mapped verbs -- rogo send <verb> ... produces the SAME envelope
# `rogo binary <arm>` builds by hand, and never touches conn.send() (text).
# ---------------------------------------------------------------------------


def test_send_s_produces_the_same_wire_bytes_as_binary_drive(monkeypatch):
    """`rogo send S 200 200` -- acceptance criterion 1: same on-wire effect
    as `rogo binary drive --left 200 --right 200` (cmd_binary_drive()'s own
    construction, motion_commands.py's cli.py: env.drive.wheels.w.add(...)
    x2)."""
    reply = envelope_pb2.ReplyEnvelope(corr_id=1, ok=envelope_pb2.Ack(q=1))
    fake = _run_send(monkeypatch, ["S", "200", "200"], envelope_reply=reply)

    assert fake.text_sent is None
    assert fake.envelope_sent is not None
    assert fake.envelope_sent.WhichOneof("cmd") == "drive"

    expected = envelope_pb2.CommandEnvelope()
    expected.drive.wheels.w.add(speed=200.0)
    expected.drive.wheels.w.add(speed=200.0)
    assert fake.envelope_sent.SerializeToString() == expected.SerializeToString()
    assert fake.disconnected is True


def test_send_d_produces_the_correct_segment_envelope(monkeypatch):
    """`rogo send D 200 200 300` -- acceptance criterion 2: the equivalent
    binary `segment` envelope, via M4's segment_for_distance() (handleD()'s
    own sign-then-distance computation)."""
    reply = envelope_pb2.ReplyEnvelope(corr_id=2, ok=envelope_pb2.Ack(q=1))
    fake = _run_send(monkeypatch, ["D", "200", "200", "300"], envelope_reply=reply)

    assert fake.text_sent is None
    assert fake.envelope_sent.WhichOneof("cmd") == "segment"

    expected_seg = legacy_translate.segment_for_distance(200.0, 200.0, 300)
    expected = envelope_pb2.CommandEnvelope(segment=expected_seg)
    assert fake.envelope_sent.SerializeToString() == expected.SerializeToString()
    # Straight forward drive (v = 200 > 0) -> sign +1 -> distance = +300.
    assert fake.envelope_sent.segment.distance == pytest.approx(300.0)


def test_send_t_produces_the_correct_segment_envelope(monkeypatch):
    reply = envelope_pb2.ReplyEnvelope(corr_id=3, ok=envelope_pb2.Ack(q=1))
    fake = _run_send(monkeypatch, ["T", "200", "200", "1000"], envelope_reply=reply)
    assert fake.envelope_sent.WhichOneof("cmd") == "segment"
    assert fake.envelope_sent.segment.distance == pytest.approx(200.0)


def test_send_rt_produces_a_segment_with_final_heading_only(monkeypatch):
    reply = envelope_pb2.ReplyEnvelope(corr_id=4, ok=envelope_pb2.Ack(q=1))
    fake = _run_send(monkeypatch, ["RT", "9000"], envelope_reply=reply)
    assert fake.envelope_sent.WhichOneof("cmd") == "segment"
    seg = fake.envelope_sent.segment
    assert seg.distance == pytest.approx(0.0)
    assert seg.final_heading == pytest.approx(1.5707963705062866, abs=1e-6)


def test_send_move_produces_a_segment_with_kv_overrides(monkeypatch):
    reply = envelope_pb2.ReplyEnvelope(corr_id=5, ok=envelope_pb2.Ack(q=1))
    fake = _run_send(
        monkeypatch, ["MOVE", "500", "9000", "9000", "v=300", "w=4500", "s=1"],
        envelope_reply=reply)
    assert fake.envelope_sent.WhichOneof("cmd") == "segment"
    seg = fake.envelope_sent.segment
    assert seg.distance == pytest.approx(500.0)
    assert seg.direction == pytest.approx(1.5707963705062866, abs=1e-6)
    assert seg.speed_max == pytest.approx(300.0)
    assert seg.yaw_rate_max == pytest.approx(0.7853981852531433, abs=1e-6)
    assert seg.stream is True


def test_send_mover_produces_a_replace_arm_not_segment(monkeypatch):
    """MOVER posts to bb.replaceIn (REPLACE semantics), not bb.segmentIn --
    the envelope's `replace` oneof arm, distinct from MOVE's `segment` arm."""
    reply = envelope_pb2.ReplyEnvelope(corr_id=6, ok=envelope_pb2.Ack(q=1))
    fake = _run_send(
        monkeypatch, ["MOVER", "0", "0", "0", "t=400", "v=-300", "w=-4500"],
        envelope_reply=reply)
    assert fake.envelope_sent.WhichOneof("cmd") == "replace"
    seg = fake.envelope_sent.replace
    assert seg.stream is True
    assert seg.v == pytest.approx(-300.0)
    assert seg.speed_max == pytest.approx(300.0)  # |v|, not v


def test_send_echo_produces_an_echo_envelope(monkeypatch):
    reply = envelope_pb2.ReplyEnvelope(corr_id=7, echo=envelope_pb2.Echo(payload=b"hi"))
    fake = _run_send(monkeypatch, ["ECHO", "hi", "there"], envelope_reply=reply)
    assert fake.envelope_sent.WhichOneof("cmd") == "echo"
    assert fake.envelope_sent.echo.payload == b"hi there"


def _run_and_expect_exit(monkeypatch, tokens: list[str]) -> _FakeConn:
    fake_conn = _FakeConn()
    monkeypatch.setattr(cli, "_make_robot", lambda args: (object(), fake_conn, {}))
    args = argparse.Namespace(message=tokens, read_timeout=500, decode=False)
    with pytest.raises(SystemExit):
        cli.cmd_send(args)
    return fake_conn


def test_send_binary_usage_error_reports_and_exits(monkeypatch, capsys):
    """Too few positional args for a binary-mapped verb -> a clean usage
    error (stderr + nonzero exit), never a raw traceback -- and no envelope
    is ever sent."""
    fake = _run_and_expect_exit(monkeypatch, ["S", "200"])
    assert fake.envelope_sent is None
    captured = capsys.readouterr()
    assert "S requires" in captured.err


# ---------------------------------------------------------------------------
# Rump verbs (PING/ID/HELLO/HELP/STOP) -- plain text, unchanged, never an
# envelope -- the rump exists so a bare terminal needs no translation.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("verb", ["STOP", "PING", "ID", "HELLO", "HELP"])
def test_send_rump_verbs_send_plain_text_not_envelope(monkeypatch, verb):
    fake = _run_send(monkeypatch, [verb], text_responses=[f"OK {verb.lower()}"])
    assert fake.envelope_sent is None
    assert fake.text_sent == verb
    assert fake.disconnected is True


def test_rump_verb_table_is_exactly_the_five_verb_safety_rump():
    """architecture-update.md (097) Step 1: "...a five-verb safety rump
    (PING, ID, HELLO, HELP, STOP)" -- no more, no fewer."""
    assert cli._SEND_RUMP_VERBS == frozenset({"PING", "ID", "HELLO", "HELP", "STOP"})


# ---------------------------------------------------------------------------
# No-binary-replacement fallback (R/TURN/G -- Planner parked; DEV; anything
# unrecognized) -- plain text, unchanged. NOT an error/failure path.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("raw", ["R 200 500", "TURN 9000", "G 100 100 200",
                                 "DEV STATE", "UNKNOWNVERB 1 2 3"])
def test_send_verbs_with_no_binary_replacement_fall_through_to_text(monkeypatch, raw):
    fake = _run_send(monkeypatch, raw.split())
    assert fake.envelope_sent is None
    assert fake.text_sent == raw


# ---------------------------------------------------------------------------
# --decode -- pretty-print a binary reply's decoded fields instead of the
# raw protobuf text-format dump.
# ---------------------------------------------------------------------------


def test_decode_reply_body_formats_populated_fields():
    reply = envelope_pb2.ReplyEnvelope(
        corr_id=42, ok=envelope_pb2.Ack(q=3, rem=12.5, t=999))
    decoded = cli._decode_reply_body(reply)
    assert "ok:" in decoded
    assert "q = 3" in decoded
    assert "rem = 12.5" in decoded
    assert "t = 999" in decoded
    assert "corr_id = 42" in decoded


def test_decode_flag_prints_decoded_fields_not_raw_repr(monkeypatch, capsys):
    reply = envelope_pb2.ReplyEnvelope(
        corr_id=9, ok=envelope_pb2.Ack(q=2, rem=5.0, t=111))
    _run_send(monkeypatch, ["S", "200", "200"], decode=True, envelope_reply=reply)
    out = capsys.readouterr().out
    assert "q = 2" in out
    assert "corr_id = 9" in out
    # The raw str(reply) text-format dump would render as "ok {\n  q: 2\n...}"
    # -- the decoded form uses "field = value" lines instead of "field: value"
    # nested inside a brace block, and never prints a bare "ok {" line.
    assert "ok {" not in out


def test_without_decode_flag_prints_raw_text_format_repr(monkeypatch, capsys):
    reply = envelope_pb2.ReplyEnvelope(
        corr_id=9, ok=envelope_pb2.Ack(q=2, rem=5.0, t=111))
    _run_send(monkeypatch, ["S", "200", "200"], decode=False, envelope_reply=reply)
    out = capsys.readouterr().out
    assert "ok {" in out


def test_decode_reply_body_empty_reply():
    reply = envelope_pb2.ReplyEnvelope(corr_id=5)
    decoded = cli._decode_reply_body(reply)
    assert decoded == "(empty reply)  corr_id=5"


# ---------------------------------------------------------------------------
# `rogo binary <arm>` byte-for-byte unaffected -- _print_binary_reply()'s
# new `decode` kwarg defaults False, so every pre-existing call site
# (cmd_binary_ping/echo/id/stop/drive/segment/replace) is unchanged.
# ---------------------------------------------------------------------------


def test_print_binary_reply_default_decode_false_matches_pre_097_004_output(capsys):
    reply = envelope_pb2.ReplyEnvelope(corr_id=1, ok=envelope_pb2.Ack(q=1, rem=0.0, t=0))
    cli._print_binary_reply({"reply": reply})
    out = capsys.readouterr().out
    assert out.strip() == str(reply).strip()


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
