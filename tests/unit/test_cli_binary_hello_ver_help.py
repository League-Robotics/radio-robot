"""tests/unit/test_cli_binary_hello_ver_help.py -- stakeholder-directed
6-verb minimal command surface (2026-07-10): ``rogo binary hello/ver/help``
(``host/robot_radio/io/cli.py``'s ``cmd_binary_hello``/``cmd_binary_ver``/
``cmd_binary_help``) build the correct ``CommandEnvelope`` oneof arm and
print the reply -- the direct-subcommand counterpart to the existing
``cmd_binary_ping``/``cmd_binary_id`` coverage (there is none dedicated,
this is the first for the `binary` subcommand family; ``rogo send``'s own
translation is covered separately by ``test_cli_send_translator.py``).

No real serial port/hardware: ``cli._make_robot`` is monkeypatched to
return a lightweight fake connection double, mirroring
``test_cli_send_translator.py``'s own ``_FakeConn``/monkeypatch pattern.
"""

from __future__ import annotations

import argparse

import pytest

from robot_radio.io import cli
from robot_radio.robot import legacy_verbs
from robot_radio.robot.pb2 import envelope_pb2


class _FakeConn:
    """Records the one ``send_envelope()`` call ``cmd_binary_*`` makes and
    returns a caller-supplied canned reply -- see
    ``test_cli_send_translator.py``'s own ``_FakeConn`` for the same
    pattern applied to ``cmd_send``."""

    def __init__(self, envelope_reply=None):
        self.envelope_sent: envelope_pb2.CommandEnvelope | None = None
        self.envelope_read_timeout: int | None = None
        self.disconnected = False
        self._envelope_reply = envelope_reply

    def send_envelope(self, envelope: envelope_pb2.CommandEnvelope,
                      read_timeout: int = 500) -> dict:
        self.envelope_sent = envelope
        self.envelope_read_timeout = read_timeout
        return {"sent": envelope, "mode": "direct", "reply": self._envelope_reply}

    def disconnect(self) -> None:
        self.disconnected = True


def _run_binary(monkeypatch, cmd_fn, *, read_timeout: int = 500, envelope_reply=None) -> _FakeConn:
    fake_conn = _FakeConn(envelope_reply=envelope_reply)
    monkeypatch.setattr(cli, "_make_robot", lambda args: (object(), fake_conn, {}))
    args = argparse.Namespace(read_timeout=read_timeout)
    cmd_fn(args)
    return fake_conn


# ---------------------------------------------------------------------------
# cmd_binary_hello -- CommandEnvelope{hello: Hello{}} -> ReplyEnvelope{id:...}
# ---------------------------------------------------------------------------


def test_cmd_binary_hello_sends_hello_arm_and_disconnects(monkeypatch, capsys):
    device = envelope_pb2.DeviceId(model="NEZHA2", name="bot1", serial=42,
                                   fw_version="1.2.3", proto_version=3)
    reply = envelope_pb2.ReplyEnvelope(corr_id=0, id=device)
    fake = _run_binary(monkeypatch, cli.cmd_binary_hello, envelope_reply=reply)

    assert fake.envelope_sent is not None
    assert fake.envelope_sent.WhichOneof("cmd") == "hello"
    assert fake.disconnected is True

    out = capsys.readouterr().out
    assert out.strip() == str(reply).strip()


# ---------------------------------------------------------------------------
# cmd_binary_ver -- CommandEnvelope{ver: Ver{}} -> ReplyEnvelope{id:...}
# ---------------------------------------------------------------------------


def test_cmd_binary_ver_sends_ver_arm_and_disconnects(monkeypatch, capsys):
    device = envelope_pb2.DeviceId(model="NEZHA2", name="bot1", serial=42,
                                   fw_version="1.2.3", proto_version=3)
    reply = envelope_pb2.ReplyEnvelope(corr_id=0, id=device)
    fake = _run_binary(monkeypatch, cli.cmd_binary_ver, envelope_reply=reply)

    assert fake.envelope_sent is not None
    assert fake.envelope_sent.WhichOneof("cmd") == "ver"
    assert fake.disconnected is True

    out = capsys.readouterr().out
    assert out.strip() == str(reply).strip()


# ---------------------------------------------------------------------------
# cmd_binary_help -- CommandEnvelope{help: Help{}} -> ReplyEnvelope{helptext:...}
# ---------------------------------------------------------------------------


def test_cmd_binary_help_sends_help_arm_and_disconnects(monkeypatch, capsys):
    reply = envelope_pb2.ReplyEnvelope(
        corr_id=0, helptext=envelope_pb2.HelpText(text="HELP HELLO PING ID VER STOP"))
    fake = _run_binary(monkeypatch, cli.cmd_binary_help, envelope_reply=reply)

    assert fake.envelope_sent is not None
    assert fake.envelope_sent.WhichOneof("cmd") == "help"
    assert fake.disconnected is True

    out = capsys.readouterr().out
    assert out.strip() == str(reply).strip()


def test_cmd_binary_help_no_reply_prints_timeout_notice(monkeypatch, capsys):
    """No queued reply (timeout) -- `_print_binary_reply()`'s own
    `"(no reply received -- timeout)"` path, same as every other `rogo
    binary <arm>` subcommand."""
    fake = _run_binary(monkeypatch, cli.cmd_binary_help, envelope_reply=None)
    assert fake.envelope_sent.WhichOneof("cmd") == "help"
    out = capsys.readouterr().out
    assert out.strip() == "(no reply received -- timeout)"


# ---------------------------------------------------------------------------
# legacy_verbs.py -- envelope_for_hello/ver/help + BINARY_DISPATCH wiring.
# The proxy (io/proxy.py) intercepts HELLO/HELP BEFORE consulting
# BINARY_DISPATCH (answered locally -- see that module's own docstring), so
# these three builders' only LIVE caller today is a future direct use of
# BINARY_DISPATCH; they are tested standalone here for exactly that reason
# -- "the binary arms must exist and be tested" even where the proxy itself
# doesn't (yet) route through them for hello/help.
# ---------------------------------------------------------------------------


def test_envelope_for_hello_builds_hello_arm():
    env = legacy_verbs.envelope_for_hello([], {})[0]
    assert env.WhichOneof("cmd") == "hello"


def test_envelope_for_ver_builds_dedicated_ver_arm_not_id():
    """VER used to alias envelope_for_id (reusing the `id` request arm
    outright); it now builds its own `{ver: Ver{}}` request -- distinct
    from ID's `{id: DeviceId{}}` on the wire, even though both still reply
    the identical DeviceId shape firmware-side."""
    env = legacy_verbs.envelope_for_ver([], {})[0]
    assert env.WhichOneof("cmd") == "ver"


def test_envelope_for_help_builds_help_arm():
    env = legacy_verbs.envelope_for_help([], {})[0]
    assert env.WhichOneof("cmd") == "help"


def test_binary_dispatch_covers_all_six_rump_verbs():
    """BINARY_DISPATCH now has a dedicated builder for every one of the
    six text safety rump verbs (HELP/HELLO/PING/ID/VER/STOP) -- completing
    the set that used to be missing HELLO/HELP entirely and aliased VER
    onto ID."""
    for verb, builder in [
        ("HELP", legacy_verbs.envelope_for_help),
        ("HELLO", legacy_verbs.envelope_for_hello),
        ("PING", legacy_verbs.envelope_for_ping),
        ("ID", legacy_verbs.envelope_for_id),
        ("VER", legacy_verbs.envelope_for_ver),
        ("STOP", legacy_verbs.envelope_for_stop),
    ]:
        assert legacy_verbs.BINARY_DISPATCH[verb] is builder


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
