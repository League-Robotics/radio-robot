"""src/tests/unit/test_rogo_agent_manual.py — `rogo --agent` (2026-08-05).

The agent manual is the complete rogo reference (same convention as
`mbdeploy --agent`), and the short helps must POINT at it. These tests pin:

  1. the manual's load-bearing content — the daemon (`rogo serve`), the
     socket protocol event shapes, the server-local verbs, the panic-path
     stop-vs-estop distinction, the RogoClient recipe, and the
     fresh-boot-telemetry-quiet gotcha — so a rename or a protocol change
     breaks a test instead of silently staling the manual;
  2. the references: `rogo --help`'s epilog and the repl `help` verb both
     name `rogo --agent`.
"""
from __future__ import annotations

from robot_radio.io.agent_manual import MANUAL
from robot_radio.io import repl as repl_mod


def test_manual_covers_the_daemon_and_socket_protocol():
    for needle in (
        "rogo serve",
        "127.0.0.1:7646",
        "ROGO_ADDR",
        '"type": "result"',
        '"type": "tlm"',
        "sub tlm",
        "unsub tlm",
        "shutdown",
        "status",
    ):
        assert needle in MANUAL, f"agent manual lost its {needle!r} coverage"


def test_manual_covers_safety_and_gotchas():
    assert "PLANNED stop" in MANUAL          # stop is not a halt
    assert "estop" in MANUAL and "verif" in MANUAL.lower()
    assert "tlm on" in MANUAL                # fresh-boot telemetry-quiet gotcha
    assert "RESETS the robot" in MANUAL      # why the daemon exists
    assert "radio_channel" in MANUAL         # bake-only channel field


def test_manual_covers_the_client_library():
    assert "RogoClient" in MANUAL
    assert "subscribe_tlm" in MANUAL
    assert "frames(duration" in MANUAL
    assert "RogoDaemonError" in MANUAL


def test_short_helps_reference_the_agent_manual(capsys):
    assert "rogo --agent" in repl_mod._HELP  # the repl `help` verb

    from unittest import mock

    import pytest

    from robot_radio.io import cli as cli_mod

    # `rogo --help` prints usage + epilog and SystemExits before any serial I/O.
    with mock.patch("sys.argv", ["rogo", "--help"]):
        with pytest.raises(SystemExit):
            cli_mod.main()
    out = capsys.readouterr().out
    assert "rogo --agent" in out  # the epilog names the agent manual


def test_agent_flag_prints_manual_and_exits_before_any_connection(capsys):
    from unittest import mock

    from robot_radio.io import cli as cli_mod

    with mock.patch("sys.argv", ["rogo", "--agent"]):
        cli_mod.main()  # must return (no subcommand required, no serial I/O)
    out = capsys.readouterr().out
    assert "rogo — Agent Manual" in out
    assert "rogo serve" in out
