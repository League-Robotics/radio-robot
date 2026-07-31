"""src/tests/unit/test_cutebot_halt.py — ticket 128-001.

Two defects in ``robot_radio.robot.cutebot.Cutebot``
(``halt-now-call-sites-must-use-estop-and-never-swallow-failure.md``):

  1. ``rotate()`` referenced an undefined name ``robot`` (line 180) —
     a guaranteed ``NameError`` on any call, dead code no one could have
     been exercising. Fixed by removing the stray, unused line.
  2. ``speed()``'s ``GeneratorExit`` handler sent ``"X"`` once and silently
     swallowed any failure (``except Exception: pass``, no log line) before
     polling for a confirmation. It now goes through the shared
     ``halt_now()`` helper (``robot_radio.robot.halt``), which retries 3x
     and logs loudly on total failure — this test covers ``Cutebot`` now
     exposing an ``estop()`` that ``halt_now()`` can call polymorphically
     (Cutebot's wire protocol has no separate planned-vs-panic stop the
     way NezhaProtocol does, so ``estop()`` sends the same ``"X"`` command
     ``stop()`` does).

No real serial port: ``SerialConnection`` is faked with the minimal
surface ``Cutebot`` actually calls.
"""
from __future__ import annotations

from robot_radio.robot.cutebot import Cutebot


class _FakeConn:
    def __init__(self) -> None:
        self.fast_sent: list[str] = []
        self.sent: list[str] = []

    def send_fast(self, cmd: str) -> None:
        self.fast_sent.append(cmd)

    def send(self, cmd: str, read_timeout: int = 500) -> dict:
        self.sent.append(cmd)
        return {"responses": []}


def test_estop_sends_the_same_x_command_as_stop():
    """Cutebot has no queue to preempt -- "X" already halts immediately --
    so estop() and stop() are the same wire command, existing as two
    methods purely so halt_now() can treat every Robot polymorphically."""
    conn = _FakeConn()
    bot = Cutebot(conn)

    bot.estop()

    assert conn.fast_sent == ["X"]


def test_rotate_does_not_raise_name_error():
    """Regression test for the guaranteed NameError this ticket fixes --
    rotate() referenced an undefined name `robot` and could never have
    been called successfully before this fix."""
    conn = _FakeConn()
    bot = Cutebot(conn)

    bot.rotate(left=90.0, right=None, speed=50)

    assert conn.sent == ["ROT+900+0+50"]
