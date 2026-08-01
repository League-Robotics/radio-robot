"""src/tests/unit/test_halt_now.py — ticket 128-001.

``robot_radio.robot.halt.halt_now()`` is the one shared "halt now" idiom
every host call site must use instead of ``proto.stop()`` (the PLANNED
stop, which queues behind whatever ``Move`` is already in flight — measured
39.8cm/5.9s vs. 2.9cm/0.10s for ``estop()``) or a bespoke swallowed-
exception retry loop
(``halt-now-call-sites-must-use-estop-and-never-swallow-failure.md``).
Modeled on the first hardened version of this idiom,
``field/geofence.py::Geofence._halt()`` (127-003), which now delegates to
it (see ``test_geofence_capture_fix_yaw_wrap.py`` for that class's own
still-unaffected coverage).

Covers:
  1. ``estop()`` succeeding on the first attempt — no retry, no log line.
  2. ``estop()`` failing twice then succeeding on the third attempt.
  3. ``estop()`` failing all three attempts — logs the loud ERROR line via
     the injected ``log`` callable, then RE-RAISES the last exception
     (never silently swallows a total halt failure — a halt that failed
     silently is indistinguishable from one that worked, which is exactly
     the defect this helper exists to close).
  4. ``log`` defaults to ``print`` when the caller passes none.
"""
from __future__ import annotations

import pytest

from robot_radio.robot.halt import halt_now


class _FakeProto:
    """``estop()`` fails for the first ``fail_count`` calls, then
    succeeds."""

    def __init__(self, fail_count: int = 0) -> None:
        self.fail_count = fail_count
        self.calls = 0

    def estop(self) -> int:
        self.calls += 1
        if self.calls <= self.fail_count:
            raise ConnectionError(f"estop attempt {self.calls} failed")
        return self.calls


def test_halt_now_succeeds_on_first_attempt_no_log():
    proto = _FakeProto(fail_count=0)
    logged: list[str] = []

    halt_now(proto, log=logged.append)

    assert proto.calls == 1
    assert logged == []


def test_halt_now_retries_and_succeeds_on_third_attempt():
    proto = _FakeProto(fail_count=2)
    logged: list[str] = []

    halt_now(proto, log=logged.append)

    assert proto.calls == 3
    assert logged == []


def test_halt_now_raises_and_logs_loudly_after_three_failures():
    proto = _FakeProto(fail_count=3)
    logged: list[str] = []

    with pytest.raises(ConnectionError, match="attempt 3"):
        halt_now(proto, log=logged.append)

    assert proto.calls == 3
    assert len(logged) == 1
    assert "ERROR" in logged[0]
    assert "estop() failed 3x" in logged[0]
    assert "ROBOT MAY STILL BE MOVING" in logged[0]


def test_halt_now_defaults_log_to_print(capsys):
    proto = _FakeProto(fail_count=3)

    with pytest.raises(ConnectionError):
        halt_now(proto)

    captured = capsys.readouterr()
    assert "ROBOT MAY STILL BE MOVING" in captured.out
