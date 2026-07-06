"""tests/testgui/test_transport.py — headless tests for SimTransport against
the sprint-081/082 ctypes ABI (ticket 083-001).

No ``QApplication`` needed: ``robot_radio.testgui.transport`` is Qt-free
except for a lazily-imported ``QMessageBox`` warning path exercised only
when the sim lib is missing (not exercised here). Run with::

    QT_QPA_PLATFORM=offscreen uv run pytest tests/testgui/test_transport.py -q

Requires the compiled ``tests/_infra/sim/build/libfirmware_host.{dylib,so}``
(``just build-sim``) — every test here skips cleanly if it is not present.

This module is not yet wired into ``pyproject.toml``'s ``testpaths`` (that is
ticket 083-004's job, which also adds this directory's own fixtures/
conftest) — run it directly, per this ticket's Testing section.
"""
from __future__ import annotations

import time
from unittest.mock import MagicMock

import pytest

from robot_radio.testgui.transport import SimTransport, _sim_lib_path

pytestmark = pytest.mark.skipif(
    not _sim_lib_path().exists(),
    reason="sim lib not built -- run `just build-sim` first",
)

# Bounded wait for the tick-thread to reach a particular state.  Generous
# relative to every observed run (well under 100 ms) so a slow CI box never
# flakes; a real hang still fails the test rather than blocking forever.
_WAIT_TIMEOUT_S = 5.0
_POLL_INTERVAL_S = 0.02
# How long to let the tick-thread run its startup sequence (SimConnection
# .connect() -> _apply_field_profile() -> STREAM 50) before a test starts
# monkeypatching SimConnection setter methods on transport._conn -- avoids a
# race between the ONE-TIME startup profile application and a test's own
# apply_error_profile() call landing on the (by-then-patched) methods.
_STARTUP_SETTLE_S = 0.3


def _wait_until(predicate, timeout_s: float = _WAIT_TIMEOUT_S) -> bool:
    """Poll ``predicate`` until it is truthy or ``timeout_s`` elapses."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(_POLL_INTERVAL_S)
    return predicate()


@pytest.fixture
def transport():
    """A connected SimTransport; disconnected on teardown even on failure."""
    t = SimTransport()
    t.on_log = lambda _s: None
    t.connect()
    assert t._connected, "SimTransport failed to connect -- is the sim lib built?"
    try:
        yield t
    finally:
        t.disconnect()


# ---------------------------------------------------------------------------
# (a) drive via send() + tick -> moving TLMFrame via on_telemetry
# ---------------------------------------------------------------------------

def test_drive_produces_moving_telemetry(transport: SimTransport) -> None:
    """``send()``-issued drive commands, once ticked, deliver a moving TLMFrame.

    Exercises the one-way ``conn.send_fast()`` drive path plus the
    tick-thread's per-iteration ``SNAP`` telemetry poll (``_tick_loop``) —
    the only way a fresh TLM sample is available in this ABI: ``STREAM``'s
    periodic re-emission does not flow through the async EVT sink (verified
    directly against the built ``libfirmware_host`` — ``conn.tick()`` never
    returns a ``TLM ...`` line on its own, however long it runs).
    """
    frames = []
    transport.on_telemetry = frames.append

    transport.send("DEV DT PORTS 1 2")
    transport.send("DEV DT VW 200 0 0")

    def _has_moving_frame() -> bool:
        return any(
            getattr(f, "mode", None) == "S" and (f.vel[0] or f.vel[1])
            for f in frames
        )

    assert _wait_until(_has_moving_frame), (
        f"no moving TLMFrame observed via on_telemetry within "
        f"{_WAIT_TIMEOUT_S}s; frames received: {frames!r}"
    )


# ---------------------------------------------------------------------------
# (b) apply_error_profile() -> correct SimConnection setter, correct value
# ---------------------------------------------------------------------------

def test_apply_error_profile_calls_setters_and_warns_no_op_fields(
    transport: SimTransport,
) -> None:
    """Every profile field reaches the right ``SimConnection`` setter with
    the right value; the no-ctypes-backing fields (``motor_offset_l/r``,
    ``slip_turn_extra``) are skipped and logged as a ``[WARN]`` instead of
    raising.
    """
    # Let the tick-thread finish its one-time startup profile application
    # before patching -- otherwise it could race the patch and either call
    # the real (pre-patch) method or inflate the mocks' call counts.
    time.sleep(_STARTUP_SETTLE_S)

    conn = transport._conn
    assert conn is not None

    setter_names = (
        "set_otos_linear_scale_error", "set_otos_angular_scale_error",
        "set_otos_linear_noise", "set_otos_yaw_noise",
        "set_otos_linear_drift", "set_otos_yaw_drift",
        "set_body_rotational_scrub", "set_body_linear_scrub",
        "set_trackwidth", "set_enc_noise", "set_enc_scale_error",
    )
    mocked = {name: MagicMock() for name in setter_names}
    for name, mock in mocked.items():
        setattr(conn, name, mock)

    logs: list[str] = []
    transport.on_log = logs.append

    profile = {
        "encoder_noise": 1.11,
        "slip_turn_extra": 0.42,   # off-neutral -> must warn, not apply
        "otos_linear_noise": 2.22,
        "otos_yaw_noise": 3.33,
        "enc_scale_err_l": 4.44,
        "enc_scale_err_r": 5.55,
        "otos_lin_scale_err": 6.66,
        "otos_ang_scale_err": 7.77,
        "otos_lin_drift": 8.88,
        "otos_yaw_drift": 9.99,
        "body_rot_scrub": 0.11,
        "body_lin_scrub": 0.22,
        "motor_offset_l": 1.5,     # off-neutral -> must warn, not apply
        "motor_offset_r": 0.5,     # off-neutral -> must warn, not apply
        "trackwidth": 130.0,
    }
    transport.apply_error_profile(profile)

    assert _wait_until(lambda: mocked["set_trackwidth"].call_count > 0), (
        "apply_error_profile() never reached the tick-thread"
    )
    # The whole _apply_profile_to_sim() call runs synchronously within one
    # tick-thread iteration once started; a short grace period covers the
    # remaining (already-in-flight) setter calls in that same pass.
    time.sleep(0.1)

    mocked["set_otos_linear_scale_error"].assert_called_once_with(6.66)
    mocked["set_otos_angular_scale_error"].assert_called_once_with(7.77)
    mocked["set_otos_linear_noise"].assert_called_once_with(2.22)
    mocked["set_otos_yaw_noise"].assert_called_once_with(3.33)
    mocked["set_otos_linear_drift"].assert_called_once_with(8.88)
    mocked["set_otos_yaw_drift"].assert_called_once_with(9.99)
    mocked["set_body_rotational_scrub"].assert_called_once_with(0.11)
    mocked["set_body_linear_scrub"].assert_called_once_with(0.22)
    mocked["set_trackwidth"].assert_called_once_with(130.0)
    mocked["set_enc_noise"].assert_called_once_with(2, 1.11)
    mocked["set_enc_scale_error"].assert_any_call(0, 4.44)
    mocked["set_enc_scale_error"].assert_any_call(1, 5.55)
    assert mocked["set_enc_scale_error"].call_count == 2

    warn_logs = [line for line in logs if "[WARN]" in line]
    assert any("motor_offset_l" in line for line in warn_logs), warn_logs
    assert any("motor_offset_r" in line for line in warn_logs), warn_logs
    assert any("slip_turn_extra" in line for line in warn_logs), warn_logs


# ---------------------------------------------------------------------------
# (c) disconnect() cleanly joins the tick-thread
# ---------------------------------------------------------------------------

def test_disconnect_joins_tick_thread_cleanly() -> None:
    t = SimTransport()
    t.on_log = lambda _s: None
    t.connect()
    assert t._connected

    thread = t._tick_thread
    assert thread is not None
    assert thread.is_alive()

    t.disconnect()

    assert not t._connected
    assert t._tick_thread is None
    assert not thread.is_alive()

    # Idempotent -- disconnecting an already-disconnected transport must
    # not hang or raise.
    t.disconnect()
