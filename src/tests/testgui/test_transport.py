"""src/tests/testgui/test_transport.py — headless tests for SimTransport against
the real ``sim_loop.SimLoop`` ABI (108-007 rewire).

No ``QApplication`` needed: ``robot_radio.testgui.transport`` is Qt-free
except for a lazily-imported ``QMessageBox`` warning path exercised only
when the sim lib is missing (not exercised here). Run with::

    QT_QPA_PLATFORM=offscreen uv run pytest src/tests/testgui/test_transport.py -q

Requires the compiled ``src/sim/build/libfirmware_host.{dylib,so}``
(``python src/sim/build.py``) — every test here skips cleanly if it
is not present.
"""
from __future__ import annotations

import time
from unittest.mock import MagicMock

import pytest

from robot_radio.testgui.transport import SimTransport, _sim_lib_path

pytestmark = pytest.mark.skipif(
    not _sim_lib_path().exists(),
    reason="sim lib not built -- run `python src/sim/build.py` first",
)

# Bounded wait for the tick-thread to reach a particular state.  Generous
# relative to every observed run (well under 100 ms) so a slow CI box never
# flakes; a real hang still fails the test rather than blocking forever.
_WAIT_TIMEOUT_S = 5.0
_POLL_INTERVAL_S = 0.02


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
# (a) drive via .protocol.twist() -> moving TLMFrame via on_telemetry
# ---------------------------------------------------------------------------

def test_drive_produces_moving_telemetry(transport: SimTransport) -> None:
    """``.protocol.twist()``-issued drive commands deliver a moving TLMFrame
    to ``on_telemetry``.

    108-007: ``SimLoop`` has no generic wire/config-channel simulation
    surface at all -- ``send()``/``command()`` on ``SimTransport`` are now
    best-effort no-ops (see that class's own docstring). Driving the sim for
    real happens exclusively through ``.protocol``'s ``twist()``/``stop()``
    surface (the same one a tour, or ``KeyboardDriver``, would use).
    """
    frames = []
    transport.on_telemetry = frames.append

    protocol = transport.protocol
    assert protocol is not None
    protocol.twist(200.0, 0.0, 500.0)

    def _has_moving_frame() -> bool:
        return any(f.vel is not None and (f.vel[0] or f.vel[1]) for f in frames)

    assert _wait_until(_has_moving_frame), (
        f"no moving TLMFrame observed via on_telemetry within "
        f"{_WAIT_TIMEOUT_S}s; frames received: {frames!r}"
    )


# ---------------------------------------------------------------------------
# (b) apply_error_profile() -> correct SimLoop setter, correct value; every
# unsupported knob logs a "not supported in this sim" [WARN] instead of
# raising.
# ---------------------------------------------------------------------------

def test_apply_error_profile_calls_setters_and_warns_no_op_fields(
    transport: SimTransport,
) -> None:
    """108-007: ``SimLoop``'s 19-symbol ABI backs exactly ONE profile
    mapping (``otos_lin_drift``/``otos_yaw_drift`` -> a single
    ``set_otos_drift(x, y, heading)`` call); every other
    ``DEFAULT_PROFILE`` key has no ``SimLoop`` setter at all and is
    skip-and-warn only when set away from its neutral default."""
    loop = transport.protocol
    assert loop is not None

    mocked = MagicMock()
    loop.set_otos_drift = mocked

    logs: list[str] = []
    transport.on_log = logs.append

    profile = {
        "encoder_noise": 1.11,          # no SimLoop setter -> warn
        "slip_turn_extra": 0.42,        # no SimLoop setter -> warn
        "otos_linear_noise": 2.22,      # no SimLoop setter -> warn
        "otos_yaw_noise": 3.33,         # no SimLoop setter -> warn
        "enc_scale_err_l": 4.44,        # no SimLoop setter -> warn
        "enc_scale_err_r": 5.55,        # no SimLoop setter -> warn
        "otos_lin_scale_err": 6.66,     # no SimLoop setter -> warn
        "otos_ang_scale_err": 7.77,     # no SimLoop setter -> warn
        "otos_lin_drift": 8.88,         # -> set_otos_drift(8.88, 0.0, 9.99)
        "otos_yaw_drift": 9.99,         # -> set_otos_drift(8.88, 0.0, 9.99)
        "body_rot_scrub": 0.11,         # no SimLoop setter -> warn
        "body_lin_scrub": 0.22,         # no SimLoop setter -> warn
        "motor_offset_l": 1.5,          # no SimLoop setter -> warn
        "motor_offset_r": 0.5,          # no SimLoop setter -> warn
        "trackwidth": 130.0,            # construction-time only -> [INFO]
    }
    transport.apply_error_profile(profile)

    mocked.assert_called_once_with(8.88, 0.0, 9.99)

    warn_logs = [line for line in logs if "[WARN]" in line]
    for key in (
        "encoder_noise", "slip_turn_extra", "otos_linear_noise",
        "otos_yaw_noise", "enc_scale_err_l", "enc_scale_err_r",
        "otos_lin_scale_err", "otos_ang_scale_err", "body_rot_scrub",
        "body_lin_scrub", "motor_offset_l", "motor_offset_r",
    ):
        assert any(key in line for line in warn_logs), (key, warn_logs)

    info_logs = [line for line in logs if "[INFO]" in line]
    assert any("trackwidth" in line and "NEXT Connect" in line for line in info_logs), info_logs


# ---------------------------------------------------------------------------
# (c) disconnect() cleanly joins SimLoop's own tick-thread
# ---------------------------------------------------------------------------

def test_disconnect_joins_tick_thread_cleanly() -> None:
    t = SimTransport()
    t.on_log = lambda _s: None
    t.connect()
    assert t._connected

    loop = t.protocol
    assert loop is not None
    thread = loop._thread
    assert thread is not None
    assert thread.is_alive()

    t.disconnect()

    assert not t._connected
    assert t.protocol is None
    assert not thread.is_alive()

    # Idempotent -- disconnecting an already-disconnected transport must
    # not hang or raise.
    t.disconnect()
