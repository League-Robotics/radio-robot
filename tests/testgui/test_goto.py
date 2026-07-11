"""tests/testgui/test_goto.py -- ticket 085-003: Camera GOTO pursuit-loop
verification (a NEW test -- there is no ``tests_old`` equivalent to port; see
architecture-update.md Grounding fact 4 and Design Rationale Decision 2).

``_GotoRunner`` (``host/robot_radio/testgui/__main__.py``, ~line 1374) is a
``QObject`` nested inside ``_build_main_window()`` with no import seam -- the
same constraint every other worker in this file is under (see
``test_tour_stop.py`` / ``test_tour_idle_detection.py``'s docstrings for the
established precedent). This file uses BOTH of this codebase's two answers to
that constraint, because ticket 003's acceptance criteria genuinely need both:

- **Part A** (below) re-implements ``run()``/``_safe_stop()``'s exact control
  flow inline (verified line-for-line against ``__main__.py``) and drives it
  with directly-controlled synthetic ``state["last_truth"]`` values -- Decision
  2's own words for this test strategy: "convergence-under-synthetic-truth".
  This is deterministic and fast, and is the only practical way to exercise
  the stale/missing-truth and explicit-``stop()`` edge cases without real-sim
  timing noise.
- **Part B** drives a REAL ``SimTransport`` (the compiled sprint-084
  ``source/`` firmware sim) because one acceptance criterion can only be
  answered by real firmware: does ``_safe_stop()``'s bare top-level ``STOP``
  actually cancel ``Subsystems::Planner``'s active ``G`` goal
  (architecture-update.md Decision 1 / Grounding fact 3)? A low-level test
  sends the exact same wire commands directly and watches telemetry
  ``mode=`` settle. Two further real-GUI tests drive the actual
  ``_GotoRunner``/``QThread``/``SimTransport`` stack (mirroring
  ``test_tour1_geometry.py``'s pattern) for the highest-fidelity confirmation
  of convergence and of the button-reenable-on-stop criterion.

One genuine gap this ticket's new test surfaced and fixed (not a behavior
bug): the GOTO x/y/eps/speed ``QSpinBox``es (``_make_goto_spin`` in
``__main__.py``) had no ``objectName`` -- unlike the tour buttons
(``tour_btn_*``) and Sim Errors spinboxes (``sim_err_*``), which made it
impossible to drive a real end-to-end GOTO test through the actual widgets.
Fixed by adding ``goto_spin_{x,y,eps,speed}`` object names (a test-seam-only
change, zero behavior change).

Run with::

    QT_QPA_PLATFORM=offscreen uv run pytest tests/testgui/test_goto.py -v
"""
from __future__ import annotations

import json
import math
import threading
import time

import pytest

from robot_radio.testgui.commands import goto_distance, goto_reached
from robot_radio.testgui.operations import build_setpose_command
from robot_radio.testgui.transport import SimTransport, _sim_lib_path

_requires_sim_lib = pytest.mark.skipif(
    not _sim_lib_path().exists(),
    reason="sim lib not built -- run `just build-sim` first",
)

# ---------------------------------------------------------------------------
# Part A -- inline reimplementation of _GotoRunner.run() / _safe_stop()
# (Decision 2: "convergence-under-synthetic-truth")
# ---------------------------------------------------------------------------

#: Mirrors _GotoRunner.TRUTH_MAX_AGE_S (host/robot_radio/testgui/__main__.py).
_TRUTH_MAX_AGE_S = 2.0


def _safe_stop(transport, log: list[str] | None = None) -> None:
    """Inline reimplementation of ``_GotoRunner._safe_stop()``."""
    try:
        transport.send("STOP")
    except Exception:  # noqa: BLE001
        pass


def _run_goto(
    transport,
    state: dict,
    stop_flag: dict,
    tx: float,  # [mm]
    ty: float,  # [mm]
    eps: float,  # [mm]
    speed: float,
    log: list[str],
    *,
    poll_s: float = 0.3,
    truth_max_age_s: float = _TRUTH_MAX_AGE_S,
    timeout_s: float = 60.0,
) -> str:
    """Inline reimplementation of ``_GotoRunner.run()`` (see module docstring).

    Control flow, including the exact log-then-stop / stop-then-log ordering
    on each exit path, matches ``__main__.py`` line-for-line as of this
    ticket. Returns one of ``"reached"``, ``"timeout"``, ``"aborted"``,
    ``"send_failed"`` -- ``_GotoRunner.run()``'s four exit paths -- so tests
    can assert on exactly which one fired.
    """
    deadline = time.monotonic() + timeout_s
    while not stop_flag.get("stop"):
        now = time.monotonic()
        if now > deadline:
            log.append("[GOTO] timed out -- aborting")
            _safe_stop(transport, log)
            return "timeout"

        truth = state.get("last_truth")
        if truth is None or (now - truth[3]) > truth_max_age_s:
            log.append("[GOTO] waiting for a fresh camera pose...")
            time.sleep(poll_s)
            continue

        x_cm, y_cm, yaw_rad, _ts = truth
        cur_x = x_cm * 10.0  # [mm]
        cur_y = y_cm * 10.0  # [mm]

        if goto_reached(tx, ty, cur_x, cur_y, eps):
            _safe_stop(transport, log)
            log.append("[GOTO] reached target -- complete")
            return "reached"

        si = build_setpose_command(x_cm, y_cm, yaw_rad)
        g = f"G {tx} {ty} {speed}"
        try:
            transport.command(si, read_timeout=200)
            transport.command(g, read_timeout=200)
        except Exception as exc:  # noqa: BLE001
            log.append(f"[GOTO] send failed: {exc}")
            return "send_failed"

        dist = goto_distance(tx, ty, cur_x, cur_y)
        log.append(f"[GOTO] dist={dist:.0f} mm")
        time.sleep(poll_s)

    log.append("[GOTO] aborted")
    _safe_stop(transport, log)
    return "aborted"


class _ScriptedTransport:
    """Fake transport whose ``command()`` calls advance a shared synthetic
    truth pose, deterministically modelling the camera observing the robot
    get closer to the target after each ``G`` re-issue -- decoupled from
    real sim physics/timing (Decision 2's "convergence-under-synthetic-truth"
    strategy). ``waypoints`` is consumed one entry per ``G`` command sent.
    """

    def __init__(
        self, state: dict, waypoints: list[tuple[float, float, float]]
    ) -> None:
        self._state = state
        self._waypoints = list(waypoints)
        self.commands: list[str] = []
        self.sends: list[str] = []

    def command(self, line: str, read_timeout: int = 200) -> str:  # [ms]
        self.commands.append(line)
        if line.startswith("G ") and self._waypoints:
            x_cm, y_cm, yaw_rad = self._waypoints.pop(0)
            self._state["last_truth"] = (x_cm, y_cm, yaw_rad, time.monotonic())
        return "OK"

    def send(self, line: str) -> None:
        self.sends.append(line)


def test_goto_converges_within_eps_and_sends_stop() -> None:
    """The pursuit loop terminates once within eps of the target and STOPs.

    Synthetic camera truth starts 1000 mm short of the target (0, 0) and
    advances to 500 mm then 10 mm short after each SI/G re-issue -- the
    loop must re-issue exactly twice, then detect arrival on the third
    truth read and issue STOP.
    """
    now = time.monotonic()
    state: dict = {"last_truth": (0.0, 0.0, 0.0, now)}
    transport = _ScriptedTransport(
        state, waypoints=[(50.0, 0.0, 0.0), (99.0, 0.0, 0.0)]
    )
    log: list[str] = []

    result = _run_goto(
        transport,
        state,
        stop_flag={},
        tx=1000,
        ty=0,
        eps=50,
        speed=200,
        log=log,
        poll_s=0.01,
        timeout_s=5.0,
    )

    assert result == "reached"
    assert transport.sends and transport.sends[-1] == "STOP", (
        "arrival must issue a STOP, per _GotoRunner._safe_stop()"
    )
    assert transport.commands == [
        "SI 0 0 0",
        "G 1000 0 200",
        "SI 500 0 0",
        "G 1000 0 200",
    ], f"expected exactly 2 SI/G round trips before arrival, got {transport.commands}"
    assert any("reached" in line for line in log)


def test_goto_stale_truth_does_not_crash_and_times_out_without_sending() -> None:
    """A truth pose older than TRUTH_MAX_AGE_S is ignored -- run() logs and
    waits, never treats it as fresh (no SI/G sent on it), and times out
    cleanly (issuing STOP) instead of crashing.
    """
    now = time.monotonic()
    # x_cm=100.0 -> cur_x=1000mm -- exactly AT the target. This pose would be
    # instantly "reached" (distance 0) if the loop treated it as fresh; using
    # it stale proves staleness -- not proximity -- is what gates the check.
    stale_truth = (100.0, 0.0, 0.0, now - (_TRUTH_MAX_AGE_S + 1.0))
    state = {"last_truth": stale_truth}
    transport = _ScriptedTransport(state, waypoints=[])
    log: list[str] = []

    result = _run_goto(
        transport,
        state,
        stop_flag={},
        tx=1000,
        ty=0,
        eps=50,  # irrelevant here -- distance would be 0 even at a tight eps
        speed=200,
        log=log,
        poll_s=0.01,
        timeout_s=0.15,
    )

    assert result == "timeout"
    assert transport.commands == [], (
        "a stale truth pose must never be treated as fresh -- no SI/G should "
        "have been sent"
    )
    assert transport.sends == ["STOP"], "a timeout must still issue a STOP"
    assert any("waiting" in line for line in log), (
        "run() must log while waiting on stale truth, not crash silently"
    )


def test_goto_missing_truth_does_not_crash_and_times_out_without_sending() -> None:
    """No truth pose at all (``state.get("last_truth")`` is ``None``) behaves
    the same as a stale one -- waits and logs, never crashes, times out
    cleanly.
    """
    state: dict = {}
    transport = _ScriptedTransport(state, waypoints=[])
    log: list[str] = []

    result = _run_goto(
        transport,
        state,
        stop_flag={},
        tx=1000,
        ty=0,
        eps=50,
        speed=200,
        log=log,
        poll_s=0.01,
        timeout_s=0.15,
    )

    assert result == "timeout"
    assert transport.commands == [], "no truth at all must never be treated as fresh"
    assert transport.sends == ["STOP"]
    assert any("waiting" in line for line in log)


def test_goto_explicit_stop_halts_promptly_without_waiting_for_arrival_or_timeout() -> None:
    """``stop()`` (a thread-safe flag, mirroring ``_GotoRunner.stop()``) must
    abort the loop immediately -- not wait for arrival (truth never advances
    here) or the timeout (30s, deliberately long) -- and still issue STOP.
    """
    now = time.monotonic()
    state = {"last_truth": (0.0, 0.0, 0.0, now)}  # 1000mm away; never advances
    transport = _ScriptedTransport(state, waypoints=[])  # no waypoints -> never "arrives"
    stop_flag: dict = {}
    log: list[str] = []

    def _click_stop_shortly() -> None:
        time.sleep(0.05)
        stop_flag["stop"] = True

    stopper = threading.Thread(target=_click_stop_shortly, daemon=True)
    stopper.start()

    t0 = time.monotonic()
    result = _run_goto(
        transport,
        state,
        stop_flag,
        tx=1000,
        ty=0,
        eps=50,
        speed=200,
        log=log,
        poll_s=0.01,
        timeout_s=30.0,  # must NOT be waited out
    )
    elapsed = time.monotonic() - t0
    stopper.join(timeout=1.0)

    assert result == "aborted"
    assert elapsed < 5.0, (
        f"stop() must halt the loop promptly, not wait for the 30s timeout "
        f"(took {elapsed:.2f}s)"
    )
    assert transport.sends and transport.sends[-1] == "STOP"
    assert any("aborted" in line for line in log)


# ---------------------------------------------------------------------------
# Part B -- confirmation against the real sim (architecture-update.md
# Decision 1 / Grounding fact 3): bare top-level STOP must cancel an
# in-flight Planner G goal, and must NOT be changed to DEV DT STOP.
# ---------------------------------------------------------------------------


@_requires_sim_lib
@pytest.mark.xfail(
    reason="097: G has no binary arm until sprint 098 (envelope.proto's "
           "`motion` oneof field is RESERVED, not declared, until "
           "Subsystems::Planner un-parks) -- `G 6000 0 200` is a gated "
           "no-op, so mode= never reports 'G'. See binary_bridge.py.",
    strict=False,
)
def test_stop_cancels_inflight_g_goal_against_real_sim() -> None:
    """``_safe_stop()``'s bare top-level ``STOP`` genuinely cancels
    ``Subsystems::Planner``'s active ``G`` goal -- confirmed against the
    real compiled firmware/sim, not just by source inspection.

    Sends the exact wire commands ``_GotoRunner`` uses (a real ``G``, then
    the exact bare ``"STOP"`` ``_safe_stop()`` sends) directly against a
    real ``SimTransport``, and watches telemetry ``mode=`` (084-005:
    ``Subsystems::Planner::state().mode`` is its sole source,
    ``telemetry_commands.cpp``) settle to and stay idle (``'I'``)
    afterward. If ``STOP`` had NOT cleared the goal -- the ``DEV DT STOP``
    regression Decision 1 explicitly guards against (084 Open Question 3:
    no arbitration between ``DEV DT`` and ``Planner``-issued motion) -- mode
    would keep reading ``'G'``.
    """
    transport = SimTransport()
    transport.on_log = lambda _s: None
    transport.connect()
    assert transport._connected, "SimTransport failed to connect -- is the sim lib built?"

    modes: list[str] = []
    transport.on_telemetry = lambda frame: modes.append((frame.mode or "").upper())

    try:
        time.sleep(0.3)  # let the tick-thread's startup (STREAM 50 ack) settle

        # A genuine in-flight goal: far enough it cannot have arrived by the
        # time we observe mode='G' below.
        transport.command("G 6000 0 200", read_timeout=500)

        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and "G" not in modes:
            time.sleep(0.02)
        assert "G" in modes, (
            "Planner never reported mode=G after a G command -- test setup "
            f"invalid (last modes observed: {modes[-20:]})"
        )

        # _safe_stop()'s exact call.
        transport.send("STOP")

        n_before = len(modes)
        deadline = time.monotonic() + 2.0
        while len(modes) < n_before + 20 and time.monotonic() < deadline:
            time.sleep(0.02)

        after = modes[n_before:]
        assert after, "no telemetry observed after STOP"
        assert all(m == "I" for m in after[-10:]), (
            f"Planner kept reporting an active goal after STOP: {after} -- "
            "the in-flight G goal was NOT cancelled (this is exactly the "
            "DEV DT STOP regression architecture-update.md Decision 1 warns "
            "against)"
        )
    finally:
        transport.disconnect()


# ---------------------------------------------------------------------------
# Part C -- real GUI end to end: the actual _GotoRunner/QThread/SimTransport
# stack (mirrors test_tour1_geometry.py's pattern), for the highest-fidelity
# confirmation of convergence and of the button-reenable-on-stop criterion.
# ---------------------------------------------------------------------------

#: Zero-error Sim Errors panel values -- see test_tour1_geometry.py (same
#: convention: additive/noise knobs 0.0, multiplicative knobs 1.0, trackwidth
#: matching the firmware's configured trackwidthMm).
_FIRMWARE_TRACKWIDTH = 128.0
_ZERO_ERROR_SPINS: dict[str, float] = {
    "sim_err_encoder_mm": 0.0,
    "sim_err_enc_scale_l": 0.0,
    "sim_err_enc_scale_r": 0.0,
    "sim_err_slip_turn": 0.0,
    "sim_err_body_rot_scrub": 1.0,
    "sim_err_body_lin_scrub": 1.0,
    "sim_err_motor_offset_l": 1.0,
    "sim_err_motor_offset_r": 1.0,
    "sim_err_trackwidth": _FIRMWARE_TRACKWIDTH,
    "sim_err_otos_linear": 0.0,
    "sim_err_otos_yaw": 0.0,
    "sim_err_otos_lin_scale": 0.0,
    "sim_err_otos_ang_scale": 0.0,
    "sim_err_otos_lin_drift": 0.0,
    "sim_err_otos_yaw_drift": 0.0,
}

#: Wall-clock ceiling for a real GOTO to finish (generous -- real 1x sim
#: pacing, ~900mm at 250mm/s plus SI/G/poll overhead is a few seconds).
_GOTO_DEADLINE_S = 30.0
#: Slack added on top of the configured eps when checking final ground-truth
#: distance -- the loop's own eps check runs against the truth sample AT the
#: time of the check; a small amount of additional coast before the next
#: truth delivery is expected and is not a convergence failure.
_GOTO_SLACK_MM = 150.0


@pytest.fixture(scope="module")
def qapp():
    """QApplication for the module (offscreen platform set by conftest)."""
    import sys

    from PySide6.QtWidgets import QApplication  # type: ignore[import-untyped]

    app = QApplication.instance() or QApplication(sys.argv)
    yield app


def _spin_events(qapp, seconds: float) -> None:
    """Process Qt events for ``seconds`` of wall time."""
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        qapp.processEvents()
        time.sleep(0.01)


def _spin_until(qapp, predicate, timeout_s: float) -> bool:
    """Process Qt events until ``predicate()`` is true or timeout."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        qapp.processEvents()
        time.sleep(0.01)
    return predicate()


def _connect_sim(qapp, monkeypatch, tmp_path):
    """Build the real GUI window and connect a real ``SimTransport``.

    Mirrors ``test_tour1_geometry.py``'s setup: pins the active robot to a
    literal, uncalibrated ("nocal") config and redirects ``sim_prefs``
    persistence to ``tmp_path`` (independent of the repo's own
    ``active_robot.json``/``sim_error_profile.json``), then zeroes every Sim
    Errors panel knob via the real spinboxes + Apply for a deterministic run.
    Teleports the plant to world (0, 0, 0) for a known starting pose (there
    is no camera/operator in Sim mode -- see ``_set_origin``'s own doc
    comment for the same rationale).

    Returns ``(window, transport)``. Caller must disconnect/hide the window
    and reset the robot-config singleton in a ``finally`` block (see each
    test body).
    """
    from PySide6.QtWidgets import (  # type: ignore[import-untyped]
        QComboBox,
        QDoubleSpinBox,
        QPushButton,
    )

    import robot_radio.testgui.__main__ as gui_main
    from robot_radio.config import robot_config as rc_mod
    from robot_radio.testgui import sim_prefs
    from robot_radio.testgui import transport as transport_mod

    baked_cfg = tmp_path / "baked_tovez_nocal.json"
    baked_cfg.write_text(json.dumps({
        "schema_version": 2,
        "identity": {"robot_name": "tovez-nocal-baked", "uid": "tovez-nocal-baked"},
        "connection": {"device_announcement_name": "tovez"},
        "geometry": {"trackwidth": 128},
        "calibration": {},
    }))
    monkeypatch.setenv("ROBOT_CONFIG", str(baked_cfg))
    rc_mod._reset_robot_config()

    monkeypatch.setattr(sim_prefs, "_PREFS_DIR", tmp_path)
    monkeypatch.setattr(sim_prefs, "_PREFS_PATH", tmp_path / "sim_error_profile.json")

    _RealSimTransport = transport_mod.SimTransport
    created: list = []

    class _CapturingSimTransport(_RealSimTransport):
        def __init__(self) -> None:
            super().__init__()
            created.append(self)

    monkeypatch.setattr(transport_mod, "SimTransport", _CapturingSimTransport)

    window, _app = gui_main._build_main_window()

    combo = window.findChild(QComboBox, "transport_combo")
    assert combo is not None
    combo.setCurrentText("Sim")

    connect_btn = window.findChild(QPushButton, "connect_btn")
    assert connect_btn is not None
    connect_btn.click()
    _spin_events(qapp, 0.3)

    assert created, "Connect did not construct a SimTransport"
    transport = created[-1]
    assert transport._connected, "SimTransport failed to connect -- is the sim lib built?"

    for name, value in _ZERO_ERROR_SPINS.items():
        spin = window.findChild(QDoubleSpinBox, name)
        assert spin is not None, f"Sim Errors spinbox {name!r} not found"
        spin.setValue(value)
    apply_btn = window.findChild(QPushButton, "sim_errors_apply_btn")
    assert apply_btn is not None
    apply_btn.click()
    _spin_events(qapp, 0.3)

    # Known starting pose (see docstring).
    transport.set_true_pose(0.0, 0.0, 0.0)
    _spin_events(qapp, 0.2)

    return window, transport


def _disconnect_sim(qapp, window) -> None:
    from PySide6.QtWidgets import QPushButton  # type: ignore[import-untyped]
    from robot_radio.config import robot_config as rc_mod

    disconnect_btn = window.findChild(QPushButton, "disconnect_btn")
    if disconnect_btn is not None and disconnect_btn.isEnabled():
        disconnect_btn.click()
        _spin_events(qapp, 0.3)
    window.hide()
    rc_mod._reset_robot_config()


@_requires_sim_lib
@pytest.mark.xfail(
    reason="097: GOTO's pursuit loop needs SI/G, neither of which has a "
           "binary arm until sprint 098 -- goto_btn is now permanently "
           "disabled (gated in the UI, __main__.py), so it never starts. "
           "See binary_bridge.py / the goto_btn tooltip set in __main__.py.",
    strict=False,
)
def test_goto_button_converges_against_real_sim_and_reenables_button(
    qapp, monkeypatch, tmp_path
) -> None:
    """Clicking GOTO with a real target drives the real ``_GotoRunner``
    against a real ``SimTransport``: the loop converges (ground truth ends
    within ``eps`` + slack of the target) and the GOTO button re-enables
    once it finishes -- the real end-to-end confirmation that
    "the runner is started against a SimTransport" and actually converges.
    """
    from PySide6.QtWidgets import QPushButton, QSpinBox  # type: ignore[import-untyped]

    window, transport = _connect_sim(qapp, monkeypatch, tmp_path)
    try:
        truth_log: list[tuple[float, float, float]] = []  # (x_mm, y_mm, h_rad)
        gui_truth_cb = transport.on_truth

        def _truth_spy(pose) -> None:
            if pose is not None:
                x_cm, y_cm, h_rad = pose
                truth_log.append((x_cm * 10.0, y_cm * 10.0, h_rad))
            if gui_truth_cb is not None:
                gui_truth_cb(pose)

        transport.on_truth = _truth_spy

        target_x, target_y, eps, speed = 900, 0, 80, 250
        for name, value in (
            ("x", target_x), ("y", target_y), ("eps", eps), ("speed", speed),
        ):
            spin = window.findChild(QSpinBox, f"goto_spin_{name}")
            assert spin is not None, f"goto_spin_{name} not found"
            spin.setValue(value)

        goto_btn = window.findChild(QPushButton, "goto_btn")
        assert goto_btn is not None
        assert goto_btn.isEnabled(), "GOTO button not enabled after connect"

        goto_btn.click()
        assert _spin_until(qapp, lambda: not goto_btn.isEnabled(), 5.0), (
            "GOTO never started (button did not disable)"
        )
        assert _spin_until(qapp, goto_btn.isEnabled, _GOTO_DEADLINE_S), (
            f"GOTO did not finish within {_GOTO_DEADLINE_S:.0f}s wall clock "
            "-- see the [GOTO] log lines for why"
        )
        _spin_events(qapp, 0.3)

        assert truth_log, "no ground truth observed -- did on_truth ever fire?"
        fx, fy, _fh = truth_log[-1]
        dist = math.hypot(fx - target_x, fy - target_y)
        assert dist <= eps + _GOTO_SLACK_MM, (
            f"GOTO ended {dist:.0f} mm from target ({target_x}, {target_y}) "
            f"-- eps={eps} + slack={_GOTO_SLACK_MM:.0f}"
        )
    finally:
        _disconnect_sim(qapp, window)


@_requires_sim_lib
@pytest.mark.xfail(
    reason="097: goto_btn is now permanently disabled (gated pending "
           "sprint 098 -- GOTO needs SI/G, neither has a binary arm yet), "
           "so it can never be enabled after connect. See __main__.py's "
           "goto_btn tooltip / _send_buttons wiring.",
    strict=False,
)
def test_goto_stop_reenables_button_synchronously_against_real_sim(
    qapp, monkeypatch, tmp_path
) -> None:
    """Stopping a running GOTO (Operations panel STOP) re-enables the GOTO
    button synchronously -- mirrors ``_stop_tour``'s fix
    (``testgui-tour-stop-reactivation.md``) -- exercised here against the
    REAL ``_GotoRunner``/``QThread``/``SimTransport`` stack, complementing
    ``test_tour_stop.py``'s deterministic fake-widget version of the same
    assertion (``test_stop_goto_reenables_button_synchronously``).
    """
    from PySide6.QtWidgets import QPushButton, QSpinBox  # type: ignore[import-untyped]

    window, transport = _connect_sim(qapp, monkeypatch, tmp_path)
    try:
        # A FAR target that cannot possibly arrive before STOP is clicked.
        for name, value in (("x", 8000), ("y", 0), ("eps", 20), ("speed", 200)):
            spin = window.findChild(QSpinBox, f"goto_spin_{name}")
            assert spin is not None, f"goto_spin_{name} not found"
            spin.setValue(value)

        goto_btn = window.findChild(QPushButton, "goto_btn")
        ops_stop_btn = window.findChild(QPushButton, "ops_btn_stop")
        assert goto_btn is not None and goto_btn.isEnabled()
        assert ops_stop_btn is not None and ops_stop_btn.isEnabled(), (
            "Operations STOP button not enabled after connect"
        )

        goto_btn.click()
        assert _spin_until(qapp, lambda: not goto_btn.isEnabled(), 5.0), (
            "GOTO never started (button did not disable)"
        )

        # Let it actually pursue for a moment before stopping mid-flight.
        _spin_events(qapp, 1.0)

        ops_stop_btn.click()

        # No additional _spin_until wait: the re-enable must already have
        # happened synchronously inside the click (mirrors
        # test_tour1_geometry.py's identical tour-stop assertion).
        assert goto_btn.isEnabled(), (
            "GOTO button was not re-enabled synchronously by Operations STOP"
        )
    finally:
        _disconnect_sim(qapp, window)
