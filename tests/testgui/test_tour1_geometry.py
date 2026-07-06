"""tests/testgui/test_tour1_geometry.py — the tour buttons must actually tour
(ported from ``tests_old/testgui/`` per sprint 085 ticket 002).

End-to-end headless GUI tests (``QT_QPA_PLATFORM=offscreen``, the real ctypes
firmware sim, built against ``source/`` by sprint 084). This is the FIRST time
Tour 1 / Tour 2 have run against real sprint-084 firmware/sim
(``architecture-update.md`` Grounding fact 1) — everything here (the
``_TourRunner`` worker, the ``SNAP``-poll ``mode=I`` completion detection, the
``D``/``RT`` wire verbs) predates the greenfield rebuild and was dormant
until now.

What changed from the ``tests_old`` version of this file
----------------------------------------------------------
The pre-rebuild file asserted a STRICT per-waypoint/final-heading trace
(``xfail(strict=True)``), root-caused to ``source_old``'s ``rotationalSlip``
(0.92) being baked into the compiled firmware ``DefaultConfig`` in a way GUI
robot selection could never override. Direct investigation this ticket
(reproduced against both the real GUI/``SimTransport``/``QThread`` stack AND,
independently, the raw ``tests/_infra/sim`` ``firmware.Sim`` ctypes wrapper —
same magnitude of drift in both, ruling out a GUI/threading bug) found that
per-waypoint/heading tracing STILL does not hold exactly in this tree either
— but for a different, already-diagnosed and explicitly out-of-scope reason:

  * The active robot (``data/robots/tovez_nocal.json`` — "no calibration")
    pushes ``SET rotSlip=0`` on Connect (the documented no-correction
    sentinel — ``calibration/push.py``'s ``calibration_commands()``), which
    ``config_commands.cpp`` accepts and ``PoseEstimator::effectiveSlip()``
    maps to ``1.0`` (no slip correction) — so, unlike ``source_old``, the
    new tree's firmware default is NOT pre-loaded with a rotational-slip
    inflation the GUI can't reach. This part of the OLD bug is fixed.
  * ``RT``'s own stop condition, however, is deliberately open-loop and
    slip-uncorrected by design — ``source/commands/motion_commands.cpp``'s
    ``handleRT`` doc comment: "closed-loop against the per-wheel encoder
    arc ... minus its rotational-slip/coast-anticipation refinement ...
    coast-anticipation is not part of this ticket's [084-003] acceptance
    bar". ``tests/sim/unit/test_motion_commands_arc_turn.py`` independently
    measures and documents a single isolated ``RT 9000`` overshooting by
    ~4-5° from the SMOOTH-stop ramp's coast (its own tolerance is ±10°).
    Chained across a 6-7 turn tour, with each ``D``/``RT`` step dispatched
    immediately once ``mode=I`` is observed (i.e. before any residual coast
    fully settles), this compounds into tens of degrees of final-heading
    drift — a real, repeatable firmware/motion-control characteristic, not
    a tour-plumbing bug, and explicitly deferred by ticket 084-003's own
    architecture decision ("no coast-anticipation this ticket"). It is out
    of scope for sprint 085 (host-only; this ticket's job is SNAP-poll
    completion verification, not motion-control accuracy tuning).

Per this ticket's (085-002) own acceptance criteria, the bar here is
therefore intentionally the SOFTER one the ticket text itself states: each
tour "runs to completion ... with no step timing out, and the robot's fused
pose ends near world origin (the tour is a closed geometric loop)" —
position-only closure, not exact waypoint/heading tracing. Measured (see
each test below): Tour 1 ends within ~20-40 mm of the origin; Tour 2 within
~95-175 mm (looser — six-plus RT legs compound more drift than Tour 1's;
still a small fraction of the tour's own leg lengths, up to 850 mm). No
production code was changed to reach this — this ticket is a real-firmware
verification pass that found the tour-completion plumbing (SNAP-poll
``mode=I``, stale-frame rejection, stop-button reactivation) already correct;
the residual heading drift above is a pre-existing, already-documented,
different-ticket's concern.

Fused pose, not ground truth
-----------------------------
The ticket's acceptance criterion is phrased "fused pose" — the firmware's
own ``TLM``/``SNAP`` ``pose=`` field (``PoseEstimator::fusedPose()``,
confirmed by reading ``source/commands/telemetry_commands.cpp``), i.e. what
an operator watching the GUI's avatar actually sees — not the sim's
ground-truth plant pose. This file reads it via a spy on
``transport.on_telemetry`` (the very cache ``_wait_for_idle`` itself polls),
and separately records the ground-truth trace for informational span/
sanity-checking only (proving the tour actually drove somewhere, not that it
idled at the origin the whole time).

Zero-error Sim profile, pinned nocal config
---------------------------------------------
Every Sim Errors panel knob is zeroed via the real spinboxes + Apply (as the
pre-rebuild file did) so the run is reproducible: every additive/noise knob
0.0, every multiplicative knob 1.0, and the plant trackwidth equal to the
firmware's configured ``trackwidthMm`` (128.0) so plant geometry matches the
firmware's kinematic calibration. The active robot config is pinned (via
``ROBOT_CONFIG``) to a literal, uncalibrated ("nocal") config matching
``data/robots/tovez_nocal.json``'s relevant fields, independent of whatever
the repo's ``active_robot.json`` pointer happens to select.

No sim fast-forward hack: real-time pacing
--------------------------------------------
Unlike the pre-rebuild file (which re-paced ``SimTransport``'s tick-thread
~5x via ``_SIM_TICK_SLEEP_S``), this file runs the sim at real 1x wall-clock
pacing. The GUI's own polling windows (``_TourRunner.SPINUP_S``/``POLL_S``)
are real host-clock sleeps, NOT scaled by the sim's tick-thread pacing — at
5x, a fixed 0.3 s ``POLL_S`` window covers 5x more *simulated* time than at
1x, which measurably changes how much residual per-step coast has settled
by the time the next command is dispatched (confirmed empirically: the same
tour's final-pose numbers shift beyond noise between 1x and 5x pacing).
Real 1x pacing is what an operator (or the radio relay / bench, whose SNAP
round-trip is real-time by nature) actually experiences, so it is what this
file measures against. Each tour therefore takes ~30-45 s wall-clock to run.

Run:
    QT_QPA_PLATFORM=offscreen uv run pytest tests/testgui/test_tour1_geometry.py -v
"""
from __future__ import annotations

import json
import math
import pathlib
import time

import pytest

_REPO = pathlib.Path(__file__).parent.parent.parent
_SIM_BUILD = _REPO / "tests" / "_infra" / "sim" / "build"

_LIB_PRESENT = any(
    (_SIM_BUILD / name).exists()
    for name in ("libfirmware_host.dylib", "libfirmware_host.so")
)

pytestmark = pytest.mark.skipif(
    not _LIB_PRESENT,
    reason="firmware sim lib not built (tests/_infra/sim: cmake --build build)",
)

# ---------------------------------------------------------------------------
# Zero-error Sim Errors panel values (spinbox objectName -> value).
#
# Additive/noise knobs zero; multiplicative knobs 1.0; trackwidth matches the
# firmware's configured trackwidthMm (128.0 — pose_estimator.h / sim_prefs.py)
# so the plant's geometry agrees with the firmware's kinematic calibration.
# ---------------------------------------------------------------------------
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

#: How near the origin the firmware's FUSED pose must end up, in mm, for a
#: tour to count as "a closed geometric loop" per this ticket's acceptance
#: criterion. Measured (see module docstring): Tour 1 ~20-40 mm, Tour 2
#: ~95-175 mm across repeated runs — this tolerance keeps ~1.7x headroom over
#: the highest observed value while still being a small fraction of the
#: tour's own leg lengths (up to 850 mm), so it is a meaningful "returned
#: near the start", not a rubber-stamp.
_ORIGIN_TOL_MM = 300.0

#: Wall-clock ceilings (real 1x sim pacing — see module docstring).
_TOUR_START_DEADLINE_S = 10.0
_TOUR_DEADLINE_S = 90.0


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


def _run_tour_headless(qapp, monkeypatch, tmp_path, button_name: str):
    """Zero the Sim Errors panel via the GUI and run one tour button.

    Returns ``(truth_trace, fused_pose)``:

    - ``truth_trace``: the recorded plant ground-truth trace as (x, y, h)
      tuples (mm, mm, rad) — informational only (span sanity-check), NOT the
      value asserted against ``_ORIGIN_TOL_MM``.
    - ``fused_pose``: the LAST ``TLMFrame.pose`` observed — (x, y, h) in
      (mm, mm, cdeg) — the firmware's own fused pose, read via a spy on
      ``transport.on_telemetry`` (the same ``state["last_tlm"]`` cache
      ``_wait_for_idle`` itself polls). ``None`` if no TLM frame with a pose
      was ever observed (a real failure — every SNAP reply in this tree
      unconditionally carries ``pose=``, per ``telemetry_commands.cpp``).

    Everything runs through the real GUI objects — the transport combo +
    Connect button, the Sim Errors spinboxes + Apply, the tour QPushButtons,
    ``_TourRunner`` on its QThread, and ``SimTransport``'s tick-thread —
    exactly the path an operator exercises.
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

    # Pin the active robot to a literal, uncalibrated ("nocal") config
    # matching data/robots/tovez_nocal.json's relevant fields: Connect now
    # pushes the active robot's calibration to the firmware (SET rotSlip=0
    # for an uncalibrated robot — the documented no-correction sentinel), and
    # this test's expectations are defined against that push; the pin keeps
    # the test independent of the repo's active_robot.json pointer (operator
    # state) regardless of which robot is currently selected on disk.
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

    # Keep the operator's persisted error profile untouched: point sim_prefs
    # persistence at a temp file for the whole test (the panel's Apply saves
    # it, and SimTransport.connect() re-loads it).
    monkeypatch.setattr(sim_prefs, "_PREFS_DIR", tmp_path)
    monkeypatch.setattr(
        sim_prefs, "_PREFS_PATH", tmp_path / "sim_error_profile.json"
    )

    # The GUI keeps its live transport in a closure with no accessor; the
    # seam is the SimTransport name _build_main_window imports (function-
    # locally) from the transport module, so the patch must land BEFORE the
    # window is built.  The subclass must be named exactly "SimTransport":
    # operations.is_sim_transport() duck-checks type(t).__name__, and the
    # origin-reset plant teleport hangs off that check.
    _RealSimTransport = transport_mod.SimTransport
    created: list = []

    class SimTransport(_RealSimTransport):
        def __init__(self) -> None:
            super().__init__()
            created.append(self)

    monkeypatch.setattr(transport_mod, "SimTransport", SimTransport)

    window, _app = gui_main._build_main_window()
    truth: list[tuple[float, float, float]] = []  # (x, y, h) in (mm, mm, rad)
    fused: dict = {"pose": None}

    try:
        combo = window.findChild(QComboBox, "transport_combo")
        assert combo is not None, "transport_combo not found"
        combo.setCurrentText("Sim")

        connect_btn = window.findChild(QPushButton, "connect_btn")
        assert connect_btn is not None, "connect_btn not found"
        connect_btn.click()
        _spin_events(qapp, 0.3)

        assert created, "Connect did not construct a SimTransport"
        transport = created[-1]
        assert transport._connected, "SimTransport failed to connect"

        # Record every plant ground-truth delivery (tick-thread callback),
        # chaining to the GUI's own handler so the canvas still updates.
        gui_truth_cb = transport.on_truth

        def _truth_spy(pose) -> None:
            if pose is not None:
                x_cm, y_cm, h_rad = pose
                truth.append((x_cm * 10.0, y_cm * 10.0, h_rad))
            if gui_truth_cb is not None:
                gui_truth_cb(pose)

        transport.on_truth = _truth_spy

        # Record every TLM frame (same cache _wait_for_idle polls) so we can
        # read the LAST fused pose observed — chaining to the GUI's own
        # handler so _state["last_tlm"] (and the canvas) still update.
        gui_tlm_cb = transport.on_telemetry

        def _tlm_spy(frame) -> None:
            pose = getattr(frame, "pose", None)
            if pose is not None:
                fused["pose"] = pose
            if gui_tlm_cb is not None:
                gui_tlm_cb(frame)

        transport.on_telemetry = _tlm_spy

        # Zero every error knob through the real spinboxes, then Apply.
        for name, value in _ZERO_ERROR_SPINS.items():
            spin = window.findChild(QDoubleSpinBox, name)
            assert spin is not None, f"Sim Errors spinbox {name!r} not found"
            spin.setValue(value)
        apply_btn = window.findChild(QPushButton, "sim_errors_apply_btn")
        assert apply_btn is not None, "sim_errors_apply_btn not found"
        apply_btn.click()
        # The apply action runs on the sim tick-thread via the command queue;
        # give it a moment to land before the tour starts.
        _spin_events(qapp, 0.3)

        tour_btn = window.findChild(QPushButton, button_name)
        stop_btn = window.findChild(QPushButton, "stop_tour_btn")
        assert tour_btn is not None, f"tour button {button_name!r} not found"
        assert stop_btn is not None
        assert tour_btn.isEnabled(), f"{button_name} not enabled after connect"

        n_truth_before = len(truth)
        tour_btn.click()

        assert _spin_until(qapp, stop_btn.isEnabled, _TOUR_START_DEADLINE_S), (
            "tour never started (Stop Tour button did not enable)"
        )
        assert _spin_until(
            qapp, lambda: not stop_btn.isEnabled(), _TOUR_DEADLINE_S
        ), (
            f"tour did not finish within {_TOUR_DEADLINE_S:.0f} s wall clock "
            "— a step timed out (see the [TOUR] log lines for which one)"
        )
        # Let the final truth/TLM deliveries drain.
        _spin_events(qapp, 0.5)
    finally:
        disconnect_btn = window.findChild(QPushButton, "disconnect_btn")
        if disconnect_btn is not None and disconnect_btn.isEnabled():
            disconnect_btn.click()
            _spin_events(qapp, 0.3)
        window.hide()
        # Drop the pinned-config singleton so later tests re-resolve.
        rc_mod._reset_robot_config()

    return truth[n_truth_before:], fused["pose"]


def _assert_tour_ran_and_closed_the_loop(trace, fused_pose, tour_name: str) -> None:
    """Sanity-check the plant actually moved, then assert fused-pose closure.

    ``trace`` (ground truth) is used ONLY for the span sanity check — proving
    the tour actually drove around, not that it idled at the origin the
    whole time. The closure assertion itself is against ``fused_pose`` (the
    firmware's own TLM ``pose=``), per this ticket's acceptance criterion.
    """
    assert len(trace) > 50, (
        f"expected a dense ground-truth trace over {tour_name}, got "
        f"{len(trace)} samples — did the sim truth callback run?"
    )

    span_x = max(p[0] for p in trace) - min(p[0] for p in trace)
    span_y = max(p[1] for p in trace) - min(p[1] for p in trace)
    assert span_x > 300.0 and span_y > 200.0, (
        f"plant barely moved (span {span_x:.0f} x {span_y:.0f} mm) — "
        f"{tour_name} did not run"
    )

    assert fused_pose is not None, (
        f"no fused pose (TLM pose=) was ever observed during {tour_name} — "
        "SNAP should unconditionally carry pose= (telemetry_commands.cpp)"
    )
    fx, fy, fh_cdeg = fused_pose
    dist = math.hypot(fx, fy)
    assert dist <= _ORIGIN_TOL_MM, (
        f"{tour_name}'s fused pose ended {dist:.0f} mm from world origin "
        f"(x={fx}, y={fy}, h={fh_cdeg / 100.0:.1f} deg) — tolerance is "
        f"{_ORIGIN_TOL_MM:.0f} mm (see module docstring for measured range "
        "and rationale)"
    )


def test_tour1_completes_and_fused_pose_returns_near_origin(qapp, monkeypatch, tmp_path):
    """Tour 1 runs to completion (no step timeout) and closes the loop.

    Measured fused-pose distance from origin across repeated runs: ~20-40 mm
    (see module docstring) — well inside ``_ORIGIN_TOL_MM``.
    """
    trace, fused_pose = _run_tour_headless(
        qapp, monkeypatch, tmp_path, "tour_btn_tour_1"
    )
    _assert_tour_ran_and_closed_the_loop(trace, fused_pose, "Tour 1")


def test_tour2_completes_and_fused_pose_returns_near_origin(qapp, monkeypatch, tmp_path):
    """Tour 2 runs to completion (no step timeout) and closes the loop.

    Tour 2 has more (and larger, mixed-sign) RT legs than Tour 1, so its
    measured fused-pose distance from origin is looser: ~95-175 mm across
    repeated runs (see module docstring) — still inside ``_ORIGIN_TOL_MM``
    and a small fraction of its own leg lengths (up to 850 mm).
    """
    trace, fused_pose = _run_tour_headless(
        qapp, monkeypatch, tmp_path, "tour_btn_tour_2"
    )
    _assert_tour_ran_and_closed_the_loop(trace, fused_pose, "Tour 2")


def test_stopping_a_running_tour_reenables_buttons_synchronously(
    qapp, monkeypatch, tmp_path
):
    """Clicking Stop Tour mid-run re-enables the tour buttons immediately.

    Complements ``test_tour_stop.py``'s inline re-implementation (which
    exercises the exact ``_stop_tour()`` control flow deterministically
    against fake worker/thread doubles) by clicking the REAL Stop Tour
    button against a live, running ``SimTransport``-backed tour — the
    ``QPushButton.clicked`` signal is a same-thread (GUI-thread) direct
    connection, so ``_stop_tour()`` runs synchronously inside ``.click()``;
    no additional event-loop spin should be needed before the buttons
    reflect the stopped state (acceptance criterion: re-enable is
    synchronous, "not dependent on the finished signal being delivered
    during the blocking thread.wait()" — testgui-tour-stop-reactivation.md).
    """
    from PySide6.QtWidgets import (  # type: ignore[import-untyped]
        QComboBox,
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
    monkeypatch.setattr(
        sim_prefs, "_PREFS_PATH", tmp_path / "sim_error_profile.json"
    )

    _RealSimTransport = transport_mod.SimTransport
    created: list = []

    class SimTransport(_RealSimTransport):
        def __init__(self) -> None:
            super().__init__()
            created.append(self)

    monkeypatch.setattr(transport_mod, "SimTransport", SimTransport)

    window, _app = gui_main._build_main_window()

    try:
        combo = window.findChild(QComboBox, "transport_combo")
        combo.setCurrentText("Sim")
        connect_btn = window.findChild(QPushButton, "connect_btn")
        connect_btn.click()
        _spin_events(qapp, 0.3)
        assert created and created[-1]._connected, "SimTransport failed to connect"

        # Tour 2 (the longer tour) so there is ample time to click Stop
        # mid-flight, well before natural completion.
        tour_btn = window.findChild(QPushButton, "tour_btn_tour_2")
        stop_btn = window.findChild(QPushButton, "stop_tour_btn")
        assert tour_btn.isEnabled()

        tour_btn.click()
        assert _spin_until(qapp, stop_btn.isEnabled, _TOUR_START_DEADLINE_S), (
            "tour never started (Stop Tour button did not enable)"
        )
        assert not tour_btn.isEnabled(), "tour buttons should disable once running"

        # Let the tour actually drive for a moment, then stop it mid-flight.
        _spin_events(qapp, 1.0)
        stop_btn.click()

        # No additional _spin_until wait: the re-enable must already have
        # happened synchronously inside stop_btn.click() itself.
        assert tour_btn.isEnabled(), (
            "Tour button was not re-enabled synchronously by Stop Tour"
        )
        assert not stop_btn.isEnabled(), (
            "Stop Tour button should disable itself once stopped"
        )
    finally:
        disconnect_btn = window.findChild(QPushButton, "disconnect_btn")
        if disconnect_btn is not None and disconnect_btn.isEnabled():
            disconnect_btn.click()
            _spin_events(qapp, 0.3)
        window.hide()
        rc_mod._reset_robot_config()
