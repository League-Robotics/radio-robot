"""tests/testgui/test_tour1_geometry.py — the tour buttons must actually tour.

End-to-end headless GUI tests (QT_QPA_PLATFORM=offscreen, real ctypes
firmware sim).  These are the acceptance tests for the tour buttons'
*geometry*, not their plumbing: with every Sim Errors knob at its zero-error
value, clicking a tour button must drive the plant ground truth through the
tour's dead-reckoned waypoints, in order, and end on the final waypoint at
the final heading.

Expected waypoints are DERIVED from the tour's wire strings (``RT`` adds a
relative heading, ``TURN`` sets an absolute one, ``D`` advances along the
current heading) so the tests track ``commands.TOUR_*`` automatically.

Tour 1 traces the four colored field corners and returns to the first:
"orange → purple → blue → green → orange" (NE ~(297,297) → NW → SW → SE →
NE).  Tour 2 is a closed loop of six left 90° turns that must end back at
the origin facing west.

Zero-error here means what the operator can reach through the Sim Errors
panel: every additive/noise knob 0.0, every multiplicative knob 1.0, and the
plant trackwidth equal to the firmware's configured ``trackwidthMm`` (128.0,
``DefaultConfig.cpp``) so plant geometry matches the firmware's kinematic
calibration.  Everything runs through the real GUI objects — the transport
combo + Connect button, the Sim Errors spinboxes + Apply, the tour
QPushButtons, ``_TourRunner`` on its QThread, and ``SimTransport``'s
tick-thread — exactly the path an operator exercises.

Currently XFAIL (strict): the tours do NOT trace their shapes even at zero
error.  Root cause (measured 2026-07-02/03, tests/_infra sim, zero-error
profile; being fixed by sprint 073 "sim turn accuracy"):

  * ``RT`` is an open-loop encoder-arc turn (``Planner::beginRotation``,
    source/control/PlannerBegin.cpp).  Its per-wheel arc target is inflated
    by the firmware calibration constant ``rotationalSlip`` (default 0.92,
    baked into the sim firmware's DefaultConfig.cpp — GUI robot selection
    does NOT change it) — compensation for real-world wheel scrub that the
    zero-error plant does not have.
  * The coast anticipation was a stale hand-tuned constant
    (``kRtCoastArcMm=8.0``, tuned for 100°/s while ``yawRateMax=70`` caps
    the spin) — replaced by ramp-dynamics math in ticket 073-001.
  * Residual after 073-001 (measured): ~+1.1–1.4° per RT from
    stop-condition tick quantization (the ROTATION stop is polled once per
    control tick; ~1.4–1.7° passes per tick at 70°/s).
  * ``TURN`` (closed-loop on the fused heading) lands within ~1° and is not
    the problem.  Open-loop RT legs accumulate: measured Tour 2
    return-to-origin miss at zero error is ~122 mm with heading +56° off
    (six RT 9000s at ~+99.3° each on the 073-001 work tree).

Sprint 073 ticket 004 owns removing these markers once tickets 001–003
land — strict=True makes an unexpected pass loud.

Run:
    uv run --group gui python -m pytest tests/testgui/test_tour1_geometry.py -v
"""
from __future__ import annotations

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
# firmware's configured trackwidthMm (DefaultConfig.cpp: 128.0) so the plant's
# geometry agrees with the firmware's kinematic calibration.
# ---------------------------------------------------------------------------
_FIRMWARE_TRACKWIDTH_MM = 128.0

_ZERO_ERROR_SPINS: dict[str, float] = {
    "sim_err_encoder_mm": 0.0,
    "sim_err_enc_scale_l": 0.0,
    "sim_err_enc_scale_r": 0.0,
    "sim_err_slip_turn": 0.0,
    "sim_err_body_rot_scrub": 1.0,
    "sim_err_body_lin_scrub": 1.0,
    "sim_err_motor_offset_l": 1.0,
    "sim_err_motor_offset_r": 1.0,
    "sim_err_trackwidth": _FIRMWARE_TRACKWIDTH_MM,
    "sim_err_otos_linear": 0.0,
    "sim_err_otos_yaw": 0.0,
    "sim_err_otos_lin_scale": 0.0,
    "sim_err_otos_ang_scale": 0.0,
    "sim_err_otos_lin_drift": 0.0,
    "sim_err_otos_yaw_drift": 0.0,
}

#: How close (mm) the ground-truth trace must pass to each waypoint, in
#: order.  Perfect execution dwells ON each waypoint (turn-in-place), and
#: truth is sampled every 200 sim-ms (<= 40 mm apart at 200 mm/s), so this
#: is generous; the current turn bug misses late waypoints by >100 mm.
_WAYPOINT_TOL_MM = 60.0
_FINAL_HEADING_TOL_DEG = 5.0

#: Wall-clock ceilings (the sim is re-paced ~5x wall speed).
_TOUR_START_DEADLINE_S = 10.0
_TOUR_DEADLINE_S = 240.0


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


def _wrap_deg(d: float) -> float:
    """Wrap a degree difference into (-180, 180]."""
    return ((d + 180.0) % 360.0) - 180.0


def _ideal_waypoints(tour: list[str]) -> tuple[list[tuple[float, float]], float]:
    """Dead-reckon a tour's wire strings into ideal waypoints.

    Returns (waypoints, final_heading_deg): one (x_mm, y_mm) waypoint per
    ``D`` leg (position after the drive), and the heading at tour end.
    Heading 0 = +x (east), CCW positive — the origin-reset convention.
    """
    x = y = h = 0.0
    pts: list[tuple[float, float]] = []
    for cmd in tour:
        parts = cmd.split()
        if parts[0] == "RT":
            h += math.radians(int(parts[1]) / 100.0)
        elif parts[0] == "TURN":
            h = math.radians(int(parts[1]) / 100.0)
        elif parts[0] == "D":
            d = float(parts[3])
            x += d * math.cos(h)
            y += d * math.sin(h)
            pts.append((x, y))
        else:
            raise ValueError(f"unknown tour wire command: {cmd!r}")
    return pts, math.degrees(h)


def _run_tour_headless(qapp, monkeypatch, tmp_path, button_name: str):
    """Zero the Sim Errors panel via the GUI and run one tour button.

    Returns the recorded plant ground-truth trace as (x_mm, y_mm, h_rad)
    tuples covering the tour run.
    """
    import json

    from PySide6.QtWidgets import (  # type: ignore[import-untyped]
        QComboBox,
        QDoubleSpinBox,
        QPushButton,
    )

    import robot_radio.testgui.__main__ as gui_main
    from robot_radio.config import robot_config as rc_mod
    from robot_radio.testgui import sim_prefs
    from robot_radio.testgui import transport as transport_mod

    # Pin the active robot to a literal config whose calibration EQUALS the
    # sim firmware's compiled-in DefaultConfig (rotationalSlip=0.92,
    # trackwidth=128): Connect now pushes the active robot's calibration to
    # the firmware, and these tests' measured expectations are defined
    # against the baked values — the pin makes that push a no-op change and
    # keeps the test independent of the repo's active_robot.json pointer
    # (operator state).
    baked_cfg = tmp_path / "baked_tovez.json"
    baked_cfg.write_text(json.dumps({
        "schema_version": 2,
        "identity": {"robot_name": "tovez-baked", "uid": "tovez-baked"},
        "connection": {"device_announcement_name": "tovez"},
        "geometry": {"trackwidth": 128},
        "calibration": {"rotational_slip": 0.92},
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

    # Re-pace the sim tick-thread ~5x wall speed: same 20 ms physics step
    # (bit-identical dynamics), shorter sleep between steps.
    monkeypatch.setattr(transport_mod, "_SIM_TICK_SLEEP_S", 0.004)

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
    truth_mm: list[tuple[float, float, float]] = []  # (x_mm, y_mm, h_rad)

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
                truth_mm.append((x_cm * 10.0, y_cm * 10.0, h_rad))
            if gui_truth_cb is not None:
                gui_truth_cb(pose)

        transport.on_truth = _truth_spy

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

        n_truth_before = len(truth_mm)
        tour_btn.click()

        assert _spin_until(qapp, stop_btn.isEnabled, _TOUR_START_DEADLINE_S), (
            "tour never started (Stop Tour button did not enable)"
        )
        assert _spin_until(
            qapp, lambda: not stop_btn.isEnabled(), _TOUR_DEADLINE_S
        ), f"tour did not finish within {_TOUR_DEADLINE_S:.0f} s wall clock"
        # Let the final truth deliveries drain.
        _spin_events(qapp, 0.5)
    finally:
        disconnect_btn = window.findChild(QPushButton, "disconnect_btn")
        if disconnect_btn is not None and disconnect_btn.isEnabled():
            disconnect_btn.click()
            _spin_events(qapp, 0.3)
        window.hide()
        # Drop the pinned-config singleton so later tests re-resolve.
        rc_mod._reset_robot_config()

    return truth_mm[n_truth_before:]


def _assert_tour_geometry(trace, tour: list[str], waypoint_labels=None) -> None:
    """Assert the ground-truth trace hits the tour's ideal waypoints in order."""
    waypoints, final_heading_deg = _ideal_waypoints(tour)
    labels = waypoint_labels or [f"waypoint {i + 1}" for i in range(len(waypoints))]

    assert len(trace) > 50, (
        f"expected a dense ground-truth trace over the tour, got {len(trace)} "
        "samples — did the sim truth callback run?"
    )

    # Sanity: the robot must actually have toured, not idled at the origin.
    span_x = max(p[0] for p in trace) - min(p[0] for p in trace)
    span_y = max(p[1] for p in trace) - min(p[1] for p in trace)
    assert span_x > 300.0 and span_y > 200.0, (
        f"plant barely moved (span {span_x:.0f} x {span_y:.0f} mm) — the tour "
        "did not run"
    )

    misses: list[str] = []
    idx = 0
    for label, (cx, cy) in zip(labels, waypoints):
        best = None
        hit = None
        for i in range(idx, len(trace)):
            d = math.hypot(trace[i][0] - cx, trace[i][1] - cy)
            if best is None or d < best:
                best = d
            if d <= _WAYPOINT_TOL_MM:
                hit = i
                break
        if hit is None:
            misses.append(
                f"{label}: expected ({cx:+.0f}, {cy:+.0f}), closest approach "
                f"after previous waypoint was {best:.0f} mm (tol "
                f"{_WAYPOINT_TOL_MM:.0f} mm)"
            )
        else:
            idx = hit
    assert not misses, (
        "tour missed waypoint(s) (in visit order):\n  " + "\n  ".join(misses)
    )

    # Final pose: on the last waypoint, at the tour's final heading.
    fx, fy, fh = trace[-1]
    lx, ly = waypoints[-1]
    d_final = math.hypot(fx - lx, fy - ly)
    assert d_final <= _WAYPOINT_TOL_MM, (
        f"tour ended {d_final:.0f} mm from its final waypoint "
        f"({lx:+.0f}, {ly:+.0f}) — got ({fx:+.0f}, {fy:+.0f})"
    )
    dh = _wrap_deg(math.degrees(fh) - final_heading_deg)
    assert abs(dh) <= _FINAL_HEADING_TOL_DEG, (
        f"final heading {math.degrees(fh):.1f}° is {dh:+.1f}° off the "
        f"expected {_wrap_deg(final_heading_deg):.0f}° — turn legs are not "
        "turning the commanded angle"
    )


_TURN_BUG_REASON = (
    "Tours do not trace their shapes at zero sim error: RT turns are "
    "open-loop encoder-arc (Planner::beginRotation) with the arc target "
    "inflated by rotationalSlip=0.92 (baked into the sim firmware's "
    "DefaultConfig — GUI robot selection does not change it) in a no-scrub "
    "world, plus coast-anticipation/tick-quantization residuals; the RT "
    "legs accumulate heading error and late waypoints miss by >100 mm. "
    "Sprint 073 owns the fix; ticket 073-004 removes these markers."
)


@pytest.mark.xfail(strict=True, reason=_TURN_BUG_REASON)
def test_tour1_traces_the_tour_at_zero_error(qapp, monkeypatch, tmp_path):
    """Zero every Sim Errors knob via the GUI, click Tour 1, and assert the
    plant ground truth visits orange → purple → blue → green → orange."""
    from robot_radio.testgui.commands import TOUR_1

    trace = _run_tour_headless(qapp, monkeypatch, tmp_path, "tour_btn_tour_1")
    _assert_tour_geometry(
        trace,
        TOUR_1,
        waypoint_labels=[
            "orange (NE)",
            "purple (NW)",
            "blue (SW)",
            "green (SE)",
            "orange again (NE)",
        ],
    )


@pytest.mark.xfail(strict=True, reason=_TURN_BUG_REASON)
def test_tour2_traces_the_tour_at_zero_error(qapp, monkeypatch, tmp_path):
    """Zero every Sim Errors knob via the GUI, click Tour 2, and assert the
    closed loop returns to the origin at its dead-reckoned final heading.

    Measured on the 073-001 work tree (2026-07-03): six RT 9000s at ~+99.3°
    each → return-to-origin miss ~122 mm, heading +56° off."""
    from robot_radio.testgui.commands import TOUR_2

    trace = _run_tour_headless(qapp, monkeypatch, tmp_path, "tour_btn_tour_2")
    _assert_tour_geometry(trace, TOUR_2)
