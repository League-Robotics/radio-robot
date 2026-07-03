"""tests/testgui/test_tour1_geometry.py — Tour 1 must actually trace its tour.

End-to-end headless GUI test (QT_QPA_PLATFORM=offscreen, real ctypes firmware
sim).  This is the acceptance test for the Tour 1 button's *geometry*, not its
plumbing: with every Sim Errors knob at its zero-error value, clicking
**Tour 1** must drive the plant ground truth through the four field corners
and back — the "orange → purple → blue → green → orange" tour:

    Set Robot @ 0,0 (origin reset, heading 0 = +x east)
    RT 4500        turn 45° toward the NE corner        (orange)
    D 200 200 420  drive to the NE corner               (orange,  ~(297,  297))
    TURN 18000     turn to face west
    D 200 200 700  drive to the NW corner               (purple,  ~(-403, 297))
    RT 9000        turn to face south
    D 200 200 500  drive to the SW corner               (blue,    ~(-403,-203))
    RT 9000        turn to face east
    D 200 200 700  drive to the SE corner               (green,   ~(297, -203))
    RT 9000        turn to face north
    D 200 200 500  drive back to the NE corner          (orange)

Zero-error here means what the operator can reach through the Sim Errors
panel: every additive/noise knob 0.0, every multiplicative knob 1.0, and the
plant trackwidth equal to the firmware's configured ``trackwidthMm`` (128.0,
``DefaultConfig.cpp``) so plant geometry matches the firmware's kinematic
calibration.  Everything runs through the real GUI objects — the transport
combo + Connect button, the Sim Errors spinboxes + Apply, the Tour 1
QPushButton, ``_TourRunner`` on its QThread, and ``SimTransport``'s
tick-thread — exactly the path an operator exercises.

Currently XFAIL (strict): the tour does NOT trace the rectangle even at zero
error.  Root cause (measured 2026-07-02, tests/_infra sim, zero-error
profile):

  * ``RT`` is an open-loop encoder-arc turn (``Planner::beginRotation``,
    source/control/PlannerBegin.cpp).  Its per-wheel arc target is inflated
    by the firmware calibration constant ``rotationalSlip`` (default 0.92,
    ``arc = deg·(π/180)·(tw/2)/slip``) — compensation for real-world wheel
    scrub that the zero-error plant does not have.  Each RT 9000 physically
    turns +95.18°, not 90°.
  * With the slip term neutralized (``SET rotSlip=0``), RT *under*-rotates
    to 86.95° (operator-observed 87.56°): the fixed coast-anticipation
    ``kRtCoastArcMm = 8.0`` mm ("~7.3° SOFT-ramp coast at 100°/s",
    sim-tuned) is stale — ``yawRateMax=70`` now caps the spin rate at
    70°/s, where the actual SOFT-ramp coast is only ~4.5°.
  * ``TURN`` (closed-loop on the fused heading) lands within ~1° and is not
    the problem.  The three open-loop RT 9000 legs accumulate the heading
    error, so the last two corners miss by >100 mm and the trace skews —
    exactly the diagonal-cut quadrilateral seen on the canvas.

Delete the xfail marker when the firmware turn model is fixed —
strict=True makes an unexpected pass loud.

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

# Expected corner waypoints (mm, plant world frame, origin at tour start).
# 420/sqrt(2) = 296.98 — the RT 4500 + D 420 diagonal leg lands on the NE
# corner; each later leg is axis-aligned from there.
_D45 = 420.0 / math.sqrt(2.0)
_CORNERS: list[tuple[str, float, float]] = [
    ("orange (NE)", _D45, _D45),
    ("purple (NW)", _D45 - 700.0, _D45),
    ("blue (SW)", _D45 - 700.0, _D45 - 500.0),
    ("green (SE)", _D45, _D45 - 500.0),
    ("orange again (NE)", _D45, _D45),
]

#: How close (mm) the ground-truth trace must pass to each corner, in order.
#: Perfect execution dwells ON each corner (turn-in-place), and truth is
#: sampled every 200 sim-ms (<= 40 mm apart at 200 mm/s), so this is
#: generous; the current turn bug misses the last corners by >100 mm.
_CORNER_TOL_MM = 60.0
#: The tour's last leg drives due north; final heading must be ~90°.
_FINAL_HEADING_DEG = 90.0
_FINAL_HEADING_TOL_DEG = 5.0

#: Wall-clock ceilings (the sim is re-paced ~5x wall speed; a full tour is
#: ~25 sim-seconds of motion plus per-leg SNAP-poll overhead).
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
    """Process Qt events until ``predicate()`` is true or timeout.

    Returns True if the predicate fired within the deadline.
    """
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


@pytest.mark.xfail(
    strict=True,
    reason=(
        "Tour 1 does not trace its rectangle at zero sim error: RT turns are "
        "open-loop encoder-arc (Planner::beginRotation) with the arc target "
        "inflated by rotationalSlip=0.92 in a no-scrub world (+5.2° per 90°), "
        "and a stale kRtCoastArcMm=8mm coast-anticipation tuned for 100°/s "
        "while yawRateMax=70 caps the spin rate (-3° per turn when the slip "
        "term is neutral); the three RT 9000 legs accumulate heading error "
        "and the last corners miss by >100 mm. Fix the firmware turn model, "
        "then remove this marker (see module docstring)."
    ),
)
def test_tour1_traces_the_tour_at_zero_error(qapp, monkeypatch, tmp_path):
    """Zero every Sim Errors knob via the GUI, click Tour 1, and assert the
    plant ground truth visits orange → purple → blue → green → orange."""
    from PySide6.QtWidgets import (  # type: ignore[import-untyped]
        QComboBox,
        QDoubleSpinBox,
        QPushButton,
    )

    import robot_radio.testgui.__main__ as gui_main
    from robot_radio.testgui import sim_prefs
    from robot_radio.testgui import transport as transport_mod

    # Keep the operator's persisted error profile untouched: point sim_prefs
    # persistence at a temp file for the whole test (the panel's Apply saves
    # it, and SimTransport.connect() re-loads it).
    monkeypatch.setattr(sim_prefs, "_PREFS_DIR", tmp_path)
    monkeypatch.setattr(
        sim_prefs, "_PREFS_PATH", tmp_path / "sim_error_profile.json"
    )

    # Re-pace the sim tick-thread ~5x wall speed: same 20 ms physics step
    # (bit-identical dynamics), shorter sleep between steps.  Full wall-clock
    # pacing would make this a multi-minute test; 5x keeps the Qt event queue
    # comfortable and the whole tour under ~1 minute.
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

        tour_btn = window.findChild(QPushButton, "tour_btn_tour_1")
        stop_btn = window.findChild(QPushButton, "stop_tour_btn")
        assert tour_btn is not None and stop_btn is not None
        assert tour_btn.isEnabled(), "Tour 1 button not enabled after connect"

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

    trace = truth_mm[n_truth_before:]
    assert len(trace) > 50, (
        f"expected a dense ground-truth trace over the tour, got {len(trace)} "
        "samples — did the sim truth callback run?"
    )

    # Sanity: the robot must actually have toured, not idled at the origin.
    span_x = max(p[0] for p in trace) - min(p[0] for p in trace)
    span_y = max(p[1] for p in trace) - min(p[1] for p in trace)
    assert span_x > 400.0 and span_y > 300.0, (
        f"plant barely moved (span {span_x:.0f} x {span_y:.0f} mm) — the tour "
        "did not run"
    )

    # Walk the trace: it must pass within tolerance of each corner IN ORDER.
    misses: list[str] = []
    idx = 0
    for label, cx, cy in _CORNERS:
        best = None
        hit = None
        for i in range(idx, len(trace)):
            d = math.hypot(trace[i][0] - cx, trace[i][1] - cy)
            if best is None or d < best:
                best = d
            if d <= _CORNER_TOL_MM:
                hit = i
                break
        if hit is None:
            misses.append(
                f"{label}: expected ({cx:+.0f}, {cy:+.0f}), closest approach "
                f"after previous corner was {best:.0f} mm (tol "
                f"{_CORNER_TOL_MM:.0f} mm)"
            )
        else:
            idx = hit
    assert not misses, (
        "Tour 1 missed corner(s) (in visit order):\n  " + "\n  ".join(misses)
    )

    # The tour's final leg drives due north — heading must end ~90°.
    final_h_deg = math.degrees(trace[-1][2])
    dh = _wrap_deg(final_h_deg - _FINAL_HEADING_DEG)
    assert abs(dh) <= _FINAL_HEADING_TOL_DEG, (
        f"final heading {final_h_deg:.1f}° is {dh:+.1f}° off the expected "
        f"{_FINAL_HEADING_DEG:.0f}° (north) — turn legs are not turning the "
        "commanded angle"
    )

    # And it must end back at orange.
    fx, fy = trace[-1][0], trace[-1][1]
    d_final = math.hypot(fx - _D45, fy - _D45)
    assert d_final <= _CORNER_TOL_MM, (
        f"tour ended {d_final:.0f} mm from the orange corner "
        f"({_D45:.0f}, {_D45:.0f}) — got ({fx:+.0f}, {fy:+.0f})"
    )
