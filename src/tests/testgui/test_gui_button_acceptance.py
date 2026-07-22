"""src/tests/testgui/test_gui_button_acceptance.py -- the standing headless
TestGUI button-acceptance suite (stakeholder directive, 2026-07-22):

    "You should be able to run all the buttons that I'm gonna press and get
    a trace of those things ... get the positions of the robot ... without
    me clicking. It has to be part of the acceptance test. Do not say it's
    done until you click on those buttons and verify the simulated robot
    goes where it's supposed to go and it goes there in a constructive
    fashion."

This module builds the REAL ``robot_radio.testgui.__main__`` window headless
(``QT_QPA_PLATFORM=offscreen``, see ``conftest.py``), connects it against the
REAL Sim stack (``SimTransport`` -> ``robot_radio.io.sim_loop.SimLoop`` ->
the compiled ``src/sim/build/libfirmware_host`` firmware simulator) via the
ACTUAL "Sim" transport-combo + Connect-button click -- so the connect-time
calibration/config push (``__main__.py``'s ``_on_connect()``:
``transport.connect()``'s own Tier 1 + Tier 2 push, THEN
``_push_robot_calibration()``'s OI/OL/OA + GET-echo-cache push) runs exactly
as it does for an operator, not a hand-rolled substitute. Then it clicks (or,
where no widget exists -- see "SEG 0" below -- calls the exact same
entry-point method the button's own handler calls) every motion-relevant
button on the button surface described in the sprint brief, and asserts:

  (a) motion actually occurred (an encoder excursion beyond the plant's own
      rest-dither noise floor -- see ``_button_acceptance_support.
      encoder_span()``),
  (b) direction and magnitude land within a stated tolerance against
      ground-truth pose (``SimLoop.get_true_pose()`` -- bypasses every
      sensor drift/noise fault knob, so this is "where the robot actually
      is", not a noisy report of it),
  (c) the move completes (ground truth goes quiet, or -- for tours -- the
      Stop Tour button re-disables) within a bounded timeout, never hangs,
  (d) end pose is recorded.

A per-button trace table (button, path, commanded, measured, elapsed,
tolerance, encoder-advanced, verdict) is printed as each row is recorded and
written incrementally to a CSV under pytest's own ``tmp_path_factory`` dir
(printed at teardown) -- see ``_button_acceptance_support.TraceRecorder``.

Button surface covered (sprint brief's own enumeration)
----------------------------------------------------------
1. Unmanaged column: distance +-100/+-500/+-700 (``run_unmanaged()`` via the
   ``unmanaged_dist_btn_*`` preset buttons), angles +-90/+-180/+-270/+-360
   (``unmanaged_ang_btn_*``), plus the Unmanaged Test buttons ("S -- drive
   700mm" = ``test_us_btn``, "T -- turn 360deg" = ``test_ut_btn`` -- these
   REBUILD the sim lib and hot-reload before driving; see
   ``TestButtonRebuildAndDrive`` below).
2. Managed column: the SAME preset set via the managed dispatch
   (``managed_dist_btn_*`` -> ``command("D 150 150 <mm>")``,
   ``managed_ang_btn_*`` -> ``command("RT <cdeg>")``), PLUS the ``SEG 0
   <cdeg>`` wire form directly (``transport.command("SEG 0 <cdeg>")``) --
   the same primitive ``turn_control.py``'s external TCP joystick socket
   sends (``__main__.py``'s ``_send_seg_turn()``); there is no QPushButton
   for this one (see that section's own docstring for why), so it is
   exercised at the entry-point level, matching the ticket's own framing
   ("exercises ... the SAME entry points every motion button calls").
   Also the Managed Test buttons (``test_s_btn``/``test_t_btn``).
3. Tour 1 and Tour 2, clicked end to end (``tour_btn_tour_1``/
   ``tour_btn_tour_2``).
4. STOP mid-motion (``ops_btn_stop``, clicked during a running Tour 1):
   motion ceases and the tour queue is flushed (no further leg-completion
   log lines after the stop).
5. GOTO: checked and confirmed DORMANT -- ``goto_btn`` is permanently
   disabled (no wire arm for the camera pursuit loop's SI/G reset -- see
   ``__main__.py``'s own tooltip on that button). A regression guard
   asserts it STAYS disabled (so a future accidental enable is caught); the
   "drive to a point" behavior test is explicitly ``skip``-marked with the
   reason, per the ticket's own instruction not to silently omit it.

Run with::

    QT_QPA_PLATFORM=offscreen uv run python -m pytest src/tests/testgui/test_gui_button_acceptance.py -v

Slow (~1-2 minutes: two full tour runs plus four sim-lib rebuild/reload
cycles) -- marked ``slow`` so ``pytest -m "not slow"`` can deselect it for a
fast local loop, but it is NOT excluded from the default suite (matches
``test_tour1_geometry.py``'s own precedent and ``pyproject.toml``'s own
``markers`` comment: "stays in the default run ... never excluded by
default"). Runtime is kept sane primarily via the Sim speed-up factor
(``SPEED_FACTOR`` below, applied through the GUI's own ``sim_speed_combo`` --
one of the five offered multiples, already regression-tested by
``test_sim_speed_factor.py``; that combo's own tooltip: "Physics integration
step is unchanged -- trajectories are identical at every speed", so this
does not affect any tolerance below).
"""
from __future__ import annotations

import math
import time

import pytest

from robot_radio.testgui.transport import (
    _sim_lib_path,
    _UNMANAGED_SPEED,
    _UNMANAGED_YAW_RATE,
)

from ._button_acceptance_support import (
    Row,
    TraceRecorder,
    allowed_error,
    encoder_span,
    settle_pose,
    signed_distance,
    signed_heading_delta,
)

pytestmark = [
    pytest.mark.skipif(
        not _sim_lib_path().exists(),
        reason="sim lib not built -- cmake --build src/sim/build (or `python build.py`)",
    ),
    pytest.mark.slow,
]

# ---------------------------------------------------------------------------
# Preset battery -- the sprint brief's own literal list.
# ---------------------------------------------------------------------------

_DIST_PRESETS_MM = (100, -100, 500, -500, 700, -700)
_ANGLE_PRESETS_DEG = (90, -90, 180, -180, 270, -270, 360, -360)

# ---------------------------------------------------------------------------
# Tolerance model -- see _button_acceptance_support's own module docstring
# for the full rationale (allowed_error = abs_margin + rel_tol * |commanded|).
# Both constants are stated per path, each with the empirical numbers that
# justify it (measured 2026-07-22 against data/robots/tovez_nocal.json,
# DEFAULT_PROFILE sim errors -- i.e. no injected fault beyond the sensor-only
# otos_linear_noise, which does not perturb ground truth).
# ---------------------------------------------------------------------------

# Unmanaged (run_unmanaged() -- one deadman-timed twist, no planner):
# observed error 100mm->+6.0%, 500mm->+0.6%, 700mm->+0.8%;
# 90deg->+3.4%, 180deg->+2.9%, 270deg->+2.7%, 360deg->+0.9%.
UNMANAGED_DIST_ABS_MARGIN_MM = 15.0    # [mm]
UNMANAGED_DIST_REL_TOL = 0.10          # fraction of |commanded|
UNMANAGED_ANGLE_ABS_MARGIN_DEG = 10.0  # [deg]
UNMANAGED_ANGLE_REL_TOL = 0.10

# Managed (D/RT/SEG via the Move-queue dispatch, command()): observed a
# near-CONSTANT absolute overshoot regardless of magnitude -- 100mm->+28.2%
# (+28.2mm), 500mm->+5.2% (+26.1mm), 700mm->+4.1% (+28.5mm); 90deg->+22.8%
# (+20.5deg), 180deg->+12.6% (+22.6deg), 270deg->+7.0% (+18.8deg),
# 360deg->+5.8% (+20.9deg) -- consistent with a fixed ~150-180ms
# stop-detection/actuation lag (150mm/s * 0.18s ~= 27mm; 2rad/s * 0.18s ~=
# 20.6deg), not a percentage error -- see this module's own docstring and
# _button_acceptance_support's for the full "known stop-tail" derivation.
MANAGED_DIST_ABS_MARGIN_MM = 45.0
MANAGED_DIST_REL_TOL = 0.10
MANAGED_ANGLE_ABS_MARGIN_DEG = 30.0
MANAGED_ANGLE_REL_TOL = 0.10

#: Per-wheel encoder excursion (mm) beyond which motion is considered to
#: have "actually occurred" -- see encoder_span()'s own docstring for why
#: this must be well above the plant's +/-1 LSB rest dither.
MIN_ENCODER_SPAN_MM = 5.0

#: A completed tour's start-vs-end ground-truth position delta must stay
#: below this -- a report-oriented sanity bound (catches a genuinely broken
#: seam: integration blowing up / stuck at the origin / NaN), NOT a tight
#: closure claim. Matches test_sim_transport_tour1.py's own precedent
#: (_MAX_CLOSURE_POSITION_MM, real bench TOUR_1 closures ranged up to
#: ~500mm even when cleanly COMPLETED).
MAX_TOUR_CLOSURE_POSITION_MM = 700.0

# ---------------------------------------------------------------------------
# Sim speed-up + settle-poll bounds.
# ---------------------------------------------------------------------------

#: Sim fast-forward multiple applied via the GUI's own sim_speed_combo --
#: one of the five offered (1/2/5/10/20), already regression-tested by
#: test_sim_speed_factor.py. Physics integration is unchanged at every
#: factor (only wall-clock pacing compresses), so this has no effect on any
#: tolerance above -- it exists purely to keep this suite's wall-clock
#: runtime sane.
SPEED_FACTOR = 10

SETTLE_QUIET_S = 0.25   # [s] wall-clock quiet window that declares "settled"
SETTLE_POLL_S = 0.03    # [s] poll interval
#: wall_timeout = max(FLOOR, (expected_sim_duration_s / SPEED_FACTOR) * SAFETY)
SETTLE_TIMEOUT_SAFETY = 6.0
SETTLE_TIMEOUT_FLOOR_S = 2.5  # [s]

#: Tour completion bound -- generous: a full Tour 1 (13 legs) measured
#: ~43s wall-clock at 1x; bounded here well past what SPEED_FACTOR should
#: bring it down to, so a real hang/regression still fails instead of
#: silently extending.
TOUR_TIMEOUT_S = 60.0

#: Rebuild+reload bound for the Test buttons (cmake incremental build +
#: gen_version.py/gen_messages.py) -- an unchanged-source rebuild measured
#: ~2.5s; bounded generously past that for a machine under load or a cold
#: build cache.
TEST_BUTTON_REBUILD_TIMEOUT_S = 90.0


def _expected_duration_s(*, distance_mm: "float | None" = None,
                          angle_deg: "float | None" = None) -> float:
    """Expected SIM-time duration (seconds) of an unmanaged/managed
    distance-or-angle preset -- both paths cruise at the SAME
    ``_UNMANAGED_SPEED``/``_UNMANAGED_YAW_RATE`` (see
    ``transport.py``'s own module comment: "Matched so a straight/turn's
    cruise speed is identical across BOTH GUI columns"), so one formula
    covers unmanaged and managed alike."""
    if distance_mm is not None:
        return abs(distance_mm) / _UNMANAGED_SPEED
    assert angle_deg is not None
    return math.radians(abs(angle_deg)) / _UNMANAGED_YAW_RATE


def _wall_timeout_for(expected_duration_s: float) -> float:
    return max(
        SETTLE_TIMEOUT_FLOOR_S,
        (expected_duration_s / SPEED_FACTOR) * SETTLE_TIMEOUT_SAFETY,
    )


def _tol_str(abs_margin: float, rel_tol: float, commanded: float, unit: str) -> str:
    allowed = allowed_error(commanded, abs_margin=abs_margin, rel_tol=rel_tol)
    return f"+/-({abs_margin:g}{unit}+{rel_tol * 100:g}%|cmd|)={allowed:.1f}{unit}"


# ---------------------------------------------------------------------------
# qapp / gui fixtures
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def qapp():
    pytest.importorskip("PySide6")
    import sys

    from PySide6.QtWidgets import QApplication  # type: ignore[import-untyped]

    app = QApplication.instance() or QApplication(sys.argv)
    yield app


class _GuiCtx:
    """Everything a test needs: the real window/app, the live (possibly
    reconnected -- see ``current`` below) ``SimTransport``, and small
    click/pump/settle helpers bound to this session.

    ``created`` accumulates EVERY ``SimTransport`` instance constructed
    during this session -- the Test buttons (S/T/US/UT) disconnect and
    reconnect with a FRESH instance each click (``__main__.py``'s
    ``_finish_test()``), so ``current`` always resolves to the latest one
    rather than a stale cached reference.
    """

    def __init__(self, window, app, created: list, frames: list) -> None:
        self.window = window
        self.app = app
        self.created = created
        self.frames = frames

    @property
    def current(self):
        assert self.created, "no SimTransport has been constructed yet"
        return self.created[-1]

    def pump(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.app.processEvents()
            time.sleep(0.005)

    def find(self, widget_cls, name: str):
        widget = self.window.findChild(widget_cls, name)
        assert widget is not None, f"{widget_cls.__name__} {name!r} not found in the GUI"
        return widget

    def click(self, name: str, *, must_be_enabled: bool = True):
        from PySide6.QtWidgets import QPushButton  # type: ignore[import-untyped]

        btn = self.find(QPushButton, name)
        if must_be_enabled:
            assert btn.isEnabled(), f"button {name!r} is disabled -- cannot click"
        btn.click()
        self.pump(0.05)
        return btn

    def reset_pose(self) -> None:
        self.current.protocol.set_true_pose(0.0, 0.0, 0.0)
        self.pump(0.1)

    def settle(self, timeout_s: float):
        return settle_pose(
            lambda: self.current.protocol.get_true_pose(),
            timeout_s=timeout_s, quiet_s=SETTLE_QUIET_S, poll_s=SETTLE_POLL_S,
            pump=lambda: self.app.processEvents(),
        )

    def log_text(self) -> str:
        from PySide6.QtWidgets import QPlainTextEdit  # type: ignore[import-untyped]

        return self.find(QPlainTextEdit, "log_pane").toPlainText()


@pytest.fixture(scope="module")
def gui(qapp, tmp_path_factory, request):
    """Build the REAL GUI window, connect via Sim with a pinned, known
    robot config (``data/robots/tovez_nocal.json`` -- geometry-pure, no
    calibration to fight when checking tolerances), and set the sim
    speed-up factor via the real ``sim_speed_combo`` widget -- exactly the
    operator flow, module-scoped so the whole button battery below shares
    ONE connected session (each Test-button click still reconnects its own
    fresh ``SimTransport`` internally -- see ``_GuiCtx.current``)."""
    import json
    import pathlib

    import robot_radio.testgui.__main__ as gui_main
    from PySide6.QtWidgets import QComboBox, QPushButton  # type: ignore[import-untyped]
    from robot_radio.config import robot_config as rc_mod
    from robot_radio.testgui import sim_prefs
    from robot_radio.testgui import transport as transport_mod

    repo_root = pathlib.Path(__file__).resolve().parents[3]
    cfg_path = repo_root / "data" / "robots" / "tovez_nocal.json"
    assert cfg_path.exists(), f"missing {cfg_path}"

    tmp_path = tmp_path_factory.mktemp("gui_button_acceptance_session")

    mp = pytest.MonkeyPatch()
    mp.setenv("ROBOT_CONFIG", str(cfg_path))
    rc_mod._reset_robot_config()
    # Isolate the persisted sim-error-profile file so this run never reads
    # (or writes) a developer's real data/testgui/sim_error_profile.json --
    # mirrors test_calibration_push_on_connect.py's own established pattern.
    mp.setattr(sim_prefs, "_PREFS_DIR", tmp_path)
    mp.setattr(sim_prefs, "_PREFS_PATH", tmp_path / "sim_error_profile.json")

    created: list = []
    frames: list = []

    _RealSimTransport = transport_mod.SimTransport

    class _RecordingSimTransport(_RealSimTransport):  # noqa: N801 -- name-checked by is_sim_transport
        """Subclass that (a) records every constructed instance into
        ``created`` (mirrors test_calibration_push_on_connect.py's own
        ``_connect_gui_with_config`` pattern) and (b) wraps EVERY
        ``on_telemetry`` assignment so telemetry frames are captured from
        the very first frame after each (re)connect -- including the Test
        buttons' disconnect/reconnect cycle, where the GUI reassigns
        ``on_telemetry`` fresh on each new instance. A plain
        ``transport.on_telemetry = frames.append`` set up AFTER a Test
        button's rebuild-and-reconnect would race: by the time the test
        detects the new instance and re-wires it, the button's own fixed
        motion may already be finished (the whole rebuild+reconnect+drive
        sequence runs inside ONE queued-signal handler,
        ``__main__.py``'s ``_finish_test()``). Wrapping the property instead
        of the callback means whichever callback ``_on_connect()`` assigns
        (production's own ``_on_telemetry_thread_v2``) is captured AND
        chained through unchanged, from the very first assignment -- i.e.
        from inside ``Transport.__init__()`` itself.
        """

        def __init__(self) -> None:
            super().__init__()
            created.append(self)

        @property
        def on_telemetry(self):
            return self.__dict__.get("_on_telemetry_recording_cb")

        @on_telemetry.setter
        def on_telemetry(self, cb):
            def _wrapped(frame, _cb=cb):
                frames.append(frame)
                if _cb is not None:
                    _cb(frame)
            self.__dict__["_on_telemetry_recording_cb"] = _wrapped

    mp.setattr(transport_mod, "SimTransport", _RecordingSimTransport)

    window, app = gui_main._build_main_window()

    combo = window.findChild(QComboBox, "transport_combo")
    combo.setCurrentText("Sim")

    speed_combo = window.findChild(QComboBox, "sim_speed_combo")
    idx = speed_combo.findData(SPEED_FACTOR)
    assert idx >= 0, f"sim_speed_combo has no {SPEED_FACTOR}x entry"
    speed_combo.setCurrentIndex(idx)

    connect_btn = window.findChild(QPushButton, "connect_btn")
    connect_btn.click()

    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        app.processEvents()
        time.sleep(0.01)
        if created and created[-1]._connected:
            break
    assert created and created[-1]._connected, "SimTransport failed to connect"

    ctx = _GuiCtx(window, app, created, frames)
    yield ctx

    disconnect_btn = window.findChild(QPushButton, "disconnect_btn")
    if disconnect_btn is not None and disconnect_btn.isEnabled():
        disconnect_btn.click()
        ctx.pump(1.0)
    window.hide()
    rc_mod._reset_robot_config()
    mp.undo()


@pytest.fixture(scope="module")
def recorder(tmp_path_factory):
    csv_path = tmp_path_factory.mktemp("gui_button_acceptance_trace") / "button_trace.csv"
    rec = TraceRecorder(csv_path)
    yield rec
    rec.close()
    # Always discoverable even if a human doesn't scroll up through pytest
    # output -- print the path one more time, unambiguously, at the very
    # end of this module's teardown.
    print(f"\n[BUTTON-ACCEPTANCE] full trace CSV: {rec.csv_path}")


# ---------------------------------------------------------------------------
# 0. Connect-time calibration/config push sanity check.
# ---------------------------------------------------------------------------


def test_connect_pushed_calibration_from_active_robot_config(gui, recorder):
    """Confirms the fixture's ``Connect`` click actually ran the real
    connect-time calibration push (``__main__.py``'s
    ``_push_robot_calibration()``) -- not just that a bare ``SimLoop``
    connected. Mirrors ``test_calibration_push_on_connect.py``'s own
    ``ml``-based assertion (``rotSlip``/``tw`` have no live firmware
    consumer this sprint -- see that file's module docstring; ``ml``
    (``MotorConfigPatch.travel_calib``) does)."""
    from robot_radio.robot import protocol as protocol_mod

    wheel_diameter_mm = 80.77  # data/robots/tovez_nocal.json wheels.wheel_diameter_mm
    expected_ml = protocol_mod._format_config_value(math.pi * wheel_diameter_mm / 360.0)

    reply = gui.current.command("GET ml", read_timeout=500)
    verdict = "PASS" if f"ml={expected_ml}" in reply else "FAIL"
    recorder.record(Row(
        button="connect_btn", path="calibration", commanded=f"ml={expected_ml}",
        measured=reply.strip(), elapsed_s=0.0, tolerance="exact match",
        encoder_advanced=None, verdict=verdict,
    ))
    assert f"ml={expected_ml}" in reply, (
        f"connect-time calibration push did not land ml -- firmware reports {reply!r}"
    )


# ---------------------------------------------------------------------------
# 1/2. Unmanaged + Managed distance/angle preset battery.
# ---------------------------------------------------------------------------


def _run_dist_preset(gui, recorder, *, button: str, mm: int, path: str,
                     abs_margin: float, rel_tol: float, dispatch) -> None:
    gui.reset_pose()
    gui.frames.clear()
    start = gui.current.protocol.get_true_pose()

    t0 = time.monotonic()
    dispatch()
    wall_timeout = _wall_timeout_for(_expected_duration_s(distance_mm=mm))
    end, elapsed, timed_out = gui.settle(wall_timeout)

    measured = signed_distance(start, end, forward_sign=float(mm))
    allowed = allowed_error(mm, abs_margin=abs_margin, rel_tol=rel_tol)
    span = encoder_span(gui.frames)
    moved = span is not None and max(span) >= MIN_ENCODER_SPAN_MM
    ok = (not timed_out) and moved and abs(measured - mm) <= allowed

    recorder.record(Row(
        button=button, path=path, commanded=f"{mm:+d}mm", measured=f"{measured:+.1f}mm",
        elapsed_s=elapsed, tolerance=_tol_str(abs_margin, rel_tol, mm, "mm"),
        encoder_advanced=moved, verdict="PASS" if ok else "FAIL",
    ))

    assert not timed_out, f"{button}: motion never settled within {wall_timeout:.1f}s (hang?)"
    assert moved, f"{button}: no encoder excursion beyond {MIN_ENCODER_SPAN_MM}mm -- motion did not occur (span={span})"
    assert abs(measured - mm) <= allowed, (
        f"{button}: commanded {mm:+d}mm, measured {measured:+.1f}mm, "
        f"exceeds tolerance +/-{allowed:.1f}mm"
    )


def _run_angle_preset(gui, recorder, *, button: str, deg: int, path: str,
                      abs_margin: float, rel_tol: float, dispatch) -> None:
    gui.reset_pose()
    gui.frames.clear()
    start = gui.current.protocol.get_true_pose()

    dispatch()
    wall_timeout = _wall_timeout_for(_expected_duration_s(angle_deg=deg))
    end, elapsed, timed_out = gui.settle(wall_timeout)

    measured = signed_heading_delta(start, end)
    allowed = allowed_error(deg, abs_margin=abs_margin, rel_tol=rel_tol)
    span = encoder_span(gui.frames)
    moved = span is not None and max(span) >= MIN_ENCODER_SPAN_MM
    ok = (not timed_out) and moved and abs(measured - deg) <= allowed

    recorder.record(Row(
        button=button, path=path, commanded=f"{deg:+d}deg", measured=f"{measured:+.1f}deg",
        elapsed_s=elapsed, tolerance=_tol_str(abs_margin, rel_tol, deg, "deg"),
        encoder_advanced=moved, verdict="PASS" if ok else "FAIL",
    ))

    assert not timed_out, f"{button}: motion never settled within {wall_timeout:.1f}s (hang?)"
    assert moved, f"{button}: no encoder excursion beyond {MIN_ENCODER_SPAN_MM}mm -- motion did not occur (span={span})"
    assert abs(measured - deg) <= allowed, (
        f"{button}: commanded {deg:+d}deg, measured {measured:+.1f}deg, "
        f"exceeds tolerance +/-{allowed:.1f}deg"
    )


@pytest.mark.parametrize("mm", _DIST_PRESETS_MM)
def test_unmanaged_distance_preset(gui, recorder, mm):
    button = f"unmanaged_dist_btn_{mm:+d}"
    _run_dist_preset(
        gui, recorder, button=button, mm=mm, path="unmanaged",
        abs_margin=UNMANAGED_DIST_ABS_MARGIN_MM, rel_tol=UNMANAGED_DIST_REL_TOL,
        dispatch=lambda: gui.click(button),
    )


@pytest.mark.parametrize("deg", _ANGLE_PRESETS_DEG)
def test_unmanaged_angle_preset(gui, recorder, deg):
    button = f"unmanaged_ang_btn_{deg:+d}"
    _run_angle_preset(
        gui, recorder, button=button, deg=deg, path="unmanaged",
        abs_margin=UNMANAGED_ANGLE_ABS_MARGIN_DEG, rel_tol=UNMANAGED_ANGLE_REL_TOL,
        dispatch=lambda: gui.click(button),
    )


@pytest.mark.parametrize("mm", _DIST_PRESETS_MM)
def test_managed_distance_preset(gui, recorder, mm):
    button = f"managed_dist_btn_{mm:+d}"
    _run_dist_preset(
        gui, recorder, button=button, mm=mm, path="managed",
        abs_margin=MANAGED_DIST_ABS_MARGIN_MM, rel_tol=MANAGED_DIST_REL_TOL,
        dispatch=lambda: gui.click(button),
    )


@pytest.mark.parametrize("deg", _ANGLE_PRESETS_DEG)
def test_managed_angle_preset(gui, recorder, deg):
    button = f"managed_ang_btn_{deg:+d}"
    _run_angle_preset(
        gui, recorder, button=button, deg=deg, path="managed",
        abs_margin=MANAGED_ANGLE_ABS_MARGIN_DEG, rel_tol=MANAGED_ANGLE_REL_TOL,
        dispatch=lambda: gui.click(button),
    )


@pytest.mark.parametrize("deg", _ANGLE_PRESETS_DEG)
def test_managed_seg_0_cdeg_turn(gui, recorder, deg):
    """``SEG 0 <cdeg>`` -- the in-place-pivot primitive
    ``turn_control.py``'s external TCP joystick socket sends
    (``__main__.py``'s ``_send_seg_turn()``/``_make_turn_handler()``; no
    QPushButton is wired to it in the current build -- see this module's
    own docstring), exercised directly at the entry-point level:
    ``transport.command("SEG 0 <cdeg>")``, the exact call
    ``_send_seg_turn()`` itself makes. SimTransport translates this
    internally onto the identical ``RT <cdeg>`` dispatch (see
    ``transport.py``'s ``SimTransport._dispatch()``), so it shares the
    managed angle tolerance."""
    cdeg = int(round(deg * 100))
    button = f"SEG 0 {cdeg}"
    _run_angle_preset(
        gui, recorder, button=button, deg=deg, path="seg",
        abs_margin=MANAGED_ANGLE_ABS_MARGIN_DEG, rel_tol=MANAGED_ANGLE_REL_TOL,
        dispatch=lambda: gui.current.command(f"SEG 0 {cdeg}", read_timeout=500),
    )


# ---------------------------------------------------------------------------
# Test buttons (S/T/US/UT) -- rebuild + hot-reload + reset + fixed drive.
# ---------------------------------------------------------------------------


def _run_test_button(gui, recorder, *, button: str, path: str, kind: str,
                     mm: "float | None" = None, deg: "float | None" = None,
                     abs_margin: float, rel_tol: float) -> None:
    """Clicks a Test button (``test_s_btn``/``test_t_btn``/``test_us_btn``/
    ``test_ut_btn``) and waits for the FULL rebuild -> hot-reload ->
    reconnect -> reset-to-origin -> fixed-motion sequence
    (``__main__.py``'s ``_run_sim_test()``/``_finish_test()``) to complete.

    Detection is via ground-truth settle, not "reconnect observed", because
    the whole sequence (including dispatching the fixed motion) runs
    synchronously inside ONE queued-signal handler once the background
    rebuild thread finishes -- by the time this function's poll loop even
    notices a new ``SimTransport`` exists, the motion may already be
    mid-flight or finished. ``settle()`` handles that correctly either way
    (see its own docstring): it reports the CURRENT settled pose regardless
    of when polling started.
    """
    before_n = len(gui.created)
    t0 = time.monotonic()
    gui.click(button)

    deadline = t0 + TEST_BUTTON_REBUILD_TIMEOUT_S
    while time.monotonic() < deadline:
        gui.app.processEvents()
        time.sleep(0.02)
        if len(gui.created) > before_n and gui.created[-1]._connected:
            break
    assert len(gui.created) > before_n and gui.created[-1]._connected, (
        f"{button}: rebuild+reconnect never completed within "
        f"{TEST_BUTTON_REBUILD_TIMEOUT_S:.0f}s"
    )
    rebuild_elapsed = time.monotonic() - t0

    gui.frames.clear()
    commanded_value = mm if mm is not None else deg
    expected_dur = _expected_duration_s(distance_mm=mm, angle_deg=deg)
    wall_timeout = _wall_timeout_for(expected_dur) + rebuild_elapsed
    # start pose: _finish_test() calls _set_origin() (teleport to 0,0,0)
    # synchronously before dispatching the fixed motion, so (0,0,0) is the
    # correct commanded-from pose regardless of exactly when we observe it.
    start = {"x": 0.0, "y": 0.0, "h": 0.0}
    end, settle_elapsed, timed_out = gui.settle(wall_timeout)
    elapsed = rebuild_elapsed + settle_elapsed

    if mm is not None:
        measured = signed_distance(start, end, forward_sign=float(mm))
        commanded_str, measured_str, unit = f"{mm:+.0f}mm", f"{measured:+.1f}mm", "mm"
    else:
        measured = signed_heading_delta(start, end)
        commanded_str, measured_str, unit = f"{deg:+.0f}deg", f"{measured:+.1f}deg", "deg"

    allowed = allowed_error(commanded_value, abs_margin=abs_margin, rel_tol=rel_tol)
    span = encoder_span(gui.frames)
    moved = span is not None and max(span) >= MIN_ENCODER_SPAN_MM
    ok = (not timed_out) and abs(measured - commanded_value) <= allowed

    recorder.record(Row(
        button=button, path=path, commanded=commanded_str, measured=measured_str,
        elapsed_s=elapsed, tolerance=_tol_str(abs_margin, rel_tol, commanded_value, unit),
        encoder_advanced=moved, verdict="PASS" if ok else "FAIL",
    ))
    assert not timed_out, f"{button}: rebuild/drive never settled within {wall_timeout:.1f}s (hang?)"
    assert abs(measured - commanded_value) <= allowed, (
        f"{button}: commanded {commanded_str}, measured {measured_str}, "
        f"exceeds tolerance +/-{allowed:.1f}{unit}"
    )


def test_test_button_unmanaged_drive_700mm(gui, recorder):
    _run_test_button(
        gui, recorder, button="test_us_btn", path="test-unmanaged", kind="US", mm=700.0,
        abs_margin=UNMANAGED_DIST_ABS_MARGIN_MM, rel_tol=UNMANAGED_DIST_REL_TOL,
    )


def test_test_button_unmanaged_turn_360deg(gui, recorder):
    _run_test_button(
        gui, recorder, button="test_ut_btn", path="test-unmanaged", kind="UT", deg=360.0,
        abs_margin=UNMANAGED_ANGLE_ABS_MARGIN_DEG, rel_tol=UNMANAGED_ANGLE_REL_TOL,
    )


def test_test_button_managed_drive_700mm(gui, recorder):
    _run_test_button(
        gui, recorder, button="test_s_btn", path="test-managed", kind="S", mm=700.0,
        abs_margin=MANAGED_DIST_ABS_MARGIN_MM, rel_tol=MANAGED_DIST_REL_TOL,
    )


def test_test_button_managed_turn_360deg(gui, recorder):
    _run_test_button(
        gui, recorder, button="test_t_btn", path="test-managed", kind="T", deg=360.0,
        abs_margin=MANAGED_ANGLE_ABS_MARGIN_DEG, rel_tol=MANAGED_ANGLE_REL_TOL,
    )


# ---------------------------------------------------------------------------
# 3. Tour 1 / Tour 2 end to end.
# ---------------------------------------------------------------------------


def _run_tour(gui, recorder, *, button: str, tour_name: str, n_legs: int) -> None:
    stop_tour_btn = gui.find(__import__("PySide6.QtWidgets", fromlist=["QPushButton"]).QPushButton,
                              "stop_tour_btn")
    gui.frames.clear()
    t0 = time.monotonic()
    gui.click(button)
    # _on_tour_clicked() calls _set_origin() synchronously before spawning
    # the worker thread, so by the time click() returns the plant has
    # already been teleported to (0,0,0) -- the correct "start" pose.
    start = gui.current.protocol.get_true_pose()

    deadline = t0 + TOUR_TIMEOUT_S
    while time.monotonic() < deadline:
        gui.app.processEvents()
        time.sleep(0.02)
        if not stop_tour_btn.isEnabled():
            break
    elapsed = time.monotonic() - t0
    timed_out = stop_tour_btn.isEnabled()
    end = gui.current.protocol.get_true_pose()

    log_text = gui.log_text()
    completed = f"{tour_name} complete" in log_text
    stopped_early = f"{tour_name} stopped at leg" in log_text
    last_leg_line = f"{tour_name} leg {n_legs}/{n_legs}:"
    all_legs_ran = last_leg_line in log_text

    dx = end["x"] - start["x"]
    dy = end["y"] - start["y"]
    closure_mm = math.hypot(dx, dy)
    closure_deg = signed_heading_delta(start, end)

    span = encoder_span(gui.frames)
    moved = span is not None and max(span) >= MIN_ENCODER_SPAN_MM
    ok = (not timed_out) and completed and not stopped_early and all_legs_ran and (
        closure_mm < MAX_TOUR_CLOSURE_POSITION_MM) and moved

    recorder.record(Row(
        button=button, path="tour", commanded=f"{n_legs} legs ({tour_name})",
        measured=f"closure {closure_mm:.1f}mm / {closure_deg:+.1f}deg",
        elapsed_s=elapsed, tolerance=f"closure<{MAX_TOUR_CLOSURE_POSITION_MM:.0f}mm, all legs complete",
        encoder_advanced=moved, verdict="PASS" if ok else "FAIL",
    ))

    assert not timed_out, f"{button}: tour never finished within {TOUR_TIMEOUT_S:.0f}s (hang?)"
    assert not stopped_early, f"{button}: tour stopped early -- log:\n{log_text}"
    assert all_legs_ran, f"{button}: not every leg ran to completion -- log:\n{log_text}"
    assert completed, f"{button}: no completion narration found -- log:\n{log_text}"
    assert moved, f"{button}: no encoder excursion beyond {MIN_ENCODER_SPAN_MM}mm across the whole tour"
    assert closure_mm < MAX_TOUR_CLOSURE_POSITION_MM, (
        f"{button}: closure {closure_mm:.1f}mm implausibly large (start={start}, end={end})"
    )
    assert math.isfinite(closure_deg)


def test_tour_1_runs_to_completion(gui, recorder):
    _run_tour(gui, recorder, button="tour_btn_tour_1", tour_name="Tour 1", n_legs=13)


def test_tour_2_runs_to_completion(gui, recorder):
    _run_tour(gui, recorder, button="tour_btn_tour_2", tour_name="Tour 2", n_legs=15)


# ---------------------------------------------------------------------------
# 4. STOP mid-motion -- motion ceases, queue flushed.
# ---------------------------------------------------------------------------


def test_stop_mid_tour_halts_motion_and_flushes_the_queue(gui, recorder):
    """Starts Tour 1 again, lets it run briefly, clicks the Operations
    panel's STOP button (``ops_btn_stop`` -> ``_stop_all_motion()`` ->
    cancel-and-join the tour worker + wire ``STOP`` + ``STREAM 0``), then
    asserts (a) the tour worker actually stops (``stop_tour_btn``
    re-disables, ``tour_btn_tour_1`` re-enables) promptly, (b) no further
    ``[TOUR] ... leg N/13`` narration appears after a grace window (the
    queue was flushed -- a still-running worker would keep emitting
    per-leg lines), and (c) ground truth pose goes quiet shortly after.

    Runs at 1x sim speed (temporarily) for a deterministic mid-leg
    interrupt window -- at 10x a fixed wall-clock sleep is too close to the
    whole tour's own (compressed) duration to reliably land mid-flight.
    Restores SPEED_FACTOR afterward so later tests in this module are
    unaffected.
    """
    from PySide6.QtWidgets import QComboBox, QPushButton  # type: ignore[import-untyped]

    speed_combo = gui.find(QComboBox, "sim_speed_combo")
    speed_combo.setCurrentIndex(speed_combo.findData(1))
    gui.pump(0.1)

    try:
        tour_btn = gui.find(QPushButton, "tour_btn_tour_1")
        stop_tour_btn = gui.find(QPushButton, "stop_tour_btn")
        assert tour_btn.isEnabled(), "tour_btn_tour_1 must be enabled before this test"

        t0 = time.monotonic()
        tour_btn.click()
        gui.pump(0.5)  # solidly inside leg 2 (a turn) per this tour's own 1x timing
        assert stop_tour_btn.isEnabled(), "tour did not actually start running"

        gui.click("ops_btn_stop")

        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and stop_tour_btn.isEnabled():
            gui.app.processEvents()
            time.sleep(0.02)
        stop_settled = time.monotonic() - t0
        assert not stop_tour_btn.isEnabled(), "Stop Tour did not re-disable after STOP"
        assert tour_btn.isEnabled(), "tour_btn_tour_1 did not re-enable after STOP"

        log_after_stop = gui.log_text()
        leg_lines_at_stop = log_after_stop.count("Tour 1 leg ")

        # Grace window: a still-running worker would keep emitting leg lines.
        gui.pump(1.5)
        elapsed = time.monotonic() - t0
        log_after_grace = gui.log_text()
        leg_lines_after_grace = log_after_grace.count("Tour 1 leg ")

        pose, settle_elapsed, timed_out = gui.settle(3.0)

        queue_flushed = leg_lines_after_grace == leg_lines_at_stop
        ok = (not timed_out) and queue_flushed

        recorder.record(Row(
            button="ops_btn_stop", path="stop", commanded="STOP mid Tour 1",
            measured=f"{leg_lines_at_stop} legs logged at stop, "
                     f"{leg_lines_after_grace} after +1.5s grace",
            elapsed_s=elapsed, tolerance="no further leg lines after STOP",
            encoder_advanced=None, verdict="PASS" if ok else "FAIL",
        ))

        assert queue_flushed, (
            f"tour kept advancing after STOP: {leg_lines_at_stop} leg lines at "
            f"stop-time vs {leg_lines_after_grace} after a 1.5s grace window -- "
            f"queue was not flushed"
        )
        assert not timed_out, "ground truth pose never went quiet after STOP"
    finally:
        speed_combo.setCurrentIndex(speed_combo.findData(SPEED_FACTOR))
        gui.pump(0.1)


# ---------------------------------------------------------------------------
# 5. GOTO -- dormant, no live sim path.
# ---------------------------------------------------------------------------


def test_goto_button_stays_permanently_disabled(gui, recorder):
    """Regression guard: ``goto_btn`` must stay disabled (never silently
    enabled without also fixing the underlying camera-pursuit SI/G gap --
    see ``__main__.py``'s own tooltip on this button, and the skipped test
    right below)."""
    from PySide6.QtWidgets import QPushButton  # type: ignore[import-untyped]

    btn = gui.find(QPushButton, "goto_btn")
    recorder.record(Row(
        button="goto_btn", path="goto", commanded="n/a", measured="disabled (dormant)",
        elapsed_s=0.0, tolerance="n/a", encoder_advanced=None,
        verdict="PASS" if not btn.isEnabled() else "FAIL",
    ))
    assert not btn.isEnabled(), (
        "goto_btn was enabled -- if the camera-pursuit SI/G gap has been "
        "closed, un-skip test_goto_drives_to_target_pose_in_sim below and "
        "give it a real sim path"
    )


@pytest.mark.skip(
    reason="GOTO has no live sim path: goto_btn is permanently disabled "
           "pending sprint 098's SI/G binary pose-reset arm -- its pursuit "
           "loop repeatedly re-anchors pose via SI between re-issued G's, "
           "and SI has no binary arm yet (see __main__.py's own goto_btn "
           "creation comment: 'World-absolute camera GOTO requires sprint "
           "098'). Nothing to click-and-verify motion for until that lands "
           "-- see test_goto_button_stays_permanently_disabled above for "
           "the live regression guard on the button's disabled state."
)
def test_goto_drives_to_target_pose_in_sim(gui, recorder):
    raise AssertionError("unreachable -- see skip reason")
