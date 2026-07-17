"""src/tests/testgui/test_tour1_geometry.py — the tour buttons must actually
tour (107-004 rewrite: ``FakeTransport``-backed, no ctypes sim).

End-to-end headless GUI tests (``QT_QPA_PLATFORM=offscreen``) driving the
REAL GUI objects — the transport combo + Connect button, the tour
``QPushButton``s, ``_TourRunner`` on its own ``QThread`` — exactly the path
an operator exercises. Only the BACKING transport is fake; nothing about
"drive it through the real GUI" changes from this file's original
(085-002/086-004/097) incarnation.

What changed from the pre-107-004 version of this file
--------------------------------------------------------
The original file drove tours against a real ``SimTransport`` wrapping the
``src/sim`` ctypes firmware simulator (built via ``cmake --build
build`` in that directory), gated the whole file behind an ``_LIB_PRESENT``
``skipif`` when that library wasn't built. Two independent, later changes
made that approach dead:

1. The ctypes sim was deleted wholesale at sprint 102 ticket 005 (``git show
   72d8be7e --stat``) — ``_LIB_PRESENT`` has been permanently ``False``
   ever since, so every test in this file has silently SKIPPED on every
   ``uv run python -m pytest`` run since before the single-loop rebuild
   (``tests/testgui`` was also dropped from ``pyproject.toml``'s
   ``testpaths`` at the same ticket, so nobody even saw the skip). Rebuilding
   that sim library is explicitly out of scope this sprint
   (``architecture-update.md`` Decision 1, sprint 107).
2. 107-003 rewired ``_TourRunner`` onto ``planner.tour.run_tour()``, driven
   directly against ``transport.protocol`` (a ``NezhaProtocol``-shaped
   ``TwistTransport``) — a property ``_HardwareTransport`` (the
   ``SerialTransport``/``RelayTransport`` base) exposes but ``SimTransport``
   does NOT. Tour buttons are therefore explicitly GATED OFF (disabled, with
   an explanatory tooltip — see ``__main__.py``'s ``_tour_hw_tooltip()``/
   ``_TOUR_SIM_TOOLTIP`` and the ``is_sim_transport()`` gating in
   ``_on_connect()``) whenever connected via Sim: "Tours require a
   real-hardware connection this sprint". Even with a rebuilt sim library,
   clicking a tour button against a live ``SimTransport`` connection today
   is a no-op (the button is disabled) — the pre-107-004 version of this
   file's whole approach (Sim connect -> click tour button) no longer
   reaches ``_TourRunner`` at all.

This rewrite drives the SAME real GUI/``_TourRunner``/``QThread`` stack, but
connects via the "Serial" combo entry (so 107-003's real-hardware-only tour
gate leaves the buttons enabled — see point 2 above) with
``transport.SerialTransport`` monkeypatched to ``_FakeHardwareTransport``
(this file, below): a ``Transport`` that looks like a connected, non-Sim
backend to every ``is_sim_transport()``/``isinstance(..., SimTransport)``
check in ``__main__.py``, but talks to nothing real. Its ``.protocol``
exposes ``_FakeTwistTransport`` — a double satisfying
``planner.executor.TwistTransport``'s structural protocol (``twist()``/
``stop()``/``read_pending_binary_tlm_frames()``), mirroring
``src/tests/unit/test_planner_executor.py``'s own ``FakeTransport`` convention
(the project's established double style for this exact protocol) — driven
through the real GUI/``_TourRunner``/``QThread``, not called directly.
``_FakeTwistTransport`` synthesizes a plausible, monotonically-advancing
encoder pose on every ``twist()`` (open-loop unicycle integration: heading
+= omega*dt, x/y advance along the post-turn heading by v_x*dt, dt taken
from ``twist()``'s own ``duration`` argument), so ``run_tour()``'s own
closure computation (``planner/tour.py``'s ``TourClosure`` — the pose
delta between leg 1's ``begin()`` and the final leg's settle window) has
something meaningful to compute against, and every ``StreamingExecutor``
safety check that needs feedback (bounded-overshoot, heading trim) sees
believable, moving telemetry rather than a frozen frame.

Bar for this file, per ticket 107-004's own scope
---------------------------------------------------
``run_tour()``'s own leg-chaining/closure-math/preemption behavior against a
scripted ``FakeTransport`` is ALREADY exhaustively covered by
``src/tests/unit/test_planner_tour.py`` (ticket 002) — this file does not
re-prove that. This file's job is proving the GUI's own wiring is correct:
the tour buttons actually reach ``_TourRunner``/``run_tour()`` with real
Qt/``QThread`` machinery in the loop, each tour runs to completion (no leg
timing out) with per-leg ``[TOUR]`` log narration appearing in the log pane,
and Stop Tour mid-run re-enables the tour buttons synchronously against a
REAL, running tour (complementing ``test_tour_stop.py``'s own inline,
Qt-free re-implementation of the exact same control flow). No physics
accuracy claim is made (or checked) here — the fake transport integrates
open-loop with zero slip/noise, so a tight closure number would prove
nothing about the real robot; ticket 005's bench script is where physical
closure is measured against real hardware.

Run:
    QT_QPA_PLATFORM=offscreen uv run pytest src/tests/testgui/test_tour1_geometry.py -v
"""
from __future__ import annotations

import math
import time

import pytest

# 107-004: turn a missing `gui` dependency group into a clean skip, not a
# hard collection/run error, so re-adding src/tests/testgui/ to testpaths this
# ticket never breaks a headless CI run that hasn't `uv sync --group gui`'d
# (the aprilcam/OpenCV-bearing group `default-groups` deliberately excludes
# -- pyproject.toml's own comment). Every other qapp fixture in this
# directory picked up the same guard this ticket (see each file's own
# import line).
pytest.importorskip("PySide6")

# _FakeTwistTransport's own nominal tick interval -- read off PlannerParams'
# own default rather than duplicated as a hand-picked literal (0.15) that
# would silently drift if that default ever changed. Valid because
# _TourRunner.run() (__main__.py) always constructs a FRESH, default
# PlannerParams() for every tour run -- no override reaches this file (see
# _FakeTwistTransport's own class docstring for why this matters).
from robot_radio.planner.model import PlannerParams  # noqa: E402

_NOMINAL_TICK_INTERVAL_S = PlannerParams().streaming_interval  # [s]


# ---------------------------------------------------------------------------
# Tour geometry integrity -- Qt-free, fast: the GUI's own TOURS dict (what
# the tour buttons are actually labeled/wired from) is the SAME data as
# planner.tour's TOUR_1/TOUR_2 (commands.py imports them directly -- see
# that module's own 107-002 comment), not a stale duplicate copy.
# ---------------------------------------------------------------------------


def test_gui_tours_dict_is_the_real_tour_1_and_tour_2_geometry():
    from robot_radio.planner.tour import TOUR_1, TOUR_2, parse_tour
    from robot_radio.testgui.commands import TOURS

    assert TOURS["Tour 1"] is TOUR_1
    assert TOURS["Tour 2"] is TOUR_2

    legs_1 = parse_tour(TOURS["Tour 1"])
    legs_2 = parse_tour(TOURS["Tour 2"])
    assert len(legs_1) == 13
    assert len(legs_2) == 15
    # Every step parses to a recognized "distance"/"turn" leg (parse_tour()
    # raises ValueError on anything else) -- the assertion above already
    # proves this by not raising, but assert the leg-kind mix explicitly so
    # a future tour that accidentally drops all its turns (or vice versa)
    # fails loudly here instead of only inside the slow end-to-end tests
    # below.
    assert {leg.kind for leg in legs_1} == {"distance", "turn"}
    assert {leg.kind for leg in legs_2} == {"distance", "turn"}


# ---------------------------------------------------------------------------
# _FakeTwistTransport -- TwistTransport double (mirrors
# src/tests/unit/test_planner_executor.py's own FakeTransport convention),
# driven through the real GUI/_TourRunner/QThread stack below, never called
# directly by this file's own test bodies.
# ---------------------------------------------------------------------------


class _FakeTwistTransport:
    """Synthesizes a plausible, monotonically-advancing encoder pose on
    every ``twist()`` (see module docstring) -- open-loop unicycle
    integration, no slip/noise. ``enc``/``pose``/``otos`` all carry the SAME
    synthesized values so this file's outcome is independent of whichever
    field the active robot config's ``geometry.otos_untrusted`` happens to
    select for ``HeadingCorrector`` (``planner/heading.py``).

    ``dt`` is a FIXED, NOMINAL interval (``_NOMINAL_TICK_INTERVAL_S``, sourced
    from ``PlannerParams()``'s own default ``streaming_interval`` -- not a
    hand-duplicated literal, so this fake tracks that default automatically
    if it ever changes), NOT measured wall-clock time and NOT ``twist()``'s
    own ``duration`` argument. Two things were tried and rejected first:

    1. ``twist()``'s own ``duration`` argument -- the firmware deadman ARM
       window (``streaming_interval + link_latency_margin``, deliberately
       padded past the tick cadence so the deadman never expires between
       ticks -- ``executor.py``'s own docstring, binding requirement #8),
       not the actual spacing between two ``twist()`` sends. Integrating
       against it double-counts the latency margin on every tick and
       over-advances the synthesized pose (confirmed: trips the executor's
       own bounded-overshoot check almost immediately).
    2. Real ``time.monotonic()``-measured elapsed time between calls --
       correct in isolation (``run_tour()`` really does pace ticks via
       ``time.sleep(params.streaming_interval)``), but flaky under the FULL
       suite's load (1000+ preceding tests): scheduling jitter/GC pauses can
       stretch an individual tick's actual wall-clock gap well past the
       nominal 150ms, over-advancing that one tick's synthesized distance
       enough to trip the SAME bounded-overshoot check the ``duration`` bug
       did -- confirmed via team-lead's full-suite run
       (``test_tour2_runs_to_completion_with_per_leg_log_narration`` failed
       in full-suite ordering, passed in isolation; not a Qt/PlannerParams/
       fixture leak -- this class's OWN wall-clock coupling was the
       isolation defect). A fake whose correctness depends on the test
       process's real-time scheduling fidelity is unsound test design
       regardless of how it behaves on an idle machine; the nominal-interval
       fix removes the coupling to real-time scheduling entirely, matching
       what ``_TourRunner.run()`` ACTUALLY configures (a fresh, default
       ``PlannerParams()`` every call -- no override reaches this file), not
       what the wall clock happens to measure on any given run.

    ``read_pending_binary_tlm_frames()`` always returns a single frame
    reflecting the CURRENT state (mirrors ``test_planner_tour.py``'s own
    "current frame" double convention -- simpler than
    ``test_planner_executor.py``'s batch-queue double, and deliberately so
    here: a real telemetry stream pushes continuously (~25Hz) independent of
    whether a command was just sent, so it is never genuinely empty by the
    time ``StreamingExecutor.begin()``'s own bounded retry drains it for a
    fresh leg. An empty-then-refilled queue (this class's own first
    attempt) starves a leg's ``begin()`` of a fresh baseline whenever the
    inter-leg settle window's own discard-read empties the queue before the
    new leg's first ``twist()`` -- confirmed: baseline silently resets to
    0.0 instead of the prior leg's own end-state, and the next leg
    immediately false-trips the bounded-overshoot check against its own
    stale, un-baselined progress).
    """

    def __init__(self) -> None:
        self.twist_calls: list[tuple[float, float, float]] = []
        self.move_calls: list[dict] = []
        self.stop_calls: int = 0
        self._corr_id = 0
        self._pending_acks: list[int] = []  # 109-008: move ids awaiting delivery -- see move()
        self._x = 0.0    # [mm]
        self._y = 0.0    # [mm]
        self._heading = 0.0  # [rad]
        self._enc = 0.0  # [mm] forward-distance accumulator (StreamingExecutor
        # reads (enc[0]+enc[1])/2 as "linear" progress -- both wheels report
        # the same value, this fake models no per-wheel differential)

    def twist(self, v_x: float, omega: float, duration: float) -> int:  # [mm/s] [rad/s] [ms]
        self._corr_id += 1
        self.twist_calls.append((v_x, omega, duration))
        dt = _NOMINAL_TICK_INTERVAL_S  # [s] -- see class docstring
        self._heading += omega * dt
        self._x += v_x * math.cos(self._heading) * dt
        self._y += v_x * math.sin(self._heading) * dt
        self._enc += v_x * dt
        return self._corr_id

    def stop(self) -> int:
        self._corr_id += 1
        self.stop_calls += 1
        return self._corr_id

    def move(self, *, distance: float = 0.0, delta_heading: float = 0.0,
             v_max: float = 0.0, omega: float = 0.0, time: float = 0.0,
             replace: bool = False, id: "int | None" = None) -> int:  # [mm] [rad] [mm/s] [rad/s] [ms]
        """109-008: MOVE-queue counterpart of ``twist()`` above -- this fake
        has no real firmware queue/timing to model, so it integrates the
        WHOLE commanded arc in one shot (open-loop unicycle, same shape
        ``twist()`` uses per-tick) and immediately queues a `DONE`
        completion ack for this id (mirrors ``planner.tour``'s own "anything
        but OK is terminal" contract -- see that module's file header)."""
        self._corr_id += 1
        move_id = id if id is not None else self._corr_id
        self.move_calls.append(dict(distance=distance, delta_heading=delta_heading,
                                    v_max=v_max, omega=omega, time=time,
                                    replace=replace, id=move_id))
        self._heading += delta_heading
        self._x += distance * math.cos(self._heading)
        self._y += distance * math.sin(self._heading)
        self._enc += distance
        self._pending_acks.append(move_id)
        return move_id

    def read_pending_binary_tlm_frames(self) -> list:
        return [self._make_frame()]

    def _make_frame(self):
        from robot_radio.robot.pb2 import telemetry_pb2
        from robot_radio.robot.protocol import AckEntry, TLMFrame

        enc_i = int(self._enc)
        pose = (
            int(self._x), int(self._y),
            int(round(math.degrees(self._heading) * 100.0)),  # [cdeg]
        )
        acks = tuple(
            AckEntry(corr_id=move_id, ok=True, err_code=0, status=telemetry_pb2.ACK_STATUS_DONE)
            for move_id in self._pending_acks)
        return TLMFrame(enc=(enc_i, enc_i), pose=pose, otos=pose,
                        fault_bits=0, event_bits=0, acks=acks)


# ---------------------------------------------------------------------------
# _FakeHardwareTransport -- looks like a connected, non-Sim Transport to
# every is_sim_transport()/isinstance(..., SimTransport) check in
# __main__.py (see module docstring, point 2), so 107-003's real-hardware-
# only tour gate leaves the buttons enabled. Talks to nothing real.
# ---------------------------------------------------------------------------


def _make_fake_hardware_transport_class():
    """Builds ``_FakeHardwareTransport`` lazily (needs ``transport.Transport``,
    a PySide6-adjacent import deferred like everywhere else in this
    directory)."""
    from robot_radio.testgui.transport import Transport

    class _FakeHardwareTransport(Transport):
        """Fake, non-Sim ``Transport`` -- every command/send is a tolerant
        no-op (mirrors how ``__main__.py`` itself treats an empty/failed
        reply: best-effort, logged, never raised past the caller -- see
        e.g. ``_check_firmware_version()``'s/``_push_robot_calibration()``'s
        own ``except Exception`` handling). ``.protocol`` exposes a fresh
        ``_FakeTwistTransport`` once "connected"."""

        def __init__(self, port: str = "") -> None:
            super().__init__()
            self._port = port
            self._connected = False
            self._twist_transport = _FakeTwistTransport()
            self._suspended = False

        def connect(self) -> None:
            self._connected = True

        def disconnect(self) -> None:
            self._connected = False

        def send(self, line: str) -> None:
            pass

        def command(self, line: str, read_timeout: int = 200) -> str:  # [ms]
            return ""

        # -- 107-003's twist surface (see transport.py's _HardwareTransport
        # -- this fake is the "Serial" stand-in, so it needs the SAME
        # protocol/suspend/resume surface _TourRunner.run() actually calls).
        @property
        def protocol(self):
            return self._twist_transport if self._connected else None

        def suspend_telemetry_reader(self) -> None:
            self._suspended = True

        def resume_telemetry_reader(self) -> None:
            self._suspended = False

    return _FakeHardwareTransport


# ---------------------------------------------------------------------------
# GUI harness
# ---------------------------------------------------------------------------

#: Wall-clock ceilings -- real 1x pacing (StreamingExecutor's own
#: streaming_interval-paced sleep_fn=time.sleep; _TourRunner.run() calls
#: run_tour() with no clock_fn/sleep_fn override, so this is genuinely real
#: time, same as the pre-107-004 version of this file against real
#: SimTransport pacing).
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


def _connect_via_fake_hardware(qapp, monkeypatch):
    """Build the real GUI window and Connect via "Serial", monkeypatched
    onto ``_FakeHardwareTransport`` (see class docstring above). Returns
    ``(window, fake_transport_class)`` -- callers here never need the
    constructed instance itself (every assertion reads button state / the
    log pane), only that Connect succeeded.

    Mirrors this file's own pre-107-004 ``SimTransport``-monkeypatch
    technique (``__main__.py``'s transport classes are imported function-
    locally inside ``_build_main_window()``, so patching the module
    attribute BEFORE that call is the only seam available -- there is no
    dependency-injection hook).
    """
    from PySide6.QtWidgets import (  # type: ignore[import-untyped]
        QComboBox,
        QLineEdit,
        QPushButton,
    )

    import robot_radio.testgui.__main__ as gui_main
    from robot_radio.testgui import transport as transport_mod

    fake_cls = _make_fake_hardware_transport_class()
    monkeypatch.setattr(transport_mod, "SerialTransport", fake_cls)

    window, _app = gui_main._build_main_window()

    combo = window.findChild(QComboBox, "transport_combo")
    assert combo is not None, "transport_combo not found"
    combo.setCurrentText("Serial")

    # Pre-fill a bogus-but-non-empty port so _on_connect()'s auto-detect
    # branch (which scans real serial ports via list_ports()) is skipped
    # entirely -- SerialTransport(port) is monkeypatched to ignore the
    # value anyway.
    port_edit = window.findChild(QLineEdit, "port_edit")
    assert port_edit is not None, "port_edit not found"
    port_edit.setText("FAKE0")

    connect_btn = window.findChild(QPushButton, "connect_btn")
    assert connect_btn is not None, "connect_btn not found"
    connect_btn.click()
    _spin_events(qapp, 0.3)

    return window, fake_cls


# ---------------------------------------------------------------------------
# End-to-end: tour buttons drive _TourRunner/run_tour() via the real GUI
# ---------------------------------------------------------------------------


def _run_tour_via_gui(qapp, monkeypatch, button_name: str, tour_label: str) -> str:
    """Connect via the fake hardware transport, click ``button_name``, wait
    for it to run to completion, and return the log pane's full text (for
    ``[TOUR]`` narration assertions). Always disconnects/hides the window in
    a ``finally``."""
    from PySide6.QtWidgets import QPlainTextEdit, QPushButton  # type: ignore[import-untyped]

    window, _fake_cls = _connect_via_fake_hardware(qapp, monkeypatch)

    try:
        tour_btn = window.findChild(QPushButton, button_name)
        stop_btn = window.findChild(QPushButton, "stop_tour_btn")
        assert tour_btn is not None, f"tour button {button_name!r} not found"
        assert stop_btn is not None
        assert tour_btn.isEnabled(), (
            f"{button_name} not enabled after connecting via Serial -- "
            "107-003's real-hardware-only tour gate should leave tour "
            "buttons enabled for any non-Sim transport"
        )

        tour_btn.click()

        assert _spin_until(qapp, stop_btn.isEnabled, _TOUR_START_DEADLINE_S), (
            "tour never started (Stop Tour button did not enable)"
        )
        assert _spin_until(
            qapp, lambda: not stop_btn.isEnabled(), _TOUR_DEADLINE_S
        ), (
            f"{tour_label} did not finish within {_TOUR_DEADLINE_S:.0f} s "
            "wall clock -- a leg timed out (see the [TOUR] log lines for "
            "which one)"
        )
        # Let the final [TOUR] complete/closure log line land.
        _spin_events(qapp, 0.2)

        log_pane = window.findChild(QPlainTextEdit, "log_pane")
        assert log_pane is not None, "log_pane not found"
        return log_pane.toPlainText()
    finally:
        disconnect_btn = window.findChild(QPushButton, "disconnect_btn")
        if disconnect_btn is not None and disconnect_btn.isEnabled():
            disconnect_btn.click()
            _spin_events(qapp, 0.3)
        window.hide()


@pytest.mark.slow
def test_tour1_runs_to_completion_with_per_leg_log_narration(qapp, monkeypatch):
    """Tour 1 (13 legs) runs to completion via the real GUI/_TourRunner
    stack against the fake transport -- no leg times out -- and every leg's
    ``[TOUR] Tour 1 leg i/13: ...`` narration line lands in the log pane.

    Real wall-clock pacing (``run_tour()``'s own ``StreamingExecutor`` paces
    ticks via ``time.sleep(params.streaming_interval)``, and ``_TourRunner.
    run()`` injects no faster clock) -- ~45s. Marked ``slow`` so a fast local
    loop can deselect it (``pytest -m "not slow"``); stays in the default run
    (this ticket's own AC5)."""
    log_text = _run_tour_via_gui(qapp, monkeypatch, "tour_btn_tour_1", "Tour 1")

    assert "[TOUR] Tour 1 starting" in log_text
    for i in range(1, 14):
        assert f"[TOUR] Tour 1 leg {i}/13:" in log_text, (
            f"leg {i}/13 narration missing from the log pane -- full log:\n{log_text}"
        )
    assert "[TOUR] Tour 1 complete" in log_text
    assert "stopped at leg" not in log_text


@pytest.mark.slow
def test_tour2_runs_to_completion_with_per_leg_log_narration(qapp, monkeypatch):
    """Tour 2 (15 legs, mixed-sign turns) runs to completion the same way
    (~45s, real wall-clock pacing -- see ``test_tour1_runs_...``'s own
    docstring)."""
    log_text = _run_tour_via_gui(qapp, monkeypatch, "tour_btn_tour_2", "Tour 2")

    assert "[TOUR] Tour 2 starting" in log_text
    for i in range(1, 16):
        assert f"[TOUR] Tour 2 leg {i}/15:" in log_text, (
            f"leg {i}/15 narration missing from the log pane -- full log:\n{log_text}"
        )
    assert "[TOUR] Tour 2 complete" in log_text
    assert "stopped at leg" not in log_text


def test_stopping_a_running_tour_reenables_buttons_synchronously(qapp, monkeypatch):
    """Clicking Stop Tour mid-run re-enables the tour buttons immediately.

    Complements ``test_tour_stop.py``'s inline re-implementation (which
    exercises the exact ``_stop_tour()`` control flow deterministically
    against fake worker/thread doubles) by clicking the REAL Stop Tour
    button against a live, running tour -- the ``QPushButton.clicked``
    signal is a same-thread (GUI-thread) direct connection, so
    ``_stop_tour()`` runs synchronously inside ``.click()``; no additional
    event-loop spin should be needed before the buttons reflect the stopped
    state (acceptance criterion: re-enable is synchronous, "not dependent on
    the finished signal being delivered during the blocking thread.wait()"
    -- testgui-tour-stop-reactivation.md).
    """
    from PySide6.QtWidgets import QPushButton  # type: ignore[import-untyped]

    window, _fake_cls = _connect_via_fake_hardware(qapp, monkeypatch)

    try:
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
