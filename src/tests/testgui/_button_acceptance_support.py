"""src/tests/testgui/_button_acceptance_support.py -- shared helpers for
``test_gui_button_acceptance.py`` (the standing headless TestGUI
button-acceptance suite, stakeholder directive 2026-07-22): "run all the
buttons that I'm gonna press and get a trace of those things ... without
me clicking. It has to be part of the acceptance test."

Qt-free by design: everything here operates on plain ``SimLoop``/
``SimTransport`` objects and plain data, so it can be imported and unit
exercised without a ``QApplication``. The one exception is ``settle_pose()``,
which pumps the Qt event loop while polling ground-truth pose so that Qt
signal/slot delivery (background threads posting queued events back to the
GUI thread) keeps flowing while a test waits for motion to finish -- passing
``qapp=None`` skips that and is fine for tests that drive the sim directly
(no GUI objects in play).

Tolerance model
----------------
Every distance/angle assertion in the acceptance suite uses the SAME
two-term model against ``SimLoop.get_true_pose()`` (ground truth, not a
noisy sensor reading -- see that method's own docstring: it "bypasses every
drift/noise fault knob"):

    allowed_error = abs_margin + rel_tol * abs(commanded)

A single relative percentage (the naive "within +/-N%" framing) does not
fit the REAL measured behavior: the managed (D/RT/SEG -- Move-queue) path
carries a near-CONSTANT absolute overshoot regardless of commanded
magnitude (~26-29mm / ~19-23deg, empirically measured 2026-07-22 against
``data/robots/tovez_nocal.json`` at zero injected sim error -- consistent
with a fixed ~150-180ms stop-detection/actuation lag: 150mm/s * 0.18s ~=
27mm, 2rad/s * 0.18s ~= 20.6deg -- see this project's own "Actuation
latency & delay-in-plan" knowledge note). Expressed as a pure percentage,
that fixed lag makes the SHORTEST preset (100mm / 90deg) look like a wild
~22-28% miss while the LONGEST preset (700mm / 360deg) looks like a tight
~4-6% hit -- same underlying defect-free behavior, wildly different
percentage. The abs_margin term absorbs the fixed lag; rel_tol absorbs the
residual proportional error. See ``test_gui_button_acceptance.py``'s own
module-level tolerance constants for the concrete numbers per path.

Turn-prediction campaign (2026-07-22, ``App::MoveQueue``'s former stop-
condition time-lead anticipation constant, since DELETED -- see below) /
wire-testgui-live-push-of-estimator-stop-lead fix (same day, follow-up):
``MANAGED_ANGLE_ABS_MARGIN_DEG``/``MANAGED_DIST_ABS_MARGIN_MM`` below were
ORIGINALLY left deliberately UNCHANGED by the turn-prediction campaign,
even though ``test_tour_closure_gate.py``'s own sweep (against the SAME
managed Move-queue path, with the lead pushed live via
``EstimatorConfigPatch``) showed the fixed lag shrinking measurably --
because the GUI's own connect-time push (``__main__.py``'s
``_push_robot_calibration()``) did not yet source the estimator fields/
shaper limits from the robot JSON at all: ``robot_radio.config.
robot_config.RobotConfig`` had no ``estimator`` field, and
``EstimatorConfigPatch`` has no ``SET key=value`` text form
(``calibration_commands()`` never covered it), so a live TestGUI Sim
session ran with anticipation/shaping OFF regardless of what the robot
JSON said. THAT FIX closed the gap: ``RobotConfig.estimator`` (new
``EstimatorConfig`` model) + ``__main__.py``'s ``_push_estimator_config()``
push ``estimator.weight_heading_otos``/``weight_omega_otos``/
``staleness_ms``/``control.a_max``/``a_decel``/``alpha_max``/
``alpha_decel``/``j_max``/``yaw_jerk_max`` via
``NezhaProtocol.estimator_config()`` on every Connect/robot-select, both
transports -- see ``clasi/issues/wire-testgui-live-push-of-estimator-stop-lead.md``
(now resolved) for the full history.

118 ticket 004 (land-at-zero-completion-delete-stop-lead.md): the time-
lead anticipation constant itself is DELETED -- the completion mechanism
it drove no longer exists (see ``App::MoveQueue::tick()``'s own doc
comment for the land-at-zero predicate that replaces it), so the connect-
time push above now carries nine fields, not ten.

``MANAGED_ANGLE_ABS_MARGIN_DEG``/``MANAGED_DIST_ABS_MARGIN_MM`` above are
STILL left unchanged (they cover 180/270/360-degree and all distance
presets, not re-measured/re-tuned by this fix) -- this fix instead adds a
NEW, tight ``MANAGED_ANGLE_90_*`` band (test_gui_button_acceptance.py) for
exactly the +/-90deg magnitude this campaign's own shaper tuning targets,
and a ``TOUR_TURN_ERROR_MAX_DEG`` per-leg bound for Tour 1/Tour 2, both
measured through the REAL GUI Connect -> click flow (not a direct SimLoop
push) -- see those constants' own module-level comments in
test_gui_button_acceptance.py for the concrete numbers and the "stakeholder
sign-off to widen" contract.
"""
from __future__ import annotations

import csv
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

#: Ground-truth pose dict shape returned by ``SimLoop.get_true_pose()``.
Pose = dict


def settle_pose(
    get_pose: "Callable[[], Pose]",
    *,
    timeout_s: float,
    quiet_s: float = 0.25,
    poll_s: float = 0.03,
    pump: "Callable[[], None] | None" = None,
) -> "tuple[Pose, float, bool]":
    """Poll ``get_pose()`` (normally ``SimLoop.get_true_pose``) until the
    pose stops changing for ``quiet_s`` of wall-clock time, bounded by
    ``timeout_s``. Returns ``(final_pose, elapsed_s, hit_timeout)``.

    This is the acceptance suite's own "did the motion actually finish"
    signal: SimTransport's managed/unmanaged motion dispatch is
    fire-and-poll (see ``transport.py``'s ``SimTransport._run_motion_async``/
    ``run_unmanaged`` docstrings -- neither blocks until the Move's own
    completion ack, unlike the hardware transport's
    ``_await_move_completion``), so there is no single call that blocks
    until "done". Polling ground truth for a quiet window is the direct,
    honest substitute: the firmware's OWN stop condition (distance/angle/
    deadman-timeout) is what actually halts the plant, and a quiet ground
    truth is the observable proof that happened -- this function does not
    assume any particular stop mechanism, so it works identically for
    ``run_unmanaged()`` (deadman twist), the managed Move-queue dispatch,
    and a raw ``twist()``.

    ``pump``, when given, is called once per poll iteration (e.g.
    ``qapp.processEvents``) so a caller driving the real GUI keeps its Qt
    event loop alive while waiting -- otherwise queued signals (log lines,
    telemetry bridge slots) queue up unprocessed and a later
    ``processEvents()`` burst-delivers a big backlog instead of the steady
    trickle production sees.
    """
    start = time.monotonic()
    last = get_pose()
    last_change = start
    while True:
        now = time.monotonic()
        if now - start > timeout_s:
            return get_pose(), now - start, True
        time.sleep(poll_s)
        if pump is not None:
            pump()
        cur = get_pose()
        if (abs(cur["x"] - last["x"]) > 0.05 or abs(cur["y"] - last["y"]) > 0.05
                or abs(cur["h"] - last["h"]) > math.radians(0.05)):
            last_change = time.monotonic()
        last = cur
        if time.monotonic() - last_change > quiet_s:
            return cur, time.monotonic() - start, False


def signed_distance(start: Pose, end: Pose, *, forward_sign: float) -> float:  # [mm]
    """Euclidean distance traveled between ``start`` and ``end``, signed by
    ``forward_sign`` (+1.0 for a commanded-forward preset, -1.0 for
    commanded-reverse) -- ``get_true_pose()`` only reports position, not a
    direction of travel, so the caller supplies the expected sign (the sim
    trivially satisfies this: a straight preset never travels backward
    relative to its own commanded direction)."""
    dx = end["x"] - start["x"]
    dy = end["y"] - start["y"]
    return math.copysign(math.hypot(dx, dy), forward_sign)


def signed_heading_delta(start: Pose, end: Pose) -> float:  # [deg]
    """Signed heading change from ``start`` to ``end``, in degrees, folded
    into a continuous (non-wrapped-to-+/-180) range appropriate for a
    commanded turn well past 180 degrees (e.g. a 360-degree preset) by
    assuming the turn never reverses direction or overshoots by more than
    a half turn -- true for every preset this suite drives."""
    dh = math.degrees(end["h"] - start["h"])
    while dh > 540:
        dh -= 360
    while dh < -540:
        dh += 360
    return dh


def allowed_error(commanded: float, *, abs_margin: float, rel_tol: float) -> float:
    """The two-term tolerance ``abs_margin + rel_tol * |commanded|`` --
    see this module's own docstring for the rationale."""
    return abs_margin + rel_tol * abs(commanded)


@dataclass
class TurnCheck:
    """One turn leg's commanded-vs-achieved heading, measured against sim
    ground truth -- the SAME shape ``test_tour_closure_gate.py``'s own
    ``TurnCheck``/``_run_tour_capture()`` use. Duplicated here (not
    imported cross-module) because that module's own ``_run_tour_capture()``
    drives a bare ``SimLoop`` directly via ``planner.tour.run_tour()``,
    while ``TourLegCapture`` below instruments the REAL button-driven tour
    (``__main__.py``'s ``_TourRunner``) -- same instrumentation shape,
    different call site, per this fix's own acceptance wording ("assert
    per-leg errors from the same run_tour instrumentation the closure gate
    uses")."""

    index: int
    commanded_deg: float
    achieved_deg: float
    error_deg: float


def normalize_deg(delta_deg: float) -> float:
    """Wrap a signed degree delta to (-180, 180] -- same convention
    ``test_tour_closure_gate.py``'s own ``_normalize_deg()`` uses."""
    while delta_deg > 180.0:
        delta_deg -= 360.0
    while delta_deg <= -180.0:
        delta_deg += 360.0
    return delta_deg


class TourLegCapture:
    """Monkeypatches ``robot_radio.planner.tour.run_tour`` -- the SAME
    function ``__main__.py``'s ``_TourRunner.run()`` calls (imported fresh
    at call time via ``from robot_radio.planner.tour import ...
    run_tour``, so patching the module attribute before a Tour button click
    takes effect) -- so a live GUI Tour-button click captures PER-LEG turn
    accuracy against sim ground truth, exactly the way
    ``test_tour_closure_gate.py``'s own ``_run_tour_capture()`` does for its
    bare-``SimLoop`` runs, but applied to the REAL button-driven tour
    instead.

    Wraps (never replaces) the real ``run_tour()``: the wrapper's own
    ``on_leg`` reads ``get_true_pose()`` before/after each leg to compute
    ``TurnCheck.achieved_deg``/``error_deg`` for ``turn`` legs, appends to
    ``self.turns``, then calls through to whatever ``on_leg`` the caller
    (``_TourRunner``) itself passed -- so the GUI's own per-leg log
    narration is completely unaffected; this is a pure observer.

    Reads a RAW, immediate ``get_true_pose()`` at each leg boundary --
    deliberately NOT a ``settle_pose()`` quiet-window wait, unlike every
    other assertion in this suite. Tried the settle-wait first (2026-07-22)
    on the theory that it would absorb real-tick-thread scheduling jitter
    the way it does for single-preset assertions; it made per-leg error
    dramatically WORSE and consistently biased low (~30-45deg short on
    EVERY leg) instead of better. Root cause: `run_tour()`'s own one-leg
    lookahead (`send_leg(index+1)` issued before the CURRENT leg's own
    terminal is even awaited) means the Move-queue keeps the robot in
    CONTINUOUS motion leg-to-leg -- there is no idle/quiet window at a tour
    leg boundary the way there is after an isolated single-preset click
    (which IS followed by a real stop). `settle_pose()`'s quiet-window wait
    never finds quiescence (the robot is already driving the NEXT leg), so
    it just times out and returns whatever pose happens to be current at
    that point -- capturing the plant PARTWAY INTO the next leg's own
    motion instead of the current leg's endpoint. Reverted to the raw
    synchronous read (the SAME technique `test_tour_closure_gate.py`'s own
    `_run_tour_capture()` uses) -- the real jitter that DOES exist in the
    raw read (measured up to ~3.9deg run-to-run on a real tick thread,
    vs. ~1.4-2.2deg on that file's own deterministic-stepped harness with
    the IDENTICAL pushed config) is absorbed by
    ``TOUR_TURN_ERROR_MAX_DEG``'s own margin instead -- see that
    constant's comment in test_gui_button_acceptance.py.

    Construct with the standard function-scoped ``monkeypatch`` fixture so
    the patch is undone automatically at test teardown -- no manual
    lifecycle needed.
    """

    def __init__(self, monkeypatch, get_true_pose: "Callable[[], Pose]") -> None:
        import robot_radio.planner.tour as tour_mod

        self._get_true_pose = get_true_pose
        self.turns: list[TurnCheck] = []
        real_run_tour = tour_mod.run_tour

        def _wrapped(transport, params, heading, legs, *, on_leg=None, **kwargs):
            true_poses = [self._get_true_pose()]

            def _capture(index, total, leg, leg_result):
                pose = self._get_true_pose()
                if leg.kind == "turn":
                    before_h_deg = math.degrees(true_poses[-1]["h"])
                    after_h_deg = math.degrees(pose["h"])
                    achieved = normalize_deg(after_h_deg - before_h_deg)
                    self.turns.append(TurnCheck(
                        index=index, commanded_deg=leg.value, achieved_deg=achieved,
                        error_deg=normalize_deg(achieved - leg.value)))
                true_poses.append(pose)
                if on_leg is not None:
                    on_leg(index, total, leg, leg_result)

            return real_run_tour(transport, params, heading, legs, on_leg=_capture, **kwargs)

        monkeypatch.setattr(tour_mod, "run_tour", _wrapped)

    def clear(self) -> None:
        self.turns = []

    def worst_deg(self) -> float:
        """Worst |error_deg| across every captured turn leg, or 0.0 if none
        captured yet (caller should treat an empty ``self.turns`` after a
        tour run as its own failure -- see the calling test)."""
        return max((abs(t.error_deg) for t in self.turns), default=0.0)


def encoder_span(frames: "list") -> "tuple[float, float] | None":
    """Max-minus-min excursion of each wheel's encoder position across
    ``frames`` (a list of ``TLMFrame``, each carrying ``.enc = (left,
    right)`` in mm) -- ``(left_span, right_span)``, or ``None`` if no frame
    carries encoder data. Used as the "did the robot actually move" signal,
    independent of ground-truth pose: a real encoder emits +/-1 LSB rest
    dither (project knowledge: WheelPlant's seeded rest-gated dither for
    the wedge detector), so this checks for an excursion clearly beyond
    that noise floor, not merely "changed at all"."""
    lefts = [f.enc[0] for f in frames if f.enc is not None]
    rights = [f.enc[1] for f in frames if f.enc is not None]
    if not lefts or not rights:
        return None
    return (max(lefts) - min(lefts), max(rights) - min(rights))


@dataclass
class Row:
    """One row of the per-button trace table -- see
    ``test_gui_button_acceptance.py``'s module docstring for the
    acceptance contract this backs."""

    button: str
    path: str  # "unmanaged" / "managed" / "seg" / "test" / "tour" / "stop" / "goto"
    commanded: str
    measured: str
    elapsed_s: float
    tolerance: str
    encoder_advanced: "bool | None"
    verdict: str  # "PASS" / "FAIL" / "XFAIL" / "SKIP"

    def as_dict(self) -> dict:
        return {
            "button": self.button,
            "path": self.path,
            "commanded": self.commanded,
            "measured": self.measured,
            "elapsed_s": f"{self.elapsed_s:.2f}",
            "tolerance": self.tolerance,
            "encoder_advanced": (
                "" if self.encoder_advanced is None else str(self.encoder_advanced)
            ),
            "verdict": self.verdict,
        }


class TraceRecorder:
    """Accumulates ``Row`` entries, writes each one to a CSV file
    immediately (so a later hang/crash still leaves prior rows durably on
    disk), and prints the full table on ``close()``.

    ``csv_path`` is normally built from pytest's own ``tmp_path_factory``
    fixture (the "pytest tmp/output dir" the ticket asks for) so a human
    can find "what the buttons did" after a run without hunting through
    captured stdout.
    """

    _FIELDS = ["button", "path", "commanded", "measured", "elapsed_s",
               "tolerance", "encoder_advanced", "verdict"]

    def __init__(self, csv_path: Path) -> None:
        self.csv_path = csv_path
        self.rows: list[Row] = []
        self._fh = csv_path.open("w", newline="")
        self._writer = csv.DictWriter(self._fh, fieldnames=self._FIELDS)
        self._writer.writeheader()
        self._fh.flush()

    def record(self, row: Row) -> None:
        self.rows.append(row)
        self._writer.writerow(row.as_dict())
        self._fh.flush()
        print(
            f"[BUTTON] {row.button:<28} path={row.path:<10} "
            f"commanded={row.commanded:<16} measured={row.measured:<20} "
            f"elapsed={row.elapsed_s:6.2f}s tol={row.tolerance:<24} "
            f"enc_advanced={row.encoder_advanced} verdict={row.verdict}"
        )

    def print_table(self) -> None:
        print(f"\n=== GUI button-acceptance trace table ({len(self.rows)} rows) ===")
        print(f"CSV: {self.csv_path}")
        header = f"{'button':<28} {'path':<10} {'commanded':<16} {'measured':<20} {'elapsed':>8} {'verdict':<6}"
        print(header)
        print("-" * len(header))
        for row in self.rows:
            print(
                f"{row.button:<28} {row.path:<10} {row.commanded:<16} "
                f"{row.measured:<20} {row.elapsed_s:7.2f}s {row.verdict:<6}"
            )

    def close(self) -> None:
        self.print_table()
        self._fh.close()
