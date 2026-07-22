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

Turn-prediction campaign (2026-07-22, ``App::MoveQueue``'s new stop-
condition anticipation lead, ``move_queue.h``): ``MANAGED_ANGLE_ABS_MARGIN_DEG``/
``MANAGED_DIST_ABS_MARGIN_MM`` below are DELIBERATELY left UNCHANGED by
this campaign, even though ``test_tour_closure_gate.py``'s own sweep
(against the SAME managed Move-queue path, with the fix pushed live via
``EstimatorConfigPatch``) shows the fixed lag above shrinking from
~13-23deg to ~4-7deg at ``stop_lead_ms=90``. The GUI's own connect-time
calibration push (``__main__.py``'s ``_push_robot_calibration()``) does
NOT yet source ``estimator.stop_lead_ms`` from the robot JSON --
``robot_radio.config.robot_config.RobotConfig`` has no ``estimator`` field
at all today (nothing host-side reads that JSON section; only
``gen_boot_config.py`` does, at ARM build time) -- so a live TestGUI Sim
session (this suite's own connect path) still runs with
``App::MoveQueue::stopLead_ == 0`` (anticipation OFF), identically to
before this campaign. Tightening these two constants without ALSO wiring
that push would silently claim an accuracy improvement this suite does not
actually exercise. Wiring the GUI's own live push is tracked as a
follow-up (see ``clasi/issues/`` for this campaign's own tracking entry) --
out of this campaign's own scope (real hardware needs no such wiring at
all: it gets the fix from boot config, unconditionally, on every reboot).
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
