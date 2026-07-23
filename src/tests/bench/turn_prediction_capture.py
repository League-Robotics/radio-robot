#!/usr/bin/env python3
"""src/tests/bench/turn_prediction_capture.py -- turn-prediction campaign,
Phase A: capture a turn-heavy sim session (several 90-degree turns, both
directions, plus a Tour 1 run) to a tlm_log CSV -- the raw material
``docs/archive/turn_prediction.ipynb`` analyzes (ZOH prediction
quality vs anticipation lead, and measured stop-detection lag/overshoot per
turn). That notebook was relocated out of ``src/tests/notebooks/`` (118
ticket 004): it is a superseded historical record of the deleted
anticipation-lead campaign (see the notebook's own first cell), and this
project's grep gate for the deleted field's own wire-key name requires no
matching string survive anywhere under `src/` or `data/`, which its own
extensive historical narrative cannot honor while staying legible.

Reuses ``tlm_log.py``'s own ``CSV_FIELDNAMES``/``frame_to_row()`` row shape
(the SAME shape ``estimator_capture.py``, sprint 117 ticket 006, already
writes), but drives and drains the sim with **deterministic manual
stepping** (``SimLoop.step()``, ``connect(start_tick_thread=False)``)
instead of that module's own real-time background-thread pattern --
`test_tour_closure_gate.py`'s own established precedent for a Tour run
(its own module docstring: real-time threading is "the ONE deliberately
real-time-threaded... test", kept only as a TestGUI-fidelity smoke check;
every accuracy/reliability measurement in this tree runs deterministically
stepped instead, "removes real (non-deterministic) tick-thread scheduling
jitter as a variable"). Two concrete reasons this module needs that same
discipline, discovered writing it:

1. A real-time capture thread (``tlm_log.stream_to_csv()``) and a
   real-time driving loop each independently drain ``SimLoop``'s own
   single-consumer ``_tlm_queue`` (``read_pending_binary_tlm_frames()``) --
   racing two drains against the SAME queue silently starves whichever one
   loses the race, which is exactly how this module's own first draft's
   "read the last frame's own `now` before injecting each turn" bookkeeping
   came back mostly `None` (the capture thread had already drained
   everything). Single-threaded, step-then-drain-immediately removes the
   race by construction: there is only ever one consumer.
2. Real-time-threaded `run_tour()` against this sim inherits the SAME
   known live cycle-order-reorder-experiment flakiness
   `test_tour_closure_gate.py`'s own
   `test_two_compatible_distance_legs_carry_velocity_through_the_boundary_at_tour_level`
   documents at length (stale/alternating encoder reads under real-time
   scheduling) -- reproduced here as a Tour 1 FAULT at leg 3 on this
   module's own first draft. Deterministic stepping sidesteps it exactly
   as that file's own docstring says it does for its OTHER (non-real-time)
   tests.

Depends on the ``SimLoop.move()`` corr_id/move_id-aliasing fix (this same
campaign, ``sim_loop.py``) for the Tour 1 run's own per-leg completion
detection to be trustworthy -- see that method's own doc comment for the
failure mode this closed.

Usage::

    uv run python src/tests/bench/turn_prediction_capture.py
    uv run python src/tests/bench/turn_prediction_capture.py --csv out.csv --manifest out.json
"""
from __future__ import annotations

import argparse
import csv as csv_module
import json
import math
import pathlib
import sys
from dataclasses import dataclass

_BENCH_DIR = pathlib.Path(__file__).resolve().parent
if str(_BENCH_DIR) not in sys.path:
    sys.path.insert(0, str(_BENCH_DIR))

from tlm_log import CSV_FIELDNAMES, frame_to_row  # noqa: E402  (path must be set up first)

_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
DEFAULT_CSV = _REPO_ROOT / "src" / "tests" / "bench" / "out" / "turn_prediction_capture.csv"
DEFAULT_MANIFEST = _REPO_ROOT / "src" / "tests" / "bench" / "out" / "turn_prediction_manifest.json"

# The same fixture robot config estimator_capture.py/test_tour_closure_gate.py
# already use for a configured SimLoop (sprint 114's fail-closed
# configuration-completeness gate -- an unconfigured sim silently produces a
# flat, all-zero trace instead of raising, see estimator_capture.py's own
# _run_sim() comment).
DEFAULT_ROBOT_JSON = _REPO_ROOT / "data" / "robots" / "tovez_nocal.json"

# omega=2.0 rad/s matches PlannerParams.omega_max's own default (model.py) --
# the SAME turn rate TOUR_1's own "RT" legs and the campaign's own
# established diagnosis ("+20.6deg overshoot at omega=2rad/s") both use, so
# this capture's own turns are directly comparable to both.
_TURN_OMEGA = 2.0    # [rad/s]
_TURN_TARGET = math.pi / 2  # [rad] 90 degrees
_TURN_TIMEOUT_MS = 3000.0   # [ms] generous safety backstop -- never expected to fire
_TURN_SETTLE_MARGIN_S = 0.6  # [s] extra time past the turn's own nominal duration, so the
                             # physical coast/settle tail (the very thing being measured) is
                             # fully captured before the NEXT turn's Move preempts it.

_CYCLE_S = 0.05  # [s] SimLoop.step()'s own per-cycle virtual-time advance (sim_loop.py's own doc comment)
_PRIME_CYCLES = 5      # a few cycles before the first turn, so the estimator/odometry basis is warm
_TRAILING_CYCLES = 60  # ~3s of trailing capture after the LAST turn/leg, so its own settle telemetry lands

_MANIFEST_MOVE_ID_BASE = 8000  # distinct from _TOUR_MOVE_ID_BASE (planner.tour, 1<<20) and
                                # from SimLoop's own small auto-corr_id counter -- no collision.


@dataclass(frozen=True)
class TurnSpec:
    label: str
    omega: float  # [rad/s] signed


# 4 CCW + 4 CW, interleaved (never more than one same-direction run in a
# row) -- guards against a directional bias in the sample being mistaken
# for a measurement artifact of "always turning the same way".
DEFAULT_TURN_PATTERN: "tuple[TurnSpec, ...]" = tuple(
    TurnSpec(label=f"turn_{'ccw' if i % 2 == 0 else 'cw'}_{i // 2}",
            omega=_TURN_OMEGA if i % 2 == 0 else -_TURN_OMEGA)
    for i in range(8)
)


class _CsvSink:
    """Deterministic-stepping CSV writer.

    `step()` OWNS the sim's own tlm queue outright -- steps, drains, and
    writes every frame itself -- and is safe to use ONLY when nothing else
    is concurrently draining that same queue (the turn-driving/priming/
    trailing portions of a capture, where nothing else touches it).

    `write_frame()` is the alternate entry point for a portion where
    something ELSE already owns the drain -- `run_tour()`'s own
    `_wait_for_move_terminal()`/`_drain_and_poll()` machinery, during the
    Tour 1 portion of a capture. Discovered the hard way (this module's own
    early draft): `run_tour()` polls `transport.read_pending_binary_tlm_
    frames()` itself, once per poll iteration, to find each leg's own
    completion ack; if THIS sink's `step()` also drains that SAME queue
    (e.g. from inside the `sleep_fn` callback `run_tour()` invokes each
    poll iteration), the two race the single-consumer queue and `run_tour()`
    silently never sees the completion frame it is waiting for -- every leg
    times out. `write_frame()` lets a caller (`_run_tour1()`'s own
    `row_callback`) hand this sink the SAME frame `run_tour()` already
    drained, instead of this sink draining a second time."""

    def __init__(self, sim, csv_path: "str | pathlib.Path") -> None:
        self._sim = sim
        csv_path = pathlib.Path(csv_path)
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = csv_path.open("w", newline="")
        self._writer = csv_module.DictWriter(self._fh, fieldnames=CSV_FIELDNAMES)
        self._writer.writeheader()
        self.row_count = 0
        self.last_now_ms: "float | None" = None

    def write_frame(self, frame) -> None:
        self._writer.writerow(frame_to_row(frame))
        self.row_count += 1
        if frame.t is not None:
            self.last_now_ms = frame.t

    def step(self, cycles: int = 1) -> None:
        for _ in range(cycles):
            self._sim.step(1)
            self._sim._drain_tlm_into_queue()  # noqa: SLF001 -- mirrors test_tour_closure_gate.py's own precedent
            for frame in self._sim.read_pending_binary_tlm_frames():
                self.write_frame(frame)

    def close(self) -> None:
        self._fh.close()


def _make_sim(robot_json: "str | pathlib.Path"):
    from robot_radio.config.robot_config import load_robot_config
    from robot_radio.io.sim_loop import SimLoop

    robot_config = load_robot_config(robot_json)
    track_width = robot_config.trackwidth if robot_config.trackwidth is not None else 128.0
    sim = SimLoop(track_width=track_width)
    sim.connect(start_tick_thread=False)  # deterministic -- see this module's own header
    sim.configure_from_robot(robot_config)
    return sim


def _drive_turns(sim, sink: _CsvSink, pattern: "tuple[TurnSpec, ...]") -> "list[dict]":
    """Issues each `pattern` turn as a bounded ANGLE-stop Move, deterministically
    stepping the sim through its own nominal duration plus a settle margin
    before the next. Returns the manifest list `turn_events.find_turn_events()`
    consumes -- one dict per turn, in issue order, recording `issue_now_ms`
    (the robot clock's own last-known instant, from `sink`'s own last drain,
    immediately before injecting THIS turn's own Move -- exact under
    deterministic single-consumer stepping, unlike a real-time capture where
    it is only ever a lower bound)."""
    manifest: "list[dict]" = []
    move_id = _MANIFEST_MOVE_ID_BASE
    for spec in pattern:
        move_id += 1
        issue_now_ms = sink.last_now_ms
        sim.move(omega=spec.omega, stop_angle=_TURN_TARGET, timeout=_TURN_TIMEOUT_MS,
                 replace=True, id=move_id)
        manifest.append({
            "kind": "turn", "label": spec.label, "move_id": move_id,
            "omega": spec.omega, "target_rad": _TURN_TARGET, "issue_now_ms": issue_now_ms,
        })
        cycles = int(math.ceil((_TURN_TARGET / abs(spec.omega) + _TURN_SETTLE_MARGIN_S) / _CYCLE_S))
        sink.step(cycles)
    return manifest


def _run_tour1(sim, sink: _CsvSink) -> dict:
    from types import SimpleNamespace

    from robot_radio.planner.heading import HeadingCorrector
    from robot_radio.planner.model import PlannerParams
    from robot_radio.planner.tour import TOUR_1, parse_tour, run_tour

    class _SteppedClock:
        """Mirrors test_tour_closure_gate.py's own `_SteppedClock` -- a fake
        clock in lockstep with the stepper's own step count, reporting
        "seconds" (`_CYCLE_S` per step) even though no wall clock is ever
        read, so `run_tour()`'s timeout/poll-interval math (written in real
        seconds) keeps its existing meaning under deterministic stepping."""

        def __init__(self) -> None:
            self.now_s = 0.0

        def now(self) -> float:
            return self.now_s

    clock = _SteppedClock()

    def _stepper(_requested_interval: float) -> None:
        # Steps and feeds the sim's OWN internal tlm queue, but does NOT
        # drain `read_pending_binary_tlm_frames()` itself -- `run_tour()`'s
        # own polling (`_drain_and_poll()`) is the ONE consumer of that
        # queue for the whole duration of this call; see `_CsvSink`'s own
        # doc comment for why a second consumer here breaks completion
        # detection outright.
        sim.step(1)
        sim._drain_tlm_into_queue()  # noqa: SLF001 -- mirrors test_tour_closure_gate.py's own precedent
        clock.now_s += _CYCLE_S

    def _row_callback(tick_index, leg_index, leg, tick_result, frame) -> None:
        # `run_tour()`'s own drain already happened this poll iteration
        # (inside `_wait_for_move_terminal()`, before `_stepper` above even
        # runs) -- `frame` is that SAME already-drained frame, handed to
        # this sink instead of drained a second time.
        if frame is not None:
            sink.write_frame(frame)

    params = PlannerParams()
    heading = HeadingCorrector(
        params, robot_config=SimpleNamespace(geometry=SimpleNamespace(otos_untrusted=True)))
    legs = parse_tour(TOUR_1)
    result = run_tour(sim, params, heading, legs,
                      clock_fn=clock.now, sleep_fn=_stepper, poll_interval=_CYCLE_S,
                      row_callback=_row_callback)
    return {
        "kind": "tour_summary", "tour": "TOUR_1", "leg_count": len(legs),
        "completed": result.stopped_at is None,
        "stopped_at": result.stopped_at, "stopped_outcome": (
            result.stopped_outcome.value if result.stopped_outcome is not None else None),
    }


def capture_turn_prediction_session(
    csv_path: "str | pathlib.Path" = DEFAULT_CSV,
    manifest_path: "str | pathlib.Path" = DEFAULT_MANIFEST,
    *, robot_json: "str | pathlib.Path" = DEFAULT_ROBOT_JSON,
    pattern: "tuple[TurnSpec, ...]" = DEFAULT_TURN_PATTERN,
    run_tour1: bool = True,
) -> "tuple[int, list[dict]]":
    """Drives `pattern` (isolated 90-degree turns, both directions) then --
    unless `run_tour1` is False -- a full TOUR_1 run, against a freshly
    configured, DETERMINISTICALLY-STEPPED `SimLoop` (see this module's own
    header for why), writing every frame produced to `csv_path` as it goes.
    Writes `manifest_path` (JSON) alongside the CSV -- the per-turn
    bookkeeping `turn_events.find_turn_events()` needs to locate each
    turn's own window in the CSV; a trailing `{"kind": "tour_summary",
    ...}` entry records whether the Tour 1 run (if requested) completed
    cleanly, for the notebook's own sanity check.

    Returns `(row_count, manifest)`.
    """
    sim = _make_sim(robot_json)
    sink = _CsvSink(sim, csv_path)
    manifest: "list[dict]" = []
    try:
        sink.step(_PRIME_CYCLES)
        manifest.extend(_drive_turns(sim, sink, pattern))
        sim.stop()
        sink.step(10)

        if run_tour1:
            manifest.append(_run_tour1(sim, sink))

        sink.step(_TRAILING_CYCLES)
    finally:
        try:
            sim.stop()
            sink.step(2)
        except Exception:
            pass
        sink.close()
        sim.disconnect()

    manifest_path = pathlib.Path(manifest_path)
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.write_text(json.dumps(manifest, indent=2))

    return sink.row_count, manifest


def _args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--csv", default=str(DEFAULT_CSV), help=f"output CSV path (default {DEFAULT_CSV})")
    p.add_argument("--manifest", default=str(DEFAULT_MANIFEST),
                   help=f"output manifest JSON path (default {DEFAULT_MANIFEST})")
    p.add_argument("--robot-json", default=str(DEFAULT_ROBOT_JSON),
                   help=f"robot config to configure the SimLoop from (default {DEFAULT_ROBOT_JSON})")
    p.add_argument("--no-tour", action="store_true", help="skip the trailing Tour 1 run")
    return p.parse_args()


def main() -> int:
    args = _args()
    row_count, manifest = capture_turn_prediction_session(
        args.csv, args.manifest, robot_json=args.robot_json, run_tour1=not args.no_tour)
    turn_count = sum(1 for m in manifest if m.get("kind") == "turn")
    print(f"wrote {row_count} rows to {args.csv}")
    print(f"wrote {turn_count} turn manifest entries (+{len(manifest) - turn_count} other) "
          f"to {args.manifest}")
    tour_summary = next((m for m in manifest if m.get("kind") == "tour_summary"), None)
    if tour_summary is not None:
        print(f"TOUR_1: completed={tour_summary['completed']} "
              f"stopped_at={tour_summary['stopped_at']} outcome={tour_summary['stopped_outcome']}")
    return 0 if row_count > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
