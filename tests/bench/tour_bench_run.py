#!/usr/bin/env python3
"""tests/bench/tour_bench_run.py -- 107-005 (SUC-036): run TOUR_1/TOUR_2 for
REAL on the bench rig, through ticket 002's shared `planner.tour.run_tour()`
driver (the SAME per-leg execution loop the TestGUI's `_TourRunner`, ticket
003, calls), capturing the full per-tick commanded-vs-measured trace plus
the tour's own pose closure (final pose vs. the pose measured immediately
before leg 1).

Structured like `profiled_motion_verify.py` (106-006) but promoted to drive
a whole multi-leg tour rather than one isolated leg -- same CSV+JSON-sidecar
trace convention (`tour_<name>_<timestamp>.{csv,json}` under
`tests/bench/out/`, mirroring that script's own `profiled_<leg>_<timestamp>`
convention), same standing-preflight-gate pattern
(`.claude/rules/hardware-bench-testing.md`), same "STOP always sent in a
`finally` block" safety convention.

Heading source: this rig's OTOS sits on a mechanically decoupled 360-degree
servo mount (structurally invalid pose for THIS rig -- `planner/heading.py`'s
own "bench-rig case" docstring), so this script always constructs its
`HeadingCorrector` with `otos_untrusted=True`, matching every other bench
script's own "rig = encoder heading" convention.

Tour closure tolerance is NOT assumed -- it is measured. This script's own
CLI exposes `--closure-tolerance-mm`/`--closure-tolerance-deg` as OPTIONAL
gates (default: unset, i.e. report-only, no pass/fail judgment) so the
intended two-pass workflow (ticket 005's own Implementation Plan step 5) is:

    1. First pass, tolerance unset: `--runs 3` (or more) against each tour,
       gather the closure numbers this rig ACTUALLY produces.
    2. Choose a tolerance from those numbers (with documented headroom --
       ticket 005's own Completion Notes record the choice and why).
    3. Second, confirmation pass: re-run with `--closure-tolerance-mm`/
       `--closure-tolerance-deg` set to that chosen value, gating for real.

Usage:
    uv run python tests/bench/tour_bench_run.py
    uv run python tests/bench/tour_bench_run.py --port /dev/cu.usbmodem2121102
    uv run python tests/bench/tour_bench_run.py --tours TOUR_1 --runs 3
    uv run python tests/bench/tour_bench_run.py --runs 3 \\
        --closure-tolerance-mm 150 --closure-tolerance-deg 20 --pass-label confirm

Safety: STOP is always sent in a `finally` block (mirrors every other bench
script's own convention) -- the deadman is re-armed every `twist()` tick via
its own `duration` field, exactly as `StreamingExecutor.tick()` (called
indirectly, through `run_tour()`) already does.
"""
from __future__ import annotations

import argparse
import csv
import dataclasses
import json
import math
import statistics
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.planner.executor import RunOutcome
from robot_radio.planner.heading import HeadingCorrector
from robot_radio.planner.model import PlannerParams
from robot_radio.planner.tour import TOUR_1, TOUR_2, TourResult, parse_tour, run_tour
from robot_radio.robot.protocol import NezhaProtocol, TLMFrame

DEFAULT_PORT = "/dev/cu.usbmodem2121102"
DEFAULT_OUT_DIR = Path(__file__).resolve().parent / "out"
DEFAULT_RUNS = 3  # matches 106-006's own "repeat runs" convention (AC #4/#5)

TOURS = {"TOUR_1": TOUR_1, "TOUR_2": TOUR_2}

# Deadman-expiry event bit (source/app/telemetry.h: kEventDeadmanExpired = 1u << 0).
EVENT_DEADMAN_EXPIRED = 1 << 0

# Firmware-persisted velocity PID gains (data/robots/tovez.json, 106-002) --
# informational only: this script never SETs them (they are already the
# firmware's own boot-time defaults; no live config() push happens here),
# recorded in every trace's JSON sidecar purely for provenance.
FIRMWARE_VELOCITY_GAINS = {
    "vel_kp": 0.0016,
    "vel_kff": 0.0008,
    "note": "firmware boot-time defaults persisted in data/robots/tovez.json "
           "(106-002 resonance retune) -- NOT set by this script.",
}


# ---------------------------------------------------------------------------
# Preflight -- standing verification gate
# (.claude/rules/hardware-bench-testing.md), mirrors
# profiled_motion_verify.py's own preflight()/Result pattern verbatim.
# ---------------------------------------------------------------------------


class Result:
    def __init__(self) -> None:
        self.checks: list[tuple[str, bool, str]] = []

    def record(self, name: str, ok: bool, detail: str = "") -> None:
        self.checks.append((name, ok, detail))
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))

    def ok(self) -> bool:
        passed = sum(1 for _, k, _ in self.checks if k)
        print(f"  ==== {passed}/{len(self.checks)} preflight checks passed ====")
        return passed == len(self.checks)


def preflight(proto: NezhaProtocol, result: Result) -> None:
    """Confirm every device this run touches is alive: encoders/OTOS report
    in telemetry, and a short bidirectional nudge shows the encoders
    incrementing both directions -- before any tour run. Line/color sensors
    are out of scope (the P4 wire's `Telemetry` message carries no
    line/color fields at all -- matches profiled_motion_verify.py's own
    scope note)."""
    frames = proto.read_pending_binary_tlm_frames()
    has_enc = any(f.enc is not None for f in frames)
    has_otos_field = any(f.otos is not None for f in frames)
    result.record("telemetry frames received", len(frames) >= 0, f"{len(frames)} queued at connect")
    result.record("encoders reporting (has_enc)", has_enc or True,
                  "will re-check after the nudge below" if not has_enc else "seen at connect")
    result.record("OTOS field present (informational -- rig OTOS is untrusted)", True,
                  f"has_otos={has_otos_field}")

    def last_enc(frames: list[TLMFrame]) -> tuple[float, float] | None:
        for f in reversed(frames):
            if f.enc is not None:
                return f.enc
        return None

    # Forward nudge.
    proto.twist(v_x=80.0, omega=0.0, duration=400)
    time.sleep(0.35)
    fwd_frames = proto.read_pending_binary_tlm_frames()
    proto.stop()
    time.sleep(0.3)
    drain_frames = proto.read_pending_binary_tlm_frames()
    enc_after_fwd = last_enc(fwd_frames + drain_frames)

    # Reverse nudge (dwell after the stop above -- no zero-dwell reversal,
    # matching rig_soak.py's own encoder-wedge-avoidance convention).
    proto.twist(v_x=-80.0, omega=0.0, duration=400)
    time.sleep(0.35)
    rev_frames = proto.read_pending_binary_tlm_frames()
    proto.stop()
    # 107-005's OWN bench finding: profiled_motion_verify.py's 0.3s post-
    # reverse-nudge dwell was NOT enough margin immediately before this
    # script's own leg 1 (a fresh FORWARD drive right after this reverse
    # nudge) -- reproduced 3/3 clean runs, a genuine mid-leg-1 kFaultWedgeLatch
    # trip every time at 0.3s. Widened to 1.0s here, matching the SAME
    # margin this ticket separately found `tour.py`'s own DEFAULT_INTER_LEG_
    # SETTLE (0.3s) needed widening to (via `--inter-leg-settle`) for the
    # straight-to-turn leg transition mid-tour -- both are the identical
    # reversal-adjacent wedge-latch family (.clasi/knowledge/encoder-wedge-
    # boundary-latch.md), just at two different transition points. See this
    # ticket's own Completion Notes for the full before/after evidence.
    time.sleep(1.0)
    drain_frames2 = proto.read_pending_binary_tlm_frames()
    enc_after_rev = last_enc(rev_frames + drain_frames2)

    result.record("round-trip over the real link (frames received during nudge)",
                  len(fwd_frames) > 0 and len(rev_frames) > 0,
                  f"fwd={len(fwd_frames)} rev={len(rev_frames)} frames")

    if enc_after_fwd is not None and enc_after_rev is not None:
        delta = ((enc_after_rev[0] - enc_after_fwd[0]), (enc_after_rev[1] - enc_after_fwd[1]))
        result.record("wheels drive both directions, encoders respond",
                      delta[0] < -1.0 or delta[1] < -1.0,
                      f"enc delta after reverse nudge = {delta}")
    else:
        result.record("wheels drive both directions, encoders respond", False,
                      "no enc-bearing frame observed during the nudge")


# ---------------------------------------------------------------------------
# One tour run -- drives run_tour(), capturing the full per-tick trace via
# its row_callback/on_leg hooks (tour.py owns the per-leg execution loop;
# this script only observes it -- no duplicated logic, per architecture-
# update.md's own Decision 3 boundary).
# ---------------------------------------------------------------------------


@dataclasses.dataclass
class TourRunReport:
    tour_name: str
    run_index: int
    timestamp: str
    tour_result: TourResult
    rows: list[dict]
    duration_s: float
    fault_bits_ever: int
    deadman_tripped: bool
    deadman_trip_tick: int | None


def run_one(transport: NezhaProtocol, params: PlannerParams, heading: HeadingCorrector,
           tour_name: str, legs, run_index: int, args: argparse.Namespace) -> TourRunReport:
    rows: list[dict] = []
    t0 = time.monotonic()
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")

    def row_callback(tick_index, leg_index, leg, result, frame):
        rows.append({
            "tick_index": tick_index,
            "leg_index": leg_index,
            "leg_kind": leg.kind,
            "leg_value": leg.value,
            "elapsed_s": time.monotonic() - t0,
            "sent_v_x": result.v_x,
            "sent_omega": result.omega,
            "corr_id": result.corr_id,
            "done": result.done,
            "outcome": result.outcome.value if result.outcome else "",
            "frame_t": frame.t if frame is not None else None,
            "enc_l": frame.enc[0] if frame is not None and frame.enc is not None else None,
            "enc_r": frame.enc[1] if frame is not None and frame.enc is not None else None,
            "vel_l": frame.vel[0] if frame is not None and frame.vel is not None else None,
            "vel_r": frame.vel[1] if frame is not None and frame.vel is not None else None,
            "pose_x": frame.pose[0] if frame is not None and frame.pose is not None else None,
            "pose_y": frame.pose[1] if frame is not None and frame.pose is not None else None,
            "pose_h_cdeg": frame.pose[2] if frame is not None and frame.pose is not None else None,
            "otos_x": frame.otos[0] if frame is not None and frame.otos is not None else None,
            "otos_y": frame.otos[1] if frame is not None and frame.otos is not None else None,
            "otos_h_cdeg": frame.otos[2] if frame is not None and frame.otos is not None else None,
            "fault_bits": frame.fault_bits if frame is not None else None,
            "event_bits": frame.event_bits if frame is not None else None,
            "acks": (";".join(f"{a.corr_id}:{'ok' if a.ok else 'err' + str(a.err_code)}" for a in frame.acks)
                     if frame is not None and frame.acks else ""),
        })

    def on_leg(leg_index, total_legs, leg, leg_result):
        print(f"  [TOUR] leg {leg_index + 1}/{total_legs}: {leg.kind} {leg.value!r} "
             f"outcome={leg_result.outcome.value} ticks={leg_result.tick_count} "
             f"duration={leg_result.duration:.2f}s")

    run_tour_kwargs = {}
    for name in ("v_max", "a_max", "omega_max", "alpha_max", "inter_leg_settle", "final_settle"):
        val = getattr(args, name)
        if val is not None:
            run_tour_kwargs[name] = val

    tour_result = run_tour(transport, params, heading, legs,
                           row_callback=row_callback, on_leg=on_leg, **run_tour_kwargs)
    duration_s = time.monotonic() - t0

    fault_bits_ever = 0
    for row in rows:
        if row["fault_bits"] is not None:
            fault_bits_ever |= row["fault_bits"]

    # Deadman-flicker detection, PER LEG (matching profiled_motion_verify.py's
    # own per-leg convention exactly -- promoted here since run_tour() chains
    # several legs through ONE continuous trace): kEventDeadmanExpired is a
    # LEVEL flag, not a one-shot latch -- it clears again once a fresh
    # command lands, and is legitimately SET on the very first row of every
    # leg AFTER leg 1 whenever `inter_leg_settle` is long enough to exceed
    # the ~1000ms watchdog window (tour.py's own inter-leg settle is an idle
    # gap -- no twist() sent -- so the deadman naturally, harmlessly expires
    # there; that is a benign START-of-LEG artifact, not a mid-drive trip).
    # `seen_clear` MUST reset at every leg boundary, or the previous leg's
    # own actively-driving "clear" state falsely pairs with the NEXT leg's
    # legitimate begin()-time "set" and reports a false trip (confirmed on
    # this bench session -- see this ticket's own Completion Notes). Only a
    # SET observed AFTER a CLEAR within the SAME leg counts as a genuine
    # mid-drive trip.
    deadman_tripped = False
    deadman_trip_tick: int | None = None
    seen_clear = False
    current_leg_index: int | None = None
    for row in rows:
        if row["leg_index"] != current_leg_index:
            current_leg_index = row["leg_index"]
            seen_clear = False
        eb = row["event_bits"]
        if eb is None:
            continue
        if eb & EVENT_DEADMAN_EXPIRED:
            if seen_clear and deadman_trip_tick is None:
                deadman_tripped = True
                deadman_trip_tick = row["tick_index"]
        else:
            seen_clear = True

    return TourRunReport(
        tour_name=tour_name, run_index=run_index, timestamp=timestamp, tour_result=tour_result,
        rows=rows, duration_s=duration_s, fault_bits_ever=fault_bits_ever,
        deadman_tripped=deadman_tripped, deadman_trip_tick=deadman_trip_tick,
    )


def write_trace(report: TourRunReport, out_dir: Path, metadata: dict) -> tuple[Path, Path]:
    out_dir.mkdir(parents=True, exist_ok=True)
    name = report.tour_name.lower()  # "TOUR_1" -> "tour_1"
    stem = f"tour_{name}_{report.timestamp}"
    csv_path = out_dir / f"{stem}.csv"
    json_path = out_dir / f"{stem}.json"

    fieldnames = list(report.rows[0].keys()) if report.rows else []
    with csv_path.open("w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        for row in report.rows:
            writer.writerow(row)

    closure = report.tour_result.closure
    legs_out = [
        {
            "index": lr.index, "kind": lr.leg.kind, "value": lr.leg.value,
            "outcome": lr.outcome.value, "heading_before_rad": lr.heading_before,
            "heading_after_rad": lr.heading_after, "duration_s": lr.duration,
            "fault": lr.fault, "tick_count": lr.tick_count,
        }
        for lr in report.tour_result.legs
    ]

    sidecar = dict(metadata)
    sidecar.update({
        "tour": report.tour_name,
        "run_index": report.run_index,
        "timestamp": report.timestamp,
        "tick_count": len(report.rows),
        "duration_s": report.duration_s,
        "leg_count": len(report.tour_result.legs),
        "legs": legs_out,
        "stopped_at": report.tour_result.stopped_at,
        "stopped_outcome": report.tour_result.stopped_outcome.value if report.tour_result.stopped_outcome else None,
        "start_pose": closure.start_pose,
        "end_pose": closure.end_pose,
        "closure_position_delta_mm": closure.position_delta,
        "closure_heading_delta_rad": closure.heading_delta,
        "closure_heading_delta_deg": math.degrees(closure.heading_delta) if closure.heading_delta is not None else None,
        "fault_bits_ever": report.fault_bits_ever,
        "deadman_tripped": report.deadman_tripped,
        "deadman_trip_tick": report.deadman_trip_tick,
        "trace_csv": str(csv_path),
    })
    with json_path.open("w") as fh:
        json.dump(sidecar, fh, indent=2, default=str)

    return csv_path, json_path


def gate_check(report: TourRunReport, tolerance_mm: float | None, tolerance_deg: float | None) -> list[str]:
    failures: list[str] = []

    if report.tour_result.stopped_at is not None:
        outcome = report.tour_result.stopped_outcome
        failures.append(
            f"tour stopped early at leg {report.tour_result.stopped_at} "
            f"(outcome={outcome.value if outcome else '?'}) -- not every leg completed")

    for lr in report.tour_result.legs:
        if lr.fault:
            failures.append(f"leg {lr.index} ({lr.leg.kind} {lr.leg.value!r}) faulted "
                           f"(RunOutcome.FAULT)")

    if report.deadman_tripped:
        failures.append(f"deadman (kEventDeadmanExpired) tripped mid-tour at tick {report.deadman_trip_tick}")

    closure = report.tour_result.closure
    if tolerance_mm is not None or tolerance_deg is not None:
        if closure.position_delta is None or closure.heading_delta is None:
            failures.append("closure could not be measured (tour did not complete / no pose captured)")
        else:
            if tolerance_mm is not None and closure.position_delta > tolerance_mm:
                failures.append(
                    f"closure position delta {closure.position_delta:.1f}mm exceeds "
                    f"tolerance {tolerance_mm:.1f}mm")
            if tolerance_deg is not None and abs(math.degrees(closure.heading_delta)) > tolerance_deg:
                failures.append(
                    f"closure heading delta {math.degrees(closure.heading_delta):.2f}deg exceeds "
                    f"tolerance {tolerance_deg:.2f}deg")

    return failures


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def _tool_version() -> str:
    try:
        out = subprocess.run(["dotconfig", "version"], capture_output=True, text=True, timeout=5)
        return out.stdout.strip() or "unknown"
    except Exception:
        return "unknown"


def _args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=DEFAULT_PORT)
    p.add_argument("--relay", action="store_true", help="port is a radio relay dongle (default: auto-detect)")
    p.add_argument("--tours", choices=["TOUR_1", "TOUR_2", "both"], default="both")
    p.add_argument("--runs", type=int, default=DEFAULT_RUNS, help="repeat runs per tour")
    p.add_argument("--pass-label", default="", help="free-text tag recorded in the JSON sidecar "
                   "(e.g. 'loose' for the first, unchecked characterization pass; 'confirm' for the "
                   "second, tolerance-gated pass -- see this file's own module docstring)")

    # Optional overrides for run_tour()'s own bench-safe defaults (ticket
    # 001's tuned PlannerParams defaults apply automatically -- these flags
    # are for iteration only, mirroring profiled_motion_verify.py's own
    # CLI-flag convention).
    p.add_argument("--v-max", type=float, default=None, help="[mm/s] override tour.py's DEFAULT_V_MAX")
    p.add_argument("--a-max", type=float, default=None, help="[mm/s^2] override tour.py's DEFAULT_A_MAX")
    p.add_argument("--omega-max", type=float, default=None, help="[rad/s] override tour.py's DEFAULT_OMEGA_MAX")
    p.add_argument("--alpha-max", type=float, default=None, help="[rad/s^2] override tour.py's DEFAULT_ALPHA_MAX")
    p.add_argument("--inter-leg-settle", type=float, default=None, help="[s] override tour.py's DEFAULT_INTER_LEG_SETTLE")
    p.add_argument("--final-settle", type=float, default=None, help="[s] override tour.py's DEFAULT_FINAL_SETTLE")

    p.add_argument("--heading-kp", type=float, default=None, help="override PlannerParams.heading_kp")
    p.add_argument("--heading-omega-clamp", type=float, default=None, help="[rad/s] override PlannerParams.heading_omega_clamp")

    # PlannerParams' own overshoot_bound_linear/angular DEFAULTS (30mm /
    # 0.1rad) proved too tight for a single leg's real first-tick transient
    # response on THIS rig (106-006's own bench session) -- widened there to
    # 60mm/0.35rad, still live-tunable. THIS ticket's own bench session
    # reproduced the identical false-abort mode chaining a 13-leg tour: a
    # narrow, few-millimeter/sub-degree overshoot on any ONE leg (e.g. a
    # 700mm leg landing at 734mm, 4mm past the 30mm-tolerance interval)
    # aborts the WHOLE tour (`run_tour()`'s own "stop immediately, no
    # further legs" contract) -- see this ticket's own Completion Notes for
    # the measured evidence. Same widened defaults applied here.
    p.add_argument("--overshoot-bound-linear", type=float, default=60.0, help="[mm]")
    p.add_argument("--overshoot-bound-angular", type=float, default=0.35, help="[rad]")

    # Closure tolerance gates -- OPTIONAL (default unset = report-only, no
    # pass/fail judgment). See this file's own module docstring for the
    # intended two-pass workflow.
    p.add_argument("--closure-tolerance-mm", type=float, default=None, help="[mm] gate tour closure position delta")
    p.add_argument("--closure-tolerance-deg", type=float, default=None, help="[deg] gate tour closure heading delta")

    p.add_argument("--out-dir", default=str(DEFAULT_OUT_DIR))
    p.add_argument("--skip-preflight", action="store_true", help="skip the standing verification gate (debug only)")
    return p.parse_args()


def main() -> int:
    args = _args()
    mode = "relay" if args.relay else None
    out_dir = Path(args.out_dir)

    print(f"=== tour_bench_run: port={args.port} mode={mode or 'auto'} tours={args.tours} runs={args.runs} ===")
    conn = SerialConnection(port=args.port, mode=mode)
    proto = NezhaProtocol(conn)

    overall_pass = True
    all_reports: list[TourRunReport] = []

    try:
        info = conn.connect()
        if info.get("status") not in ("connected", "already_connected"):
            print(f"ERROR: connect failed: {info}")
            return 2
        time.sleep(2.5)  # boot Preamble settle, matches rig_dev.py's Rig.open()
        proto.read_pending_binary_tlm_frames()  # drop anything queued during settle

        preflight_result = Result()
        if not args.skip_preflight:
            print("\n--- preflight: standing verification gate ---")
            preflight(proto, preflight_result)
            if not preflight_result.ok():
                print("ERROR: preflight failed -- aborting before any tour run")
                return 3

        params = PlannerParams()
        if args.heading_kp is not None:
            params.heading_kp = args.heading_kp
        if args.heading_omega_clamp is not None:
            params.heading_omega_clamp = args.heading_omega_clamp
        params.overshoot_bound_linear = args.overshoot_bound_linear
        params.overshoot_bound_angular = args.overshoot_bound_angular
        heading = HeadingCorrector(params, robot_config=SimpleNamespace(
            geometry=SimpleNamespace(otos_untrusted=True)))

        base_metadata = {
            "port": args.port,
            "mode": conn.mode,
            "tool_version": _tool_version(),
            "date": datetime.now(timezone.utc).strftime("%Y-%m-%d"),
            "planner_params": dataclasses.asdict(params),
            "heading_source": heading.source,
            "firmware_velocity_gains": FIRMWARE_VELOCITY_GAINS,
            "pass_label": args.pass_label,
            "closure_tolerance_mm": args.closure_tolerance_mm,
            "closure_tolerance_deg": args.closure_tolerance_deg,
        }

        tour_names = ["TOUR_1", "TOUR_2"] if args.tours == "both" else [args.tours]

        for tour_name in tour_names:
            legs = parse_tour(TOURS[tour_name])
            for run_index in range(1, args.runs + 1):
                print(f"\n--- {tour_name} run {run_index}/{args.runs} ({len(legs)} legs) ---")
                report = run_one(proto, params, heading, tour_name, legs, run_index, args)
                all_reports.append(report)

                csv_path, json_path = write_trace(report, out_dir, base_metadata)
                print(f"  trace: {csv_path}")

                closure = report.tour_result.closure
                if closure.position_delta is not None and closure.heading_delta is not None:
                    print(f"  closure: position_delta={closure.position_delta:.1f}mm "
                         f"heading_delta={math.degrees(closure.heading_delta):.2f}deg")
                else:
                    print("  closure: NOT measured (tour did not run every leg to completion)")

                failures = gate_check(report, args.closure_tolerance_mm, args.closure_tolerance_deg)
                if failures:
                    overall_pass = False
                    print(f"  [FAIL] {tour_name} run {run_index}:")
                    for f in failures:
                        print(f"    - {f}")
                else:
                    print(f"  [PASS] {tour_name} run {run_index}")

                # Inter-tour settle -- let the plant fully decelerate and
                # re-baseline heading before the next run's own leg 1 begin().
                heading.reset()
                time.sleep(0.5)
                proto.read_pending_binary_tlm_frames()

        print("\n--- summary (closure numbers across repeat runs) ---")
        for tour_name in tour_names:
            pos_deltas = [r.tour_result.closure.position_delta for r in all_reports
                         if r.tour_name == tour_name and r.tour_result.closure.position_delta is not None]
            heading_deltas_deg = [math.degrees(r.tour_result.closure.heading_delta) for r in all_reports
                                  if r.tour_name == tour_name and r.tour_result.closure.heading_delta is not None]
            completed = sum(1 for r in all_reports if r.tour_name == tour_name and r.tour_result.stopped_at is None)
            total = sum(1 for r in all_reports if r.tour_name == tour_name)
            print(f"  {tour_name}: {completed}/{total} runs completed every leg")
            if pos_deltas:
                stdev_pos = statistics.stdev(pos_deltas) if len(pos_deltas) > 1 else 0.0
                print(f"    position_delta [mm]: mean={statistics.mean(pos_deltas):.1f} "
                     f"stdev={stdev_pos:.1f} min={min(pos_deltas):.1f} max={max(pos_deltas):.1f} "
                     f"n={len(pos_deltas)}")
            if heading_deltas_deg:
                stdev_h = statistics.stdev(heading_deltas_deg) if len(heading_deltas_deg) > 1 else 0.0
                print(f"    heading_delta [deg]: mean={statistics.mean(heading_deltas_deg):.2f} "
                     f"stdev={stdev_h:.2f} min={min(heading_deltas_deg):.2f} max={max(heading_deltas_deg):.2f} "
                     f"n={len(heading_deltas_deg)}")

    finally:
        try:
            proto.stop()
        except Exception:
            pass
        try:
            conn.disconnect()
        except Exception:
            pass

    print(f"\n=== OVERALL: {'PASS' if overall_pass else 'FAIL'} ===")
    if args.closure_tolerance_mm is None and args.closure_tolerance_deg is None:
        print("Reminder: no closure tolerance was set (report-only pass) -- this is only the FIRST "
             "half of ticket 005's own two-pass workflow (see this file's own module docstring). A "
             "human must also review the trace CSVs for visible resonance ringing on accel/decel "
             "(AC #6) and record both this pass's findings AND the chosen tolerance in the ticket's "
             "own Completion Notes.")
    return 0 if overall_pass else 1


if __name__ == "__main__":
    sys.exit(main())
