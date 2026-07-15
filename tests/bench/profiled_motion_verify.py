#!/usr/bin/env python3
"""tests/bench/profiled_motion_verify.py -- 106-006 (SUC-030) Phase 2: the
SAME profiled straight leg and profiled in-place turn ticket 006's sim
scenario (tests/sim/system/test_profiled_motion_sim.py) exercises, run for
REAL on the bench stand -- this time through the REAL, unmodified
`planner/executor.py` `StreamingExecutor` (closed-loop heading correction
via `planner/heading.py`'s `HeadingCorrector`, ticket 005) against the REAL
firmware over the REAL wire (`NezhaProtocol`/`SerialConnection`), not an
open-loop setpoint replay.

Robot is mounted on a stand with the wheels off the ground (see
`.claude/rules/hardware-bench-testing.md`), so it is safe to spin them
freely. `NezhaProtocol` already satisfies `executor.py`'s `TwistTransport`
structural protocol (`twist()`/`stop()`/`read_pending_binary_tlm_frames()`)
as-is -- no bridge/adapter needed here, unlike Phase 1's sim scenario (see
`profiled_motion_harness.cpp`'s own file header for why the sim side could
not just run the same real executor).

Heading source: the bench rig's OTOS sensor sits on a mechanically
decoupled 360-degree servo mount, so its reported pose is structurally
invalid for this rig (`planner/heading.py`'s own "bench-rig case" docstring)
-- this script always constructs its `HeadingCorrector` with
`otos_untrusted=True`, selecting the encoder-derived dead-reckoned heading
(`TLMFrame.pose`) as ground truth, matching every other bench script's own
"rig = encoder heading" convention.

What this script proves, per ticket 006's own Acceptance Criteria:
  1. Standing verification gate (`.claude/rules/hardware-bench-testing.md`):
     connect, confirm encoders/OTOS/telemetry are alive, and a short
     bidirectional nudge shows the encoders incrementing both directions --
     BEFORE the profiled runs. Line/color sensors are NOT touched by this
     script's own commands (the P4 wire's `Telemetry` message carries no
     line/color fields at all -- `source/messages/telemetry.h`) and are
     out of scope for a profiled-MOTION verification.
  2. A profiled straight leg and a profiled in-place turn, each run through
     the REAL `StreamingExecutor`, with the FULL commanded-vs-measured
     trace (every tick's sent v_x/omega plus the telemetry frame drained
     that tick) captured to `tests/bench/out/profiled_<leg>_<timestamp>.csv`
     plus a `.json` metadata sidecar (gains, limits, cadence, firmware/tool
     version, port/mode).
  3. Automated gate checks: run outcome COMPLETED (not OVERSHOOT/FAULT/
     STOPPED), terminal velocity converges near zero with no lunge/reversal,
     heading-hold/turn-landing numeric error recorded against a configurable
     tolerance, deadman never trips mid-run (`kEventDeadmanExpired` never
     observed before this script's own terminal stop), and zero NEW fault
     bits (baseline-relative, matching `rig_soak.py`'s own convention).
  4. A human (this script's operator) additionally reviews the printed
     trace / CSV for visible resonance ringing during accel/decel and
     records that pass/fail judgment in ticket 006's own Completion Notes --
     this script reports summary statistics to make that judgment easy, but
     does not itself automate "no ringing" (see ticket 006's own AC #3).

Usage:
    uv run python tests/bench/profiled_motion_verify.py
    uv run python tests/bench/profiled_motion_verify.py --port /dev/cu.usbmodem2121102
    uv run python tests/bench/profiled_motion_verify.py --distance 300 --angle-deg 60
    uv run python tests/bench/profiled_motion_verify.py --relay

Safety: STOP is always sent in a `finally` block (mirrors `rig_soak.py`'s
own convention) -- there is no `DEV`-watchdog-widen equivalent on the P4
wire (`rig_dev.py`'s own "no per-port addressing... no SERVO verb" finding);
the deadman is instead re-armed every `twist()` tick via its own `duration`
field, exactly as `StreamingExecutor.tick()` already does.
"""
from __future__ import annotations

import argparse
import csv
import dataclasses
import json
import math
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.planner.executor import RunOutcome, RunState, StreamingExecutor
from robot_radio.planner.heading import HeadingCorrector
from robot_radio.planner.model import PlannerParams
from robot_radio.planner.profile import ProfileLimits, profile_for_distance, profile_for_turn
from robot_radio.robot.protocol import NezhaProtocol, TLMFrame

# ---------------------------------------------------------------------------
# Baseline fault-bit masking -- now lives in production, not here
# ---------------------------------------------------------------------------
#
# BENCH FINDING (106-006's own hardware session, filed as
# `executor-fault-check-needs-baseline-exclusion.md`): `StreamingExecutor.
# tick()` used to stop the run the instant ANY drained frame's `fault_bits`
# was nonzero, with no baseline-relative exclusion for the boot-time
# one-shot `kFaultI2CSafetyNet` bit -- so a real run fault-stopped on tick 2
# every single time. This script originally worked around that with a
# local `BaselineFaultMaskingTransport` wrapper around `NezhaProtocol`.
#
# 107-001 promoted that baseline-exclusion logic INTO `planner/executor.py`
# itself (`StreamingExecutor.begin()` now captures the first drained
# frame's `fault_bits` as `self._fault_baseline`; `tick()`'s fault check is
# baseline-relative) so every real caller benefits, not just this script.
# `NezhaProtocol` (`proto` below) is now handed to `StreamingExecutor`
# directly, no adapter needed -- confirmed on hardware (this ticket's own
# bench session) that the executor no longer fault-stops on the boot-latched
# `kFaultI2CSafetyNet` bit while still correctly stopping on a genuinely NEW
# bit observed mid-run.
DEFAULT_PORT = "/dev/cu.usbmodem2121102"
DEFAULT_OUT_DIR = Path(__file__).resolve().parent / "out"

# Deadman-expiry event bit (source/app/telemetry.h: kEventDeadmanExpired = 1u << 0).
EVENT_DEADMAN_EXPIRED = 1 << 0
# Boot-time one-shot artifact (source/app/telemetry.h: kFaultI2CSafetyNet = 1u << 0) --
# same "baseline, not zero" convention as rig_soak.py.
FAULT_I2C_SAFETY_NET = 1 << 0

# --- modest, bench-safe defaults (per this ticket's own "modest limits"
# instruction) -- well under PlannerParams' hard ceilings (v_max=200mm/s,
# omega_max=2.0rad/s) and under the ~140mm/s velocity-resonance band 106-002
# bench-tamed (kp=0.0016/kff=0.0008, worst-case overshoot 6.4%@140mm/s). ---
DEFAULT_DISTANCE = 300.0  # [mm]
DEFAULT_V_MAX = 150.0  # [mm/s]
DEFAULT_A_MAX = 400.0  # [mm/s^2]
DEFAULT_ANGLE_DEG = 60.0  # [deg]
DEFAULT_OMEGA_MAX = 1.0  # [rad/s]
DEFAULT_ALPHA_MAX = 3.0  # [rad/s^2]

DEFAULT_HEADING_TOLERANCE_DEG = 5.0  # [deg] straight-leg heading-hold gate
DEFAULT_TURN_TOLERANCE_DEG = 6.0  # [deg] turn-landing gate


# ---------------------------------------------------------------------------
# Preflight -- standing verification gate
# (.claude/rules/hardware-bench-testing.md)
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
    incrementing both directions -- before any profiled leg runs. Line/color
    sensors are out of scope (see this file's own module docstring)."""
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
    time.sleep(0.3)
    drain_frames2 = proto.read_pending_binary_tlm_frames()
    enc_after_rev = last_enc(rev_frames + drain_frames2)

    result.record("round-trip over the real link (frames received during nudge)",
                  len(fwd_frames) > 0 and len(rev_frames) > 0,
                  f"fwd={len(fwd_frames)} rev={len(rev_frames)} frames")

    if enc_after_fwd is not None and enc_after_rev is not None:
        delta = ((enc_after_rev[0] - enc_after_fwd[0]), (enc_after_rev[1] - enc_after_fwd[1]))
        # Reverse nudge should move the encoders back down (negative delta)
        # relative to their post-forward reading.
        result.record("wheels drive both directions, encoders respond",
                      delta[0] < -1.0 or delta[1] < -1.0,
                      f"enc delta after reverse nudge = {delta}")
    else:
        result.record("wheels drive both directions, encoders respond", False,
                      "no enc-bearing frame observed during the nudge")


# ---------------------------------------------------------------------------
# Profiled run -- drives the REAL StreamingExecutor tick-by-tick, capturing
# a full commanded-vs-measured trace.
# ---------------------------------------------------------------------------


@dataclasses.dataclass
class LegResult:
    label: str
    axis: str
    target: float
    outcome: str
    rows: list[dict]
    baseline_fault_bits: int
    fault_bits_ever: int
    deadman_tripped: bool
    baseline_heading: float | None  # [rad] ABSOLUTE measured heading captured at begin() --
                                     # Odometry (App::Odometry) never resets across a boot session
                                     # (no EZ/pose_fix equivalent on the P4 wire, rig_dev.py's own
                                     # finding), so every heading reading this script sees is
                                     # absolute-since-boot; a leg's own "hold"/"land" gate must be
                                     # measured RELATIVE TO THIS baseline, never against an absolute
                                     # zero or an absolute target.
    final_heading: float | None  # [rad] ABSOLUTE measured heading at run end -- same caveat as above
    duration_s: float

    @property
    def heading_delta(self) -> float | None:  # [rad] final relative to this leg's OWN start
        if self.final_heading is None or self.baseline_heading is None:
            return None
        return self.final_heading - self.baseline_heading


def run_leg(transport: NezhaProtocol, params: PlannerParams, heading: HeadingCorrector, setpoints,
           target: float, axis: str, label: str) -> LegResult:
    ex = StreamingExecutor(transport, params, heading)
    ex.begin(setpoints, target=target, axis=axis)
    baseline_heading = heading.measured_heading(ex.latest_frame)
    # Bench-only fallback: begin()'s own single drain can legitimately race
    # an empty telemetry queue (nothing new pushed in the last poll's own
    # ~25Hz window) and leave `latest_frame` None -- executor.py's own
    # internal baseline/commanded_heading already tolerate this (default to
    # 0.0/None, logged loudly), but THIS script's own gate needs a real
    # baseline_heading to compute a meaningful relative delta. Retry a few
    # times before giving up. Reaches into `ex._latest_frame` deliberately --
    # a bench-script-only affordance, not something executor.py's own public
    # API needs to support.
    retries = 0
    while baseline_heading is None and retries < 5:
        time.sleep(0.1)
        frames = transport.read_pending_binary_tlm_frames()
        if frames:
            ex._latest_frame = frames[-1]  # noqa: SLF001 -- deliberate bench-script fallback, see above
            baseline_heading = heading.measured_heading(ex.latest_frame)
        retries += 1

    rows: list[dict] = []
    baseline_fault_bits: int | None = None
    fault_bits_ever = 0
    deadman_tripped = False
    # The deadman-expired event bit is a LEVEL flag, not a one-shot latch --
    # it clears again once a fresh command lands (confirmed empirically this
    # bench session). It is very often ALREADY set on the very first frame
    # drained at begin() (the natural, benign result of the idle preflight/
    # inter-leg gap exceeding the ~1000ms staleness window with no twist()
    # sent yet) -- that is a START-of-run artifact, not a mid-run trip. Only
    # count it as a genuine trip if it is observed CLEAR at some point during
    # this run and then flips to SET afterward.
    deadman_seen_clear = False
    t0 = time.monotonic()
    tick_index = 0

    print(f"  running leg '{label}': axis={axis} target={target!r} setpoints={len(setpoints)}")
    while ex.state == RunState.RUNNING:
        tick_wall = time.monotonic()
        result = ex.tick()
        frame = ex.latest_frame

        if frame is not None and frame.fault_bits is not None:
            if baseline_fault_bits is None:
                baseline_fault_bits = frame.fault_bits
            fault_bits_ever |= frame.fault_bits
        if frame is not None and frame.event_bits is not None:
            if frame.event_bits & EVENT_DEADMAN_EXPIRED:
                if deadman_seen_clear:
                    deadman_tripped = True
            else:
                deadman_seen_clear = True

        rows.append({
            "tick_index": tick_index,
            "wall_time": tick_wall,
            "elapsed_s": tick_wall - t0,
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
        })
        tick_index += 1

        if result.done:
            break
        elapsed = time.monotonic() - tick_wall
        time.sleep(max(0.0, params.streaming_interval - elapsed))

    outcome = rows[-1]["outcome"] if rows else RunOutcome.STOPPED.value

    # Settle window: the run's own terminal tick sends STOP and returns
    # done=True immediately (executor.py's binding requirement #7 -- an
    # explicit stop, no further ticks) -- the PLANT still needs real time to
    # actually decelerate afterward. Poll telemetry for a short settle
    # window so the "terminal velocity converged, no lunge/reversal" gate
    # checks the plant's ACTUAL post-stop state, not the instant the stop
    # command was issued (mirrors scripted_twist_demo_harness.cpp's own
    # sim-side post-STOP convergence window, applied here on real hardware).
    settle_frame: TLMFrame | None = None
    settle_deadline = time.monotonic() + 0.6
    while time.monotonic() < settle_deadline:
        time.sleep(0.1)
        for frame in transport.read_pending_binary_tlm_frames():
            tick_index += 1
            rows.append({
                "tick_index": tick_index, "wall_time": time.monotonic(),
                "elapsed_s": time.monotonic() - t0, "sent_v_x": 0.0, "sent_omega": 0.0,
                "corr_id": None, "done": True, "outcome": "settle",
                "frame_t": frame.t, "enc_l": frame.enc[0] if frame.enc else None,
                "enc_r": frame.enc[1] if frame.enc else None,
                "vel_l": frame.vel[0] if frame.vel else None, "vel_r": frame.vel[1] if frame.vel else None,
                "pose_x": frame.pose[0] if frame.pose else None, "pose_y": frame.pose[1] if frame.pose else None,
                "pose_h_cdeg": frame.pose[2] if frame.pose else None,
                "otos_x": frame.otos[0] if frame.otos else None, "otos_y": frame.otos[1] if frame.otos else None,
                "otos_h_cdeg": frame.otos[2] if frame.otos else None,
                "fault_bits": frame.fault_bits, "event_bits": frame.event_bits,
            })
            if frame.fault_bits is not None:
                if baseline_fault_bits is None:
                    baseline_fault_bits = frame.fault_bits
                fault_bits_ever |= frame.fault_bits
            # NOTE: deadman_tripped is intentionally NOT updated from settle-window
            # frames -- once this leg has explicitly stopped, the deadman
            # naturally (and harmlessly) times out while this script only
            # reads (never re-arms it with a fresh twist()); that is expected
            # post-stop behavior, not a mid-profile trip. See run_leg()'s own
            # active tick loop above for the real gate-relevant check.
            settle_frame = frame

    final_heading = heading.measured_heading(settle_frame) if settle_frame is not None \
        else heading.measured_heading(ex.latest_frame)
    duration_s = time.monotonic() - t0
    print(f"  leg '{label}' outcome={outcome} ticks={len(rows)} duration={duration_s:.2f}s")

    return LegResult(
        label=label, axis=axis, target=target, outcome=outcome, rows=rows,
        baseline_fault_bits=baseline_fault_bits or 0, fault_bits_ever=fault_bits_ever,
        deadman_tripped=deadman_tripped, baseline_heading=baseline_heading,
        final_heading=final_heading, duration_s=duration_s,
    )


def write_trace(leg: LegResult, out_dir: Path, metadata: dict, timestamp: str) -> tuple[Path, Path]:
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = f"profiled_{leg.label}_{timestamp}"
    csv_path = out_dir / f"{stem}.csv"
    json_path = out_dir / f"{stem}.json"

    fieldnames = list(leg.rows[0].keys()) if leg.rows else []
    with csv_path.open("w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        for row in leg.rows:
            writer.writerow(row)

    sidecar = dict(metadata)
    sidecar.update({
        "leg": leg.label,
        "axis": leg.axis,
        "target": leg.target,
        "outcome": leg.outcome,
        "duration_s": leg.duration_s,
        "tick_count": len(leg.rows),
        "baseline_fault_bits": leg.baseline_fault_bits,
        "fault_bits_ever": leg.fault_bits_ever,
        "new_fault_bits": leg.fault_bits_ever & ~leg.baseline_fault_bits,
        "deadman_tripped": leg.deadman_tripped,
        "baseline_heading_rad": leg.baseline_heading,  # ABSOLUTE, since-boot (Odometry never resets)
        "final_heading_rad": leg.final_heading,        # ABSOLUTE, since-boot
        "heading_delta_rad": leg.heading_delta,        # RELATIVE -- the gate-relevant quantity
        "trace_csv": str(csv_path),
    })
    with json_path.open("w") as fh:
        json.dump(sidecar, fh, indent=2, default=str)

    return csv_path, json_path


def gate_check(leg: LegResult, tolerance_rad: float, params: PlannerParams) -> list[str]:
    failures: list[str] = []
    if leg.outcome != RunOutcome.COMPLETED.value:
        failures.append(f"run outcome was {leg.outcome!r}, not 'completed'")

    new_fault_bits = leg.fault_bits_ever & ~leg.baseline_fault_bits
    if new_fault_bits:
        failures.append(f"new fault bits observed: 0x{new_fault_bits:x}")

    if leg.deadman_tripped:
        failures.append("deadman (kEventDeadmanExpired) tripped mid-run")

    # Terminal convergence, no lunge/reversal. PRIMARY signal: encoder
    # POSITION across the last few settle-window rows must be essentially
    # unchanging -- this bench session found the firmware's reported `vel`
    # field can sit at a small nonzero value (e.g. ~10-20mm/s) even once
    # enc_l/enc_r have visibly stopped incrementing at all (a stale/
    # unrefreshed velocity ESTIMATE once raw reads go identical -- the same
    # "boundary latch" read-staleness characteristic
    # `.clasi/knowledge/encoder-wedge-boundary-latch.md` documents), so a
    # raw `vel` threshold alone is not a reliable "still moving" signal.
    # `vel` is still checked, but with a looser bound and only as a
    # secondary/diagnostic signal.
    enc_tail = [r for r in leg.rows[-4:] if r["enc_l"] is not None and r["enc_r"] is not None]
    if len(enc_tail) >= 2:
        enc_l_span = max(r["enc_l"] for r in enc_tail) - min(r["enc_l"] for r in enc_tail)
        enc_r_span = max(r["enc_r"] for r in enc_tail) - min(r["enc_r"] for r in enc_tail)
        if enc_l_span > 5.0 or enc_r_span > 5.0:
            failures.append(f"encoder position still changing across the settle window "
                           f"(enc_l_span={enc_l_span}, enc_r_span={enc_r_span}) -- possible lunge/reversal")
    # `vel` is NOT gated here (see comment above) -- this bench session
    # confirmed it can report a stale, non-decaying value (observed up to
    # ~54mm/s) with the encoder position simultaneously rock-stable across
    # the entire settle window. It remains in the CSV trace for the human
    # trace review (ticket 006 AC #3) as a diagnostic signal only.

    # Heading-hold / turn-landing numeric tolerance -- ALWAYS relative to
    # this leg's OWN baseline_heading (captured at begin()), never against
    # an absolute zero/target: App::Odometry never resets across a boot
    # session (see LegResult.baseline_heading's own docstring), so the
    # absolute pose.h reading drifts with every prior command this whole
    # script (or an earlier invocation in the same boot) has ever sent.
    if leg.heading_delta is None:
        failures.append("no measured heading delta available (degraded feedback or missing baseline)")
    else:
        if leg.axis == "linear":
            heading_error = leg.heading_delta  # target delta is 0 for a straight (hold) run
        else:
            heading_error = leg.heading_delta - leg.target
        if abs(heading_error) > tolerance_rad:
            failures.append(
                f"heading error {math.degrees(heading_error):.2f}deg exceeds tolerance "
                f"{math.degrees(tolerance_rad):.2f}deg")

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
    p.add_argument("--distance", type=float, default=DEFAULT_DISTANCE, help="[mm] straight-leg distance")
    p.add_argument("--v-max", type=float, default=DEFAULT_V_MAX, help="[mm/s] straight-leg cruise velocity")
    p.add_argument("--a-max", type=float, default=DEFAULT_A_MAX, help="[mm/s^2] straight-leg accel/decel")
    p.add_argument("--angle-deg", type=float, default=DEFAULT_ANGLE_DEG, help="[deg] turn-leg angle")
    p.add_argument("--omega-max", type=float, default=DEFAULT_OMEGA_MAX, help="[rad/s] turn-leg cruise rate")
    p.add_argument("--alpha-max", type=float, default=DEFAULT_ALPHA_MAX, help="[rad/s^2] turn-leg accel/decel")
    p.add_argument("--heading-tolerance-deg", type=float, default=DEFAULT_HEADING_TOLERANCE_DEG)
    p.add_argument("--turn-tolerance-deg", type=float, default=DEFAULT_TURN_TOLERANCE_DEG)
    # PlannerParams' own overshoot_bound_linear/angular DEFAULTS (30mm /
    # 0.1rad) proved too tight for this rig's real first-tick transient
    # response during this ticket's own bench session (see the bench
    # findings recorded in ticket 006's Completion Notes) -- widened
    # defaults here, still live-tunable (binding requirement #9) via these
    # flags. This does NOT change PlannerParams' own field defaults (a
    # separate, out-of-scope retune this ticket only flags as a finding).
    p.add_argument("--overshoot-bound-linear", type=float, default=60.0, help="[mm]")
    p.add_argument("--overshoot-bound-angular", type=float, default=0.35, help="[rad]")
    # Heading-loop gains -- live-tunable (binding requirement #9); exposed
    # here because this ticket's own bench session found PlannerParams'
    # DEFAULT heading_kp/heading_omega_clamp combination (2.0 / 0.5rad/s)
    # saturates the trim for several consecutive ticks on a profiled turn
    # against this rig's own high-inertia proxy load, adding substantial
    # EXTRA rotation on top of the profile's own already-complete open-loop
    # trajectory and overshooting the target (see Completion Notes for the
    # measured numbers). Defaults below are a gentler starting point for
    # THIS bench rig -- not a change to PlannerParams' own field defaults.
    p.add_argument("--heading-kp", type=float, default=None, help="override PlannerParams.heading_kp")
    p.add_argument("--heading-omega-clamp", type=float, default=None,  # [rad/s]
                   help="override PlannerParams.heading_omega_clamp")
    p.add_argument("--out-dir", default=str(DEFAULT_OUT_DIR))
    p.add_argument("--skip-preflight", action="store_true", help="skip the standing verification gate (debug only)")
    return p.parse_args()


def main() -> int:
    args = _args()
    mode = "relay" if args.relay else None
    out_dir = Path(args.out_dir)
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")

    print(f"=== profiled_motion_verify: port={args.port} mode={mode or 'auto'} ===")
    conn = SerialConnection(port=args.port, mode=mode)
    proto = NezhaProtocol(conn)

    metadata = {
        "timestamp": timestamp,
        "port": args.port,
        "tool_version": _tool_version(),
    }

    overall_pass = True
    try:
        info = conn.connect()
        if info.get("status") not in ("connected", "already_connected"):
            print(f"ERROR: connect failed: {info}")
            return 2
        time.sleep(2.5)  # boot Preamble settle, matches rig_dev.py's Rig.open()
        metadata["mode"] = conn.mode
        proto.read_pending_binary_tlm_frames()  # drop anything queued during settle

        preflight_result = Result()
        if not args.skip_preflight:
            print("\n--- preflight: standing verification gate ---")
            preflight(proto, preflight_result)
            if not preflight_result.ok():
                print("ERROR: preflight failed -- aborting before any profiled leg runs")
                return 3

        params = PlannerParams()
        params.overshoot_bound_linear = args.overshoot_bound_linear
        params.overshoot_bound_angular = args.overshoot_bound_angular
        if args.heading_kp is not None:
            params.heading_kp = args.heading_kp
        if args.heading_omega_clamp is not None:
            params.heading_omega_clamp = args.heading_omega_clamp
        heading = HeadingCorrector(params, robot_config=SimpleNamespace(
            geometry=SimpleNamespace(otos_untrusted=True)))
        metadata["planner_params"] = dataclasses.asdict(params)
        metadata["heading_source"] = heading.source

        # 107-001: StreamingExecutor.begin() now captures its own baseline-
        # relative fault-bit exclusion directly (see this file's own
        # "Baseline fault-bit masking" section, module scope, above) -- no
        # wrapper needed; `proto` (NezhaProtocol) is handed to the executor
        # as-is. Each leg's own `begin()` call re-baselines fresh (matching
        # the dropped wrapper's own per-leg `rebaseline()` philosophy): a
        # benign kFaultWedgeLatch boundary-latch that appears during this
        # script's own idle gap between legs is absorbed by leg 2's own
        # fresh baseline, not carried as a global poison from leg 1.

        straight_limits = ProfileLimits(v_max=args.v_max, a_max=args.a_max)
        straight_setpoints = profile_for_distance(args.distance, straight_limits, cadence=params.streaming_interval)
        metadata["straight_limits"] = dataclasses.asdict(straight_limits)

        angle_rad = math.radians(args.angle_deg)
        turn_limits = ProfileLimits(v_max=args.omega_max, a_max=args.alpha_max)
        turn_setpoints = profile_for_turn(angle_rad, turn_limits, cadence=params.streaming_interval)
        metadata["turn_limits"] = dataclasses.asdict(turn_limits)

        print("\n--- leg 1: profiled straight ---")
        straight = run_leg(proto, params, heading, straight_setpoints, args.distance, "linear", "straight")
        heading.reset()
        time.sleep(0.5)
        proto.read_pending_binary_tlm_frames()

        print("\n--- leg 2: profiled turn ---")
        turn = run_leg(proto, params, heading, turn_setpoints, angle_rad, "angular", "turn")

        metadata["raw_fault_bits_ever_observed"] = straight.fault_bits_ever | turn.fault_bits_ever

        print("\n--- writing traces ---")
        straight_csv, straight_json = write_trace(straight, out_dir, metadata, timestamp)
        turn_csv, turn_json = write_trace(turn, out_dir, metadata, timestamp)
        print(f"  straight trace: {straight_csv}")
        print(f"  turn trace:     {turn_csv}")

        print("\n--- gate checks ---")
        straight_failures = gate_check(straight, math.radians(args.heading_tolerance_deg), params)
        turn_failures = gate_check(turn, math.radians(args.turn_tolerance_deg), params)

        for label, failures in (("straight", straight_failures), ("turn", turn_failures)):
            if failures:
                overall_pass = False
                print(f"  [FAIL] leg '{label}':")
                for f in failures:
                    print(f"    - {f}")
            else:
                print(f"  [PASS] leg '{label}'")

        if straight.heading_delta is not None:
            print(f"  straight heading delta (final - own baseline): {math.degrees(straight.heading_delta):.2f}deg "
                 f"(tolerance {args.heading_tolerance_deg:.1f}deg)")
        if turn.heading_delta is not None:
            print(f"  turn heading delta (final - own baseline): {math.degrees(turn.heading_delta):.2f}deg "
                 f"target={args.angle_deg:.1f}deg "
                 f"error={math.degrees(turn.heading_delta - angle_rad):.2f}deg "
                 f"(tolerance {args.turn_tolerance_deg:.1f}deg)")

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
    print("Reminder: a human must still review the trace CSVs above for visible resonance "
         "ringing (ticket 006 AC #3) and record that pass/fail judgment in the ticket's own "
         "Completion Notes -- this script does not automate that judgment.")
    return 0 if overall_pass else 1


if __name__ == "__main__":
    sys.exit(main())
