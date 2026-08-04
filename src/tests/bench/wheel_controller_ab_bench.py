#!/usr/bin/env python3
"""wheel_controller_ab_bench.py -- sprint 130 ticket 006: bench acceptance for
App::Drive's unified wheel-speed controller (Stage A/B/C, tickets 004/005) --
the A/B (old additive-trim baseline vs. the new map-adaptation controller),
the right-wheel affine-residual closure across cmd 100/150/250/400 mm/s, and
the +500-button acceptance spec re-verification
(06-duty-per-speed-and-wheel-gain-disagree-with-the-plant.md).

Bench setup: `tovez` on the stand, wheels free
(`.claude/rules/hardware-bench-testing.md`). Real hardware only -- there is
no rig/sim harness for this ticket's own subject (App::Drive's controller,
not the deleted Motion::WheelTrim square_tour_sim.py exercised -- see that
script's own RETIRED docstring, square_tour_velocity.py, for why it is not
reused here).

A/B baseline note: `Motion::WheelTrim` (the "old additive trim") was DELETED
outright by ticket 005 -- there is no old code left to re-run side by side.
Per square_tour_velocity.py's own RETIRED docstring, the historical A/B
comparison point is the DATA already captured before the deletion
(square_tour_velocity_trim.csv, this directory) and the ab303ee3 commit's
own measured table (REFERENCE_RESIDUAL below) -- THIS script's hardware run
is the "B" side, scored against that historical "A" record, not a re-run of
deleted code.

+500 button: `run_unmanaged_distance_drive()` (`robot_radio.testgui.
transport`) IS the button's actual host-side implementation (a host-side
speed profile over repeated bounded WHEELS leases -- see that function's own
docstring). This script calls the SAME function against real hardware (not
a reimplementation), wrapped in a frame-logging shim (`FrameSink`) so every
telemetry sample is available for scoring against the 06-issue's agreed
acceptance spec, not just the function's own single latest-frame view.

Deliberately NOT attempted this session (stand-unmeasurable, not skipped
silently -- see the module-level `STAND_UNMEASURABLE` note printed at the
end of a run):
  - "WHEELS teleop under applied drag holds its commanded speed" -- applying
    a real disturbance load needs a coupled friction rig
    (src/tests/dev/pid_hold_speed.py's own rig, quarantined this session
    per project convention -- "rig" only on explicit stakeholder request)
    or a second person's hand on the wheel; neither is available in this
    autonomous bench session. Deferred, not faked.
  - Net heading change and camera-measured travel (06-issue criteria 8/9):
    the robot is on a STAND, wheels off the ground -- it does not
    translate or rotate in the world frame no matter what the wheels do.
    Deferred to a playfield session (ticket 012's regime).

Usage:
    uv run python src/tests/bench/wheel_controller_ab_bench.py
    uv run python src/tests/bench/wheel_controller_ab_bench.py --port /dev/cu.usbmodem2121202
    uv run python src/tests/bench/wheel_controller_ab_bench.py --skip-stage-b
"""
from __future__ import annotations

import argparse
import csv
import pathlib
import statistics
import sys
import threading
import time
from dataclasses import dataclass

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol, TLMFrame
from robot_radio.testgui.transport import (
    _UNMANAGED_EASE, _UNMANAGED_FLOOR, _UNMANAGED_SPEED, run_unmanaged_distance_drive)

_BENCH_DIR = pathlib.Path(__file__).resolve().parent
if str(_BENCH_DIR) not in sys.path:
    sys.path.insert(0, str(_BENCH_DIR))

from wheel_control_tuning import (  # noqa: E402  (path must be set up first)
    TuningNotConfirmed, describe_persistence, format_gains, push_gains)

DEFAULT_PORT = "/dev/cu.usbmodem2121202"      # tovez -- confirm with `uv run mbdeploy list`
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]

# Pre-sprint reference table (ab303ee3, "firmware: bake the measured
# duty-per-speed; hold wheel_gain at identity" -- measured on the stand,
# firmware v0.20260731.15, BEFORE Stage C bias adaptation existed). The
# "before" numbers the residual-closure criterion below is judged against.
REFERENCE_RESIDUAL = {   # cmd mm/s -> (measured_left, measured_right) mm/s
    100.0: (99.0, 70.0),
    150.0: (145.0, 115.0),
    250.0: (239.0, 210.0),
    400.0: (375.0, 344.0),
}
RESIDUAL_SPEEDS = (100.0, 150.0, 250.0, 400.0)
RESIDUAL_HOLD_S = 6.0        # [s] one continuous wheels() lease per speed
RESIDUAL_SETTLE_FRAC = 0.5   # last 50% of the hold counted as steady-state

# Stage B trial gains (Open Question 4, ticket 004): kp/ki/iMax are a
# modest, small-authority exploration point; kaff=0.23s is
# `planner.plant_tau` VERBATIM -- ticket 004's own completion notes name
# this as "wheel_pid_kaff's natural non-zero starting value" (wheel_trim.h's
# convention: kaff IS the plant time constant). pidMax=30 mm/s is sized
# comparable to (slightly above) wheel_bias_max=23.8mm/s -- Stage B is
# meant to stay a SMALL correction layered on Stage A+C, not a second
# wholesale re-derivation of the whole map.
STAGE_B_TRIAL_GAINS = {
    "pid.kp": 0.3, "pid.ki": 0.02, "pid.iMax": 20.0,
    "pid.kff": 0.23, "pid.kaw": 30.0,
}
STAGE_B_ZERO_GAINS = {
    "pid.kp": 0.0, "pid.ki": 0.0, "pid.iMax": 0.0, "pid.kff": 0.0, "pid.kaw": 0.0,
}

STAND_UNMEASURABLE = (
    "STAND-UNMEASURABLE (deferred, not faked): WHEELS-under-applied-drag "
    "(needs a coupled friction rig or a hand on the wheel -- neither "
    "available this session); net heading change and camera-measured "
    "travel (the robot is on a stand, wheels off the ground -- it does "
    "not translate/rotate in the world frame). Defer to a playfield "
    "session (ticket 012's regime).")


def _args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=DEFAULT_PORT)
    p.add_argument("--skip-stage-b", action="store_true",
                   help="skip the Stage B (fast PID) live-tuning A/B; run the "
                        "+500 spec only at shipped (all-zero) gains")
    # NOT src/tests/bench/out/ -- that directory is gitignored wholesale
    # (.gitignore:79); this ticket's own plan requires the captured
    # dataset/chart to be COMMITTED (project convention: "ALWAYS send the
    # chart"), matching where duty_sweep.py/square_tour_velocity.py's own
    # CSVs/PNGs already live -- directly under src/tests/bench/, not out/.
    p.add_argument("--csv", default=str(_REPO_ROOT / "src" / "tests" / "bench" / "wheel_controller_ab_bench.csv"))
    p.add_argument("--png", default=str(_REPO_ROOT / "src" / "tests" / "bench" / "wheel_controller_ab_bench.png"))
    return p.parse_args()


# ---------------------------------------------------------------------------
# Pure analysis helpers -- no I/O, unit-testable. `src/tests/bench/` is HITL
# CLI tooling, not pytest-collected (tests/CLAUDE.md); this module's pure
# logic is exercised by src/tests/unit/test_wheel_controller_ab_bench.py,
# which loads this file by path -- the SAME precedent
# test_duty_sweep_population.py already established for duty_sweep.py.
# ---------------------------------------------------------------------------

@dataclass
class Sample:
    t: float          # [s] relative to this run's own t=0 (host clock)
    vl: float          # [mm/s] measured
    vr: float          # [mm/s] measured
    remaining: float   # [mm] distance-to-go, the button's own formula


def rise_time_s(samples: "list[Sample]", target: float, frac: float = 0.9) -> "float | None":
    """Seconds from the first sample to the first frame where BOTH wheels
    have crossed frac*target -- the +500 spec's own "both wheels rise to
    cruise" wording (criterion 1), not just one wheel."""
    if not samples or target <= 0.0:
        return None
    threshold = frac * target
    t0 = samples[0].t
    for s in samples:
        if s.vl >= threshold and s.vr >= threshold:
            return s.t - t0
    return None


def cruise_samples(samples: "list[Sample]", ease: float = _UNMANAGED_EASE) -> "list[Sample]":
    """Samples in the flat-plateau regime (remaining > ease mm)."""
    return [s for s in samples if s.remaining > ease]


def taper_samples(samples: "list[Sample]", ease: float = _UNMANAGED_EASE) -> "list[Sample]":
    """Samples in the taper regime (0 < remaining <= ease mm)."""
    return [s for s in samples if 0.0 < s.remaining <= ease]


def ripple_mm_s(vals: "list[float]") -> float:
    """Peak-to-peak spread -- the +500 spec's own ripple wording
    ("+/-10 mm/s frame to frame")."""
    if not vals:
        return float("nan")
    return max(vals) - min(vals)


def max_lr_split(samples: "list[Sample]") -> float:
    if not samples:
        return float("nan")
    return max(abs(s.vl - s.vr) for s in samples)


def taper_neither_hits_zero(samples: "list[Sample]", stopped_floor: float = 5.0,
                            moving_floor: float = 20.0) -> bool:
    """True if, throughout the taper, no wheel reads at-or-below
    `stopped_floor` while the OTHER wheel is still clearly driving (above
    `moving_floor`) -- the spec's own "neither wheel reaches 0 while the
    other still moves" wording. Vacuously True for an empty taper (nothing
    to violate) -- callers should also check `len(samples) > 0` before
    trusting this as a real PASS."""
    for s in samples:
        lo, hi = min(s.vl, s.vr), max(s.vl, s.vr)
        if lo <= stopped_floor and hi >= moving_floor:
            return False
    return True


def score_plus500(samples: "list[Sample]", *, target: float = _UNMANAGED_SPEED,
                  ease: float = _UNMANAGED_EASE, floor: float = _UNMANAGED_FLOOR) -> dict:
    """Reduce one +500 run's samples to the 06-issue's own numbered
    criteria 1-5 (the ones a stand can measure -- criteria 8/9, heading and
    camera travel, are stand-unmeasurable by construction, see this
    module's STAND_UNMEASURABLE note)."""
    rise = rise_time_s(samples, target)
    cruise = cruise_samples(samples, ease)
    # "Plateau...held for the leg" (criterion 2): score ripple/L-R match
    # only AFTER the rise completes (both wheels first crossed 90% of
    # target) -- filtering on instantaneous velocity alone would let the
    # tail of the RISE itself (still >90% target, still inside the cruise
    # distance window) leak into the "flat plateau" sample, inflating
    # ripple with what is really rise-transient wobble, not steady-state
    # ripple. No rise (`rise is None`) means no plateau either -- a
    # genuine "never got there" failure, not an empty-list accident.
    t0 = samples[0].t if samples else 0.0
    plateau = [] if rise is None else [s for s in cruise if (s.t - t0) >= rise]
    ripple_l = ripple_mm_s([s.vl for s in plateau])
    ripple_r = ripple_mm_s([s.vr for s in plateau])
    split = max_lr_split(plateau)
    taper = taper_samples(samples, ease)
    taper_ok = bool(taper) and taper_neither_hits_zero(taper)
    return {
        "rise_time_s": rise,
        "rise_ok": rise is not None and rise <= 0.3,
        "n_plateau": len(plateau),
        "ripple_l": ripple_l,
        "ripple_r": ripple_r,
        "ripple_ok": (ripple_l == ripple_l and ripple_r == ripple_r  # not NaN
                     and ripple_l <= 10.0 and ripple_r <= 10.0),
        "max_split": split,
        "split_ok": split == split and split <= 10.0,
        "n_taper": len(taper),
        "taper_ok": taper_ok,
    }


# ---------------------------------------------------------------------------
# Bench I/O
# ---------------------------------------------------------------------------

class FrameSink:
    """Minimal transport shim for run_unmanaged_distance_drive(): exposes
    `latest_frame` (what the button's own observe() reads) and, unlike the
    TestGUI's real Transport, ALSO accumulates every frame drained with a
    host-monotonic timestamp -- the full trace this script needs to score
    the +500 spec, not just the button's own single latest-frame view."""

    def __init__(self, proto: NezhaProtocol) -> None:
        self._proto = proto
        self.latest_frame: "TLMFrame | None" = None
        self.frames: "list[tuple[float, TLMFrame]]" = []
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._pump, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)

    def _pump(self) -> None:
        while not self._stop.is_set():
            for f in self._proto.read_pending_binary_tlm_frames():
                self.latest_frame = f
                self.frames.append((time.monotonic(), f))
            time.sleep(0.01)

    def frames_since(self, t0: float) -> "list[tuple[float, TLMFrame]]":
        return [(t, f) for t, f in self.frames if t >= t0]


def hold_wheels(proto: NezhaProtocol, sink: FrameSink, v: float, duration_s: float,
                label: str) -> "list[tuple[float, TLMFrame]]":
    """Command wheels(v, v, duration_s*1000) as ONE bounded lease (no
    re-arming -- the residual/saturation checks want a clean, uninterrupted
    hold, unlike the +500 button's own repeated-burst shape) and return the
    frames captured during and just after it."""
    t0 = time.monotonic()
    print(f"  [{label}] wheels({v:g}, {v:g}, {duration_s * 1000:.0f}ms)")
    proto.wheels(v, v, duration_s * 1000.0)
    time.sleep(duration_s + 0.3)
    proto.estop()
    time.sleep(0.3)
    return sink.frames_since(t0)


def steady_state_velocity(frames: "list[tuple[float, TLMFrame]]", duration_s: float,
                          frac: float = 0.5) -> "tuple[float | None, float | None]":
    """Mean measured (left, right) velocity over the LAST `frac` of the
    hold window -- excludes the rise transient."""
    if not frames:
        return None, None
    t0 = frames[0][0]
    cutoff = t0 + duration_s * (1.0 - frac)
    vl = [f.enc_left.velocity for t, f in frames if t >= cutoff and f.enc_left is not None]
    vr = [f.enc_right.velocity for t, f in frames if t >= cutoff and f.enc_right is not None]
    return (statistics.mean(vl) if vl else None, statistics.mean(vr) if vr else None)


def run_plus500(proto: NezhaProtocol, sink: FrameSink, label: str) -> dict:
    """Call the ACTUAL +500-button implementation
    (`run_unmanaged_distance_drive`, `robot_radio.testgui.transport`)
    against real hardware, then reduce every frame captured during the call
    into `Sample`s scored against the 06-issue's spec."""
    t0 = time.monotonic()
    pre = sink.latest_frame
    base_l, base_r = (pre.enc if (pre is not None and pre.enc) else (0, 0))

    run_unmanaged_distance_drive(transport=sink, driver=proto, distance=500.0,
                                 direction=1.0, label=label, log=print)
    t1 = time.monotonic()

    frames = sink.frames_since(t0)
    samples: "list[Sample]" = []
    for t, f in frames:
        if f.enc is None or f.enc_left is None or f.enc_right is None:
            continue
        dl = f.enc[0] - base_l
        dr = f.enc[1] - base_r
        avg = (dl + dr) / 2.0
        samples.append(Sample(t=t - t0, vl=f.enc_left.velocity, vr=f.enc_right.velocity,
                              remaining=500.0 - avg))

    final = frames[-1][1] if frames else None
    final_dl = (final.enc[0] - base_l) if (final is not None and final.enc is not None) else None
    final_dr = (final.enc[1] - base_r) if (final is not None and final.enc is not None) else None

    return {
        "samples": samples,
        "elapsed_s": t1 - t0,
        "final_dl": final_dl,
        "final_dr": final_dr,
        "bias_left_end": final.bias_left if final is not None else None,
        "bias_right_end": final.bias_right if final is not None else None,
    }


def _revert_stage_b(proto: NezhaProtocol, label: str) -> bool:
    """Push `STAGE_B_ZERO_GAINS` and report the read-back. This is the
    safety-ish revert path (leave the robot at the shipped all-zero Stage B
    default) -- a failed revert must be reported LOUDLY (never swallowed
    silently) but must never itself raise: the caller always has its own
    cleanup (`estop()`/`tlmOff()`/disconnect in `main()`'s `finally`) still
    to run, and a raised exception here must not skip it or crash the
    script before that cleanup executes. Returns True if the revert was
    confirmed, False if it was not (NAK, timeout, or read-back
    disagreement) -- the caller uses this to report the true end state
    rather than assuming a revert attempt always succeeds."""
    try:
        readback = push_gains(proto, STAGE_B_ZERO_GAINS)
    except TuningNotConfirmed as exc:
        print(f"  ERROR: revert to all-zero Stage B gains NOT confirmed ({label}): {exc}")
        return False
    print(f"    confirmed reverted to all-zero ({label}): {format_gains(readback) or '(all zero)'}")
    return True


def _write_csv(path: pathlib.Path, frames: "list[tuple[float, TLMFrame]]") -> None:
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "vl", "vr", "encl", "encr", "dutyPerSpeedL", "dutyPerSpeedR",
                    "biasL", "biasR", "pidL", "pidR", "deficitL", "deficitR"])
        t0 = frames[0][0] if frames else 0.0
        for t, f in frames:
            w.writerow([
                f"{t - t0:.3f}",
                f.enc_left.velocity if f.enc_left else "",
                f.enc_right.velocity if f.enc_right else "",
                f.enc_left.position if f.enc_left else "",
                f.enc_right.position if f.enc_right else "",
                f.duty_per_speed_left, f.duty_per_speed_right,
                f.bias_left, f.bias_right, f.pid_left, f.pid_right,
                f.fault_wheel_deficit_left, f.fault_wheel_deficit_right,
            ])


def _write_chart(path: pathlib.Path, base_run: dict, tuned_run: "dict | None") -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("(matplotlib unavailable -- skipping chart)")
        return

    runs = [("+500 baseline (Stage B = 0)", base_run)]
    if tuned_run is not None:
        runs.append(("+500 tuned Stage B", tuned_run))

    fig, axes = plt.subplots(1, len(runs), figsize=(8 * len(runs), 5), squeeze=False)
    for ax, (title, run) in zip(axes[0], runs):
        ts = [s.t for s in run["samples"]]
        vl = [s.vl for s in run["samples"]]
        vr = [s.vr for s in run["samples"]]
        ax.plot(ts, vl, label="left")
        ax.plot(ts, vr, label="right")
        ax.axhline(_UNMANAGED_SPEED, color="gray", linestyle="--", linewidth=1, label="commanded 150")
        ax.axhline(_UNMANAGED_FLOOR, color="lightgray", linestyle=":", linewidth=1, label="taper floor 90")
        ax.set_title(title)
        ax.set_xlabel("t [s]")
        ax.set_ylabel("velocity [mm/s]")
        ax.legend(fontsize=8)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=120)
    print(f"  chart saved: {path}")


def main() -> int:
    args = _args()
    csv_path = pathlib.Path(args.csv)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    png_path = pathlib.Path(args.png)

    conn = SerialConnection(port=args.port)
    proto: "NezhaProtocol | None" = None
    sink: "FrameSink | None" = None

    try:
        info = conn.connect()
        if info.get("status") not in ("connected", "already_connected"):
            print(f"ERROR: connect failed: {info}")
            return 2
        print(f"  connected: mode={info.get('mode')}")
        proto = NezhaProtocol(conn)
        proto.tlmOn()
        time.sleep(1.0)
        proto.read_pending_binary_tlm_frames()

        sink = FrameSink(proto)
        sink.start()
        time.sleep(0.5)

        f0 = sink.latest_frame
        if f0 is None:
            print("ERROR: no telemetry received after TLM:ON -- robot silent, aborting")
            return 2
        print(f"  status: conn_left={f0.conn_left} conn_right={f0.conn_right} "
              f"otos_present={f0.otos_present} wedge_latch={f0.fault_wedge_latch}")
        if not (f0.conn_left and f0.conn_right):
            print("ERROR: a drive motor is not connected on the bus -- aborting before driving")
            return 2

        # --- 1. Saturation anchor (constant-free) ---
        print("\n=== 1. Saturation anchor (no constants) ===")
        sat_frames = hold_wheels(proto, sink, 800.0, 2.5, "saturation")
        if len(sat_frames) < 3:
            print("ERROR: robot went silent during the saturation hold -- aborting")
            return 3
        sat_vl, sat_vr = steady_state_velocity(sat_frames, 2.5, frac=0.4)
        print(f"  L={sat_vl:.1f}  R={sat_vr:.1f} mm/s  (historical: L 760-795 / R 696)")
        time.sleep(1.0)

        # --- 2. Residual sweep across 100/150/250/400 ---
        print("\n=== 2. Residual sweep (Stage A + live Stage C bias) ===")
        residual = {}
        for v in RESIDUAL_SPEEDS:
            frames = hold_wheels(proto, sink, v, RESIDUAL_HOLD_S, f"residual {v:g}")
            if len(frames) < 3:
                print(f"ERROR: robot went silent during the {v:g}mm/s hold -- aborting; "
                      f"residual data so far: {residual}")
                return 3
            vl, vr = steady_state_velocity(frames, RESIDUAL_HOLD_S, RESIDUAL_SETTLE_FRAC)
            last = frames[-1][1]
            residual[v] = {
                "vl": vl, "vr": vr,
                "bias_l": last.bias_left, "bias_r": last.bias_right,
                "deficit_l": last.fault_wheel_deficit_left,
                "deficit_r": last.fault_wheel_deficit_right,
            }
            ref_l, ref_r = REFERENCE_RESIDUAL[v]
            print(f"  cmd {v:g}: L={vl:.1f} ({vl / v * 100:.0f}%)  R={vr:.1f} ({vr / v * 100:.0f}%)"
                  f"   [pre-sprint: L={ref_l:.0f} R={ref_r:.0f}]"
                  f"   bias L={last.bias_left:.2f} R={last.bias_right:.2f}"
                  f"   deficit L={last.fault_wheel_deficit_left} R={last.fault_wheel_deficit_right}")
            time.sleep(1.0)

        # --- 3. +500 button, baseline (shipped, all-zero Stage B gains) ---
        print("\n=== 3. +500 button, baseline gains (shipped: Stage B all-zero) ===")
        base_run = run_plus500(proto, sink, "+500 baseline")
        if len(base_run["samples"]) < 3:
            print("ERROR: robot went silent during the baseline +500 run -- aborting")
            return 3
        base_score = score_plus500(base_run["samples"])
        print(f"  elapsed={base_run['elapsed_s']:.2f}s  "
              f"final L={base_run['final_dl']:.1f} R={base_run['final_dr']:.1f}mm")
        print(f"  rise={base_score['rise_time_s']}  "
              f"ripple L/R={base_score['ripple_l']:.1f}/{base_score['ripple_r']:.1f}  "
              f"max|vL-vR|={base_score['max_split']:.1f}  taper_ok={base_score['taper_ok']}")
        time.sleep(1.5)

        tuned_run = tuned_score = None
        kept_gains = None
        revert_confirmed = None
        if not args.skip_stage_b:
            print("\n=== 4. Stage B trial gains ===")
            print(f"  pushing live: {STAGE_B_TRIAL_GAINS}")
            try:
                trial_readback = push_gains(proto, STAGE_B_TRIAL_GAINS)
            except TuningNotConfirmed as exc:
                print(f"  ERROR: {exc}")
                print("  Stage B trial gains not confirmed on the robot -- aborting Stage B A/B")
                return 3
            print(f"    confirmed on robot: {format_gains(trial_readback)}")
            for line in describe_persistence(trial_readback):
                print(line)
            time.sleep(0.5)

            print("\n=== 5. +500 button, tuned Stage B gains ===")
            tuned_run = run_plus500(proto, sink, "+500 tuned")
            if len(tuned_run["samples"]) < 3:
                print("ERROR: robot went silent during the tuned +500 run -- reverting gains and aborting")
                _revert_stage_b(proto, "post-abort")  # loudly reports its own failure; never raises
                return 3
            tuned_score = score_plus500(tuned_run["samples"])
            print(f"  elapsed={tuned_run['elapsed_s']:.2f}s  "
                  f"final L={tuned_run['final_dl']:.1f} R={tuned_run['final_dr']:.1f}mm")
            print(f"  rise={tuned_score['rise_time_s']}  "
                  f"ripple L/R={tuned_score['ripple_l']:.1f}/{tuned_score['ripple_r']:.1f}  "
                  f"max|vL-vR|={tuned_score['max_split']:.1f}  taper_ok={tuned_score['taper_ok']}")

            # Keep the trial gains only if they clearly help (strictly
            # faster rise AND no worse ripple) -- otherwise revert to the
            # shipped all-zero default (Open Question 4's conservative
            # posture, ticket 004: the mechanism ships inert until bench
            # evidence justifies a nonzero default).
            base_rise = base_score["rise_time_s"]
            tuned_rise = tuned_score["rise_time_s"]
            keep_tuned = (
                tuned_rise is not None
                and (base_rise is None or tuned_rise < base_rise)
                and tuned_score["ripple_l"] <= base_score["ripple_l"] + 3.0
                and tuned_score["ripple_r"] <= base_score["ripple_r"] + 3.0
            )
            print(f"\n  DECISION: {'KEEP tuned Stage B gains' if keep_tuned else 'REVERT to all-zero (shipped default)'}")
            if keep_tuned:
                kept_gains = dict(STAGE_B_TRIAL_GAINS)
            else:
                revert_confirmed = _revert_stage_b(proto, "decision: revert")

        # --- CSV / chart ---
        _write_csv(csv_path, sink.frames)
        _write_chart(png_path, base_run, tuned_run)

        # --- Stage C bias convergence over the whole session ---
        biases = [(t, f.bias_left, f.bias_right) for t, f in sink.frames if f.bias_left is not None]
        if biases:
            t0b = biases[0][0]
            print("\n=== Stage C bias convergence (whole session) ===")
            print(f"  start (t=0.0s): L={biases[0][1]:.2f}  R={biases[0][2]:.2f}")
            print(f"  end   (t={biases[-1][0] - t0b:.1f}s): L={biases[-1][1]:.2f}  R={biases[-1][2]:.2f}")
            print("  clamp: +-23.8 mm/s (tovez wheel_bias_max)")

        print(f"\n{STAND_UNMEASURABLE}")
        print(f"\n  CSV: {csv_path}")
        print(f"  PNG: {png_path}")
        if kept_gains is not None:
            print(f"  Stage B gains KEPT live on the robot (persisted to flash): {kept_gains}")
        elif revert_confirmed is False:
            print("  Stage B gains: revert to all-zero FAILED to confirm -- robot may still be "
                  "running trial gains (see ERROR above)")
        else:
            print("  Stage B gains: reverted to all-zero (shipped default) before exit")

    finally:
        if sink is not None:
            sink.stop()
        if proto is not None:
            try:
                proto.estop()
            except Exception as exc:
                print(f"  WARN: estop() failed during cleanup: {exc}")
            try:
                proto.tlmOff()
            except Exception as exc:
                print(f"  WARN: tlmOff() failed during cleanup: {exc}")
        if conn.is_open:
            conn.disconnect()

    return 0


if __name__ == "__main__":
    sys.exit(main())
