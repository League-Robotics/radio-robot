#!/usr/bin/env python3
"""duty_sweep.py -- the Phase-0 population duty-sweep protocol: measure the
wheel plant (duty -> steady-state speed) over the FULL duty range, per motor,
per direction, plus a simultaneous-both-wheels grid, and derive population
constants (mean map, adaptation bounds, breakaway band).

`App::Drive` is OPEN LOOP (`src/firm/app/drive.h:14`): it converts a commanded
wheel speed to duty via `dutyPerSpeed` and writes it, with no feedback. So
`dutyPerSpeed` IS the plant model, and when it is wrong the robot simply runs at
the wrong speed -- measured 2026-07-31 at 35% of commanded. This tool measures
the real plant and reports the population constants
`wheel-speed-controller-moves-into-drive.md` Phase 0 calls for.

CORRECTIONS TO EARLIER VERSIONS OF THIS SCRIPT
-----------------------------------------------
1. **It drove through `move_wheels()` -- the PLANNER path.** `Motion::WheelTrim`
   runs a closed velocity loop there and actively corrects the exact error being
   measured, so the plant looked better than it is. This uses the `WHEELS`
   verb (`NezhaProtocol.wheels()`), which `robot_loop.cpp:262-280` routes
   straight to Drive after `planner_.estop()` -- genuinely open loop.
2. **Its duty axis was wrong by ~1.75x.** It assumed `speed = duty * 1370`.
   Drive's actual chain, from `drive.cpp:81-138`, is:

       command = (|desired| - intercept[wheel][dir]) / gain[wheel][dir]
       duty    = command * dutyPerSpeed
       duty    = copysign(outputDeadband, duty)  if 0 < |duty| < outputDeadband

3. **`dutyPerSpeed` is now BAKED IN FIRMWARE, not config-sourced -- a second,
   more subtle staleness bug this sprint fixes.** Since 2026-07-31
   (`App::Drive::kDutyPerSpeed`, drive.h), `boot_calibration.cpp`'s
   `installDriveCalibration()` calls
   `drive.setDutyPerSpeed(Drive::kDutyPerSpeed, Drive::kDutyPerSpeed)`
   UNCONDITIONALLY -- "MEASURED, NOT CONFIGURED" -- and deliberately ignores
   the robot JSON's `control.duty_per_speed_left/right` fields entirely (that
   comment names the exact issue this ticket's own `04-continuous-duty-per-
   speed-calibration.md` retires). An earlier version of this script (and the
   run that DERIVED `kDutyPerSpeed` in the first place) inverted the duty axis
   using the JSON field, which happened to be self-consistent ONLY because
   that measurement predated the C++ bake-in -- firmware was still
   config-driven that day. Today the JSON field (`0.00187325`) and the baked
   constant (`0.001182`) disagree by ~1.6x, so inverting off the JSON field now
   would silently under/over-drive the real duty by that same factor across
   the whole sweep. `KNOWN_DUTY_PER_SPEED` below is the fix: hardcode the
   value the firmware ACTUALLY uses, cited straight from `drive.h`, and cross-
   check it independently every run via a constant-free saturation reading
   (see next point). `wheel_gain_*`/`wheel_intercept_*` are NOT dead (still
   installed via `setWheelCorrection()`) and are still read from the JSON.
4. **Anchor on a constant-free measurement first.** A saturation reading
   (command a deliberately huge speed so duty clamps to 1.0 regardless of
   which constant was used to pick it, then just read the plateau) depends on
   no config constant and takes a few seconds. An earlier sweep the same day
   produced badly wrong numbers -- a claimed 28% L/R mismatch and a 0.24
   breakaway -- because it ran against firmware whose baked constants could
   not be read back (no firmware->host config read-back path exists). This
   script runs that check FIRST, for every wheel/direction and once for the
   simultaneous case, and prints it prominently for cross-checking against the
   graded sweep's own duty=~1.0 rung.

THE ACCEL/DECEL SUBTLETY
------------------------
`correctedCommand()` picks coefficients with `(|desired| > |previous|) ? accel :
decel`, where `previous` is the previous COMMANDED speed. Holding a rung means
`desired == previous` after the first cycle, so steady state uses the **DECEL**
pair. Using the accel pair here would bias every fit. Every fit/derived value in
this script is therefore a DECEL-pair measurement; the accel pair is not
characterized by this tool and stays at its identity default.

WHAT'S NEW THIS SPRINT (130-001, population duty-sweep protocol)
------------------------------------------------------------------
- Full duty range (default 0.04-1.0, was 0.04-0.60) per motor per direction,
  with a SEPARATE fit window (`--fit-from`/`--fit-duty-max`) that excludes the
  saturation/decline region 129-006 found starting ~duty 0.30 (left) / ~0.40
  (right) -- fitting the full range would bias the line toward that decline.
- A simultaneous-both-wheels grid (`--skip-simultaneous` to omit): both wheels
  driven together across the SAME duty grid as the single-wheel sweep, so
  single-vs-simultaneous is a same-duty, same-session comparison, not two
  different measurements stitched together.
- `--from-csv` to re-run the fit/population analysis/chart against an existing
  dataset without re-driving the robot.
- Population analysis: per-(wheel,direction) affine fits treated as population
  samples (4 lines with the currently-mounted robot's own two motors -- the
  ticket's own sanctioned fallback when a batch of spare motors is not
  available in one session; a real multi-unit campaign is future work, see
  clasi/issues/ for the follow-on), a parallel-lines verdict, and derived
  `biasMax`/`vMin`/breakaway-band constants.

    uv run python src/tests/bench/duty_sweep.py --port /dev/cu.usbmodem2121102
"""
from __future__ import annotations

import argparse
import csv
import json
import pathlib
import statistics
import sys
import time

_REPO = pathlib.Path(__file__).resolve().parents[3]

DEFAULT_PORT = "/dev/cu.usbmodem2121102"

# App::Drive arms each WHEELS command until `commandDeadline_ = now + duration`
# (drive.cpp:16), so the host must re-arm well inside the window or the deadman
# fires mid-rung and the wheel coasts -- which reads as a soft plant rather than
# a lost lease. Refresh at ~1/5 of the lease. See
# clasi/issues/testgui-unmanaged-drive-lease-expiry-and-terminal-pivot.md.
LEASE = 300.0        # [ms]
REFRESH = 0.060      # [s]

SETTLE = 1.0         # [s] discarded while the wheel reaches the rung
DWELL = 1.0          # [s] sampled window at steady state
REST = 0.5           # [s] full stop between rungs, so each starts from stuck
MOTION = 5.0         # [mm/s] below this the wheel is considered not turning

# The measured plant inverse the FIRMWARE ACTUALLY USES -- see point 3 above.
# MUST track src/firm/app/drive.h's `Drive::kDutyPerSpeed`; there is no
# firmware->host config read-back path, so this is a hand-kept mirror, not a
# read. If drive.h's constant ever changes, update this too or the whole duty
# axis silently goes stale again exactly as it did this session.
KNOWN_DUTY_PER_SPEED = 0.001182  # [duty/(mm/s)] == App::Drive::kDutyPerSpeed

# A deliberately saturating command: any value this large clamps duty to 1.0
# regardless of which dutyPerSpeed was used to pick it, so this reading needs
# no config constant at all (point 4 above).
SATURATION_SPEED = 1500.0  # [mm/s]


def _load_control(robot: str) -> dict:
    """Drive's calibration, straight from the robot JSON.

    Raw JSON deliberately, NOT pydantic: `ControlConfig` silently drops keys it
    does not declare (clasi/issues/misc-changes-from-2026-07-31-session.md item
    4), and a silently-missing constant here would corrupt the whole x-axis.

    `duty_per_speed_left/right` are read ONLY for the printed drift comparison
    against `KNOWN_DUTY_PER_SPEED` -- `boot_calibration.cpp` ignores them, so
    they are NOT used to compute the duty axis (see module docstring point 3).
    `wheel_gain_*`/`wheel_intercept_*` ARE still installed by firmware and are
    used below.
    """
    path = _REPO / "data" / "robots" / f"{robot}.json"
    control = json.loads(path.read_text())["control"]
    if control.get("output_deadband") is None:
        sys.exit(f"{path.name} missing required control key: output_deadband")
    return control


def _speed_for_duty(duty: float, direction: int, wheel: str, c: dict) -> float:  # -> [mm/s]
    """Invert Drive's chain: the signed commanded speed that yields `duty` in
    the given direction. Uses KNOWN_DUTY_PER_SPEED (the firmware's actual
    baked constant), not the robot JSON's dead field -- see module docstring.
    """
    gain = c.get(f"wheel_gain_{wheel}_decel") or 1.0
    intercept = c.get(f"wheel_intercept_{wheel}_decel") or 0.0
    return direction * (duty / KNOWN_DUTY_PER_SPEED * gain + intercept)


def fit_line(points):
    """Least-squares `speed = m*duty + b` over `points`; returns (m, b).

    Pure function, unit-tested against synthetic data in
    src/tests/unit/test_duty_sweep_population.py.
    """
    n = len(points)
    mx = sum(p[0] for p in points) / n
    my = sum(p[1] for p in points) / n
    den = sum((x - mx) ** 2 for x, _ in points)
    m = sum((x - mx) * (y - my) for x, y in points) / den
    return m, my - m * mx


def population_spread(values):
    """Mean, max-deviation envelope, and stdev of `values`.

    n is expected to be small (population campaigns start at 2-4 samples --
    this ticket's own currently-mounted-robot fallback) so the envelope (max
    deviation from the mean) is reported as the primary, more defensible
    figure; stdev is reported alongside for when a larger population exists.
    """
    n = len(values)
    mean = sum(values) / n
    envelope = max(abs(v - mean) for v in values) if n else 0.0
    variance = sum((v - mean) ** 2 for v in values) / n if n > 1 else 0.0
    return mean, envelope, variance ** 0.5


def parallel_lines_verdict(fits, duty_lo: float, duty_hi: float,
                            growth_threshold: float = 0.35):
    """Stakeholder hypothesis test: do the population's duty-speed lines fan
    out with duty (slope-dominated variation) or stay roughly the same
    distance apart across the fit window (intercept-dominated variation)?

    `fits`: iterable of (gain, offset) pairs, i.e. `speed = gain*duty + offset`.
    Compares the spread across lines at the LOW end of the fit window to the
    spread at the HIGH end. If the high-end spread is not much bigger than the
    low-end spread (parallel lines just ride a vertical offset), variation is
    intercept-dominated -- confirms the stakeholder hypothesis and licenses
    intercept-only (bias) adaptation (wheel-speed-controller-moves-into-
    drive.md Stage C). If the spread grows substantially with duty, the lines
    fan -- slope varies across the population and intercept-only adaptation
    would not track it.

    Returns (verdict, lo_spread, hi_spread, growth) where growth is the
    fraction of the high-end spread NOT already present at the low end.
    """
    fits = list(fits)
    lo_vals = [gain * duty_lo + offset for gain, offset in fits]
    hi_vals = [gain * duty_hi + offset for gain, offset in fits]
    lo_spread = max(lo_vals) - min(lo_vals)
    hi_spread = max(hi_vals) - min(hi_vals)
    growth = (hi_spread - lo_spread) / hi_spread if hi_spread > 1e-9 else 0.0
    verdict = ("intercept-dominated (parallel)" if growth < growth_threshold
               else "slope-dominated (fanned)")
    return verdict, lo_spread, hi_spread, growth


def bias_max_from_offsets(offsets) -> float:  # -> [mm/s]
    """`biasMax`: the trim/adaptation authority bound -- "how far a real motor
    sits from the average" (wheel-speed-controller-moves-into-drive.md Phase
    0), taken as the population's offset envelope (see population_spread).
    """
    _, envelope, _ = population_spread(list(offsets))
    return envelope


def v_min_from_breakaway(breakaway_duties, mean_gain: float) -> float:  # -> [mm/s]
    """`vMin`: the speed floor below which some wheel/direction in the
    population will not reliably move -- the WORST (highest) breakaway duty
    observed, converted to mm/s via the population mean plant gain.
    """
    worst = max(breakaway_duties)
    return worst * mean_gain


def map_gain_intercept(gain: float, offset: float, mean_gain: float):
    """Candidate Stage-A `wheel_gain_*_decel`/`wheel_intercept_*_decel` values
    for THIS wheel/direction, if the population mean gain (1/mean_gain) were
    adopted as the shared `dutyPerSpeed`: solves for the affine correction on
    v_cmd that makes the corrected chain reproduce this line's own measured
    behavior. Report-only -- see main()'s printed caveat about the
    `_wheel_correction_note` in the robot JSON (refit only against
    camera-measured travel, not bench/encoder data) before ever applying these.
    """
    map_gain = mean_gain / gain if gain else 1.0
    map_intercept = -map_gain * offset
    return map_gain, map_intercept


def _hold(proto, v_left, v_right, settle=SETTLE, dwell=DWELL, retries=1):
    """Drive one rung open loop; return velocity samples from steady state.

    A whole-window telemetry dropout (zero frames across the entire
    settle+dwell window) is rare but has been observed on a live USB session
    -- retry the hold once before giving up, so one dead window does not
    abort a multi-minute population sweep that was otherwise clean.
    """
    for attempt in range(retries + 1):
        samples = []
        t0 = time.monotonic()
        while time.monotonic() - t0 < settle + dwell:
            proto.wheels(v_left, v_right, LEASE)
            for f in proto.read_binary_tlm_frames(int(REFRESH * 1000)) or []:
                if f.vel and time.monotonic() - t0 >= settle:
                    samples.append(f.vel)
        if samples:
            return samples
        if attempt < retries:
            print("    (no telemetry this window -- retrying once)")
    return samples


def _median_abs(samples, idx) -> float:
    return statistics.median(abs(s[idx]) for s in samples)


def _saturation_read(proto, wheel: str, direction: int,
                      speed: float = SATURATION_SPEED) -> float:
    """Constant-free plateau reading: command a deliberately huge speed (duty
    clamps to 1.0 no matter which constant picked it) and read steady state.
    """
    pair = (direction * speed, 0.0) if wheel == "left" else (0.0, direction * speed)
    samples = _hold(proto, *pair)
    proto.estop()
    time.sleep(REST)
    idx = 0 if wheel == "left" else 1
    return _median_abs(samples, idx) if samples else float("nan")


def _saturation_read_simultaneous(proto, direction: int,
                                   speed: float = SATURATION_SPEED):
    samples = _hold(proto, direction * speed, direction * speed)
    proto.estop()
    time.sleep(REST)
    if not samples:
        return float("nan"), float("nan")
    return _median_abs(samples, 0), _median_abs(samples, 1)


def _single_wheel_sweep(proto, c, duties):
    """Full duty range, per motor, per direction -- one wheel driven at a
    time, the other held at zero. Returns (rows, aborted):
    rows are (mode, wheel, direction, duty, commanded, measured, n) tuples
    collected so far -- ALWAYS returned, even on an abort, so a mid-sweep
    hardware fault does not throw away otherwise-good data (see the
    `--from-csv` / 130-001 completion notes: a live session hit a hardware
    wedge -- a dead-silent robot after 45 good rungs -- and this is the fix
    that would have kept the CSV write instead of losing everything).
    """
    rows = []
    print(f"\n{'wheel':>5} {'dir':>4} {'duty':>6} {'cmd':>7} {'measured':>9}")
    print("-" * 36)
    for wheel in ("left", "right"):
        for direction in (+1, -1):
            for duty in duties:
                speed = _speed_for_duty(duty, direction, wheel, c)
                pair = (speed, 0.0) if wheel == "left" else (0.0, speed)
                samples = _hold(proto, *pair)
                proto.estop()
                time.sleep(REST)
                if not samples:
                    print("  no telemetry -- aborting sweep, keeping rows "
                          "collected so far")
                    return rows, True
                idx = 0 if wheel == "left" else 1
                meas = _median_abs(samples, idx)
                print(f"{wheel:>5} {'fwd' if direction > 0 else 'rev':>4} "
                      f"{duty:6.3f} {speed:7.0f} {meas:9.1f}")
                rows.append(("single", wheel, direction, duty, speed, meas,
                             len(samples)))
    return rows, False


def _simultaneous_sweep(proto, c, duties):
    """Both wheels driven TOGETHER, same direction, across the SAME duty grid
    as the single-wheel sweep -- so a same-duty comparison is a same-session,
    same-rung comparison, not two runs stitched together. One row per wheel
    per rung (wheel field says which side is being reported). Returns
    (rows, aborted) -- see _single_wheel_sweep's own doc comment.
    """
    rows = []
    print(f"\n{'wheel':>5} {'dir':>4} {'duty':>6} {'cmd':>7} {'measured':>9}"
          "  (SIMULTANEOUS -- both wheels driven together)")
    print("-" * 36)
    for direction in (+1, -1):
        for duty in duties:
            speed_l = _speed_for_duty(duty, direction, "left", c)
            speed_r = _speed_for_duty(duty, direction, "right", c)
            samples = _hold(proto, speed_l, speed_r)
            proto.estop()
            time.sleep(REST)
            if not samples:
                print("  no telemetry -- aborting sweep, keeping rows "
                      "collected so far")
                return rows, True
            meas_l = _median_abs(samples, 0)
            meas_r = _median_abs(samples, 1)
            print(f"{'both':>5} {'fwd' if direction > 0 else 'rev':>4} "
                  f"{duty:6.3f} {speed_l:7.0f} L={meas_l:6.1f} R={meas_r:6.1f}")
            rows.append(("simultaneous", "left", direction, duty, speed_l,
                         meas_l, len(samples)))
            rows.append(("simultaneous", "right", direction, duty, speed_r,
                         meas_r, len(samples)))
    return rows, False


def _write_csv(path, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["mode", "wheel", "direction", "duty", "commanded",
                    "measured", "n"])
        w.writerows(rows)


def _load_csv(path):
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            rows.append((r["mode"], r["wheel"], int(r["direction"]),
                         float(r["duty"]), float(r["commanded"]),
                         float(r["measured"]), int(r["n"])))
    return rows


def _fits_by_wheel_direction(rows, fit_from, fit_duty_max):
    """{(wheel, direction): (gain, offset)} from mode=='single' rows in the
    fit window. Four lines with today's two-motor fallback.
    """
    fits = {}
    for wheel in ("left", "right"):
        for direction in (+1, -1):
            pts = [(r[3], r[5]) for r in rows
                   if r[0] == "single" and r[1] == wheel and r[2] == direction
                   and fit_from <= r[3] <= fit_duty_max and r[5] > MOTION]
            if len(pts) >= 3:
                fits[(wheel, direction)] = fit_line(pts)
    return fits


def _fits_by_wheel(rows, fit_from, fit_duty_max):
    """{wheel: (gain, offset)}, both directions pooled -- preserves the
    original per-wheel headline (the L853.6/R837.8 figures this ticket
    re-verifies) alongside the finer-grained per-direction population fits.
    """
    fits = {}
    for wheel in ("left", "right"):
        pts = [(r[3], r[5]) for r in rows
               if r[0] == "single" and r[1] == wheel
               and fit_from <= r[3] <= fit_duty_max and r[5] > MOTION]
        if len(pts) >= 3:
            fits[wheel] = fit_line(pts)
    return fits


def _breakaway_edges(rows):
    """{(wheel, direction): lowest duty with sustained motion}, mode=='single'."""
    edges = {}
    for wheel in ("left", "right"):
        for direction in (+1, -1):
            wheel_rows = [r for r in rows if r[0] == "single" and r[1] == wheel
                          and r[2] == direction]
            edge = next((r[3] for r in wheel_rows if r[5] > MOTION), None)
            if edge is not None:
                edges[(wheel, direction)] = edge
    return edges


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--robot", default="tovez")
    ap.add_argument("--duty-min", type=float, default=0.04)
    ap.add_argument("--duty-max", type=float, default=1.0,
                    help="full duty range by default -- 129-006 found "
                         "saturation/decline that a 0.60 ceiling never saw")
    ap.add_argument("--rungs", type=int, default=20)
    ap.add_argument("--fit-from", type=float, default=0.12,
                    help="ignore rungs below this duty when fitting the line "
                         "-- the deadband region is not linear")
    ap.add_argument("--fit-duty-max", type=float, default=0.50,
                    help="ignore rungs above this duty when fitting the line "
                         "-- 129-006's own 0.60-ceiling sweep estimated "
                         "saturation onset at ~0.30 (left) / ~0.40 (right), "
                         "but this ticket's full-range (0.04-1.0) sweep on "
                         "2026-08-01 found the left wheel stays genuinely "
                         "linear (local slope ~1000-1200 mm/s per duty, "
                         "steady) all the way to duty~0.55 before a sharp "
                         "one-rung collapse into saturation -- 129-006's "
                         "estimate was itself session/battery-specific, not a "
                         "fixed ceiling. 0.28 (this flag's PRIOR default) "
                         "only spans 3 rungs near breakaway and understates "
                         "the true bulk-linear-region slope; 0.50 is the "
                         "data-driven choice, safely below the observed peak "
                         "on both directions measured so far. Revisit once "
                         "the right wheel's own full range is captured (this "
                         "session's own right-wheel data stops at 0.242 -- "
                         "see the completion notes).")
    ap.add_argument("--saturation-speed", type=float, default=SATURATION_SPEED)
    ap.add_argument("--skip-single", action="store_true",
                    help="skip the per-wheel single-motor sweep")
    ap.add_argument("--skip-simultaneous", action="store_true",
                    help="skip the both-wheels-together grid")
    ap.add_argument("--from-csv", default=None,
                    help="re-run fit/population analysis + chart against an "
                         "existing CSV instead of driving the robot")
    ap.add_argument("--battery-note", default=(
        "battery used as found this bench session -- NOT freshly charged or "
        "independently verified; the robot exposes no pack-voltage telemetry "
        "(wheel-speed-controller-moves-into-drive.md Supply-sag note), so "
        "charge state cannot be measured, only recorded qualitatively. The "
        "freshly-charged-vs-depleted comparison is flagged as a stakeholder-"
        "scheduled follow-on bench session per this ticket's own fallback."),
        help="freeform note on observed battery state, embedded in the report")
    args = ap.parse_args()

    c = _load_control(args.robot)
    duty_per_speed_json = c.get("duty_per_speed_left")
    print(f"{args.robot}: KNOWN_DUTY_PER_SPEED (firmware, drive.h) = "
          f"{KNOWN_DUTY_PER_SPEED:.6f}"
          + (f"  [JSON control.duty_per_speed_left = {duty_per_speed_json:.6f} "
             f"-- DEAD, ignored by boot_calibration.cpp, shown for drift only]"
             if duty_per_speed_json else "")
          + f"  deadband={c['output_deadband']}")

    step = (args.duty_max - args.duty_min) / (args.rungs - 1)
    duties = [round(args.duty_min + i * step, 4) for i in range(args.rungs)]

    rows = []
    if args.from_csv:
        rows = _load_csv(pathlib.Path(args.from_csv))
        print(f"\nloaded {len(rows)} rows from {args.from_csv} "
              "(no hardware driven)")
    else:
        from robot_radio.io.serial_conn import SerialConnection
        from robot_radio.robot.protocol import NezhaProtocol

        conn = SerialConnection(port=args.port)
        conn.connect()
        proto = NezhaProtocol(conn)
        try:
            print("\nSATURATION (constant-free -- needs no config, cross-"
                  "checks the graded sweep's own duty~1.0 rung):")
            for wheel in ("left", "right"):
                for direction in (+1, -1):
                    meas = _saturation_read(proto, wheel, direction,
                                            args.saturation_speed)
                    print(f"  {wheel:<5} {'fwd' if direction > 0 else 'rev':<4}"
                          f" (single):       {meas:7.1f} mm/s")
            for direction in (+1, -1):
                meas_l, meas_r = _saturation_read_simultaneous(
                    proto, direction, args.saturation_speed)
                print(f"  both  {'fwd' if direction > 0 else 'rev':<4}"
                      f" (simultaneous): L={meas_l:7.1f}  R={meas_r:7.1f} mm/s")

            aborted = False
            if not args.skip_single:
                single_rows, aborted = _single_wheel_sweep(proto, c, duties)
                rows.extend(single_rows)
            if not aborted and not args.skip_simultaneous:
                sim_rows, aborted = _simultaneous_sweep(proto, c, duties)
                rows.extend(sim_rows)
        finally:
            try:
                proto.estop()
            except Exception:
                pass
            conn.disconnect()

        out_csv = _REPO / "src" / "tests" / "bench" / "duty_sweep.csv"
        _write_csv(out_csv, rows)
        print(f"\nwrote {out_csv} ({len(rows)} rows)")
        if aborted:
            print("SWEEP WAS CUT SHORT by a telemetry/hardware fault -- the "
                  "CSV above holds only the rungs collected before the abort. "
                  "Re-run once the robot is confirmed responsive (PING/"
                  "telemetry) again.")

    # --- dead zone / breakaway edges ----------------------------------------
    print("\nDEAD ZONE / BREAKAWAY (lowest duty with sustained motion, "
          "single-wheel):")
    edges = _breakaway_edges(rows)
    for wheel in ("left", "right"):
        for direction in (+1, -1):
            edge = edges.get((wheel, direction))
            print(f"  {wheel:<5} {'fwd' if direction > 0 else 'rev':<4}: {edge}")

    # --- plant gain: per-wheel headline (both directions pooled) -----------
    print(f"\nPLANT GAIN, per wheel (fit over duty in "
          f"[{args.fit_from}, {args.fit_duty_max}], both directions pooled):")
    fits_wheel = _fits_by_wheel(rows, args.fit_from, args.fit_duty_max)
    for wheel, (m, b) in fits_wheel.items():
        print(f"  {wheel:<5} speed = {m:7.1f}*duty {b:+7.1f}   "
              f"=> dutyPerSpeed = {1.0 / m:.6f}")

    if len(fits_wheel) == 2:
        gL, gR = fits_wheel["left"][0], fits_wheel["right"][0]
        mean = (gL + gR) / 2.0
        spread = abs(gL - gR)
        print(f"\nL/R spread: {spread:.1f} mm/s per unit duty "
              f"({spread / mean * 100:.1f}%)")
        print(f"RECOMMENDED shared dutyPerSpeed = {1.0 / mean:.6f}  "
              f"(current baked kDutyPerSpeed = {KNOWN_DUTY_PER_SPEED:.6f})")

    # --- per-wheel-per-direction fits: the population sample ---------------
    print(f"\nPLANT GAIN, per wheel PER DIRECTION (the population sample for "
          "the analysis below):")
    fits_dir = _fits_by_wheel_direction(rows, args.fit_from, args.fit_duty_max)
    for (wheel, direction), (m, b) in fits_dir.items():
        print(f"  {wheel:<5} {'fwd' if direction > 0 else 'rev':<4} "
              f"speed = {m:7.1f}*duty {b:+7.1f}")

    # --- saturation / decline shape over the FULL range ---------------------
    print("\nSATURATION / DECLINE over the full duty range (single-wheel):")
    for wheel in ("left", "right"):
        for direction in (+1, -1):
            wr = [r for r in rows if r[0] == "single" and r[1] == wheel
                  and r[2] == direction]
            if not wr:
                continue
            peak_row = max(wr, key=lambda r: r[5])
            top_row = max(wr, key=lambda r: r[3])
            decline_pct = (0.0 if peak_row[5] <= 0 else
                           (peak_row[5] - top_row[5]) / peak_row[5] * 100.0)
            print(f"  {wheel:<5} {'fwd' if direction > 0 else 'rev':<4} "
                  f"peak {peak_row[5]:.1f} mm/s @ duty={peak_row[3]:.3f}; "
                  f"at duty={top_row[3]:.3f} measured {top_row[5]:.1f} mm/s "
                  f"({decline_pct:+.1f}% vs peak)")

    # --- simultaneous vs single-wheel comparison (129-006 finding) ----------
    print("\nSIMULTANEOUS vs SINGLE-WHEEL (same duty rung, same session):")
    single_by_key = {(r[1], r[2], r[3]): r[5] for r in rows if r[0] == "single"}
    sim_rows_present = [r for r in rows if r[0] == "simultaneous"]
    deltas = []
    for r in sim_rows_present:
        key = (r[1], r[2], r[3])
        single_meas = single_by_key.get(key)
        if single_meas is None:
            continue
        delta = r[5] - single_meas
        deltas.append(delta)
        print(f"  {r[1]:<5} {'fwd' if r[2] > 0 else 'rev':<4} duty={r[3]:.3f}: "
              f"single={single_meas:7.1f}  simultaneous={r[5]:7.1f}  "
              f"delta={delta:+7.1f} mm/s")
    if deltas:
        mean_delta = sum(deltas) / len(deltas)
        worst = min(deltas)
        print(f"\n  mean delta {mean_delta:+.1f} mm/s, worst (most negative) "
              f"{worst:+.1f} mm/s over {len(deltas)} matched rungs.")
        verdict = ("REPRODUCES 129-006: combined load underperforms "
                   "single-wheel load" if mean_delta < -MOTION else
                   "no significant combined-load degradation this session")
        print(f"  verdict: {verdict}")

    # --- parallel-lines test -------------------------------------------------
    print("\nPARALLEL-LINES TEST (stakeholder hypothesis: intercept, not "
          "slope, is what varies across the population):")
    if len(fits_dir) >= 2:
        verdict, lo_spread, hi_spread, growth = parallel_lines_verdict(
            fits_dir.values(), args.fit_from, args.fit_duty_max)
        gains = [g for g, _ in fits_dir.values()]
        offsets = [b for _, b in fits_dir.values()]
        gain_mean, gain_env, gain_sd = population_spread(gains)
        off_mean, off_env, off_sd = population_spread(offsets)
        print(f"  spread @ duty={args.fit_from}: {lo_spread:.1f} mm/s   "
              f"spread @ duty={args.fit_duty_max}: {hi_spread:.1f} mm/s   "
              f"growth={growth * 100:.0f}%")
        print(f"  gain  population: mean={gain_mean:.1f}  envelope=+-"
              f"{gain_env:.1f} ({gain_env / gain_mean * 100:.1f}%)  "
              f"stdev={gain_sd:.1f}")
        print(f"  offset population: mean={off_mean:.1f}  envelope=+-"
              f"{off_env:.1f} mm/s  stdev={off_sd:.1f}")
        print(f"  VERDICT: {verdict}")

        # --- derived population constants -----------------------------------
        print("\nDERIVED POPULATION CONSTANTS (generated values the boot "
              "config would bake -- n={} samples, this robot's own two motors,"
              " see fallback note):".format(len(fits_dir)))
        bias_max = bias_max_from_offsets(offsets)
        breakaway = _breakaway_edges(rows)
        if breakaway:
            v_min = v_min_from_breakaway(breakaway.values(), gain_mean)
            print(f"  vMin (speed floor)      = {v_min:.1f} mm/s  "
                  f"(worst breakaway duty {max(breakaway.values()):.3f} x "
                  f"mean gain {gain_mean:.1f})")
            print(f"  breakaway band           = "
                  f"[{min(breakaway.values()):.3f}, "
                  f"{max(breakaway.values()):.3f}] duty  "
                  f"({min(breakaway.values()) * gain_mean:.1f}-"
                  f"{max(breakaway.values()) * gain_mean:.1f} mm/s)")
        print(f"  biasMax (trim authority) = +-{bias_max:.1f} mm/s  "
              "(offset population envelope)")

        print("\n  Candidate Stage-A map gain/intercept per wheel/direction "
              "(NOT applied to the robot JSON -- report only. The "
              "_wheel_correction_note in data/robots/tovez.json explicitly "
              "warns against refitting this pair from bench/encoder data; "
              "these are ticket 004 design inputs, pending a camera-measured "
              "refit before anything is actually written):")
        for (wheel, direction), (gain, offset) in fits_dir.items():
            map_gain, map_intercept = map_gain_intercept(gain, offset, gain_mean)
            print(f"    {wheel:<5} {'fwd' if direction > 0 else 'rev':<4} "
                  f"decel: gain={map_gain:.4f}  intercept={map_intercept:+.1f} "
                  "mm/s")
    else:
        print("  too few per-direction fits to run the test")

    # --- 129-006 power-delivery-ceiling verdict -----------------------------
    print("\n129-006 POWER-DELIVERY-CEILING VERDICT:")
    print(f"  battery: {args.battery_note}")
    print("  Within-session evidence only (no fresh-battery re-run performed "
          "this session -- see battery note above). The saturation readings "
          "above and the simultaneous-vs-single comparison are this session's "
          "own evidence; a hard-limit-vs-battery-artifact verdict that also "
          "controls for charge state is flagged as the same stakeholder-"
          "scheduled follow-on bench session as the population campaign.")

    # --- chart --------------------------------------------------------------
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("(matplotlib unavailable -- skipping chart)")
        return 0

    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(16, 6))
    styles = {("left", 1): ("#1f77b4", "-"), ("left", -1): ("#1f77b4", "--"),
              ("right", 1): ("#2ca02c", "-"), ("right", -1): ("#2ca02c", "--")}
    for (wheel, direction), (colr, ls) in styles.items():
        pts = [(r[3], r[5]) for r in rows
               if r[0] == "single" and r[1] == wheel and r[2] == direction]
        pts.sort()
        ax.plot([p[0] for p in pts], [p[1] for p in pts], color=colr, ls=ls,
                marker="o", ms=3,
                label=f"{wheel} {'fwd' if direction > 0 else 'rev'}")
    for (wheel, direction), (m, b) in fits_dir.items():
        colr, ls = styles[(wheel, direction)]
        xs = [args.fit_from, args.fit_duty_max]
        ax.plot(xs, [m * x + b for x in xs], color=colr, lw=0.8, ls=":")
    ax.set_xlabel("duty [-]")
    ax.set_ylabel("steady-state speed [mm/s]")
    ax.set_title("Single-wheel duty sweep -- open loop (WHEELS verb)")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    for wheel, colr in (("left", "#1f77b4"), ("right", "#2ca02c")):
        for direction, ls in ((1, "-"), (-1, "--")):
            single_pts = sorted((r[3], r[5]) for r in rows
                                if r[0] == "single" and r[1] == wheel
                                and r[2] == direction)
            sim_pts = sorted((r[3], r[5]) for r in rows
                             if r[0] == "simultaneous" and r[1] == wheel
                             and r[2] == direction)
            if single_pts:
                ax2.plot([p[0] for p in single_pts], [p[1] for p in single_pts],
                         color=colr, ls=ls, marker="o", ms=3,
                         label=f"{wheel} {'fwd' if direction > 0 else 'rev'} "
                               "single")
            if sim_pts:
                ax2.plot([p[0] for p in sim_pts], [p[1] for p in sim_pts],
                         color=colr, ls=ls, marker="x", ms=5, alpha=0.5,
                         label=f"{wheel} {'fwd' if direction > 0 else 'rev'} "
                               "simultaneous")
    ax2.set_xlabel("duty [-]")
    ax2.set_ylabel("steady-state speed [mm/s]")
    ax2.set_title("Simultaneous (x) vs single-wheel (o) -- same duty grid")
    ax2.legend(fontsize=7)
    ax2.grid(True, alpha=0.3)

    fig.tight_layout()
    out_png = _REPO / "src" / "tests" / "bench" / "duty_sweep.png"
    fig.savefig(out_png, dpi=130)
    print(f"\nwrote {out_png}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
