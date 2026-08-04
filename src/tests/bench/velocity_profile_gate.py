#!/usr/bin/env python3
"""velocity_profile_gate.py -- the standing distance-fidelity acceptance gate.

**The question this answers: when we ask the drive layer for a velocity
profile, how much of the promised distance does each wheel actually
deliver?** Not "did it track the velocity" -- velocity error that
integrates to zero is free. The acceptance number is the ratio

    delivered distance (encoders, after all motion settles)
    ------------------------------------------------------
    commanded distance (the analytic area under the velocity profile)

reported per wheel, per profile, as a percentage. 100% is perfect. The
reference failure this catches is a loop that overshoots the stop and
coasts past its promise (a stock-Ki board measured 125.6% on exactly this
test) or one that lands short.

Two profiles, deliberately **the same total time and the same area**, so
the only thing that differs is the shape of the demand:

    trapezoid   3.0 s   0.75 s ramp up / 1.5 s plateau / 0.75 s ramp down
                        peak 200 mm/s   ->  area 200 * (3.0 - 0.75) = 450 mm
    square      3.0 s   instantaneous on, instantaneous off
                        peak 450 / 3.0 = 150 mm/s  ->  area 450 mm

The trapezoid is designed first and the square is sized to match its
area at the same duration (hence the lower peak) -- so both panels of the
chart carry the SAME commanded distance and the two ratios are directly
comparable. `--peak`/`--duration`/`--ramp` move all three together and
the relationship is preserved.

Both wheels are commanded identically and reported separately; a
left/right split is a real finding (wheel imbalance), not chart noise.

Three regimes, one script (`src/tests/CLAUDE.md`'s domain split):

    # sim -- no hardware, the firmware host build + SimPlant
    uv run python src/tests/bench/velocity_profile_gate.py --sim

    # bench -- robot on the stand, wheels off the ground, direct USB
    uv run python src/tests/bench/velocity_profile_gate.py --port /dev/cu.usbmodem2121202

    # field -- untethered through the relay dongle (ROLE=RADIOBRIDGE in
    #          `mbdeploy list`); SerialConnection does the !GO handshake
    uv run python src/tests/bench/velocity_profile_gate.py --port /dev/cu.usbmodemRELAY --profiles trapezoid

Prints a PASS/FAIL line per wheel per profile and **exits nonzero on any
failure**, so it can gate a sprint. Writes a 2x2 chart (velocity on top,
distance below) and a CSV of every sample.

FIELD CAVEAT: each profile drives ~450 mm FORWARD and the two profiles do
not return to start, so a full run needs ~1 m of clear travel. On the
playfield run ONE profile per placement (`--profiles trapezoid`) under the
standard camera-supervised procedure (`.claude/rules/playfield-testing.md`)
-- this script has no geofence of its own.

LIVE TUNING (133-003) -- the flags a wheel-controller session turns:

    --vmin --asteady --poserr --gain-left --gain-right     DBG verbs, RAM only
    --pid-kp --pid-ki --pid-kff --pid-imax --pid-kaw       WHEEL_CONTROL group

Everything is asserted **after connect**, because opening this port resets
the board -- a value set by any other process is already gone. Nothing is
taken on trust: a DBG verb must produce the firmware's own `... applied`
echo, and the WHEEL_CONTROL group is read back and compared field by field
(an ack is not evidence). Any unconfirmed push aborts the run rather than
reporting a distance against gains that were never set.

The two channels do NOT behave the same way afterwards, and the gate says
so on every run: the five `pid_*` fields are persisted to flash and survive
a reboot, while `v_min`/`a_steady`/`pos_err_max`/the duty gains are RAM-only
and revert on the next reset. Promote a keeper into
`data/robots/<robot>.json`.

The DBG channel exists only in a `ROBOT_DEBUG` image. If `--vmin` reports
"was not confirmed", the overwhelmingly likely cause is a plain `just
build`; rebuild with `uv run python build.py --clean --robot-debug`.

`--wheel {both,left,right}` drives a single-motor rig (the idle channel is
still commanded a hard zero and still reported, which is how cross-talk
would show up). `--attempts N` discards a truncated capture and reconnects:
a short capture reads as a short DISTANCE (77 mm of 450 mm, once) and is
otherwise indistinguishable from real under-travel.

Motion is issued as bare WHEELS commands (`NezhaProtocol.wheels()`) --
straight to `App::Drive`, bypassing the planner, no shaping and no
odometry stop condition -- because the profile IS the test. Letting the
planner shape it would measure the planner instead of the wheel-speed
controller. The host re-arms the command every `--tick` with a longer
`--lease`, so a single dropped command cannot stall the profile and a
dead host still stops the robot within one lease.
"""
from __future__ import annotations

import argparse
import csv
import pathlib
import sys
import time
from dataclasses import dataclass, field
from statistics import median, stdev

_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_BENCH_DIR = pathlib.Path(__file__).resolve().parent

# Categorical slots 1 and 2 of the validated default palette; `commanded`
# is a reference line in muted ink, never a third categorical hue.
COLOR_LEFT = "#2a78d6"
COLOR_RIGHT = "#eb6834"
COLOR_COMMANDED = "#52514e"
COLOR_GRID = "#d9d8d3"

DEFAULT_PORT = "/dev/cu.usbmodem2121202"  # tovez; ALWAYS confirm with `mbdeploy list`


# ---------------------------------------------------------------------------
# Profiles -- pure, no I/O, unit-testable without hardware or a sim build.
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Profile:
    """One commanded wheel-velocity profile: a symmetric trapezoid with
    `ramp` seconds on each end, degenerating to a square pulse at
    `ramp == 0`. Both wheels get the same profile."""

    name: str
    duration: float  # [s] total commanded window
    peak: float      # [mm/s] plateau speed
    ramp: float      # [s] rise time == fall time (0 for a square pulse)

    def __post_init__(self) -> None:
        if self.duration <= 0:
            raise ValueError(f"{self.name}: duration must be > 0, got {self.duration!r}")
        if self.ramp < 0 or 2 * self.ramp > self.duration:
            raise ValueError(
                f"{self.name}: ramp must be in [0, duration/2], got {self.ramp!r}")

    def velocity(self, t: float) -> float:  # [s] -> [mm/s]
        """Commanded speed at time `t`, measured from profile start. Zero
        outside [0, duration) -- the tail window after the profile ends is
        commanded zero, not simply un-commanded, so the drive layer is
        actively asked to stop rather than left to coast off a lease."""
        if t < 0.0 or t >= self.duration:
            return 0.0
        if self.ramp <= 0.0:
            return self.peak
        if t < self.ramp:
            return self.peak * (t / self.ramp)
        if t >= self.duration - self.ramp:
            return self.peak * ((self.duration - t) / self.ramp)
        return self.peak

    @property
    def distance(self) -> float:  # [mm]
        """Analytic area under the profile -- THE denominator of the
        acceptance ratio. A trapezoid loses exactly one ramp's worth of
        plateau time to its two half-ramps."""
        return self.peak * (self.duration - self.ramp)

    def distance_at(self, t: float) -> float:  # [s] -> [mm]
        """Commanded distance accumulated by time `t` (the running integral
        -- the dashed reference curve on the distance panels)."""
        if t <= 0.0:
            return 0.0
        if t >= self.duration:
            return self.distance
        if self.ramp <= 0.0:
            return self.peak * t
        if t < self.ramp:
            return 0.5 * self.peak * t * t / self.ramp
        rise = 0.5 * self.peak * self.ramp
        if t < self.duration - self.ramp:
            return rise + self.peak * (t - self.ramp)
        plateau = self.peak * (self.duration - 2 * self.ramp)
        remaining = self.duration - t
        fall = 0.5 * self.peak * (self.ramp - remaining * remaining / self.ramp)
        return rise + plateau + fall


def make_profiles(peak: float, duration: float, ramp: float) -> "list[Profile]":
    """The matched pair. The trapezoid is designed first; the square is then
    sized to deliver the SAME area over the SAME duration, which forces its
    peak below the trapezoid's. Returned trapezoid-first (design order); the
    chart plots square-first (shape order, simplest demand on the left)."""
    trapezoid = Profile("trapezoid", duration=duration, peak=peak, ramp=ramp)
    square = Profile("square", duration=duration,
                     peak=trapezoid.distance / duration, ramp=0.0)
    return [trapezoid, square]


# ---------------------------------------------------------------------------
# Results
# ---------------------------------------------------------------------------

@dataclass
class Sample:
    t: float           # [s] relative to profile start (robot clock, anchored)
    commanded: float   # [mm/s] speed asked of both wheels at that instant
    velocity_left: float   # [mm/s] measured
    velocity_right: float  # [mm/s] measured
    distance_left: float   # [mm] delivered since baseline
    distance_right: float  # [mm] delivered since baseline


@dataclass
class Run:
    profile: Profile
    samples: "list[Sample]" = field(default_factory=list)
    issued_distance: float = 0.0  # [mm] integral of what was actually SENT
    faults: "list[str]" = field(default_factory=list)
    notes: "list[str]" = field(default_factory=list)

    def _final(self, pick) -> float:
        return pick(self.samples[-1]) if self.samples else float("nan")

    @property
    def delivered_left(self) -> float:  # [mm] settled, end of tail
        return self._final(lambda s: s.distance_left)

    @property
    def delivered_right(self) -> float:  # [mm]
        return self._final(lambda s: s.distance_right)

    def delivered_at_command_end(self) -> "tuple[float, float]":  # [mm] x2
        """Distance on the books the instant the command reached zero --
        reported alongside the settled number so a run that lands late
        (coast) is distinguishable from one that lands long (overshoot)."""
        within = [s for s in self.samples if s.t <= self.profile.duration]
        if not within:
            return (float("nan"), float("nan"))
        return (within[-1].distance_left, within[-1].distance_right)

    @property
    def ratio_left(self) -> float:
        return self.delivered_left / self.profile.distance

    @property
    def ratio_right(self) -> float:
        return self.delivered_right / self.profile.distance

    def plateau_tracking(self) -> "tuple[float, float]":
        """Median measured speed per wheel over the commanded PLATEAU only,
        as a fraction of that plateau. Not gated -- reported because it
        separates the two ways a run can miss its distance: a loop that
        never reaches its setpoint (this number is below 1.0 and the
        shortfall is just its integral) versus one that tracks fine and
        then mismanages the ends (this number is ~1.0 but the ratio is
        not)."""
        plateau = self._plateau_samples()
        if not plateau or self.profile.peak <= 0:
            return (float("nan"), float("nan"))
        return (median(s.velocity_left for s in plateau) / self.profile.peak,
                median(s.velocity_right for s in plateau) / self.profile.peak)

    def _plateau_samples(self) -> "list[Sample]":
        return [s for s in self.samples
                if abs(s.commanded - self.profile.peak) < 1e-6]

    def plateau_ripple(self) -> "tuple[float, float]":
        """Sample standard deviation [mm/s] of measured speed per wheel over
        the commanded plateau -- the plateau's own roughness, independent of
        where its mean sits.

        Reported, never gated. It is here because 133-002's sigma-delta duty
        quantizer CHANGES this number by construction: the brick takes an
        integer percent, one count is ~8.5 mm/s at the shipped
        duty_per_speed, and carrying the rounding residual trades a
        stair-stepped plateau for a dithered one. Any historical distance
        figure -- specifically the issue's unexplained "square runs 3-4%
        long" -- was measured against the OLD ripple and cannot be
        re-attributed to a control cause until it has been re-measured
        against this one. Printing ripple beside the ratio is what makes
        that comparison possible instead of a matter of recollection.

        NaN when the plateau holds fewer than two samples (a square profile
        sampled sparsely, or a truncated capture).
        """
        plateau = self._plateau_samples()
        if len(plateau) < 2:
            return (float("nan"), float("nan"))
        return (stdev(s.velocity_left for s in plateau),
                stdev(s.velocity_right for s in plateau))

    def plateau_span(self) -> "tuple[float, float]":
        """Peak-to-peak measured speed [mm/s] per wheel over the plateau.
        Reported beside plateau_ripple() because the two answer different
        questions about the same window: a dithered plateau and a
        stair-stepped one can share a standard deviation while differing
        sharply in worst-case excursion, and it is the excursion an operator
        sees as audible/visible roughness on the stand."""
        plateau = self._plateau_samples()
        if not plateau:
            return (float("nan"), float("nan"))
        left = [s.velocity_left for s in plateau]
        right = [s.velocity_right for s in plateau]
        return (max(left) - min(left), max(right) - min(right))


def ratios(run: Run) -> "list[tuple[str, float]]":
    return [("left", run.ratio_left), ("right", run.ratio_right)]


def verdict(run: Run, tolerance: float) -> bool:
    """PASS iff every wheel's delivered/commanded ratio is within
    `tolerance` of 1.0 and no fault was observed during the run."""
    if run.faults:
        return False
    return all(r == r and abs(r - 1.0) <= tolerance for _, r in ratios(run))


# ---------------------------------------------------------------------------
# Transport-agnostic streaming
# ---------------------------------------------------------------------------

def _frame_positions(frame) -> "tuple[float, float] | None":
    """Per-wheel accumulated position [mm] off one telemetry frame, from the
    float-valued `EncoderReading`s rather than `TLMFrame.enc`'s rounded
    integer pair (450 mm over a 1-count grid is plenty, but the ratio is
    reported to a tenth of a percent -- take the precision that is free)."""
    left, right = frame.enc_left, frame.enc_right
    if left is None or right is None:
        return None
    return (float(left.position), float(right.position))


def _frame_velocities(frame) -> "tuple[float, float]":
    if frame.enc_left is not None and frame.enc_right is not None:
        return (float(frame.enc_left.velocity), float(frame.enc_right.velocity))
    if frame.vel is not None:
        return (float(frame.vel[0]), float(frame.vel[1]))
    return (0.0, 0.0)


# Bits that invalidate a distance measurement: a frozen encoder makes the
# delivered number a fiction, and a deficit bit means the controller ran out
# of authority -- either way the ratio must not be reported as a clean pass.
_GATING_FAULTS = (
    ("wheel_frozen_left", "fault_wheel_frozen_left"),
    ("wheel_frozen_right", "fault_wheel_frozen_right"),
    ("wheel_deficit_left", "fault_wheel_deficit_left"),
    ("wheel_deficit_right", "fault_wheel_deficit_right"),
    ("i2c_nak_timeout", "fault_i2c_nak_timeout"),
)

# Surfaced but NOT gated. `wedge_latch` (flags bit 7) is the RAW,
# unconditional stuck-encoder latch, which by its own firmware
# documentation "also fires on a healthy robot merely parked at rest" --
# and this gate parks the robot at rest for a whole baseline window before
# every profile, so it latches on every otherwise-clean run. The
# motion-qualified stall detector is `wheel_frozen_*` (bits 19/20), which
# IS gated above.
_INFORMATIONAL_FAULTS = (
    ("wedge_latch", "fault_wedge_latch"),
)

# Surfaced but NOT gated, and read INVERTED -- these flags are positive
# ("this wheel is connected"), so their ABSENCE is the condition worth
# reporting. 133-002 hazard, surfaced here because nothing else will
# surface it: `Drive::positionError()` no longer gates on encoder
# FRESHNESS. That was deliberate (the old accumulator's freshness gate
# silently and permanently dropped real distance out of the sum), but it
# means a wheel whose encoder has stopped reporting now reads as falling
# further and further behind, and can draw correction up to
# ki * pos_err_max -- bounded after that only by pid_i_max/pid_max, where
# it previously drew exactly nothing. It is inert on every robot as
# shipped (all of them carry pid_i_max = 0, so the I term is off
# entirely); ticket 004 is the session that makes it live, which is
# exactly why a disconnect must be VISIBLE the moment the gains stop being
# zero. See i_term_hazard() below for the combined reading.
_INFORMATIONAL_ABSENCES = (
    ("wheel_disconnected_left", "conn_left"),
    ("wheel_disconnected_right", "conn_right"),
)

# The subset of note labels that mean "this wheel's position reading may be
# stale or manufactured". i_term_hazard() keys on these.
_STALE_ENCODER_NOTES = ("wedge_latch", "wheel_disconnected_left",
                        "wheel_disconnected_right")


def _fault_names(frame) -> "list[str]":
    return [label for label, attr in _GATING_FAULTS if getattr(frame, attr, False)]


def _note_names(frame) -> "list[str]":
    present = [label for label, attr in _INFORMATIONAL_FAULTS
               if getattr(frame, attr, False)]
    # `getattr(..., True)` on the absence side: a frame shape that does not
    # carry the flag at all must read as "connected", never as a spurious
    # disconnect warning on every single frame.
    absent = [label for label, attr in _INFORMATIONAL_ABSENCES
              if not getattr(frame, attr, True)]
    return present + absent


def i_term_hazard(notes: "list[str]", tuning: "dict[str, float]") -> "str | None":
    """The one cross-cutting reading this gate can make that no single frame
    can: an encoder that stopped reporting AT THE SAME TIME as a live
    position-domain I term.

    Either alone is ordinary -- a parked wheel latches `wedge_latch` on
    every clean run, and a nonzero `pid_ki` is the whole point of ticket
    004. Together they are the 133-002 hazard above, and the resulting
    distance number is not a control measurement at all. Returns a warning
    string, or None when the combination did not occur.

    `tuning` is the WHEEL_CONTROL read-back (`_assert_tuning`'s own return
    value), so this reads what the robot CONFIRMED, never what the host
    asked for.
    """
    stale = [n for n in notes if n in _STALE_ENCODER_NOTES]
    if not stale:
        return None
    ki = float(tuning.get("pid_ki", 0.0) or 0.0)
    i_max = float(tuning.get("pid_i_max", 0.0) or 0.0)
    if ki == 0.0 or i_max == 0.0:
        return None  # I term is off entirely; a stale encoder draws nothing
    pos_err_max = float(tuning.get("pos_err_max", 0.0) or 0.0)
    ceiling = min(ki * pos_err_max, i_max) if pos_err_max > 0 else i_max
    return (f"{', '.join(stale)} observed WITH a live position I term "
            f"(pid_ki={ki:g}, pid_i_max={i_max:g}, pos_err_max={pos_err_max:g}) "
            f"-- a wheel that stops reporting can now draw up to "
            f"{ceiling:.1f} mm/s of correction it has not earned; treat this "
            f"run's distance as unmeasured, not as a control result")


def stream_profile(transport, profile: Profile, *, tick: float, lease: float,
                   tail: float, settle: float, wheel: str = "both",
                   verbose: bool = True) -> Run:
    """Stream one profile as re-armed WHEELS commands and capture telemetry.

    The command is refreshed every `tick` seconds with a `lease` several
    ticks long, so a dropped command is covered by the previous lease and
    the robot still stops on its own if this process dies. Capture runs
    `tail` seconds past the end of the profile with a commanded zero, which
    is where an overshooting loop spends the distance that makes its ratio
    exceed 100%.
    """
    run = Run(profile=profile)

    # --- baseline: a settled zero-command window, ending at the reference
    # encoder positions everything downstream is measured against.
    transport.read_pending_binary_tlm_frames()
    baseline: "tuple[float, float] | None" = None
    baseline_robot_clock: "float | None" = None
    deadline = time.monotonic() + settle
    while time.monotonic() < deadline:
        transport.wheels(0.0, 0.0, lease * 1000.0)
        time.sleep(tick)
        for frame in transport.read_pending_binary_tlm_frames():
            positions = _frame_positions(frame)
            if positions is not None:
                baseline = positions
                baseline_robot_clock = frame.t
    if baseline is None:
        run.faults.append("no telemetry during the baseline window")
        return run

    # --- the profile itself, then the commanded-zero tail.
    total = profile.duration + tail
    start = time.monotonic()
    issued = 0.0
    step = 0
    anchor: "float | None" = None
    held_since = start
    held_speed = 0.0

    def drain() -> None:
        nonlocal anchor
        for frame in transport.read_pending_binary_tlm_frames():
            positions = _frame_positions(frame)
            if positions is None:
                continue
            run.faults.extend(f for f in _fault_names(frame) if f not in run.faults)
            run.notes.extend(n for n in _note_names(frame) if n not in run.notes)
            if anchor is None:
                anchor = float(frame.t)
            t_rel = (float(frame.t) - anchor) / 1000.0
            velocity_left, velocity_right = _frame_velocities(frame)
            run.samples.append(Sample(
                t=t_rel,
                commanded=profile.velocity(t_rel),
                velocity_left=velocity_left,
                velocity_right=velocity_right,
                distance_left=positions[0] - baseline[0],
                distance_right=positions[1] - baseline[1]))

    while True:
        stamp = time.monotonic()
        now = stamp - start
        if now >= total:
            break
        # Sample the profile at the MIDPOINT of the tick this command will
        # be held over: for a piecewise-linear profile that makes the sum of
        # (speed * interval) equal the analytic area, so `issued_distance` is
        # a real cross-check on `profile.distance` rather than a
        # systematically low left-edge Riemann sum.
        commanded = profile.velocity(now + tick / 2.0)
        # `wheel` selects which channel(s) carry the profile. A single-motor
        # rig (e.g. a loaded wheel stack on one channel) is driven alone,
        # with the other commanded a hard zero rather than left unwritten --
        # applySpeedFloor()'s dominant-wheel scale reads the PAIR, so an
        # unwritten sibling would still take part in the floor decision.
        left = commanded if wheel in ("both", "left") else 0.0
        right = commanded if wheel in ("both", "right") else 0.0
        transport.wheels(left, right, lease * 1000.0)
        # Account the command that just ENDED, over the interval it was
        # really held -- never the nominal tick. The two differ by however
        # much the host loop drifted, and charging the drift to the profile
        # would report a scheduling artifact as a profile error.
        issued += held_speed * (stamp - held_since)
        held_since, held_speed = stamp, commanded
        step += 1

        drain()
        # Pace to an ABSOLUTE tick boundary rather than sleeping a fixed
        # `tick` after however long the send and drain took, so the loop
        # cannot accumulate drift across the profile.
        remaining = (start + step * tick) - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)

    issued += held_speed * (time.monotonic() - held_since)
    drain()  # frames still in flight when the loop broke
    run.issued_distance = issued
    if verbose:
        drift = abs(issued - profile.distance) / profile.distance * 100.0
        print(f"    issued {step} commands, integral {issued:.1f} mm "
              f"vs design {profile.distance:.1f} mm ({drift:.2f}% off)")
        if drift > 2.0:
            print("    WARNING: the issued profile drifted from its design -- host "
                  "scheduling jitter; treat the ratio below as approximate")
    return run


# ---------------------------------------------------------------------------
# Chart
# ---------------------------------------------------------------------------

def render_chart(runs: "list[Run]", path: pathlib.Path, *, subtitle: str,
                 tolerance: float) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    # Chart order is shape order (simplest demand first), independent of the
    # run order, which is design order.
    ordered = sorted(runs, key=lambda r: r.profile.ramp)
    columns = len(ordered)
    figure, axes = plt.subplots(2, columns, figsize=(7.4 * columns, 8.6),
                                squeeze=False)

    headline = "   ".join(
        f"{run.profile.name}  L {run.ratio_left * 100:.1f}%  R {run.ratio_right * 100:.1f}%"
        for run in ordered)
    figure.suptitle(
        f"Distance fidelity — delivered ÷ commanded, per wheel\n{headline}",
        fontsize=15, y=0.985)
    figure.text(0.5, 0.925, subtitle, ha="center", fontsize=9.5,
                color=COLOR_COMMANDED)

    for column, run in enumerate(ordered):
        profile = run.profile
        times = [s.t for s in run.samples]
        span = max(times) if times else profile.duration
        grid = [i * span / 400.0 for i in range(401)]

        # ---- top: velocity ------------------------------------------------
        ax = axes[0][column]
        ax.plot(grid, [profile.velocity(t) for t in grid], linestyle=(0, (5, 3)),
                linewidth=2, color=COLOR_COMMANDED, label="commanded", zorder=3)
        ax.plot(times, [s.velocity_left for s in run.samples], linewidth=2,
                color=COLOR_LEFT, label="left wheel", zorder=4)
        ax.plot(times, [s.velocity_right for s in run.samples], linewidth=2,
                color=COLOR_RIGHT, label="right wheel", zorder=5)
        ax.set_title(f"{profile.name} — peak {profile.peak:.0f} mm/s", fontsize=12)
        ax.set_ylabel("wheel speed  [mm/s]")
        ax.legend(loc="upper right", frameon=False, fontsize=9)

        # ---- bottom: distance ---------------------------------------------
        ax = axes[1][column]
        ax.plot(grid, [profile.distance_at(t) for t in grid], linestyle=(0, (5, 3)),
                linewidth=2, color=COLOR_COMMANDED, label="commanded", zorder=3)
        ax.plot(times, [s.distance_left for s in run.samples], linewidth=2,
                color=COLOR_LEFT, label="left wheel", zorder=4)
        ax.plot(times, [s.distance_right for s in run.samples], linewidth=2,
                color=COLOR_RIGHT, label="right wheel", zorder=5)
        ax.axhline(profile.distance, color=COLOR_COMMANDED, linewidth=1,
                   linestyle=":", alpha=0.6, zorder=2)

        # Direct end-of-line labels carry the acceptance number itself, so
        # the percentage is readable without a second y-scale (the skill's
        # one-axis rule -- a right-hand % scale would be a dual axis).
        # Labels sit BELOW their line-end (above would collide with the
        # commanded reference whenever a wheel lands close to 100%) and are
        # pushed apart when the two wheels land within a label's height of
        # each other -- which is exactly what a healthy, balanced robot
        # does, so the collision case is the COMMON one, not the edge case.
        ends = sorted(((run.delivered_left, COLOR_LEFT),
                       (run.delivered_right, COLOR_RIGHT)),
                      key=lambda pair: pair[0], reverse=True)
        minimum_gap = 0.075 * profile.distance
        placed: "float | None" = None
        for value, color in ends:
            if value != value:
                continue
            anchor_y = value
            if placed is not None and placed - anchor_y < minimum_gap:
                anchor_y = placed - minimum_gap
            placed = anchor_y
            percent = value / profile.distance * 100.0
            ax.annotate(f"{percent:.1f}%",
                        xy=(span, anchor_y), xytext=(-6, -13),
                        textcoords="offset points", ha="right", va="top",
                        fontsize=11, fontweight="bold", color=color)
        ax.set_ylabel(f"travel  [mm]   (commanded {profile.distance:.0f} mm)")
        ax.set_xlabel("time  [s]     shaded = after the command reached zero")
        ax.legend(loc="upper left", frameon=False, fontsize=9)

        for ax in (axes[0][column], axes[1][column]):
            ax.axvspan(profile.duration, span, color=COLOR_COMMANDED, alpha=0.06,
                       linewidth=0, zorder=0)
            ax.axvline(profile.duration, color=COLOR_COMMANDED, linewidth=1,
                       linestyle=":", alpha=0.6, zorder=1)
            ax.grid(True, color=COLOR_GRID, linewidth=0.7, alpha=0.8)
            ax.set_axisbelow(True)
            for side in ("top", "right"):
                ax.spines[side].set_visible(False)
            ax.set_xlim(0, span)

    figure.text(0.5, 0.012,
                f"PASS band: 100% ± {tolerance * 100:.0f}%.   Both profiles carry the "
                f"same commanded distance and the same 3 s window — only the shape of "
                f"the demand differs.", ha="center", fontsize=9, color=COLOR_COMMANDED)
    figure.tight_layout(rect=(0, 0.025, 1, 0.915))
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=140, facecolor="white")
    plt.close(figure)


SUMMARY_COLUMNS = ("board", "run", "profile", "wheel", "commanded", "delivered",
                   "ratio", "plateau_tracking", "plateau_ripple",
                   "plateau_span", "delivered_at_command_end",
                   "issued_distance", "faults", "notes", "tuning")


def _tuning_tag(tuning: "dict[str, float] | None") -> str:
    """The live tuning rendered as one compact, sortable cell.

    `.claude/rules/configuration-discipline.md`: "record the pushed values
    with the results, so a captured dataset is self-describing instead of
    depending on session memory." A survey CSV whose rows do not say which
    gains produced them is exactly the artifact that rule exists to
    prevent -- and this gate's whole purpose in sprint 133 is to feed a
    tuning session that will produce dozens of such rows.
    """
    if not tuning:
        return ""
    return " ".join(f"{name}={value:g}"
                    for name, value in sorted(tuning.items()) if value)


def append_summary(runs: "list[Run]", path: pathlib.Path, *, board: str,
                   run_tag: str, tuning: "dict[str, float] | None" = None) -> None:
    """APPEND one row per profile per wheel to a shared summary CSV -- the
    machine-readable spine of a multi-board survey, where each gate
    invocation contributes rows to one growing table rather than leaving 30
    separate per-run CSVs to be reconciled by hand. Header is written only
    when the file is new, so concurrent survey legs can append safely.

    `tuning` is the WHEEL_CONTROL read-back, recorded verbatim on every row
    -- see _tuning_tag()."""
    path.parent.mkdir(parents=True, exist_ok=True)
    fresh = not path.exists() or path.stat().st_size == 0
    tuning_tag = _tuning_tag(tuning)
    with open(path, "a", newline="") as handle:
        writer = csv.writer(handle)
        if fresh:
            writer.writerow(SUMMARY_COLUMNS)
        for run in runs:
            track_left, track_right = run.plateau_tracking()
            ripple_left, ripple_right = run.plateau_ripple()
            span_left, span_right = run.plateau_span()
            at_end_left, at_end_right = run.delivered_at_command_end()
            per_wheel = (
                ("left", run.delivered_left, run.ratio_left, track_left,
                 ripple_left, span_left, at_end_left),
                ("right", run.delivered_right, run.ratio_right, track_right,
                 ripple_right, span_right, at_end_right))
            for wheel, delivered, ratio, tracking, ripple, span, at_end in per_wheel:
                writer.writerow([
                    board, run_tag, run.profile.name, wheel,
                    f"{run.profile.distance:.2f}", f"{delivered:.2f}", f"{ratio:.5f}",
                    f"{tracking:.5f}", f"{ripple:.3f}", f"{span:.3f}",
                    f"{at_end:.2f}", f"{run.issued_distance:.2f}",
                    "|".join(run.faults), "|".join(run.notes), tuning_tag])


def write_csv(runs: "list[Run]", path: pathlib.Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["profile", "t", "commanded", "velocity_left",
                         "velocity_right", "distance_left", "distance_right"])
        for run in runs:
            for s in run.samples:
                writer.writerow([run.profile.name, f"{s.t:.3f}", f"{s.commanded:.1f}",
                                 f"{s.velocity_left:.1f}", f"{s.velocity_right:.1f}",
                                 f"{s.distance_left:.2f}", f"{s.distance_right:.2f}"])


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    target = p.add_mutually_exclusive_group()
    target.add_argument("--port", default=None,
                        help=f"robot USB port, or the RADIOBRIDGE relay port for an "
                             f"untethered run (default {DEFAULT_PORT} -- ALWAYS confirm "
                             f"with `mbdeploy list`)")
    target.add_argument("--sim", action="store_true",
                        help="run against the firmware host build + SimPlant, no hardware")
    p.add_argument("--relay", action="store_true",
                   help="force relay mode for --port (default: auto-detect the role)")
    p.add_argument("--robot-json", default=str(_REPO_ROOT / "data" / "robots" / "tovez.json"),
                   help="robot config pushed to the sim before the run (--sim only)")
    p.add_argument("--profiles", default="trapezoid,square",
                   help="comma-separated subset to run (default both)")
    p.add_argument("--peak", type=float, default=200.0,
                   help="[mm/s] trapezoid plateau speed; sets the commanded distance "
                        "for BOTH profiles (default 200)")
    p.add_argument("--duration", type=float, default=3.0,
                   help="[s] commanded window, same for both profiles (default 3.0)")
    p.add_argument("--ramp", type=float, default=0.75,
                   help="[s] trapezoid rise == fall time (default 0.75)")
    # Live tuning, asserted after connect (see _assert_tuning). Omitted
    # flags leave the firmware's own boot-baked value alone.
    p.add_argument("--vmin", type=float, default=None,
                   help="[mm/s] App::Drive speed floor via DBG:vmin "
                        "(ROBOT_DEBUG builds only)")
    p.add_argument("--wheel", choices=("both", "left", "right"), default="both",
                   help="which channel(s) carry the profile. Use left/right "
                        "for a single-motor rig; the idle channel is still "
                        "commanded a hard zero and still reported, which is "
                        "how cross-talk would show up.")
    p.add_argument("--poserr", type=float, default=None,
                   help="[mm] Stage B position-error clamp -- how far behind "
                        "its reference a wheel may fall before the loop stops "
                        "asking for catch-up. 0 = unclamped")
    p.add_argument("--asteady", type=float, default=None,
                   help="[mm/s^2] Stage B integrator / Stage C bias steady "
                        "gate; baked at 30, which freezes both through every "
                        "ramp this rig drives")
    p.add_argument("--gain-left", type=float, default=None,
                   help="multiplier on the BOOT-INSTALLED duty_per_speed, LEFT "
                        "wheel (>1 = faster). The L/R imbalance trim. "
                        "Idempotent: the firmware multiplies the value boot "
                        "installed, not the current one, so re-pushing the "
                        "same number does not compound and 1.0 restores boot.")
    p.add_argument("--gain-right", type=float, default=None,
                   help="multiplier on the BOOT-INSTALLED duty_per_speed, "
                        "RIGHT wheel")
    p.add_argument("--pid-kp", type=float, default=None,
                   help="WHEEL_CONTROL pid_kp -- Stage B proportional gain "
                        "(ships at 0 == inert)")
    p.add_argument("--pid-ki", type=float, default=None,
                   help="WHEEL_CONTROL pid_ki [1/s] -- Stage B's POSITION-term "
                        "gain since 133-002 (mm of error -> mm/s of "
                        "correction), not a velocity integrator")
    p.add_argument("--pid-kff", type=float, default=None,
                   help="WHEEL_CONTROL pid_kaff [s] -- Stage B accel "
                        "feedforward, ~the plant time constant; seeds at "
                        "plant_tau 0.23")
    p.add_argument("--pid-imax", type=float, default=None,
                   help="WHEEL_CONTROL pid_i_max [mm/s] -- the I term's OUTPUT "
                        "clamp. 0 disables the I term ENTIRELY, so --pid-ki "
                        "does nothing until this is nonzero too")
    p.add_argument("--pid-kaw", type=float, default=None,
                   help="WHEEL_CONTROL pid_max [mm/s] -- Stage B total "
                        "fast-loop authority cap (NOT an anti-windup gain; "
                        "there is no accumulator to unwind since 133-002)")
    p.add_argument("--attempts", type=int, default=6,
                   help="retries per profile when telemetry truncates mid-run")
    p.add_argument("--tick", type=float, default=0.05,
                   help="[s] command refresh interval (default 0.05 == 20 Hz)")
    p.add_argument("--lease", type=float, default=0.25,
                   help="[s] WHEELS command duration; must exceed --tick so a dropped "
                        "command is covered (default 0.25)")
    p.add_argument("--tail", type=float, default=1.5,
                   help="[s] commanded-zero capture past the profile end, where an "
                        "overshooting loop spends its extra distance (default 1.5)")
    p.add_argument("--settle", type=float, default=1.0,
                   help="[s] stopped baseline window before each profile (default 1.0)")
    p.add_argument("--rest", type=float, default=1.5,
                   help="[s] pause between profiles (default 1.5)")
    p.add_argument("--tolerance", type=float, default=0.05,
                   help="PASS band on the delivered/commanded ratio (default 0.05 == +-5%%)")
    p.add_argument("--chart", default=str(_REPO_ROOT / "src" / "tests" / "bench" / "output"
                                          / "velocity_profile_gate.png"))
    p.add_argument("--csv", default=str(_REPO_ROOT / "src" / "tests" / "bench" / "output"
                                        / "velocity_profile_gate.csv"))
    p.add_argument("--no-chart", action="store_true")
    p.add_argument("--summary-csv", default=None,
                   help="APPEND one row per profile per wheel to this shared CSV "
                        "(multi-board surveys: every leg appends to one table)")
    p.add_argument("--board", default=None,
                   help="board identity recorded in --summary-csv (default: the "
                        "device name reported in the connect banner, or 'sim')")
    p.add_argument("--run", dest="run_tag", default="1",
                   help="run identity recorded in --summary-csv, e.g. cold/warm1")
    return p.parse_args()


def _open_sim(args):
    from robot_radio.config.robot_config import load_robot_config
    from robot_radio.io.sim_loop import SimLoop

    config = load_robot_config(args.robot_json)
    track_width = config.trackwidth if config.trackwidth is not None else 128.0
    sim = SimLoop(track_width=track_width)
    sim.connect()
    # Sprint 114's fail-closed gate: an unconfigured robot refuses every
    # motion command and produces a silent all-zero trace.
    sim.configure_from_robot(config)
    print(f"  sim connected: firmware={sim.firmware_version()} track_width={track_width}")
    tuning_flags = [flag for flag, _ in _PID_FLAG_TO_KEY] + list(_DBG_FLAGS)
    if any(getattr(args, flag, None) is not None for flag in tuning_flags):
        # Say so rather than silently ignoring them. A sim run that
        # SEEMS to honour --pid-ki and does not is precisely the "measured
        # the boot defaults every time" failure this gate's own tuning path
        # exists to end.
        print("  WARNING: the tuning flags are a HARDWARE path (DBG verbs + "
              "the WHEEL_CONTROL wire group). --sim ignores every one of "
              "them; this run measures the sim's own configured values.")
    # Sim reports no WHEEL_CONTROL group, so downstream readings that key on
    # the live gains (i_term_hazard()) correctly find nothing live.
    return sim, sim.disconnect, "sim", {}


_DEBUG_LINES: "list[str]" = []

# The five `--pid-*` flag names, mapped to the historical flat gain keys
# `wheel_control_tuning.FLAT_KEY_TO_FIELD` translates to real WheelControl
# protobuf field names. The flag spellings are the ones the tuning session
# already uses and are deliberately NOT renamed; the one translation that
# matters (`pid.kff -> pid_kaff`, `pid.kaw -> pid_max`) lives in that shared
# module, tested, rather than being re-derived in each script that needs it.
_PID_FLAG_TO_KEY = (
    ("pid_kp", "pid.kp"),
    ("pid_ki", "pid.ki"),
    ("pid_kff", "pid.kff"),
    ("pid_imax", "pid.iMax"),
    ("pid_kaw", "pid.kaw"),
)

# The four RAM-only DBG flags, kept beside the wire flags above so the two
# channels' membership is stated once (the sim warning and the "which
# survives a reboot" note both read it).
_DBG_FLAGS = ("vmin", "asteady", "poserr", "gain_left", "gain_right")


def _assert_tuning(conn, proto, args) -> "dict[str, float]":
    """Push tuning AFTER connecting -- which is the only moment it can be
    pushed at all -- confirm every value LANDED, and return the
    WHEEL_CONTROL group as the robot reports it.

    Opening this port RESETS the board (measured: uptime 29361 -> 34911 ms
    held open, but ~4051 ms after a close/reopen, and suppressing DTR/RTS
    does not prevent it). So every RAM-only value set by a DIFFERENT
    process is already gone by the time this one runs. Asserting here,
    inside the process that then streams the profile, is what makes a tuned
    measurement possible at all -- an earlier sweep set values externally
    and unknowingly measured the boot defaults every time.

    Two confirmation mechanisms, one per channel, and NEITHER is an ack:

      * DBG verbs are confirmed by the firmware's own ``... applied`` echo,
        which carries the value that actually landed.
      * WHEEL_CONTROL fields are confirmed by READING THE GROUP BACK and
        comparing. `.claude/rules/configuration-discipline.md`: "an ack is
        not evidence: config that acks OK and lands nowhere is a live
        failure mode in this codebase, not a hypothetical."

    Raises RuntimeError on any unconfirmed push. Refusing to run is the
    point -- a gate that reported a distance against gains it never set
    would produce a confident wrong answer, which is worse than no answer.
    """
    # Imported here, not at module scope, for the same reason
    # _open_hardware()/_open_sim() import their own dependencies locally:
    # `--sim` and the pure-profile unit tests must not need the robot_radio
    # wire stack (or protobuf) present at all. The sys.path insert makes the
    # sibling import work when this file is loaded BY PATH (the
    # src/tests/unit/ importlib idiom) as well as when it is run as a
    # script, where its own directory is already sys.path[0].
    if str(_BENCH_DIR) not in sys.path:
        sys.path.insert(0, str(_BENCH_DIR))
    from wheel_control_tuning import (TuningNotConfirmed, describe_persistence,
                                      push_gains)

    def send_dbg(text: str) -> "str | None":
        """Send one DBG verb; return the robot's own echo line, or None if
        no confirmation arrived. The ECHO is returned rather than a bool so
        the caller can print what the robot said it applied, not what the
        host asked for -- the two differing is exactly the bug class this
        whole mechanism exists to catch."""
        proto.tlmOff()          # binary TLM would bury the cleartext echo
        time.sleep(0.4)
        conn.read_pending_lines()
        _DEBUG_LINES.clear()
        conn.send_fast(f"DBG:{text}")
        echo, deadline = None, time.monotonic() + 3.0
        while echo is None and time.monotonic() < deadline:
            for line in list(_DEBUG_LINES):
                if "applied" in line:
                    echo = line.strip()
                    break
            if echo is None:
                time.sleep(0.05)
        proto.tlmOn()
        time.sleep(1.0)
        proto.read_pending_binary_tlm_frames()
        return echo

    def assert_dbg(verb: str, description: str) -> None:
        echo = send_dbg(verb)
        if echo is None:
            raise RuntimeError(
                f"DBG:{verb} was not confirmed -- refusing to report a run "
                f"whose {description} is unknown. If the echo never arrives "
                f"at all, the most likely cause is a firmware image built "
                f"WITHOUT the DBG channel: rebuild with "
                f"`uv run python build.py --clean --robot-debug`.")
        print(f"  asserted: {verb:<24} robot echo: {echo}")

    if args.vmin is not None:
        assert_dbg(f"vmin {args.vmin}", "speed floor")
    if args.asteady is not None:
        assert_dbg(f"asteady {args.asteady}", "steady gate")
    if args.poserr is not None:
        assert_dbg(f"pos {args.poserr}", "position-error clamp")
    if args.gain_left is not None or args.gain_right is not None:
        # Both operands are always sent: the verb takes a PAIR and rewrites
        # both wheels, and the firmware rejects a one-operand form outright
        # rather than defaulting the other to 1.0 -- so omitting one here
        # would produce no push at all, not a half push.
        gain_left = args.gain_left if args.gain_left is not None else 1.0
        gain_right = args.gain_right if args.gain_right is not None else 1.0
        assert_dbg(f"gain {gain_left} {gain_right}", "wheel duty gains")

    gains = {key: float(getattr(args, flag))
             for flag, key in _PID_FLAG_TO_KEY
             if getattr(args, flag, None) is not None}

    # push_gains() reads the group back UNCONDITIONALLY, even when this run
    # pushed nothing. The five pid_* fields persist in flash, so a value some
    # EARLIER session pushed is still in force here; a run that printed only
    # what it pushed itself would describe a robot that does not exist.
    try:
        live_fields = push_gains(proto, gains)
    except TuningNotConfirmed as exc:
        raise RuntimeError(
            f"{exc} -- refusing to report a run whose gains are unknown") from exc

    print("\n  === live WHEEL_CONTROL (read back from the robot) ===")
    for line in describe_persistence(live_fields):
        print(line)
    return live_fields


def _open_hardware(args):
    """Connect, assert tuning, and return
    ``(transport, close, device_name, tuning)`` -- `tuning` being the
    WHEEL_CONTROL group as the ROBOT reports it after the push, which is
    what every downstream reading is attributed against."""
    from robot_radio.io.serial_conn import SerialConnection
    from robot_radio.robot.protocol import NezhaProtocol

    port = args.port or DEFAULT_PORT
    conn = SerialConnection(port=port, mode="relay" if args.relay else None,
                            on_debug=lambda line: _DEBUG_LINES.append(str(line)))
    info = conn.connect()
    if info.get("status") not in ("connected", "already_connected"):
        raise RuntimeError(f"connect failed: {info}")
    announcement = info.get("announcement") or {}
    device_name = announcement.get("device_name") or "unknown"
    print(f"  connected: {port}  mode={info.get('mode')}  "
          f"device={device_name}  role={announcement.get('role')}")
    proto = NezhaProtocol(conn)
    # Telemetry is silent at idle by design -- without this the baseline
    # window sees nothing and the run aborts before it starts.
    proto.tlmOn()
    time.sleep(1.5)
    proto.read_pending_binary_tlm_frames()
    tuning = _assert_tuning(conn, proto, args)

    def close():
        try:
            proto.tlmOff()
        except Exception as exc:
            print(f"  WARN: tlmOff() failed during cleanup: {exc}")
        conn.disconnect()

    return proto, close, device_name, tuning


def main() -> int:
    args = _args()
    if args.lease <= args.tick:
        print(f"ERROR: --lease ({args.lease}) must exceed --tick ({args.tick}) or a "
              f"single dropped command stalls the profile")
        return 2

    wanted = [name.strip() for name in args.profiles.split(",") if name.strip()]
    catalog = {p.name: p for p in make_profiles(args.peak, args.duration, args.ramp)}
    unknown = [name for name in wanted if name not in catalog]
    if unknown:
        print(f"ERROR: unknown profile(s) {unknown}; choose from {sorted(catalog)}")
        return 2
    profiles = [catalog[name] for name in wanted]

    print(f"\n  commanded distance: {profiles[0].distance:.0f} mm "
          f"(identical for every profile in this run)")
    for profile in profiles:
        print(f"    {profile.name:>10}: {profile.duration:.2f} s  peak "
              f"{profile.peak:.1f} mm/s  ramp {profile.ramp:.2f} s  "
              f"area {profile.distance:.1f} mm")

    transport, close, device_name, tuning = (
        _open_sim(args) if args.sim else _open_hardware(args))
    board = args.board or device_name
    runs: "list[Run]" = []
    try:
        for index, profile in enumerate(profiles):
            print(f"\n  --- {profile.name} ---")
            if index:
                time.sleep(args.rest)
            # A truncated capture is not a bad result, it is a MISSING one:
            # this rig drops telemetry mid-run roughly half the time, and a
            # short capture reads as a short distance (77mm of 450mm on the
            # first attempt here) that is indistinguishable from real
            # under-travel. Require a nearly-complete capture, and retry.
            want = max(1, int(0.80 * (profile.duration + args.tail) / 0.05))
            run = None
            for attempt in range(1, args.attempts + 1):
                try:
                    candidate = stream_profile(transport, profile, tick=args.tick,
                                               lease=args.lease, tail=args.tail,
                                               settle=args.settle,
                                               wheel=args.wheel)
                except Exception as exc:
                    print(f"    attempt {attempt}: stream failed ({exc})")
                    candidate = None
                n = len(candidate.samples) if candidate else 0
                if candidate is not None and n >= want:
                    run = candidate
                    break
                print(f"    attempt {attempt}: {n} of ~{want} samples -- "
                      f"telemetry truncated, discarding")
                if candidate is not None and (run is None
                                              or n > len(run.samples)):
                    run = candidate           # keep the best, in case all fail
                if args.sim or attempt == args.attempts:
                    continue
                # The board does not merely drop frames when it truncates --
                # it goes silent, and the next DBG gets no echo either. A
                # close/reopen resets it (the same port-open reset that makes
                # RAM tuning so fragile), which is the one reliable recovery.
                print("    reconnecting to clear the wedge...")
                try:
                    close()
                except Exception:
                    pass
                time.sleep(2.0)
                # Re-asserting the tuning is the WHOLE point of reconnecting
                # through _open_hardware() rather than just reopening the
                # port: the reconnect resets the board, which wipes every
                # RAM-only DBG value. A retry that skipped this would
                # silently finish the run against boot defaults.
                transport, close, device_name, tuning = _open_hardware(args)
                board = args.board or device_name
            if run is None:
                print(f"    ERROR: {profile.name} never produced a usable "
                      f"capture in {args.attempts} attempts")
                return 1
            runs.append(run)
            if run.faults:
                print(f"    FAULTS OBSERVED: {', '.join(run.faults)}")
            if run.notes:
                print(f"    (informational, not gated: {', '.join(run.notes)})")
            hazard = i_term_hazard(run.notes, tuning)
            if hazard:
                print(f"    !! {hazard}")
            at_end_left, at_end_right = run.delivered_at_command_end()
            track_left, track_right = run.plateau_tracking()
            print(f"    samples: {len(run.samples)}")
            print(f"    plateau tracking vs {profile.peak:.0f} mm/s commanded: "
                  f"left {track_left * 100:.1f}%   right {track_right * 100:.1f}%")
            print(f"    left : {run.delivered_left:8.1f} mm  "
                  f"({run.ratio_left * 100:6.1f}%)   at command end "
                  f"{at_end_left:8.1f} mm")
            print(f"    right: {run.delivered_right:8.1f} mm  "
                  f"({run.ratio_right * 100:6.1f}%)   at command end "
                  f"{at_end_right:8.1f} mm")
    finally:
        try:
            transport.estop()
        except Exception as exc:
            print(f"  WARN: estop() failed during cleanup: {exc}")
        close()

    print(f"\n  === distance fidelity (delivered / commanded, "
          f"PASS band 100% +- {args.tolerance * 100:.0f}%) ===")
    print(f"  {'profile':>10}  {'wheel':>6}  {'delivered':>10}  {'commanded':>10}  "
          f"{'ratio':>8}  verdict")
    ok = True
    for run in runs:
        for wheel, ratio in ratios(run):
            delivered = run.delivered_left if wheel == "left" else run.delivered_right
            passed = ratio == ratio and abs(ratio - 1.0) <= args.tolerance
            ok = ok and passed
            print(f"  {run.profile.name:>10}  {wheel:>6}  {delivered:>9.1f}mm  "
                  f"{run.profile.distance:>9.1f}mm  {ratio * 100:>7.1f}%  "
                  f"{'PASS' if passed else 'FAIL'}")
        if run.faults:
            ok = False
            print(f"  {run.profile.name:>10}  {'--':>6}  FAULT: {', '.join(run.faults)}")
        if run.delivered_right:
            imbalance = run.delivered_left / run.delivered_right
            print(f"  {run.profile.name:>10}  {'L/R':>6}  imbalance {imbalance:.4f} "
                  f"({(imbalance - 1.0) * 100:+.1f}%)  [reported, not gated]")
        track_left, track_right = run.plateau_tracking()
        print(f"  {run.profile.name:>10}  {'track':>6}  plateau speed reached: "
              f"L {track_left * 100:.1f}%  R {track_right * 100:.1f}% of commanded "
              f"[reported, not gated]")
        # Ripple travels WITH the ratio, in the same table, on purpose: the
        # ratio alone cannot be compared against a historical figure taken
        # under a different duty quantizer (133-002's sigma-delta), and
        # without this column that incomparability is invisible.
        ripple_left, ripple_right = run.plateau_ripple()
        span_left, span_right = run.plateau_span()
        print(f"  {run.profile.name:>10}  {'ripple':>6}  plateau sd: "
              f"L {ripple_left:.2f}  R {ripple_right:.2f} mm/s   "
              f"peak-to-peak: L {span_left:.1f}  R {span_right:.1f} mm/s "
              f"[reported, not gated]")
        hazard = i_term_hazard(run.notes, tuning)
        if hazard:
            print(f"  {run.profile.name:>10}  {'!!':>6}  {hazard}")

    if runs and not args.no_chart:
        target = "sim" if args.sim else (args.port or DEFAULT_PORT)
        subtitle = (f"{'sim' if args.sim else 'hardware'} · {target} · "
                    f"{profiles[0].duration:.1f} s window · "
                    f"{profiles[0].distance:.0f} mm commanded · "
                    f"WHEELS straight to App::Drive, no planner, no shaping")
        chart_path = pathlib.Path(args.chart)
        render_chart(runs, chart_path, subtitle=subtitle, tolerance=args.tolerance)
        print(f"\n  chart: {chart_path}")
    if runs:
        csv_path = pathlib.Path(args.csv)
        write_csv(runs, csv_path)
        print(f"  csv:   {csv_path}")
        if args.summary_csv:
            summary_path = pathlib.Path(args.summary_csv)
            append_summary(runs, summary_path, board=board, run_tag=args.run_tag,
                           tuning=tuning)
            print(f"  summary appended ({board}/{args.run_tag}): {summary_path}")

    print(f"\n  {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
