#!/usr/bin/env python3
"""square_tour_sim.py -- run a SQUARE TOUR (4 x 500 mm legs, 4 x 90 deg
turns) through the REAL Motion::Planner (libmotionplanner via ctypes)
against a simulated pair of mismatched motors: the LEFT motor responds 10%
FASTER than the right to the same duty.

The sim MIRRORS THE REAL LOOP SCHEDULE (stakeholder 2026-07-26), not an
idealized lockstep: within each 50 ms cycle the plant runs continuously
(1 ms substeps) while the loop's events land at their real offsets --

    t+ 0 ms   select L
    t+ 4 ms   collect L   -> left sample taken NOW (sampleTime = t+4)
    t+ 8 ms   clear
    t+12 ms   collect R   -> right sample taken NOW (sampleTime = t+12)
    t+13 ms   planner tick (sense->estimate->plan->PID), cycleStart = t
    t+14 ms   duty write L takes effect (brick slew applies per write)
    t+18 ms   duty write R takes effect
    t+50 ms   next cycle

so the encoders are genuinely read at DIFFERENT times (8 ms L/R skew,
carried in each sample's own sampleTime -- the planner's per-wheel ZOH
estimation must and does absorb it), and the two duty writes land at
different times too. Smoothing: jerk-limited S-curve profile (jerkMax /
yawJerkMax) + acceleration feedforward (velKaff = tau * kff) so ramp
corners carry no whiplash and no integral-windup overshoot.

Produces a dual-axis plot: true wheel speeds vs time (left axis) and
commanded duty per wheel (right axis).

    uv run python src/motion/planner/bench/square_tour_sim.py
"""

import ctypes
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "py"))
from planner_harness import (  # noqa: E402
    KIND_ANGLE, KIND_DISTANCE, VELOCITY_TWIST, Move, PlannerLimits,
    RobotState, TickResult, loadLibrary)

CYCLE = 50           # [ms] loop period
SUBSTEP = 0.001      # [s] plant integration substep
TRACK = 128.0        # [mm]
TAU = 0.23           # [s] measured plant time constant
SLEW = 0.25          # [duty/write] brick write slew cap
QUANTUM = 0.0716     # [mm] encoder position quantum
VEL_NOISE = 15.0     # [mm/s] reported-velocity zig-zag amplitude

# The stakeholder's asymmetry: same duty, left runs 10% faster.
GAIN_RIGHT = 1300.0            # [mm/s per duty]
GAIN_LEFT = GAIN_RIGHT * 1.10  # [mm/s per duty] +10%
GAIN_NOMINAL = 0.5 * (GAIN_LEFT + GAIN_RIGHT)

# Schedule offsets within the cycle. [ms]
T_COLLECT_L = 4
T_COLLECT_R = 12
T_TICK = 13
T_WRITE_L = 14
T_WRITE_R = 18

LEG = 500.0          # [mm]
CRUISE = 200.0       # [mm/s]
TURN = math.pi / 2   # [rad]
OMEGA = 1.5          # [rad/s]


class SimWheel:
    """Continuous first-order plant; duty changes land at write events."""

    def __init__(self, gain: float):
        self.gain = gain      # [mm/s per duty]
        self.applied = 0.0    # [duty] after per-write slew
        self.velocity = 0.0   # [mm/s] true
        self.position = 0.0   # [mm] true

    def write(self, dutyCommand: float) -> None:
        delta = dutyCommand - self.applied
        if delta > SLEW:
            self.applied += SLEW
        elif delta < -SLEW:
            self.applied -= SLEW
        else:
            self.applied = dutyCommand

    def substep(self, dt: float) -> None:
        self.velocity += (self.applied * self.gain - self.velocity) * (dt / TAU)
        self.position += self.velocity * dt


def tourLimits() -> PlannerLimits:
    limits = PlannerLimits()
    limits.vMax = 400.0
    limits.aMax = 300.0
    limits.aDecel = 250.0
    limits.omegaMax = 3.0
    limits.alphaMax = 6.0
    limits.alphaDecel = 5.0
    limits.trackWidth = TRACK
    limits.controlPeriod = float(CYCLE)
    limits.actuationDelay = 40.0  # [ms] sample age (~t+8 avg) to actuation midpoint
    limits.velocityFilterWeight = 0.35
    limits.otosStaleness = 200
    limits.headingOtosWeight = 0.0
    limits.requireSettle = True
    limits.settleWindow = 2500.0
    limits.headingHoldGain = 2.0
    limits.velKff = 1.0 / GAIN_NOMINAL
    limits.velKp = 0.0009
    limits.velKi = 0.004
    limits.velIMax = 0.25
    limits.velIAccelGate = 50.0
    # Smoothing (this revision): accel feedforward from the measured tau,
    # jerk-limited S-curve ramps.
    limits.velKaff = TAU / GAIN_NOMINAL
    limits.jerkMax = 1500.0    # [mm/s^3] aMax reached in ~0.2 s
    limits.yawJerkMax = 30.0   # [rad/s^3] alphaMax reached in ~0.2 s
    return limits


def tourMoves() -> list[Move]:
    moves = []
    moveId = 1
    for _ in range(4):
        moves.append(Move(id=moveId, kind=KIND_DISTANCE, threshold=LEG,
                          timeout=30000.0, velocityKind=VELOCITY_TWIST,
                          v_x=CRUISE))
        moveId += 1
        moves.append(Move(id=moveId, kind=KIND_ANGLE, threshold=TURN,
                          timeout=30000.0, velocityKind=VELOCITY_TWIST,
                          omega=OMEGA))
        moveId += 1
    return moves


def main() -> None:
    lib = loadLibrary()
    limits = tourLimits()
    planner = lib.plannerCreate(ctypes.byref(limits))
    state = RobotState()
    result = TickResult()
    dutyLeft = ctypes.c_float()
    dutyRight = ctypes.c_float()
    left = SimWheel(GAIN_LEFT)
    right = SimWheel(GAIN_RIGHT)

    moves = tourMoves()
    nextMove = 0
    completions = []  # (t_s, moveId)

    log = dict(t=[], velLeft=[], velRight=[], dutyLeft=[], dutyRight=[])
    sampleParity = 0
    pendingDutyL = 0.0
    pendingDutyR = 0.0
    tourDoneAtCycle = None

    def takeSample(wheel, plant, sampleTime: int, parity: int) -> None:
        # Zero-mean PER WHEEL: alternate the sign by CYCLE, same for both
        # wheels (matches duty_plant.h). Keying it to a global per-sample
        # counter gave L permanently +noise and R permanently -noise -- a
        # constant bias that wound the PID integral and overdrove every
        # turn ~70% (found via the single-turn trace, 2026-07-26).
        wheel.position = round(plant.position / QUANTUM) * QUANTUM
        noise = VEL_NOISE if parity % 2 == 0 else -VEL_NOISE
        wheel.velocity = plant.velocity + noise
        wheel.sampleTime = sampleTime
        wheel.connected = True

    # Seed at-rest samples so the first tick has a basis.
    takeSample(state.wheelLeft, left, 0, 0)
    takeSample(state.wheelRight, right, 0, 1)

    maxCycles = 1200
    for cycle in range(maxCycles):
        t0 = cycle * CYCLE  # [ms] this cycle's start

        while nextMove < len(moves) and lib.plannerMove(
                planner, ctypes.byref(moves[nextMove]), False):
            nextMove += 1

        # Walk the cycle in 1 ms steps; fire schedule events at their
        # offsets; integrate the plant continuously in between.
        for offset in range(CYCLE):
            tMs = t0 + offset
            if offset == T_COLLECT_L:
                takeSample(state.wheelLeft, left, tMs, cycle)
            elif offset == T_COLLECT_R:
                takeSample(state.wheelRight, right, tMs, cycle)
            elif offset == T_TICK:
                state.time.cycleStart = t0
                lib.plannerTick(planner, ctypes.byref(state),
                                ctypes.byref(result))
                lib.plannerUpdate(planner, ctypes.byref(state))
                lib.plannerDuty(planner, ctypes.byref(dutyLeft),
                                ctypes.byref(dutyRight))
                pendingDutyL = dutyLeft.value
                pendingDutyR = dutyRight.value
                if result.completed:
                    completions.append((tMs / 1000.0, result.moveId))
            elif offset == T_WRITE_L:
                left.write(pendingDutyL)
            elif offset == T_WRITE_R:
                right.write(pendingDutyR)
            left.substep(SUBSTEP)
            right.substep(SUBSTEP)
            # Log at 1 ms resolution so the plot shows the true dynamics.
            log["t"].append(tMs / 1000.0)
            log["velLeft"].append(left.velocity)
            log["velRight"].append(right.velocity)
            log["dutyLeft"].append(pendingDutyL)
            log["dutyRight"].append(pendingDutyR)

        if len(completions) == len(moves) and tourDoneAtCycle is None:
            tourDoneAtCycle = cycle + 30  # drain tail for the plot
        if tourDoneAtCycle is not None and cycle >= tourDoneAtCycle:
            break

    path = 0.5 * (left.position + right.position)
    headingDeg = math.degrees((right.position - left.position) / TRACK)
    peak = max(max(log["velLeft"]), max(log["velRight"]))

    print(f"tour: {len(completions)}/{len(moves)} moves completed in "
          f"{log['t'][-1]:.1f} s")
    print(f"  total path {path:.1f} mm (target {4 * LEG:.0f}); "
          f"error {path - 4 * LEG:+.2f} mm")
    print(f"  final heading {headingDeg:.2f} deg (target 360); "
          f"error {headingDeg - 360.0:+.2f} deg")
    print(f"  peak wheel speed {peak:.1f} mm/s (cruise {CRUISE:.0f}; "
          f"overshoot {100.0 * (peak - CRUISE) / CRUISE:+.1f}%)")
    lib.plannerDestroy(planner)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axSpeed = plt.subplots(figsize=(14, 6))
    axDuty = axSpeed.twinx()

    axSpeed.plot(log["t"], log["velLeft"], color="#1f77b4", lw=1.4,
                 label="left wheel speed [mm/s] (gain +10%)")
    axSpeed.plot(log["t"], log["velRight"], color="#2ca02c", lw=1.4,
                 label="right wheel speed [mm/s]")
    axDuty.plot(log["t"], log["dutyLeft"], color="#1f77b4", lw=1.0,
                ls="--", alpha=0.6, label="left duty")
    axDuty.plot(log["t"], log["dutyRight"], color="#2ca02c", lw=1.0,
                ls="--", alpha=0.6, label="right duty")

    for tc, moveId in completions:
        axSpeed.axvline(tc, color="#888888", lw=0.6, alpha=0.5)
        kind = "leg" if moveId % 2 == 1 else "turn"
        axSpeed.annotate(f"{kind} {(moveId + 1) // 2}",
                         xy=(tc, 0.98), xycoords=("data", "axes fraction"),
                         fontsize=7.5, rotation=90, va="top", ha="right",
                         color="#666666")

    # Zero-aligned dual axes: both symmetric about zero so y=0 coincides.
    speedLim = 1.1 * max(max(map(abs, log["velLeft"])),
                         max(map(abs, log["velRight"])))
    dutyLim = 1.15 * max(max(map(abs, log["dutyLeft"])),
                         max(map(abs, log["dutyRight"])), 0.01)
    axSpeed.set_ylim(-speedLim, speedLim)
    axDuty.set_ylim(-dutyLim, dutyLim)
    axSpeed.set_xlabel("time [s]")
    axSpeed.set_ylabel("true wheel speed [mm/s]")
    axDuty.set_ylabel("commanded duty [-1..1]")
    axSpeed.axhline(0.0, color="black", lw=0.5)
    axSpeed.axhline(CRUISE, color="#cccccc", lw=0.7, ls=":")
    axSpeed.set_title(
        f"Square tour, schedule-faithful loop sim (staggered reads/writes) "
        f"-- mismatched motors (left +10%)\n"
        f"S-curve jerk limit + accel feedforward; path {path:.1f}/"
        f"{4 * LEG:.0f} mm, heading {headingDeg:.1f}/360 deg, peak "
        f"{peak:.0f} mm/s")
    lines1, labels1 = axSpeed.get_legend_handles_labels()
    lines2, labels2 = axDuty.get_legend_handles_labels()
    axSpeed.legend(lines1 + lines2, labels1 + labels2, loc="lower right",
                   fontsize=8, framealpha=0.9)
    axSpeed.grid(True, alpha=0.25)
    fig.tight_layout()
    out = Path(__file__).parent / "square_tour_sim.png"
    fig.savefig(out, dpi=130)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
