#!/usr/bin/env python3
"""square_tour_sim.py -- run a SQUARE TOUR (4 x 500 mm legs, 4 x 90 deg
turns) through the REAL Motion::Planner (libmotionplanner via ctypes)
against a simulated pair of mismatched motors: the LEFT motor responds 10%
FASTER than the right to the same duty. One-loop co-located topology
(sense -> plan -> PID -> duty, same interval), the same shape as
tests/planner_duty_scenarios_test.cpp.

Produces a dual-axis plot: true wheel speeds vs time (left axis) and
commanded duty per wheel (right axis), with segment boundaries marked.

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

PERIOD = 0.05        # [s]
TRACK = 128.0        # [mm]
TAU = 0.23           # [s] measured plant time constant
SLEW = 0.25          # [duty/cycle] brick write slew cap
QUANTUM = 0.0716     # [mm] encoder position quantum
VEL_NOISE = 15.0     # [mm/s] reported-velocity zig-zag amplitude

# The stakeholder's asymmetry: same duty, left runs 10% faster.
GAIN_RIGHT = 1300.0            # [mm/s per duty]
GAIN_LEFT = GAIN_RIGHT * 1.10  # [mm/s per duty] +10%
GAIN_NOMINAL = 0.5 * (GAIN_LEFT + GAIN_RIGHT)

LEG = 500.0          # [mm]
CRUISE = 200.0       # [mm/s]
TURN = math.pi / 2   # [rad]
OMEGA = 1.5          # [rad/s]


class SimWheel:
    def __init__(self, gain: float):
        self.gain = gain      # [mm/s per duty]
        self.applied = 0.0    # [duty] after slew
        self.velocity = 0.0   # [mm/s] true
        self.position = 0.0   # [mm] true

    def step(self, dutyCommand: float, dt: float) -> None:
        delta = dutyCommand - self.applied
        if delta > SLEW:
            self.applied += SLEW
        elif delta < -SLEW:
            self.applied -= SLEW
        else:
            self.applied = dutyCommand
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
    limits.controlPeriod = PERIOD * 1000.0
    limits.actuationDelay = 50.0
    limits.velocityFilterWeight = 0.35
    limits.otosStaleness = 200
    limits.headingOtosWeight = 0.0
    limits.requireSettle = True
    limits.settleWindow = 800.0
    limits.headingHoldGain = 1.5
    limits.velKff = 1.0 / GAIN_NOMINAL
    limits.velKp = 0.0009
    limits.velKi = 0.006
    limits.velIMax = 0.25
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
    completions = []  # (t, moveId)

    # Publish the initial (at-rest) samples.
    now = 0
    stepCount = 0

    def publish() -> None:
        for wheel, plant in ((state.wheelLeft, left),
                             (state.wheelRight, right)):
            wheel.position = round(plant.position / QUANTUM) * QUANTUM
            noise = VEL_NOISE if stepCount % 2 == 0 else -VEL_NOISE
            wheel.velocity = plant.velocity + noise
            wheel.sampleTime = now
            wheel.connected = True

    publish()

    log = dict(t=[], velLeft=[], velRight=[], dutyLeft=[], dutyRight=[])
    tourDone = None
    maxTicks = 3000
    for tick in range(maxTicks):
        # Keep the 5-deep queue topped up.
        while nextMove < len(moves) and lib.plannerMove(
                planner, ctypes.byref(moves[nextMove]), False):
            nextMove += 1

        state.time.cycleStart = now
        lib.plannerTick(planner, ctypes.byref(state), ctypes.byref(result))
        lib.plannerUpdate(planner, ctypes.byref(state))
        lib.plannerDuty(planner, ctypes.byref(dutyLeft),
                        ctypes.byref(dutyRight))
        if result.completed:
            completions.append((now / 1000.0, result.moveId))

        # Plant integrates THIS tick's duty over the interval.
        left.step(dutyLeft.value, PERIOD)
        right.step(dutyRight.value, PERIOD)
        now += int(PERIOD * 1000)
        stepCount += 1
        publish()

        log["t"].append(now / 1000.0)
        log["velLeft"].append(left.velocity)
        log["velRight"].append(right.velocity)
        log["dutyLeft"].append(dutyLeft.value)
        log["dutyRight"].append(dutyRight.value)

        if len(completions) == len(moves) and tourDone is None:
            tourDone = tick + 40  # drain tail for the plot
        if tourDone is not None and tick >= tourDone:
            break

    path = 0.5 * (left.position + right.position)
    headingDeg = math.degrees((right.position - left.position) / TRACK)
    print(f"tour: {len(completions)}/{len(moves)} moves completed in "
          f"{log['t'][-1]:.1f} s")
    print(f"  total path {path:.1f} mm (target {4 * LEG:.0f} + pivots)")
    print(f"  final heading {headingDeg:.2f} deg (target 360)")
    perLeg = 4 * LEG
    print(f"  distance error {path - perLeg:+.2f} mm; heading error "
          f"{headingDeg - 360.0:+.2f} deg")
    lib.plannerDestroy(planner)

    # ---- plot: speeds (left axis) + duties (right axis) ----
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axSpeed = plt.subplots(figsize=(14, 6))
    axDuty = axSpeed.twinx()

    axSpeed.plot(log["t"], log["velLeft"], color="#1f77b4", lw=1.6,
                 label="left wheel speed [mm/s] (gain +10%)")
    axSpeed.plot(log["t"], log["velRight"], color="#2ca02c", lw=1.6,
                 label="right wheel speed [mm/s]")
    axDuty.plot(log["t"], log["dutyLeft"], color="#1f77b4", lw=1.0,
                ls="--", alpha=0.65, label="left duty")
    axDuty.plot(log["t"], log["dutyRight"], color="#2ca02c", lw=1.0,
                ls="--", alpha=0.65, label="right duty")

    for i, (tc, moveId) in enumerate(completions):
        axSpeed.axvline(tc, color="#888888", lw=0.6, alpha=0.5)
        kind = "leg" if moveId % 2 == 1 else "turn"
        axSpeed.text(tc, axSpeed.get_ylim()[1] * 0.0 - 0.0, "",
                     fontsize=7)
        axSpeed.annotate(f"{kind} {(_ := (moveId + 1) // 2)}",
                         xy=(tc, 0.98), xycoords=("data", "axes fraction"),
                         fontsize=7.5, rotation=90, va="top", ha="right",
                         color="#666666")

    axSpeed.set_xlabel("time [s]")
    axSpeed.set_ylabel("true wheel speed [mm/s]")
    axDuty.set_ylabel("commanded duty [-1..1]")
    axSpeed.axhline(0.0, color="black", lw=0.5)
    axSpeed.set_title(
        f"Square tour through Motion::Planner -- mismatched motors "
        f"(left gain +10%)\n"
        f"4x {LEG:.0f} mm legs @ {CRUISE:.0f} mm/s + 4x 90 deg turns; "
        f"path {path:.1f}/{4 * LEG:.0f} mm, heading "
        f"{headingDeg:.1f}/360 deg")
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
