#!/usr/bin/env python3
"""square_tour_velocity.py -- RETIRED (130-005).

This script exercised Motion::WheelTrim, the planner-side velocity-domain
closed loop:

    planner profile  -> + velocity trim  -> cmdVelocity   [mm/s]
      -> Drive's calibrated inverse map  -> duty          [-1, 1]
        -> motor / gearbox / wheel                        [mm/s]

Per `wheel-speed-controller-moves-into-drive.md` Phase 3 (DECIDED,
stakeholder 2026-08-01), that controller moved wholesale into App::Drive --
Motion::Planner sheds ALL wheel-actuation code, `Motion::WheelTrim` is
deleted outright (no redirect stub), and its `applyTrimGains()`/
`trimLeft()`/`trimRight()`/`trimIntegralLeft()`/`trimIntegralRight()` are
gone from the C++ API this script's ctypes harness bound. Setting
`PlannerLimits.trimKp` et al. (`tourLimits()` below) is now a silent no-op
-- the constructor no longer reads those fields into anything -- so this
script would not fail loudly, it would just quietly stop exercising what
its own docstring claims to measure. Retired rather than left running:

  - The "old additive trim" baseline this script's own CSVs already
    captured (`square_tour_velocity_trim.csv`/`_trim_sym.csv`, this
    directory) IS the historical A/B comparison point ticket 130-006's
    bench acceptance needs -- they are kept, not deleted.
  - A successor bench script exercising App::Drive's Stage B/C controller
    directly (bias/fast-PID observability, now on Drive's own accessors
    and the wire Telemetry frame -- see drive.h/telemetry.proto) is
    ticket 130-006's job, on real hardware.

Historical docstring (for context, no longer accurate as a usage guide):
WHY A SECOND TOUR SCRIPT. square_tour_sim.py drives the plant from the
planner's DUTY output, which the robot does not use -- RobotLoop stages
`state.wheel*.cmdVelocity` and App::Drive converts it through its own
calibrated map. THE DISTURBANCE. Drive was given ONE shared duty-per-speed
constant while the two gearboxes differ (+-4.8% here, deliberately larger
than the ~2% measured on the real robot) so the effect is visible in one
tour -- that residual is precisely what no feedforward can remove and what
the (now-relocated) controller exists to close. SCHEDULE FIDELITY. The
plant ran continuously (1 ms substeps) while the loop's events land at
their real within-cycle offsets, so the two encoders are genuinely read
8 ms apart and each duty write lands when it really does -- plant model
shared with square_tour_sim.py.
"""

import argparse
import ctypes
import csv
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))  # 128-010: planner_harness.py is co-located
from planner_harness import (  # noqa: E402
    KIND_ANGLE, KIND_DISTANCE, VELOCITY_TWIST, Move, MovePhase, PlannerLimits,
    RobotState, TickResult, loadLibrary)

import square_tour_sim  # noqa: E402
from square_tour_sim import (  # noqa: E402
    CYCLE, GAIN_LEFT, GAIN_NOMINAL, GAIN_RIGHT, QUANTUM, SUBSTEP,
    T_COLLECT_L, T_COLLECT_R, T_TICK, T_WRITE_L, T_WRITE_R, TAU, TRACK,
    VEL_NOISE, SimWheel)

# Static-friction floor, from src/tests/firmware/duty_min/RESULTS.md
# (standalone probe, 2026-07-27): the TRUE dead zone is 1-6% duty per
# wheel and wanders with usage state; 6% is the worst case observed.
#
# NOT square_tour_sim.py's 0.18 -- that value is PlannerLimits::dutyFloor,
# the settling-creep stiction KICK, and using it as a general breakaway
# would mean a wheel could not start below ~246 mm/s, which would make
# this tour's own 150 mm/s cruise unreachable from rest.
BREAKAWAY = 0.06     # [duty]
square_tour_sim.BREAKAWAY = BREAKAWAY  # SimWheel reads the module global

LEG = 500.0          # [mm]
CRUISE = 150.0       # [mm/s] matches the hardware tour
TURN = math.pi / 2   # [rad]
OMEGA = 1.2          # [rad/s] matches the hardware tour
STOP_ID = 99         # move id of the tour's closing planned stop

# Decel leeway (PlannerLimits::decelPlanFraction): plan the brake START at
# 40% of the decel ceiling, holding the rest in reserve. Swept 0.0-0.6 on
# this tour: 0.4 gave the best closure (16.0 mm vs 23.3 mm at full
# authority) because the plant can actually TRACK the gentler ramp down,
# so it arrives at each boundary already slow instead of coasting past.
DECEL_LEEWAY = 0.4   # [1]

# --- App::Drive's velocity -> duty map, mirrored ------------------------
# Drive holds ONE shared duty-per-speed constant plus a per-wheel,
# per-direction affine correction. Here the correction is identity (a
# freshly calibrated robot) and the shared constant is the mean of the two
# gearboxes -- so each wheel carries half the mismatch as a residual.
DUTY_PER_SPEED = 1.0 / GAIN_NOMINAL  # [duty/(mm/s)]


# NezhaMotor::writeShapedDuty()'s output deadband: a nonzero duty below
# this is BOOSTED up to it before it reaches the bus. Without this the
# model silently swallows small commands, and a profile that lands gently
# stalls dead just short of its target -- which is exactly the terminal
# stall this project has hit on real hardware.
OUTPUT_DEADBAND = 0.03  # [-1,1] data/robots/tovez.json control.output_deadband


def driveDuty(desired: float, previous: float) -> float:
    """App::Drive::tick() + NezhaMotor::writeShapedDuty()'s deadband boost,
    for one wheel. `previous` selects the accel vs decel correction row
    exactly as Drive does."""
    if desired == 0.0:
        return 0.0  # stop is stop; never offset it
    # Identity correction (gain 1, intercept 0) -- a calibrated robot. The
    # row selection is kept so the shape of the real code is preserved.
    _row = 0 if abs(desired) > abs(previous) else 1
    duty = desired * DUTY_PER_SPEED
    if 0.0 < abs(duty) < OUTPUT_DEADBAND:
        duty = math.copysign(OUTPUT_DEADBAND, duty)
    return duty


def tourLimits(trim: bool, leeway: float = DECEL_LEEWAY, kaff: float = 0.5
               ) -> PlannerLimits:
    limits = PlannerLimits()
    limits.vMax = 400.0
    limits.aMax = 300.0
    limits.aDecel = 250.0
    limits.omegaMax = 3.0
    limits.alphaMax = 6.0
    limits.alphaDecel = 5.0
    limits.trackWidth = TRACK
    limits.controlPeriod = float(CYCLE)
    limits.actuationDelay = float(CYCLE)  # [ms] command staged at next cycle
    limits.velocityFilterWeight = 0.35
    limits.otosStaleness = 200
    limits.headingOtosWeight = 0.0
    limits.requireSettle = False
    # Rest floors for the closing planned stop. These bound the residual
    # coast: the stop completes once the FILTERED measured speed is under
    # the floor, and the plant then coasts a further ~v*tau. At 0.16 rad/s
    # with tau 0.23 s that is ~2 deg of uncorrected rotation, which was
    # most of this tour's final heading error. Sized just above the
    # filtered velocity-noise floor instead.
    limits.settleRestVelocity = 6.0   # [mm/s]
    limits.settleRestOmega = 0.06     # [rad/s]
    limits.settleWindow = 2500.0
    limits.dutyFloor = BREAKAWAY
    # Outer heading loop, on top of the trim's inner wheel-rate loop.
    #
    # These are NOT redundant, which was worth measuring rather than
    # assuming: the trim holds each wheel to its COMMANDED RATE, but the
    # integrator that does the real work is gated to the hold phase, and
    # ~31% of this tour is spent in accel/decel ramps where proportional
    # action alone leaves a few percent of gain error uncorrected. That
    # residual integrates into ANGLE, which no rate loop can retire. The
    # heading loop closes on the accumulated angle itself.
    #
    # (The ratio lock is not a third corrector -- it constrains the
    # commanded pair and never reads a measurement.)
    limits.headingHoldGain = 2.0 if trim else 0.0
    # Duty stage OFF -- this tour drives the velocity plane.
    limits.velKff = 0.0
    limits.velKp = 0.0
    limits.velKi = 0.0
    limits.velIMax = 0.0
    limits.velKaff = 0.0
    limits.velIAccelGate = 1.0e9
    limits.jerkMax = 1500.0    # [mm/s^3] aMax reached in ~0.2 s
    limits.yawJerkMax = 30.0   # [rad/s^3]
    limits.decelPlanFraction = leeway
    if trim:
        # Commissioning gains: kp well under 1 (measured wheel velocity is
        # a raw difference quotient), kaff at half the measured plant tau,
        # and a bounded trim authority so feedback trims the profile
        # rather than replacing it.
        limits.trimKp = 0.25       # [1]
        limits.trimKi = 0.5        # [1/s]
        limits.trimIMax = 60.0     # [mm/s]
        limits.trimKaff = TAU * kaff  # [s]
        limits.trimMax = 120.0     # [mm/s]
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


def runTour(trim: bool, symmetric: bool, leeway: float = DECEL_LEEWAY,
            kaff: float = 0.5) -> dict:
    lib = loadLibrary()
    limits = tourLimits(trim, leeway, kaff)
    planner = lib.plannerCreate(ctypes.byref(limits))
    state = RobotState()
    result = TickResult()

    gainLeft = GAIN_NOMINAL if symmetric else GAIN_LEFT
    gainRight = GAIN_NOMINAL if symmetric else GAIN_RIGHT
    left = SimWheel(gainLeft)
    right = SimWheel(gainRight)

    profiledLeft = ctypes.c_float()
    profiledRight = ctypes.c_float()
    trimLeft = ctypes.c_float()
    trimRight = ctypes.c_float()
    integralLeft = ctypes.c_float()
    integralRight = ctypes.c_float()
    phase = ctypes.c_uint8()

    moves = tourMoves()
    nextMove = 0
    stopQueued = [False]
    completions = []  # (t_s, moveId)

    log = dict(t=[], velLeft=[], velRight=[],
               measLeft=[], measRight=[], cmdLeft=[], cmdRight=[],
               profLeft=[], profRight=[], trimLeft=[], trimRight=[],
               phase=[], x=[], y=[], heading=[])

    # Ground-truth pose, integrated arc-exactly from TRUE wheel travel.
    trueX = trueY = trueHeading = 0.0
    lastLeftPos = lastRightPos = 0.0

    stopDone = [False]
    pendingDutyL = pendingDutyR = 0.0
    cmdL = cmdR = 0.0
    previousCmdL = previousCmdR = 0.0
    settledCycles = 0

    def takeSample(wheel, plant, sampleTime: int, parity: int) -> None:
        wheel.position = round(plant.position / QUANTUM) * QUANTUM
        noise = VEL_NOISE if parity % 2 == 0 else -VEL_NOISE
        wheel.velocity = plant.velocity + noise
        wheel.sampleTime = sampleTime
        wheel.connected = True

    takeSample(state.wheelLeft, left, 0, 0)
    takeSample(state.wheelRight, right, 0, 1)

    for cycle in range(1500):
        t0 = cycle * CYCLE  # [ms]
        while nextMove < len(moves) and lib.plannerMove(
                planner, ctypes.byref(moves[nextMove]), False):
            nextMove += 1
        # Close the tour with a PLANNED STOP -- an ordinary queue entry
        # that completes only once the body is actually at rest. Without
        # it the last turn's completion is the end of the run and the
        # measurement is taken mid-coast: the plant's ~230 ms time
        # constant carries the robot several degrees past the point the
        # profile landed on, which reads as tour error but is really "we
        # stopped measuring before the robot stopped moving."
        if nextMove >= len(moves) and not stopQueued[0]:
            if lib.plannerPlannedStop(planner, STOP_ID):
                stopQueued[0] = True

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
                lib.plannerTrim(planner, ctypes.byref(profiledLeft),
                                ctypes.byref(profiledRight),
                                ctypes.byref(trimLeft),
                                ctypes.byref(trimRight),
                                ctypes.byref(integralLeft),
                                ctypes.byref(integralRight),
                                ctypes.byref(phase))
                previousCmdL, previousCmdR = cmdL, cmdR
                cmdL = state.wheelLeft.cmdVelocity
                cmdR = state.wheelRight.cmdVelocity
                # THE HARDWARE PATH: velocity command -> Drive -> duty.
                pendingDutyL = driveDuty(cmdL, previousCmdL)
                pendingDutyR = driveDuty(cmdR, previousCmdR)
                if result.completed and result.moveId == STOP_ID:
                    stopDone[0] = True
                if result.completed and result.moveId != STOP_ID:
                    completions.append(dict(t=tMs / 1000.0,
                                            moveId=result.moveId,
                                            x=trueX, y=trueY,
                                            heading=trueHeading))
            elif offset == T_WRITE_L:
                left.write(pendingDutyL)
            elif offset == T_WRITE_R:
                right.write(pendingDutyR)

            left.substep(SUBSTEP)
            right.substep(SUBSTEP)

            # Ground-truth pose from true wheel travel (arc-exact).
            dLeft = left.position - lastLeftPos
            dRight = right.position - lastRightPos
            lastLeftPos, lastRightPos = left.position, right.position
            ds = 0.5 * (dLeft + dRight)
            dTheta = (dRight - dLeft) / TRACK
            midHeading = trueHeading + 0.5 * dTheta
            trueX += ds * math.cos(midHeading)
            trueY += ds * math.sin(midHeading)
            trueHeading += dTheta

            log["t"].append(tMs / 1000.0)
            log["velLeft"].append(left.velocity)
            log["velRight"].append(right.velocity)
            # What the CONTROLLER sees: the last encoder sample published
            # into the blackboard (noisy, held between collects) -- distinct
            # from the plant's true velocity above.
            log["measLeft"].append(state.wheelLeft.velocity)
            log["measRight"].append(state.wheelRight.velocity)
            log["cmdLeft"].append(cmdL)
            log["cmdRight"].append(cmdR)
            log["profLeft"].append(profiledLeft.value)
            log["profRight"].append(profiledRight.value)
            log["trimLeft"].append(trimLeft.value)
            log["trimRight"].append(trimRight.value)
            log["phase"].append(phase.value)
            log["x"].append(trueX)
            log["y"].append(trueY)
            log["heading"].append(trueHeading)

        if stopDone[0]:
            # Keep integrating well past the stop (~4 plant time constants)
            # so the reported pose is the one a camera would see, not one
            # taken mid-coast.
            settledCycles += 1
            if settledCycles > 20:
                break

    lib.plannerDestroy(planner)
    return dict(log=log, completions=completions, x=trueX, y=trueY,
                heading=trueHeading, trim=trim, symmetric=symmetric,
                gainLeft=gainLeft, gainRight=gainRight)


def report(run: dict) -> dict:
    """Closure error: how far from the start the robot finished, and how
    far its final heading is from a full 360 deg turn. Plus the per-move
    breakdown -- a single closure number hides whether the error came from
    the legs, the turns, or a veer during the legs."""
    closure = math.hypot(run["x"], run["y"])                     # [mm]
    headingError = math.degrees(run["heading"] - 2.0 * math.pi)  # [deg]
    perimeter = 4.0 * LEG

    moves = []
    prevX = prevY = prevHeading = 0.0
    for entry in run["completions"]:
        travelled = math.hypot(entry["x"] - prevX, entry["y"] - prevY)
        turned = math.degrees(entry["heading"] - prevHeading)
        isLeg = entry["moveId"] % 2 == 1
        moves.append(dict(
            moveId=entry["moveId"], t=entry["t"], isLeg=isLeg,
            travelled=travelled, turned=turned,
            # A leg wants LEG mm of travel and zero net turn; a turn wants
            # 90 deg of rotation and zero net translation.
            error=(travelled - LEG) if isLeg else (turned - 90.0),
            crossError=turned if isLeg else travelled))
        prevX, prevY, prevHeading = entry["x"], entry["y"], entry["heading"]

    return dict(closure=closure, headingError=headingError,
                closurePercent=100.0 * closure / perimeter,
                finalX=run["x"], finalY=run["y"],
                finalHeading=math.degrees(run["heading"]), moves=moves)


def writeChart(run: dict, stats: dict, out: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    log = run["log"]
    fig = plt.figure(figsize=(15, 11))
    grid = fig.add_gridspec(3, 2, height_ratios=[1.25, 1.25, 1.0],
                            hspace=0.34, wspace=0.22)

    # --- Panel 1 (full width): wheel speeds, commanded vs actual --------
    ax = fig.add_subplot(grid[0, :])
    phaseColors = {MovePhase.ACCEL: "#e8f4ff", MovePhase.HOLD: "#eafaea",
                   MovePhase.DECEL: "#fff0e6"}
    # Shade the accel / hold / decel bands behind the traces.
    start, current = 0, log["phase"][0]
    for i in range(1, len(log["phase"]) + 1):
        atEnd = i == len(log["phase"])
        if atEnd or log["phase"][i] != current:
            if current in phaseColors:
                ax.axvspan(log["t"][start], log["t"][i - 1],
                           color=phaseColors[current], zorder=0, lw=0)
            if not atEnd:
                start, current = i, log["phase"][i]
    ax.plot(log["t"], log["measLeft"], color="#9ecae9", lw=0.8,
            label="left measured (encoder)", zorder=2)
    ax.plot(log["t"], log["measRight"], color="#f2b8b5", lw=0.8,
            label="right measured (encoder)", zorder=2)
    ax.plot(log["t"], log["cmdLeft"], color="#1f77b4", ls="--", lw=1.1,
            label="left commanded", zorder=3)
    ax.plot(log["t"], log["velLeft"], color="#1f77b4", lw=1.7,
            label="left true (plant)", zorder=4)
    ax.plot(log["t"], log["cmdRight"], color="#d62728", ls="--", lw=1.1,
            label="right commanded", zorder=3)
    ax.plot(log["t"], log["velRight"], color="#d62728", lw=1.7,
            label="right true (plant)", zorder=4)
    for entry in run["completions"]:
        tSec, moveId = entry["t"], entry["moveId"]
        ax.axvline(tSec, color="#999999", lw=0.7, ls=":", zorder=1)
        ax.annotate(f"{'leg' if moveId % 2 else 'turn'} {(moveId + 1) // 2}",
                    xy=(tSec, ax.get_ylim()[1]), xytext=(2, -10),
                    textcoords="offset points", rotation=90, fontsize=7,
                    color="#666666", va="top")
    ax.axhline(0.0, color="#cccccc", lw=0.6, zorder=1)
    ax.set_ylabel("wheel speed  [mm/s]")
    ax.set_xlabel("time  [s]")
    ax.legend(loc="upper right", fontsize=8, ncol=3)
    ax.grid(alpha=0.25)
    ax.set_title("Wheel speed: commanded vs actual   "
                 "(shading: blue = accel, green = hold, orange = decel)",
                 fontsize=10)

    # --- Panel 2: the motion --------------------------------------------
    ax2 = fig.add_subplot(grid[1, 0])
    ax2.plot(log["x"], log["y"], color="#2b8a3e", lw=1.8, zorder=3)
    ideal = [(0, 0), (LEG, 0), (LEG, LEG), (0, LEG), (0, 0)]
    ax2.plot([p[0] for p in ideal], [p[1] for p in ideal], color="#bbbbbb",
             ls="--", lw=1.0, zorder=2, label="ideal square")
    ax2.plot([0], [0], marker="o", color="#2b8a3e", ms=7, zorder=4,
             label="start")
    ax2.plot([run["x"]], [run["y"]], marker="X", color="#d62728", ms=10,
             zorder=5, label="finish")
    ax2.annotate(f"closure {stats['closure']:.1f} mm",
                 xy=(run["x"], run["y"]), xytext=(10, -14),
                 textcoords="offset points", fontsize=9, color="#d62728")
    ax2.set_aspect("equal", adjustable="datalim")
    ax2.set_xlabel("x  [mm]")
    ax2.set_ylabel("y  [mm]")
    ax2.legend(loc="best", fontsize=8)
    ax2.grid(alpha=0.25)
    ax2.set_title("Path (ground truth)", fontsize=10)

    # --- Panel 3: the trim ----------------------------------------------
    ax3 = fig.add_subplot(grid[1, 1])
    ax3.plot(log["t"], log["trimLeft"], color="#1f77b4", lw=1.2,
             label="left trim")
    ax3.plot(log["t"], log["trimRight"], color="#d62728", lw=1.2,
             label="right trim")
    ax3.axhline(0.0, color="#cccccc", lw=0.6)
    ax3.set_xlabel("time  [s]")
    ax3.set_ylabel("velocity trim  [mm/s]")
    ax3.legend(loc="upper right", fontsize=8)
    ax3.grid(alpha=0.25)
    ax3.set_title("Closed-loop correction added to the profile"
                  if run["trim"] else "Trim DISABLED (open loop)",
                  fontsize=10)

    # --- Panel 4 (full width): tracking error ---------------------------
    ax4 = fig.add_subplot(grid[2, :])
    errLeft = [c - v for c, v in zip(log["cmdLeft"], log["velLeft"])]
    errRight = [c - v for c, v in zip(log["cmdRight"], log["velRight"])]
    ax4.plot(log["t"], errLeft, color="#1f77b4", lw=0.9, label="left error")
    ax4.plot(log["t"], errRight, color="#d62728", lw=0.9, label="right error")
    ax4.axhline(0.0, color="#cccccc", lw=0.6)
    ax4.set_xlabel("time  [s]")
    ax4.set_ylabel("commanded - actual  [mm/s]")
    ax4.legend(loc="upper right", fontsize=8)
    ax4.grid(alpha=0.25)
    ax4.set_title("Velocity tracking error", fontsize=10)

    mode = ("trim ON" if run["trim"] else "trim OFF")
    plant = ("symmetric plant" if run["symmetric"]
             else f"asymmetric plant (L {run['gainLeft']:.0f} / "
                  f"R {run['gainRight']:.0f} mm/s per duty)")
    fig.suptitle(
        f"Square tour, velocity plane -- {mode}, {plant}\n"
        f"closure {stats['closure']:.1f} mm "
        f"({stats['closurePercent']:.2f}% of the {4 * LEG:.0f} mm perimeter)   "
        f"heading error {stats['headingError']:+.2f} deg",
        fontsize=12)
    fig.savefig(out, dpi=130, bbox_inches="tight")
    plt.close(fig)


def writeCsv(run: dict, out: Path) -> None:
    log = run["log"]
    columns = ["t", "phase", "profLeft", "profRight", "trimLeft", "trimRight",
               "cmdLeft", "cmdRight", "measLeft", "measRight",
               "velLeft", "velRight", "x", "y", "heading"]
    with out.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(columns)
        for i in range(len(log["t"])):
            writer.writerow([f"{log[c][i]:.5g}" if c != "phase"
                             else MovePhase.NAMES[log[c][i]] for c in columns])


def main() -> None:
    sys.exit(
        "square_tour_velocity.py is RETIRED (130-005): it exercised "
        "Motion::WheelTrim, deleted outright when the wheel-speed "
        "controller moved into App::Drive. See this file's own module "
        "docstring for the historical baseline CSVs and the ticket "
        "130-006 successor. Not run.")

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--no-trim", action="store_true",
                        help="disable the velocity trim (open loop)")
    parser.add_argument("--symmetric", action="store_true",
                        help="matched gearboxes (no wheel-gain mismatch)")
    parser.add_argument("--label", default=None, help="output filename stem")
    args = parser.parse_args()

    trim = not args.no_trim
    run = runTour(trim=trim, symmetric=args.symmetric)
    stats = report(run)

    label = args.label or (
        f"square_tour_velocity_{'trim' if trim else 'open'}"
        f"{'_sym' if args.symmetric else ''}")
    here = Path(__file__).resolve().parent
    chart = here / f"{label}.png"
    csvOut = here / f"{label}.csv"
    writeChart(run, stats, chart)
    writeCsv(run, csvOut)

    print(f"=== square tour, velocity plane "
          f"[{'trim ON' if trim else 'trim OFF'}, "
          f"{'symmetric' if args.symmetric else 'asymmetric'} plant] ===")
    print(f"  moves completed  : {len(run['completions'])} / 8")
    print()
    print("  per-move error report")
    print("    move        target        achieved       error    cross-axis")
    for m in stats["moves"]:
        index = (m["moveId"] + 1) // 2
        if m["isLeg"]:
            print(f"    leg  {index}    {LEG:7.1f} mm   {m['travelled']:8.2f} mm"
                  f"   {m['error']:+7.2f} mm   {m['crossError']:+6.2f} deg veer")
        else:
            print(f"    turn {index}    {90.0:7.1f} deg  {m['turned']:8.2f} deg"
                  f"   {m['error']:+7.2f} deg  {m['crossError']:+6.2f} mm drift")
    legErrors = [abs(m["error"]) for m in stats["moves"] if m["isLeg"]]
    turnErrors = [abs(m["error"]) for m in stats["moves"] if not m["isLeg"]]
    print(f"    worst leg  |error| {max(legErrors):6.2f} mm      "
          f"worst turn |error| {max(turnErrors):5.2f} deg")
    print()
    print(f"  final position   : "
          f"({stats['finalX']:+8.2f}, {stats['finalY']:+8.2f}) mm")
    print(f"  closure error    : {stats['closure']:8.2f} mm  "
          f"({stats['closurePercent']:.2f}% of {4 * LEG:.0f} mm perimeter)")
    print(f"  final heading    : {stats['finalHeading']:8.2f} deg "
          f"(ideal 360.00)")
    print(f"  heading error    : {stats['headingError']:+8.2f} deg")
    print(f"  chart            : {chart}")
    print(f"  per-tick csv     : {csvOut}")


if __name__ == "__main__":
    main()
