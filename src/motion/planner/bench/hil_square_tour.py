#!/usr/bin/env python3
"""hil_square_tour.py -- the SQUARE TOUR on the REAL robot: 4 x 500 mm
legs + 4 x 90 deg turns, planned by the real Motion::Planner on the host
(HIL velocity bridge, the stable topology), executed by the pid-removal
firmware's interim Drive PID, measured by the REAL encoders.

Logs true wheel speeds (telemetry) vs the planner's commanded wheel
velocities and renders the same dual-trace plot as the simulation tour.
Robot on the stand, wheels free.

    uv run python src/motion/planner/bench/hil_square_tour.py \
        --port /dev/cu.usbmodem2121102
"""

import argparse
import ctypes
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "py"))
from planner_harness import (  # noqa: E402
    KIND_ANGLE, KIND_DISTANCE, VELOCITY_TWIST, Move, TickResult)
from hil_drive import PERIOD, BRIDGE_HOLD, BRIDGE_TIMEOUT, HilSession, hilLimits  # noqa: E402

TRACK = 128.0  # [mm]
LEG = 500.0    # [mm]
CRUISE = 150.0  # [mm/s]
TURN = math.pi / 2
OMEGA = 1.2    # [rad/s]


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
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    args = parser.parse_args()

    session = HilSession(args.port)
    time.sleep(4.0)  # pid-removal firmware boots slower; wait out the preamble
    session.proto.read_pending_binary_tlm_frames()

    moves = tourMoves()
    nextMove = 0
    result = TickResult()
    log = dict(t=[], velLeft=[], velRight=[], cmdLeft=[], cmdRight=[])
    completions = []
    encStart = None
    t0 = time.monotonic()
    try:
        for _ in range(3000):  # <= 150 s wall
            loopStart = time.monotonic()
            while nextMove < len(moves) and session.lib.plannerMove(
                    session.planner, ctypes.byref(moves[nextMove]), False):
                nextMove += 1
            if session.ingestTelemetry():
                if encStart is None:
                    encStart = (session.state.wheelLeft.position,
                                session.state.wheelRight.position)
                session.lib.plannerTick(session.planner,
                                        ctypes.byref(session.state),
                                        ctypes.byref(result))
                session.lib.plannerUpdate(session.planner,
                                          ctypes.byref(session.state))
                vL = session.state.wheelLeft.cmdVelocity
                vR = session.state.wheelRight.cmdVelocity
                if abs(vL) > 0.5 or abs(vR) > 0.5:
                    session.proto.move_wheels(v_left=vL, v_right=vR,
                                              stop_time=BRIDGE_HOLD,
                                              timeout=BRIDGE_TIMEOUT,
                                              replace=True)
                else:
                    session.proto.estop()
                tNow = time.monotonic() - t0
                log["t"].append(tNow)
                log["velLeft"].append(session.state.wheelLeft.velocity)
                log["velRight"].append(session.state.wheelRight.velocity)
                log["cmdLeft"].append(vL)
                log["cmdRight"].append(vR)
                if result.completed:
                    completions.append((tNow, result.moveId,
                                        bool(result.settled)))
                if (len(completions) == len(moves) and
                        abs(vL) < 0.5 and abs(vR) < 0.5):
                    break
            elapsed = time.monotonic() - loopStart
            time.sleep(max(0.0, PERIOD - elapsed))
    finally:
        session.proto.estop()
        time.sleep(0.4)
        session.ingestTelemetry()
        encEnd = (session.state.wheelLeft.position,
                  session.state.wheelRight.position)
        session.close()

    dL = encEnd[0] - encStart[0]
    dR = encEnd[1] - encStart[1]
    path = 0.5 * (dL + dR)
    headingDeg = math.degrees((dR - dL) / TRACK)
    settled = sum(1 for _, _, s in completions if s)
    print(f"HIL tour: {len(completions)}/{len(moves)} completed "
          f"({settled} settled) in {log['t'][-1]:.1f} s")
    print(f"  encoders: dL {dL:+.1f} dR {dR:+.1f} mm -> path {path:.1f} mm "
          f"(target {4 * LEG:.0f}; error {path - 4 * LEG:+.1f})")
    print(f"  heading {headingDeg:.1f} deg (target 360; "
          f"error {headingDeg - 360.0:+.1f})")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(14, 6))
    ax.plot(log["t"], log["velLeft"], color="#1f77b4", lw=1.3,
            label="left wheel speed [mm/s] (real encoder)")
    ax.plot(log["t"], log["velRight"], color="#2ca02c", lw=1.3,
            label="right wheel speed [mm/s] (real encoder)")
    ax.plot(log["t"], log["cmdLeft"], color="#1f77b4", lw=1.0, ls="--",
            alpha=0.6, label="left commanded [mm/s]")
    ax.plot(log["t"], log["cmdRight"], color="#2ca02c", lw=1.0, ls="--",
            alpha=0.6, label="right commanded [mm/s]")
    for tc, moveId, s in completions:
        ax.axvline(tc, color="#888888", lw=0.6, alpha=0.5)
        kind = "leg" if moveId % 2 == 1 else "turn"
        ax.annotate(f"{kind} {(moveId + 1) // 2}{'' if s else ' !'}",
                    xy=(tc, 0.98), xycoords=("data", "axes fraction"),
                    fontsize=7.5, rotation=90, va="top", ha="right",
                    color="#666666")
    lim = 1.1 * max(max(map(abs, log["velLeft"])),
                    max(map(abs, log["velRight"])), CRUISE)
    ax.set_ylim(-lim, lim)
    ax.axhline(0.0, color="black", lw=0.5)
    ax.axhline(CRUISE, color="#cccccc", lw=0.7, ls=":")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("wheel speed [mm/s]")
    ax.set_title(
        f"HIL square tour -- REAL motors/encoders, pid-removal firmware, "
        f"planner on host\npath {path:.1f}/{4 * LEG:.0f} mm, heading "
        f"{headingDeg:.1f}/360 deg, {settled}/{len(moves)} settled")
    ax.legend(loc="lower right", fontsize=8, framealpha=0.9)
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    out = Path(__file__).parent / "hil_square_tour.png"
    fig.savefig(out, dpi=130)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
