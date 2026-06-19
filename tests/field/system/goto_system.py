#!/usr/bin/env python3
"""goto_system.py — minimal single-G drive + live telemetry demo.

The simplest possible exercise of the robot driving interface, used as a lens
for improving it.  It:

  1. opens the robot in one of three modes — ``sim`` (in-process firmware),
     ``bench`` (real robot, sim OTOS), or ``production`` (real robot);
  2. issues ONE go-to-XY command (forward + left, robot-relative millimetres);
  3. streams every telemetry frame while it drives and prints each one.

No camera, no plotting — just the driving API and the telemetry it returns.

    uv run python tests/system/goto_system.py
    uv run python tests/system/goto_system.py --mode bench
    uv run python tests/system/goto_system.py --mode production --port /dev/cu.usbmodemXXXX
"""

from __future__ import annotations

import argparse
import math
import sys
import time

from robot_radio.testkit import make_target


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--mode", choices=["sim", "bench", "production"], default="sim",
                   help="Where to drive (default: sim)")
    p.add_argument("--forward", type=int, default=500,
                   help="Forward distance, mm (default 500 = 50 cm)")
    p.add_argument("--left", type=int, default=150,
                   help="Leftward distance, mm (default 150 = 15 cm)")
    p.add_argument("--speed", type=int, default=200,
                   help="Cruise speed, mm/s (default 200)")
    p.add_argument("--port", default=None,
                   help="Serial port for bench/production (auto-detect if omitted)")
    p.add_argument("--real-time", action="store_true",
                   help="(sim only) pace ticks to wall-clock time")
    return p.parse_args(argv)


def print_tick(robot) -> None:
    """Print one telemetry frame from the robot's live state.

    ``pose`` is the fused pose (encoder odometry + OTOS EKF); ``otos`` is the raw
    OTOS (optical odometry sensor) pose from the ``otos=`` field, or ``n/a`` when
    the firmware omitted it (OTOS read stale, e.g. lifted off the surface).  Both
    are ``(x, y, heading)`` — x/y in mm, heading in degrees.  Velocity is mm/s,
    yaw rate rad/s, encoders mm.  Units are dropped from the line for compactness.
    """
    s = robot.state
    enc = s.encoders or (0, 0)
    p = s.pose
    if s.otos_pose is not None:
        ox, oy, oh = s.otos_pose
        otos_str = f"({ox:7.1f},{oy:7.1f},{math.degrees(oh):+6.1f})"
    else:
        otos_str = f"({'n/a':^22})"
    print(
        f"  pose=({p.x:7.1f},{p.y:7.1f},{math.degrees(p.heading):+6.1f})  "
        f"otos={otos_str}  "
        f"v={s.v:6.1f} w={s.omega:+5.2f}  "
        f"enc=({enc[0]:>5},{enc[1]:>5})  "
        f"line={s.line} color={s.color}"
    )


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)

    # Open + connect (PING/ID liveness preflight; raises if the robot is silent).
    tr = make_target(args.mode, real_time=args.real_time, port=args.port)
    robot = tr.robot
    ident = robot.connect()
    print(f"[{args.mode}] connected: {ident}")

    # Synchronize ALL odometry subsystems to (0,0,0) at the start so every run is
    # deterministic and follows roughly the same path:
    #   OZ            — zero the OTOS chip's own raw tracking (it persists across
    #                   resets, so the boot-zero isn't enough once the robot has
    #                   been placed/moved before the run).
    #   ZERO enc pose — reset the encoder accumulators AND the fused EKF pose.
    # The robot is still here (before the G move), which the OTOS/heading zero
    # requires for a clean reset.
    robot.send("OZ")
    robot.send("ZERO enc pose")
    time.sleep(0.3)  # let the zero settle before streaming/driving

    print(f"G  forward={args.forward}mm  left={args.left}mm  @ {args.speed}mm/s\n")

    outcome = "error"
    try:
        l_enc, r_enc, outcome = robot.go_to(
            args.forward, args.left, args.speed, on_tick=print_tick,
        )
        print(f"\noutcome={outcome}  final encoders=({l_enc},{r_enc})mm")
    finally:
        robot.stop()
        try:
            tr.conn.disconnect()
        except Exception:
            pass

    # "settled" = robot moved then stopped (used when EVT done was dropped over
    # radio); treat it as a clean completion alongside the EVT-confirmed "done".
    return 0 if outcome in ("done", "settled") else 1


if __name__ == "__main__":
    sys.exit(main())
