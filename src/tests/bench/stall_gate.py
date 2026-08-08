#!/usr/bin/env python
"""Prove the firmware's stall detector on real hardware, end to end.

Stakeholder directive 2026-08-08: "implement a stall detector... stop the
motors on detecting a stall... a little telemetry flag that says you've
stalled... because you keep running into the rails and stalling."

WHAT A STALL IS, and what it is not. The firmware distinguishes three wheel
faults and only one of them halts the robot:

    wheelFrozen (flags 19/20)  the ENCODER stopped reporting. A sensor fault
                               -- the wheel may be spinning perfectly well.
    deficit     (flags 21/22)  the wheel IS turning, just slower than asked.
                               Reports only.
    STALL       (flags 24/25)  commanded to move, encoders say it is not
                               moving, and the encoder is healthy enough to
                               be believed. The wheel is physically held.
                               THIS ONE STOPS THE ROBOT.

TWO CHECKS, because they prove different things:

  --mechanism (default, hands-off)  Live-pushes an absurd stall.speed so that
      a freely spinning wheel reads as "not turning", then commands a bounded
      PIVOT. Proves detect -> halt -> flag -> latch and the host decode, with
      nobody touching the robot. It does NOT prove that a real jam produces
      the condition -- it manufactures the condition.

  --physical                        Baked thresholds, no pushes. Commands a
      bounded pivot and waits for a HUMAN to hold a wheel. This is the one
      that proves the real thing.

WHY A PIVOT and not a straight leg: the robot lives on the playfield, and a
pivot rotates in place -- it cannot drive off the mat while this test runs.
Every Move here is additionally bounded by its own stop condition and
timeout, so a detector that does nothing at all still stops the robot.

The mechanism run RESTORES stall.speed in a finally block. A live push is
wiped by a reboot anyway, but leaving an absurd threshold live would mean
the robot halts on every subsequent command until someone noticed.

Usage:
    uv run python src/tests/bench/stall_gate.py --port /dev/cu.usbmodemXXXX
    uv run python src/tests/bench/stall_gate.py --port ... --physical
"""
from __future__ import annotations

import argparse
import sys
import time

sys.path.insert(0, "src/tests/bench")

from robot_radio.io.serial_conn import SerialConnection  # noqa: E402
from robot_radio.robot.pb2 import robot_config_pb2  # noqa: E402
from robot_radio.robot.protocol import NezhaProtocol, TLMFrame  # noqa: E402

PIVOT_OMEGA = 1.0        # [rad/s]
PIVOT_ANGLE = 6.0        # [rad] deliberately long: the halt should end this,
                         # not the stop condition. If the detector does
                         # nothing, the timeout below still bounds it.
PIVOT_TIMEOUT = 4000.0   # [ms]
WATCH = 6.0              # [s] how long to watch for the stall after commanding
PHYSICAL_WATCH = 25.0    # [s] longer -- a human has to reach in and grab a wheel

ABSURD_SPEED = 1000.0    # [mm/s] any real wheel speed is below this


def newest(conn):
    last = None
    for env in conn.drain_binary_tlm():
        t = getattr(env, "tlm", None)
        if t is not None:
            last = TLMFrame.from_pb2(t)
    return last


def wait_frame(conn, timeout=3.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        f = newest(conn)
        if f is not None:
            return f
        time.sleep(0.02)
    return None


def halt_verified(p, conn, tries=5) -> bool:
    """estop until the ENCODERS confirm it. An ack is not evidence: the Nezha
    brick latches its last commanded speed and a lost zero write is permanent
    (.claude/rules/playfield-testing.md)."""
    for _ in range(tries):
        try:
            p.estop()
        except Exception:
            pass
        time.sleep(0.3)
        a = wait_frame(conn)
        time.sleep(0.4)
        b = wait_frame(conn)
        if (a and b and a.enc and b.enc
                and abs(b.enc[0] - a.enc[0]) <= 3
                and abs(b.enc[1] - a.enc[1]) <= 3):
            return True
    return False


def push_and_verify(p, field: str, value: float, tries: int = 4) -> bool:
    """Push one config field and READ IT BACK. An ack is not proof the value
    landed -- config that acks OK and applies nowhere is a live failure mode
    in this codebase (.claude/rules/configuration-discipline.md). Retried
    because the relay DROPS inbound commands outright, so one attempt is one
    attempt, not one delivery."""
    for attempt in range(tries):
        p.set_config_field(robot_config_pb2.WHEEL_CONTROL, field, value)
        time.sleep(0.4)
        cfg = p.get_config(robot_config_pb2.WHEEL_CONTROL)
        got = getattr(cfg, field, None) if cfg is not None else None
        if got is not None and abs(got - value) < 1e-3:
            print(f"  {field} -> pushed {value}, read back {got}  OK")
            return True
        print(f"  {field} -> pushed {value}, read back {got} "
              f"-- retrying ({attempt + 1}/{tries})")
    return False


# --wall: the real thing. Drive into the playfield rail and let the detector
# be what stops the robot. Everything else here manufactures the condition;
# this one produces it the way the rails produce it in normal use.
#
# Deliberately REVERSE, chosen from the geometry at the time this was written
# rather than baked in: the robot sat at x=+51cm facing WNW with the east rail
# ~16cm behind it, so backing into it is a ~7cm run (the body reaches the rail
# before the centre travels the full 16). Driving forward would have meant
# crossing the entire field first. Re-check the pose before assuming reverse
# is still the short way to a wall.
WALL_SPEED = -80.0       # [mm/s] negative = reverse; |80| clears stall_demand 40
WALL_DISTANCE = 250.0    # [mm] deliberately PAST the rail, so the robot keeps
                         # pushing and the detector has something to detect
WALL_TIMEOUT = 4000.0    # [ms] backstop if the detector does nothing at all
WALL_WATCH = 8.0         # [s]


def run_pivot_and_watch(p, conn, watch: float, banner: str,
                        wall: bool = False):
    """Command a bounded move, then watch for the stall flag and a halt."""
    print(banner)
    conn.drain_binary_tlm()
    before = wait_frame(conn)
    enc0 = before.enc if before else None

    if wall:
        p.move_twist(WALL_SPEED, 0.0, 0.0, stop_distance=WALL_DISTANCE,
                     timeout=WALL_TIMEOUT, move_id=0)
    else:
        p.move_twist(0.0, 0.0, PIVOT_OMEGA, stop_angle=PIVOT_ANGLE,
                     timeout=PIVOT_TIMEOUT, move_id=0)

    t0 = time.time()
    stalled_at = None
    last = before
    while time.time() - t0 < watch:
        f = newest(conn)
        if f is not None:
            last = f
            if f.stalled and stalled_at is None:
                stalled_at = time.time() - t0
                print(f"  STALL reported at t={stalled_at:.2f}s  "
                      f"L={f.fault_stall_left} R={f.fault_stall_right}")
        time.sleep(0.02)

    # Did the wheels actually stop? Encoders, not the flag.
    a = wait_frame(conn)
    time.sleep(1.0)
    b = wait_frame(conn)
    moved = None
    if a and b and a.enc and b.enc:
        moved = (abs(b.enc[0] - a.enc[0]), abs(b.enc[1] - a.enc[1]))

    print(f"  encoders at start {enc0} -> end {last.enc if last else None}")
    print(f"  post-halt encoder movement over 1s: {moved}")
    print(f"  flags 0x{last.flags:08x}" if last else "  no frames")
    return stalled_at, moved, last


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True)
    ap.add_argument("--physical", action="store_true",
                    help="real jam test: baked thresholds, a human holds a wheel")
    ap.add_argument("--wall", action="store_true",
                    help="THE REAL TEST: drive into the playfield rail at baked "
                         "thresholds and let the stall detector be what stops "
                         "the robot")
    args = ap.parse_args()

    conn = SerialConnection(port=args.port)
    conn.connect(skip_ping=False)
    p = NezhaProtocol(conn)
    pushed = False
    failures = []
    try:
        conn.send_cleartext("TLM:ON")
        time.sleep(0.8)
        conn.drain_binary_tlm()

        cfg = p.get_config(robot_config_pb2.WHEEL_CONTROL)
        if cfg is None:
            print("FAIL: cannot read WHEEL_CONTROL config")
            return 2
        print(f"baked stall config: speed={cfg.stall_speed} "
              f"demand={cfg.stall_demand} window={cfg.stall_window}")
        if cfg.stall_window <= 0:
            print("FAIL: stall_window is 0 -- the detector is DISABLED in the "
                  "baked config, so nothing below can pass")
            return 2

        if not halt_verified(p, conn):
            print("FAIL: could not confirm the robot is stopped before starting")
            return 2

        if args.wall:
            print("\n=== WALL: baked thresholds, driving into the rail ===")
            stalled_at, moved, last = run_pivot_and_watch(
                p, conn, WALL_WATCH,
                f"reversing at {abs(WALL_SPEED):.0f}mm/s into the rail",
                wall=True)
            if stalled_at is None:
                failures.append("no stall reported -- the robot hit the rail "
                                "and the detector did not notice")
            if moved and max(moved) > 3:
                failures.append(f"wheels still moving after the halt: {moved}")
            if last is not None and not last.stalled:
                failures.append("stall flag did not LATCH")
        elif args.physical:
            print("\n=== PHYSICAL: baked thresholds, real jam ===")
            print(f"HOLD A WHEEL when the robot starts turning. "
                  f"{PHYSICAL_WATCH:.0f}s window.")
            stalled_at, moved, last = run_pivot_and_watch(
                p, conn, PHYSICAL_WATCH,
                "commanding pivot -- grab a wheel now")
            if stalled_at is None:
                failures.append("no stall reported during the physical hold")
            if moved and max(moved) > 3:
                failures.append(f"wheels still moving after the halt: {moved}")
        else:
            print("\n=== MECHANISM: absurd stall.speed, hands off ===")
            if not push_and_verify(p, "stall_speed", ABSURD_SPEED):
                print("FAIL: could not push/verify stall.speed")
                return 2
            pushed = True
            stalled_at, moved, last = run_pivot_and_watch(
                p, conn, WATCH,
                "commanding pivot -- a freely spinning wheel now reads as jammed")
            if stalled_at is None:
                failures.append("no stall reported -- detector did not fire")
            elif stalled_at > 2.0:
                failures.append(f"stall took {stalled_at:.2f}s (window is "
                                f"{cfg.stall_window/1000:.1f}s + a cycle)")
            if moved and max(moved) > 3:
                failures.append(f"wheels still moving after the halt: {moved}")
            if last is not None and not last.stalled:
                failures.append("stall flag did not LATCH -- it cleared itself "
                                "before the host could see it")

        print()
        if failures:
            for f in failures:
                print(f"FAIL: {f}")
            return 1
        print("PASS: stall detected, robot halted, flag latched")
        return 0
    finally:
        if pushed:
            print("\nrestoring stall.speed to the baked value")
            try:
                push_and_verify(p, "stall_speed", 15.0)
            except Exception as e:
                print(f"  RESTORE FAILED ({e}) -- reboot the robot before driving")
        halt_verified(p, conn)
        conn.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
