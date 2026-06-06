#!/usr/bin/env python3
"""enc_watch.py — bench probe: drive slowly on the stand and watch the encoders.

Connects (relay-aware, pushes stored calibration), zeroes the I2C diagnostic
counters, drives both wheels forward slowly for a few seconds, and prints the
live encoder totals + per-wheel velocity each ~250 ms. Ends with a DBG I2C dump.

USE ON THE STAND ONLY (wheels spin free). It just needs to answer one question
after a fresh power-cycle: do the wheel encoders count when the wheels turn?

    uv run python tests/dev/enc_watch.py [--speed 60] [--secs 4]
"""

from __future__ import annotations

import argparse
import sys
import time


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=None)
    p.add_argument("--speed", type=int, default=60, help="wheel speed mm/s (default 60)")
    p.add_argument("--secs", type=float, default=4.0, help="drive seconds (default 4)")
    args = p.parse_args()

    from robot_radio.io.cli import _make_robot
    from robot_radio.robot.nezha import Nezha
    from robot_radio.robot.protocol import parse_tlm, parse_response

    class _A:
        port = args.port
        verbose = False

    print("Connecting (relay, pushes calibration) …")
    robot, conn, _ = _make_robot(_A())
    if not isinstance(robot, Nezha):
        print("ERROR: need a Nezha"); return 2
    proto = robot._proto

    def dbg_i2c() -> str:
        r = proto.send("DBG I2C", 400)
        for line in r.get("responses", []):
            if line.strip().startswith("I2C "):
                return line.strip()
        return "(no I2C line)"

    try:
        print(f"  start ENC: {proto.send('ENC', 300).get('responses', [])}")
        print(f"  I2C before: {dbg_i2c()}")
        proto.send("DBG I2C RESET", 300)
        proto.send("STOP", 200)
        proto.send("STREAM 0", 200)
        time.sleep(0.05)
        proto.send("SET sTimeout=10000", 300)
        proto.zero_encoders()
        proto.stream(50)

        print(f"\n  driving {args.speed} mm/s for {args.secs:.0f}s — watching encoders:")
        enc = (0, 0)
        vel = (0, 0)
        moved = False
        t0 = time.monotonic()
        last_send = 0.0
        last_print = 0.0
        while time.monotonic() - t0 < args.secs:
            now = time.monotonic()
            if now - last_send >= 0.15:
                proto.drive(args.speed, args.speed)
                last_send = now
            for line in proto.read_lines(duration_ms=25):
                if "EVT safety_stop" in line:
                    proto.drive(args.speed, args.speed)
                tlm = parse_tlm(line)
                if tlm is None:
                    continue
                if tlm.enc is not None:
                    enc = tlm.enc
                    if abs(enc[0]) >= 5 or abs(enc[1]) >= 5:
                        moved = True
                if tlm.vel is not None:
                    vel = tlm.vel
            if now - last_print >= 0.25:
                last_print = now
                print(f"    t={now-t0:4.1f}s  enc=L{enc[0]:>6} R{enc[1]:>6} mm   "
                      f"vel=L{vel[0]:>5} R{vel[1]:>5} mm/s")

        for _ in range(4):
            proto.stop(); time.sleep(0.05)
        proto.stream(0)

        print(f"\n  final ENC (cmd): {proto.send('ENC', 300).get('responses', [])}")
        print(f"  I2C after:  {dbg_i2c()}")
        print("\n========================================")
        if moved:
            print("  ENCODERS COUNTED ✓  — the wheels register motion.")
        else:
            print("  ENCODERS FROZEN ✗  — encoder stayed ~0 while driving (WEDGED).")
        print("========================================")
    finally:
        try:
            for _ in range(3):
                proto.stop(); time.sleep(0.04)
            proto.stream(0)
            conn.disconnect()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
