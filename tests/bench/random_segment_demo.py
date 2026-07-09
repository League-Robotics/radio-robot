"""Bench HITL: stream random motion segments at the drivetrain and watch them
string together (with Ruckig re-planning between segments) into smooth,
continuous motion -- plus deliberate queue-drain pauses that show a graceful
decel-to-zero start/stop.

Segments are posted rapidly in bursts (they queue on bb.segmentIn, an 8-slot
WorkQueue, and the Drivetrain executes them head-to-tail); between bursts a
pause lets the queue drain so you see a clean graceful stop, then it starts
again. On the stand it doesn't matter where the robot "goes" -- random is the
point; you're watching starting, stopping, straights, and turns flow smoothly.

Bus-guarded: refuses to run if the Nezha brick is off the I2C bus (conn!=1,1).

Run: uv run python tests/bench/random_segment_demo.py [--port ...] [--seconds 45]
"""
from __future__ import annotations

import argparse
import random
import re
import sys
import time

from robot_radio.io.serial_conn import SerialConnection

_TLM = re.compile(
    r"enc=(-?\d+),(-?\d+)\s+vel=(-?\d+),(-?\d+)\s+active=([01])\s+conn=([01]),([01])"
)


def read_tlm(conn):
    for _ in range(3):
        r = conn.send("TLM", read_timeout=400)
        m = _TLM.search(" ".join(r.get("responses", [])))
        if m:
            return tuple(int(x) for x in m.groups())
    return None


def random_segment() -> str:
    """A random single-segment command (MOVE/D/RT), varied but bench-safe."""
    kind = random.random()
    if kind < 0.40:
        # straight (fwd or rev), varied length
        mm = random.choice([-1, 1]) * random.randint(150, 500)
        return f"MOVE {mm} 0 0"
    if kind < 0.70:
        # pure in-place turn, +/- 45..160 deg (centidegrees)
        cdeg = random.choice([-1, 1]) * random.randint(4500, 16000)
        return f"RT {cdeg}"
    if kind < 0.90:
        # translate then pivot to a final heading
        mm = random.randint(150, 450)
        cdeg = random.choice([-1, 1]) * random.randint(4500, 12000)
        return f"MOVE {mm} 0 {cdeg}"
    # a D straight (different verb, same segment path)
    s = random.choice([-1, 1]) * random.randint(200, 400)
    return f"D {s} {s} {random.randint(200, 450)}"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/cu.usbmodem2121102")
    ap.add_argument("--seconds", type=float, default=45.0)
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args()
    random.seed(args.seed)

    conn = SerialConnection(args.port, mode="direct")
    print("connect:", conn.connect(skip_ping=False).get("status"))

    t = read_tlm(conn)
    if not t or (t[5], t[6]) != (1, 1):
        print(f"!! Nezha brick off the I2C bus (conn={t[5:] if t else '??'}) -- reseat + power on. Aborting.")
        return 3
    print("bus OK (conn=1,1) -- streaming random segments\n")

    end = time.monotonic() + args.seconds
    last_tlm = 0.0
    n = 0
    try:
        while time.monotonic() < end:
            # A burst of 1-4 segments fired back-to-back -> they queue and
            # string together (Ruckig re-plans across the boundaries).
            burst = random.randint(1, 4)
            for _ in range(burst):
                cmd = random_segment()
                n += 1
                print(f"[{n:02d}] >> {cmd}")
                conn.send_fast(cmd)
                time.sleep(0.05)
            # Pause: short -> the next burst strings on; long -> the queue
            # drains to a graceful stop before the next start.
            pause = random.choice([0.4, 0.8, 1.2, 2.0, 3.0])
            t0 = time.monotonic()
            while time.monotonic() - t0 < pause:
                time.sleep(0.25)
                if time.monotonic() - last_tlm > 0.7:
                    last_tlm = time.monotonic()
                    st = read_tlm(conn)
                    if st:
                        print(f"       tlm enc={st[0]},{st[1]} vel={st[2]},{st[3]} "
                              f"active={st[4]} conn={st[5]},{st[6]}")
                        if (st[5], st[6]) != (1, 1):
                            print("       !! bus dropped mid-run"); return 3
    finally:
        conn.send("STOP", read_timeout=500)
        time.sleep(0.5)
        fin = read_tlm(conn)
        print(f"\nSTOP. final tlm: {fin}")
    print(f"\nStreamed {n} random segments over ~{args.seconds:.0f}s.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
