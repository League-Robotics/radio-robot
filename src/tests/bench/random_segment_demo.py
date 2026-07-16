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

Run: uv run python src/tests/bench/random_segment_demo.py [--port ...] [--seconds 45]
"""
from __future__ import annotations

import argparse
import random
import re
import sys
import time

from robot_radio.io.serial_conn import SerialConnection

_TLM = re.compile(
    r"enc=(?P<enc_l>-?\d+(?:\.\d+)?),(?P<enc_r>-?\d+(?:\.\d+)?)"
    r"\s+vel=(?P<vel_l>-?\d+(?:\.\d+)?),(?P<vel_r>-?\d+(?:\.\d+)?)"
    r"\s+cmd=(?P<cmd_l>-?\d+),(?P<cmd_r>-?\d+)"
    r"\s+acc=(?P<acc_l>-?\d+),(?P<acc_r>-?\d+)"
    r"\s+active=(?P<active>[01])"
    r"\s+conn=(?P<conn_l>[01]),(?P<conn_r>[01])"
    r"\s+glitch=(?P<glitch_l>\d+),(?P<glitch_r>\d+)"
    r"\s+ts=(?P<ts_l>\d+),(?P<ts_r>\d+)"
    r"\s+now=(?P<now>\d+)"
)


def read_tlm(conn):
    """Returns a dict of TLM fields or None."""
    for _ in range(3):
        r = conn.send("TLM", read_timeout=400)
        m = _TLM.search(" ".join(r.get("responses", [])))
        if m:
            return {k: float(v) for k, v in m.groupdict().items()}
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
    if not t or (t["conn_l"], t["conn_r"]) != (1, 1):
        conn_txt = f"{t['conn_l']},{t['conn_r']}" if t else "??"
        print(f"!! Nezha brick off the I2C bus (conn={conn_txt}) -- reseat + power on. Aborting.")
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
                        print(f"       tlm enc={st['enc_l']},{st['enc_r']} "
                              f"vel={st['vel_l']},{st['vel_r']} "
                              f"cmd={st['cmd_l']},{st['cmd_r']} "
                              f"active={st['active']} conn={st['conn_l']},{st['conn_r']}")
                        if (st["conn_l"], st["conn_r"]) != (1, 1):
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
