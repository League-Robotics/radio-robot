"""Bench HITL: drive the gutted drivetrain firmware (post-094 + D/T/RT) over
direct USB serial and confirm motion via TLM encoder deltas.

Single-segment verbs, each sent then given time to execute and settle:
  D <l> <r> <mm>   -> straight, distance-bounded (one Motion::Segment)
  T <l> <r> <ms>   -> straight over a time (distance = v*t)
  RT <relAngle>    -> pure in-place relative turn
  MOVE ...         -> the full segment verb (straight+pivot)

Motion correctness only -- distance/turn accuracy is NOT checked. Success =
the encoders move in the commanded sense (you also HEAR the motors on the
stand). No watchdog exists in this firmware, and segments self-terminate, so
nothing runs away; STOP triggers a graceful decel.

Run: uv run python tests/bench/dtr_drive_demo.py [--port /dev/cu.usbmodemXXXX]
"""
from __future__ import annotations

import argparse
import re
import sys
import time

from robot_radio.io.serial_conn import SerialConnection

_TLM = re.compile(
    r"enc=(-?\d+),(-?\d+)\s+vel=(-?\d+),(-?\d+)\s+active=([01])\s+conn=([01]),([01])"
)


def read_tlm(conn):
    """Returns (encL, encR, velL, velR, active, connL, connR) or None."""
    for _ in range(3):
        r = conn.send("TLM", read_timeout=600)
        text = " ".join(r.get("responses", [])) if isinstance(r, dict) else str(r)
        m = _TLM.search(text)
        if m:
            return tuple(int(x) for x in m.groups())
    return None


def require_bus(conn) -> None:
    """Refuse to drive if the Nezha brick is off the I2C bus (conn != 1,1).
    A disconnected bus ACKs every command on the micro:bit but never drives
    a motor -- this guard makes that impossible to mistake for 'no motion'."""
    t = read_tlm(conn)
    if t is None:
        print("!! could not read TLM -- aborting")
        raise SystemExit(2)
    connL, connR = t[5], t[6]
    print(f"    bus check: conn={connL},{connR}")
    if (connL, connR) != (1, 1):
        print("\n!!!! NEZHA BRICK OFF THE I2C BUS (conn={},{}) !!!!".format(connL, connR))
        print("     The micro:bit ACKs commands but cannot drive/read motors.")
        print("     Reseat the micro:bit into the Nezha brick + motor power on, then retry.")
        raise SystemExit(3)
    print("    bus OK (conn=1,1) -- motors reachable")


def do(conn, line: str, settle_s: float) -> None:
    print(f"\n=== {line}  (settle {settle_s}s) ===")
    before = read_tlm(conn)
    r = conn.send(line, read_timeout=800)
    print(f"    reply: {r.get('lines', r) if isinstance(r, dict) else r}")
    time.sleep(settle_s)
    after = read_tlm(conn)
    if before and after:
        dL, dR = after[0] - before[0], after[1] - before[1]
        print(f"    enc delta: L={dL:+d}  R={dR:+d}   (before={before[:2]} after={after[:2]})")
        if abs(dL) < 3 and abs(dR) < 3:
            print("    !! NO MOTION DETECTED")
        else:
            print("    -> motion confirmed")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/cu.usbmodem2121102")
    args = ap.parse_args()

    conn = SerialConnection(
        args.port, mode="direct",
        on_send=lambda c: print(f">>> {c}"),
        on_recv=lambda l: print(f"<<< {l}"),
    )
    info = conn.connect(skip_ping=False)
    print("connect:", info)

    try:
        print("\n--- liveness ---")
        print("PING:", conn.send("PING", read_timeout=600))
        print("HELLO:", conn.send("HELLO", read_timeout=600))

        print("\n--- bus preflight ---")
        require_bus(conn)   # aborts loudly if the Nezha brick is off the bus

        do(conn, "D 300 300 400", 3.0)      # straight forward
        do(conn, "D -300 -300 400", 3.0)    # straight reverse
        do(conn, "RT 9000", 3.0)            # +90 deg in-place turn
        do(conn, "RT -9000", 3.0)           # -90 deg
        do(conn, "T 300 300 1500", 3.0)     # timed straight
        do(conn, "MOVE 300 0 9000", 4.5)    # translate then pivot to +90

        print("\n--- STOP (graceful) ---")
        print("STOP:", conn.send("STOP", read_timeout=600))
    finally:
        try:
            conn.send("STOP", read_timeout=400)
        except Exception:
            pass
        if hasattr(conn, "close"):
            conn.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
