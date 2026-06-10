#!/usr/bin/env python3
"""safe_cmd_bench.py — verify the SAFE on/off command on the bench robot.

[1] SAFE off : with keepalives cut, the robot must KEEP MOVING (watchdog off) —
    we stop it manually with X (that's the classroom T-driving mode).
[2] SAFE on 600 : with keepalives cut, the robot must safety-stop ~600 ms later.
Leaves the watchdog ON (safe default) at the end.
"""
import sys
import time

from robot_radio.io.serial_conn import SerialConnection

PORT = "/dev/cu.usbmodem2121102"


def saw_safety(c, window_s):
    t0 = time.time()
    while time.time() - t0 < window_s:
        for ln in c.read_lines(120):
            if "safety_stop" in ln:
                return (time.time() - t0) * 1000.0
    return None


def main():
    c = SerialConnection(port=PORT, mode="direct")
    c.connect()
    print("PING:", [x for x in c.send("PING")["responses"] if "pong" in x][:1])

    # [1] SAFE off — motion must continue with no keepalive
    print("\n[1] SAFE off → cut keepalive → robot must KEEP MOVING")
    print("    reply:", c.send("SAFE off")["responses"])
    c.send("S -45 45")            # slow spin
    c.stop_keepalive()
    fired1 = saw_safety(c, 1.6)
    print(f"    safety_stop seen = {fired1 is not None}   (want False — safety OFF)")
    c.send("X")                   # WE stop it (no watchdog to do it)
    c.start_keepalive()
    time.sleep(0.4)

    # [2] SAFE on 600 — motion must safety-stop ~600 ms after silence
    print("\n[2] SAFE on 600 → cut keepalive → EVT safety_stop ~600 ms")
    print("    reply:", c.send("SAFE on 600")["responses"])
    c.send("S -45 45")
    c.stop_keepalive()
    ms = saw_safety(c, 2.0)
    print(f"    EVT safety_stop at {ms:.0f} ms" if ms else "    NO safety_stop (FAIL)")
    c.start_keepalive()
    c.send("X")

    # restore safe default
    print("\n[restore]", c.send("SAFE on")["responses"])
    c.disconnect()

    ok = (fired1 is None) and (ms is not None)
    print("\nRESULT:", "PASS — SAFE off frees motion, SAFE on re-arms" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
