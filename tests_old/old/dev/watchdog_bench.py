#!/usr/bin/env python3
"""watchdog_bench.py — verify the safety-stop watchdog on the bench robot.

Direct serial. Demonstrates the fix for the runaway:
  (A) with the background keepalive running, a slow in-place spin keeps going;
  (B) when keepalives STOP (simulating the host program dying), the robot emits
      `EVT safety_stop` and the wheels stop within sTimeout — on its own.
"""
import sys
import time

from robot_radio.io.serial_conn import SerialConnection

PORT = "/dev/cu.usbmodem2121102"   # robot direct USB


def last_tlm(lines):
    for ln in reversed(lines):
        if "TLM" in ln:
            return ln
    return None


def main():
    c = SerialConnection(port=PORT, mode="direct")
    r = c.connect()
    if not c.is_open:
        print("connect failed:", r)
        return 1
    ping = c.send("PING")
    print("connected; PING ->", [x for x in ping["responses"] if "pong" in x][:1])
    c.send("SET sTimeout=500")

    # ── Part A: motion stays alive while the keepalive thread runs ──
    print("\n[A] slow in-place spin for 2.5s WITH keepalive (must NOT safety-stop)")
    c.send("S -45 45")          # gentle spin (the failure mode from before)
    time.sleep(2.5)
    a_lines = c.read_lines(400)
    a_stop = any("safety_stop" in ln for ln in a_lines)
    print(f"    safety_stop during keepalive = {a_stop}   (want False)")
    print(f"    last TLM: {last_tlm(a_lines)}")

    # ── Part B: cut keepalive → watchdog must fire and stop the wheels ──
    print("\n[B] cutting keepalive (simulating host death) — expect EVT safety_stop")
    c.stop_keepalive()
    t0 = time.time()
    evt_ms = None
    while time.time() - t0 < 2.0:
        for ln in c.read_lines(120):
            if "safety_stop" in ln:
                evt_ms = (time.time() - t0) * 1000.0
                break
        if evt_ms is not None:
            break
    if evt_ms is not None:
        print(f"    EVT safety_stop received at {evt_ms:.0f} ms after silence")
    else:
        print("    NO EVT safety_stop seen  (FAIL)")
    time.sleep(0.3)
    print(f"    TLM after stop: {last_tlm(c.read_lines(400))}")

    # leave the robot idle and safe
    c.start_keepalive()
    c.send("X")
    c.disconnect()

    ok = (not a_stop) and (evt_ms is not None)
    print("\nRESULT:", "PASS — keepalive holds, silence safety-stops" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
