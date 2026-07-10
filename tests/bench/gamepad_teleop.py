"""Gamepad teleop: Logitech F310 left stick -> streamed MOVE micro-segments,
rate-adapted against the firmware's motion-queue depth.

Mapping (left stick):
  - forward/back (Y axis)  -> drive: segment distance, signed
  - side-to-side (X axis)  -> turn:  segment finalHeading, signed (CCW+ left)
Pushed together, each segment carries BOTH (translate-then-pivot at micro
scale ~ blended steering).

Flow control (the point of this program): every accepted `MOVE` ack carries
`q=<depth>` -- segments queued in the drivetrain (ring + executing +
undrained segmentIn). We aim to hold q ~= QUEUE_TARGET (3):
  - q >= 4 (or `ERR full`)  -> slow down (longer send period, LARGER segments
                               so queued time still matches the stick)
  - q <= 2                  -> speed up (shorter period, smaller segments)
Segment size is always sized to the CURRENT period, so the queue holds
~3 * period of future motion -- the stick-to-wheel lag. At the hoped-for
20-30ms period that's ~60-90ms of lag.

Transports:
  uv run python tests/bench/gamepad_teleop.py --port /dev/cu.usbmodemXXX             # direct USB
  uv run python tests/bench/gamepad_teleop.py --port /dev/cu.usbmodemYYY --relay     # radio relay
`--selftest` drives a scripted 8s stick pattern (sine drive + weave) with NO
gamepad -- used to validate the streaming/flow-control path over the radio
before handing the sticks to a human.

Exit: Ctrl-C (sends STOP), or the F310's BACK button.
"""
from __future__ import annotations

import argparse
import math
import os
import re
import sys
import time

from robot_radio.io.serial_conn import SerialConnection

# --- Tunables ---------------------------------------------------------------
MAX_SPEED = 400.0        # [mm/s] stick fully forward
MAX_YAW_RATE = 120.0     # [deg/s] stick fully sideways
DEADZONE = 0.12          # stick fraction ignored around center
QUEUE_TARGET = 3         # aim: this many segments in flight
PERIOD_MIN = 0.020       # [s] fastest send cadence
PERIOD_MAX = 0.200       # [s] slowest send cadence
PERIOD_STEP = 1.15       # multiplicative rate adaptation per ack

_MOVE_ACK = re.compile(r"OK move .*q=(\d+)")
_ERR_FULL = re.compile(r"ERR full")


class Teleop:
    """Fire-and-forget MOVE streamer. Sends are send_fast() (never blocking
    on the reply -- a blocking send caps the cadence at the transport RTT,
    ~200ms over USB-CDC and worse over radio); acks are collected
    asynchronously by on_recv() off the connection's reader thread, and the
    LATEST reported q= drives the rate adaptation."""

    def __init__(self):
        self.conn = None      # set after SerialConnection(on_recv=self.on_recv)
        self.period = 0.050   # [s] adaptive send period, starts mid-range
        self.sent = 0
        self.acked = 0
        self.full = 0
        self.q_last = 0

    def on_recv(self, line):
        m = _MOVE_ACK.search(line)
        if m:
            self.acked += 1
            self.q_last = int(m.group(1))
            # Rate adaptation against the reported backlog.
            if self.q_last >= QUEUE_TARGET + 1:
                self.period = min(PERIOD_MAX, self.period * PERIOD_STEP)
            elif self.q_last <= QUEUE_TARGET - 1:
                self.period = max(PERIOD_MIN, self.period / PERIOD_STEP)
        elif _ERR_FULL.search(line):
            self.full += 1
            self.period = min(PERIOD_MAX, self.period * PERIOD_STEP * PERIOD_STEP)

    def send_segment(self, drive, turn):
        """One stick sample -> one MOVE micro-segment sized to the current
        period. drive/turn are [-1, 1]. Returns True if motion commanded."""
        if abs(drive) < DEADZONE and abs(turn) < DEADZONE:
            return False   # centered: send nothing; queue drains -> graceful stop
        distance = int(round(drive * MAX_SPEED * self.period))          # [mm]
        heading_cdeg = int(round(turn * MAX_YAW_RATE * self.period * 100))  # [cdeg]
        if distance == 0 and heading_cdeg == 0:
            return False
        self.sent += 1
        # s=1: STREAMING segment -- merges into the in-flight plan (chained
        # at speed); without it each micro-segment solves from rest and the
        # robot crawls no matter the send rate.
        self.conn.send_fast(f"MOVE {distance} 0 {heading_cdeg} s=1")
        return True

    def stop(self):
        self.conn.send("STOP", read_timeout=400)

    def status(self):
        lost = self.sent - self.acked - self.full
        return (f"period={self.period*1000:5.0f}ms q={self.q_last} "
                f"sent={self.sent} acked={self.acked} full={self.full} lost~={lost}")


def run_gamepad(tele):
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")   # no window needed
    import pygame
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("!! no gamepad detected (is the F310 plugged in, switch on 'D' or 'X'?)")
        return 2
    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"gamepad: {js.get_name()}  ({js.get_numaxes()} axes)")
    print("left stick drives (Y=fwd/back, X=turn). BACK button or Ctrl-C to quit.")
    last_status = 0.0
    try:
        while True:
            pygame.event.pump()
            # F310: axis 0 = left stick X (right +), axis 1 = left stick Y (down +)
            drive = -js.get_axis(1)   # push forward -> +drive
            turn = -js.get_axis(0)    # push left -> +turn (CCW+, matches cdeg)
            if any(js.get_button(b) for b in range(js.get_numbuttons())
                   if js.get_button(b) and b == 6):   # BACK
                print("BACK pressed -- stopping")
                break
            tele.send_segment(drive, turn)
            if time.monotonic() - last_status > 0.5:
                last_status = time.monotonic()
                print(f"\r{tele.status()}  drive={drive:+.2f} turn={turn:+.2f}   ",
                      end="", flush=True)
            time.sleep(tele.period)
    except KeyboardInterrupt:
        print("\nCtrl-C -- stopping")
    finally:
        tele.stop()
        print("\n" + tele.status())
    return 0


def run_selftest(tele, seconds=8.0):
    """Scripted stick pattern (no gamepad): 0-3s sine drive forward, 3-5s
    forward+weave, 5-6.5s reverse, then release. Validates streaming, q=
    flow control, and graceful drain over whatever transport is connected."""
    print(f"selftest: scripted stick for {seconds}s -- wheels WILL move")
    t0 = time.monotonic()
    periods = []
    qs = []
    try:
        while (t := time.monotonic() - t0) < seconds:
            if t < 3.0:
                drive, turn = 0.7 * math.sin(t * math.pi / 3.0) + 0.2, 0.0
            elif t < 5.0:
                drive, turn = 0.5, 0.6 * math.sin((t - 3.0) * 2.5)
            elif t < 6.5:
                drive, turn = -0.5, 0.0
            else:
                drive, turn = 0.0, 0.0
            tele.send_segment(drive, turn)
            periods.append(tele.period)
            qs.append(tele.q_last)
            time.sleep(tele.period)
    finally:
        tele.stop()
    import statistics as st
    print(f"\nselftest done: {tele.status()}")
    if periods:
        print(f"  period: median {st.median(periods)*1000:.0f}ms  "
              f"range [{min(periods)*1000:.0f},{max(periods)*1000:.0f}]ms")
    if qs:
        hist = {q: qs.count(q) for q in sorted(set(qs))}
        print(f"  q histogram: {hist}")
    ok = tele.acked > 0 and tele.acked >= tele.sent * 0.7
    print(f"  verdict: {'PASS' if ok else 'FAIL'} "
          f"(ack rate {tele.acked}/{tele.sent})")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default="/dev/cu.usbmodem2121102")
    ap.add_argument("--relay", action="store_true",
                    help="port is a radio relay dongle (default: direct USB)")
    ap.add_argument("--selftest", action="store_true",
                    help="scripted 8s stick pattern, no gamepad needed")
    args = ap.parse_args()

    tele = Teleop()
    conn = SerialConnection(args.port, mode=("relay" if args.relay else "direct"),
                            on_recv=tele.on_recv)
    info = conn.connect(skip_ping=False)
    if not conn.is_open:
        print(f"connect failed: {info}")
        return 2
    print(f"connected: {args.port} mode={'relay' if args.relay else 'direct'}")
    tele.conn = conn
    try:
        if args.selftest:
            return run_selftest(tele)
        return run_gamepad(tele)
    finally:
        try:
            tele.stop()
        except Exception:
            pass
        conn.disconnect()


if __name__ == "__main__":
    sys.exit(main())
