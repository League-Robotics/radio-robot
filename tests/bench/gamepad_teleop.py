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
PERIOD = 0.150           # [s] send cadence default; --period-ms to experiment
                         #     (100ms = 10 msg/s, near the radio's ~12/s
                         #     budget; 200ms = comfortable 5/s)
DEADMAN_FACTOR = 3       # deadman window = 3x the period: rides through two
                         #     dropped messages, still stops promptly on loss
                         #     (stick release sends an explicit t=50 v=0 stop,
                         #     so the deadman only covers link drops)
QUEUE_HARD_MAX = 5       # backlog panic threshold -> skip sends until it drains

_MOVER_ACK = re.compile(r"OK mover .*q=(\d+)")
_ERR_FULL = re.compile(r"ERR full")


class Teleop:
    """Deadman-velocity teleop (stakeholder design, 2026-07-09): every
    period, send `MOVER 0 0 0 t=<3x period> v=<signed> w=<signed>` -- a
    REPLACE-semantics command. The firmware replans from its CURRENT
    velocity toward the stick's velocity every time one arrives (no queue to
    manage, no buffer bookkeeping); if messages stop (stick released, link
    dropped), the last segment's t= window expires and the robot decels
    gracefully. Sends are fire-and-forget (send_fast); acks are collected
    asynchronously for the status line."""

    def __init__(self):
        self.conn = None      # set after SerialConnection(on_recv=self.on_recv)
        self.drive_axis = 1   # left stick Y (see --probe if your mapping differs)
        self.turn_axis = 0    # left stick X
        self.drive_sign = -1.0   # stick pushes forward -> +drive
        self.turn_sign = 1.0     # bench-set 2026-07-09 ("invert the X axis")
        self.period = PERIOD  # [s] fixed cadence (attr kept for status/selftest)
        self.sent = 0
        self.acked = 0
        self.full = 0
        self.q_last = 0
        self._was_driving = False

    def on_recv(self, line):
        m = _MOVER_ACK.search(line)
        if m:
            self.acked += 1
            self.q_last = int(m.group(1))
        elif _ERR_FULL.search(line):
            self.full += 1

    def send_segment(self, drive, turn):
        """One stick sample -> one MOVER message. drive/turn are [-1, 1].
        Returns True if motion commanded."""
        if abs(drive) < DEADZONE and abs(turn) < DEADZONE:
            if self._was_driving:
                # Snappier stop than waiting out the deadman: replace with a
                # zero-velocity, short-window MOVER once on release.
                self.conn.send_fast("MOVER 0 0 0 t=50 v=0 w=0")
                self._was_driving = False
            return False
        self._was_driving = True
        v = int(round(drive * MAX_SPEED))                    # [mm/s] signed
        w = int(round(turn * MAX_YAW_RATE * 100))            # [cdeg/s] signed
        deadman_ms = int(self.period * 1000 * DEADMAN_FACTOR)
        self.sent += 1
        self.conn.send_fast(f"MOVER 0 0 0 t={deadman_ms} v={v} w={w}")
        return True

    def stop(self):
        self.conn.send("STOP", read_timeout=400)

    def status(self):
        lost = self.sent - self.acked - self.full
        return (f"period={self.period*1000:5.0f}ms q={self.q_last} "
                f"sent={self.sent} acked={self.acked} full={self.full} lost~={lost}")


def run_gamepad(tele):
    # NOTE: no SDL_VIDEODRIVER=dummy. On macOS the headless/dummy video
    # driver breaks INITIAL joystick enumeration (only the hot-plug event
    # path works) -- that was the "unplug/replug every launch" bug. With the
    # normal driver an already-plugged pad is simply there, like every other
    # joystick app. (F310 one-time setup: rear switch on 'D'; a native
    # Dual Action has no switch and just works.)
    os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
    import pygame
    pygame.init()
    pygame.joystick.init()
    for _ in range(10):          # brief settle for enumeration callbacks
        pygame.event.pump()
        if pygame.joystick.get_count() > 0:
            break
        time.sleep(0.1)
    if pygame.joystick.get_count() == 0:
        print("!! no gamepad detected")
        return 2
    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"gamepad: {js.get_name()}  ({js.get_numaxes()} axes)")
    print(f"drive axis={tele.drive_axis}  turn axis={tele.turn_axis}"
          "   (override with --drive-axis/--turn-axis; find them with --probe)")
    print("left stick drives (Y=fwd/back, X=turn). BACK button or Ctrl-C to quit.")
    last_status = 0.0
    try:
        while True:
            pygame.event.pump()
            # Default F310/Dual Action: axis 0 = left X (right +), axis 1 =
            # left Y (down +) -- but macOS mappings vary; --probe to verify.
            # Turn sign +1 per bench driving 2026-07-09 ("invert the X axis");
            # flip either at runtime with --invert-drive/--invert-turn.
            drive = tele.drive_sign * js.get_axis(tele.drive_axis)
            turn = tele.turn_sign * js.get_axis(tele.turn_axis)
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


def run_probe():
    """No robot needed: print every axis/hat/button live so the stick's axis
    indices can be identified. Push the LEFT stick around and read off which
    axis index moves for forward/back vs left/right."""
    os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
    import pygame
    pygame.init()
    pygame.joystick.init()
    for _ in range(10):
        pygame.event.pump()
        if pygame.joystick.get_count() > 0:
            break
        time.sleep(0.1)
    if pygame.joystick.get_count() == 0:
        print("!! no gamepad detected")
        return 2
    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"gamepad: {js.get_name()}  axes={js.get_numaxes()} "
          f"hats={js.get_numhats()} buttons={js.get_numbuttons()}")
    print("move the LEFT stick; Ctrl-C to quit. (fwd/back should swing one axis full range)")
    try:
        while True:
            pygame.event.pump()
            axes = "  ".join(f"a{i}={js.get_axis(i):+.2f}" for i in range(js.get_numaxes()))
            hats = "  ".join(f"h{i}={js.get_hat(i)}" for i in range(js.get_numhats()))
            btns = "".join(str(js.get_button(b)) for b in range(js.get_numbuttons()))
            print(f"\r{axes}  {hats}  b={btns}   ", end="", flush=True)
            time.sleep(0.08)
    except KeyboardInterrupt:
        print()
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default="/dev/cu.usbmodem2121102")
    ap.add_argument("--relay", action="store_true",
                    help="port is a radio relay dongle (default: direct USB)")
    ap.add_argument("--selftest", action="store_true",
                    help="scripted 8s stick pattern, no gamepad needed")
    ap.add_argument("--probe", action="store_true",
                    help="no robot: print live axes/hats/buttons to identify the stick mapping")
    ap.add_argument("--drive-axis", type=int, default=1,
                    help="joystick axis index for fwd/back (default 1)")
    ap.add_argument("--turn-axis", type=int, default=0,
                    help="joystick axis index for left/right (default 0)")
    ap.add_argument("--invert-drive", action="store_true",
                    help="flip the fwd/back sense")
    ap.add_argument("--invert-turn", action="store_true",
                    help="flip the left/right sense")
    ap.add_argument("--period-ms", type=int, default=int(PERIOD * 1000),
                    help="MOVER send cadence [ms] (deadman = 3x this)")
    args = ap.parse_args()

    if args.probe:
        return run_probe()

    tele = Teleop()
    tele.drive_axis = args.drive_axis
    tele.turn_axis = args.turn_axis
    if args.invert_drive:
        tele.drive_sign = -tele.drive_sign
    if args.invert_turn:
        tele.turn_sign = -tele.turn_sign
    tele.period = max(0.05, args.period_ms / 1000.0)
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
