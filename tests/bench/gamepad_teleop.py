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
PERIOD = 0.200           # [s] send cadence -- one message per ~200ms (5 msg/s,
                         #     well inside the radio's ~12 msg/s budget)
REM_TARGET_S = 0.400     # [s] buffer depth held in the plan, as TIME of motion
                         #     at the current stick speed. Deep enough that the
                         #     plan CRUISES between messages (a shallow buffer
                         #     puts the to-rest tail mid-stream = 5Hz pulsing);
                         #     also the stick-release overrun bound.
REM_GAIN = 0.8           # fraction of the buffer error corrected per message
QUEUE_HARD_MAX = 5       # backlog panic threshold -> skip sends until it drains

_MOVE_ACK = re.compile(r"OK move .*q=(\d+) rem=(-?\d+)")
_ERR_FULL = re.compile(r"ERR full")


class Teleop:
    """Fire-and-forget MOVE streamer. Sends are send_fast() (never blocking
    on the reply); acks are collected asynchronously by on_recv() off the
    connection's reader thread.

    Control law (anti-pulsing, 2026-07-09): fixed ~200ms cadence; each
    message carries (a) `v=` -- the STICK speed as the segment's speed
    ceiling, so the plan cruises at what the driver asked instead of running
    each buffer time-optimally (surge/brake), and (b) a distance that covers
    one period's consumption PLUS a closed-loop correction holding the
    plan's remaining distance (`rem=` from the ack) at ~REM_TARGET_S of
    motion -- deep enough that the to-rest decel tail never starts while
    the stick is deflected. Release the stick -> nothing more is sent -> the
    plan's own tail is the graceful stop (~REM_TARGET_S of overrun)."""

    def __init__(self):
        self.conn = None      # set after SerialConnection(on_recv=self.on_recv)
        self.drive_axis = 1   # left stick Y (see --probe if your mapping differs)
        self.turn_axis = 0    # left stick X
        self.period = PERIOD  # [s] fixed cadence (kept as attr for status/selftest)
        self.sent = 0
        self.acked = 0
        self.full = 0
        self.q_last = 0
        self.rem = 0.0        # [mm] plan's remaining translation, from the ack

    def on_recv(self, line):
        m = _MOVE_ACK.search(line)
        if m:
            self.acked += 1
            self.q_last = int(m.group(1))
            self.rem = float(m.group(2))
        elif _ERR_FULL.search(line):
            self.full += 1

    def send_segment(self, drive, turn):
        """One stick sample -> one MOVE message. drive/turn are [-1, 1].
        Returns True if motion commanded."""
        if abs(drive) < DEADZONE and abs(turn) < DEADZONE:
            self.rem = 0.0   # stream over; next press re-primes the buffer
            return False     # centered: send nothing; plan tail = graceful stop
        if self.q_last >= QUEUE_HARD_MAX:
            return False     # backlog panic: skip this slot, let it drain

        speed = abs(drive) * MAX_SPEED                     # [mm/s] stick speed
        yaw = abs(turn) * MAX_YAW_RATE                     # [deg/s]
        # Distance: one period of consumption + buffer-depth correction.
        target_rem = speed * REM_TARGET_S                  # [mm]
        correction = REM_GAIN * (target_rem - self.rem)    # [mm]
        magnitude = max(0.0, speed * self.period + correction)
        distance = int(round(magnitude if drive > 0 else -magnitude))  # [mm]
        heading_cdeg = int(round(turn * MAX_YAW_RATE * self.period * 100))  # [cdeg]
        if distance == 0 and heading_cdeg == 0:
            return False
        self.sent += 1
        # s=1 streaming merge; v=/w= cap the plan at the STICK's own speed so
        # it cruises there (segments without caps run time-optimally: surge).
        v_cap = max(20, int(round(speed)))                 # [mm/s]
        w_cap = max(500, int(round(yaw * 100)))            # [cdeg/s]
        self.conn.send_fast(
            f"MOVE {distance} 0 {heading_cdeg} v={v_cap} w={w_cap} s=1")
        return True

    def stop(self):
        self.conn.send("STOP", read_timeout=400)

    def status(self):
        lost = self.sent - self.acked - self.full
        return (f"period={self.period*1000:5.0f}ms q={self.q_last} rem={self.rem:.0f}mm "
                f"sent={self.sent} acked={self.acked} full={self.full} lost~={lost}")


def run_gamepad(tele):
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")   # no window needed
    os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
    import pygame
    pygame.init()
    pygame.joystick.init()
    # Wait for the pad (hot-plug arrives via events, so pump). NOTE macOS +
    # F310: the rear switch MUST be on 'D' (DirectInput = standard HID).
    # In 'X' mode the pad is an XInput/vendor-class device -- macOS binds no
    # HID driver and SDL cannot see it at all. The switch only takes effect
    # on re-enumeration: flip to D, then unplug/replug.
    deadline = time.monotonic() + 20.0
    warned = False
    while pygame.joystick.get_count() == 0:
        pygame.event.pump()
        if not warned:
            print("waiting for gamepad... (F310: rear switch on 'D', then unplug/replug)")
            warned = True
        if time.monotonic() > deadline:
            print("!! no gamepad after 20s -- is the switch on 'D'? (X mode is"
                  " invisible to macOS: no HID driver binds to XInput devices)")
            return 2
        time.sleep(0.25)
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
            drive = -js.get_axis(tele.drive_axis)   # push forward -> +drive
            turn = -js.get_axis(tele.turn_axis)     # push left -> +turn (CCW+)
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
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
    import pygame
    pygame.init()
    pygame.joystick.init()
    deadline = time.monotonic() + 20.0
    while pygame.joystick.get_count() == 0:
        pygame.event.pump()
        if time.monotonic() > deadline:
            print("!! no gamepad (D mode? replugged? Input Monitoring granted + app restarted?)")
            return 2
        time.sleep(0.25)
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
    args = ap.parse_args()

    if args.probe:
        return run_probe()

    tele = Teleop()
    tele.drive_axis = args.drive_axis
    tele.turn_axis = args.turn_axis
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
