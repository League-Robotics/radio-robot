#!/usr/bin/env python3
"""teleop.py — minimal keyboard teleop for the mecanum robot (no camera).

Drives the 4-wheel mecanum robot with the firmware's 3-DOF VW command
(`VW <vx> <vy> <omega>`: vx,vy mm/s, omega mrad/s) over the radio relay.

Keys (hold to move, release to stop):
    ↑ / ↓        forward / back        (vx)
    ← / →        strafe left / right   (vy: +left)
    a / d        turn left / right     (omega: a=CCW, d=CW)
    space        stop now
    q            quit (sends stop)

Run:
    uv run python tests/bench/teleop.py                 # relay auto/default
    uv run python tests/bench/teleop.py --port /dev/cu.usbmodem2121302

Notes:
- The robot's safety watchdog stops it after ~500 ms of silence; this loop
  streams VW at ~20 Hz and SerialConnection's keepalive feeds the watchdog.
- A deadman timer zeroes the velocity ~0.5 s after the last keypress, so the
  robot stops shortly after you release a key (terminals can't see key-up).
- One key at a time = one pure motion (no diagonals); that's intentional and
  minimal. Hit space or just stop pressing to halt.
"""
from __future__ import annotations
import argparse
import pathlib
import select
import sys
import termios
import time
import tty

_HOST = pathlib.Path(__file__).resolve().parents[2] / "host"
if str(_HOST) not in sys.path:
    sys.path.insert(0, str(_HOST))
from robot_radio.io.serial_conn import SerialConnection  # noqa: E402


def read_keys(timeout):
    """Return the list of key tokens available within `timeout` s.

    Arrow keys arrive as the 3-byte CSI sequence ESC '[' {A,B,C,D}; we map
    them to 'up'/'down'/'right'/'left'. Plain chars pass through.
    """
    out = []
    end = time.monotonic() + timeout
    while True:
        remaining = end - time.monotonic()
        if remaining <= 0:
            break
        r, _, _ = select.select([sys.stdin], [], [], remaining)
        if not r:
            break
        ch = sys.stdin.read(1)
        if ch == "\x1b":
            # possible CSI arrow: read up to 2 more bytes if present
            r2, _, _ = select.select([sys.stdin], [], [], 0.005)
            if r2 and sys.stdin.read(1) == "[":
                r3, _, _ = select.select([sys.stdin], [], [], 0.005)
                if r3:
                    code = sys.stdin.read(1)
                    out.append({"A": "up", "B": "down", "C": "right", "D": "left"}.get(code, ""))
                    continue
            out.append("esc")
        else:
            out.append(ch)
    return [k for k in out if k]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/cu.usbmodem2121302",
                    help="serial port (default: the radio relay)")
    ap.add_argument("--speed", type=int, default=130, help="drive/strafe speed, mm/s")
    ap.add_argument("--turn", type=int, default=900, help="turn rate, mrad/s")
    ap.add_argument("--deadman", type=float, default=0.6, help="stop N s after last keypress")
    args = ap.parse_args()

    print(f"Connecting to {args.port} ...")
    conn = SerialConnection(args.port)
    res = conn.connect()
    if res.get("error"):
        print(f"connect failed: {res['error']}")
        return 2
    png = conn.send("PING", read_ms=800, stop_token="OK").get("responses")
    if not any("pong" in r for r in png):
        print(f"robot not responding (PING -> {png}). Power/relay OK?")
        conn.disconnect()
        return 2
    print(f"connected (mode={conn.mode}). PING -> {png}")
    print(__doc__.split('Run:')[0].split('Keys')[1])  # show the key map

    vx = vy = om = 0
    last_key = time.monotonic()
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while True:
            keys = read_keys(0.05)  # ~20 Hz loop
            for k in keys:
                if k == "q":
                    raise KeyboardInterrupt
                last_key = time.monotonic()
                if k == "up":      vx, vy, om = +args.speed, 0, 0
                elif k == "down":  vx, vy, om = -args.speed, 0, 0
                elif k == "left":  vx, vy, om = 0, +args.speed, 0   # +vy = left
                elif k == "right": vx, vy, om = 0, -args.speed, 0
                elif k in ("a", "A"): vx, vy, om = 0, 0, +args.turn  # CCW
                elif k in ("d", "D"): vx, vy, om = 0, 0, -args.turn  # CW
                elif k == " ":     vx, vy, om = 0, 0, 0
            # deadman: stop shortly after the last keypress (no key-up in a TTY)
            if time.monotonic() - last_key > args.deadman:
                vx, vy, om = 0, 0, 0
            conn.send_fast(f"VW {vx} {vy} {om}")
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        for _ in range(3):
            conn.send_fast("VW 0 0 0")
            conn.send_fast("X")
            time.sleep(0.03)
        conn.disconnect()
        print("\nstopped, disconnected.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
