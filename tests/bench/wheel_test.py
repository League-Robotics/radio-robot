#!/usr/bin/env python3
"""wheel_test.py — keyboard VW teleop for the mecanum robot (046).

Drives the production firmware with `VW <vx> <vy> <omega>` (mm/s, mm/s, mrad/s),
streamed so the safety watchdog stays fed. Holding keys combines axes.

Controls (letters are primary — single bytes, can't misparse; arrows also work):
    W / S        forward / back        (or Up / Down arrow)
    A / D        strafe left / right   (or Left / Right arrow)
    Q / E        turn left / right (CCW / CW)
    space        stop now
    Ctrl-C       quit

A live line shows the VW being sent, so you can see each keypress land.

Run:
    uv run python tests/bench/wheel_test.py                          # USB (default)
    uv run python tests/bench/wheel_test.py /dev/cu.usbmodem2121302  # relay (untethered)
"""
import sys
import os
import time
import select
import termios
import tty
import pathlib

_HOST = pathlib.Path(__file__).resolve().parents[2] / "host"
if str(_HOST) not in sys.path:
    sys.path.insert(0, str(_HOST))
from robot_radio.io.serial_conn import SerialConnection  # noqa: E402

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem2121302"  # the radio relay
SPEED = 70      # mm/s forward / strafe (gentle — easier to feather)
TURN = 500      # mrad/s rotation (~29 deg/s)
TIMEOUT = 0.65  # s an axis stays active after its key was last seen.
#               # Must exceed the OS "Delay Until Repeat" (~0.5 s default) so a HELD
#               # key auto-repeats before the deadman fires — otherwise a held key
#               # stutters. Tip: System Settings > Keyboard > Key Repeat = Fast,
#               # Delay Until Repeat = Short gives the smoothest feel.

# key token -> (axis, value)
KEYS = {
    "w": ("vx", +SPEED), "s": ("vx", -SPEED),
    "a": ("vy", +SPEED), "d": ("vy", -SPEED),
    "q": ("w",  +TURN),  "e": ("w",  -TURN),
    "up": ("vx", +SPEED), "down": ("vx", -SPEED),
    "left": ("vy", +SPEED), "right": ("vy", -SPEED),
}
_ARROW = {b"\x1b[A": "up", b"\x1b[B": "down", b"\x1b[C": "right", b"\x1b[D": "left"}


def parse_keys(data: bytes):
    """Turn a raw byte buffer into key tokens (arrows -> up/down/left/right)."""
    out = []
    i = 0
    while i < len(data):
        three = data[i:i + 3]
        if three in _ARROW:
            out.append(_ARROW[three])
            i += 3
        elif data[i] == 0x03:        # Ctrl-C
            out.append("quit")
            i += 1
        elif data[i] == 0x20:        # space
            out.append("stop")
            i += 1
        elif data[i] == 0x1b:        # stray ESC byte (partial arrow) — ignore
            i += 1
        else:
            out.append(chr(data[i]).lower())
            i += 1
    return out


def main() -> int:
    print(f"connecting to {PORT} …")
    conn = SerialConnection(PORT)
    res = conn.connect()   # full relay !GO handshake + PING (production answers PING)
    if res.get("error"):
        print(f"connect failed: {res['error']}  (is togov on this port? try the relay arg)")
        return 2
    print(f"connected (mode={conn.mode}).")
    print("  W/S fwd/back   A/D strafe   Q/E turn   space=stop   Ctrl-C=quit   (arrows work too)")

    active = {}  # axis -> (value, expiry)
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while True:
            now = time.monotonic()
            r, _, _ = select.select([sys.stdin], [], [], 0.05)
            if r:
                for k in parse_keys(os.read(fd, 64)):
                    if k == "quit":
                        raise KeyboardInterrupt
                    if k == "stop":
                        active.clear()
                    elif k in KEYS:
                        axis, val = KEYS[k]
                        active[axis] = (val, now + TIMEOUT)

            active = {a: (v, e) for a, (v, e) in active.items() if e > now}
            vx = active.get("vx", (0, 0))[0]
            vy = active.get("vy", (0, 0))[0]
            w = active.get("w", (0, 0))[0]
            conn.send_fast(f"VW {vx} {vy} {w}")
            sys.stdout.write(f"\r  sending VW vx={vx:+5d} vy={vy:+5d} omega={w:+6d}   ")
            sys.stdout.flush()
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        for _ in range(3):
            conn.send_fast("VW 0 0 0")
            time.sleep(0.03)
        conn.disconnect()
        print("\nstopped.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
