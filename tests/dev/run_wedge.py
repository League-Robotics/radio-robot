#!/usr/bin/env python3
"""run_wedge.py — drive the in-firmware DBG WEDGE harness and stream its output.

Talks raw serial to the robot's DIRECT USB port (the harness prints via
uBit.serial, not the radio relay). Sends `DBG WEDGE <rate>`, prints every line,
and stops when the harness reports a wedge or ends (or on --secs timeout).

    uv run python tests/dev/run_wedge.py [--rate 50] [--secs 120]
"""
import argparse, sys, time
import serial


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--port", default="/dev/cu.usbmodem2121102")
    p.add_argument("--secs", type=float, default=120.0)
    p.add_argument("--rate", type=int, default=50, help="loop rate in Hz")
    args = p.parse_args()

    s = serial.Serial(args.port, 115200, timeout=0.2)
    time.sleep(0.3)
    s.reset_input_buffer()
    s.write(f"DBG WEDGE {args.rate}\r\n".encode())
    print(f"-> DBG WEDGE {args.rate} on {args.port}\n", flush=True)

    t0 = time.time()
    buf = b""
    try:
        while time.time() - t0 < args.secs:
            data = s.read(256)
            if not data:
                continue
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                txt = line.decode("ascii", "replace").rstrip("\r")
                if txt:
                    print(txt, flush=True)
                if "WEDGE-" in txt or "WEDGETEST end" in txt:
                    print(f"\n[done after {time.time()-t0:.1f}s]")
                    s.close()
                    return 0
        print(f"\n[timeout {args.secs:.0f}s — sending stop byte]")
        s.write(b"x\r\n")
        time.sleep(0.5)
        print(s.read(400).decode("ascii", "replace"))
    except KeyboardInterrupt:
        s.write(b"x\r\n")
        time.sleep(0.5)
        print(s.read(400).decode("ascii", "replace"))
    finally:
        s.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
