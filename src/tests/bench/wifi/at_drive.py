#!/usr/bin/env python3
"""Drive the atbridge: send AT commands to the WiFi module through gopiv.

Usage: at_drive.py PORT [--wait S] [--settle S] CMD [CMD ...]

Each CMD is sent with CRLF; everything received is printed with a relative
timestamp. A CMD of the form "sleep:N" just waits N seconds. A CMD of
"raw:..." sends the payload with NO CRLF (for AT+CIPSEND data phases).
Opening the port resets the micro:bit (DTR), so the bridge re-probes the
RJ11 ports on every run -- the script waits for "ATBRIDGE: ready" first.
"""
import sys
import time

import serial

def main() -> int:
    args = sys.argv[1:]
    port = args.pop(0)
    wait = 2.0  # [s] per-command listen window
    if args and args[0] == "--wait":
        args.pop(0)
        wait = float(args.pop(0))
    cmds = args

    ser = serial.Serial(port, 115200, timeout=0.1)
    t0 = time.time()

    def pump(seconds: float, label: str = "") -> str:
        got = b""
        deadline = time.time() + seconds
        while time.time() < deadline:
            chunk = ser.read(4096)
            if chunk:
                got += chunk
                sys.stdout.write(chunk.decode("utf-8", "replace"))
                sys.stdout.flush()
        return got.decode("utf-8", "replace")

    # The board does NOT reset on port open (boot banner is flash-time
    # only) -- just drain whatever is pending and go.
    print(f"--- opened {port}, draining")
    pump(1.0)

    for cmd in cmds:
        if cmd.startswith("sleep:"):
            pump(float(cmd.split(":", 1)[1]))
            continue
        print(f"\n--- [{time.time()-t0:6.2f}s] >> {cmd!r}")
        if cmd.startswith("raw:"):
            ser.write(cmd[4:].encode())
        else:
            ser.write(cmd.encode() + b"\r\n")
        pump(wait)

    ser.close()
    return 0

if __name__ == "__main__":
    sys.exit(main())
