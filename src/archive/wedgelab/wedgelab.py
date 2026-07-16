#!/usr/bin/env python3
"""wedgelab.py — host driver for the WEDGELAB standalone firmware (src/test).

Usage:
  uv run python src/test/wedgelab.py ping
  uv run python src/test/wedgelab.py cmd "set preidle 4000"
  uv run python src/test/wedgelab.py run "run legs 60" --label baseline
  uv run python src/test/wedgelab.py script experiments.txt --label sweepA

`cmd`  sends one line, captures ~2 s of output.
`run`  sends one line, captures until a RESULT/RECOVER line (or --timeout).
`script` runs a file of lines; '#' comments; each 'run ...' line waits for
RESULT, other lines wait 0.5 s.  Everything is teed to src/test/out/.

Single-port discipline: NOTHING else may hold /dev/cu.usbmodem* while this
runs (a second open kills the holder with Errno 6).
"""

from __future__ import annotations

import argparse
import pathlib
import sys
import time

import serial

OUT = pathlib.Path(__file__).parent / "out"
DEFAULT_PORT = "/dev/cu.usbmodem2121102"
END_MARKERS = ("RESULT ", "RECOVER ok", "RECOVER FAILED", "PONG", "ERR ",
               "STOPV ok (user)", "STOPV FAILED", "(heal-end)")


def log_path(label: str) -> pathlib.Path:
    OUT.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return OUT / f"{stamp}_{label}.log"


class Lab:
    def __init__(self, port: str, log: pathlib.Path):
        self.ser = serial.Serial(port, 115200, timeout=0.3)
        self.log = open(log, "a")
        self.log.write(f"# opened {port} at {time.strftime('%F %T')}\n")
        # Opening (or the previous close) can RESET the board — knobs and
        # counters do not survive across invocations. Sync to a live prompt:
        # absorb any boot banner, then ping until PONG.
        self.pump(1.5)
        for _ in range(10):
            self.ser.write(b"ping\n")
            self.ser.flush()
            if self.pump(1.0, ("PONG",)):
                break
        else:
            self.emit("!! no PONG from firmware — is WEDGELAB flashed?")

    def emit(self, line: str) -> None:
        print(line)
        self.log.write(line + "\n")
        self.log.flush()

    def send(self, cmd: str) -> None:
        self.emit(f">> {cmd}")
        self.ser.write((cmd + "\n").encode())
        self.ser.flush()

    STALL_S = 45.0  # no output for this long during a marker-wait => stalled

    def pump(self, secs: float, until_markers: tuple[str, ...] | None = None) -> bool:
        """Stream output for up to `secs`; True if an until-marker arrived.
        Aborts early (False) if a marker-wait sees NO output for STALL_S —
        a silently-hung run must never eat the whole timeout again."""
        t0 = time.monotonic()
        last = t0
        while time.monotonic() - t0 < secs:
            raw = self.ser.readline()
            if not raw:
                if until_markers and time.monotonic() - last > self.STALL_S:
                    self.emit(f"!! no output for {self.STALL_S:.0f}s — treating as stalled")
                    return False
                continue
            line = raw.decode("utf-8", "ignore").rstrip()
            if not line:
                continue
            last = time.monotonic()
            self.emit(line)
            if until_markers and any(m in line for m in until_markers):
                return True
        return False

    def run_line(self, cmd: str, timeout: float) -> None:
        self.send(cmd)
        if cmd.startswith(("run ", "heal", "recover", "stop")):
            done = self.pump(timeout, END_MARKERS)
            if not done:
                self.emit(f"!! timeout after {timeout}s — sending abort byte")
                self.ser.write(b"\n")
                self.pump(5, END_MARKERS)
        else:
            self.pump(0.6)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["ping", "cmd", "run", "script"])
    ap.add_argument("arg", nargs="?", default="")
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--label", default="lab")
    ap.add_argument("--timeout", type=float, default=300.0)
    args = ap.parse_args()

    lab = Lab(args.port, log_path(args.label))
    lab.pump(0.3)  # drain any boot noise

    if args.mode == "ping":
        lab.send("ping")
        ok = lab.pump(3, ("PONG",))
        print("OK" if ok else "NO RESPONSE")
        return 0 if ok else 1
    if args.mode == "cmd":
        lab.send(args.arg)
        lab.pump(2)
        return 0
    if args.mode == "run":
        lab.run_line(args.arg, args.timeout)
        return 0
    if args.mode == "script":
        for line in pathlib.Path(args.arg).read_text().splitlines():
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            lab.run_line(line, args.timeout)
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(main())
