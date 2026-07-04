"""wedge_latch_repro.py — minimal, reliable encoder-latch (wedge) reproducer.

Mechanism (KB: 2026-07-01-encoder-wedge-boundary-latch-flavor.md): the Nezha
0x46 readback latch strikes at D-deceleration/stop boundaries, where the
motor-write throttle is bypassed.  So the minimal reproducer is simply
BACK-TO-BACK SHORT D LEGS — maximum decel boundaries per minute — through
the production path, with the firmware's own detector (`EVT enc_wedged`) and
the TLM freeze signature as ground truth.  No estimator, no tours, no GUI.

Field baseline that motivated this: 4-9 latch episodes per 13-leg tour
(2026-07-03 evening sessions).

Output: episodes per leg, inter-arrival legs, and the exact leg indices —
the dataset a fix iterates against (run before/after any candidate fix).

Usage:
    uv run python tests/bench/wedge_latch_repro.py [--port PORT] [--legs 60]
        [--dist 150] [--speed 250]

Robot must be ON THE STAND (motors spin).  Direct USB (EVT frames needed).
"""
from __future__ import annotations

import argparse
import json
import pathlib
import re
import sys
import time

import serial

_REPO = pathlib.Path(__file__).resolve().parents[2]
OUTDIR = _REPO / "tests" / "bench" / "out"
OUTDIR.mkdir(parents=True, exist_ok=True)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/cu.usbmodem2121102")
    ap.add_argument("--legs", type=int, default=60)
    ap.add_argument("--dist", type=int, default=150)
    ap.add_argument("--speed", type=int, default=250)
    ap.add_argument("--leg-timeout", type=float, default=8.0)
    args = ap.parse_args()

    def log(m):
        print(f"{time.strftime('%H:%M:%S')} {m}", flush=True)

    ser = serial.Serial(args.port, 115200, timeout=0.3)
    episodes: list[dict] = []
    legs_run = 0
    raw_log: list[str] = []

    def send(cmd: str):
        ser.write((cmd + "\n").encode())
        ser.flush()

    def pump(secs: float, until: str | None = None) -> list[str]:
        t0 = time.monotonic()
        got = []
        while time.monotonic() - t0 < secs:
            try:
                ln = ser.readline().decode("utf-8", "ignore").strip()
            except Exception as exc:  # noqa: BLE001
                got.append(f"<serial:{exc}>")
                break
            if not ln:
                continue
            got.append(ln)
            raw_log.append(ln)
            if until and until in ln:
                break
        return got

    try:
        pump(4.0)                      # boot banner after DTR reset
        send("PING")
        pump(1.5, "pong")
        send("SET sTimeout=60000")
        pump(0.8)
        send("SET rotSlip=0")
        pump(0.8)

        t_start = time.monotonic()
        for leg in range(1, args.legs + 1):
            # Re-prime + rebaseline at rest every leg start is implicit: the
            # D verb's own encoder reset is the documented un-latcher.
            send(f"D {args.speed} {args.speed} {args.dist}")
            lines = pump(args.leg_timeout, "EVT done D")
            legs_run = leg
            wedged = [ln for ln in lines if "enc_wedged" in ln]
            timeout_stop = any("reason=time" in ln for ln in lines)
            for w in wedged:
                m = re.search(r"wheel=([LR]) enc=(-?\d+)", w)
                episodes.append({"leg": leg, "wheel": m.group(1) if m else "?",
                                 "enc": int(m.group(2)) if m else None,
                                 "time_stop": timeout_stop})
                log(f"  leg {leg:3d}: LATCH {w[:70]}")
            if timeout_stop and not wedged:
                episodes.append({"leg": leg, "wheel": "?", "enc": None,
                                 "time_stop": True})
                log(f"  leg {leg:3d}: reason=time without EVT (boundary latch)")
            if leg % 10 == 0:
                rate = len(episodes) / leg
                log(f"[{leg}/{args.legs}] episodes so far: {len(episodes)} "
                    f"({rate:.2f}/leg)")
            time.sleep(0.4)            # settle between legs
    finally:
        send("X")
        pump(0.5)
        send("SET sTimeout=500")
        pump(0.5)
        ser.close()
        result = {
            "legs": legs_run, "dist": args.dist, "speed": args.speed,
            "episodes": episodes,
            "episodes_per_leg": round(len(episodes) / max(1, legs_run), 3),
        }
        out = OUTDIR / "wedge_latch_repro.json"
        out.write_text(json.dumps(result, indent=1))
        (OUTDIR / "wedge_latch_repro_raw.log").write_text("\n".join(raw_log))
        log(f"==== {len(episodes)} episodes in {legs_run} legs "
            f"({result['episodes_per_leg']}/leg) -> {out}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
