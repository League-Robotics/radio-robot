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

Port mapping (NezhaHAL.cpp, fixed): wheel L = Nezha port M2 (chip id 2),
wheel R = port M1 (chip id 1).  All reporting below is BY PORT so motor-swap
experiments (does the latch follow the motor or the port?) read directly.
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

# Fixed HAL wiring (NezhaHAL.cpp): LEFT wheel = chip M2, RIGHT wheel = chip M1.
PORT_OF_WHEEL = {"L": "M2", "R": "M1", "?": "?"}


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
                wheel = m.group(1) if m else "?"
                port = PORT_OF_WHEEL[wheel]
                enc = int(m.group(2)) if m else None
                episodes.append({"leg": leg, "port": port, "wheel": wheel,
                                 "enc": enc, "time_stop": timeout_stop})
                log(f"  leg {leg:3d}: LATCH port={port} @enc={enc}mm"
                    f"{'  (leg died on TIME backstop)' if timeout_stop else ''}")
            if timeout_stop and not wedged:
                episodes.append({"leg": leg, "port": "?", "wheel": "?",
                                 "enc": None, "time_stop": True})
                log(f"  leg {leg:3d}: reason=time without EVT (boundary latch, "
                    f"detector blind)")
            if leg % 10 == 0:
                n_m2 = sum(1 for e in episodes if e["port"] == "M2")
                n_m1 = sum(1 for e in episodes if e["port"] == "M1")
                log(f"[{leg}/{args.legs}] episodes: {len(episodes)} "
                    f"({len(episodes)/leg:.2f}/leg)  M2={n_m2} M1={n_m1}")
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
        n_m2 = sum(1 for e in episodes if e["port"] == "M2")
        n_m1 = sum(1 for e in episodes if e["port"] == "M1")
        n_q = len(episodes) - n_m2 - n_m1
        result["by_port"] = {"M2": n_m2, "M1": n_m1, "unattributed": n_q}
        out.write_text(json.dumps(result, indent=1))
        log(f"==== {len(episodes)} episodes in {legs_run} legs "
            f"({result['episodes_per_leg']}/leg): "
            f"port M2={n_m2} port M1={n_m1} unattributed={n_q} -> {out}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
