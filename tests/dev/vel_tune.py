#!/usr/bin/env python3
"""vel_tune.py — live velocity-PID tuning sweep on the stand.

Drives both wheels at a steady speed and measures the velocity ripple (mean +
standard deviation per wheel) for a series of live SET configurations — so we
can find gains that hold a smooth line instead of the sawtooth limit-cycle.

All configs are applied to the SAME connection (one micro:bit reset, one ZERO
enc, no reconnect hammering). Steady driving only — NO mid-drive OTOS/stream
perturbation (that pattern is what triggers the encoder wedge). Pushes
calibration + zeroes encoders first (required after the DTR reset on connect).

    uv run python tests/dev/vel_tune.py [--speed 200] [--secs 4]

Each config line: a dict of SET key=value (vel.kP, vel.kI, vel.kFF, vel.filt,
sync, ...). Edit CONFIGS below to sweep.
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time

# Sweep list — each dict is applied via SET, then measured. The first is the
# current firmware default (baseline).
CONFIGS = [
    {"name": "baseline (sync=1 filt=.3 kP=.3 kI=.05)",
     "sync": 1.0, "vel.filt": 0.3, "vel.kP": 0.3, "vel.kI": 0.05, "vel.kFF": 0.15},
    {"name": "sync OFF",
     "sync": 0.0, "vel.filt": 0.3, "vel.kP": 0.3, "vel.kI": 0.05, "vel.kFF": 0.15},
    {"name": "sync OFF + heavy filt .15",
     "sync": 0.0, "vel.filt": 0.15, "vel.kP": 0.3, "vel.kI": 0.05, "vel.kFF": 0.15},
    {"name": "sync OFF + filt .15 + kI .12 (dt-comp)",
     "sync": 0.0, "vel.filt": 0.15, "vel.kP": 0.3, "vel.kI": 0.12, "vel.kFF": 0.15},
    {"name": "sync OFF + filt .1 + kP .15 + kI .12",
     "sync": 0.0, "vel.filt": 0.10, "vel.kP": 0.15, "vel.kI": 0.12, "vel.kFF": 0.15},
]


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default="/dev/cu.usbmodem2121102")
    p.add_argument("--speed", type=int, default=200)
    p.add_argument("--secs", type=float, default=4.0)
    args = p.parse_args()

    from robot_radio.io.cli import _make_robot
    from robot_radio.robot.protocol import parse_tlm

    class _A:
        port = args.port
        verbose = False

    print(f"Connecting (pushes cal) … speed={args.speed} secs={args.secs}")
    robot, conn, _ = _make_robot(_A())
    proto = robot._proto
    spd = args.speed

    def drive_measure(secs):
        """Drive at spd, stream 50 Hz, return (Lmean,Lsd,Rmean,Rsd,n) over last 70%."""
        samples = []
        proto.drive(spd, spd)
        last = time.monotonic()
        t0 = time.monotonic()
        wedged = False
        while time.monotonic() - t0 < secs:
            now = time.monotonic()
            if now - last >= 0.12:
                proto.drive(spd, spd)
                last = now
            for ln in proto.read_lines(duration_ms=20):
                if "EVT safety_stop" in ln:
                    proto.drive(spd, spd)
                if "enc_wedged" in ln:
                    wedged = True
                tl = parse_tlm(ln)
                if tl and tl.vel is not None:
                    samples.append((tl.vel[0], tl.vel[1]))
        # settle stop between configs (brief, keeps the chip from idle-reading)
        for _ in range(3):
            proto.stop(); time.sleep(0.04)
        if len(samples) < 8:
            return None
        tail = samples[int(len(samples) * 0.3):]
        L = [s[0] for s in tail]
        R = [s[1] for s in tail]
        return (statistics.mean(L), statistics.pstdev(L),
                statistics.mean(R), statistics.pstdev(R), len(tail), wedged)

    try:
        proto.send("SET sTimeout=10000", 300)
        proto.zero_encoders()
        proto.stream(20)
        time.sleep(0.2)
        print(f"{'config':44s} {'Lmean':>6} {'Lsd':>5} {'Rmean':>6} {'Rsd':>5}  note")
        print("-" * 86)
        for cfg in CONFIGS:
            for k, v in cfg.items():
                if k == "name":
                    continue
                proto.send(f"SET {k}={v}", 200)
            time.sleep(0.2)
            res = drive_measure(args.secs)
            if res is None:
                print(f"{cfg['name']:44s}   (no samples)")
                continue
            Lm, Ls, Rm, Rs, n, wedged = res
            note = "WEDGED!" if wedged else ""
            print(f"{cfg['name']:44s} {Lm:6.0f} {Ls:5.1f} {Rm:6.0f} {Rs:5.1f}  {note}")
            if wedged:
                print("  -> encoder wedged; aborting sweep (needs power-cycle)")
                break
            time.sleep(0.3)
    finally:
        try:
            for _ in range(4):
                proto.stop(); time.sleep(0.04)
            proto.stream(0)
            conn.disconnect()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
