#!/usr/bin/env python3
"""speed_map.py -- build the commanded -> actual wheel-speed mapping that
seeds the velocity PID's feedforward (its "first guess" at a new setpoint).

Design: 50 speeds equally spaced over [0, 500] mm/s. One PASS visits every
speed in some order, stepping speed-to-speed WITHOUT stopping (the wheels
never return to rest inside a pass) and recording each wheel's velocity
0.5 s after the command lands. Three passes per round -- ascending,
descending, random -- and the whole round is repeated twice, so ordering
effects (hysteresis, backlash, thermal drift) show up as separation
between the pass types rather than hiding inside one average.

Both wheels are driven together at the same commanded speed, which is the
condition the robot actually operates in (and the load condition the
mapping has to hold under); each wheel's own velocity is recorded, so one
trial yields a point for each motor.

Every command is ack-verified with retry: a dropped command would leave
the previous speed running and silently corrupt the sample.

    uv run python src/tests/bench/speed_map.py --port /dev/cu.usbmodem2121102
"""
from __future__ import annotations

import argparse
import csv
import random
import statistics
import sys
import time

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol

DEFAULT_PORT = "/dev/cu.usbmodem2121102"
SPEED_MIN = 0.0        # [mm/s]
SPEED_MAX = 500.0      # [mm/s] the measured ceiling
SPEED_COUNT = 50
DWELL = 0.5            # [s] settle window before sampling, per the design
HOLD_MS = 900.0        # [ms] command window -- outlives DWELL, superseded by the next step
ROUNDS = 2             # the whole asc/desc/random round, twice
SAMPLE_LO = 0.42       # [s] median the velocity registers over this window
SAMPLE_HI = 0.58       # [s]  ... around the 0.5 s mark (robust to outlier samples)
RANDOM_SEED = 20260727


def speeds() -> list[float]:
    step = (SPEED_MAX - SPEED_MIN) / (SPEED_COUNT - 1)
    return [round(SPEED_MIN + step * i, 2) for i in range(SPEED_COUNT)]


def sendVerified(proto, v, moveId) -> bool:
    """Command both wheels at v; return once the enqueue ack confirms it
    landed. The DAPLink bridge drops ~20% of inbound packets."""
    for _ in range(5):
        corr = proto.wheels(v_left=v, v_right=v, duration=HOLD_MS,
                            move_id=moveId)
        ack = proto.wait_for_ack(corr, timeout=250)
        if ack is not None and ack.ok:
            return True
    return False


def sampleAfterDwell(proto):
    """Collect telemetry for DWELL seconds from now; return the median
    left/right velocity over the [SAMPLE_LO, SAMPLE_HI] window."""
    velsL, velsR = [], []
    t0 = time.monotonic()
    while True:
        elapsed = time.monotonic() - t0
        if elapsed >= DWELL:
            break
        for f in proto.read_pending_binary_tlm_frames():
            if f.enc_left is None:
                continue
            at = time.monotonic() - t0
            if SAMPLE_LO <= at <= SAMPLE_HI:
                velsL.append(f.enc_left.velocity)
                velsR.append(f.enc_right.velocity)
        time.sleep(0.005)
    mL = statistics.median(velsL) if velsL else float("nan")
    mR = statistics.median(velsR) if velsR else float("nan")
    return mL, mR, len(velsL)


def runPass(proto, order, label, rnd, rows, moveIdSeed):
    """Step through `order` without stopping. `prev` is the speed the
    wheels were actually holding when this step's command landed -- the
    direction of approach (accelerating vs decelerating into the new
    setpoint) is the hysteresis variable, so it is recorded per trial
    rather than inferred from the pass label: a random pass contains both.
    Each pass starts from rest (the estop between passes), so the first
    trial's prev is 0."""
    print(f"  pass {label} ({len(order)} speeds)...", flush=True)
    misses = 0
    prev = 0.0
    for i, v in enumerate(order):
        if not sendVerified(proto, v, moveIdSeed + i):
            misses += 1
            print(f"    WARNING: {v:.1f} mm/s never acked -- sample dropped")
            continue  # prev unchanged: the wheels still hold the old speed
        mL, mR, n = sampleAfterDwell(proto)
        rows.append((rnd, label, i, prev, v, mL, mR, n))
        prev = v
    if misses:
        print(f"    ({misses} commands never acked)")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", default=DEFAULT_PORT)
    p.add_argument("--out", default="src/tests/bench/output/speed_map.csv")
    args = p.parse_args()

    conn = SerialConnection(port=args.port)
    conn.connect()
    proto = NezhaProtocol(conn)
    print("waiting out boot preamble...")
    time.sleep(6.0)
    proto.read_pending_binary_tlm_frames()

    levels = speeds()
    rng = random.Random(RANDOM_SEED)
    rows = []  # (round, pass, index, commanded, measuredL, measuredR, samples)
    moveId = 9700
    try:
        for rnd in range(1, ROUNDS + 1):
            print(f"round {rnd}/{ROUNDS}")
            asc = list(levels)
            desc = list(reversed(levels))
            shuf = list(levels)
            rng.shuffle(shuf)
            for label, order in (("asc", asc), ("desc", desc), ("rand", shuf)):
                runPass(proto, order, label, rnd, rows, moveId)
                moveId += 100
                proto.estop()          # rest between passes, not between steps
                time.sleep(1.0)
                proto.read_pending_binary_tlm_frames()
    finally:
        proto.estop()
        time.sleep(0.3)
        conn.disconnect()

    out = args.out
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["round", "pass", "index", "prev", "commanded",
                    "measuredL", "measuredR", "samples"])  # [mm/s]
        w.writerows(rows)
    print(f"wrote {out} ({len(rows)} trials)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
