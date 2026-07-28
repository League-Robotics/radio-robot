#!/usr/bin/env python3
"""duty_sweep.py -- measure the wheel dead zone and duty-vs-speed curve
with the real encoders.

Open-loop characterization: each trial commands one wheel at a constant
level (via the wheel-speeds command; the firmware's open-loop scale maps
speed*kDutyPerSpeed -> duty, so commanded speed = duty * 1370 exactly),
holds 500 ms, records the encoder delta and end velocity, then rests so
every trial starts from a stopped, stuck wheel. No control loop crosses
the serial link.

    uv run python src/tests/bench/duty_sweep.py --port /dev/cu.usbmodem2121102
"""
from __future__ import annotations

import argparse
import sys
import time

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol

DEFAULT_PORT = "/dev/cu.usbmodem2121102"
GAIN = 1370.0        # [mm/s per duty] the firmware's open-loop scale (kDutyPerSpeed inverse)
DUTY_MAX = 0.60
DUTY_STEP = 0.01  # to 0.30; 0.02 above (see duties below)
HOLD = 0.5           # [s] level hold
REST = 0.5           # [s] full-stop rest so each trial starts stuck
REPEATS = 3
MOTION_MM = 1.0      # [mm] encoder delta that counts as "moved"


def latestEnc(proto):
    last = None
    for f in proto.read_pending_binary_tlm_frames():
        if f.enc_left is not None:
            last = f
    return last


def waitEnc(proto, timeout=1.0):  # [s]
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        f = latestEnc(proto)
        if f is not None:
            return f
        time.sleep(0.02)
    raise RuntimeError("no telemetry")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", default=DEFAULT_PORT)
    args = p.parse_args()

    conn = SerialConnection(port=args.port)
    conn.connect()
    proto = NezhaProtocol(conn)
    print("waiting out boot preamble...")
    time.sleep(6.0)
    proto.read_pending_binary_tlm_frames()

    duties = [round(0.01 * i, 3) for i in range(1, 31)] +              [round(0.30 + 0.02 * i, 3) for i in range(1, 16)]
    results = []  # (wheel, direction, duty, repeat, delta_mm, end_vel)

    for wheel in ("left", "right"):
        for direction in (+1, -1):
            for duty in duties:
                for rep in range(REPEATS):
                    start = waitEnc(proto)
                    s0 = (start.enc_left.position if wheel == "left"
                          else start.enc_right.position)
                    v = direction * duty * GAIN
                    kw = {"v_left": v, "v_right": 0.0} if wheel == "left" \
                         else {"v_left": 0.0, "v_right": v}
                    proto.move_wheels(stop_time=HOLD * 1000.0,
                                      timeout=HOLD * 1000.0 + 2000.0,
                                      replace=True, **kw)
                    time.sleep(HOLD)
                    end = waitEnc(proto)
                    s1 = (end.enc_left.position if wheel == "left"
                          else end.enc_right.position)
                    ev = (end.enc_left.velocity if wheel == "left"
                          else end.enc_right.velocity)
                    proto.estop()
                    time.sleep(REST)
                    proto.read_pending_binary_tlm_frames()
                    results.append((wheel, direction, duty, rep, s1 - s0, ev))
                # progress line per level
                deltas = [r[4] for r in results[-REPEATS:]]
                print(f"{wheel} {'+' if direction > 0 else '-'} duty={duty:.2f} "
                      f"deltas={['%.1f' % d for d in deltas]}")
    conn.disconnect()

    import csv
    csvPath = "src/tests/bench/duty_sweep.csv"
    with open(csvPath, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["wheel", "direction", "duty", "repeat", "delta", "endVelocity"])  # [mm] [mm/s]
        w.writerows(results)
    print(f"wrote {csvPath}")

    # Dead-zone edge: lowest duty whose EVERY repeat moved.
    print("\nDEAD ZONE (lowest duty with sustained motion, all repeats):")
    for wheel in ("left", "right"):
        for direction in (+1, -1):
            edge = None
            for duty in duties:
                reps = [r for r in results
                        if r[0] == wheel and r[1] == direction and r[2] == duty]
                if all(abs(r[4]) > MOTION_MM for r in reps):
                    edge = duty
                    break
            print(f"  {wheel:5s} {'fwd' if direction > 0 else 'rev'}: {edge}")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(10, 6))
    styles = {("left", 1): ("#1f77b4", "-"), ("left", -1): ("#1f77b4", "--"),
              ("right", 1): ("#2ca02c", "-"), ("right", -1): ("#2ca02c", "--")}
    for (wheel, direction), (colr, ls) in styles.items():
        xs, ys = [], []
        for duty in duties:
            reps = [r for r in results
                    if r[0] == wheel and r[1] == direction and r[2] == duty]
            xs.append(duty)
            ys.append(sum(abs(r[4]) / HOLD for r in reps) / len(reps))
        ax.plot(xs, ys, color=colr, ls=ls, marker="o", ms=3,
                label=f"{wheel} {'fwd' if direction > 0 else 'rev'}")
    ax.axline((0, 0), slope=GAIN, color="#999999", lw=0.8, ls=":",
              label=f"nominal {GAIN:.0f} mm/s per duty")
    ax.set_xlabel("duty [-]")
    ax.set_ylabel("mean speed over the 500 ms hold [mm/s]")
    ax.set_title("Duty sweep -- real encoders, from rest, 500 ms holds x3")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    out = "src/tests/bench/duty_sweep.png"
    fig.savefig(out, dpi=130)
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
