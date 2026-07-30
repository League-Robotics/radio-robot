#!/usr/bin/env python3
"""crawl_sweep.py -- characterize crawl mode: sub-breakaway wheel-speed
requests become fixed 0.30-duty pulses, Bresenham-dithered onboard so the
average duty matches the request. This sweep asks for effective duties
0.02..0.28, holds each 2 s (both wheels together), and measures achieved
mean speed and speed ripple from the encoders.

    uv run python src/tests/bench/crawl_sweep.py --port /dev/cu.usbmodem2121102
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol

DEFAULT_PORT = "/dev/cu.usbmodem2121102"
GAIN = 1370.0   # [mm/s per duty] firmware open-loop scale (commanded speed = duty*1370)
HOLD = 2.0      # [s] per level -- long enough to average many pulse cycles
REST = 0.6      # [s]
LEVELS = [round(0.02 * i, 2) for i in range(1, 15)]  # effective duty 0.02..0.28


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", default=DEFAULT_PORT)
    args = p.parse_args()

    conn = SerialConnection(port=args.port)
    conn.connect()
    proto = NezhaProtocol(conn)
    print("waiting out boot preamble...")
    time.sleep(6.0)
    # 125-005 (telemetry-emit-policy-rebuild-spec.md Part 7): under the new
    # kAuto default a parked robot emits nothing unsolicited -- and the
    # first level's own pre-move "anchor positions" poll below reads a
    # baseline frame before the run's first move_wheels(), on a robot that
    # has never moved. Without this, `start` stays None and the run crashes
    # computing `end.enc_left.position - start.enc_left.position`.
    proto.tlmOn()
    proto.read_pending_binary_tlm_frames()

    results = []  # (duty, meanSpeedL, rippleL, meanSpeedR, rippleR)
    for duty in LEVELS:
        v = duty * GAIN
        # anchor positions
        start = None
        t0 = time.monotonic()
        while start is None and time.monotonic() - t0 < 1.0:
            for f in proto.read_pending_binary_tlm_frames():
                if f.enc_left is not None:
                    start = f
            time.sleep(0.02)
        proto.move_wheels(v_left=v, v_right=v, stop_time=HOLD * 1000.0,
                          timeout=HOLD * 1000.0 + 2000.0, replace=True)
        velsL, velsR = [], []
        end = start
        t0 = time.monotonic()
        while time.monotonic() - t0 < HOLD:
            for f in proto.read_pending_binary_tlm_frames():
                if f.enc_left is not None:
                    end = f
                    velsL.append(f.enc_left.velocity)
                    velsR.append(f.enc_right.velocity)
            time.sleep(0.02)
        proto.estop()
        time.sleep(REST)
        proto.read_pending_binary_tlm_frames()

        meanL = (end.enc_left.position - start.enc_left.position) / HOLD
        meanR = (end.enc_right.position - start.enc_right.position) / HOLD
        ripL = statistics.pstdev(velsL) if len(velsL) > 1 else 0.0
        ripR = statistics.pstdev(velsR) if len(velsR) > 1 else 0.0
        results.append((duty, meanL, ripL, meanR, ripR))
        print(f"eff duty {duty:.2f} (cmd {v:5.1f} mm/s): "
              f"L {meanL:6.1f} +-{ripL:5.1f}   R {meanR:6.1f} +-{ripR:5.1f}")
    try:
        proto.tlmOff()
    except Exception:
        pass
    conn.disconnect()

    import csv
    with open("src/tests/bench/crawl_sweep.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["effDuty", "meanL", "rippleL", "meanR", "rippleR"])  # [mm/s]
        w.writerows(results)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    duties = [r[0] for r in results]
    fig, ax = plt.subplots(figsize=(10, 6))
    for idx, (label, colr) in ((1, ("left", "#1f77b4")), (3, ("right", "#2ca02c"))):
        means = [r[idx] for r in results]
        rips = [r[idx + 1] for r in results]
        ax.errorbar(duties, means, yerr=rips, color=colr, marker="o", ms=4,
                    capsize=3, label=f"{label} mean speed (+- ripple)")
    ax.plot(duties, [d * GAIN for d in duties], color="#999999", ls=":",
            label="commanded (open-loop nominal)")
    ax.set_xlabel("requested effective duty [-]")
    ax.set_ylabel("wheel speed [mm/s]")
    ax.set_title("Crawl mode -- 0.30-duty Bresenham pulses, 2 s holds, real encoders")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig("src/tests/bench/crawl_sweep.png", dpi=130)
    print("wrote src/tests/bench/crawl_sweep.{csv,png}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
