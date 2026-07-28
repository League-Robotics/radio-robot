#!/usr/bin/env python3
"""speed_sweep.py -- the full commanded-vs-achieved wheel-speed curve
through BOTH drive regimes: crawl (sub-breakaway requests pulse at 0.30
duty, Bresenham-dithered onboard) and continuous (open-loop duty).
Both wheels together, forward and reverse, 2 s holds, real encoders.

    uv run python src/tests/bench/speed_sweep.py --port /dev/cu.usbmodem2121102
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol

DEFAULT_PORT = "/dev/cu.usbmodem2121102"
GAIN = 535.0        # [mm/s per duty] mean measured gain (calibrated firmware)
CRAWL_PULSE = 0.20  # [-] firmware crawl amplitude -> crawl below ~107 mm/s commanded
HOLD = 2.0          # [s]
REST = 0.6          # [s]
LEVELS = list(range(10, 101, 10)) + list(range(120, 401, 20)) + \
         [430, 460, 490, 520, 550]  # [mm/s] commanded (ceiling ~500)


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

    results = []  # (cmd_signed, meanL, ripL, meanR, ripR)
    for direction in (+1, -1):
        for level in LEVELS:
            v = direction * float(level)
            start = None
            t0 = time.monotonic()
            while start is None and time.monotonic() - t0 < 1.0:
                for f in proto.read_pending_binary_tlm_frames():
                    if f.enc_left is not None:
                        start = f
                time.sleep(0.02)
            # Retry until the enqueue ack confirms arrival: the DAPLink
            # USB->UART bridge drops whole inbound packets under outbound
            # telemetry load (measured ~20%, zero malformed count -- see
            # the command-loss hunt). replace=True makes a duplicate
            # arrival harmless.
            for attempt in range(4):
                corr = proto.move_wheels(v_left=v, v_right=v,
                                         stop_time=HOLD * 1000.0,
                                         timeout=HOLD * 1000.0 + 2000.0,
                                         replace=True)
                ack = proto.wait_for_ack(corr, timeout=400)
                if ack is not None and ack.ok:
                    break
            else:
                print(f"  WARNING: cmd {v:+.1f} never acked after 4 tries")
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
            results.append((v, meanL, ripL, meanR, ripR))
            regime = "crawl" if level < GAIN * CRAWL_PULSE else "cont."
            print(f"cmd {v:+7.1f} ({regime}): L {meanL:+7.1f} +-{ripL:5.1f}"
                  f"   R {meanR:+7.1f} +-{ripR:5.1f}")
    conn.disconnect()

    import csv
    with open("src/tests/bench/speed_sweep.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["cmd", "meanL", "rippleL", "meanR", "rippleR"])  # [mm/s]
        w.writerows(results)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(10, 7))
    for idx, label, colr in ((1, "left", "#1f77b4"), (3, "right", "#2ca02c")):
        xs = [r[0] for r in results]
        ys = [r[idx] for r in results]
        es = [r[idx + 1] for r in results]
        ax.errorbar(xs, ys, yerr=es, color=colr, marker="o", ms=3, lw=1.0,
                    capsize=2, ls="none", label=f"{label} achieved (+- ripple)")
    lim = max(abs(r[0]) for r in results)
    ax.plot([-lim, lim], [-lim, lim], color="#999999", ls=":", lw=0.8,
            label="achieved == commanded")
    for edge in (GAIN * CRAWL_PULSE, -GAIN * CRAWL_PULSE):
        ax.axvline(edge, color="#cc7777", lw=0.8, ls="--", alpha=0.7)
    ax.annotate("crawl | continuous", xy=(GAIN * CRAWL_PULSE, 0),
                fontsize=8, rotation=90, va="bottom", ha="right", color="#cc7777")
    ax.axhline(0, color="black", lw=0.5)
    ax.axvline(0, color="black", lw=0.5)
    ax.set_xlabel("commanded wheel speed [mm/s]")
    ax.set_ylabel("achieved wheel speed [mm/s]")
    ax.set_title("Speed sweep -- crawl + continuous regimes, 2 s holds, real encoders")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig("src/tests/bench/speed_sweep.png", dpi=130)
    print("wrote src/tests/bench/speed_sweep.{csv,png}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
