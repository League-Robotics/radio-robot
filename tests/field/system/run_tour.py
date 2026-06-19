#!/usr/bin/env python3
"""run_tour.py — drive a sequence of world-coord colored-square targets.

For each label in --order, runs ``goto_world.py`` (which localizes the robot via
the camera, computes the standard world→robot G, sim-gates it, drives closed-loop
with camera correction, and re-localizes), parses its machine-readable RESULT
line, and appends a CSV row (start / end / target / error / arrived / passes).

Safety: the tour STOPS immediately if any leg reports a BOUNDS-ABORT or ends
outside a hard safe box (|x|>--hard-x or |y|>--hard-y).  Targets come from
``square_targets.json`` (colored squares detected live via the MCP camera).

    uv run python tests/system/run_tour.py --order red-E orange-NE red-W ...
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))

_RESULT = re.compile(
    r"RESULT start=\(([-\d.]+),([-\d.]+),([-\d.]+)\) "
    r"end=\(([-\d.]+),([-\d.]+),([-\d.]+)\) "
    r"target=\(([-\d.]+),([-\d.]+)\) error=([-\d.]+) "
    r"arrived=(\w+) passes=(\d+)"
)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--order", nargs="+", required=True, help="square labels, in order")
    ap.add_argument("--robot-tag", type=int, default=100)
    ap.add_argument("--field-x", type=float, default=42.0)
    ap.add_argument("--field-y", type=float, default=40.0)
    ap.add_argument("--arrive", type=float, default=5.0)
    ap.add_argument("--speed", type=int, default=130)
    ap.add_argument("--hard-x", type=float, default=46.0)
    ap.add_argument("--hard-y", type=float, default=42.0)
    ap.add_argument("--csv", default=os.path.join(ROOT, "goto_world_results.csv"))
    a = ap.parse_args(argv)

    data = json.load(open(os.path.join(HERE, "square_targets.json")))
    sq = {s["label"]: s for s in data["squares"]}

    with open(a.csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["iter", "target", "target_x", "target_y", "start_x", "start_y",
                    "start_yaw", "end_x", "end_y", "end_yaw", "error_cm",
                    "arrived", "passes"])
        for i, label in enumerate(a.order, 1):
            if label not in sq:
                print(f"!! unknown square '{label}' — skipping"); continue
            t = sq[label]
            print(f"\n===== ITER {i}: {label} ({t['x']:+.1f},{t['y']:+.1f}) =====", flush=True)
            p = subprocess.run(
                [sys.executable, os.path.join(HERE, "goto_world.py"),
                 "--xy", str(t["x"]), str(t["y"]),
                 "--robot-tag", str(a.robot_tag),
                 "--field-x", str(a.field_x), "--field-y", str(a.field_y),
                 "--arrive", str(a.arrive), "--speed", str(a.speed)],
                capture_output=True, text=True,
            )
            out = p.stdout + p.stderr
            for line in out.splitlines():
                if any(k in line for k in ("robot @", "pass ", "SIM ", "leg outcome",
                                           "now @", "FINAL", "BOUNDS", "WARNING",
                                           "Traceback", "Error")):
                    print("   ", line, flush=True)
            m = _RESULT.search(out)
            if not m:
                print("   !! no RESULT line — STOPPING tour."); break
            sx, sy, syaw, ex, ey, eyaw, txx, tyy, err, arr, pas = m.groups()
            w.writerow([i, label, txx, tyy, sx, sy, syaw, ex, ey, eyaw, err, arr, pas])
            f.flush()
            print(f"   -> end=({ex},{ey}) error={err}cm arrived={arr} passes={pas}", flush=True)
            if "BOUNDS-ABORT" in out:
                print("   !! BOUNDS-ABORT — STOPPING tour for safety."); break
            if abs(float(ex)) > a.hard_x or abs(float(ey)) > a.hard_y:
                print(f"   !! ended outside hard safe box (±{a.hard_x}x±{a.hard_y}) "
                      "— STOPPING tour for safety."); break

    print(f"\nCSV written: {a.csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
