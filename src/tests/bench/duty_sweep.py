#!/usr/bin/env python3
"""duty_sweep.py -- measure the wheel plant: duty -> steady-state speed.

`App::Drive` is OPEN LOOP (`src/firm/app/drive.h:14`): it converts a commanded
wheel speed to duty via `dutyPerSpeed` and writes it, with no feedback. So
`dutyPerSpeed` IS the plant model, and when it is wrong the robot simply runs at
the wrong speed -- measured 2026-07-31 at 35% of commanded. Three constants in
the tree claim to be this gain and disagree by up to 2.34x. This settles it.

TWO CORRECTIONS TO THE PRE-2026-07-31 VERSION OF THIS SCRIPT
------------------------------------------------------------
1. **It drove through `move_wheels()` -- the PLANNER path.** `Motion::WheelTrim`
   runs a closed velocity loop there and actively corrects the exact error being
   measured, so the plant looked better than it is. This now uses the `WHEELS`
   verb (`NezhaProtocol.wheels()`), which `robot_loop.cpp:262-280` routes
   straight to Drive after `planner_.estop()` -- genuinely open loop.
2. **Its duty axis was wrong by ~1.75x.** It assumed `speed = duty * 1370`.
   Drive's actual chain, from `drive.cpp:81-138`, is:

       command = (|desired| - intercept[wheel][dir]) / gain[wheel][dir]
       duty    = command * dutyPerSpeed
       duty    = copysign(outputDeadband, duty)  if 0 < |duty| < outputDeadband

   Every constant lives in the robot JSON, so this inverts that chain to pick
   the commanded speed that produces each target duty. No reflash needed --
   reading the firmware's own arithmetic beats perturbing it.

THE ACCEL/DECEL SUBTLETY
------------------------
`correctedCommand()` picks coefficients with `(|desired| > |previous|) ? accel :
decel`, where `previous` is the previous COMMANDED speed. Holding a rung means
`desired == previous` after the first cycle, so steady state uses the **DECEL**
pair. Using the accel pair here would bias every fit.

    uv run python src/tests/bench/duty_sweep.py --port /dev/cu.usbmodem2121102
"""
from __future__ import annotations

import argparse
import csv
import json
import pathlib
import statistics
import sys
import time

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol

_REPO = pathlib.Path(__file__).resolve().parents[3]

DEFAULT_PORT = "/dev/cu.usbmodem2121102"

# App::Drive arms each WHEELS command until `commandDeadline_ = now + duration`
# (drive.cpp:16), so the host must re-arm well inside the window or the deadman
# fires mid-rung and the wheel coasts -- which reads as a soft plant rather than
# a lost lease. Refresh at ~1/5 of the lease. See
# clasi/issues/testgui-unmanaged-drive-lease-expiry-and-terminal-pivot.md.
LEASE = 300.0        # [ms]
REFRESH = 0.060      # [s]

SETTLE = 1.0         # [s] discarded while the wheel reaches the rung
DWELL = 1.0          # [s] sampled window at steady state
REST = 0.5           # [s] full stop between rungs, so each starts from stuck
MOTION = 5.0         # [mm/s] below this the wheel is considered not turning


def _load_control(robot: str) -> dict:
    """Drive's calibration, straight from the robot JSON.

    Raw JSON deliberately, NOT pydantic: `ControlConfig` silently drops keys it
    does not declare (clasi/issues/misc-changes-from-2026-07-31-session.md item
    4), and a silently-missing constant here would corrupt the whole x-axis.
    """
    path = _REPO / "data" / "robots" / f"{robot}.json"
    control = json.loads(path.read_text())["control"]
    missing = [k for k in ("duty_per_speed_left", "duty_per_speed_right",
                           "output_deadband") if control.get(k) is None]
    if missing:
        sys.exit(f"{path.name} missing required control keys: {missing}")
    return control


def _speed_for_duty(duty: float, wheel: str, c: dict) -> float:  # -> [mm/s]
    """Invert Drive's chain: the commanded speed that yields `duty`."""
    gain = c.get(f"wheel_gain_{wheel}_decel") or 1.0
    intercept = c.get(f"wheel_intercept_{wheel}_decel") or 0.0
    return duty / c[f"duty_per_speed_{wheel}"] * gain + intercept


def _fit(points):
    """Least-squares `speed = m*duty + b` over `points`; returns (m, b)."""
    n = len(points)
    mx = sum(p[0] for p in points) / n
    my = sum(p[1] for p in points) / n
    den = sum((x - mx) ** 2 for x, _ in points)
    m = sum((x - mx) * (y - my) for x, y in points) / den
    return m, my - m * mx


def _hold(proto, v_left, v_right):
    """Drive one rung open loop; return velocity samples from steady state."""
    samples = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < SETTLE + DWELL:
        proto.wheels(v_left, v_right, LEASE)
        for f in proto.read_binary_tlm_frames(int(REFRESH * 1000)) or []:
            if f.vel and time.monotonic() - t0 >= SETTLE:
                samples.append(f.vel)
    return samples


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--robot", default="tovez")
    ap.add_argument("--duty-min", type=float, default=0.04)
    ap.add_argument("--duty-max", type=float, default=0.60)
    ap.add_argument("--rungs", type=int, default=15)
    ap.add_argument("--fit-from", type=float, default=0.12,
                    help="ignore rungs below this duty when fitting the line "
                         "-- the deadband region is not linear")
    args = ap.parse_args()

    c = _load_control(args.robot)
    print(f"{args.robot}: duty_per_speed L={c['duty_per_speed_left']:.6f} "
          f"R={c['duty_per_speed_right']:.6f} deadband={c['output_deadband']}")

    step = (args.duty_max - args.duty_min) / (args.rungs - 1)
    duties = [round(args.duty_min + i * step, 4) for i in range(args.rungs)]

    conn = SerialConnection(port=args.port)
    conn.connect()
    proto = NezhaProtocol(conn)
    rows = []
    try:
        print(f"\n{'wheel':>5} {'dir':>4} {'duty':>6} {'cmd':>7} {'measured':>9}")
        print("-" * 36)
        for wheel in ("left", "right"):
            for direction in (+1, -1):
                for duty in duties:
                    speed = direction * _speed_for_duty(duty, wheel, c)
                    pair = (speed, 0.0) if wheel == "left" else (0.0, speed)
                    samples = _hold(proto, *pair)
                    proto.estop()
                    time.sleep(REST)
                    if not samples:
                        print("  no telemetry -- aborting")
                        return 1
                    idx = 0 if wheel == "left" else 1
                    meas = statistics.median(abs(s[idx]) for s in samples)
                    print(f"{wheel:>5} {'fwd' if direction > 0 else 'rev':>4} "
                          f"{duty:6.3f} {speed:7.0f} {meas:9.1f}")
                    rows.append((wheel, direction, duty, speed, meas,
                                 len(samples)))
    finally:
        try:
            proto.estop()
        except Exception:
            pass
        conn.disconnect()

    out_csv = _REPO / "src" / "tests" / "bench" / "duty_sweep.csv"
    with open(out_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["wheel", "direction", "duty", "commanded", "measured", "n"])
        w.writerows(rows)
    print(f"\nwrote {out_csv}")

    # --- dead zone: lowest duty that actually turned the wheel ---------------
    print("\nDEAD ZONE (lowest duty with sustained motion):")
    for wheel in ("left", "right"):
        for direction in (+1, -1):
            edge = next((r[2] for r in rows if r[0] == wheel
                         and r[1] == direction and r[4] > MOTION), None)
            print(f"  {wheel:<5} {'fwd' if direction > 0 else 'rev'}: {edge}")

    # --- plant gain ---------------------------------------------------------
    print(f"\nPLANT GAIN (fit over duty >= {args.fit_from}):")
    gains = {}
    for wheel in ("left", "right"):
        pts = [(r[2], r[4]) for r in rows
               if r[0] == wheel and r[2] >= args.fit_from and r[4] > MOTION]
        if len(pts) < 3:
            print(f"  {wheel}: too few usable points ({len(pts)})")
            continue
        m, b = _fit(pts)
        gains[wheel] = m
        print(f"  {wheel:<5} speed = {m:7.1f}*duty {b:+7.1f}   "
              f"=> duty_per_speed = {1.0 / m:.6f}  "
              f"(config {c[f'duty_per_speed_{wheel}']:.6f})")

    if len(gains) == 2:
        mean = (gains["left"] + gains["right"]) / 2.0
        spread = abs(gains["left"] - gains["right"])
        print(f"\nL/R spread: {spread:.1f} mm/s per unit duty "
              f"({spread / mean * 100:.1f}%)")
        print(f"RECOMMENDED shared default duty_per_speed = {1.0 / mean:.6f}")

    # --- chart --------------------------------------------------------------
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("(matplotlib unavailable -- skipping chart)")
        return 0

    fig, ax = plt.subplots(figsize=(10, 6))
    styles = {("left", 1): ("#1f77b4", "-"), ("left", -1): ("#1f77b4", "--"),
              ("right", 1): ("#2ca02c", "-"), ("right", -1): ("#2ca02c", "--")}
    for (wheel, direction), (colr, ls) in styles.items():
        pts = [(r[2], r[4]) for r in rows
               if r[0] == wheel and r[1] == direction]
        ax.plot([p[0] for p in pts], [p[1] for p in pts], color=colr, ls=ls,
                marker="o", ms=3,
                label=f"{wheel} {'fwd' if direction > 0 else 'rev'}")
    for wheel, m in gains.items():
        ax.axline((0, 0), slope=m, lw=0.8, ls=":", color="#666666")
    nominal = 1.0 / c["duty_per_speed_left"]
    ax.axline((0, 0), slope=nominal, color="#cc0000", lw=1.0, ls=":",
              label=f"config claims {nominal:.0f} mm/s per duty")
    ax.set_xlabel("duty [-]")
    ax.set_ylabel("steady-state speed [mm/s]")
    ax.set_title("Duty sweep -- open loop (WHEELS verb), real encoders")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    out_png = _REPO / "src" / "tests" / "bench" / "duty_sweep.png"
    fig.savefig(out_png, dpi=130)
    print(f"wrote {out_png}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
