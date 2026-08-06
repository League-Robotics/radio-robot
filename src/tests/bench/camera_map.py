#!/usr/bin/env python3
"""Solve the camera's robot-tag distortion, using the OTOS as the reference.

The camera reports the ROBOT's tag inflated, because that tag sits 117mm above
the playfield plane and the parallax is not corrected. Dividing by a single
factor about the field origin is only right if the camera's nadir IS the
origin -- and the residuals say it is not: the error runs ~17mm at SW and
~94mm at NE.

So fit the real thing. For an elevated tag the distortion is a homothety about
the nadir:

    camera = k * true + (1 - k) * centre

which is linear in (k, centre), so three unknowns -- k, centre_x, centre_y --
fall out of an ordinary least-squares fit over enough (camera, true) pairs.
The OTOS supplies `true`: it is tape-calibrated to 0.2%, which is an order of
magnitude better than the effect being measured.

Drives a spread of points, takes both fixes at rest at each, and prints the
fit plus its residuals.

    uv run python src/tests/bench/camera_map.py
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from goto_otos import (CAMERA, ROBOT_TAG, RELAY_PORT, Robot,  # noqa: E402
                       goto, inside_fence)

SETTLE = 1.5   # [s] rest dwell before a fix

# A spread that separates a scale from a centre offset: corners pin the
# gradient, the middle pins the offset. Collinear points cannot.
STATIONS = [("NE", (400.0, 240.0)), ("SE", (400.0, -240.0)),
            ("SW", (-400.0, -240.0)), ("NW", (-400.0, 240.0)),
            ("N", (0.0, 260.0)), ("S", (0.0, -260.0)),
            ("MID", (0.0, 0.0))]


def raw_camera(dc, tries: int = 25):
    """The robot tag's RAW camera position [mm] -- no de-inflation applied."""
    for _ in range(tries):
        tf = dc.get_tags(CAMERA)
        for t in getattr(tf, "tags", []):
            if t.id == ROBOT_TAG and getattr(t, "world_xy", None) is not None:
                return (float(t.world_xy[0]) * 10.0, float(t.world_xy[1]) * 10.0)
        time.sleep(0.12)
    return None


def fit(samples):
    """Least-squares k and centre for camera = k*true + (1-k)*centre."""
    n = len(samples)
    # Solve [true_x 1 0; true_y 0 1] [k bx by]' = [cam_x; cam_y]
    # Normal equations, built by hand to avoid a numpy dependency.
    Stt = Sxt = Syt = St = Sx = Sy = 0.0
    for (tx, ty), (cx, cy) in samples:
        Stt += tx * tx + ty * ty
        Sxt += cx * tx
        Syt += cy * ty
        St += 0.0
        Sx += 0.0
        Sy += 0.0
    Sum_tx = sum(s[0][0] for s in samples)
    Sum_ty = sum(s[0][1] for s in samples)
    Sum_cx = sum(s[1][0] for s in samples)
    Sum_cy = sum(s[1][1] for s in samples)

    # Unknowns k, bx, by:
    #   sum over x-rows and y-rows for k; bx from x-rows only; by from y-rows.
    # d(SSE)/dk, d/dbx, d/dby = 0 gives:
    #   k*Stt + bx*Sum_tx + by*Sum_ty = Sxt + Syt
    #   k*Sum_tx + n*bx                = Sum_cx
    #   k*Sum_ty +          n*by       = Sum_cy
    # Substitute bx, by and solve for k.
    denom = Stt - (Sum_tx * Sum_tx + Sum_ty * Sum_ty) / n
    if abs(denom) < 1e-9:
        return None
    k = ((Sxt + Syt) - (Sum_tx * Sum_cx + Sum_ty * Sum_cy) / n) / denom
    bx = (Sum_cx - k * Sum_tx) / n
    by = (Sum_cy - k * Sum_ty) / n
    if abs(1.0 - k) < 1e-6:
        return k, None, None
    return k, (bx / (1.0 - k), by / (1.0 - k)), (bx, by)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=RELAY_PORT)
    args = ap.parse_args()

    from aprilcam.config import Config
    from aprilcam.client.control import DaemonControl
    dc = DaemonControl.connect_default(Config.load())

    from goto_otos import CAM_TAG_INFLATION
    seed_raw = raw_camera(dc)
    if seed_raw is None:
        print("camera cannot see tag 100 -- are the lights on?")
        return 1

    bot = Robot(args.port)
    samples = []
    try:
        tf = dc.get_tags(CAMERA)
        yaw = next((float(t.yaw) for t in getattr(tf, "tags", [])
                    if t.id == ROBOT_TAG), 0.0)
        sx, sy = seed_raw[0] / CAM_TAG_INFLATION, seed_raw[1] / CAM_TAG_INFLATION
        print(f"seeding OTOS at ({sx:.1f}, {sy:.1f}) {math.degrees(yaw):.1f}deg")
        if not bot.seed(sx, sy, yaw):
            print("seed did not read back -- stopping")
            return 1

        for label, target in STATIONS:
            if not inside_fence(*target):
                continue
            goto(bot, dc, target, label)
            time.sleep(SETTLE)
            cam = raw_camera(dc)
            otos = bot.pose_blocking()
            if cam is None or otos is None:
                print(f"     {label}: lost a fix, skipping")
                continue
            samples.append(((otos[0], otos[1]), cam))
            print(f"     {label}: otos=({otos[0]:7.1f},{otos[1]:7.1f})  "
                  f"camera_raw=({cam[0]:7.1f},{cam[1]:7.1f})")
    finally:
        bot.close()

    if len(samples) < 4:
        print(f"only {len(samples)} usable stations -- need 4+ to fit")
        return 1

    result = fit(samples)
    if result is None:
        print("degenerate station geometry -- points too collinear to fit")
        return 1
    k, centre, offset = result

    print()
    print(f"fit over {len(samples)} stations:")
    print(f"  inflation k = {k:.4f}   (single-factor assumption used {CAM_TAG_INFLATION})")
    if centre:
        print(f"  nadir       = ({centre[0]:.1f}, {centre[1]:.1f}) mm "
              f"-- the origin assumption said (0, 0)")

    worst_old = worst_new = 0.0
    print()
    print(f"  {'station':<9}{'old err':>10}{'fitted err':>12}")
    for (tx, ty), (cx, cy) in samples:
        ox = math.hypot(cx / CAM_TAG_INFLATION - tx, cy / CAM_TAG_INFLATION - ty)
        px, py = k * tx + offset[0], k * ty + offset[1]
        nx = math.hypot(cx - px, cy - py)
        worst_old, worst_new = max(worst_old, ox), max(worst_new, nx)
        print(f"  {'':<9}{ox:9.1f}mm{nx:11.1f}mm")
    print(f"  {'worst':<9}{worst_old:9.1f}mm{worst_new:11.1f}mm")
    print()
    print("  apply as:  true = (camera - offset) / k")
    print(f"             offset = ({offset[0]:.2f}, {offset[1]:.2f})  k = {k:.4f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
