#!/usr/bin/env python3
"""tour_goto.py — random board tour using the firmware go-to (G) command.

Re-runnable. Each of N iterations:
  1. Read the robot's world pose from the camera.
  2. Rank all sites (8 colored squares + 4 orange dots) by distance, DROP the
     closest 4, and pick one at random from the rest (so every hop is a real
     drive, never a tiny nudge to an adjacent square).
  3. Drive to it with the firmware **G** arc go-to: compute the robot-relative
     (forward, left) target from the camera fix, send `G`, watch the camera, and
     re-issue G from a fresh fix until within tolerance. Camera-fenced for safety.

  uv run python tests/bench/tour_goto.py            # 10 iterations
  uv run python tests/bench/tour_goto.py --iters 20 --seed 7
"""
import argparse
import math
import random
import sys
import time

ROBOT = 100
# Hard safety fence (A1-centred): emergency-stop the robot before the ArUco
# edge. Targets live well inside (max ±50 / ±30).
SFX, SFY = 63.0, 41.0

SITES = {
    "NW-purple": (-35.0, 24.0), "N-black": (0.0, 24.0), "NE-orange": (35.0, 24.0),
    "E-red": (35.0, 0.0), "SE-green": (35.0, -24.0), "S-magenta": (0.0, -24.0),
    "SW-blue": (-35.0, -24.0), "W-red": (-35.0, 0.0),
    "NW-dot": (-50.0, 30.0), "NE-dot": (50.0, 30.0),
    "SE-dot": (50.0, -30.0), "SW-dot": (-50.0, -30.0),
}


def in_fence(x, y):
    return -SFX <= x <= SFX and -SFY <= y <= SFY


def open_daemon():
    from aprilcam.config import Config
    from aprilcam.client.control import DaemonControl
    dc = DaemonControl.connect_default(Config.load())
    return dc, dc.list_cameras()[0]


def get_tag(dc, cam, tid, timeout=0.6):
    dl = time.monotonic() + timeout
    while time.monotonic() < dl:
        for t in dc.get_tags(cam).tags:
            if t.id == tid and t.world_xy is not None and t.yaw is not None:
                return float(t.world_xy[0]), float(t.world_xy[1]), float(t.yaw)
        time.sleep(0.02)
    return None


def robot_pose(dc, cam, n=4):
    xs = ys = mx = my = 0.0
    got = 0
    for _ in range(n):
        r = get_tag(dc, cam, ROBOT, 0.35)
        if r:
            xs += r[0]; ys += r[1]
            mx += math.cos(r[2]); my += math.sin(r[2])
            got += 1
    return (xs / got, ys / got, math.atan2(my, mx)) if got else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iters", type=int, default=10)
    ap.add_argument("--seed", type=int, default=None, help="RNG seed (omit = fresh random)")
    ap.add_argument("--arrive", type=float, default=6.0, help="arrival tolerance cm")
    ap.add_argument("--speed", type=int, default=200, help="G drive speed mm/s")
    ap.add_argument("--drop", type=int, default=4, help="exclude the N closest sites")
    ap.add_argument("--port", default="/dev/cu.usbmodem2121302")
    args = ap.parse_args()
    rng = random.Random(args.seed)

    dc, cam = open_daemon()
    from robot_radio.io.cli import _make_robot

    class A:
        port = args.port
        verbose = False

    robot, conn, _ = _make_robot(A())
    proto = robot._proto
    proto.send("SET sTimeout=800", 200)

    def stop():
        for _ in range(2):
            proto.stop(); time.sleep(0.03)

    def goto_via_g(tx, ty):
        """Drive to world (tx,ty) with the firmware G arc go-to, camera-corrected."""
        for _ in range(6):
            p = robot_pose(dc, cam)
            if p is None:
                return "LOST", (0.0, 0.0), 0.0
            rx, ry, cyaw = p
            d = math.hypot(tx - rx, ty - ry)
            if d <= args.arrive:
                return "OK", (rx, ry), d
            # World→robot frame: forward_unit=(-sinθ,cosθ), left_unit=(-cosθ,-sinθ)
            c, s = math.cos(cyaw), math.sin(cyaw)
            dx, dy = tx - rx, ty - ry
            fwd = (-s * dx + c * dy) * 10.0   # cm→mm
            lft = (-c * dx - s * dy) * 10.0
            proto.send(f"G {int(round(fwd))} {int(round(lft))} {args.speed}", 200)
            # Monitor with the camera: stop on arrival / fence / stall / timeout.
            t0 = time.monotonic()
            last = (rx, ry)
            still = 0.0
            while time.monotonic() - t0 < 9.0:
                time.sleep(0.15)
                q = get_tag(dc, cam, ROBOT, 0.3)
                if not q:
                    continue
                qx, qy, _ = q
                if math.hypot(tx - qx, ty - qy) <= args.arrive:
                    break
                if not in_fence(qx, qy):
                    stop()
                    return "FENCE", (qx, qy), math.hypot(tx - qx, ty - qy)
                if math.hypot(qx - last[0], qy - last[1]) < 0.8:
                    still += 0.15
                    if still > 0.7:        # robot stopped (G done or stalled)
                        break
                else:
                    still = 0.0
                last = (qx, qy)
            stop()
        p = robot_pose(dc, cam)
        rx, ry = (p[0], p[1]) if p else (0.0, 0.0)
        return "MAXG", (rx, ry), math.hypot(tx - rx, ty - ry)

    errs = []
    t_start = time.monotonic()
    try:
        for it in range(args.iters):
            p = robot_pose(dc, cam)
            if p is None:
                print(f"[{it + 1}] lost the robot tag — stopping")
                break
            rx, ry, _ = p
            ranked = sorted(SITES.items(),
                            key=lambda kv: math.hypot(kv[1][0] - rx, kv[1][1] - ry))
            pool = ranked[args.drop:] or ranked   # drop closest N, keep the rest
            name, (tx, ty) = rng.choice(pool)
            dist = math.hypot(tx - rx, ty - ry)
            print(f"[{it + 1}/{args.iters}] from ({rx:+5.1f},{ry:+5.1f}) → "
                  f"{name:9s} ({tx:+.0f},{ty:+.0f})  {dist:4.0f}cm out")
            st, xy, d = goto_via_g(tx, ty)
            if st == "OK":
                errs.append(d)
            print(f"          arrived ({xy[0]:+5.1f},{xy[1]:+5.1f})  {d:4.1f}cm  [{st}]")
        dt = time.monotonic() - t_start
        if errs:
            print(f"\n{len(errs)}/{args.iters} reached in {dt:.0f}s — "
                  f"mean {sum(errs) / len(errs):.1f}cm  max {max(errs):.1f}cm")
        else:
            print(f"\ntour ended in {dt:.0f}s")
    finally:
        stop()
        try:
            conn.disconnect()
        except Exception:
            pass
        try:
            dc.close()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
