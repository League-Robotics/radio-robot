#!/usr/bin/env python3
"""four_corners.py — autonomous tour: centre → corners → centre.

Visits, in order:
    1. tag 1 (centre, 0,0)
    2. green   (SE,  +35,-24)
    3. purple  (NW,  -35,+24)
    4. tag 1
    5. orange  (NE,  +35,+24)
    6. blue     (SW, -35,-24)
    7. tag 1

Pure deterministic control — no AI in the loop. Heading comes straight from the
robot tag's orientation_yaw (CCW-positive, 0=north): forward = (-sin yaw, cos yaw).
Each leg: TURN to face the waypoint (RT bulk + crawl fine, sign found by probe),
then DRIVE forward in camera-measured hops (endpoint-guarded; a stall triggers a
stronger break-free burst). Connects once and runs the whole route.

  uv run python tests/bench/four_corners.py
"""
import argparse
import math
import pathlib
import sys
import time

_BENCH = pathlib.Path(__file__).resolve().parent
if str(_BENCH) not in sys.path:
    sys.path.insert(0, str(_BENCH))
from bench_safety import BenchRun  # noqa: E402

ROBOT = 100
FX, FY = 67 - 9, 44.65 - 9        # A1-centred geofence from the ArUco corners

# Waypoints (A1-centred cm). "Corners" are the diagonal colored squares.
WP = {
    "tag1":   (0.0, 0.0),
    "green":  (35.0, -24.0),
    "purple": (-35.0, 24.0),
    "orange": (35.0, 24.0),
    "blue":   (-35.0, -24.0),
}
ROUTE = ["tag1", "green", "purple", "tag1", "orange", "blue", "tag1"]


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def in_fence(x, y):
    return -FX <= x <= FX and -FY <= y <= FY


def open_daemon():
    from aprilcam.config import Config
    from aprilcam.client.control import DaemonControl
    dc = DaemonControl.connect_default(Config.load())
    return dc, dc.list_cameras()[0]


def get_tag(dc, cam, tid, timeout=1.0):
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
    ap.add_argument("--port", default="/dev/cu.usbmodem2121302")
    ap.add_argument("--arrive", type=float, default=4.0)
    ap.add_argument("--tol", type=float, default=4.0, help="heading tol deg")
    ap.add_argument("--speed", type=int, default=140)
    ap.add_argument("--laps", type=int, default=1)
    args = ap.parse_args()

    dc, cam = open_daemon()
    from robot_radio.io.cli import _make_robot

    class A:
        port = args.port
        verbose = False

    robot, conn, _ = _make_robot(A())
    proto = robot._proto
    proto.send("SET sTimeout=700", 200)

    def stop():
        for _ in range(2):
            proto.stop(); time.sleep(0.03)

    def crawl(deg, dps=28.0):
        if abs(deg) < 0.3:
            return
        om = int(round(math.copysign(dps, deg) * math.pi / 180.0 * 1000.0))
        proto.send(f"_VW 0 {om}", 70)
        time.sleep(max(0.06, min(0.5, abs(deg) / dps)))
        stop(); time.sleep(0.15)

    def rt(deg):
        proto.send(f"RT {int(round(deg * 100))} #1", 200)
        proto.wait_for_evt_done("RT", timeout=12000, corr_id="1")
        time.sleep(0.2); stop()

    def tgt_yaw(rx, ry, tx, ty):
        return math.atan2(rx - tx, ty - ry)

    def face(tx, ty):
        r = robot_pose(dc, cam)
        if r is None:
            return
        rx, ry, cyaw = r
        if abs(math.degrees(wrap(tgt_yaw(rx, ry, tx, ty) - cyaw))) <= args.tol:
            return
        crawl(8.0)                                     # probe sign
        r1 = robot_pose(dc, cam)
        s = 1.0 if (r1 and wrap(r1[2] - cyaw) >= 0.0) else -1.0
        for _ in range(12):
            r = robot_pose(dc, cam)
            if r is None:
                continue
            rx, ry, cyaw = r
            e = math.degrees(wrap(tgt_yaw(rx, ry, tx, ty) - cyaw))
            if abs(e) <= args.tol:
                return
            cmd = s * e
            if abs(cmd) > 20.0:
                rt(cmd)
            else:
                crawl(math.copysign(max(2.5, abs(cmd) * 0.6), cmd))

    def reacquire():
        """Pose read tolerant of transient blur: stop and retry before giving up."""
        r = robot_pose(dc, cam)
        if r is not None:
            return r
        stop()
        for _ in range(8):
            time.sleep(0.2)
            r = robot_pose(dc, cam)
            if r is not None:
                return r
        return None

    def drive_to(tx, ty, name):
        last = (0.0, 0.0)
        for _ in range(16):
            r = reacquire()
            if r is None:
                return "LOST", last, 0.0
            rx, ry, cyaw = r
            last = (rx, ry)
            d = math.hypot(tx - rx, ty - ry)
            if d <= args.arrive:
                return "OK", (rx, ry), d
            face(tx, ty)
            r = reacquire()
            if r is None:
                return "LOST", last, 0.0
            rx, ry, cyaw = r
            last = (rx, ry)
            if abs(math.degrees(wrap(tgt_yaw(rx, ry, tx, ty) - cyaw))) > 35.0:
                continue
            fwd = (-math.sin(cyaw), math.cos(cyaw))
            hop = min(18.0, d - args.arrive * 0.5)
            ex, ey = rx + hop * fwd[0], ry + hop * fwd[1]
            while hop > 2.0 and not in_fence(ex, ey):
                hop -= 2.0
                ex, ey = rx + hop * fwd[0], ry + hop * fwd[1]
            if hop <= 2.0:
                return "FENCE", (rx, ry), d
            sx, sy, d0 = rx, ry, d
            spd = args.speed
            miss = 0
            t0 = time.monotonic()
            while time.monotonic() - t0 < 5.0:
                proto.drive(spd, spd)
                time.sleep(0.1)
                rr = get_tag(dc, cam, ROBOT, 0.12)
                if rr:
                    miss = 0
                    moved = math.hypot(rr[0] - sx, rr[1] - sy)
                    dist = math.hypot(tx - rr[0], ty - rr[1])
                    if dist > d0 + 5.0:
                        break
                    if moved >= hop or dist <= args.arrive or not in_fence(rr[0], rr[1]):
                        break
                    # stall → stronger break-free burst
                    if time.monotonic() - t0 > 1.0 and moved < 1.5:
                        spd = 210
                else:
                    miss += 1
                    if miss >= 6:        # ~0.7s blind — stop and re-acquire up top
                        break
            stop()
        return "MAXHOPS", (rx, ry), d

    t_start = time.monotonic()
    errs = []
    try:
        with BenchRun(proto, max_seconds=len(ROUTE) * args.laps * 30 + 60):
            for lap in range(args.laps):
                for name in ROUTE:
                    tx, ty = WP[name]
                    st, xy, d = drive_to(tx, ty, name)
                    errs.append((name, st, d))
                    print(f"  {name:7s} → ({xy[0]:+5.1f},{xy[1]:+5.1f})  {d:4.1f}cm  [{st}]")
        dt = time.monotonic() - t_start
        oks = [d for _, s, d in errs if s == "OK"]
        print(f"\n  tour done in {dt:.0f}s — {len(oks)}/{len(errs)} legs arrived, "
              f"mean {sum(oks)/len(oks):.1f}cm  max {max(oks):.1f}cm" if oks else
              f"\n  tour done in {dt:.0f}s")
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
