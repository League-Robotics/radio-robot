#!/usr/bin/env python3
"""goto_tag.py — turn to face a target tag, THEN drive to it.

Heading comes straight from the robot tag's own orientation_yaw (CCW-positive,
0 = north), so forward_unit = (-sin yaw, cos yaw). No calibration drive, no
offset. To face a point: target_yaw = atan2(-dx, dy).

Order is strict: TURN until facing the target, THEN drive forward in short hops,
re-facing between hops. The RT turn sign is found by a small in-place probe, and
a forward hop is refused if it would leave the field or head away from the goal.

  uv run python tests/bench/goto_tag.py --tag 1
"""
import argparse
import math
import sys
import time

ROBOT = 100


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def open_daemon():
    from aprilcam.config import Config
    from aprilcam.client.control import DaemonControl
    dc = DaemonControl.connect_default(Config.load())
    cam = dc.list_cameras()[0]
    return dc, cam


def get_tag(dc, cam, tid, timeout=1.5):
    dl = time.monotonic() + timeout
    while time.monotonic() < dl:
        for t in dc.get_tags(cam).tags:
            if t.id == tid and t.world_xy is not None and t.yaw is not None:
                return float(t.world_xy[0]), float(t.world_xy[1]), float(t.yaw)
        time.sleep(0.02)
    return None


def robot_pose(dc, cam, n=4):
    """Median-filtered robot (x, y, yaw)."""
    xs = ys = mx = my = 0.0
    got = 0
    for _ in range(n):
        r = get_tag(dc, cam, ROBOT, 0.4)
        if r:
            xs += r[0]; ys += r[1]
            mx += math.cos(r[2]); my += math.sin(r[2])
            got += 1
        time.sleep(0.02)
    if not got:
        return None
    return (xs / got, ys / got, math.atan2(my, mx))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", type=int, default=None, help="target tag id")
    ap.add_argument("--x", type=float, default=None, help="target world x (cm)")
    ap.add_argument("--y", type=float, default=None, help="target world y (cm)")
    ap.add_argument("--name", default="target", help="label for the target")
    ap.add_argument("--port", default="/dev/cu.usbmodem2121302")
    ap.add_argument("--arrive", type=float, default=5.0)
    ap.add_argument("--tol", type=float, default=4.0, help="heading tolerance deg")
    ap.add_argument("--speed", type=int, default=140)
    args = ap.parse_args()

    dc, cam = open_daemon()
    if args.x is not None and args.y is not None:
        tx, ty = args.x, args.y
        tname = args.name
        print(f"target {tname} @ ({tx:+.1f},{ty:+.1f}) cm")
    else:
        tid = args.tag if args.tag is not None else 1
        tgt = get_tag(dc, cam, tid, 3.0)
        if tgt is None:
            dc.close()
            raise SystemExit(f"target tag {tid} not seen")
        tx, ty, _ = tgt
        tname = f"tag {tid}"
        print(f"target {tname} @ ({tx:+.1f},{ty:+.1f}) cm")

    from robot_radio.io.cli import _make_robot

    class A:
        port = args.port
        verbose = False

    robot, conn, _ = _make_robot(A())
    proto = robot._proto
    proto.send("SET sTimeout=700", 200)

    FX, FY = 67 - 10, 44.65 - 10   # aruco-based fence (A1-centred), margin 10

    def in_fence(x, y):
        return -FX <= x <= FX and -FY <= y <= FY

    def stop():
        for _ in range(2):
            proto.stop()
            time.sleep(0.03)

    def crawl(deg, dps=25.0):
        if abs(deg) < 0.3:
            return
        om = int(round(math.copysign(dps, deg) * math.pi / 180.0 * 1000.0))
        dur = max(0.06, min(0.5, abs(deg) / dps))
        proto.send(f"_VW 0 {om}", 80)
        time.sleep(dur)
        stop()
        time.sleep(0.18)

    def rt(deg):
        proto.send(f"RT {int(round(deg * 100))} #1", 200)
        proto.wait_for_evt_done("RT", timeout_ms=15000, corr_id="1")
        time.sleep(0.25)
        stop()

    def target_yaw(rx, ry):
        return math.atan2(rx - tx, ty - ry)   # = atan2(-dx, dy)

    def face():
        """Turn (in place) until the robot faces the target. Returns final err°."""
        r = robot_pose(dc, cam)
        if r is None:
            return 999.0
        rx, ry, cyaw = r
        if abs(math.degrees(wrap(target_yaw(rx, ry) - cyaw))) <= args.tol:
            return math.degrees(wrap(target_yaw(rx, ry) - cyaw))
        # probe RT/omega sign: small in-place turn, see which way yaw moved
        crawl(8.0)
        r1 = robot_pose(dc, cam)
        s = 1.0 if (r1 and wrap(r1[2] - cyaw) >= 0.0) else -1.0
        for _ in range(14):
            r = robot_pose(dc, cam)
            if r is None:
                continue
            rx, ry, cyaw = r
            e = math.degrees(wrap(target_yaw(rx, ry) - cyaw))
            print(f"    facing: yaw {math.degrees(cyaw):+.0f}°  want "
                  f"{math.degrees(target_yaw(rx, ry)):+.0f}°  err {e:+.0f}°")
            if abs(e) <= args.tol:
                return e
            cmd = s * e
            if abs(cmd) > 20.0:
                rt(cmd)
            else:
                crawl(math.copysign(max(2.5, abs(cmd) * 0.6), cmd))
        return e

    status = "?"
    try:
        for hop_i in range(14):
            r = robot_pose(dc, cam)
            if r is None:
                status = "LOST"
                break
            rx, ry, cyaw = r
            d = math.hypot(tx - rx, ty - ry)
            print(f"\n  at ({rx:+.1f},{ry:+.1f}) yaw {math.degrees(cyaw):+.0f}°  "
                  f"{d:.1f}cm to {tname}")
            if d <= args.arrive:
                status = "ARRIVED"
                break

            # 1) TURN to face the target
            face()

            # 2) endpoint guard — must be facing it and stay on the field
            r = robot_pose(dc, cam)
            if r is None:
                continue
            rx, ry, cyaw = r
            if abs(math.degrees(wrap(target_yaw(rx, ry) - cyaw))) > 35.0:
                print("    still not facing target — re-turning")
                continue
            fwd = (-math.sin(cyaw), math.cos(cyaw))
            hop = min(15.0, d - args.arrive * 0.5)
            ex, ey = rx + hop * fwd[0], ry + hop * fwd[1]
            while hop > 2.0 and not in_fence(ex, ey):
                hop -= 2.0
                ex, ey = rx + hop * fwd[0], ry + hop * fwd[1]
            if hop <= 2.0:
                print("    forward would leave the field — stopping")
                status = "FENCE"
                break

            # 3) DRIVE forward `hop` cm (camera-measured), stop on arrival/divergence
            print(f"    driving forward {hop:.0f}cm toward target")
            sx, sy = rx, ry
            d0 = d
            t0 = time.monotonic()
            while time.monotonic() - t0 < 6.0:
                proto.drive(args.speed, args.speed)
                time.sleep(0.1)
                rr = get_tag(dc, cam, ROBOT, 0.12)
                if rr:
                    moved = math.hypot(rr[0] - sx, rr[1] - sy)
                    dist = math.hypot(tx - rr[0], ty - rr[1])
                    if dist > d0 + 5.0:        # going the WRONG way — abort hop
                        print("    distance increasing — stopping (wrong way)")
                        break
                    if moved >= hop or dist <= args.arrive or not in_fence(rr[0], rr[1]):
                        break
            stop()
        else:
            status = "MAXHOPS"

        r = robot_pose(dc, cam)
        if r:
            d = math.hypot(tx - r[0], ty - r[1])
            print(f"\n  [{status}] final ({r[0]:+.1f},{r[1]:+.1f}) — {d:.1f}cm from {tname}")
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
    return 0 if status == "ARRIVED" else 1


if __name__ == "__main__":
    sys.exit(main())
