#!/usr/bin/env python3
"""drive2.py — camera closed-loop drive to a named colored square.

Behaviour (turn-then-go, NOT arcing):
  1. Resolve the square BY NAME from playfield.json (same map the aprilcam
     `where` tool uses).
  2. MEASURE the robot's forward heading with one small, slow forward nudge —
     the displacement direction in world coords IS forward. This avoids guessing
     the tag-yaw→heading convention (which is what made earlier versions arc off
     in the wrong direction).
  3. Repeat until the tag is over the square:
        - TURN IN PLACE to face the square (firmware RT, encoder dead-reckoning),
        - DRIVE STRAIGHT a short segment, then stop and re-check the camera.
     So directions are re-planned multiple times per run, and every move is a
     clean turn-in-place or a straight line — never an arc.
  4. Speed scales with distance: full until 30 cm out, then ramps to a crawl so
     it settles the tag on the square.

World frame: A1-centred, origin at AprilTag 1, +x east, +y north, centimetres.

Safety: geofence (stay on the table), stop-and-reacquire on camera loss, a
firmware keepalive watchdog (stops if this process dies), overall timeout, and
a wrong-way guard.

  uv run python tests/bench/drive2.py "northwest purple"
  uv run python tests/bench/drive2.py "east red" --speed 150 --arrive 4
  uv run python tests/bench/drive2.py "south magenta" --plan-only   # no motion
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time

DEFAULT_PLAYFIELD = "/Volumes/Proj/proj/RobotProjects/AprilTags/data/aprilcam/playfield.json"
ROBOT_TAG_ID = 100

# Speed profile (mm/s) and geometry (cm).
V_MAX = 150
V_MIN = 60
SLOW_RADIUS_CM = 30
ARRIVE_CM = 4.0
FACE_TOL_DEG = 7.0       # re-turn in place when heading error exceeds this
SEG_DT = 0.45            # straight-segment drive time (s) between camera re-checks
NUDGE_SPEED = 80         # heading-calibration nudge speed (mm/s)
NUDGE_DT = 0.6           # heading-calibration nudge time (s)
NUDGE_MIN_CM = 1.5       # nudge must move at least this far or we abort
CAM_POLL_S = 0.1
LOST_ABORT_S = 3.0       # give up only after the tag is gone this long
S_TIMEOUT_MS = 500       # firmware keepalive watchdog
GEOFENCE_MARGIN_CM = 14.0


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


# ---------------------------------------------------------------------------
# Target square resolution (mirrors the aprilcam `where` keyword search)
# ---------------------------------------------------------------------------
_FILLER = {"the", "square", "rectangle", "rect", "dot", "colored", "coloured",
           "block", "tile", "go", "to", "drive"}
_CARDINAL_SYN = {"nw": "northwest", "ne": "northeast", "sw": "southwest",
                 "se": "southeast", "n": "north", "s": "south", "e": "east", "w": "west"}


def load_features(path: str) -> list[dict]:
    with open(path) as f:
        pf = json.load(f)
    return [it for key in ("rectangles", "dots") for it in pf.get(key, []) if it.get("color")]


def resolve_target(query: str, feats: list[dict]) -> dict:
    toks = [_CARDINAL_SYN.get(t, t) for t in query.lower().replace("-", " ").split()
            if t not in _FILLER]
    if not toks:
        raise SystemExit("Error: empty target name")

    def score(feat) -> int:
        hay = " ".join(str(feat.get(k, "")) for k in ("color", "cardinal", "slug")).lower()
        return sum(1 for t in toks if t in hay)

    scored = [(score(f), f) for f in feats]
    best = max(s for s, _ in scored)
    if best < len(toks):
        opts = ", ".join(f'{f["color"]} {f["cardinal"]}' for f in feats if f["type"] == "rectangle")
        raise SystemExit(f"Error: no square matches '{query}'. Squares: {opts}")
    winners = [f for s, f in scored if s == best]
    rects = [f for f in winners if f.get("type") == "rectangle"]
    pool = rects or winners
    if len(pool) > 1:
        opts = ", ".join(f'{f["color"]} {f["cardinal"]} ({f["type"]})' for f in pool)
        raise SystemExit(f"Error: '{query}' is ambiguous — matches: {opts}. Add the cardinal.")
    return pool[0]


# ---------------------------------------------------------------------------
# Camera daemon
# ---------------------------------------------------------------------------
def open_daemon(cam_index: int):
    from aprilcam.config import Config
    from aprilcam.client.control import DaemonControl
    dc = DaemonControl.connect_default(Config.load())
    cams = dc.list_cameras()
    cam = cams[0] if cams else dc.open_camera(index=cam_index)
    return dc, cam


def read_pose(dc, cam, timeout_s=2.0):
    """Return (x_cm, y_cm, yaw_rad) for the robot tag, or None."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        for t in dc.get_tags(cam).tags:
            if t.id == ROBOT_TAG_ID and t.world_xy is not None and t.yaw is not None:
                return float(t.world_xy[0]), float(t.world_xy[1]), float(t.yaw)
        time.sleep(0.03)
    return None


def geofence(path: str, margin_cm: float):
    with open(path) as f:
        pf = json.load(f)
    xs = [a["x"] for a in pf.get("aruco_tags", [])]
    ys = [a["y"] for a in pf.get("aruco_tags", [])]
    if not xs:
        return (-1e9, 1e9, -1e9, 1e9)
    return (min(xs) + margin_cm, max(xs) - margin_cm, min(ys) + margin_cm, max(ys) - margin_cm)


def in_fence(x, y, fence) -> bool:
    return fence[0] <= x <= fence[1] and fence[2] <= y <= fence[3]


def draw(dc, cam, rx, ry, fwd, tx, ty):
    try:
        hx, hy = rx + 12.0 * math.cos(fwd), ry + 12.0 * math.sin(fwd)
        dc.publish_overlay(cam, [
            {"type": "point", "params": [tx, ty, 4.0], "color": [255, 220, 0], "thickness": -1},
            {"type": "arrow", "params": [rx, ry, tx, ty], "color": [150, 150, 150], "thickness": 2},
            {"type": "point", "params": [rx, ry, 5.0], "color": [70, 130, 255], "thickness": -1},
            {"type": "arrow", "params": [rx, ry, hx, hy], "color": [240, 240, 240], "thickness": 3},
        ], ttl=0.6)
    except Exception:
        pass


def v_for_dist(dist_cm: float, vmax: float) -> float:
    if dist_cm >= SLOW_RADIUS_CM:
        return vmax
    return max(V_MIN, vmax * (dist_cm / SLOW_RADIUS_CM))


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("target", help='colored square name, e.g. "northwest purple"')
    ap.add_argument("--port", default=None)
    ap.add_argument("--cam-index", type=int, default=1)
    ap.add_argument("--playfield", default=DEFAULT_PLAYFIELD)
    ap.add_argument("--speed", type=int, default=V_MAX, help="max speed mm/s")
    ap.add_argument("--arrive", type=float, default=ARRIVE_CM, help="arrival tolerance cm")
    ap.add_argument("--margin", type=float, default=GEOFENCE_MARGIN_CM)
    ap.add_argument("--timeout", type=float, default=90.0)
    ap.add_argument("--plan-only", action="store_true")
    args = ap.parse_args()

    feats = load_features(args.playfield)
    target = resolve_target(args.target, feats)
    tx, ty = float(target["x"]), float(target["y"])
    tname = f'{target["color"]} {target["cardinal"]} {target["type"]}'
    print(f"target: {tname}  @ world ({tx:+.0f}, {ty:+.0f}) cm")

    dc, cam = open_daemon(args.cam_index)
    fence = geofence(args.playfield, args.margin)
    print(f"geofence: x∈[{fence[0]:+.0f},{fence[1]:+.0f}] y∈[{fence[2]:+.0f},{fence[3]:+.0f}] cm")
    if not in_fence(tx, ty, fence):
        dc.close()
        raise SystemExit("target is outside the geofence — refusing")

    pose = read_pose(dc, cam, timeout_s=3.0)
    if pose is None:
        dc.close()
        raise SystemExit(f"camera did not see robot tag {ROBOT_TAG_ID} (calibrated)")
    rx, ry, cyaw = pose
    dist0 = math.hypot(tx - rx, ty - ry)
    print(f"robot @ ({rx:+.0f},{ry:+.0f}) cm  tag_yaw {math.degrees(cyaw):+.0f}°  dist {dist0:.1f}cm")
    draw(dc, cam, rx, ry, cyaw, tx, ty)

    if args.plan_only:
        print("plan-only: not moving.")
        dc.publish_overlay(cam, [], ttl=0)
        dc.close()
        return 0

    from robot_radio.io.cli import _make_robot
    from robot_radio.robot.protocol import NezhaProtocol

    class _A:
        port = args.port
        verbose = False

    robot, conn, _ = _make_robot(_A())
    proto = getattr(robot, "_proto", None)
    if not isinstance(proto, NezhaProtocol):
        conn.disconnect()
        dc.close()
        raise SystemExit("need a Nezha robot with NezhaProtocol")

    def stop_all():
        for _ in range(3):
            proto.stop()
            time.sleep(0.03)

    def turn_in_place(deg: float):
        """RT relative turn, blocking on EVT done RT."""
        corr = "1"
        proto.send(f"RT {int(round(deg * 100))} #{corr}", 400)
        proto.wait_for_evt_done("RT", timeout_ms=15000, corr_id=corr)
        time.sleep(0.15)

    status = "?"
    try:
        proto.send(f"SET sTimeout={S_TIMEOUT_MS}", 200)

        # ---- 0) MEASURE forward heading with a small straight nudge ----
        p0 = read_pose(dc, cam, 1.5)
        if p0 is None:
            raise RuntimeError("lost tag before nudge")
        x0, y0, _ = p0
        proto.drive(NUDGE_SPEED, NUDGE_SPEED)
        t = time.monotonic()
        while time.monotonic() - t < NUDGE_DT:
            proto.drive(NUDGE_SPEED, NUDGE_SPEED)
            time.sleep(0.1)
        stop_all()
        time.sleep(0.25)
        p1 = read_pose(dc, cam, 1.5)
        if p1 is None:
            raise RuntimeError("lost tag after nudge")
        x1, y1, _ = p1
        moved = math.hypot(x1 - x0, y1 - y0)
        if moved < NUDGE_MIN_CM:
            status = "NO_MOVE"
            raise RuntimeError(f"nudge moved only {moved:.1f}cm — motors/relay?")
        fwd = math.atan2(y1 - y0, x1 - x0)   # MEASURED forward heading (world)
        print(f"measured forward heading = {math.degrees(fwd):+.0f}° "
              f"(nudge {moved:.1f}cm)")

        # ---- main loop: turn-to-face, then straight segment, re-check ----
        t_start = time.monotonic()
        last_seen = time.monotonic()
        min_dist = dist0
        wrongway = 0
        rx, ry = x1, y1
        while True:
            now = time.monotonic()
            if now - t_start > args.timeout:
                status = "TIMEOUT"
                break
            p = read_pose(dc, cam, 0.12)
            if p is None:
                stop_all()
                if now - last_seen > LOST_ABORT_S:
                    status = "CAMERA_LOST"
                    break
                continue
            last_seen = now
            rx, ry, cyaw = p
            dist = math.hypot(tx - rx, ty - ry)
            bearing = math.atan2(ty - ry, tx - rx)
            err = wrap(bearing - fwd)
            draw(dc, cam, rx, ry, fwd, tx, ty)

            if dist <= args.arrive:
                status = "ARRIVED"
                break
            if not in_fence(rx, ry, fence):
                status = "GEOFENCE"
                break
            min_dist = min(min_dist, dist)
            wrongway = wrongway + 1 if dist > min_dist + 8.0 else 0
            if wrongway >= 4:
                status = "DIVERGING"
                break

            # 1) face the target if we're off by more than the tolerance
            if abs(err) > math.radians(FACE_TOL_DEG):
                print(f"  turn {math.degrees(err):+.0f}° to face (dist {dist:.0f}cm)")
                turn_in_place(math.degrees(err))
                fwd = wrap(fwd + err)        # predicted; corrected by the next segment
                continue

            # 2) drive a straight segment toward the target
            v = v_for_dist(dist, args.speed)
            sx, sy = rx, ry
            seg_t = time.monotonic()
            broke = None
            while time.monotonic() - seg_t < SEG_DT:
                proto.drive(int(v), int(v))      # straight: equal wheels
                time.sleep(CAM_POLL_S)
                p = read_pose(dc, cam, 0.08)
                if p is not None:
                    last_seen = time.monotonic()
                    rx, ry, cyaw = p
                    draw(dc, cam, rx, ry, fwd, tx, ty)
                    if math.hypot(tx - rx, ty - ry) <= args.arrive:
                        broke = "ARRIVED"
                        break
                    if not in_fence(rx, ry, fence):
                        broke = "GEOFENCE"
                        break
            stop_all()
            time.sleep(0.12)
            # re-measure forward heading from the segment displacement (if useful)
            p = read_pose(dc, cam, 0.8)
            if p is not None:
                rx, ry, cyaw = p
                if math.hypot(rx - sx, ry - sy) > 2.0:
                    fwd = math.atan2(ry - sy, rx - sx)
            if broke in ("ARRIVED", "GEOFENCE"):
                status = broke
                break

        stop_all()
        time.sleep(0.3)
        p = read_pose(dc, cam, 1.0)
        if p is not None:
            rx, ry, _ = p
            d = math.hypot(tx - rx, ty - ry)
            print(f"[{status}] final: robot ({rx:+.0f},{ry:+.0f}) cm  {d:.1f}cm from {tname}")
        else:
            print(f"[{status}] (no final camera fix)")
    finally:
        try:
            stop_all()
            dc.publish_overlay(cam, [], ttl=0)
        except Exception:
            pass
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
