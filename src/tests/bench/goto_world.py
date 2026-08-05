#!/usr/bin/env python
"""goto_world.py -- drive tovez to a world coordinate, closed on the camera.

    goto_world.py home               # the park pose: (60, 0) facing 180
    goto_world.py NE                 # an orange dot by cardinal
    goto_world.py 60 0               # explicit world x,y [cm]
    goto_world.py SW --heading 180   # also finish on a heading [deg]
    goto_world.py NE SE NE           # a sequence of waypoints

Every pass: read the overhead camera for the robot's true pose, decide the
move, execute it over the radio relay, then re-read the camera and report the
residual. Odometry is never trusted for position.

Move selection (thresholds measured on the bench 2026-08-05):
  * bearing error > TURN_FIRST_DEG -> pivot to face the target, then drive.
    Large pivots land within ~2.5 deg; small ones overshoot 1.5-3x and
    translate while they rotate, so they are avoided entirely.
  * otherwise -> ONE constant-curvature arc carrying v_x and omega together.
    Measured cross-track on a 36 cm arc: 1.4 mm.

Calibration, all camera-measured the same session:
  * distance over-delivers ~1.12x above 25 cm, ~1.35x below 5 cm
  * positive commanded omega DECREASES camera yaw (forward motion)
"""
from __future__ import annotations

import argparse
import json
import math
import time

RELAY_PORT = "/dev/cu.usbmodem214102"
CAMERA = "arducam-ov9782-usb-camera"
ROBOT_TAG = 100

CRUISE = 150.0          # [mm/s]
PIVOT_OMEGA = 1.6       # [rad/s]
YAW_SIGN = -1.0         # positive omega decreases camera yaw (forward)

TURN_FIRST_DEG = 30.0   # beyond this, pivot to face before moving
POS_TOL = 0.6           # [cm]
HEADING_TOL = 5.0       # [deg]
MAX_PASSES = 14   # legs are capped now, so more passes per waypoint

# Close targets that sit BEHIND the robot: reverse instead of pivoting round.
# A 159 deg pivot to reach a point 4 cm away moves the robot as much as the
# drive does; reversing is one command and preserves heading.
REVERSE_MAX_CM = 12.0
REVERSE_MIN_DEG = 120.0

# Never commit to more than this in one command. A 114 cm arc drove the robot
# to within 1.1 cm of the north edge before anything could look at the camera
# again -- the fence was only checked BETWEEN moves, so it could do nothing but
# narrate the excursion. Short legs + a re-fix is the standing playfield rule.
MAX_LEG_CM = 25.0

FENCE_X, FENCE_Y = 65.0, 42.0   # [cm] inside the +-67.15 / +-44.65 field

CARDINALS = {
    "NE": "northeast", "NW": "northwest",
    "SE": "southeast", "SW": "southwest",
}

#: Named poses. HOME is the session's start/park pose.
NAMED = {
    "HOME": (60.0, 0.0, 180.0),   # x [cm], y [cm], heading [deg]
}


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


# NO host-side distance fudging. Ask for the distance you want and let the
# planner deliver it; the residuals this tool reports are then a HONEST
# measurement of the planner's own accuracy, which is the point. A scale
# table lived here briefly and was deleted (stakeholder, 2026-08-05): it
# masked the behaviour being dialled in, and one bad bucket drove the robot
# to the field edge.


def orange_dots(dc) -> dict:
    """{'NE': (x, y), ...} for the four large orange dots, from the daemon."""
    resp = dc.list_playfields()
    out = {}
    for pf in getattr(resp, "playfields", []):
        blob = json.loads(pf.json_blob)
        for d in blob.get("dots", []):
            if d.get("color") == "orange":
                for short, long in CARDINALS.items():
                    if d.get("cardinal") == long:
                        out[short] = (float(d["x"]), float(d["y"]))
    return out


def read_pose(dc, tries: int = 25):
    """(x_cm, y_cm, yaw_rad) of the robot's centre of rotation, or None."""
    for _ in range(tries):
        tf = dc.get_tags(CAMERA)
        for t in getattr(tf, "tags", []):
            if t.id == ROBOT_TAG and getattr(t, "world_xy", None) is not None:
                return (float(t.world_xy[0]), float(t.world_xy[1]), float(t.yaw))
        time.sleep(0.15)
    return None


class Driver:
    """One relay connection, reused for every command in the run."""

    def __init__(self, port: str):
        from robot_radio.io.serial_conn import SerialConnection
        from robot_radio.robot.protocol import NezhaProtocol
        self._conn = SerialConnection(port=port, mode="relay")
        self._conn.connect()
        self._p = NezhaProtocol(self._conn)
        self._id = int(time.time()) % 800000 + 100000

    def _next_id(self) -> int:
        self._id += 1
        return self._id

    def pivot(self, deg: float) -> None:
        omega = math.copysign(PIVOT_OMEGA, deg * YAW_SIGN)
        self._p.move_twist(0.0, 0.0, omega, stop_angle=math.radians(abs(deg)),
                           timeout=12000.0, move_id=self._next_id())
        time.sleep(abs(deg) / math.degrees(PIVOT_OMEGA) + 2.2)

    def arc(self, arc_cm: float, curvature: float) -> None:
        """curvature [1/cm], positive = left (yaw increasing)."""
        omega = (CRUISE / 10.0) * curvature * YAW_SIGN
        cmd_mm = arc_cm * 10.0          # commanded as asked -- no fudge
        self._p.move_twist(CRUISE, 0.0, omega, stop_distance=cmd_mm,
                           timeout=25000.0, move_id=self._next_id())
        time.sleep(cmd_mm / CRUISE + 2.2)

    def reverse(self, cm: float) -> None:
        """Straight back. Heading is preserved, so no omega and no sign trap."""
        cmd_mm = cm * 10.0
        self._p.move_twist(-CRUISE, 0.0, 0.0, stop_distance=cmd_mm,
                           timeout=20000.0, move_id=self._next_id())
        time.sleep(cmd_mm / CRUISE + 2.2)

    def halt(self) -> None:
        try:
            self._p.estop()
        except Exception:
            pass

    def close(self) -> None:
        self.halt()
        try:
            self._conn.disconnect()
        except Exception:
            pass


def fence_ok(pose) -> bool:
    if pose is None:
        print("  no camera fix -- refusing to move", flush=True)
        return False
    if abs(pose[0]) > FENCE_X or abs(pose[1]) > FENCE_Y:
        print(f"  FENCE: ({pose[0]:.1f},{pose[1]:.1f}) outside limit", flush=True)
        return False
    return True


def goto(dc, drv, tx: float, ty: float, heading_deg=None) -> dict:
    """Drive to (tx, ty) [cm]; returns the measured residual."""
    for p in range(MAX_PASSES):
        pose = read_pose(dc)
        if not fence_ok(pose):
            return {"ok": False, "reason": "fence/no-fix"}
        x, y, yaw = pose
        dx, dy = tx - x, ty - y
        dist = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx)
        berr = wrap(bearing - yaw)

        print(f"  pass {p}: at ({x:6.2f},{y:6.2f}) yaw {math.degrees(yaw):+7.2f}  "
              f"dist {dist:5.2f} cm  bearing err {math.degrees(berr):+7.2f}", flush=True)

        if dist <= POS_TOL:
            break

        if dist <= REVERSE_MAX_CM and abs(math.degrees(berr)) >= REVERSE_MIN_DEG:
            print(f"    reverse {dist:.1f} cm (target is behind)", flush=True)
            drv.reverse(dist)
            continue

        if abs(math.degrees(berr)) > TURN_FIRST_DEG:
            print(f"    pivot {math.degrees(berr):+.1f} deg to face target", flush=True)
            drv.pivot(math.degrees(berr))
            continue

        # one constant-curvature arc through the target, in the body frame
        c, s = math.cos(-yaw), math.sin(-yaw)
        fx = dx * c - dy * s
        fy = dx * s + dy * c
        chord2 = fx * fx + fy * fy
        k = 2.0 * fy / chord2
        if abs(k) > 1e-9:
            dtheta = 2.0 * math.atan2(fy, fx)
            arc_cm = abs(dtheta / k)
            radius = 1.0 / k
        else:
            arc_cm = dist
            radius = float("inf")
        # Cap the leg. Same curvature, shorter commitment, then re-fix -- a
        # long arc cannot be steered once it is in flight.
        leg = min(arc_cm, MAX_LEG_CM)
        capped = " (capped)" if leg < arc_cm else ""
        print(f"    arc {leg:.1f} cm of {arc_cm:.1f}{capped}, "
              f"radius {radius:+.0f} cm", flush=True)
        drv.arc(leg, k)

    # optional final heading
    if heading_deg is not None:
        for _ in range(3):
            pose = read_pose(dc)
            if pose is None:
                break
            herr = math.degrees(wrap(math.radians(heading_deg) - pose[2]))
            if abs(herr) <= HEADING_TOL:
                break
            print(f"    heading pivot {herr:+.1f} deg", flush=True)
            drv.pivot(herr)

    pose = read_pose(dc)
    if pose is None:
        return {"ok": False, "reason": "lost the tag"}
    x, y, yaw = pose
    err = math.hypot(tx - x, ty - y)
    out = {
        "ok": err <= POS_TOL * 2,
        "x": x, "y": y, "heading_deg": math.degrees(yaw),
        "target": (tx, ty),
        "error_cm": err,
        "error_x_cm": x - tx, "error_y_cm": y - ty,
    }
    if heading_deg is not None:
        out["heading_error_deg"] = math.degrees(wrap(math.radians(heading_deg) - yaw))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("target", nargs="+",
                    help="NE/NW/SE/SW, or 'x y' pairs, or a sequence of dots")
    ap.add_argument("--heading", type=float, default=None, help="[deg] final heading")
    ap.add_argument("--port", default=RELAY_PORT)
    args = ap.parse_args()

    from aprilcam.config import Config
    from aprilcam.client.control import DaemonControl

    dc = DaemonControl.connect_default(Config.load())
    dots = orange_dots(dc)

    waypoints = []          # (label, x, y, heading_deg | None)
    toks = list(args.target)
    while toks:
        t = toks.pop(0)
        key = t.upper()
        if key in NAMED:
            nx, ny, nh = NAMED[key]
            waypoints.append((key, nx, ny, args.heading if args.heading is not None else nh))
        elif key in dots:
            waypoints.append((key, *dots[key], args.heading))
        else:
            y = toks.pop(0)
            waypoints.append((f"({t},{y})", float(t), float(y), args.heading))

    print(f"orange dots: {dict(sorted(dots.items()))}", flush=True)
    drv = Driver(args.port)
    results = []
    try:
        for name, tx, ty, hdg in waypoints:
            hs = f", heading {hdg:.0f} deg" if hdg is not None else ""
            print(f"\n=== goto {name} -> ({tx:.1f}, {ty:.1f}) cm{hs} ===", flush=True)
            r = goto(dc, drv, tx, ty, hdg)
            results.append((name, r))
            if not r.get("ok"):
                print(f"  RESULT {name}: {r}", flush=True)
            else:
                msg = (f"  RESULT {name}: at ({r['x']:.2f},{r['y']:.2f}) "
                       f"heading {r['heading_deg']:+.1f}  "
                       f"ERROR {r['error_cm']*10:.1f} mm "
                       f"(dx {r['error_x_cm']*10:+.1f}, dy {r['error_y_cm']*10:+.1f})")
                if "heading_error_deg" in r:
                    msg += f"  heading err {r['heading_error_deg']:+.1f} deg"
                print(msg, flush=True)
    finally:
        drv.close()
        dc.close()

    print("\n--- summary ---", flush=True)
    for name, r in results:
        if r.get("ok"):
            print(f"  {name:>10}: {r['error_cm']*10:6.1f} mm", flush=True)
        else:
            print(f"  {name:>10}: FAILED ({r.get('reason')})", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
