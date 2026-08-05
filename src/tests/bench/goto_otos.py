#!/usr/bin/env python3
"""Drive to world coordinates on the OTOS, re-planning a curve as it closes.

The camera is used ONCE, to seed the world pose. After that the robot
navigates on its own OTOS: read where you are, solve the arc to the target,
send it as a replaceable Move, and re-solve before that arc finishes -- so
motion is continuous and each new solve starts from the velocity the last one
left. The camera is re-read only at the end, to score the result.

Two frames have to be reconciled first, which is the whole reason this needs
saying out loud:

  * targets (the flat tags 12/14/15/16 on the orange dots) sit on the
    playfield plane and the camera reports them TRUE.
  * the robot's own tag 100 sits 117mm above that plane and the camera
    inflates its displacement by 1.1212 (2026-08-05, tape-measured; see
    .claude/rules/ and the tovez.json linear_scale note).

So the robot's camera fix is divided by that factor and the targets are not.
Get this wrong and the robot chases coordinates in a frame it is not in --
which is exactly what made earlier camera-driven tours fight themselves.

    uv run python src/tests/bench/goto_otos.py 300 0
    uv run python src/tests/bench/goto_otos.py NE SE SW NW
"""
from __future__ import annotations

import argparse
import math
import time

RELAY_PORT = "/dev/cu.usbmodem214102"
CAMERA = "arducam-ov9782-usb-camera"
ROBOT_TAG = 100

# Camera overstates the ELEVATED robot tag's radius from the field origin by
# this much. Flat tags are unaffected. Remove once AprilCam corrects for tag
# height -- verified against tape at 989.7/990.0mm on two 990mm legs.
CAM_TAG_INFLATION = 1.1212

CRUISE = 150.0          # [mm/s]
APPROACH = 90.0         # [mm/s] inside SLOW_RADIUS
SLOW_RADIUS = 180.0     # [mm]
PIVOT_OMEGA = 1.4       # [rad/s]
YAW_SIGN = -1.0         # commanded omega is opposite to world CCW (measured)

# Pivot only when the target is genuinely behind us. This robot overshoots
# SMALL turns by 1.5-3x, so correcting a few degrees with a pivot flips the
# error sign and the robot wags between two pivots forever -- a limit cycle,
# not a tuning problem. Small errors are steered out with curvature instead.
TURN_FIRST = math.radians(50.0)
FINE_TURN = math.radians(45.0)
ARRIVE = 25.0           # [mm] terminal tolerance -- must exceed one control step
REPLAN = 0.55           # [s] between re-solves
FINE_RADIUS = 110.0     # [mm] inside this, commit exact moves and wait
MAX_ARC = 260.0         # [mm] never commit further than this before re-solving
TIMEOUT = 45.0          # [s] per waypoint

FENCE_X, FENCE_Y = 600.0, 390.0   # [mm] TRUE coordinates

# The orange dots: exactly +-500mm x, +-300mm y (stakeholder, 2026-08-05).
# Confirmed by laying flat tags 12/14/15/16 on them and reading (+-50, +-30)cm
# back from the camera -- that check was to validate the DOTS, and the dots
# won; the tags themselves are not placed accurately enough to survey from.
DOTS = {"NW": (-500.0, 300.0), "NE": (500.0, 300.0),
        "SW": (-500.0, -300.0), "SE": (500.0, -300.0)}


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class Robot:
    def __init__(self, port: str):
        from robot_radio.io.serial_conn import SerialConnection
        from robot_radio.robot.protocol import NezhaProtocol
        self.conn = SerialConnection(port=port, mode="relay")
        self.conn.connect()
        self.p = NezhaProtocol(self.conn)
        self._id = int(time.time()) % 500000 + 100000
        self.conn.send_cleartext("TLM:ON")
        time.sleep(0.4)
        self.conn.drain_binary_tlm()

    def _next_id(self) -> int:
        self._id += 1
        return self._id

    def pose(self, window: float = 0.12):
        """Newest OTOS pose from telemetry: (x, y, heading). ~20Hz, no polling."""
        time.sleep(window)
        best = None
        for env in self.conn.drain_binary_tlm():
            t = getattr(env, "tlm", None)
            if t is None or not (t.flags & 0x02):   # no OTOS -> no pose
                continue
            if best is None or t.now > best.now:
                best = t
        if best is None:
            return None
        return (float(best.otos.x), float(best.otos.y),
                float(best.otos.heading) * 0.001)

    def pose_blocking(self, timeout: float = 4.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            p = self.pose()
            if p is not None:
                return p
        return None

    def seed(self, x: float, y: float, heading: float) -> bool:
        self.conn.send_cleartext(f"SEED:{x:.1f},{y:.1f},{heading:.5f}",
                                 read_timeout=900)
        time.sleep(0.5)
        got = self.pose_blocking()
        return got is not None and math.hypot(got[0] - x, got[1] - y) < 25.0

    def arc(self, speed: float, omega_world: float, distance: float) -> None:
        """Replaceable curved Move. Preempts whatever is running."""
        self.p.move_twist(speed, 0.0, YAW_SIGN * omega_world,
                          stop_distance=max(distance, 1.0),
                          timeout=20000.0, move_id=self._next_id(), replace=True)

    def pivot(self, error: float) -> None:
        """Replaceable in-place turn. error is CCW-positive [rad]."""
        omega = math.copysign(PIVOT_OMEGA, error)
        self.p.move_twist(0.0, 0.0, YAW_SIGN * omega,
                          stop_angle=abs(error), timeout=12000.0,
                          move_id=self._next_id(), replace=True)

    def halt(self) -> None:
        try:
            self.p.estop()
        except Exception:
            pass

    def close(self) -> None:
        self.halt()
        try:
            self.conn.disconnect()
        except Exception:
            pass


def camera_fixes(dc, tries: int = 12, want_tags: int = 4):
    """(robot_pose_true, {tag_id: (x,y)}) -- robot de-inflated, tags as-is.

    Accumulates across frames. A single frame routinely misses tags, so
    returning on the first robot sighting loses the targets entirely.
    """
    robot, tags = None, {}
    for _ in range(tries):
        tf = dc.get_tags(CAMERA)
        for t in getattr(tf, "tags", []):
            xy = getattr(t, "world_xy", None)
            if xy is None:
                continue
            if t.id == ROBOT_TAG:
                robot = (float(xy[0]) * 10.0 / CAM_TAG_INFLATION,
                         float(xy[1]) * 10.0 / CAM_TAG_INFLATION,
                         float(t.yaw))
            elif t.id != 1:
                tags[t.id] = (float(xy[0]) * 10.0, float(xy[1]) * 10.0)
        if robot is not None and len(tags) >= want_tags:
            break
        time.sleep(0.12)
    return robot, tags


def solve_arc(pose, target):
    """Curvature and arc length from pose to target, in the body frame."""
    dx, dy = target[0] - pose[0], target[1] - pose[1]
    chord = math.hypot(dx, dy)
    bearing = math.atan2(dy, dx)
    error = wrap(bearing - pose[2])

    if chord < 1e-6:
        return 0.0, 0.0, error, chord
    # Circular arc through both points, tangent to the current heading.
    curvature = 2.0 * math.sin(error) / chord
    if abs(curvature) < 1e-9:
        return 0.0, chord, error, chord
    arc = abs(2.0 * error / curvature)
    return curvature, arc, error, chord


def inside_fence(x, y):
    return abs(x) < FENCE_X and abs(y) < FENCE_Y


def goto(bot, dc, target, label):
    started = time.time()
    print(f"  -> {label} ({target[0]:.0f}, {target[1]:.0f})")

    while True:
        if time.time() - started > TIMEOUT:
            bot.halt()
            print(f"     TIMEOUT after {TIMEOUT:.0f}s")
            return False

        pose = bot.pose_blocking()
        if pose is None:
            bot.halt()
            print("     lost OTOS telemetry")
            return False

        curvature, arc, error, chord = solve_arc(pose, target)

        if chord <= ARRIVE:
            bot.halt()
            print(f"     arrived: OTOS says {chord:.1f}mm out, "
                  f"{time.time()-started:.1f}s")
            return True

        if not inside_fence(*pose[:2]):
            bot.halt()
            print(f"     FENCE: OTOS at ({pose[0]:.0f},{pose[1]:.0f})")
            return False

        if chord < FINE_RADIUS:
            # Terminal approach. Continuous re-planning cannot settle inside a
            # tolerance smaller than one re-plan's travel (~50mm), so stop
            # steering and instead commit an EXACT short move, let it finish,
            # and re-measure. Converges instead of orbiting.
            if abs(error) > FINE_TURN:
                bot.pivot(error)
                time.sleep(min(abs(error) / PIVOT_OMEGA + 0.5, 3.0))
                continue
            step = min(arc, chord * 1.8)
            bot.arc(APPROACH, APPROACH * curvature, step)
            time.sleep(step / APPROACH + 1.2)
            continue

        if abs(error) > TURN_FIRST:
            bot.pivot(error)
            time.sleep(min(abs(error) / PIVOT_OMEGA + 0.35, 3.0))
            continue

        speed = APPROACH if chord < SLOW_RADIUS else CRUISE
        # Commit only as far as the next re-solve, so the curve is continuously
        # corrected instead of being flown open-loop to the target.
        step = min(arc, MAX_ARC, max(chord, 20.0))
        bot.arc(speed, speed * curvature, step)
        time.sleep(REPLAN)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("target", nargs="+", help="'x y' pairs, or NE/NW/SE/SW dots")
    ap.add_argument("--port", default=RELAY_PORT)
    args = ap.parse_args()

    from aprilcam.config import Config
    from aprilcam.client.control import DaemonControl
    dc = DaemonControl.connect_default(Config.load())

    start, tags = camera_fixes(dc)
    if start is None:
        print("camera cannot see tag 100 -- are the lights on?")
        return 1

    # The dots are whatever flat tags are on the field, named by quadrant.
    dots = dict(DOTS)
    print(f"dots: { {k: (round(v[0]), round(v[1])) for k, v in sorted(dots.items())} }")

    waypoints = []
    toks = list(args.target)
    while toks:
        t = toks.pop(0)
        if t.upper() in dots:
            waypoints.append((t.upper(), dots[t.upper()]))
        elif toks:
            y = toks.pop(0)
            waypoints.append((f"({t},{y})", (float(t), float(y))))
        else:
            print(f"unknown target {t!r}; known dots: {sorted(dots)}")
            return 1

    bot = Robot(args.port)
    try:
        print(f"seeding OTOS from camera: ({start[0]:.1f}, {start[1]:.1f}) "
              f"{math.degrees(start[2]):.1f}deg  [de-inflated /{CAM_TAG_INFLATION}]")
        if not bot.seed(*start):
            print("seed did not read back -- stopping")
            return 1

        results = []
        for label, target in waypoints:
            if not inside_fence(*target):
                print(f"  -> {label} is outside the fence -- skipping")
                continue
            ok = goto(bot, dc, target, label)
            time.sleep(1.2)
            cam, _ = camera_fixes(dc)
            otos = bot.pose_blocking()
            if cam and otos:
                cam_err = math.dist(cam[:2], target)
                drift = math.dist(cam[:2], otos[:2])
                results.append((label, cam_err, drift))
                print(f"     camera says {cam_err:.1f}mm from target; "
                      f"OTOS-vs-camera {drift:.1f}mm")
            if not ok:
                break

        if results:
            print()
            print(f"{'waypoint':<10} {'camera err':>11} {'otos-cam':>10}")
            for label, err, drift in results:
                print(f"{label:<10} {err:10.1f}mm {drift:9.1f}mm")
            errs = sorted(r[1] for r in results)
            print(f"median camera error: {errs[len(errs)//2]:.1f}mm over "
                  f"{len(results)} waypoints, ONE camera fix at the start")
    finally:
        bot.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
