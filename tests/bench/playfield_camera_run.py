#!/usr/bin/env python3
"""playfield_camera_run.py — camera-ground-truthed playfield test over the relay.

Single process that BOTH drives the robot (robot_radio over the radio relay) AND
watches it on the overhead AprilTag camera (aprilcam DaemonControl). The camera
gives true world position as ground truth AND a hard field-edge geofence — far
safer than the OTOS-displacement geofence, which assumes a centred start.

Must run in an environment that has BOTH packages. The AprilTags project venv
qualifies (it has aprilcam + serial + robot_radio on sys.path):

    /Volumes/Proj/proj/RobotProjects/AprilTags/.venv/bin/python \
        tests/bench/playfield_camera_run.py

Sequence:
  1. Recenter: camera-closed-loop drive the robot to ~origin (face origin via RT,
     forward hop via G, re-check) so the square has clearance.
  2. Turn closure: 4x relative RT +90; camera-measured heading must close to ~0.
  3. Square: 4x [G forward SIDE + RT +90]; record the CAMERA world pose at each
     corner; report the true return error.

Completion is detected by polling SNAP `mode` (V/G/T -> I) — the relay drops async
EVT, so wait_for_evt_done would hang. Every move is camera-geofenced; safe-stop
(X) on any abort/exception.
"""
from __future__ import annotations
import math
import sys
import time

_RR_HOST = "/Volumes/Proj/proj/RobotProjects/radio-robot-elite/host"
if _RR_HOST not in sys.path:
    sys.path.insert(0, _RR_HOST)

from robot_radio.io.serial_conn import SerialConnection
from aprilcam.config import Config
from aprilcam.client.control import DaemonControl

PORT = "/dev/cu.usbmodem2121302"
ROBOT_TAG = 100
# Field is 134.3 x 89.3 cm (origin = AprilTag 1 centre). Abort if the tag centre
# leaves this inset box (robot footprint ~8 cm beyond the tag → stays on table).
X_MAX, Y_MAX = 57.0, 36.0
SIDE_MM = 200          # 20 cm square sides
SPEED = 150            # mm/s

conn = SerialConnection(PORT)
dc = None
cam = None


# --------------------------------------------------------------------------- #
# Robot transport (relay, SNAP-poll completion)                               #
# --------------------------------------------------------------------------- #
def send(cmd, ms=250):
    return conn.send(cmd, read_ms=ms, stop_token="OK").get("responses", [])


def snap_mode():
    for _ in range(6):
        conn.send_fast("SNAP")
        for ln in conn.read_lines(350, stop_token="TLM"):
            if "TLM" in ln:
                for tok in ln.split():
                    if tok.startswith("mode="):
                        return tok.split("=", 1)[1]
        time.sleep(0.04)
    return None


def wait_idle(timeout_ms, min_ms=400):
    t0 = time.time()
    deadline = t0 + timeout_ms / 1000.0
    saw_active = False
    idle = 0
    while time.time() < deadline:
        m = snap_mode()
        if m is not None and m != "I":
            saw_active = True
            idle = 0
        elif m == "I":
            if saw_active or (time.time() - t0) * 1000.0 >= min_ms:
                idle += 1
                if idle >= (2 if saw_active else 3):
                    return True
        time.sleep(0.08)
    return False


def safe_stop():
    for _ in range(3):
        try:
            conn.send_fast("X")
        except Exception:
            pass
        time.sleep(0.04)


def rt(cdeg, timeout_ms=15000):
    send(f"RT {int(cdeg)}")
    return wait_idle(timeout_ms)


def g_fwd(mm, timeout_ms=20000):
    send(f"G {int(mm)} 0 {SPEED}")
    return wait_idle(timeout_ms)


# --------------------------------------------------------------------------- #
# Camera ground truth                                                         #
# --------------------------------------------------------------------------- #
def cam_pose(timeout=1.5):
    """(x_cm, y_cm, yaw_deg) for the robot tag, or None. yaw: 0=+x, CCW+."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        for t in dc.get_tags(cam).tags:
            if t.id == ROBOT_TAG and t.world_xy is not None and t.yaw is not None:
                return (float(t.world_xy[0]), float(t.world_xy[1]),
                        math.degrees(float(t.yaw)))
        time.sleep(0.03)
    return None


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def on_field(x, y):
    return abs(x) <= X_MAX and abs(y) <= Y_MAX


def require_on_field():
    p = cam_pose()
    if p is None:
        print("  !! lost camera fix — safe-stopping"); safe_stop(); raise SystemExit("no fix")
    if not on_field(p[0], p[1]):
        print(f"  !! GEOFENCE: robot at ({p[0]:.0f},{p[1]:.0f}) cm outside "
              f"+-{X_MAX:.0f}/{Y_MAX:.0f} — safe-stopping"); safe_stop(); raise SystemExit("geofence")
    return p


# --------------------------------------------------------------------------- #
# Steps                                                                       #
# --------------------------------------------------------------------------- #
def recenter(tol_cm=8.0, max_iters=10):
    print("\n== RECENTER to origin (camera closed-loop) ==")
    # One-time RT-sign probe (in place, safe): does a commanded RT +30 raise the
    # camera yaw (CCW) or lower it (reflected frame)? Drive all facing turns by
    # this measured sign so a wrong convention can't run the robot at an edge.
    ya = require_on_field()[2]
    rt(3000)
    yb = require_on_field()[2]
    sign = 1.0 if wrap_deg(yb - ya) >= 0 else -1.0
    print(f"  RT-sign probe: cmd +30 deg, camera measured {wrap_deg(yb - ya):+.0f} deg -> sign={sign:+.0f}")
    for i in range(max_iters):
        x, y, yaw = require_on_field()
        d = math.hypot(x, y)
        print(f"  [{i}] at ({x:+.0f},{y:+.0f}) cm  d={d:.0f}  yaw={yaw:+.0f}")
        if d <= tol_cm:
            print(f"  centred ({d:.0f} cm from origin).")
            return True
        bearing = math.degrees(math.atan2(-y, -x))     # heading toward origin
        dyaw = wrap_deg(bearing - yaw)
        rt(int(round(sign * dyaw * 100)))
        require_on_field()
        p = cam_pose()
        hop = min(10.0, math.hypot(p[0], p[1]))         # cm — small, edge-safe
        g_fwd(int(round(hop * 10)))
        require_on_field()
    x, y, _ = require_on_field()
    print(f"  recenter incomplete: {math.hypot(x,y):.0f} cm from origin after {max_iters} iters")
    return math.hypot(x, y) <= tol_cm * 2


def turn_closure():
    print("\n== TURN CLOSURE: 4x RT +90 (camera-measured) ==")
    p0 = require_on_field()
    yaw0 = p0[2]
    print(f"  start yaw={yaw0:+.0f}")
    for k in range(4):
        rt(9000)
        p = require_on_field()
        print(f"  after RT {k+1}/4: yaw={p[2]:+.0f}  at ({p[0]:+.0f},{p[1]:+.0f})")
    yaw1 = cam_pose()[2]
    err = abs(wrap_deg(yaw1 - yaw0))
    print(f"  closure error = {err:.1f} deg  (start {yaw0:+.0f} -> end {yaw1:+.0f})")
    return err


def square():
    print(f"\n== SQUARE: {SIDE_MM} mm, G forward + RT +90, camera-measured ==")
    start = require_on_field()
    print(f"  start corner: ({start[0]:+.0f},{start[1]:+.0f}) cm")
    corners = [(start[0], start[1])]
    for i in range(4):
        g_fwd(SIDE_MM)
        p = require_on_field()
        corners.append((p[0], p[1]))
        print(f"  leg {i+1}/4 end: ({p[0]:+.0f},{p[1]:+.0f}) cm  yaw={p[2]:+.0f}")
        if i < 3:
            rt(9000)
            require_on_field()
    end = corners[-1]
    ret = math.hypot(end[0] - start[0], end[1] - start[1])
    side_lens = [math.hypot(corners[j+1][0]-corners[j][0], corners[j+1][1]-corners[j][1])
                 for j in range(4)]
    print(f"  corners (cm): {['(%+.0f,%+.0f)' % c for c in corners]}")
    print(f"  side lengths (cm): {['%.1f' % s for s in side_lens]}  (commanded {SIDE_MM/10:.0f})")
    print(f"  RETURN ERROR (camera) = {ret:.1f} cm")
    return ret


def main():
    global dc, cam
    res = conn.connect()
    if res.get("error"):
        sys.exit(f"robot connect failed: {res['error']}")
    print(f"robot connected (mode={conn.mode})")
    dc = DaemonControl.connect_default(Config.load())
    cam = dc.list_cameras()[0]
    print(f"camera: {cam}")
    png = conn.send("PING", read_ms=600, stop_token="OK").get("responses")
    print(f"PING -> {png}")

    t0 = time.time()
    try:
        safe_stop()
        recenter()
        clo = turn_closure()
        ret = square()
        print("\n==== PLAYFIELD RESULT (camera ground truth) ====")
        print(f"  turn closure : {clo:.1f} deg")
        print(f"  square return: {ret:.1f} cm")
        print(f"  elapsed      : {time.time()-t0:.0f} s")
    finally:
        print("\n[safe-stop] X + disconnect")
        safe_stop()
        try:
            conn.disconnect()
        except Exception:
            pass
        try:
            if dc is not None:
                dc.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
