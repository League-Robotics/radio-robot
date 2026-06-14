#!/usr/bin/env python
"""Camera closed-loop driver — the robot's onboard turning/odometry can't be
trusted (turns the wrong way, overshoots, pose drifts ~50cm), so we DON'T use the
firmware goto/TURN/SI at all. Instead the overhead camera is the only authority:

  loop:
    read camera pose (averaged)            <- CHECK
    if out of safe bounds -> X, abort      <- never run into the boards
    if arrived -> done
    compute bearing to target              <- CALCULATE
    if heading off -> ONE small in-place turn pulse toward it
    else            -> ONE small forward pulse
    (re-check the camera at the top of the loop)

Motion is raw timed wheel pulses (T l r ms) — reliable, no firmware heading
logic. The turn direction is auto-detected from the camera on the first turn,
so there are no hand-derived sign assumptions. Every pulse is small and bounds-
checked, so a wrong move is caught at the next camera read, well short of the edge.

  uv run --group calibrate python host_tests/playfield_tour/camera_drive.py
"""
import json
import math
import os
import random
import time

import serial
from aprilcam.config import Config
from aprilcam.client.control import DaemonControl

RELAY = "/dev/cu.usbmodem2121402"
ROBOT_TAG = 100
PLAYFIELD = "/Volumes/Proj/proj/RobotProjects/AprilTags/data/aprilcam/playfield.json"
WHEEL = 110              # mm/s pulse speed
TURN_MS = (120, 420)     # min/max turn pulse
FWD_STEP_CM = 6.0        # max forward pulse distance (small => low overshoot)
ARRIVE_CM = 6.0
HEAD_TOL_DEG = 12.0      # within this bearing error, drive forward instead of turning
# Hard safety bounds (cm) — robot CENTER must stay inside. Field edges are
# +-50.5 x / +-44.5 y; the robot body extends ~7cm, so these margins keep the
# whole robot clear of the boards even allowing for a pulse of overshoot.
ABORT_X = 38.0
ABORT_Y = 31.0

dc = DaemonControl.connect_default(Config.load())
cam = dc.list_cameras()[0]
p = serial.Serial(RELAY, 115200, timeout=0.3)
time.sleep(1.6); p.reset_input_buffer()

# Target slots: the defined colored rectangles (all well inside the field).
_pf = json.load(open(PLAYFIELD))
SLOTS = [(s["slug"], float(s["x"]), float(s["y"])) for s in _pf["rectangles"]]

# Persistent path file the live view reads (under the daemon's data dir).
PATHS_JSON = os.path.join(os.path.dirname(PLAYFIELD), "cameras",
                          "arducam-ov9782-usb-camera", "paths.json")

_turn_sign = None        # +1 if T(-WHEEL,+WHEEL) increases camera yaw (CCW)
TRACK = []               # accumulated camera (x,y) for the live overlay


def s(c):
    p.write((c + "\n").encode()); p.flush()


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def campose(n=5):
    """Averaged (x_cm, y_cm, yaw_rad) for the robot tag, or None."""
    xs, ys, ss, cc = [], [], [], []
    for _ in range(n * 3):
        tf = dc.get_tags(cam)
        t = next((t for t in tf.tags if t.id == ROBOT_TAG and t.world_xy), None)
        if t:
            xs.append(t.world_xy[0]); ys.append(t.world_xy[1])
            ss.append(math.sin(t.yaw)); cc.append(math.cos(t.yaw))
        if len(xs) >= n:
            break
        time.sleep(0.04)
    if not xs:
        return None
    return (sum(xs) / len(xs), sum(ys) / len(ys),
            math.atan2(sum(ss) / len(ss), sum(cc) / len(cc)))


def in_bounds(x, y):
    return abs(x) <= ABORT_X and abs(y) <= ABORT_Y


def write_paths(payload):
    """Atomically write the paths.json the live view reads (persists across runs)."""
    tmp = PATHS_JSON + ".tmp"
    with open(tmp, "w") as f:
        f.write(json.dumps(payload))
    os.replace(tmp, PATHS_JSON)


def draw_track():
    """Persist the accumulated camera path so the live view shows it (and it stays).

    Small light crosses at each sample, with a thin connecting line."""
    if len(TRACK) >= 2:
        wp = [{"x": x, "y": y, "size_cm": 0.8, "symbol": "x",
               "symbol_color": [255, 235, 130], "line_color": [255, 235, 130]}
              for x, y in TRACK]
        write_paths([{"path_id": "tour", "playfield_id": cam, "waypoints": wp}])


def pulse(l, r, ms):
    """One timed wheel pulse, keepalive-fed; stop after it completes."""
    s(f"T {l} {r} {ms}")
    t0 = time.time()
    while time.time() - t0 < ms / 1000.0 + 0.2:
        p.write(b"+\n"); p.flush(); time.sleep(0.05)
    s("X"); time.sleep(0.12); p.read(8192)


def turn_pulse(want_ccw, ms):
    """In-place turn pulse toward CCW/CW, using the auto-detected wheel sign."""
    global _turn_sign
    if _turn_sign is None:
        # Probe: spin T(-,+) briefly, see which way the camera yaw moves.
        a = campose()
        pulse(-WHEEL, WHEEL, 250)
        b = campose()
        _turn_sign = 1 if (a and b and wrap(b[2] - a[2]) > 0) else -1
        print(f"  [turn-sign] T(-,+) -> {'CCW' if _turn_sign > 0 else 'CW'}")
        return
    # T(-WHEEL,+WHEEL) gives CCW iff _turn_sign>0.
    ccw_cmd = (_turn_sign > 0)
    if want_ccw == ccw_cmd:
        pulse(-WHEEL, WHEEL, ms)
    else:
        pulse(WHEEL, -WHEEL, ms)


def goto(tx, ty, max_iters=40):
    """Drive to (tx,ty) cm in a bounds-checked camera loop. Returns True if arrived."""
    print(f"--> goto ({tx:+.0f},{ty:+.0f})")
    for _ in range(max_iters):
        pp = campose()
        if pp is None:
            s("X"); print("  lost tag — STOP"); return False
        x, y, yaw = pp
        TRACK.append((x, y)); draw_track()
        if not in_bounds(x, y):
            s("X"); print(f"  OUT OF BOUNDS ({x:+.0f},{y:+.0f}) — STOP"); return False
        d = math.hypot(tx - x, ty - y)
        herr = wrap(math.atan2(ty - y, tx - x) - yaw)
        print(f"  at ({x:+5.1f},{y:+5.1f}) yaw={math.degrees(yaw):+4.0f}  "
              f"d={d:4.1f}  herr={math.degrees(herr):+4.0f}")
        if d <= ARRIVE_CM:
            print("  ARRIVED"); return True
        if abs(herr) > math.radians(HEAD_TOL_DEG):
            ms = int(min(TURN_MS[1], max(TURN_MS[0], abs(math.degrees(herr)) / 60.0 * 1000)))
            turn_pulse(herr > 0, ms)
        else:
            step = min(FWD_STEP_CM, d)
            # Predict where this pulse ends; refuse it if that leaves the safe box.
            ex, ey = x + step * math.cos(yaw), y + step * math.sin(yaw)
            if not in_bounds(ex, ey):
                s("X"); print(f"  next step exits bounds ({ex:+.0f},{ey:+.0f}) — STOP"); return False
            pulse(WHEEL, WHEEL, int(min(500, max(150, step * 10 / WHEEL * 1000))))
    print("  gave up (max iters)"); return False


def tour(hops=5):
    """Visit `hops` rectangles, each a random pick of the 5 farthest from the robot."""
    for i in range(hops):
        pp = campose()
        if pp is None:
            print("lost tag"); return
        ranked = sorted(SLOTS, key=lambda sl: math.hypot(sl[1] - pp[0], sl[2] - pp[1]),
                        reverse=True)
        slug, tx, ty = random.choice(ranked[:5])
        print(f"=== hop {i + 1}/{hops}: {slug} ({tx:+.0f},{ty:+.0f}) ===")
        goto(tx, ty)


if __name__ == "__main__":
    try:
        write_paths([])         # clear any stale path from a previous run
        for c in ("!GO", "X", "STOP", "SET sTimeout=60000"):
            s(c); time.sleep(0.35); p.read(8192)
        tour(6)
    finally:
        s("X"); p.close(); dc.close()
