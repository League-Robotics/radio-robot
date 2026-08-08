#!/usr/bin/env python
"""Drive the playfield and build a reflectance + colour map of it.

Rasters the field, lever-arms every sensor reading out to its own world
position, and accumulates readings into a per-cm grid. Saves the raw grids
(so a run is never lost to a rendering bug) and renders a map to compare
against the real field.

    uv run python src/tests/bench/field_map.py --port /dev/cu.usbmodemRELAY --relay
    uv run python src/tests/bench/field_map.py --replay out/field_map.npz   # render only

WHY THE GRID HOLDS SAMPLES INSTEAD OF A RUNNING MEAN
Each cell keeps up to N individual readings and is reduced with a MEDIAN at
render time. A running mean cannot be un-poisoned: one pass with a bad pose
fix drags a cell forever, and on this field a single wrong reading is
exactly what a mis-seeded leg produces. A median over ~10 samples shrugs it
off. Cell layout is [count, s0..s9] per the stakeholder's own design.

COORDINATES
World frame is the camera's: A1-centred (tag 1 = origin), centimetres,
+x east, +y north, field 134.3 x 89.3 cm, so limits are +/-67.15 / +/-44.65.
Grid cell (row, col) is 1 cm, with col 0 at x = -67 and row 0 at y = -44.

Sensor mount geometry comes from the robot JSON's `perception` block --
NOT from literals here. Body frame is x forward, y left-positive, origin at
the centre of rotation (the point tag 100 is registered to), so the lever
arm for a sensor at body (bx, by) with the robot at (x, y, heading) is

    world_x = x + bx*cos(h) - by*sin(h)
    world_y = y + bx*sin(h) + by*cos(h)

KNOWN UNVERIFIED INPUT
`perception.line_array.channel_y` maps telemetry channel index to lateral
position and has never been measured. If the array is wired the other way
the map comes out MIRRORED about the centre line and still looks plausible.
See that field's own note in the robot JSON.

SAFETY
The geofence is checked INSIDE each leg at ~10 Hz, not between legs, where
it could only narrate a crash that already happened. Halting is estop() --
never stop(), which is a planned stop that queues behind the in-flight move
(measured: 39.8 cm of travel vs 2.9 cm). estop() is issued repeatedly and
confirmed, because a single estop() that is lost is indistinguishable from
one that worked (.claude/rules/playfield-testing.md).
"""
from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys
import time

import numpy as np

sys.path.insert(0, "src/tests/bench")

from robot_radio.io.serial_conn import SerialConnection  # noqa: E402
from robot_radio.robot.protocol import NezhaProtocol, TLMFrame  # noqa: E402

REPO = pathlib.Path(__file__).resolve().parents[3]
OUT = pathlib.Path(__file__).resolve().parent / "output"

# Field, from the camera's own calibration (.claude/rules/playfield-testing.md).
FIELD_W = 134.3  # [cm]
FIELD_H = 89.3   # [cm]
X_LIM = FIELD_W / 2.0
Y_LIM = FIELD_H / 2.0

GRID_W = 134  # [cells] 1 cm each
GRID_H = 90
MAX_SAMPLES = 10

# Keep the WHOLE robot inside the rails, not just its centre. The fence is on
# the centre of rotation, so it must be inset by the robot's own reach plus a
# margin -- the line array sits 96 mm ahead of that centre.
FENCE_MARGIN = 14.0  # [cm]

# HARD limit, enforced on EVERY drive -- including recovery. Distinct from the
# raster fence on purpose, and the distinction is load-bearing.
#
# The raster fence is tight, sized to the scan. Recovery legitimately STARTS
# outside it (that is what makes it recovery), so enforcing the raster fence
# there would deadlock a lost robot. The consequence of having no fence at all
# on those paths was measured on 2026-08-08: three excursions, two of them
# into the rails, every one on an UNFENCED path -- the drive-to-centre and the
# calibration drives. The fence was protecting the part of the run that was
# going well and absent from the part that was going badly.
#
# So: recovery is fenced too, just at the rails instead of at the scan box.
# The fence tests the CENTRE OF ROTATION, but the robot is not a point. Its
# reach must be subtracted from the rail, or the fence passes while the front
# of the machine is already against the wall -- which is exactly what happened
# on 2026-08-08: every excursion sat INSIDE a 4cm-inset fence and still hit the
# rails, because at a reported y=+40.4 the line array was at ~+50 and the rail
# is at 44.65.
ROBOT_REACH = 9.6   # [cm] line array ahead of the centre of rotation
# Measured on tovez 2026-08-08: a fence trip at x=+52.5 came to rest at
# x=+56.8, i.e. 4.3cm of travel between the breach being SEEN and the robot
# stopping -- half again the 2.9cm quoted in .claude/rules/playfield-testing.md
# for a bare estop. The difference is the detection leg: that 2.9cm figure is
# stopping distance from the moment estop lands, while this budget also has to
# cover the telemetry frame that carried the breach and the command going back
# out over the relay. Budget what was actually observed, not the ideal.
STOP_TRAVEL = 4.5   # [cm]
RAIL_MARGIN = 2.5   # [cm]

# The Navigator overshoots the end of a move before settling, so a leg endpoint
# is NOT where the robot stops. Measured: 52.5 reached on a 46.0 target here,
# and this file already recorded 54 on a 48 target. A scan whose x_span leaves
# less than this between the leg end and the rail fence will trip on a transit
# no matter how well the legs themselves are driven -- which is what ended the
# 2026-08-08 run at leg 5. Used to CLAMP the requested span rather than leaving
# it to whoever picks the flags.
NAV_OVERSHOOT = 7.0  # [cm]
RAIL_INSET = ROBOT_REACH + STOP_TRAVEL + RAIL_MARGIN  # [cm]
RAIL_FENCE = (X_LIM - RAIL_INSET, Y_LIM - RAIL_INSET)  # [cm]

# Fence used by the edge sweep. Deliberately looser than RAIL_FENCE, on
# stakeholder direction 2026-08-08 ("you don't need a huge 10cm inset around
# the rail... you just need to not run off the playfield").
#
# The reason the tighter fence was wrong here: RAIL_FENCE insets the rail by
# the robot's forward REACH (9.6cm, the line array ahead of the centre of
# rotation) equally in x and y. During this sweep the robot faces WEST for
# every pass, so its y-extent is its half-WIDTH, not its reach -- the reach
# points along x, where it is correctly budgeted. Applying a forward-reach
# inset sideways cost ~4cm of usable band at each end and put the mat's north
# and south borders out of reach for no physical reason.
#
# The x half is set by stopping arithmetic, not by taste. A breach is only
# SEEN at the fence and the robot then coasts, so what has to fit inside the
# rail is fence_x + STOP_TRAVEL + ROBOT_REACH: 52.0 + 4.5 + 9.6 = 66.1
# against X_LIM 67.15, about 1cm of slack. Anything looser puts the line
# array through the east rail on a trip.
#
# The y half is deliberately looser than RAIL_FENCE's 28.05, and that is the
# whole point of having a second fence: rows run to |y| = 30, which the
# isotropic inset forbade for no physical reason. At 34.0 the centre of
# rotation still stays >10cm clear of the rails at Y_LIM 44.65.
SWEEP_FENCE = (52.0, 34.0)  # [cm] |x|, |y| limits on the centre of rotation

# How far a "straight" leg may wander off its own y before the leg is aborted.
# This is the guard that catches an excursion while there is still room to
# stop; the geofence only speaks once the robot is already at the field edge.
# One raster pitch is the natural size: wander further than that and the leg
# is sampling a row it does not belong to, so the data is wrong even when the
# robot is safe.
CROSS_TRACK_LIMIT = 6.0  # [cm]

# How much encoder movement still counts as "stopped" when confirming a halt.
# Encoder position is [mm]; a stationary robot jitters by well under this,
# while a robot still rolling at even 50mm/s moves ~12mm between the two
# reads halt_verified takes 0.25s apart.
HALT_ENCODER_SLACK = 3  # [mm]

# A squaring pivot is a pure rotation, but the pose it reports is NOT taken at
# the centre of rotation: data/robots/tovez.json puts the OTOS chip at
# offset (-47.7, +3.5)mm, i.e. r = 4.78cm off centre. Rotating by theta sweeps
# that chip around a chord of 2*r*sin(theta/2) -- up to 9.56cm for a 180deg
# turn -- with the robot's centre never moving at all. The sensor orbits; the
# robot does not translate.
#
# So this limit must clear a full 180deg sweep or it fires on correct pivots.
# Measured 2026-08-08 with an 8cm limit: a ~115deg correction reported exactly
# 8.1cm of "drift" (2*4.78*sin(57.5deg) = 8.06) and halted the run twice.
# 14cm clears the 9.56cm worst case with margin while still catching what this
# guard is actually for -- a "pivot" that is driving across the field, which
# shows up as tens of cm.
OTOS_LEVER = 4.78          # [cm] hypot(47.7, 3.5)mm, from the robot JSON
PIVOT_DRIFT_LIMIT = 14.0   # [cm] > 2*OTOS_LEVER (9.56) + slop


def within(fence, x, y):
    return abs(x) <= fence[0] and abs(y) <= fence[1]


def load_perception(robot_json: pathlib.Path) -> dict:
    cfg = json.loads(robot_json.read_text())
    p = cfg.get("perception")
    if not p:
        raise SystemExit(f"{robot_json} has no `perception` block -- add the sensor mount geometry")
    return p


def lever_arm(x, y, heading, bx, by):  # [cm] [cm] [rad] [mm] [mm] -> [cm] [cm]
    """Project a body-frame sensor offset out to world coordinates."""
    ch, sh = math.cos(heading), math.sin(heading)
    return (x + (bx * ch - by * sh) / 10.0,
            y + (bx * sh + by * ch) / 10.0)


def to_cell(x, y):  # [cm] [cm] -> (row, col) or None
    col = int(math.floor(x + X_LIM))
    row = int(math.floor(y + Y_LIM))
    if 0 <= row < GRID_H and 0 <= col < GRID_W:
        return row, col
    return None


class FieldMap:
    """[count, s0..s9] per cell, per the stakeholder's own layout."""

    def __init__(self) -> None:
        self.refl = np.zeros((GRID_H, GRID_W, MAX_SAMPLES + 1), dtype=np.uint8)
        # Colour keeps all four channels per sample.
        self.color = np.zeros((GRID_H, GRID_W, MAX_SAMPLES + 1, 4), dtype=np.uint8)

    def add_refl(self, row, col, value) -> None:
        n = int(self.refl[row, col, 0])
        if n < MAX_SAMPLES:
            self.refl[row, col, 1 + n] = value
            self.refl[row, col, 0] = n + 1

    def add_color(self, row, col, rgbc) -> None:
        n = int(self.color[row, col, 0, 0])
        if n < MAX_SAMPLES:
            self.color[row, col, 1 + n] = rgbc
            self.color[row, col, 0, 0] = n + 1

    @property
    def refl_cells(self) -> int:
        return int((self.refl[:, :, 0] > 0).sum())

    @property
    def color_cells(self) -> int:
        return int((self.color[:, :, 0, 0] > 0).sum())

    def save(self, path: pathlib.Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(path, refl=self.refl, color=self.color)

    @classmethod
    def load(cls, path: pathlib.Path) -> "FieldMap":
        d = np.load(path)
        m = cls()
        m.refl, m.color = d["refl"], d["color"]
        return m

    def reduce_refl(self) -> np.ma.MaskedArray:
        """Median of each cell's samples; masked where nothing was seen."""
        out = np.zeros((GRID_H, GRID_W), dtype=float)
        seen = self.refl[:, :, 0] > 0
        for r, c in zip(*np.nonzero(seen)):
            n = int(self.refl[r, c, 0])
            out[r, c] = float(np.median(self.refl[r, c, 1:1 + n]))
        return np.ma.masked_where(~seen, out)

    def reduce_color(self) -> tuple[np.ndarray, np.ndarray]:
        """Median r,g,b per cell (0-255 floats) and the seen mask."""
        out = np.zeros((GRID_H, GRID_W, 3), dtype=float)
        seen = self.color[:, :, 0, 0] > 0
        for r, c in zip(*np.nonzero(seen)):
            n = int(self.color[r, c, 0, 0])
            out[r, c] = np.median(self.color[r, c, 1:1 + n, :3], axis=0)
        return out, seen


# --------------------------------------------------------------------------
# Camera
# --------------------------------------------------------------------------

def camera_fix(cam, samples=7, settle=0.35):
    """Median-of-N camera fix of tag 100, taken AT REST.

    Median, not mean: a single frame that loses the tag or catches it mid-
    blur is a large outlier, and the mean carries it into the seed.
    """
    xs, ys, hs = [], [], []
    for _ in range(samples):
        tag = cam()
        if tag is not None:
            xs.append(tag[0])
            ys.append(tag[1])
            hs.append(tag[2])
        time.sleep(settle / samples)
    if len(xs) < max(3, samples // 2):
        return None
    # Circular median for heading -- a plain median across the +/-pi wrap
    # would land halfway around the circle.
    hx = float(np.median([math.cos(h) for h in hs]))
    hy = float(np.median([math.sin(h) for h in hs]))
    return float(np.median(xs)), float(np.median(ys)), math.atan2(hy, hx)


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


# --------------------------------------------------------------------------
# Camera access -- DaemonControl, not MCP. A median-of-7 fix is 7 round trips
# and MCP is far too slow to take one every metre of a raster.
# --------------------------------------------------------------------------

CAMERA = "arducam-ov9782-usb-camera"
ROBOT_TAG = 100

# The orange dots, surveyed: exactly +/-500mm x, +/-300mm y (stakeholder
# 2026-08-05, confirmed by laying flat tags on them). Ground truth both for
# the tag-height calibration below and for scoring the finished map.
DOTS = {"NW": (-50.0, 30.0), "NE": (50.0, 30.0),
        "SW": (-50.0, -30.0), "SE": (50.0, -30.0)}  # [cm]

# --------------------------------------------------------------------------
# Ground truth for the KIPR Botball mat, surveyed from the overhead camera
# 2026-08-08: feature corners read off a captured frame and pushed through
# AprilCam's calibrated homography (`pixel_to_world`), NOT paced off by hand.
#
# This exists so the finished map can be COMPARED to something rather than
# admired. Without it, a reflectance plot is a grey blob that everyone agrees
# looks vaguely right; with it, "the sensor found the Start-box border at
# x=+36" is a checkable claim.
#
# Note the mat very nearly fills the field: x -61..+63, y -34..+32 against a
# field of +/-67.15 x +/-44.65. There is no wide apron of bare wood to scan
# around, which is why an off-pattern leg reaches a rail so quickly.
MAT_EXTENT = {"x": (-61.3, 63.1), "y": (-33.8, 32.1)}  # [cm]

# Each entry: (label, [(x, y), ...] closed polygon in cm).
MAT_TRUTH = [
    ("blue diamond", [(-59.6, 13.0), (-44.0, 28.4), (-28.8, 12.5), (-44.4, -2.8)]),
    ("green square", [(-14.7, 13.1), (9.7, 12.7), (9.1, -12.9), (-15.1, -12.3)]),
    ("yellow square", [(-37.4, -7.9), (-17.4, -8.4), (-17.7, -28.0), (-37.6, -27.4)]),
    # The Start box's left border: one long black line, the single strongest
    # and most unambiguous feature on the mat, and the natural first thing to
    # check a scan against.
    ("start border", [(36.9, 30.5), (35.4, -29.6)]),
]


def connect_camera():
    from aprilcam.client.control import DaemonControl
    from aprilcam.config import Config
    return DaemonControl.connect_default(Config.load())


def read_tag(dc, tag_id=ROBOT_TAG, inflation=1.0):
    """One camera read of a tag -> (x_cm, y_cm, yaw_rad), or None."""
    tf = dc.get_tags(CAMERA)
    for t in getattr(tf, "tags", []):
        if t.id != tag_id:
            continue
        xy = getattr(t, "world_xy", None)
        if xy is None:
            return None
        return (float(xy[0]) / inflation, float(xy[1]) / inflation, float(t.yaw))
    return None


def calibrate_tag_height(dc, samples=15):
    """Decide whether the elevated robot tag still needs de-inflation.

    Tag 100 rides 12 cm up, so the camera projects it OUTWARD from the field
    centre -- historically by 12.12% of its radius, corrected host-side by
    dividing (goto_otos.py's CAM_TAG_INFLATION). AprilCam now claims to apply
    parallax correction itself from the registered per-tag height, and
    applying BOTH would over-correct by the same 12%.

    Rather than trust either claim, measure: park the robot on a surveyed
    orange dot and compare the camera's radius to the dot's true radius. This
    must be re-run whenever the tag mount height changes.

    Returns the inflation factor to divide by (1.0 == the daemon already
    handles it).
    """
    fix = camera_fix(lambda: read_tag(dc), samples=samples)
    if fix is None:
        return None, "camera cannot see tag 100"
    x, y, _ = fix
    r_meas = math.hypot(x, y)
    # Nearest surveyed dot is the assumed truth.
    name, (dx, dy) = min(DOTS.items(), key=lambda kv: math.hypot(x - kv[1][0], y - kv[1][1]))
    r_true = math.hypot(dx, dy)
    if r_true < 1.0:
        return None, "robot is at the origin -- park it on a dot to calibrate"
    factor = r_meas / r_true
    detail = (f"nearest dot {name} true=({dx:+.1f},{dy:+.1f})cm r={r_true:.1f}cm | "
              f"camera=({x:+.1f},{y:+.1f})cm r={r_meas:.1f}cm | ratio={factor:.4f}")
    return factor, detail


# --------------------------------------------------------------------------
# Collection
# --------------------------------------------------------------------------

def accumulate(frame: TLMFrame, pose, perception, fmap: FieldMap, channel_y) -> int:
    """Lever-arm one telemetry frame's sensors into the map. Returns cells hit."""
    x, y, heading = pose
    hits = 0

    if frame.line_present and frame.line:
        lx = perception["line_array"]["x"]
        for ch, value in enumerate(frame.line):
            if ch >= len(channel_y):
                break
            wx, wy = lever_arm(x, y, heading, lx, channel_y[ch])
            cell = to_cell(wx, wy)
            if cell:
                fmap.add_refl(cell[0], cell[1], value)
                hits += 1

    if frame.color_present and frame.color:
        cs = perception["color_sensor"]
        wx, wy = lever_arm(x, y, heading, cs["x"], cs["y"])
        cell = to_cell(wx, wy)
        if cell:
            fmap.add_color(cell[0], cell[1], list(frame.color))
            hits += 1

    return hits


# --------------------------------------------------------------------------
# Parallax calibration
# --------------------------------------------------------------------------

# Ground truth, from the AprilCam playfield map (`get_playfield`). Positions
# in cm, A1-centred. These are surveyed, not guessed, so they are what the
# camera and the finished map are both scored against.
RECTS = {  # 5 x 4 cm colour patches
    "purple":  (-35.0,  24.0),
    "black":   (  0.0,  24.0),
    "orange":  ( 35.0,  24.0),
    "green":   ( 35.0, -24.0),
    "magenta": (  0.0, -24.0),
    "blue":    (-35.0, -24.0),
}
SMALL_DOTS = {"green": (0.0, 30.0), "yellow": (0.0, -30.0),
              "red": (50.0, 0.0), "blue": (-50.0, 0.0)}


def solve_parallax(observations):
    """Least-squares fit of  measured = c + k * (true - c).

    Tag 100 rides 12cm above the field, so the camera projects it OUTWARD
    from the optical axis. For a pinhole camera looking straight down that
    is a pure radial scaling k = H/(H-h) about the axis -- but ONLY about
    the axis. Assuming the axis sits exactly over the field origin, as a
    single global divisor does, turns any real camera offset into a
    direction-dependent position error that warps the map rather than
    shifting it.

    So solve for the axis (cx, cy) AND the scale k together. Four surveyed
    dots give 8 equations for 3 unknowns. Linear in (k, cx*(1-k), cy*(1-k)):

        mx = k*tx + cx*(1-k)   ->   mx = k*tx + ax
        my = k*ty + cy*(1-k)   ->   my = k*ty + ay

    Returns (k, cx, cy, residual_cm).
    """
    A, b = [], []
    for (tx, ty), (mx, my) in observations:
        A.append([tx, 1.0, 0.0]); b.append(mx)
        A.append([ty, 0.0, 1.0]); b.append(my)
    sol, *_ = np.linalg.lstsq(np.array(A), np.array(b), rcond=None)
    k, ax, ay = float(sol[0]), float(sol[1]), float(sol[2])
    if abs(1.0 - k) < 1e-9:
        cx = cy = 0.0
    else:
        cx, cy = ax / (1.0 - k), ay / (1.0 - k)
    resid = []
    for (tx, ty), (mx, my) in observations:
        px, py = k * tx + ax, k * ty + ay
        resid.append(math.hypot(px - mx, py - my))
    return k, cx, cy, float(np.mean(resid))


def undo_parallax(x, y, k, cx, cy):
    """Map a camera reading of the ELEVATED tag back to the field plane."""
    return (cx + (x - cx) / k, cy + (y - cy) / k)


# --------------------------------------------------------------------------
# Driving
# --------------------------------------------------------------------------

def halt_verified(bot, tries: int = 4) -> bool:
    """estop() until the robot is OBSERVED stopped, not just commanded to be.

    goto_otos.Robot.halt() fires estop() once and returns. Measured on vevov
    2026-08-03: a stop issued ONCE by a host that then went quiet produced
    936mm of continued travel with no decay, and estop() failed 5 of 6
    attempts -- the Nezha brick latches its last commanded speed, so a lost
    zero write is permanent.

    Confirmed on the ENCODERS, not on kFlagActive. The flag drops for a cycle
    whenever Motion::Navigator re-issues with replace=true, so "flag is clear"
    can be read off a robot that is still rolling -- a false POSITIVE on the
    one check whose whole job is to catch a halt that did not take. Encoder
    position standing still across two reads is the physical fact; the flag is
    an opinion about ownership.
    """
    def positions(window=0.6):
        """Newest encoder pair seen within `window`. Polls rather than taking
        one drain: a single drain right after an estop often comes back empty,
        which is absence of evidence, not evidence the robot is moving -- and
        reading it as a failed halt made this function cry wolf."""
        deadline = time.time() + window
        last = None
        while time.time() < deadline:
            for env in bot.conn.drain_binary_tlm():
                t = getattr(env, "tlm", None)
                if t is None:
                    continue
                f = TLMFrame.from_pb2(t)
                if f.enc is not None:
                    last = f.enc
            if last is not None:
                return last
            time.sleep(0.02)
        return last

    for _ in range(tries):
        try:
            bot.p.estop()
        except Exception:
            pass
        first = positions()
        time.sleep(0.3)
        second = positions()
        # Both reads must land AND agree to within a wheel's worth of noise.
        if (first is not None and second is not None
                and abs(second[0] - first[0]) <= HALT_ENCODER_SLACK
                and abs(second[1] - first[1]) <= HALT_ENCODER_SLACK):
            return True
    return False


def raster(x_span, y_span, pitch):  # [cm] [cm] [cm]
    """Boustrophedon waypoints: long legs along x, stepping in y.

    Legs run along the LONG axis so the robot spends most of its time driving
    straight, where pose is best; every turn is a chance to accumulate heading
    error, so fewer of them is a better map.
    """
    pts = []
    y = -y_span
    left = True
    while y <= y_span + 1e-6:
        pts.append((-x_span if left else x_span, y))
        pts.append((x_span if left else -x_span, y))
        left = not left
        y += pitch
    return pts


def collect_during(bot, fmap, perception, channel_y, seconds, fence):
    """Drain telemetry into the map, enforcing the fence INSIDE the move.

    Sensor values and pose are taken from the SAME frame. Reading pose
    separately would lever-arm a sensor sample through a pose from a
    different instant -- at 300mm/s that is centimetres of smear, which is
    the whole resolution of the map.
    """
    t0 = time.time()
    hits = 0
    breached = None
    while time.time() - t0 < seconds:
        for env in bot.conn.drain_binary_tlm():
            t = getattr(env, "tlm", None)
            if t is None or not (t.flags & 0x02):
                continue
            x = float(t.otos.x) / 10.0                    # [mm] -> [cm]
            y = float(t.otos.y) / 10.0
            h = float(t.otos.heading) * 0.001             # [rad]
            if abs(x) > fence[0] or abs(y) > fence[1]:
                breached = (x, y)
                break
            hits += accumulate(TLMFrame.from_pb2(t), (x, y, h), perception, fmap, channel_y)
        if breached:
            break
        time.sleep(0.02)
    return hits, breached


def seed_retry(bot, x, y, heading, tries=4):  # [cm] [cm] [rad]
    """SEED until it reads back. Retries because the relay DROPS inbound.

    Measured 2026-08-07 (see clasi/issues/inbound-command-loss-needs-
    retransmit-not-a-slower-telemetry-stream.md): commands are lost host->
    robot on the half-duplex radio -- a move_wheels went missing with the
    encoders proving the robot never received it. Robot.seed() already
    verifies its own read-back, so a False here means the command did not
    land, not that the pose is wrong. One attempt is not enough over the
    relay; the retransmit belongs here until the protocol grows a real ARQ.
    """
    for attempt in range(tries):
        if bot.seed(x * 10.0, y * 10.0, wrap(heading)):
            return True
        print(f"    seed attempt {attempt + 1}/{tries} did not read back -- retrying")
        time.sleep(0.4)
    return False


def wait_for_goto(bot, goto_id, timeout=45.0, fmap=None, perception=None,
                  channel_y=None, fence=None, arrived_at=None,
                  hard_fence=RAIL_FENCE, fix_fn=None):
    """Block until the GO_TO's COMPLETION ACK arrives. Optionally map en route.

    The completion ack is the protocol's own end-of-move signal (docs/
    protocol-v5.md: a later frame's ack ring carries corr_id == the move id).

    Do NOT substitute "kFlagActive went clear" for it. Measured on tovez
    2026-08-08: the flag flickers False MID-MOVE -- Motion::Navigator re-issues
    internal planner moves with replace=true, so there are cycles with no owner
    and the flag momentarily drops. A single-sample test on it breaks out about
    one second into a four-second leg, and if the caller then halts, the robot
    stops in place having barely moved -- which is exactly how the first
    four-dot calibration run produced four readings 14cm apart for corners
    100cm apart, and a nonsense k=0.0778 fit.

    Returns (completed, breached_at_or_None).
    """
    deadline = time.time() + timeout
    last_cam = [0.0]
    while time.time() < deadline:
        for env in bot.conn.drain_binary_tlm():
            t = getattr(env, "tlm", None)
            if t is None:
                continue
            frame = TLMFrame.from_pb2(t)
            if t.flags & 0x02:
                x, y = float(t.otos.x) / 10.0, float(t.otos.y) / 10.0
                # `hard_fence` always; the caller's tighter fence as well when
                # one is given. No drive path is ever unfenced.
                #
                # The floor is a PARAMETER, defaulting to RAIL_FENCE, because
                # RAIL_FENCE is not a universal truth -- it insets the rail by
                # the robot's forward REACH in BOTH axes, which is only right
                # while the robot is driving along x. The edge sweep faces west
                # for every pass, so its y-extent is a half-width, and it
                # passes SWEEP_FENCE here instead. Left unparameterised, the
                # floor silently overrode the looser fence it was handed and
                # the sweep's first row (y=+30 against a 28.05 floor) breached
                # before the robot had mapped a single cell.
                if not within(hard_fence, x, y) or (
                        fence is not None and not within(fence, x, y)):
                    return False, (x, y)
                if fmap is not None:
                    h = float(t.otos.heading) * 0.001
                    accumulate(frame, (x, y, h), perception, fmap, channel_y)
            for ack in (frame.acks or []):
                if ack.corr_id == goto_id:
                    return True, None
        # Camera check on TRUTH, for the same reason as collect_straight: an
        # odometry fence cannot catch a fault whose symptom IS bad odometry.
        if fix_fn is not None and time.time() - last_cam[0] > CAM_CHECK_PERIOD:
            last_cam[0] = time.time()
            keep_lights_on()
            c = fix_fn()
            if c is not None and not within(hard_fence, c[0], c[1]):
                print(f"    CAMERA fence during transit: robot truly at "
                      f"({c[0]:+.1f},{c[1]:+.1f})cm -- halting")
                return False, (c[0], c[1])
        time.sleep(0.02)

    # The ack never came -- but did the robot actually get there? Acks ride
    # telemetry, and the relay drops inbound AND can lose a frame, so a
    # completed leg can look like a failed one. Ask the pose instead of
    # throwing away a leg that succeeded (this was costing a leg roughly every
    # four on 2026-08-08). Only trust it if the robot is genuinely parked at
    # the target, not merely somewhere plausible.
    if arrived_at is not None:
        pose = bot.pose_blocking(timeout=2.0)
        if pose is not None:
            dist = math.hypot(pose[0] / 10.0 - arrived_at[0],
                              pose[1] / 10.0 - arrived_at[1])
            if dist < 12.0:
                return True, None
    return False, None


# Sign relating commanded omega to the heading this script measures.
# POSITIVE commanded omega DECREASES heading (.claude/rules/playfield-testing.md;
# re-measured on tovez 2026-08-08: err was -12.4deg, omega was commanded at
# -0.9 rad/s, and the camera yaw went UP by 23.2deg). So to reduce a positive
# heading you command positive omega, i.e. the correction is -sign(err).
OMEGA_SIGN = -1.0

# Turn rate for the squaring pivot, in two speeds.
#
# The robot carries ~130ms of actuation latency, so whatever it is doing when
# the stop condition is reached it keeps doing for another 130ms. At 0.9 rad/s
# that is 6.7deg of overshoot against a 1.4deg tolerance, which cannot
# converge -- measured 2026-08-08, a 12.4deg demand produced a 23.2deg turn.
# At 0.35 rad/s the same latency is worth only ~2.6deg.
#
# But slow-everywhere is wasteful here, because the corrections are not small.
# A reversing scan alternates leg direction, so the GO_TO that carries the
# robot to the next leg's start leaves it facing the way it travelled -- up to
# 180deg from where the leg needs to point. At 0.35 rad/s that is a 9s pivot,
# every leg. So: coarse rate until the error is small, fine rate to land it.
TURN_OMEGA_FAST = 0.9    # [rad/s] while the error is still large
TURN_OMEGA_FINE = 0.35   # [rad/s] final approach, where overshoot decides
TURN_FINE_BAND = 0.35    # [rad] (~20deg) error below which we slow down

# The pivot is commanded in ODOMETRY angle but judged in CAMERA angle, and on
# the vinyl mat those are wildly different because the tyres scrub: a pivot
# that odometry scored at 53deg turned the robot 2.9deg (tovez, 2026-08-08).
# So ask for more than the error and let the camera-refereed loop converge.
# The cap stops a large over-ask from spinning past the target and hunting.
# The pivot is commanded OPEN-ENDED and stopped by the host watching the
# camera, because the firmware's own stop_angle is measured in the corrupted
# quantity. These two just have to be bigger than any real correction.
TURN_OPEN_ANGLE = 6.5        # [rad] ~2 full turns; never the actual stop
TURN_OPEN_TIMEOUT = 15000.0  # [ms] backstop if the host stops watching
TURN_WATCH_LIMIT = 12.0      # [s] longest we watch one pivot before retrying
TURN_BLIND_FRAMES = 20       # consecutive camera misses before we halt

# How often a moving pass is checked against camera truth. At 180mm/s the
# robot covers ~5cm between checks, which is inside every margin here.
CAM_CHECK_PERIOD = 0.3       # [s]

# The bench-room lights are on a network relay and they switch themselves OFF
# on their own (.claude/rules/hardware-bench-testing.md). That used to cost a
# dark frame; now it is a SAFETY issue, because every guard on this drive path
# is refereed by the camera and a dark room blinds all of them at once. It
# happened three times in the session of 2026-08-08, once mid-sweep. So the
# run re-asserts the lights on a timer rather than trusting them to stay on.
LIGHTS_URL = "http://192.168.1.122/rpc/Switch.Set?id=0&on=true"
LIGHTS_PERIOD = 60.0  # [s]

# Placing a row start. A single GO_TO stops at the Navigator's own arrival
# tolerance, which is wider than the row pitch -- so the row start is
# re-issued until the CAMERA agrees it is where it was asked for.
ROW_ARRIVE = 25.0       # [mm] tighter than the Navigator default
ROW_PLACE_TOL = 2.0     # [cm] good enough for a 4cm pitch
ROW_PLACE_TRIES = 3


def keep_lights_on(state=[0.0]):  # noqa: B006  -- deliberate call-scoped cache
    """Poke the light relay, at most every LIGHTS_PERIOD. Never raises: a
    failure to reach the relay must not take down a run that is mid-drive."""
    if time.time() - state[0] < LIGHTS_PERIOD:
        return
    state[0] = time.time()
    try:
        import urllib.request
        urllib.request.urlopen(LIGHTS_URL, timeout=3).read()
    except Exception:
        pass


def _spin_to_heading(bot, fix_fn, target, err0, tol):  # [rad] [rad] [rad]
    """Watch the camera during an open-ended pivot; return when to stop.

    Returns True if the camera saw us reach `tol`, None if we ran out of
    time (caller may retry), False if the camera went away entirely -- the
    one case where continuing to spin a robot nobody can see is unacceptable.

    Stops on a SIGN FLIP as well as on tolerance: overshooting past the target
    means the next round would just chase it back, and a pivot nobody halts is
    how a bounded correction becomes a spin.
    """
    t0 = time.time()
    blind = 0
    while time.time() - t0 < TURN_WATCH_LIMIT:
        c = fix_fn()
        if c is None:
            blind += 1
            if blind > TURN_BLIND_FRAMES:
                print("    lost tag 100 mid-pivot -- halting")
                return False
            time.sleep(0.05)
            continue
        blind = 0
        err = wrap(target - c[2])
        if abs(err) < tol or (err * err0) < 0:
            return True
        time.sleep(0.02)
    return None


def align_heading(bot, target, tol=0.05, tries=8, fix_fn=None):  # [rad] [rad]
    """Square the robot to `target` heading before a straight leg.

    Scored against the CAMERA when `fix_fn` is given, and it always should be
    on the playfield. Odometry is not usable as the referee for a pivot on
    this surface. Measured on tovez 2026-08-08, seeded from the camera
    immediately beforehand: align_heading drove odometry from +126.9deg to
    -178.1deg -- 53deg of rotation by its own reckoning -- while the camera
    showed the robot had physically turned 2.9deg, from +126.9 to +129.8.
    The subsequent "straight west" leg then ran at a true +128deg and put the
    robot 20cm off where every guard believed it was, on bare wood past the
    mat's north edge, with the line sensor reading ~229 where odometry said
    mid-mat.
    Cause: the wheels SCRUB on the vinyl mat during a pivot. The encoders
    turn, the robot does not, and heading here is encoder-derived
    (data/robots/tovez.json: otos_untrusted true, weight_heading_otos 0.0),
    so odometry over-reports rotation by roughly an order of magnitude. The
    same function scored 0.9deg residual earlier that day -- at x=+52.4,
    which is OFF the mat on bare wood, where the tyres grip.
    So the pivot is commanded open-loop and then MEASURED against the camera,
    re-seeding odometry from truth each round so the fence and the
    cross-track check downstream are working from reality.

    move_twist drives along the robot's CURRENT heading, not along a world
    axis -- so a "straight" leg issued at an unknown heading walks diagonally.
    Measured 2026-08-08: a leg meant to run along -x drifted to y=+38.8cm and
    tripped the fence, because GO_TO makes no promise about the heading it
    leaves the robot at.

    This is the ONLY turning a reversing scan needs: a small correction once
    per leg, never a 180 deg pivot.

    `tol` is set by the leg length, not by taste. Residual heading error
    becomes cross-track error over the leg: at the old 0.05 rad (2.9deg) a
    92cm leg can wander 4.6cm, which on its own nearly exhausts the 6cm
    cross-track budget and aborted leg 4 of the 2026-08-08 run. 0.025 rad
    (1.4deg) halves that to ~2.3cm, and is achievable -- the camera-scored
    probe the same day landed a 33.8deg correction with 0.9deg residual.

    Returns True ONLY if the heading was actually verified inside `tol`.

    That word "only" is the whole point, and it is a bug fix. This function
    used to `return True` after exhausting its four correction attempts --
    the same answer it gives on success -- so a heading it had FAILED to
    correct was indistinguishable from one it had. The caller then ran a
    full-mat-width straight leg (2 * x_span, ~100cm) along that unverified
    heading. At 11 deg of residual error a 100cm leg walks 20cm sideways;
    at 20 deg it walks 34cm. That is how the run of 2026-08-08 put the
    robot into the rails three times without ever issuing a command that
    pointed at a rail: the robot drove exactly where it was told, along a
    heading nobody had checked. Not a geofence failure -- the geofence was
    the last line, and the excursions started upstream of it.
    """
    def truth():
        """Heading to steer by: the camera when we have it, else odometry."""
        if fix_fn is not None:
            c = camera_fix(fix_fn, samples=5)
            if c is not None:
                # Put odometry back on truth every round. Whatever the pivot
                # did to the encoder-derived heading, the fence and the
                # cross-track check that follow must not inherit it.
                seed_retry(bot, c[0], c[1], c[2], tries=2)
                return c[2], (c[0], c[1])
        p = bot.pose_blocking(timeout=3.0)
        if p is None:
            return None, None
        return p[2], (p[0] / 10.0, p[1] / 10.0)

    for _ in range(tries):
        heading, pos = truth()
        if heading is None:
            return False
        pose = bot.pose_blocking(timeout=3.0)
        if pose is None:
            return False
        err = wrap(target - heading)
        if abs(err) < tol:
            return True
        # Correlate on a REAL move id and wait for that move's completion ack.
        # move_id=0 plus "break when kFlagActive is clear" was the third copy
        # of the same defect in this file (see drive_closing): the flag drops
        # for a cycle whenever Motion::Navigator re-issues with replace=true,
        # so this loop broke out immediately and halt_verified() estopped the
        # pivot before it had turned. Four attempts then re-read an UNCHANGED
        # pose -- which is why the run of 2026-08-08 reported the identical
        # residual (-12.3deg) from two independent align_heading() calls, a
        # tell that no turning was happening at all rather than that it was
        # converging badly.
        rate = TURN_OMEGA_FINE if abs(err) <= TURN_FINE_BAND else TURN_OMEGA_FAST
        # Do NOT stop this pivot on `stop_angle`. That condition is evaluated
        # in ODOMETRY angle, which is exactly the quantity the scrub corrupts:
        # measured 2026-08-08, odometry over-reports rotation ~18x on the mat,
        # so a stop_angle of 167deg ended the move after ~9deg of real turn,
        # and ten attempts spent 66s to cover 90deg. Command an open-ended
        # rotation instead and stop it from OUT HERE, watching the camera.
        turn_id = bot._next_id()
        bot.p.move_twist(0.0, 0.0, OMEGA_SIGN * math.copysign(rate, err),
                         stop_angle=TURN_OPEN_ANGLE,
                         timeout=TURN_OPEN_TIMEOUT, move_id=turn_id)
        if fix_fn is not None:
            spun = _spin_to_heading(bot, fix_fn, target, err, tol)
            halt_verified(bot)
            time.sleep(0.4)
            if spun is False:
                return False
            continue
        t0 = time.time()
        anchor = (pose[0] / 10.0, pose[1] / 10.0)
        done = False
        # A 180deg pivot at the fine rate is ~9s; allow for that plus latency.
        while time.time() - t0 < 20.0 and not done:
            for env in bot.conn.drain_binary_tlm():
                f = getattr(env, "tlm", None)
                if f is None:
                    continue
                if f.flags & 0x02:
                    x, y = float(f.otos.x) / 10.0, float(f.otos.y) / 10.0
                    # Guard on DRIFT, not on absolute position. This is a pure
                    # pivot (v_x = v_y = 0), so it does not translate and an
                    # absolute fence is the wrong test: it deadlocks the robot
                    # exactly when re-orienting is what would save it. Measured
                    # 2026-08-08: parked at x=+52.4 against a 52.15 rail fence,
                    # align_heading tripped and returned False before turning
                    # at all -- unable to turn because it was too close to a
                    # wall, which is precisely backwards. What IS worth
                    # aborting on is a "pivot" that turns out to be driving.
                    if math.hypot(x - anchor[0], y - anchor[1]) > PIVOT_DRIFT_LIMIT:
                        print(f"    pivot DRIFTED {math.hypot(x - anchor[0], y - anchor[1]):.1f}cm "
                              f"-- not a pure rotation, halting")
                        halt_verified(bot)
                        return False
                for ack in (TLMFrame.from_pb2(f).acks or []):
                    if ack.corr_id == turn_id:
                        done = True
            time.sleep(0.02)
        halt_verified(bot)
        time.sleep(0.4)

    # Attempts spent. Say what actually happened -- do NOT report the
    # success answer for a failure (see the docstring).
    heading, _pos = truth()
    if heading is None:
        return False
    err = wrap(target - heading)
    if abs(err) < tol:
        return True
    print(f"    align_heading GAVE UP: residual {math.degrees(err):+.1f}deg "
          f"(tol {math.degrees(tol):.1f}deg) -- refusing to run the leg")
    return False


def straight_scan(bot, dc, fmap, perception, channel_y, fence, x_span, y_span,
                  pitch, speed, fix_fn, reseed_every):
    """Boustrophedon that NEVER turns around: it reverses.

    Stakeholder suggestion 2026-08-08, and it is the right shape for this
    robot. A differential drive reverses exactly as well as it goes forward,
    so alternating the SIGN of v_x sweeps the mat without a single
    end-of-leg turn. That matters for three measured reasons:

    1. The turns were where the position error came from. Every 180 deg
       pivot is a chance to accumulate heading error that then smears a
       whole leg of samples.
    2. They were where the fence trips came from -- the Navigator overshoots
       the end of a leg before settling, which tripped the fence at x=-54 on
       a -48 target.
    3. They cost most of the run time, which is why earlier attempts only
       got a dozen legs in before something went wrong.

    Heading stays fixed, so the lever arm is unaffected: the sensors sit at
    body +96mm forward whichever way the robot is travelling.
    """
    from goto_otos import FRAME_WORLD

    legs = []
    y = -y_span
    forward = True
    while y <= y_span + 1e-6:
        legs.append((y, forward))
        forward = not forward
        y += pitch
    # Work outward from the centre: the robot starts centred, so this keeps
    # the very first transit short instead of sending it to a far corner.
    legs.sort(key=lambda leg: abs(leg[0]))

    travelled = 0.0
    for i, (y, forward) in enumerate(legs):
        # Step onto this leg's start with a GO_TO (short move), then run the
        # leg itself as a pure straight MOVE.
        start_x = -x_span if forward else x_span
        _c, gid = bot.goto_wire(start_x * 10.0, y * 10.0, FRAME_WORLD,
                                speed=speed, timeout=40000.0)
        done, breach = wait_for_goto(bot, gid, timeout=45.0, fmap=fmap,
                                     perception=perception, channel_y=channel_y,
                                     fence=fence, arrived_at=(start_x, y))
        if breach:
            return breach
        if travelled >= reseed_every or i == 0:
            fix = camera_fix(fix_fn, samples=9)
            if fix is not None and seed_retry(bot, fix[0], fix[1], fix[2]):
                travelled = 0.0
                print(f"  [{i}] re-seeded at ({fix[0]:+.1f},{fix[1]:+.1f})cm")

        # Square up to the x axis, then run the leg straight. Heading is 0
        # (east) for BOTH directions -- a backward leg reverses along the same
        # heading rather than turning around, which is the whole point.
        if not align_heading(bot, 0.0):
            # Do NOT run a full-width leg on a heading that would not square
            # up. Re-fix from the camera, re-seed, and try once more -- a
            # stale pose is the likeliest reason the correction chased its
            # own tail. Still failing means stop, not "drive anyway".
            fix = camera_fix(fix_fn, samples=9)
            if fix is None or not seed_retry(bot, fix[0], fix[1], fix[2]):
                print("  heading unresolvable and no camera re-seed -- halting")
                return None
            travelled = 0.0
            if not align_heading(bot, 0.0):
                print("  heading will not square up even after a camera "
                      "re-seed -- halting rather than driving blind")
                return None
        # Cross-track is measured from where the robot ACTUALLY is, not from
        # the leg's nominal y. The GO_TO onto the leg start lands within its
        # arrival tolerance, not exactly -- measured 2026-08-08, a leg planned
        # at y=+7.0 began at y=+8.7. Scoring that leg against +7.0 spends
        # 1.7cm of the budget before the robot has moved, and then aborts a
        # leg that was actually running straight. The leg is a straight line
        # from wherever it starts; that is what the guard should hold it to.
        # The map is unaffected either way -- every sample is stamped with the
        # measured pose, never with the planned one.
        here = bot.pose_blocking(timeout=3.0)
        leg_ref = (here[1] / 10.0) if here is not None else y

        dist = 2.0 * x_span * 10.0                      # [mm]
        v_x = speed if forward else -speed
        bot.p.move_twist(v_x, 0.0, 0.0, stop_distance=dist,
                         timeout=int(dist / speed * 1000 * 3), move_id=0)
        breach = collect_straight(bot, fmap, perception, channel_y, fence,
                                  seconds=dist / speed + 6.0,
                                  leg_y=leg_ref, cross_track=CROSS_TRACK_LIMIT)
        if breach:
            return breach
        travelled += 2.0 * x_span
        print(f"  [{i:2d}] y={y:+6.1f} {'->' if forward else '<-'}  "
              f"refl_cells={fmap.refl_cells} colour_cells={fmap.color_cells}")
    return None


def collect_straight(bot, fmap, perception, channel_y, fence, seconds,
                     leg_y=None, cross_track=None,
                     hard_fence=RAIL_FENCE, fix_fn=None):  # [cm] [cm]
    """Map during a straight MOVE; stop early once it goes inactive.

    Pass `fix_fn` and the fence is checked against the CAMERA as well as
    against odometry, a few times a second. That is not belt-and-braces, it
    is the only check that was working: on 2026-08-08 this pass ran with
    odometry reporting y=+18.6 while the camera had the robot at y=+38.5, out
    on the bare wood past the mat's north edge. Every guard here said mid-mat
    and safe, because every guard read odometry. The line sensor agreed with
    the camera -- the cells odometry placed mid-mat came back at ~229, which
    is wood, not white vinyl.
    An odometry-only fence cannot catch a fault whose whole symptom is that
    odometry is wrong.

    `leg_y`/`cross_track` add the guard that actually catches an excursion
    EARLY. A "straight" leg is supposed to hold one y; watching how far it
    has departed from that y is a direct measurement of the leg going wrong,
    and it fires whatever the cause -- residual heading error, a slipping
    wheel on the vinyl, a dropped command leaving a stale twist running.

    The geofence alone cannot do this job. It only speaks up once the robot
    is already at the edge of the field, by which point the leg has been
    wrong for most of its length and the remaining stopping distance is the
    only thing between the robot and the rail. Cross-track fires while the
    robot is still in open space, mid-mat, with room to stop.
    """
    t0 = time.time()
    idle = 0
    last_cam = 0.0
    while time.time() - t0 < seconds:
        # Camera check a few times a second, on TRUTH, independent of
        # anything odometry believes.
        if fix_fn is not None and time.time() - last_cam > CAM_CHECK_PERIOD:
            last_cam = time.time()
            keep_lights_on()
            c = fix_fn()
            if c is not None:
                if not within(hard_fence, c[0], c[1]):
                    print(f"    CAMERA fence: robot truly at "
                          f"({c[0]:+.1f},{c[1]:+.1f})cm -- halting")
                    return (c[0], c[1])
                if (leg_y is not None and cross_track is not None
                        and abs(c[1] - leg_y) > cross_track):
                    print(f"    CAMERA cross-track: leg y={leg_y:+.1f} but "
                          f"robot truly at y={c[1]:+.1f} "
                          f"({abs(c[1] - leg_y):.1f}cm off) -- halting")
                    return (c[0], c[1])
        for env in bot.conn.drain_binary_tlm():
            t = getattr(env, "tlm", None)
            if t is None or not (t.flags & 0x02):
                continue
            x, y = float(t.otos.x) / 10.0, float(t.otos.y) / 10.0
            # `hard_fence` defaults to RAIL_FENCE, so every existing caller is
            # unchanged; the edge sweep passes its own, for the reason spelled
            # out in wait_for_goto.
            if not within(hard_fence, x, y) or not within(fence, x, y):
                return (x, y)
            if (leg_y is not None and cross_track is not None
                    and abs(y - leg_y) > cross_track):
                print(f"    CROSS-TRACK abort: leg y={leg_y:+.1f} but robot "
                      f"at y={y:+.1f} ({abs(y - leg_y):.1f}cm off, limit "
                      f"{cross_track:.1f}cm)")
                return (x, y)
            accumulate(TLMFrame.from_pb2(t), (x, y, float(t.otos.heading) * 0.001),
                       perception, fmap, channel_y)
            idle = idle + 1 if not (t.flags & 0x04) else 0
        if idle > 40 and time.time() - t0 > 2.0:
            break
        time.sleep(0.02)
    return None


def edge_sweep(bot, fmap, perception, channel_y, fix_fn, x_east, x_west,
               y_start, pitch, rows, speed):
    """Scan in EVEN ROWS: every pass runs east->west, at constant y, always.

    Stakeholder spec 2026-08-08. The obvious alternative -- a boustrophedon
    that alternates the direction of travel -- gives every other row a
    different heading, and that is what turns a scan into a zigzag rather
    than a set of rows. Whatever residual heading error align_heading leaves
    behind tilts a westbound pass one way and an eastbound pass the other, so
    consecutive rows converge at one end and splay at the other; adjacent
    passes then disagree about where their own y is and the map reads as
    smeared bands instead of lines. Driving every pass on the SAME heading
    (pi, west) removes that degree of freedom outright: the residual is the
    same sign on every row, so the rows stay parallel and the whole scan is
    offset rather than sheared.

    Two more reasons the extra transit is worth paying for:

    - The lever arm is unchanged pass to pass. The sensors sit ahead of the
      centre of rotation, so the swath they trace is a function of heading;
      hold heading fixed and every row samples the same body geometry, which
      is what makes rows comparable to each other at all.
    - There is one heading to square to, not two. Every row squares to the
      same pi, so a correction that works on row 0 works on row 15; a
      reversing scan has to land BOTH 0 and pi correctly, and gets a fresh
      chance to leave a large residual on every second row.

    Only the RETURNS are angled. The GO_TO that carries the robot from one
    row's west end back to the next row's east end also steps it down one
    pitch in y, so the row change costs no separate manoeuvre -- the diagonal
    IS the return.

    Returns the breach position if the fence or the cross-track guard fired,
    otherwise None. None also covers a give-up (lost tag, unseedable pose,
    heading that will not square) -- those print their own reason first.
    """
    from goto_otos import FRAME_WORLD

    for i in range(rows):
        y_target = y_start - i * pitch

        # This GO_TO *is* the angled return from the previous row's west end:
        # back east and down one pitch in a single move.
        #
        # Issued REPEATEDLY until the camera says the row start is actually
        # where it was asked for. One GO_TO is not enough to place a row: the
        # Navigator stops at its own arrival tolerance, which is far wider
        # than the 4cm pitch these rows need. Measured 2026-08-08, row 0 was
        # asked for y=+30.0 and began at y=+16.6 -- a 13.4cm miss, three
        # pitches of error, which makes "even rows" impossible no matter how
        # straight each pass is. Re-issuing from closer in converges, because
        # a fixed arrival tolerance is a smaller absolute error the shorter
        # the move.
        breach = None
        for attempt in range(ROW_PLACE_TRIES):
            _c, gid = bot.goto_wire(x_east * 10.0, y_target * 10.0,
                                    FRAME_WORLD, speed=speed,
                                    arrive=ROW_ARRIVE, timeout=40000.0)
            _done, breach = wait_for_goto(bot, gid, timeout=45.0, fmap=fmap,
                                          perception=perception,
                                          channel_y=channel_y,
                                          fence=SWEEP_FENCE,
                                          arrived_at=(x_east, y_target),
                                          hard_fence=SWEEP_FENCE,
                                          fix_fn=fix_fn)
            if breach:
                return breach
            c = camera_fix(fix_fn, samples=5)
            if c is None:
                break
            if abs(c[1] - y_target) <= ROW_PLACE_TOL:
                break
            print(f"  [{i:2d}] row start y={c[1]:+.1f} wanted {y_target:+.1f} "
                  f"({abs(c[1]-y_target):.1f}cm out) -- re-issuing "
                  f"({attempt + 1}/{ROW_PLACE_TRIES})")
            # Re-seed before re-issuing, or the retry is a no-op: the GO_TO is
            # driven in the ROBOT's frame, and if that frame has drifted the
            # robot already believes it is standing on the target. Measured
            # 2026-08-08: three re-issues in a row returned the identical
            # y=+25.9, because each one was asking a robot that thought it had
            # arrived to drive to where it thought it already was.
            if not seed_retry(bot, c[0], c[1], c[2]):
                break
        # A missing completion ack is deliberately NOT fatal here. The camera
        # fix immediately below re-establishes the frame from the camera
        # rather than from the Navigator's opinion of where it finished, and
        # the pass itself is a fixed distance from wherever the robot actually
        # is -- so a dropped ack costs nothing that the next two lines do not
        # already re-measure.

        fix = camera_fix(fix_fn, samples=9)
        if fix is None:
            print(f"  [{i:2d}] camera cannot see tag 100 -- stopping the sweep")
            return None
        if not seed_retry(bot, fix[0], fix[1], fix[2]):
            print(f"  [{i:2d}] seed did not read back after retries -- "
                  "stopping the sweep")
            return None

        # Drive the pass as a GO_TO to the far end of the SAME row, not as an
        # open-loop straight twist.
        #
        # This is the lesson of the whole 2026-08-08 session in one line: on
        # this mat the wheels scrub, encoder-derived heading over-reports
        # rotation by an order of magnitude, and nothing that steers by
        # odometry heading holds a line. An open-loop move_twist commits to
        # whatever heading it starts with and cannot notice it is wrong -- one
        # such pass, begun squared and seeded, finished 20cm north of where
        # every guard believed it was, out on the bare wood.
        # A GO_TO is closed-loop on a WORLD target: the Navigator keeps
        # steering at the far end of the row for the whole pass, so heading
        # error is corrected continuously instead of integrated. It costs a
        # turn at each end, which the stakeholder would rather avoid -- but
        # even rows were the actual requirement, and no amount of squaring up
        # beforehand buys them while the heading reference is this bad.
        _c, pid = bot.goto_wire(x_west * 10.0, y_target * 10.0, FRAME_WORLD,
                                speed=speed, arrive=ROW_ARRIVE,
                                timeout=int(abs(x_east - x_west) * 10.0
                                            / speed * 1000 * 4))
        done, breach = wait_for_goto(bot, pid, timeout=60.0, fmap=fmap,
                                     perception=perception,
                                     channel_y=channel_y, fence=SWEEP_FENCE,
                                     arrived_at=(x_west, y_target),
                                     hard_fence=SWEEP_FENCE, fix_fn=fix_fn)
        if breach:
            return breach
        end = camera_fix(fix_fn, samples=5)
        end_y = end[1] if end is not None else float("nan")
        print(f"  [{i:2d}] y={y_target:+6.1f} E->W ended y={end_y:+6.1f}  "
              f"refl_cells={fmap.refl_cells} colour_cells={fmap.color_cells}")
    return None


def drive_closing(bot, goto_id, r0, timeout=45.0, slack=4.0):
    """Run a move that must reduce the robot's radius from the field centre.

    Used only for recovery, where the robot may start outside the fence and an
    absolute limit would deadlock it. The invariant here is directional, not
    positional: driving home is always allowed, driving further out never is.

    Ends on the COMPLETION ACK, exactly like wait_for_goto, and for the same
    measured reason: `kFlagActive` (bit 2) flickers False MID-MOVE because
    Motion::Navigator re-issues its internal planner moves with replace=true,
    leaving cycles with no motion owner. This function used to return True on
    the first frame with that bit clear, which it sees almost immediately --
    so the caller halted a fraction of a second into every attempt and the
    robot barely moved. Measured 2026-08-08: three recovery attempts from
    (-15.7,+4.9) left the robot at (-16.4,+4.5), i.e. 0.8cm of travel, and
    reported "could not converge" for a 16cm drive.

    This is the SECOND place that flag fooled this script. If a third drive
    loop is ever added, it waits on the ack too.
    """
    worst = r0 + slack
    t0 = time.time()
    while time.time() - t0 < timeout:
        for env in bot.conn.drain_binary_tlm():
            t = getattr(env, "tlm", None)
            if t is None or not (t.flags & 0x02):
                continue
            r = math.hypot(float(t.otos.x) / 10.0, float(t.otos.y) / 10.0)
            if r > worst:
                return False
            worst = min(worst, r + slack)
            for ack in (TLMFrame.from_pb2(t).acks or []):
                if ack.corr_id == goto_id:
                    return True
        time.sleep(0.02)
    return True


def recover_to_centre(bot, dc, fix_fn, speed):
    """Put the robot at the CENTRE and localised before anything else runs.

    Never start a raster from wherever the robot was left. Three reasons, all
    observed on 2026-08-08:

    1. The previous run may have ended outside the fence, so the very first
       fence check trips and the run dies before it maps anything.
    2. A first waypoint far from the robot means a long diagonal transit that
       the raster never planned and the fence was not sized for.
    3. The camera is most trustworthy at the centre. Parallax on the elevated
       tag scales with radius from the optical axis, so a fix taken here is
       the least-corrected one available -- which is exactly why it is the
       right place to establish the frame the whole run depends on.

    Returns True once the camera CONFIRMS the robot is near the centre.
    """
    from goto_otos import FRAME_WORLD

    for attempt in range(3):
        fix = camera_fix(fix_fn, samples=9)
        if fix is None:
            print("  centre: camera cannot see tag 100")
            return False
        if not seed_retry(bot, fix[0], fix[1], fix[2]):
            print("  centre: seed did not read back")
            return False
        # Accept 10cm, not 6. The GO_TO stops at the Navigator's own arrival
        # tolerance, which is WIDER than 6cm -- so a 6cm test can never pass:
        # the move reports complete without the robot driving, and the loop
        # burns all three attempts reading the identical position. Measured
        # 2026-08-08: recovery reached (-3.8,+6.3) (7.4cm out) and then sat
        # there for attempts 2 and 3. Being within 10cm of centre is adequate
        # for what this function is for -- start from the middle, localised --
        # and the seed that follows is taken from the CAMERA at wherever it
        # actually stopped, so nothing downstream assumes 0,0.
        if math.hypot(fix[0], fix[1]) < 10.0:
            print(f"  centred at ({fix[0]:+.1f},{fix[1]:+.1f})cm, frame established")
            return True
        print(f"  at ({fix[0]:+.1f},{fix[1]:+.1f})cm -- driving to centre "
              f"(attempt {attempt + 1}/3)")
        # Recovery legitimately STARTS outside the fence, so an absolute test
        # would deadlock a lost robot. Require the drive to CLOSE on the
        # centre instead: any increase in radius means it is heading the wrong
        # way and gets halted immediately.
        _c, gid = bot.goto_wire(0.0, 0.0, FRAME_WORLD, speed=speed, timeout=40000.0)
        if not drive_closing(bot, gid, math.hypot(fix[0], fix[1])):
            print("  recovery drove AWAY from the centre -- halted")
            halt_verified(bot)
            return False
        halt_verified(bot)
        time.sleep(0.8)
    print("  centre: could not converge")
    return False


def calibrate_from_dots(bot, dc, speed, out_path, points):
    """Drive to each surveyed orange dot, fix at rest, solve the parallax.

    Driven rather than hand-placed so all four observations share one pose
    convention and one settle discipline. The robot's own arrival error does
    not matter: the fit uses the CAMERA reading against the DOT's surveyed
    position, and the camera sees where the robot actually stopped.

    That is only true if the robot really is on the dot, so each leg is
    re-driven from a fresh camera fix and the arrival residual is reported --
    a dot the robot missed by 5cm would otherwise quietly bias the fit.
    """
    from goto_otos import FRAME_WORLD

    obs = []
    for name, (tx, ty) in points:
        fix = camera_fix(lambda: read_tag(dc))
        if fix is None:
            print(f"  {name}: camera lost tag 100")
            return None
        if not seed_retry(bot, fix[0], fix[1], fix[2]):
            print(f"  {name}: re-seed did not read back after retries")
            return None
        _corr, goto_id = bot.goto_wire(tx * 10.0, ty * 10.0, FRAME_WORLD,
                                       speed=speed, timeout=40000.0)
        done, breach = wait_for_goto(bot, goto_id, timeout=40.0, fence=RAIL_FENCE)
        if breach:
            print(f"  {name}: RAIL FENCE hit at ({breach[0]:+.1f},{breach[1]:+.1f})cm")
            halt_verified(bot)
            return None
        if not done:
            print(f"  {name}: no completion ack in 40s -- halting")
            halt_verified(bot)
            return None
        halt_verified(bot)
        time.sleep(0.8)                                     # settle: fix AT REST
        fix = camera_fix(lambda: read_tag(dc), samples=11)
        if fix is None:
            # The camera does NOT image the whole field: create_playfield
            # reports two corner markers at NEGATIVE pixel y, i.e. above the
            # frame. Tag 1 sits directly under the lens and never drops, while
            # the far corners do. Rather than fail, retreat toward the centre
            # -- the parallax model is linear, so a smaller well-seen box fits
            # it just as well and extrapolates.
            print(f"  {name}: tag not visible at ({tx:+.0f},{ty:+.0f}) -- "
                  f"outside the camera's usable area; skipping")
            continue
        obs.append(((tx, ty), (fix[0], fix[1])))
        print(f"  {name}: truth=({tx:+6.1f},{ty:+6.1f})  camera=({fix[0]:+6.1f},{fix[1]:+6.1f})")

    if len(obs) < 3:
        print(f"\nonly {len(obs)} usable points -- need 3 to solve for scale AND "
              "axis. Re-run with a smaller --calibrate-box.")
        return None
    k, cx, cy, resid = solve_parallax(obs)
    print(f"\nsolved: k={k:.4f}  optical axis=({cx:+.2f},{cy:+.2f})cm  "
          f"mean residual={resid:.2f}cm")
    if resid > 2.0:
        print("  WARNING: residual > 2cm -- the robot probably did not land on the "
              "dots, or the model does not fit. Do NOT trust this map's absolute "
              "positions until this is understood.")
    cal = {"k": k, "cx": cx, "cy": cy, "residual_cm": resid,
           "observations": [{"truth": list(t), "camera": list(m)} for t, m in obs],
           "note": ("measured = c + k*(true - c). Apply undo_parallax() to every "
                    "camera reading of tag 100. Re-run whenever the tag mount "
                    "height or the camera moves.")}
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(cal, indent=2) + "\n")
    print(f"wrote {out_path}")
    return cal


def score_against_truth(fmap: FieldMap) -> None:
    """Print per-feature contrast against the camera-surveyed mat.

    Turns "the map looks about right" into a number per feature. For each
    surveyed shape it compares cells ON the printed outline with cells well
    OFF it but still nearby, so the comparison is local and does not care
    about lighting drift across the field.

    Read the p90 column, not the median. The printed lines are ~5mm wide and
    the four line-sensor channels are POINTS (at -32/-8/+8/+32mm), so a pass
    lays down four thin tracks rather than a swath: most cells inside the
    outline band never had a sensor over the line at all, and the median is
    therefore dominated by the white mat beside it. p90 is what the sensor
    saw when it actually crossed.
    """
    refl = fmap.reduce_refl()
    if not refl.count():
        print("\nno reflectance cells -- nothing to score")
        return
    xs = np.linspace(-X_LIM, X_LIM, GRID_W)
    ys = np.linspace(-Y_LIM, Y_LIM, GRID_H)
    XX, YY = np.meshgrid(xs, ys)

    def seg_dist(px, py, a, b):
        ax, ay = a
        bx, by = b
        dx, dy = bx - ax, by - ay
        L = dx * dx + dy * dy
        if L == 0:
            return np.hypot(px - ax, py - ay)
        t = np.clip(((px - ax) * dx + (py - ay) * dy) / L, 0, 1)
        return np.hypot(px - (ax + t * dx), py - (ay + t * dy))

    print(f"\n{'feature':<15}{'n':>6}{'on p90':>9}{'off med':>9}{'contrast':>10}")
    print("-" * 49)
    for label, pts in MAT_TRUTH:
        segs = list(zip(pts, pts[1:] + ([pts[0]] if len(pts) > 2 else [])))
        d = np.full(XX.shape, 1e9)
        for a, b in segs:
            d = np.minimum(d, seg_dist(XX, YY, a, b))
        on = (d < 1.5) & ~refl.mask
        off = (d > 4.0) & (d < 9.0) & ~refl.mask
        if on.sum() < 5 or off.sum() < 5:
            print(f"{label:<15}{on.sum():>6}   NOT COVERED by this scan")
            continue
        on_p90 = float(np.percentile(refl.data[on], 90))
        off_med = float(np.median(refl.data[off]))
        print(f"{label:<15}{on.sum():>6}{on_p90:>9.1f}{off_med:>9.1f}"
              f"{on_p90 - off_med:>+10.1f}")


# --------------------------------------------------------------------------
# Rendering
# --------------------------------------------------------------------------

def render(fmap: FieldMap, path: pathlib.Path, title: str) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    refl = fmap.reduce_refl()
    color, cseen = fmap.reduce_color()

    fig, axes = plt.subplots(1, 2, figsize=(17, 6))
    extent = [-X_LIM, X_LIM, -Y_LIM, Y_LIM]

    ax = axes[0]
    # Two things about this scale, both measured on the KIPR mat 2026-08-08.
    #
    # 1. The line sensor reads INVERTED: high counts = DARK surface. White mat
    #    sits near 0, bare wood near 240, and the printed lines in between.
    #    `gray_r` maps high counts to dark ink, so the plot looks like the
    #    thing being scanned instead of its photographic negative.
    # 2. Scale on PERCENTILES, never min/max. The mat's whole signal lives in
    #    roughly 0-150 (89% of cells below 60), but any cell that catches bare
    #    wood off the mat edge reads ~240 -- and a handful of those stretched
    #    a min/max ramp so far that every printed line on the mat rendered as
    #    the same near-black as the mat itself. The features were IN the data
    #    the whole time; the colourbar was hiding them.
    # p90, not p98, for the upper end. The surface is BIMODAL -- mat (0-150)
    # and bare wood (~240) are two separated populations, not one spread --
    # so even p98 lands in the wood mode as soon as ~2% of cells see off-mat
    # floor, and the mat collapses into the bottom sixth of the ramp again.
    # Measured on this dataset: p50=22, p75=35, p90=65, p98=238. p90 puts the
    # mat's printed lines across the full ramp; wood and the darkest lines
    # both clip to solid black, which is the right trade -- the features being
    # read are ON the mat.
    if refl.count():
        vals = refl.compressed()
        vmin, vmax = (float(np.percentile(vals, 2)),
                      float(np.percentile(vals, 90)))
        if vmax - vmin < 1.0:      # degenerate (uniform surface) -- fall back
            vmin, vmax = float(vals.min()), float(vals.max()) + 1.0
    else:
        vmin, vmax = 0.0, 255.0
    im = ax.imshow(refl, origin="lower", extent=extent, cmap="gray_r",
                   vmin=vmin, vmax=vmax)
    ax.set_title(f"line sensor, HIGH = DARK (median of up to {MAX_SAMPLES}/cell) "
                 f"-- {fmap.refl_cells} cells, scale p2-p90 [{vmin:.0f},{vmax:.0f}]")
    fig.colorbar(im, ax=ax, label="raw counts (high = dark surface)")

    ax = axes[1]
    rgb = np.zeros((GRID_H, GRID_W, 3), dtype=float)
    if cseen.any():
        # ONE stretch shared by all three channels, never per-channel. The
        # colour sensor is ambient-only with no illumination LED, so absolute
        # values track room lighting and an unstretched image of a dim scene
        # is uniformly black -- hence the stretch. But stretching each channel
        # over its OWN range renormalises the channel balance, which IS the
        # hue: in the synthetic self-test a deliberately neutral grey
        # (120,120,115) came out vivid GREEN. A shared scale brightens the
        # image without inventing colour.
        vis = color[cseen]
        lo, hi = float(vis.min()), float(vis.max())
        if hi > lo:
            rgb = np.clip((color - lo) / (hi - lo), 0, 1)
    rgb[~cseen] = 0.15
    ax.imshow(rgb, origin="lower", extent=extent)
    ax.set_title(f"colour (per-channel contrast stretch) -- {fmap.color_cells} cells")

    for ax in axes:
        for name, (dx, dy) in DOTS.items():
            ax.plot(dx, dy, "o", mfc="none", mec="red", ms=12, mew=2)
            ax.annotate(name, (dx, dy), color="red", fontsize=8,
                        xytext=(4, 4), textcoords="offset points")
        # Camera-surveyed truth, so the map is checked and not just admired.
        for label, pts in MAT_TRUTH:
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            if len(pts) > 2:          # closed shape
                xs.append(pts[0][0])
                ys.append(pts[0][1])
            ax.plot(xs, ys, "-", color="cyan", lw=1.6, alpha=0.9)
        ax.plot([MAT_EXTENT["x"][0], MAT_EXTENT["x"][1], MAT_EXTENT["x"][1],
                 MAT_EXTENT["x"][0], MAT_EXTENT["x"][0]],
                [MAT_EXTENT["y"][0], MAT_EXTENT["y"][0], MAT_EXTENT["y"][1],
                 MAT_EXTENT["y"][1], MAT_EXTENT["y"][0]],
                "--", color="cyan", lw=1.0, alpha=0.6)
        ax.set_xlabel("x [cm]")
        ax.set_ylabel("y [cm]")

    fig.suptitle(title)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=110)
    print(f"wrote {path}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/cu.usbmodem214102",
                    help="relay dongle or direct robot port (auto-detected)")
    ap.add_argument("--robot-json", default="data/robots/tovez.json")
    ap.add_argument("--pitch", type=float, default=4.0,
                    help="[cm] y step between raster legs. NOTE the four line "
                         "sensors are POINTS at -32/-8/+8/+32mm, so a pass "
                         "traces four thin lines, NOT a filled 64mm swath -- "
                         "intra-pass gaps are 1.6-2.4cm whatever the pitch. "
                         "Coverage is inherently striped; the pitch only sets "
                         "how the stripes from adjacent passes interleave. "
                         "4cm is a reasonable resolution/driving-time trade "
                         "for features (dots, squares) of 5cm and up.")
    ap.add_argument("--speed", type=float, default=180.0, help="[mm/s]")
    ap.add_argument("--x-span", type=float, default=53.0, help="[cm] +/- from centre")
    ap.add_argument("--y-span", type=float, default=30.0, help="[cm] +/- from centre")
    ap.add_argument("--reseed-every", type=float, default=100.0,
                    help="[cm] of travel between camera re-fixes")
    ap.add_argument("--calibrate-dots", action="store_true",
                    help="drive to all four surveyed orange dots, solve the "
                         "parallax model (scale AND optical-axis offset), save "
                         "it, and exit. Run this before the first map, and "
                         "again whenever the tag mount or camera moves.")
    ap.add_argument("--calibrate-only", action="store_true",
                    help="single-point check: park on a dot and report the "
                         "radial ratio only (assumes the axis is over the "
                         "origin -- --calibrate-dots is the real one)")
    ap.add_argument("--calibration", default="field_parallax.json")
    ap.add_argument("--calibrate-box", nargs=2, type=float, metavar=("X", "Y"),
                    default=[35.0, 20.0],
                    help="[cm] half-extents of the calibration box. NOT the "
                         "surveyed dots by default: the camera cannot image "
                         "the whole field (two playfield corners project above "
                         "the frame), and tag 100 drops out near the edges. "
                         "The parallax fit is linear, so a smaller box that is "
                         "reliably seen fits it and extrapolates.")
    ap.add_argument("--replay", help="render an existing .npz and exit -- no robot needed")
    ap.add_argument("--out", default=None,
                    help="basename under output/ (default: field_map, or "
                         "sweep with --sweep)")
    ap.add_argument("--sweep", action="store_true",
                    help="edge sweep: every pass runs east->west at constant "
                         "y, stepping south, with only the RETURNS angled. "
                         "Even rows on one heading instead of a zigzag; "
                         "fenced by SWEEP_FENCE, not the raster's clamp")
    ap.add_argument("--rows", type=int, default=16,
                    help="[count] sweep passes")
    ap.add_argument("--y-start", type=float, default=30.0,
                    help="[cm] y of the sweep's first (northmost) pass")
    # Not wider, because a leg endpoint is NOT where the robot stops: this
    # file measures the Navigator overshooting the end of a transit by ~6.5cm
    # ("52.5 reached on a 46.0 target", NAV_OVERSHOOT = 7.0). 44 + 7 = 51
    # lands inside SWEEP_FENCE's 52; 46 would not, and the run would die on a
    # transit it had driven correctly.
    ap.add_argument("--x-east", type=float, default=44.0,
                    help="[cm] east end of every sweep pass -- where each "
                         "pass starts")
    ap.add_argument("--x-west", type=float, default=-44.0,
                    help="[cm] west end of every sweep pass")
    ap.add_argument("--straight", action="store_true",
                    help="scan by REVERSING instead of turning around at the "
                         "end of each leg -- no pivots, so no accumulated "
                         "heading error, no end-of-leg overshoot, and far "
                         "less time per leg")
    ap.add_argument("--resume", action="store_true",
                    help="load the existing .npz and ADD to it, instead of "
                         "starting empty -- a long raster is easily split "
                         "across runs, and a lost goto should not cost the "
                         "legs already driven")
    ap.add_argument("--y-min", type=float, default=0.0,
                    help="[cm] skip legs whose |y| is below this, to fill in "
                         "an outer band without re-driving mapped ground")
    args = ap.parse_args()

    # One output file per MODE, not per run. The sweep is meant to be repeated
    # -- re-run it and it OVERWRITES sweep.npz rather than littering output/
    # with a new file every attempt, which is what makes "the latest sweep"
    # unambiguous when someone goes looking for it later.
    if args.out is None:
        args.out = "sweep" if args.sweep else "field_map"

    if args.replay:
        fmap = FieldMap.load(pathlib.Path(args.replay))
        score_against_truth(fmap)
        render(fmap, OUT / f"{args.out}.png", f"field map (replay of {args.replay})")
        return 0

    perception = load_perception(pathlib.Path(args.robot_json))
    channel_y = perception["line_array"]["channel_y"]
    print(f"sensor geometry from {args.robot_json}:")
    print(f"  colour  body x={perception['color_sensor']['x']}mm y={perception['color_sensor']['y']}mm")
    print(f"  line    body x={perception['line_array']['x']}mm  channel_y={channel_y}mm")
    print("  channel order VERIFIED 2026-08-08: ch1 = leftmost, ch4 = rightmost")

    dc = connect_camera()
    cal_path = OUT / args.calibration

    if args.calibrate_only:
        factor, detail = calibrate_tag_height(dc)
        if factor is None:
            print(f"CALIBRATION FAILED: {detail}")
            return 2
        print(f"\nsingle-point ratio: {detail}")
        print("  NOTE: this assumes the camera's optical axis is exactly over "
              "the field origin. If it is not, one global divisor warps the map "
              "direction-dependently instead of shifting it. Use "
              "--calibrate-dots for the real fit.")
        return 0

    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    from goto_otos import FRAME_WORLD, Robot

    bot = Robot(args.port)

    if args.calibrate_dots:
        try:
            bx, by = args.calibrate_box
            pts = [("NW", (-bx, by)), ("NE", (bx, by)),
                   ("SE", (bx, -by)), ("SW", (-bx, -by))]
            print(f"calibration box: +/-{bx}cm x +/-{by}cm")
            cal = calibrate_from_dots(bot, dc, args.speed, cal_path, pts)
        finally:
            halt_verified(bot)
            bot.close()
        return 0 if cal else 2

    if not cal_path.exists():
        print(f"no parallax calibration at {cal_path} -- run --calibrate-dots first.\n"
              "Refusing to map with an uncalibrated elevated tag: at 12cm of tag "
              "height that is several cm of position error at the field edge, "
              "which is larger than the features being mapped.")
        bot.close()
        return 2
    cal = json.loads(cal_path.read_text())
    K, CX, CY = cal["k"], cal["cx"], cal["cy"]
    print(f"\nparallax calibration: k={K:.4f} axis=({CX:+.2f},{CY:+.2f})cm "
          f"residual={cal['residual_cm']:.2f}cm")

    def fix_corrected():
        raw = read_tag(dc)
        if raw is None:
            return None
        x, y = undo_parallax(raw[0], raw[1], K, CX, CY)
        return (x, y, raw[2])
    npz_path = OUT / f"{args.out}.npz"
    if args.resume and npz_path.exists():
        fmap = FieldMap.load(npz_path)
        print(f"resuming from {npz_path}: {fmap.refl_cells} reflectance cells, "
              f"{fmap.color_cells} colour cells")
    else:
        fmap = FieldMap()
    # Derive the fence from the raster extent, then clamp to the field. Setting
    # the two independently is how the first run tripped the fence on its very
    # first leg: a y-span of 30 against a fence at 30.65 left 6mm of slack, and
    # ordinary overshoot at the end of a leg ate it. The fence must always sit
    # OUTSIDE where the raster deliberately drives, but inside the rails.
    # 10cm of slack, not 6: the Navigator overshoots the end of a leg by a few
    # cm before it settles, and a 6cm margin tripped the fence at x=-54 on a
    # -48 target while still comfortably inside the mat.
    # The fence guards the RAILS, not the mat -- the mat is taped down and
    # driving past its edge is harmless (stakeholder, 2026-08-08). Keep a real
    # margin from the field limits and give the raster room for the
    # Navigator's end-of-transit overshoot.
    # Clamp the span so the Navigator's end-of-transit overshoot still lands
    # inside the rail fence. Sizing the scan and sizing the fence independently
    # is how a run dies on a transit it drove correctly: on 2026-08-08 an
    # x_span of 46 overshot to 52.5 against a 52.15 rail fence and ended the
    # run at leg 5, having driven every leg cleanly. The fence is not the thing
    # to relax here -- the scan is.
    # The sweep is deliberately NOT clamped by any of this. The clamp exists to
    # keep the raster's x_span/y_span inside RAIL_FENCE minus the Navigator's
    # overshoot; applied to the sweep it would pull x_east/x_west back in by
    # ~6cm at each end -- and the ends are the part of the mat this sweep
    # exists to reach. The sweep is fenced by SWEEP_FENCE instead, checked
    # inside every move by wait_for_goto and collect_straight.
    if args.sweep:
        fence = SWEEP_FENCE
        waypoints = []
        print(f"\nedge sweep: {args.rows} rows, pitch {args.pitch}cm, "
              f"x {args.x_east:+.1f} -> {args.x_west:+.1f}cm, "
              f"y {args.y_start:+.1f} -> "
              f"{args.y_start - (args.rows - 1) * args.pitch:+.1f}cm, "
              f"fence +/-{SWEEP_FENCE[0]:.1f}/{SWEEP_FENCE[1]:.1f}cm")
    else:
        max_x = RAIL_FENCE[0] - NAV_OVERSHOOT
        max_y = RAIL_FENCE[1] - NAV_OVERSHOOT
        if args.x_span > max_x or args.y_span > max_y:
            print(f"  clamping span to fit the rail fence: "
                  f"x {args.x_span:.1f}->{min(args.x_span, max_x):.1f}, "
                  f"y {args.y_span:.1f}->{min(args.y_span, max_y):.1f}cm "
                  f"(rail fence {RAIL_FENCE[0]:.1f}/{RAIL_FENCE[1]:.1f}, "
                  f"Navigator overshoot {NAV_OVERSHOOT:.1f})")
            args.x_span = min(args.x_span, max_x)
            args.y_span = min(args.y_span, max_y)

        fence = (min(X_LIM - 6.0, args.x_span + 14.0),
                 min(Y_LIM - 6.0, args.y_span + 14.0))
        waypoints = raster(args.x_span, args.y_span, args.pitch)
        # Start with the leg whose y is nearest the centre and work outward: the
        # robot begins AT the centre, so this keeps the first transit short instead
        # of sending it diagonally across the field to a far corner. Sort the LEGS
        # (waypoint PAIRS), never the waypoints individually -- that would scramble
        # each leg's start/end and turn the raster into random criss-crossing.
        legs = [waypoints[i:i + 2] for i in range(0, len(waypoints), 2)]
        legs = [leg for leg in legs if abs(leg[0][1]) >= args.y_min]
        legs.sort(key=lambda leg: abs(leg[0][1]))
        waypoints = [pt for leg in legs for pt in leg]
        print(f"\nraster: {len(waypoints)} waypoints, pitch {args.pitch}cm, "
              f"fence +/-{fence[0]:.1f}/{fence[1]:.1f}cm")

    travelled = 0.0
    last = None
    try:
        if not recover_to_centre(bot, dc, fix_corrected, args.speed):
            halt_verified(bot)
            bot.close()
            return 2

        if args.sweep:
            breach = edge_sweep(bot, fmap, perception, channel_y, fix_corrected,
                                args.x_east, args.x_west, args.y_start,
                                args.pitch, args.rows, args.speed)
            if breach:
                print(f"  GEOFENCE breach at ({breach[0]:+.1f},{breach[1]:+.1f})cm")
            halt_verified(bot)
            fmap.save(npz_path)
            print(f"\nwrote {npz_path}  ({fmap.refl_cells} reflectance cells, "
                  f"{fmap.color_cells} colour cells)")
            score_against_truth(fmap)
            render(fmap, OUT / f"{args.out}.png",
                   "playfield reflectance + colour map (edge sweep)")
            bot.close()
            return 0

        if args.straight:
            breach = straight_scan(bot, dc, fmap, perception, channel_y, fence,
                                   args.x_span, args.y_span, args.pitch,
                                   args.speed, fix_corrected, args.reseed_every)
            if breach:
                print(f"  GEOFENCE breach at ({breach[0]:+.1f},{breach[1]:+.1f})cm")
            halt_verified(bot)
            fmap.save(npz_path)
            print(f"\nwrote {npz_path}  ({fmap.refl_cells} reflectance cells, "
                  f"{fmap.color_cells} colour cells)")
            score_against_truth(fmap)
            render(fmap, OUT / f"{args.out}.png", "playfield reflectance + colour map")
            bot.close()
            return 0
        for i, (tx, ty) in enumerate(waypoints):
            fix = camera_fix(fix_corrected)
            if fix is None:
                print("  camera lost tag 100 -- halting")
                break
            if last is None or travelled >= args.reseed_every:
                if not seed_retry(bot, fix[0], fix[1], fix[2]):
                    print("  re-seed did not read back after retries -- halting")
                    break
                travelled = 0.0
                print(f"  [{i}] re-seeded at ({fix[0]:+.1f},{fix[1]:+.1f})cm")

            # Wait on the COMPLETION ACK while mapping, never a fixed dwell.
            # A blind collect window is wrong in both directions: it keeps
            # sampling a stationary robot after a short leg (piling thousands
            # of samples into a handful of cells), and it moves on before a
            # long leg has finished, so the next leg is issued from a pose the
            # robot has not reached. Both were observed on the first raster
            # attempt -- 3695 samples produced 5 cells, and the run tripped the
            # fence at a position two legs stale.
            _corr, goto_id = bot.goto_wire(tx * 10.0, ty * 10.0, FRAME_WORLD,
                                           speed=args.speed, timeout=40000.0)
            done, breach = wait_for_goto(bot, goto_id, timeout=45.0, fmap=fmap,
                                         perception=perception,
                                         channel_y=channel_y, fence=fence,
                                         arrived_at=(tx, ty))
            hits = fmap.refl_cells
            if not done and not breach:
                print(f"  [{i}] no completion ack in 45s -- halting")
                halt_verified(bot)
                break
            if breach:
                print(f"  GEOFENCE breach at ({breach[0]:+.1f},{breach[1]:+.1f})cm -- halting")
                halt_verified(bot)
                break
            if last is not None:
                travelled += math.hypot(tx - last[0], ty - last[1])
            last = (tx, ty)
            print(f"  [{i:2d}] -> ({tx:+6.1f},{ty:+6.1f})cm  "
                  f"refl_cells={fmap.refl_cells} colour_cells={fmap.color_cells}")
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        if not halt_verified(bot):
            print("WARNING: could not CONFIRM the robot stopped -- check it physically")
        bot.close()

    npz = npz_path
    fmap.save(npz)
    print(f"\nwrote {npz}  ({fmap.refl_cells} reflectance cells, {fmap.color_cells} colour cells)")
    score_against_truth(fmap)
    render(fmap, OUT / f"{args.out}.png", "playfield reflectance + colour map")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
