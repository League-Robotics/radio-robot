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
    zero write is permanent. Confirm the active flag actually drops.
    """
    for _ in range(tries):
        try:
            bot.p.estop()
        except Exception:
            pass
        time.sleep(0.25)
        frames = [getattr(e, "tlm", None) for e in bot.conn.drain_binary_tlm()]
        frames = [f for f in frames if f is not None]
        if frames and not (frames[-1].flags & 0x04):  # kFlagActive clear
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


def calibrate_from_dots(bot, dc, speed, out_path):
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
    for name in ("NW", "NE", "SE", "SW"):
        tx, ty = DOTS[name]
        fix = camera_fix(lambda: read_tag(dc))
        if fix is None:
            print(f"  {name}: camera lost tag 100")
            return None
        if not bot.seed(fix[0] * 10.0, fix[1] * 10.0, wrap(fix[2])):
            print(f"  {name}: re-seed did not read back")
            return None
        bot.goto_wire(tx * 10.0, ty * 10.0, FRAME_WORLD, speed=speed, timeout=40000.0)
        time.sleep(1.0)
        deadline = time.time() + 35.0
        while time.time() < deadline:
            frames = [getattr(e, "tlm", None) for e in bot.conn.drain_binary_tlm()]
            frames = [f for f in frames if f is not None]
            if frames and not (frames[-1].flags & 0x04):   # kFlagActive clear
                break
            time.sleep(0.1)
        halt_verified(bot)
        time.sleep(0.8)                                     # settle: fix AT REST
        fix = camera_fix(lambda: read_tag(dc), samples=11)
        if fix is None:
            print(f"  {name}: camera lost tag 100 at rest")
            return None
        obs.append(((tx, ty), (fix[0], fix[1])))
        print(f"  {name}: truth=({tx:+6.1f},{ty:+6.1f})  camera=({fix[0]:+6.1f},{fix[1]:+6.1f})")

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
    im = ax.imshow(refl, origin="lower", extent=extent, cmap="viridis",
                   vmin=refl.min() if refl.count() else 0,
                   vmax=refl.max() if refl.count() else 255)
    ax.set_title(f"reflectance (median of up to {MAX_SAMPLES}/cell) -- "
                 f"{fmap.refl_cells} cells")
    fig.colorbar(im, ax=ax, label="raw counts")

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
    ap.add_argument("--replay", help="render an existing .npz and exit -- no robot needed")
    ap.add_argument("--out", default="field_map")
    args = ap.parse_args()

    if args.replay:
        fmap = FieldMap.load(pathlib.Path(args.replay))
        render(fmap, OUT / f"{args.out}.png", f"field map (replay of {args.replay})")
        return 0

    perception = load_perception(pathlib.Path(args.robot_json))
    channel_y = perception["line_array"]["channel_y"]
    print(f"sensor geometry from {args.robot_json}:")
    print(f"  colour  body x={perception['color_sensor']['x']}mm y={perception['color_sensor']['y']}mm")
    print(f"  line    body x={perception['line_array']['x']}mm  channel_y={channel_y}mm")
    print("  NOTE: channel_y order is UNVERIFIED -- a reversed array mirrors the map")

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
            cal = calibrate_from_dots(bot, dc, args.speed, cal_path)
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
    fmap = FieldMap()
    fence = (X_LIM - FENCE_MARGIN, Y_LIM - FENCE_MARGIN)
    waypoints = raster(args.x_span, args.y_span, args.pitch)
    print(f"\nraster: {len(waypoints)} waypoints, pitch {args.pitch}cm, "
          f"fence +/-{fence[0]:.1f}/{fence[1]:.1f}cm")

    travelled = 0.0
    last = None
    try:
        for i, (tx, ty) in enumerate(waypoints):
            fix = camera_fix(fix_corrected)
            if fix is None:
                print("  camera lost tag 100 -- halting")
                break
            if last is None or travelled >= args.reseed_every:
                if not bot.seed(fix[0] * 10.0, fix[1] * 10.0, wrap(fix[2])):
                    print("  re-seed did not read back -- halting")
                    break
                travelled = 0.0
                print(f"  [{i}] re-seeded at ({fix[0]:+.1f},{fix[1]:+.1f})cm")

            bot.goto_wire(tx * 10.0, ty * 10.0, FRAME_WORLD,
                          speed=args.speed, timeout=40000.0)
            hits, breach = collect_during(bot, fmap, perception, channel_y,
                                          seconds=25.0, fence=fence)
            if breach:
                print(f"  GEOFENCE breach at ({breach[0]:+.1f},{breach[1]:+.1f})cm -- halting")
                halt_verified(bot)
                break
            if last is not None:
                travelled += math.hypot(tx - last[0], ty - last[1])
            last = (tx, ty)
            print(f"  [{i:2d}] -> ({tx:+6.1f},{ty:+6.1f})cm  +{hits:5d} samples  "
                  f"refl_cells={fmap.refl_cells} colour_cells={fmap.color_cells}")
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        if not halt_verified(bot):
            print("WARNING: could not CONFIRM the robot stopped -- check it physically")
        bot.close()

    npz = OUT / f"{args.out}.npz"
    fmap.save(npz)
    print(f"\nwrote {npz}  ({fmap.refl_cells} reflectance cells, {fmap.color_cells} colour cells)")
    render(fmap, OUT / f"{args.out}.png", "playfield reflectance + colour map")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
