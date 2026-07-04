#!/usr/bin/env python3
# PARKED (ticket 077-006, sprint 077 greenfield rebuild) — needs new-tree
# motion/odometry; reactivates once square runs, G/goto, OTOS, and camera
# sync return in a later sprint. Carried over verbatim from
# tests_old/bench/world_goto_chart.py: it drives via the old v1 G (go-to-XY)
# command and OV (OTOS world-pose sync), and reads pose=/otos= from the old
# v1 TLM frame — none of which the new source/ dev-loop firmware exposes
# (DEV M/DT STATE only; no go-to command, no OTOS, no world pose). It also
# imports the sibling `bench_safety.BenchRun` helper, which was not carried
# over (not in this ticket's locked tests/bench/ file list) — this file will
# fail to import until that dependency exists too. Do not attempt to make
# this runnable against the new tree until motion/odometry return.
"""world_goto_chart.py — drive to a playfield square and chart three world poses.

Demonstrates the "synchronise world location → command robot to a world XY"
feature and overlays the three independent estimates of where the robot is, so
they can be compared (they should coincide):

  1. CAMERA   — AprilTag 100 world_xy from the aprilcam daemon (ground truth)
  2. OTOS     — the firmware OTOS odometry pose (TLM pose frame)
  3. ENCODER  — a WPILib DifferentialDriveOdometry fed from wheel-encoder
                totals + OTOS heading (the encoder kinematic model)

Per cycle the tool:
  - reads the robot's camera pose and picks a target colored rectangle from
    playfield.json that is in a *different* quadrant than the robot (the robot's
    own-quadrant corner square and the two axis squares bordering that quadrant
    are excluded — five candidates remain; the farthest is chosen),
  - marks the robot start and the target on the aprilcam live view,
  - synchronises the robot's OTOS world pose to the camera (OV command),
  - sends the firmware G (go-to-XY) command and *monitors* (does not re-drive)
    while streaming encoder + OTOS telemetry and polling the camera,
  - draws all three growing trajectories on the aprilcam view, and
  - saves a matplotlib PNG of the run for inspection.

World frame: A1-centred, origin at AprilTag 1, +x east, +y north, centimetres.

Run modes:
    # Autonomous — run one (or N) cycles and dump a PNG per cycle:
    uv run python tests/bench/world_goto_chart.py --auto --cycles 1 --image out.png

    # Safe pipeline check — pick target + draw markers + planned path, NO motion:
    uv run python tests/bench/world_goto_chart.py --auto --plan-only

    # Interactive — SPACE runs one full cycle (new target each press), q quits:
    uv run python tests/bench/world_goto_chart.py
"""

from __future__ import annotations

import argparse
import json
import math
import os
import pathlib
import sys
import time
from pathlib import Path

_BENCH = pathlib.Path(__file__).resolve().parent
if str(_BENCH) not in sys.path:
    sys.path.insert(0, str(_BENCH))
from bench_safety import BenchRun  # noqa: E402

# --- A1-centred world-frame colours for the three estimate lines (RGB 0-255) --
COL_CAMERA = [60, 220, 90]     # green  — ground truth
COL_OTOS = [0, 190, 255]       # cyan   — firmware OTOS odometry
COL_ENCODER = [255, 150, 0]    # orange — WPILib encoder odometry
COL_ROBOT = [70, 130, 255]     # blue   — robot marker
COL_TARGET = [255, 220, 0]     # yellow — target marker
COL_PATH = [150, 150, 150]     # grey   — planned straight path

ROBOT_TAG_ID = 100


# ---------------------------------------------------------------------------
# Args
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=None, help="Serial port (auto-detect if omitted)")
    p.add_argument("--cam-index", type=int, default=1,
                   help="OS camera index to open in the daemon if none is open (default 1, Arducam)")
    p.add_argument("--playfield", default=None,
                   help="Explicit playfield JSON file (default: fetch from the aprilcam daemon)")
    p.add_argument("--playfield-name", default=None,
                   help="Named playfield to fetch from the daemon (default: the first one)")
    p.add_argument("--speed", type=int, default=120, help="Hop drive speed mm/s (default 120)")
    p.add_argument("--arrive", type=float, default=6.0,
                   help="Arrival tolerance cm (default 6)")
    p.add_argument("--timeout", type=float, default=40.0,
                   help="Host-side per-cycle drive timeout s (default 40)")
    p.add_argument("--cycles", type=int, default=1, help="Cycles to run in --auto mode")
    p.add_argument("--image", default="tests/bench/out/world_goto.png",
                   help="PNG output path (index suffix added for multiple cycles)")
    p.add_argument("--auto", action="store_true",
                   help="Run --cycles cycles non-interactively, dump PNG(s), then exit")
    p.add_argument("--plan-only", action="store_true",
                   help="Pick target + draw markers + planned path + PNG, but send NO motion")
    # --- safety: geofence + segmented driving ---
    p.add_argument("--margin", type=float, default=14.0,
                   help="Geofence inset (cm) inside the playfield ArUco-corner extent "
                        "— the robot tag is kept this far from the table edge (default 14)")
    p.add_argument("--hop", type=float, default=12.0,
                   help="Max distance (cm) of one re-checked drive hop (default 12)")
    p.add_argument("--nudge", type=float, default=5.0,
                   help="Pre-flight forward nudge (cm) to confirm direction (default 5)")
    p.add_argument("--cam-loss-stop", type=float, default=0.35,
                   help="STOP if the robot tag is unseen for this many seconds (default 0.35)")
    return p.parse_args()


# ---------------------------------------------------------------------------
# Playfield + target selection
# ---------------------------------------------------------------------------

def load_playfield(path: str) -> dict:
    with open(path) as f:
        return json.load(f)


def playfield_from_daemon(dc, name: str | None = None) -> dict:
    """Fetch the playfield map from the aprilcam daemon (no local file needed).

    Uses ``DaemonControl.list_playfields()`` and parses an entry's ``json_blob``,
    which has the same ``{playfield, aruco_tags, rectangles, ...}`` structure the
    old hardcoded ``playfield.json`` did. Picks the entry whose slug matches
    *name*, or the first one if *name* is None.
    """
    resp = dc.list_playfields()
    entries = list(resp.playfields)
    if not entries:
        raise RuntimeError(
            "aprilcam daemon has no playfield defined — add one to the daemon's "
            "playfields dir, or pass --playfield <file>")
    entry = entries[0]
    if name is not None:
        match = next((e for e in entries if e.name == name), None)
        if match is None:
            avail = ", ".join(e.name for e in entries) or "(none)"
            raise RuntimeError(f"playfield '{name}' not found on daemon; available: {avail}")
        entry = match
    return json.loads(entry.json_blob)


def _sign(v: float, eps: float = 1e-6) -> int:
    if v > eps:
        return 1
    if v < -eps:
        return -1
    return 0


def select_target(playfield: dict, robot_xy: tuple[float, float],
                  avoid_slug: str | None = None) -> dict:
    """Pick a target rectangle in a different quadrant than the robot.

    The robot's quadrant is (sign(x), sign(y)).  A rectangle is *excluded* when
    it lies in that quadrant or on either axis bordering it — i.e. each of its
    coordinate signs is zero or matches the robot's.  That removes the robot's
    own corner square plus the two axis squares on its quadrant's borders
    (three rectangles); the farthest of the remaining five is returned.
    """
    rx, ry = robot_xy
    sx, sy = _sign(rx), _sign(ry)
    # Robot exactly on an axis is rare; treat 0 as +1 so a quadrant is defined.
    sx = sx or 1
    sy = sy or 1

    rects = playfield.get("rectangles", [])

    def excluded(r: dict) -> bool:
        ex, ey = _sign(float(r["x"])), _sign(float(r["y"]))
        return (ex == 0 or ex == sx) and (ey == 0 or ey == sy)

    candidates = [r for r in rects if not excluded(r)]
    if avoid_slug is not None:
        filtered = [r for r in candidates if r["slug"] != avoid_slug]
        if filtered:
            candidates = filtered
    if not candidates:
        raise RuntimeError("no valid target rectangle in a different quadrant")

    # Farthest candidate → clearest cross-field drive.
    return max(candidates, key=lambda r: math.hypot(float(r["x"]) - rx,
                                                     float(r["y"]) - ry))


# ---------------------------------------------------------------------------
# Daemon (camera truth + live overlay)
# ---------------------------------------------------------------------------

def open_daemon(cam_index: int):
    """Connect to the aprilcam daemon and return (dc, cam_name)."""
    from aprilcam.config import Config
    from aprilcam.client.control import DaemonControl

    dc = DaemonControl.connect_default(Config.load())
    cams = dc.list_cameras()
    if cams:
        cam = cams[0]
    else:
        cam = dc.open_camera(index=cam_index)
    return dc, cam


def read_cam_pose(dc, cam, tag_id=ROBOT_TAG_ID, timeout_s=2.0):
    """Return (x_cm, y_cm, yaw_rad) for tag_id, or None if not seen calibrated."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        tf = dc.get_tags(cam)
        for t in tf.tags:
            if t.id == tag_id and t.world_xy is not None and t.yaw is not None:
                return float(t.world_xy[0]), float(t.world_xy[1]), float(t.yaw)
        time.sleep(0.03)
    return None


def _poly(points: list[tuple[float, float]], max_pts: int = 120) -> list[float]:
    """Flatten (x,y) points to [x0,y0,x1,y1,...], downsampled to <= max_pts."""
    if len(points) > max_pts:
        step = len(points) / max_pts
        points = [points[int(i * step)] for i in range(max_pts)]
    flat: list[float] = []
    for x, y in points:
        flat.extend([x, y])
    return flat


def publish_scene(dc, cam, *, robot, fwd, target_xy, target_label,
                  cam_pts, otos_pts, enc_pts, fence=None, ttl=0.6):
    """Push the full scene (geofence, markers, planned path, trajectories) to the view."""
    tx, ty = target_xy
    elems: list[dict] = []
    if fence is not None:
        xlo, xhi, ylo, yhi = fence
        # safe-drive geofence outline (robot must stay inside this)
        elems.append({"type": "rect", "params": [xlo, ylo, xhi, yhi],
                      "color": [200, 60, 60], "thickness": 2})
    elems += [
        # target square + label
        {"type": "rect", "params": [tx - 2.5, ty - 2.0, tx + 2.5, ty + 2.0],
         "color": COL_TARGET, "thickness": -1},
        {"type": "text", "params": [tx + 3.0, ty + 3.0], "text": target_label,
         "color": COL_TARGET},
    ]
    if cam_pts:
        sx, sy = cam_pts[0]
        # planned straight path start→target
        elems.append({"type": "polyline", "params": [sx, sy, tx, ty],
                      "color": COL_PATH, "thickness": 1})
    if len(cam_pts) >= 2:
        elems.append({"type": "polyline", "params": _poly(cam_pts),
                      "color": COL_CAMERA, "thickness": 3})
    if len(otos_pts) >= 2:
        elems.append({"type": "polyline", "params": _poly(otos_pts),
                      "color": COL_OTOS, "thickness": 2})
    if len(enc_pts) >= 2:
        elems.append({"type": "polyline", "params": _poly(enc_pts),
                      "color": COL_ENCODER, "thickness": 2})
    if robot is not None:
        rx, ry = robot
        ax, ay = rx + 12.0 * math.cos(fwd), ry + 12.0 * math.sin(fwd)
        elems.append({"type": "point", "params": [rx, ry, 4.0],
                      "color": COL_ROBOT, "thickness": -1})
        elems.append({"type": "arrow", "params": [rx, ry, ax, ay],
                      "color": [240, 240, 240], "thickness": 2})
    try:
        dc.publish_overlay(cam, elems, ttl=ttl)
    except Exception as exc:
        print(f"  (overlay publish failed: {exc})", file=sys.stderr)


# ---------------------------------------------------------------------------
# Robot connection
# ---------------------------------------------------------------------------

def connect_robot(args):
    """Connect via rogo's canonical path (auto-detects relay/direct mode).

    Returns (conn, proto, nezha).  Uses ``robot_radio.io.cli._make_robot`` so
    the radio relay is detected by the HELLO handshake and calibration is
    pushed — exactly as ``rogo`` does it.  A bare direct-mode open does NOT
    work for this radio robot (it sits behind a micro:bit relay).
    """
    from robot_radio.io.cli import _make_robot
    from robot_radio.robot.nezha import Nezha

    robot, conn, _result = _make_robot(args)
    if not isinstance(robot, Nezha):
        try:
            conn.disconnect()
        except Exception:
            pass
        raise RuntimeError("this tool requires a Nezha robot")
    proto = robot._proto
    print(f"  robot connected on {conn.port if hasattr(conn, 'port') else '?'} "
          f"(mode={getattr(conn, 'mode', '?')})")
    return conn, proto, robot


# ---------------------------------------------------------------------------
# Safety — geofence (from the calibrated playfield) + direction pre-flight
# ---------------------------------------------------------------------------

class _AbortDrive(Exception):
    """Raised to bail out of a drive cycle cleanly (motors stopped in finally)."""


def geofence_from_playfield(playfield: dict, margin_cm: float) -> tuple[float, float, float, float]:
    """Geofence (xlo, xhi, ylo, yhi) cm = ArUco-corner extent inset by margin.

    The table IS the calibrated playfield: its ArUco corner markers bound the
    drivable surface.  Insetting by ``margin_cm`` keeps the robot tag (hence the
    robot body) clear of the edge.  The robot must never leave this box.
    """
    xs = [float(u["x"]) for u in playfield.get("aruco_tags", [])]
    ys = [float(u["y"]) for u in playfield.get("aruco_tags", [])]
    if not xs or not ys:
        raise RuntimeError("playfield has no aruco_tags to derive a geofence from")
    return (min(xs) + margin_cm, max(xs) - margin_cm,
            min(ys) + margin_cm, max(ys) - margin_cm)


def in_fence(x: float, y: float, fence: tuple[float, float, float, float]) -> bool:
    xlo, xhi, ylo, yhi = fence
    return xlo <= x <= xhi and ylo <= y <= yhi


def _ang_err(a: float, b: float) -> float:
    """Absolute smallest angle between headings a and b (radians)."""
    return abs((a - b + math.pi) % (2 * math.pi) - math.pi)


def preflight_nudge(proto, dc, cam, fence, nudge_cm, pump, record):
    """Drive a small forward nudge and confirm the camera sees expected motion.

    Catches a wrong world→robot sign (which would otherwise send the robot
    across the table the wrong way) BEFORE any real hop.  Returns (ok, msg).
    The nudge is slow and short; it aborts immediately if it would leave the
    geofence.
    """
    p0 = read_cam_pose(dc, cam, timeout_s=1.0)
    if p0 is None:
        return False, "no camera fix before nudge"
    x0, y0, yaw0 = p0
    fwd0 = yaw0   # tag orientation IS the robot's forward heading (0=east, CCW+)

    # Refuse to nudge if the forward endpoint would leave the geofence (the robot
    # is facing the edge). Caller should reposition / re-face toward the centre.
    nx, ny = x0 + nudge_cm * math.cos(fwd0), y0 + nudge_cm * math.sin(fwd0)
    if not in_fence(nx, ny, fence):
        return False, ("robot faces the table edge (nudge would exit geofence) — "
                       "turn it toward the centre and retry")

    proto.go_to(int(round(nudge_cm * 10)), 0, 80)   # forward, slow (80 mm/s)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 3.0:
        evt = pump()
        cp = read_cam_pose(dc, cam, timeout_s=0.12)
        if cp is not None:
            record(cp[0], cp[1], cp[2])
            if not in_fence(cp[0], cp[1], fence):
                proto.stop()
                return False, "nudge would leave geofence — aborted"
        if evt in ("DONE", "SAFETY_STOP"):
            break
    proto.stop()
    time.sleep(0.2)

    p1 = read_cam_pose(dc, cam, timeout_s=1.0)
    if p1 is None:
        return False, "no camera fix after nudge"
    dx, dy = p1[0] - x0, p1[1] - y0
    moved = math.hypot(dx, dy)
    if moved < nudge_cm * 0.35:
        return False, f"robot barely moved ({moved:.1f}cm) — motors/relay?"
    err = math.degrees(_ang_err(math.atan2(dy, dx), fwd0))
    if err > 60.0:
        return False, (f"moved {err:.0f}° off its facing — world→robot sign "
                       f"looks wrong; refusing the cross-table drive")
    return True, f"moved {moved:.1f}cm, {err:.0f}° off facing — direction OK"


# ---------------------------------------------------------------------------
# One cycle
# ---------------------------------------------------------------------------

def run_cycle(args, dc, cam, proto, nezha, playfield, *, avoid_slug=None,
              image_path: str | None = None):
    """Execute one sync→drive→chart cycle. Returns the chosen target slug."""
    from robot_radio.robot.protocol import parse_tlm
    from wpimath.geometry import Pose2d, Rotation2d, Translation2d
    from wpimath.kinematics import DifferentialDriveOdometry

    geom = _robot_geometry()
    trackwidth = geom["trackwidth"]

    # 1. Where is the robot (camera truth)?
    pose = read_cam_pose(dc, cam, timeout_s=3.0)
    if pose is None:
        raise RuntimeError(f"camera did not see robot tag {ROBOT_TAG_ID} (calibrated)")
    cx, cy, cyaw = pose
    fwd = cyaw    # tag orientation IS the robot's forward heading (0=east, CCW+)

    # Geofence from the calibrated playfield — the robot must stay inside it.
    fence = geofence_from_playfield(playfield, args.margin)
    xlo, xhi, ylo, yhi = fence
    print(f"  geofence (table-safe): x∈[{xlo:+.0f},{xhi:+.0f}] y∈[{ylo:+.0f},{yhi:+.0f}]cm"
          f"  (={args.margin:.0f}cm inside the ArUco corners)")

    # 2. Pick a target in a different quadrant.
    target = select_target(playfield, (cx, cy), avoid_slug=avoid_slug)
    tx, ty = float(target["x"]), float(target["y"])
    label = f'{target["color"]} {target["cardinal"]}'
    dist0 = math.hypot(tx - cx, ty - cy)
    print(f"  robot=({cx:+.1f},{cy:+.1f})cm yaw={math.degrees(cyaw):+.0f}°  "
          f"→ target {target['slug']} ({tx:+.0f},{ty:+.0f})cm  dist={dist0:.1f}cm")

    cam_pts: list[tuple[float, float]] = [(cx, cy)]
    otos_pts: list[tuple[float, float]] = [(cx, cy)]
    enc_pts: list[tuple[float, float]] = [(cx, cy)]

    # Draw the initial scene (geofence + robot + target + planned path).
    publish_scene(dc, cam, robot=(cx, cy), fwd=fwd, target_xy=(tx, ty),
                  target_label=label, cam_pts=cam_pts, otos_pts=otos_pts,
                  enc_pts=enc_pts, fence=fence, ttl=30.0)

    if args.plan_only:
        print("  plan-only: no motion sent.")
        if image_path:
            render_png(image_path, playfield, (cx, cy), (tx, ty), label,
                       cam_pts, otos_pts, enc_pts, dist0, status="PLAN-ONLY",
                       fence=fence)
        return target["slug"]

    # Refuse to drive if the robot is already at/over the safe boundary, or if
    # the target somehow sits outside it (it never should — targets are interior).
    if not in_fence(cx, cy, fence):
        raise RuntimeError(
            f"robot at ({cx:+.1f},{cy:+.1f}) is outside the safe geofence — "
            "move it toward the centre before driving")
    if not in_fence(tx, ty, fence):
        raise RuntimeError(f"target {target['slug']} is outside the geofence — refusing")

    # 3. Synchronise OTOS world pose to camera (OV command).
    h_cdeg = int(round(math.degrees(cyaw) * 100.0))
    nezha.set_world_pose(int(round(cx * 10)), int(round(cy * 10)), h_cdeg)
    print(f"  sync: OV {round(cx*10)} {round(cy*10)} {h_cdeg}")

    # Clean serial + generous firmware watchdog, enable TLM streaming.
    try:
        proto.send("STOP", 200)
        proto.send("STREAM 0", 200)
        time.sleep(0.05)
    except Exception:
        pass
    try:
        proto.send("SET sTimeout=10000", 300)
    except Exception:
        pass
    proto.stream(50)   # ~20 Hz TLM

    # 4. WPILib encoder odometry, anchored to the camera start pose.
    odo = DifferentialDriveOdometry(
        Rotation2d(fwd), 0.0, 0.0,
        Pose2d(Translation2d(cx / 100.0, cy / 100.0), Rotation2d(fwd)),
    )
    enc_off: tuple[float, float] | None = None

    # 5. Segmented, vision-bounded drive.
    cur_otos: tuple[float, float] | None = None
    cur_enc: tuple[float, float] | None = None

    def pump_telemetry() -> str:
        """Drain serial; update OTOS + WPILib encoder odometry. Return EVT or ''."""
        nonlocal cur_otos, cur_enc, enc_off
        evt = ""
        for line in proto.read_lines(duration=20):
            if "EVT done G" in line:
                evt = "DONE"
            elif "EVT safety_stop" in line:
                evt = "SAFETY_STOP"
            tlm = parse_tlm(line)
            if tlm is None:
                continue
            if tlm.pose is not None:
                cur_otos = (tlm.pose[0] / 10.0, tlm.pose[1] / 10.0)
            if tlm.enc is not None and tlm.pose is not None:
                lt_m, rt_m = tlm.enc[0] / 1000.0, tlm.enc[1] / 1000.0
                if enc_off is None:
                    enc_off = (lt_m, rt_m)
                odo.update(Rotation2d(tlm.pose[2] / 18000.0 * math.pi),
                           lt_m - enc_off[0], rt_m - enc_off[1])
                tr = odo.getPose().translation()
                cur_enc = (tr.x * 100.0, tr.y * 100.0)
        return evt

    def record_and_draw(cxx: float, cyy: float, fwdd: float, ttl: float = 0.6) -> None:
        cam_pts.append((cxx, cyy))
        if cur_otos is not None:
            otos_pts.append(cur_otos)
        if cur_enc is not None:
            enc_pts.append(cur_enc)
        publish_scene(dc, cam, robot=(cxx, cyy), fwd=fwdd, target_xy=(tx, ty),
                      target_label=label, cam_pts=cam_pts, otos_pts=otos_pts,
                      enc_pts=enc_pts, fence=fence, ttl=ttl)

    status = "TIMEOUT"
    CAM_LOSS = args.cam_loss_stop
    HOP_TIMEOUT = max(2.0, (args.hop / max(args.speed, 1)) * 10.0 * 2.5 + 1.5)
    t_start = time.monotonic()

    try:
        # Pass the Nezha (not proto) — SafeRun's "has .stop() ⇒ Nezha" heuristic
        # now misfires on NezhaProtocol (which gained a .stop()), so it would try
        # nezha._proto on a proto. The Nezha is the intended arg (cf. smoke_ritual).
        with BenchRun(nezha, max_seconds=int(args.timeout) + 60):
            # 5a. Pre-flight: nudge forward and confirm direction before committing.
            ok, msg = preflight_nudge(proto, dc, cam, fence, args.nudge,
                                      pump_telemetry, record_and_draw)
            print(f"  nudge: {msg}")
            if not ok:
                status = "NUDGE_ABORT"
                raise _AbortDrive()

            # 5b. Short re-checked hops toward the target. The robot never commits
            #     to more than one hop; camera + geofence are checked throughout.
            while time.monotonic() - t_start < args.timeout:
                cp = read_cam_pose(dc, cam, timeout_s=CAM_LOSS)
                if cp is None:
                    status = "CAMERA_LOST"
                    break
                cx, cy, cyaw = cp
                fwd = cyaw
                pump_telemetry()
                record_and_draw(cx, cy, fwd)

                if not in_fence(cx, cy, fence):
                    status = "GEOFENCE"
                    break
                dist = math.hypot(tx - cx, ty - cy)
                if dist <= args.arrive:
                    status = "DONE"
                    break

                # Plan a hop toward the target, clamped to stay inside the fence.
                bearing = math.atan2(ty - cy, tx - cx)
                hop = min(args.hop, dist)
                hx, hy = cx + hop * math.cos(bearing), cy + hop * math.sin(bearing)
                while hop > 2.0 and not in_fence(hx, hy, fence):
                    hop -= 2.0
                    hx, hy = cx + hop * math.cos(bearing), cy + hop * math.sin(bearing)
                if hop <= 2.0:
                    status = "FENCE_LIMIT"
                    break

                dxw, dyw = (hx - cx) * 10.0, (hy - cy) * 10.0
                dxr = dxw * math.cos(fwd) + dyw * math.sin(fwd)
                dyr = -dxw * math.sin(fwd) + dyw * math.cos(fwd)
                proto.go_to(int(round(dxr)), int(round(dyr)), int(args.speed))

                # Watch this hop the whole time: geofence + camera-loss STOP.
                hop_t0 = time.monotonic()
                last_seen = time.monotonic()
                stop_all = False
                while time.monotonic() - hop_t0 < HOP_TIMEOUT:
                    evt = pump_telemetry()
                    cp = read_cam_pose(dc, cam, timeout_s=0.12)
                    if cp is None:
                        if time.monotonic() - last_seen > CAM_LOSS:
                            status = "CAMERA_LOST"
                            stop_all = True
                            break
                        continue
                    last_seen = time.monotonic()
                    cx, cy, cyaw = cp
                    fwd = cyaw
                    record_and_draw(cx, cy, fwd)
                    if not in_fence(cx, cy, fence):
                        status = "GEOFENCE"
                        stop_all = True
                        break
                    if math.hypot(tx - cx, ty - cy) <= args.arrive:
                        status = "DONE"
                        stop_all = True
                        break
                    if evt in ("DONE", "SAFETY_STOP"):
                        break   # hop's G finished — re-plan from the outer loop
                proto.stop()    # discrete, controlled stop between hops
                if stop_all:
                    break
    except _AbortDrive:
        pass
    finally:
        try:
            for _ in range(3):
                proto.stop()
                time.sleep(0.04)
            proto.stream(0)
        except Exception:
            pass

    final_dist = math.hypot(tx - cam_pts[-1][0], ty - cam_pts[-1][1])
    print(f"  result: {status}  final dist to target={final_dist:.1f}cm  "
          f"({'reached' if final_dist <= args.arrive else 'miss'})")

    # Leave the final scene persistently on the view.
    publish_scene(dc, cam, robot=cam_pts[-1], fwd=fwd, target_xy=(tx, ty),
                  target_label=label, cam_pts=cam_pts, otos_pts=otos_pts,
                  enc_pts=enc_pts, fence=fence, ttl=60.0)

    if image_path:
        render_png(image_path, playfield, (cam_pts[0]), (tx, ty), label,
                   cam_pts, otos_pts, enc_pts, final_dist, status=status,
                   fence=fence)
    return target["slug"]


def _robot_geometry() -> dict:
    """Read trackwidth (mm) from the active robot config, default 126."""
    try:
        from robot_radio.io.cli import get_robot_config
        cfg = get_robot_config()
        tw = getattr(getattr(cfg, "geometry", None), "trackwidth", None)
        if tw:
            return {"trackwidth": float(tw)}
    except Exception:
        pass
    return {"trackwidth": 126.0}


# ---------------------------------------------------------------------------
# PNG rendering (matplotlib, Agg)
# ---------------------------------------------------------------------------

def render_png(path, playfield, start_xy, target_xy, label,
               cam_pts, otos_pts, enc_pts, final_dist, status, fence=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    pf = playfield.get("playfield", {})
    half_w = float(pf.get("width_cm", 134.3)) / 2.0
    half_h = float(pf.get("height_cm", 89.3)) / 2.0

    fig, ax = plt.subplots(figsize=(11, 8))
    ax.set_facecolor("#101418")
    # Field outline + axes.
    ax.add_patch(Rectangle((-half_w, -half_h), 2 * half_w, 2 * half_h,
                           fill=False, edgecolor="#445", lw=1.5))
    ax.axhline(0, color="#334", lw=0.8)
    ax.axvline(0, color="#334", lw=0.8)

    # All colored rectangles for context.
    _named = {"purple": "#a050ff", "black": "#888", "orange": "#ff9a00",
              "red": "#ff3b30", "green": "#34c759", "magenta": "#ff2d95",
              "blue": "#0a84ff", "yellow": "#ffd60a"}
    for r in playfield.get("rectangles", []):
        c = _named.get(r["color"], "#aaa")
        ax.add_patch(Rectangle((float(r["x"]) - 2.5, float(r["y"]) - 2.0), 5, 4,
                               facecolor=c, edgecolor="white", lw=0.5, alpha=0.65))
    for u in playfield.get("aruco_tags", []):
        ax.plot(float(u["x"]), float(u["y"]), "s", color="#556", ms=4)

    # Safe-drive geofence (robot must stay inside this dashed box).
    if fence is not None:
        xlo, xhi, ylo, yhi = fence
        ax.add_patch(Rectangle((xlo, ylo), xhi - xlo, yhi - ylo, fill=False,
                               edgecolor="#d05050", lw=1.5, ls="--",
                               label="geofence (safe)"))

    def line(pts, color, label_):
        if len(pts) >= 1:
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            ax.plot(xs, ys, "-o", color=color, ms=2.5, lw=1.8, label=label_)

    line(cam_pts, "#3cdc5a", "camera (truth)")
    line(otos_pts, "#00becf", "OTOS odometry")
    line(enc_pts, "#ff9600", "encoder odometry (WPILib)")

    sx, sy = start_xy
    tx, ty = target_xy
    ax.plot(sx, sy, "o", color="#4080ff", ms=12, label="robot start")
    ax.plot(tx, ty, "*", color="#ffdc00", ms=20, label=f"target: {label}")

    ax.set_aspect("equal")
    ax.set_xlim(-half_w - 6, half_w + 6)
    ax.set_ylim(-half_h - 6, half_h + 6)
    ax.set_xlabel("x (cm, +east)")
    ax.set_ylabel("y (cm, +north)")
    ax.set_title(f"world-goto — {status} — final dist to target {final_dist:.1f} cm",
                 color="white")
    ax.legend(loc="upper right", fontsize=8, facecolor="#202428", labelcolor="white")
    ax.tick_params(colors="#aaa")
    fig.tight_layout()
    fig.savefig(path, dpi=120, facecolor="#101418")
    plt.close(fig)
    print(f"  saved {path}")


# ---------------------------------------------------------------------------
# Spacebar reader (interactive mode)
# ---------------------------------------------------------------------------

def _getch() -> str:
    import termios
    import tty
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def _image_for(base: str, idx: int, total: int) -> str:
    if total <= 1:
        return base
    stem, ext = os.path.splitext(base)
    return f"{stem}_{idx + 1:02d}{ext}"


def main() -> int:
    args = _parse_args()

    print("Connecting to aprilcam daemon …")
    dc, cam = open_daemon(args.cam_index)
    print(f"  camera: {cam}")

    # Playfield map: explicit file if --playfield is given, else fetched from the
    # aprilcam daemon (list_playfields) — no hardcoded file dependency.
    if args.playfield:
        playfield = load_playfield(args.playfield)
        print(f"  playfield: {args.playfield} (file)")
    else:
        playfield = playfield_from_daemon(dc, args.playfield_name)
        pf_name = playfield.get("display_name") or playfield.get("name") or "?"
        print(f"  playfield: '{pf_name}' (from daemon)")

    conn = proto = nezha = None
    if not args.plan_only:
        print("Connecting to robot …")
        conn, proto, nezha = connect_robot(args)

    try:
        if args.auto or args.plan_only:
            avoid = None
            for i in range(max(1, args.cycles)):
                print(f"\n=== cycle {i + 1}/{args.cycles} ===")
                img = _image_for(args.image, i, args.cycles)
                avoid = run_cycle(args, dc, cam, proto, nezha, playfield,
                                  avoid_slug=avoid, image_path=img)
        else:
            print("\nInteractive — SPACE = run one cycle, q = quit.")
            avoid = None
            n = 0
            while True:
                sys.stdout.write("\n[SPACE]=go  [q]=quit > ")
                sys.stdout.flush()
                ch = _getch()
                if ch in ("q", "\x03", "\x04"):
                    print("quit")
                    break
                if ch != " ":
                    continue
                n += 1
                print(f"\n=== cycle {n} ===")
                img = _image_for(args.image, n - 1, 999)
                try:
                    avoid = run_cycle(args, dc, cam, proto, nezha, playfield,
                                      avoid_slug=avoid, image_path=img)
                except Exception as exc:
                    print(f"  cycle error: {exc}")
    finally:
        if proto is not None:
            try:
                for _ in range(3):
                    proto.stop()
                    time.sleep(0.04)
                proto.stream(0)
            except Exception:
                pass
        if conn is not None:
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
