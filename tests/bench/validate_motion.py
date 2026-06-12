#!/usr/bin/env python3
"""validate_motion.py — methodical, camera-verified validation of robot motion.

Builds up capability step by step, measuring everything against the overhead
camera (AprilTag 100) as ground truth and printing commanded-vs-measured.

Steps (run one with --step, or several comma-separated):
  turn   : relative turn accuracy — command N°, measure actual rotation
  turnto : turn to an ABSOLUTE heading (read heading, turn, re-check)
  circle : orient to north, then step through headings around the circle
  fwd    : drive forward a defined distance, measure actual distance
  outback: drive out, turn around, drive back; check return
  goto   : go to specific board XY (turn-to-face + drive)

World frame: A1-centred, +x east, +y north, cm. We work in MATH heading
(atan2, 0°=east/+x, CCW positive) internally and also print a compass heading
(0°=north/+y, clockwise) for the human.

  uv run python tests/bench/validate_motion.py --step turn
"""
from __future__ import annotations

import argparse
import math
import pathlib
import sys
import time

_BENCH = pathlib.Path(__file__).resolve().parent
if str(_BENCH) not in sys.path:
    sys.path.insert(0, str(_BENCH))
from bench_safety import BenchRun  # noqa: E402

ROBOT_TAG_ID = 100


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def compass(math_heading_rad: float) -> float:
    """math heading (0=E, CCW+) -> compass degrees (0=N, CW)."""
    return (90.0 - math.degrees(math_heading_rad)) % 360.0


# --- camera ---------------------------------------------------------------
def open_daemon(cam_index: int):
    from aprilcam.config import Config
    from aprilcam.client.control import DaemonControl
    dc = DaemonControl.connect_default(Config.load())
    cams = dc.list_cameras()
    cam = cams[0] if cams else dc.open_camera(index=cam_index)
    return dc, cam


def read_pose(dc, cam, timeout_s=1.5):
    """(x_cm, y_cm, yaw_rad) for the robot tag, or None."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        for t in dc.get_tags(cam).tags:
            if t.id == ROBOT_TAG_ID and t.world_xy is not None and t.yaw is not None:
                return float(t.world_xy[0]), float(t.world_xy[1]), float(t.yaw)
        time.sleep(0.02)
    return None


# --- robot ----------------------------------------------------------------
def open_robot(port):
    from robot_radio.io.cli import _make_robot

    class _A:
        pass
    a = _A()
    a.port = port
    a.verbose = False
    robot, conn, _ = _make_robot(a)
    return robot, conn, robot._proto


# --- a turn that we MEASURE with the camera while it happens ----------------
def measured_turn(dc, cam, proto, deg: float, settle=0.4, timeout=14.0):
    """Send RT(deg); sample the camera throughout; return signed measured° (yaw).

    Rotation is accumulated from per-sample yaw deltas (unwrap-safe for any
    total angle). Returns (measured_deg, start_yaw, end_yaw).
    """
    p = read_pose(dc, cam, 1.5)
    if p is None:
        raise RuntimeError("no camera fix before turn")
    prev = p[2]
    start = prev
    acc = 0.0
    proto.send(f"RT {int(round(deg * 100))} #1", 200)
    done = False
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        q = read_pose(dc, cam, 0.05)
        if q is not None:
            acc += wrap(q[2] - prev)
            prev = q[2]
        for ln in proto.read_lines(duration_ms=25):
            if "done" in ln and "RT" in ln:
                done = True
        if done:
            break
    # settle + final sample
    t1 = time.monotonic()
    while time.monotonic() - t1 < settle:
        q = read_pose(dc, cam, 0.05)
        if q is not None:
            acc += wrap(q[2] - prev)
            prev = q[2]
    for _ in range(3):
        proto.stop()
        time.sleep(0.03)
    return math.degrees(acc), start, prev


def rt_turn(proto, deg, timeout=14.0):
    """Blocking RT relative turn (RT+ = CCW)."""
    proto.send(f"RT {int(round(deg * 100))} #1", 200)
    proto.wait_for_evt_done("RT", timeout_ms=int(timeout * 1000), corr_id="1")
    time.sleep(0.3)
    for _ in range(2):
        proto.stop()
        time.sleep(0.03)


def crawl_turn(proto, deg, omega_dps=25.0):
    """One short, NO-RAMP open-loop spin pulse (~deg°) via _VW, then stop.

    Bypasses the RT coast-anticipation (which can't do small angles) and the
    accel ramp, so it 'shocks' the heading a small amount without hunting.
    Caller should under-correct (pass a fraction of the error) and re-measure."""
    if abs(deg) < 0.3:
        return
    omega_mrad = int(round(math.copysign(omega_dps, deg) * math.pi / 180.0 * 1000.0))
    dur = max(0.06, min(0.5, abs(deg) / omega_dps))
    proto.send(f"_VW 0 {omega_mrad}", 80)
    time.sleep(dur)
    for _ in range(2):
        proto.stop()
        time.sleep(0.03)
    time.sleep(0.18)


def calibrate_forward(dc, cam, proto, speed=85, min_cm=6.0):
    """Measure the FIXED offset between the tag's yaw and world-forward, with a
    real drive (NOT a tiny nudge). forward_world = wrap(offset - cyaw); the tag
    yaw is CW-positive while world-forward is CCW-positive (reflection+offset).

    Retries with a longer drive until it gets >= min_cm of clean displacement,
    so a jammed/edge 1.8cm reading can never produce a garbage heading."""
    for attempt in range(4):
        p0 = read_pose(dc, cam, 1.5)
        if p0 is None:
            raise RuntimeError("no fix before calibration drive")
        x0, y0, _ = p0
        secs = 0.7 + 0.5 * attempt
        t = time.monotonic()
        while time.monotonic() - t < secs:
            proto.drive(speed, speed)
            time.sleep(0.1)
        for _ in range(3):
            proto.stop()
            time.sleep(0.03)
        time.sleep(0.25)
        p1 = read_pose(dc, cam, 1.5)
        if p1 is None:
            raise RuntimeError("no fix after calibration drive")
        x1, y1, c1 = p1
        moved = math.hypot(x1 - x0, y1 - y0)
        if moved >= min_cm:
            fwd_world = math.atan2(y1 - y0, x1 - x0)   # true forward heading (math)
            offset = wrap(fwd_world - c1)              # ADDITIVE: forward = cyaw + offset
            print(f"  heading-calibrated: forward={math.degrees(fwd_world):+.0f}° "
                  f"(compass {compass(fwd_world):.0f}°) from a {moved:.1f}cm drive; "
                  f"offset={math.degrees(offset):+.0f}° (expect ≈ +90° if tag-top=forward)")
            return offset
        print(f"  calibration drive only {moved:.1f}cm (attempt {attempt + 1}) "
              f"— too short to read heading; retrying longer")
    raise RuntimeError("could not get a clean calibration drive (robot jammed at an edge?)")


def forward_world(offset, cyaw):
    # ADDITIVE: tag yaw is CCW-positive (same as world). forward heading =
    # cyaw + offset, where offset = θ_mount + π/2 (≈ +90° if the tag's top
    # points along the robot's forward). (The old `offset - cyaw` had the sign
    # of cyaw flipped, which sent go-to-point off in the wrong direction.)
    return wrap(cyaw + offset)


def heading_now(dc, cam, offset, n=4):
    """Median-vector forward heading from n camera reads (rejects yaw noise)."""
    mx = my = 0.0
    got = 0
    for _ in range(n):
        p = read_pose(dc, cam, 0.4)
        if p is not None:
            f = forward_world(offset, p[2])
            mx += math.cos(f)
            my += math.sin(f)
            got += 1
        time.sleep(0.02)
    return math.atan2(my, mx) if got else None


def turnto(dc, cam, proto, offset, target_world, tol_deg=3.0, iters=12):
    """Closed-loop turn to an absolute heading. First PROBES the RT sign with a
    small in-place turn (so a wrong sign can never run the robot away), then
    converges: RT for the bulk, under-correcting crawl pulses for the fine
    approach. Returns (final_forward_world, err_deg)."""
    f0 = heading_now(dc, cam, offset)
    if f0 is None:
        return 0.0, 999.0
    if abs(math.degrees(wrap(target_world - f0))) <= tol_deg:
        return f0, math.degrees(wrap(target_world - f0))

    # Probe: a small in-place turn; see which way the measured heading moved.
    crawl_turn(proto, 8.0)
    f1 = heading_now(dc, cam, offset)
    if f1 is None:
        f1 = f0
    s = 1.0 if wrap(f1 - f0) >= 0.0 else -1.0     # RT(+) moves heading by sign s

    for _ in range(iters):
        fwd = heading_now(dc, cam, offset)
        if fwd is None:
            continue
        e_deg = math.degrees(wrap(target_world - fwd))
        if abs(e_deg) <= tol_deg:
            f2 = heading_now(dc, cam, offset)     # confirm on a fresh read
            if f2 is not None and abs(math.degrees(wrap(target_world - f2))) <= tol_deg:
                return f2, math.degrees(wrap(target_world - f2))
            continue
        cmd = s * e_deg                           # RT command that moves heading toward target
        if abs(cmd) > 20.0:
            rt_turn(proto, cmd)
        else:
            crawl_turn(proto, math.copysign(max(2.5, abs(cmd) * 0.6), cmd))
    fwd = heading_now(dc, cam, offset) or 0.0
    return fwd, math.degrees(wrap(target_world - fwd))


def step_turnto(dc, cam, proto, args):
    print("\n=== STEP 2: localize + closed-loop turn to ABSOLUTE heading ===")
    offset = calibrate_forward(dc, cam, proto)
    # targets in compass terms the human asked for: N, E, S, W
    targets = [("north", math.radians(90)), ("east", math.radians(0)),
               ("south", math.radians(-90)), ("west", math.radians(180))]
    print(f"  {'target':>7} {'final compass°':>15} {'err°':>7}")
    errs = []
    for name, tw in targets:
        fwd, err = turnto(dc, cam, proto, offset, tw, tol_deg=args.tol)
        errs.append(abs(err))
        print(f"  {name:>7} {compass(fwd):15.0f} {err:+7.1f}")
        time.sleep(0.3)
    print(f"  -> mean |err| = {sum(errs)/len(errs):.1f}°  max |err| = {max(errs):.1f}°  "
          f"(tol {args.tol}°)")


def field_center(dc, cam, timeout_s=2.0):
    """Camera-frame world_xy of the A1 centre anchor (AprilTag id 1).
    In the script's camera source this is ~(0,0) (the frame IS A1-centred)."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        for t in dc.get_tags(cam).tags:
            if t.id == 1 and t.world_xy is not None:
                return float(t.world_xy[0]), float(t.world_xy[1])
        time.sleep(0.03)
    return None


def geofence(margin_cm=10.0):
    # A1-centred; ArUco corners are at ±67 x / ±44.65 y. Inset by margin so the
    # robot stays clear of the edges (squares at ±35/±24 are well inside).
    return (-67 + margin_cm, 67 - margin_cm, -44.65 + margin_cm, 44.65 - margin_cm)


def in_fence(x, y, f):
    return f[0] <= x <= f[1] and f[2] <= y <= f[3]


def drive_forward(dc, cam, proto, fence, target_cm, vmax=140, vmin=55, timeout=20.0):
    """Closed-loop straight drive; stop at target_cm of displacement (camera).
    Returns (status, measured_cm, end_xy)."""
    p0 = read_pose(dc, cam, 1.5)
    if p0 is None:
        return "NOFIX", 0.0, None
    x0, y0, _ = p0
    last = time.monotonic()
    t0 = last
    x, y = x0, y0
    while time.monotonic() - t0 < timeout:
        p = read_pose(dc, cam, 0.1)
        now = time.monotonic()
        if p is None:
            for _ in range(2):
                proto.stop()
            if now - last > 0.5:
                return "CAMLOST", math.hypot(x - x0, y - y0), (x, y)
            continue
        last = now
        x, y, _ = p
        d = math.hypot(x - x0, y - y0)
        if fence is not None and not in_fence(x, y, fence):
            for _ in range(3):
                proto.stop()
            return "GEOFENCE", d, (x, y)
        rem = target_cm - d
        if rem <= 0.5:
            break
        v = vmax if rem > 8.0 else max(vmin, vmax * rem / 8.0)
        proto.drive(int(v), int(v))
        time.sleep(0.08)
    for _ in range(3):
        proto.stop()
        time.sleep(0.04)
    time.sleep(0.3)
    p = read_pose(dc, cam, 1.0)
    if p is not None:
        x, y, _ = p
    return "OK", math.hypot(x - x0, y - y0), (x, y)


def step_fwd(dc, cam, proto, args):
    print("\n=== STEP 4: drive a defined distance forward (camera-measured) ===")
    center = field_center(dc, cam)
    fence = geofence(center) if center else (-40, 40, -34, 34)
    offset = calibrate_forward(dc, cam, proto)
    print(f"  {'cmd cm':>7} {'measured cm':>12} {'err cm':>7} {'status':>9}")
    errs = []
    for target in [float(x) for x in args.dists.split(",")]:
        # aim toward whichever fence wall has the MOST room, and cap the
        # distance so the endpoint stays inside the fence.
        p = read_pose(dc, cam, 1.5)
        rx, ry, _ = p
        cands = [(0.0, fence[1] - rx), (math.pi, rx - fence[0]),
                 (math.pi / 2, fence[3] - ry), (-math.pi / 2, ry - fence[2])]
        heading, room = max(cands, key=lambda c: c[1])
        d_cmd = min(target, room - 5.0)
        turnto(dc, cam, proto, offset, heading, tol_deg=args.tol)
        st, meas, _ = drive_forward(dc, cam, proto, fence, d_cmd)
        err = meas - d_cmd
        if st == "OK":
            errs.append(abs(err))
        note = "" if abs(d_cmd - target) < 0.5 else f" (capped from {target:.0f})"
        print(f"  {d_cmd:7.1f} {meas:12.1f} {err:+7.1f} {st:>9}{note}")
        time.sleep(0.3)
    if errs:
        print(f"  -> mean |err| = {sum(errs)/len(errs):.1f}cm")


def goto_xy(dc, cam, proto, offset, fence, tx, ty, arrive=4.0, hop=14.0, timeout=45.0):
    """Drive to world (tx,ty) by re-planning: face the target, drive a short
    straight hop, re-check. Built on the validated turnto + drive_forward.
    Returns (status, (x,y), dist_cm)."""
    t0 = time.monotonic()
    rx = ry = 0.0
    min_d = float("inf")
    while time.monotonic() - t0 < timeout:
        p = read_pose(dc, cam, 1.0)
        if p is None:
            for _ in range(2):
                proto.stop()
            continue
        rx, ry, _ = p
        d = math.hypot(tx - rx, ty - ry)
        print(f"    world (camera): ({rx:+.1f},{ry:+.1f}) cm  {d:.1f}cm to target")
        if d <= arrive:
            return "OK", (rx, ry), d
        # divergence guard: if we're getting notably FARTHER from the target,
        # the heading is wrong — stop instead of driving away.
        min_d = min(min_d, d)
        if d > min_d + 10.0:
            for _ in range(2):
                proto.stop()
            return "DIVERGING", (rx, ry), d
        turnto(dc, cam, proto, offset, math.atan2(ty - ry, tx - rx), tol_deg=4.0)

        # ENDPOINT GUARD — read the robot's ACTUAL heading from the tag, confirm
        # forward points at the target, and project where a hop lands. Never
        # drive a heading that runs off the field.
        q = read_pose(dc, cam, 1.0)
        if q is None:
            continue
        rx, ry, cyaw = q
        fwd = forward_world(offset, cyaw)
        bearing = math.atan2(ty - ry, tx - rx)
        facing_err = abs(math.degrees(wrap(bearing - fwd)))
        if facing_err > 35.0:
            print(f"    tag heading is {facing_err:.0f}° off the target — re-turning, NOT driving")
            continue
        hop_cm = min(hop, max(0.0, d - arrive * 0.5))
        if fence is not None:
            ex, ey = rx + hop_cm * math.cos(fwd), ry + hop_cm * math.sin(fwd)
            while hop_cm > 2.0 and not in_fence(ex, ey, fence):
                hop_cm -= 2.0
                ex, ey = rx + hop_cm * math.cos(fwd), ry + hop_cm * math.sin(fwd)
            if hop_cm <= 2.0:
                print(f"    forward ({math.degrees(fwd):+.0f}°) would leave the field — stopping")
                return "FENCE_LIMIT", (rx, ry), d
        st, _, _ = drive_forward(dc, cam, proto, fence, hop_cm, vmax=130)
        if st in ("GEOFENCE", "CAMLOST", "NOFIX"):
            return st, (rx, ry), d
    return "TIMEOUT", (rx, ry), math.hypot(tx - rx, ty - ry)


def recenter(dc, cam, proto, offset, fence, center):
    # Drive to the A1 centre anchor (the true middle of the field).
    print(f"  recentering to A1 ({center[0]:+.0f},{center[1]:+.0f})…")
    st, xy, d = goto_xy(dc, cam, proto, offset, fence, center[0], center[1], arrive=8.0)
    print(f"    -> at ({xy[0]:+.0f},{xy[1]:+.0f}) cm, {d:.0f}cm from centre [{st}]")


def step_outback(dc, cam, proto, args):
    print("\n=== STEP 5: drive out, turn around, drive back ===")
    center = field_center(dc, cam)
    if center is None:
        print("  no A1 centre tag visible — skipping")
        return
    fence = geofence(center)
    offset = calibrate_forward(dc, cam, proto)
    recenter(dc, cam, proto, offset, fence, center)
    p = read_pose(dc, cam, 1.5)
    sx, sy, _ = p
    # out-point ~30cm along whichever axis has room
    cands = [(sx + 30, sy), (sx - 30, sy), (sx, sy + 25), (sx, sy - 25)]
    out = max(cands, key=lambda c: min(fence[1] - c[0], c[0] - fence[0],
                                       fence[3] - c[1], c[1] - fence[2]))
    print(f"  start ({sx:+.0f},{sy:+.0f}) -> out ({out[0]:+.0f},{out[1]:+.0f})")
    st, a, _ = goto_xy(dc, cam, proto, offset, fence, out[0], out[1], arrive=4.0)
    print(f"  reached out ({a[0]:+.0f},{a[1]:+.0f}) [{st}], "
          f"{math.hypot(out[0]-a[0], out[1]-a[1]):.1f}cm from out-target")
    st, b, _ = goto_xy(dc, cam, proto, offset, fence, sx, sy, arrive=4.0)
    ret_err = math.hypot(sx - b[0], sy - b[1])
    print(f"  returned to ({b[0]:+.0f},{b[1]:+.0f}) [{st}], "
          f"{ret_err:.1f}cm from the start point")
    print(f"  -> round-trip return error = {ret_err:.1f}cm")


def step_goto(dc, cam, proto, args):
    print("\n=== STEP 6: go to specific board positions ===")
    center = field_center(dc, cam)
    if center is None:
        print("  no A1 centre tag visible — skipping")
        return
    cx, cy = center
    fence = geofence(center)
    offset = calibrate_forward(dc, cam, proto)
    # colored squares are A1-centred in playfield.json; camera target = A1 + offset
    squares = [("A1 centre", 0, 0), ("east red", 35, 0), ("north black", 0, 24),
               ("southwest blue", -35, -24)]
    print(f"  {'target':>16} {'A1 offset':>10} {'reached':>12} {'err cm':>7} {'status':>9}")
    errs = []
    for name, ox, oy in squares:
        tx, ty = cx + ox, cy + oy
        st, xy, d = goto_xy(dc, cam, proto, offset, fence, tx, ty, arrive=4.0)
        if st == "OK":
            errs.append(d)
        print(f"  {name:>16} {f'({ox:+d},{oy:+d})':>10} {f'({xy[0]:+.0f},{xy[1]:+.0f})':>12} "
              f"{d:7.1f} {st:>9}")
        time.sleep(0.3)
    if errs:
        print(f"  -> mean arrival error = {sum(errs)/len(errs):.1f}cm")


def step_center(dc, cam, proto, args):
    print("\n=== go to CENTRE (0,0) ===")
    offset = calibrate_forward(dc, cam, proto)
    # fence=None: driving toward the centre is always inward/safe, even if the
    # robot is currently sitting just outside the fence at an edge.
    st, xy, d = goto_xy(dc, cam, proto, offset, None, 0.0, 0.0, arrive=args.arrive)
    print(f"  at ({xy[0]:+.1f},{xy[1]:+.1f}) cm — {d:.1f}cm from centre [{st}]")


def step_roam(dc, cam, proto, args):
    print("\n=== STEP 7: roam the whole board — predict, go, check ===")
    offset = calibrate_forward(dc, cam, proto)
    fence = geofence()
    # A1-centred board squares, ordered to criss-cross the whole field.
    order = [("SW blue", -35, -24), ("NE orange", 35, 24), ("SE green", 35, -24),
             ("NW purple", -35, 24), ("E red", 35, 0), ("W red", -35, 0),
             ("N black", 0, 24), ("S magenta", 0, -24), ("centre A1", 0, 0)]
    errs = []
    for name, tx, ty in order:
        p = read_pose(dc, cam, 1.5)
        if p is None:
            print("  lost the tag — stopping roam")
            break
        rx, ry, _ = p
        print(f"\n  PREDICT: from ({rx:+.0f},{ry:+.0f}) drive to {name} "
              f"({tx:+d},{ty:+d}) — expect to land within {args.arrive:.0f}cm")
        st, xy, d = goto_xy(dc, cam, proto, offset, fence, float(tx), float(ty),
                            arrive=args.arrive)
        otos = proto.otos_get_position()
        otos_s = f"({otos[0]},{otos[1]})mm h={otos[2]}" if otos else "n/a"
        if st == "OK":
            errs.append(d)
        print(f"  RESULT:  reached ({xy[0]:+.0f},{xy[1]:+.0f}) → {d:.1f}cm from target "
              f"[{st}]   onboard OTOS drift: {otos_s}")
        time.sleep(0.3)
    if errs:
        print(f"\n  -> roam arrival: mean {sum(errs)/len(errs):.1f}cm  "
              f"max {max(errs):.1f}cm  over {len(errs)} legs")


def step_circle(dc, cam, proto, args):
    print("\n=== STEP 3: orient to north, then step around the circle ===")
    offset = calibrate_forward(dc, cam, proto)
    print("  orienting to north (compass 0°)…")
    turnto(dc, cam, proto, offset, math.radians(90), tol_deg=args.tol)
    headings = [0, 45, 90, 135, 180, 225, 270, 315, 0]
    print(f"  {'go to compass°':>15} {'measured compass°':>18} {'err°':>7}")
    errs = []
    for h in headings:
        target_world = wrap(math.radians(90.0 - h))   # compass→math heading
        fwd, err = turnto(dc, cam, proto, offset, target_world, tol_deg=args.tol)
        errs.append(abs(err))
        print(f"  {h:15d} {compass(fwd):18.0f} {err:+7.1f}")
        time.sleep(0.25)
    print(f"  -> mean |err| = {sum(errs)/len(errs):.1f}°  max |err| = {max(errs):.1f}°")


def step_turn(dc, cam, proto, args):
    print("\n=== STEP 1: relative turn accuracy (camera-measured) ===")
    angles = [float(x) for x in args.angles.split(",")]
    errs = []
    print(f"  {'cmd°':>6} {'measured°':>10} {'err°':>7}")
    for a in angles:
        proto.send("SET sync=1", 150)   # worst case for the old coupling bug
        m, s, e = measured_turn(dc, cam, proto, a)
        err = m - a
        errs.append(abs(err))
        print(f"  {a:6.0f} {m:10.1f} {err:+7.1f}")
        time.sleep(0.4)
    print(f"  -> mean |err| = {sum(errs)/len(errs):.1f}°   "
          f"max |err| = {max(errs):.1f}°")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--step", default="turn", help="comma-separated: turn,turnto,circle,fwd,outback,goto")
    ap.add_argument("--port", default="/dev/cu.usbmodem2121302")
    ap.add_argument("--cam-index", type=int, default=1)
    ap.add_argument("--angles", default="90,-90,45,-45,135,-180")
    ap.add_argument("--tol", type=float, default=3.0, help="closed-loop heading tolerance °")
    ap.add_argument("--dists", default="20,30", help="forward distances cm for fwd step")
    ap.add_argument("--arrive", type=float, default=4.0, help="goto arrival tolerance cm")
    args = ap.parse_args()

    dc, cam = open_daemon(args.cam_index)
    p = read_pose(dc, cam, 3.0)
    if p is None:
        dc.close()
        raise SystemExit(f"camera does not see robot tag {ROBOT_TAG_ID}")
    print(f"robot @ ({p[0]:+.0f},{p[1]:+.0f}) cm  tag_yaw {math.degrees(p[2]):+.0f}° "
          f"(compass {compass(p[2]):.0f}°)")

    robot, conn, proto = open_robot(args.port)
    proto.send("SET sTimeout=600", 200)
    try:
        with BenchRun(proto, max_seconds=600):
            steps = args.step.split(",")
            if "turn" in steps:
                step_turn(dc, cam, proto, args)
            if "turnto" in steps:
                step_turnto(dc, cam, proto, args)
            if "circle" in steps:
                step_circle(dc, cam, proto, args)
            if "fwd" in steps:
                step_fwd(dc, cam, proto, args)
            if "outback" in steps:
                step_outback(dc, cam, proto, args)
            if "goto" in steps:
                step_goto(dc, cam, proto, args)
            if "roam" in steps:
                step_roam(dc, cam, proto, args)
            if "center" in steps or "centre" in steps:
                step_center(dc, cam, proto, args)
    finally:
        for _ in range(3):
            proto.stop()
            time.sleep(0.03)
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
