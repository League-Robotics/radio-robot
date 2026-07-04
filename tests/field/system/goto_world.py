#!/usr/bin/env python3
"""goto_world.py — camera-localized drive to a world coordinate, sim-gated.

Given a target on the playfield in WORLD coordinates (cm, A1-centred), this:

  1. opens the camera and localizes the robot (AprilTag → world pose);
  2. computes the single firmware ``G`` command (world → robot-relative mm) that
     would take it there — a plain rigid-body projection by the robot's forward
     heading (same convention as ``Navigator.navigate`` / ``go_to_world``);
  3. SIMULATES that exact ``G`` in the in-process firmware sim, transforms the
     predicted path back into world coordinates, and checks it (a) stays on the
     playfield and (b) terminates near the target;
  4. ONLY if the simulation passes, executes the ``G`` on the real robot —
     aborting live if the camera track leaves the field — and reports the final
     camera-measured error.

``--random`` drives to a random COLORED RECTANGLE on the OPPOSITE north/south side
from the robot, so each leg crosses the x-axis (3 squares per side ⇒ 3 choices).

The world→robot map is a plain rotation R(H) by the robot's forward heading H
(the camera's tag orientation directly, 0 = east, CCW+); ``robot_to_world`` is
its transpose-inverse R(-H), used to transform the simulated robot-relative
path back into world — keeping the prediction self-consistent with how the
robot actually moves.

    uv run python tests/system/goto_world.py --x 30 --y -10
    uv run python tests/system/goto_world.py --random
    uv run python tests/system/goto_world.py --random --sim-only
"""

from __future__ import annotations

import argparse
import math
import random
import sys
import time


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--xy", nargs=2, type=float, metavar=("X_CM", "Y_CM"),
                   help="Target world coordinate in cm (A1-centred)")
    g.add_argument("--random", action="store_true",
                   help="Drive to a random colored SQUARE on the opposite n/s side "
                        "(crosses the x-axis; 3 choices per leg)")
    g.add_argument("--where", metavar="QUERY",
                   help="Resolve a playfield feature via the aprilcam daemon's where "
                        "function and drive there. Say a SQUARE ('blue square', "
                        "'black square') or a DOT ('green dot', 'red dot', "
                        "'northeast orange dot'). Bare colors that are unique resolve "
                        "directly (black/purple/magenta→square, yellow/red→dot); "
                        "blue/green/orange exist as both, so add 'square' or 'dot'.")

    p.add_argument("--sim-only", action="store_true",
                   help="Only simulate + report; do NOT drive the real robot")
    p.add_argument("--speed", type=int, default=160, help="G arc speed mm/s (default 160)")
    p.add_argument("--arrive", type=float, default=8.0,
                   help="Arrival tolerance in cm (sim + real) (default 8)")
    p.add_argument("--port", default=None, help="Serial port (auto-detect if omitted)")
    p.add_argument("--camera", default=None,
                   help="Camera name (default: first camera in the aprilcam daemon)")
    p.add_argument("--robot-tag", type=int, default=100,
                   help="AprilTag id on the robot (default 100; the field-centre "
                        "reference tag is id 1, so do NOT use 1 for the robot)")
    p.add_argument("--seed", type=int, default=None, help="RNG seed for --random")
    p.add_argument("--no-paths", action="store_true",
                   help="don't draw the onboard-odometry and camera trajectories "
                        "as overlays on AprilCamView")
    # Field bounds (cm, half-extent from the A1-centred origin). The real field
    # is 134.3x89.3 cm → ±67 x ±44.65. Default safe box is ±58 x ±40 so the
    # cardinal DOTS (x=±50) and the squares (±35/±24) are all reachable while
    # the robot body stays well inside the physical edge.
    p.add_argument("--field-x", type=float, default=58.0,
                   help="Playfield safe half-width in x cm (default 58)")
    p.add_argument("--field-y", type=float, default=40.0,
                   help="Playfield safe half-width in y cm (default 40)")
    p.add_argument("--min-dist", type=float, default=25.0,
                   help="(--random) minimum distance from current pose, cm (default 25)")
    p.add_argument("--margin", type=float, default=12.0,
                   help="(--random) keep targets this far inside the field, cm (default 12)")
    return p.parse_args(argv)


def world_to_robot(x_cm, y_cm, yaw_rad, tx_cm, ty_cm) -> tuple[float, float]:
    """World target → robot-relative (fwd, left).

    Plain rigid-body projection.  ``yaw_rad`` is the robot's FORWARD heading in
    world — the camera's tag orientation directly (0 = east, CCW+), with NO
    offset.  Forward unit = (cosH, sinH); left unit = (-sinH, cosH) (90° CCW).
    The firmware G's +left is the robot's physical left (CCW), so:

        fwd = dx*cosH + dy*sinH
        lft = -dx*sinH + dy*cosH

    This is the same convention as ``Navigator.navigate`` /
    ``nezha_kinematic.go_to_world`` — no handedness fudge.
    """
    dx = tx_cm - x_cm
    dy = ty_cm - y_cm
    fwd = dx * math.cos(yaw_rad) + dy * math.sin(yaw_rad)
    lft = -dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)
    return fwd * 10.0, lft * 10.0


def robot_to_world(fwd, lft, rx_cm, ry_cm, yaw_rad) -> tuple[float, float]:
    """Robot-relative (fwd, left) displacement → world (x_cm, y_cm).

    Inverse rotation of world_to_robot (a proper rotation, transpose-inverse):
        dx = fwd*cosH - lft*sinH
        dy = fwd*sinH + lft*cosH
    """
    dx = fwd * math.cos(yaw_rad) - lft * math.sin(yaw_rad)
    dy = fwd * math.sin(yaw_rad) + lft * math.cos(yaw_rad)
    return rx_cm + dx / 10.0, ry_cm + dy / 10.0


def resolve_where(pf, query: str) -> tuple[str, float, float]:
    """Resolve a feature QUERY to (slug, x_cm, y_cm) via the daemon's `where`.

    Uses the aprilcam daemon's where_is (the same function `rogo where` uses) and
    HONORS the feature kind in the query:

      * '<color> square' / '...rect'  → the colored SQUARE  (slug ``rect-*``)
      * '<color> dot'                 → the colored DOT      (slug ``dot-*``)
      * bare '<color>'                → whatever uniquely matches; colors that
        exist as BOTH a square and a dot (blue/green/orange) come back ambiguous
        so you add 'square' or 'dot'. 'orange dot' needs a direction too (there
        are four), e.g. 'northeast orange dot'.

    If a match carries a live detection (more accurate than the designed map
    layout), its world_xy is used; otherwise the map location is used.
    """
    result = pf._dc.where_is(query, pf._cam)
    matches = result.get("matches", [])
    status = result.get("status")
    if not matches:
        raise SystemExit(
            f"[goto_world] where '{query}': {status} — no playfield feature matched."
        )
    # Honor explicit feature kind; only fall back to all matches if the filter
    # would empty the set (so a stray ranking never silently flips dot↔square).
    q = query.lower()

    def _of(prefix):
        return [m for m in matches if str(m.get("slug", "")).startswith(prefix)]

    if "dot" in q:
        cands = _of("dot") or matches
    elif "square" in q or "rect" in q:
        cands = _of("rect") or matches
    else:
        cands = matches
    if len(cands) > 1:
        opts = ", ".join(m.get("slug", "?") for m in cands)
        raise SystemExit(
            f"[goto_world] where '{query}': ambiguous — matches {opts}. "
            f"Add 'square' or 'dot' and/or a direction "
            f"(e.g. 'blue square', 'green dot', 'northeast orange dot')."
        )
    m = cands[0]
    live = m.get("live_detection") or {}
    loc = m.get("location") or {}
    if live.get("world_xy"):
        x, y = live["world_xy"]                       # live detection (accurate)
    elif loc.get("x") is not None and loc.get("y") is not None:
        x, y = loc["x"], loc["y"]                     # designed map layout
    else:
        raise SystemExit(
            f"[goto_world] where '{query}': match {m.get('slug')} has no location."
        )
    return m.get("slug", query), float(x), float(y)


def get_rectangles(pf) -> list[tuple[str, float, float]]:
    """The colored SQUARE features as [(slug, x_cm, y_cm), ...], from the daemon map."""
    result = pf._dc.where_is("rectangle", pf._cam)
    rects: list[tuple[str, float, float]] = []
    for m in result.get("matches", []):
        slug = str(m.get("slug", ""))
        if not slug.startswith("rect"):
            continue
        loc = m.get("location") or {}
        rec = m.get("record", {})
        x = loc.get("x", rec.get("x"))
        y = loc.get("y", rec.get("y"))
        if x is not None and y is not None:
            rects.append((slug, float(x), float(y)))
    return rects


def pick_random_rect(rects, ry, rng) -> tuple[str, float, float]:
    """Pick a random colored rectangle on the OPPOSITE north/south side from the
    robot, so every leg crosses the x-axis. Three squares per side ⇒ 3 choices.
    """
    if ry >= 0.0:
        cands = [r for r in rects if r[2] < 0.0]    # robot north -> a south square
    else:
        cands = [r for r in rects if r[2] > 0.0]    # robot south -> a north square
    if not cands:
        cands = list(rects)
    return rng.choice(cands)


def simulate(fwd, lft, speed, rx, ry, yaw, half_x, half_y, tx, ty, arrive):
    """Run the G in the firmware sim; return (ok, reason, path_world, final, err)."""
    from robot_radio.testkit import make_target

    sim = make_target("sim")
    path: list[tuple[float, float]] = [(rx, ry)]

    def on_tick(robot) -> None:
        s = robot.state.pose  # mm, sim frame: robot starts at (0,0,0), fwd=+x, left=+y
        path.append(robot_to_world(s.x, s.y, rx, ry, yaw))

    try:
        sim.robot.go_to(int(round(fwd)), int(round(lft)), speed,
                        on_tick=on_tick, timeout_s=30.0)
    finally:
        try:
            sim.conn.disconnect()
        except Exception:
            pass

    off = [(wx, wy) for (wx, wy) in path if abs(wx) > half_x or abs(wy) > half_y]
    final = path[-1]
    err = math.hypot(final[0] - tx, final[1] - ty)
    if off:
        wx, wy = max(off, key=lambda p: max(abs(p[0]), abs(p[1])))
        return False, f"path leaves field at ({wx:.1f},{wy:.1f}) cm", path, final, err
    if err > arrive:
        return False, f"ends {err:.1f}cm from target (> {arrive:.1f}cm)", path, final, err
    return True, "ok", path, final, err


def drive_to_target(robot, pf, pose_src, tx, ty, *, speed=160, arrive=8.0,
                    field_x=58.0, field_y=40.0, sim_only=False, draw_paths=True,
                    clear_first=False, max_passes=4, label="goto",
                    verbose=True) -> dict:
    """Closed-loop drive to world target (tx, ty) cm (A1-centred). Reusable leg.

    Re-localizes off the camera and drives the residual each pass until within
    ``arrive`` cm or progress stalls.  Samples the firmware fused pose (onboard)
    and the camera ground truth every tick; draws them as RED/GREEN AprilCamView
    overlays (path ids ``<label>_onboard`` / ``<label>_camera``) when draw_paths.
    Does NOT own the connection (caller opens/closes robot + playfield).

    Returns a dict: start/end (x,y,yaw), target (x,y), error, arrived, passes,
    onboard/camera point lists, sim_only, aborted.
    """
    p = print if verbose else (lambda *a, **k: None)
    if clear_first:
        try:
            pf.clear_paths()
        except Exception:
            pass

    rx, ry, yaw = pose_src.read()
    sx, sy, syaw = rx, ry, yaw
    aborted = {"hit": False}
    # Four world-frame (A1-centred cm) traces for the AprilCamView overlay:
    #   camera  = ground truth (daemon)
    #   fused   = firmware EKF pose (state.pose, SI-anchored to world each pass)
    #   otos    = OTOS-only (telemetry otos = readTransformed world pose)
    #   encoder = host dead-reckoning from wheel mm, anchored to the camera fix
    camera_pts: list[tuple[float, float]] = []   # GREEN  (truth)
    fused_pts:  list[tuple[float, float]] = []   # RED    (state.pose)
    otos_pts:   list[tuple[float, float]] = []   # BLUE   (state.otos_pose)
    enc_pts:    list[tuple[float, float]] = []   # ORANGE (host odometry)
    onboard_pts = fused_pts                       # back-compat alias (== fused)
    enc_state: dict = {}                          # encoder dead-reckoning accumulator
    # Replicate the firmware encoder heading (Odometry::predict):
    #   dTheta = ((dR - dL) / trackwidthMm) * rotationalSlip
    # trackwidth is now the real 128mm physical track; rotational_slip absorbs the
    # residual wheel scrub.  The encoder heading is still noisier than the OTOS.
    TRACKWIDTH = 128.0                          # tovez.json geometry.trackwidth (physical)
    ROT_SLIP = 0.92                                # tovez.json calibration.rotational_slip
    try:
        robot._proto.stream_fields("enc,pose,otos")    # ensure all sources stream
    except Exception:
        pass

    def on_tick(_robot) -> bool | None:
        st = _robot.state
        cx = cy = cyaw = None
        try:
            cx, cy, cyaw = pose_src.read()
            camera_pts.append((cx, cy))
            if abs(cx) > field_x or abs(cy) > field_y:
                p(f"  [BOUNDS-ABORT] camera ({cx:+.1f},{cy:+.1f}) left the "
                  f"±{field_x:.0f}x±{field_y:.0f} safe box")
                aborted["hit"] = True
                return False
        except Exception:
            pass
        try:
            pp = st.pose                                  # fused EKF (world)
            if pp is not None:
                fused_pts.append((pp.x / 10.0, pp.y / 10.0))   # mm -> cm
        except Exception:
            pass
        try:
            op = st.otos_pose                             # OTOS-only (world, telemetry)
            if op is not None:
                otos_pts.append((op[0] / 10.0, op[1] / 10.0))
        except Exception:
            pass
        try:
            enc = st.encoders                             # wheel mm -> host odometry
            if enc is not None:
                if not enc_state:
                    if cx is not None and cyaw is not None:
                        enc_state.update(eL=enc[0], eR=enc[1], x=cx, y=cy, th=cyaw)
                else:
                    dL = enc[0] - enc_state["eL"]
                    dR = enc[1] - enc_state["eR"]
                    enc_state["eL"], enc_state["eR"] = enc[0], enc[1]
                    dC = (dL + dR) / 2.0                   # mm
                    dTh = ((dR - dL) / TRACKWIDTH) * ROT_SLIP   # rad, CCW+ (firmware calc)
                    th_mid = enc_state["th"] + dTh / 2.0
                    enc_state["x"] += (dC / 10.0) * math.cos(th_mid)
                    enc_state["y"] += (dC / 10.0) * math.sin(th_mid)
                    enc_state["th"] += dTh
                    enc_pts.append((enc_state["x"], enc_state["y"]))
        except Exception:
            pass
        return None

    arrived = False
    enc0 = None
    ex, ey, eyaw = rx, ry, yaw
    prev_err = float("inf")
    attempt = 0
    for attempt in range(1, max_passes + 1):
        if attempt > 1:
            try:
                rx, ry, yaw = pose_src.read()
            except Exception:
                p("[drive] lost robot tag mid-approach — stopping.")
                break
        ex, ey, eyaw = rx, ry, yaw
        dist = math.hypot(tx - rx, ty - ry)
        if dist <= arrive:
            arrived = True
            p(f"[drive] pass {attempt}: within tol ({dist:.1f}cm ≤ {arrive:.0f}cm) — arrived.")
            break

        fwd, lft = world_to_robot(rx, ry, yaw, tx, ty)
        p(f"[drive] pass {attempt}: @ ({rx:+.1f},{ry:+.1f}) yaw={math.degrees(yaw):+.0f}°  "
          f"{dist:.1f}cm out → G fwd={fwd:+.0f} left={lft:+.0f} @ {speed}mm/s")

        ok, reason, _path, _sfinal, serr = simulate(
            fwd, lft, speed, rx, ry, yaw, field_x, field_y, tx, ty, arrive,
        )
        if not ok:
            p(f"[drive]   SIM FAIL: {reason} — NOT driving this leg.")
            break
        p(f"[drive]   SIM PASS: predicts {serr:.1f}cm from target.")

        if sim_only:
            p("[drive] --sim-only: not driving.")
            return {"start": (sx, sy, syaw), "end": (ex, ey, eyaw), "target": (tx, ty),
                    "error": math.hypot(tx - ex, ty - ey), "arrived": False,
                    "passes": attempt, "onboard": onboard_pts, "camera": camera_pts,
                    "fused": fused_pts, "otos": otos_pts, "encoder": enc_pts,
                    "sim_only": True, "aborted": False}

        robot.update_world_pose(rx, ry, yaw)   # SI: anchor firmware pose to world
        if enc0 is None:
            enc0 = robot.refresh().encoders or (0, 0)

        timeout_s = dist * 10.0 / max(speed, 1) + 6.0
        _el, _er, outcome = robot.go_to(int(round(fwd)), int(round(lft)),
                                        speed, on_tick=on_tick, timeout_s=timeout_s)
        p(f"[drive]   leg outcome={outcome}")
        if aborted["hit"]:
            robot.stop()
            break
        time.sleep(0.4)
        try:
            ex, ey, eyaw = pose_src.read()
        except Exception:
            p("[drive] lost robot tag after leg.")
            break
        ferr = math.hypot(tx - ex, ty - ey)
        p(f"[drive]   now @ ({ex:+.1f},{ey:+.1f}) error={ferr:.1f}cm")
        if ferr <= arrive:
            arrived = True
            break
        if ferr > prev_err - 0.5:
            p(f"[drive]   no further progress ({ferr:.1f} ≥ {prev_err:.1f}cm) — stopping.")
            break
        prev_err = ferr

    ferr = math.hypot(tx - ex, ty - ey)
    p(f"[drive] FINAL @ ({ex:+.1f},{ey:+.1f})cm yaw={math.degrees(eyaw):+.0f}°  "
      f"error={ferr:.1f}cm  {'ARRIVED' if arrived else 'NOT-ARRIVED'} in {attempt} pass(es)")

    if enc0 is not None and not aborted["hit"]:
        try:
            el2, er2 = robot.refresh().encoders or enc0
            wheel_cm = (abs(el2 - enc0[0]) + abs(er2 - enc0[1])) / 2.0 / 10.0
            cam_cm = math.hypot(ex - sx, ey - sy)
            if wheel_cm > 10.0 and cam_cm < wheel_cm * 0.3:
                p(f"[drive] WARNING: wheels drove ~{wheel_cm:.0f}cm but the camera shows "
                  f"only ~{cam_cm:.0f}cm — robot may be on a STAND.")
        except Exception:
            pass

    if draw_paths:
        try:
            traces = [("camera",  camera_pts, (80, 230, 120)),   # GREEN  truth
                      ("fused",   fused_pts,  (255, 80, 80)),     # RED    EKF
                      ("otos",    otos_pts,   (80, 160, 255)),    # BLUE   OTOS-only
                      ("encoder", enc_pts,    (255, 190, 40))]    # ORANGE encoder-only
            for name, pts, col in traces:
                if pts:
                    pf.add_path(f"{label}_{name}", pts, symbol="filled_circle",
                                color=col, size_cm=0.4)
            p(f"[drive] AprilCamView paths: camera={len(camera_pts)}(GREEN) "
              f"fused={len(fused_pts)}(RED) otos={len(otos_pts)}(BLUE) "
              f"encoder={len(enc_pts)}(ORANGE)")
        except Exception as exc:
            p(f"[drive] path draw failed: {exc}")

    return {"start": (sx, sy, syaw), "end": (ex, ey, eyaw), "target": (tx, ty),
            "error": ferr, "arrived": arrived, "passes": attempt,
            "onboard": onboard_pts, "camera": camera_pts,
            "fused": fused_pts, "otos": otos_pts, "encoder": enc_pts,
            "sim_only": False, "aborted": aborted["hit"]}


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)

    from robot_radio.testkit import make_target
    from robot_radio.field.playfield import Playfield
    from robot_radio.testkit.pose import CameraPose

    # --- Robot (production) + camera localization ---------------------------
    tr = make_target("production", port=args.port)
    robot = tr.robot
    print(f"[goto_world] connected: fw={robot.connect().get('fw')}")

    pf = Playfield.open(args.camera)
    pose_src = CameraPose(pf, tag_id=args.robot_tag)

    rx, ry, yaw = pose_src.read()
    print(f"[goto_world] robot @ ({rx:+.1f}, {ry:+.1f}) cm  yaw={math.degrees(yaw):+.0f}°")

    # --- Target -------------------------------------------------------------
    if args.where:
        slug, tx, ty = resolve_where(pf, args.where)
        print(f"[goto_world] where '{args.where}' → {slug} @ ({tx:+.1f}, {ty:+.1f}) cm")
        if abs(tx) > args.field_x or abs(ty) > args.field_y:
            print(f"[goto_world] FAIL: {slug} ({tx:+.1f},{ty:+.1f}) is outside the "
                  f"safe field (±{args.field_x:.0f} x ±{args.field_y:.0f})")
            return 2
    elif args.random:
        rng = random.Random(args.seed)
        rects = get_rectangles(pf)
        if not rects:
            raise SystemExit("[goto_world] --random: no colored rectangles on the playfield.")
        slug, tx, ty = pick_random_rect(rects, ry, rng)
        print(f"[goto_world] random rect → {slug} @ ({tx:+.1f}, {ty:+.1f}) cm")
    else:
        tx, ty = args.xy
        if abs(tx) > args.field_x or abs(ty) > args.field_y:
            print(f"[goto_world] FAIL: target ({tx:+.1f},{ty:+.1f}) is outside the "
                  f"field (±{args.field_x:.0f} x ±{args.field_y:.0f})")
            return 2

    # --- Drive (closed-loop) + draw the two trajectory overlays --------------
    res = drive_to_target(
        robot, pf, pose_src, tx, ty,
        speed=args.speed, arrive=args.arrive,
        field_x=args.field_x, field_y=args.field_y,
        sim_only=args.sim_only, draw_paths=not args.no_paths,
        label="goto", verbose=True,
    )

    # Machine-parseable summary line (consumed by the tour harness for the CSV).
    sx, sy, syaw = res["start"]
    ex, ey, eyaw = res["end"]
    print(f"[goto_world] RESULT start=({sx:.1f},{sy:.1f},{math.degrees(syaw):.0f}) "
          f"end=({ex:.1f},{ey:.1f},{math.degrees(eyaw):.0f}) "
          f"target=({tx:.1f},{ty:.1f}) error={res['error']:.1f} "
          f"arrived={res['arrived']} passes={res['passes']}")

    _cleanup(tr, pf)
    if args.sim_only:
        return 0
    return 0 if res["arrived"] else 1


def _cleanup(tr, pf) -> None:
    try:
        tr.robot.stop()
    except Exception:
        pass
    try:
        tr.conn.disconnect()
    except Exception:
        pass
    try:
        pf.close()
    except Exception:
        pass


if __name__ == "__main__":
    sys.exit(main())
