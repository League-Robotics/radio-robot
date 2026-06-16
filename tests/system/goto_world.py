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

``--random`` picks a reachable point on the playfield at least 25 cm from the
robot's current position (and inside the field).

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
                   help="Pick a random reachable point >=25 cm away, on the field")

    p.add_argument("--sim-only", action="store_true",
                   help="Only simulate + report; do NOT drive the real robot")
    p.add_argument("--speed", type=int, default=160, help="G arc speed mm/s (default 160)")
    p.add_argument("--arrive", type=float, default=8.0,
                   help="Arrival tolerance in cm (sim + real) (default 8)")
    p.add_argument("--port", default=None, help="Serial port (auto-detect if omitted)")
    p.add_argument("--camera", default=None,
                   help="Camera name (default: first camera in the aprilcam daemon)")
    p.add_argument("--robot-tag", type=int, default=1,
                   help="AprilTag id on the robot (default 1)")
    p.add_argument("--seed", type=int, default=None, help="RNG seed for --random")
    p.add_argument("--no-paths", action="store_true",
                   help="don't draw the onboard-odometry and camera trajectories "
                        "as overlays on AprilCamView")
    # Field bounds (cm, half-extent from the A1-centred origin). The LEAG field
    # is ~101x89 cm → ~±50 x ±44; keep a margin so the robot body stays inside.
    p.add_argument("--field-x", type=float, default=48.0,
                   help="Playfield safe half-width in x cm (default 48)")
    p.add_argument("--field-y", type=float, default=40.0,
                   help="Playfield safe half-width in y cm (default 40)")
    p.add_argument("--min-dist", type=float, default=25.0,
                   help="(--random) minimum distance from current pose, cm (default 25)")
    p.add_argument("--margin", type=float, default=12.0,
                   help="(--random) keep targets this far inside the field, cm (default 12)")
    return p.parse_args(argv)


def world_to_robot(x_cm, y_cm, yaw_rad, tx_cm, ty_cm) -> tuple[float, float]:
    """World target → robot-relative (fwd_mm, left_mm).

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


def robot_to_world(fwd_mm, lft_mm, rx_cm, ry_cm, yaw_rad) -> tuple[float, float]:
    """Robot-relative (fwd_mm, left_mm) displacement → world (x_cm, y_cm).

    Inverse rotation of world_to_robot (a proper rotation, transpose-inverse):
        dx = fwd*cosH - lft*sinH
        dy = fwd*sinH + lft*cosH
    """
    dx = fwd_mm * math.cos(yaw_rad) - lft_mm * math.sin(yaw_rad)
    dy = fwd_mm * math.sin(yaw_rad) + lft_mm * math.cos(yaw_rad)
    return rx_cm + dx / 10.0, ry_cm + dy / 10.0


def pick_random_target(rx, ry, half_x, half_y, margin, min_dist, rng) -> tuple[float, float]:
    lo_x, hi_x = -(half_x - margin), (half_x - margin)
    lo_y, hi_y = -(half_y - margin), (half_y - margin)
    if hi_x <= lo_x or hi_y <= lo_y:
        raise SystemExit("Field too small for the requested margin.")
    for _ in range(5000):
        tx = rng.uniform(lo_x, hi_x)
        ty = rng.uniform(lo_y, hi_y)
        if math.hypot(tx - rx, ty - ry) >= min_dist:
            return tx, ty
    raise SystemExit(
        f"Could not find a target >={min_dist}cm from ({rx:.1f},{ry:.1f}) "
        f"inside the field — move the robot toward the centre and retry."
    )


def simulate(fwd_mm, lft_mm, speed, rx, ry, yaw, half_x, half_y, tx, ty, arrive):
    """Run the G in the firmware sim; return (ok, reason, path_world, final, err)."""
    from robot_radio.testkit import make_target

    sim = make_target("sim")
    path: list[tuple[float, float]] = [(rx, ry)]

    def on_tick(robot) -> None:
        s = robot.state.pose  # mm, sim frame: robot starts at (0,0,0), fwd=+x, left=+y
        path.append(robot_to_world(s.x, s.y, rx, ry, yaw))

    try:
        sim.robot.go_to(int(round(fwd_mm)), int(round(lft_mm)), speed,
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
    if args.random:
        rng = random.Random(args.seed)
        tx, ty = pick_random_target(rx, ry, args.field_x, args.field_y,
                                    args.margin, args.min_dist, rng)
        print(f"[goto_world] random target → ({tx:+.1f}, {ty:+.1f}) cm")
    else:
        tx, ty = args.xy
        if abs(tx) > args.field_x or abs(ty) > args.field_y:
            print(f"[goto_world] FAIL: target ({tx:+.1f},{ty:+.1f}) is outside the "
                  f"field (±{args.field_x:.0f} x ±{args.field_y:.0f})")
            return 2

    # --- Drive: closed-loop — re-localize and correct until within tolerance --
    # A single open-loop G arc undershoots (encoder undercount); re-reading the
    # camera after each leg and driving the residual converges to the target.
    sx, sy, syaw = rx, ry, yaw          # original start (for the report/CSV)
    MAX_PASSES = 4
    aborted = {"hit": False}
    onboard_pts: list[tuple[float, float]] = []  # firmware fused pose (onboard), world cm
    camera_pts: list[tuple[float, float]] = []   # camera ground truth, world cm

    def on_tick(_robot) -> bool | None:
        # Sample BOTH trajectories each tick (time-aligned): the firmware's fused
        # pose (onboard odometry) and the camera ground truth.  Drawn afterward as
        # two AprilCamView overlays so the odometry-vs-reality divergence is visible.
        try:
            p = _robot.state.pose
            if p is not None:
                onboard_pts.append((p.x / 10.0, p.y / 10.0))   # mm → cm
        except Exception:
            pass
        try:
            cx, cy, _ = pose_src.read()
            camera_pts.append((cx, cy))
            if abs(cx) > args.field_x or abs(cy) > args.field_y:
                print(f"  [BOUNDS-ABORT] camera ({cx:+.1f},{cy:+.1f}) left the "
                      f"±{args.field_x:.0f}x±{args.field_y:.0f} safe box")
                aborted["hit"] = True
                return False
        except Exception:
            pass  # transient tag dropout — keep driving
        return None

    arrived = False
    enc0 = None
    ex, ey, eyaw = rx, ry, yaw
    prev_err = float("inf")
    attempt = 0
    for attempt in range(1, MAX_PASSES + 1):
        if attempt > 1:
            try:
                rx, ry, yaw = pose_src.read()
            except Exception:
                print("[goto_world] lost robot tag mid-approach — stopping.")
                break
        ex, ey, eyaw = rx, ry, yaw
        dist = math.hypot(tx - rx, ty - ry)
        if dist <= args.arrive:
            arrived = True
            print(f"[goto_world] pass {attempt}: within tol ({dist:.1f}cm ≤ "
                  f"{args.arrive:.0f}cm) — arrived.")
            break

        fwd_mm, lft_mm = world_to_robot(rx, ry, yaw, tx, ty)
        print(f"[goto_world] pass {attempt}: @ ({rx:+.1f},{ry:+.1f}) "
              f"yaw={math.degrees(yaw):+.0f}°  {dist:.1f}cm out → "
              f"G fwd={fwd_mm:+.0f} left={lft_mm:+.0f} @ {args.speed}mm/s")

        # SIM-gate EVERY leg: never drive a path that leaves the field.
        ok, reason, _path, _sfinal, serr = simulate(
            fwd_mm, lft_mm, args.speed, rx, ry, yaw,
            args.field_x, args.field_y, tx, ty, args.arrive,
        )
        if not ok:
            print(f"[goto_world]   SIM FAIL: {reason} — NOT driving this leg.")
            break
        print(f"[goto_world]   SIM PASS: predicts {serr:.1f}cm from target.")

        if args.sim_only:
            print("[goto_world] --sim-only: done (not driving).")
            _cleanup(tr, pf)
            return 0

        # Anchor the firmware world pose to the camera fix, then drive the leg.
        robot.update_world_pose(rx, ry, yaw)
        time.sleep(0.1)
        if enc0 is None:
            enc0 = robot.refresh().encoders or (0, 0)

        timeout_s = dist * 10.0 / max(args.speed, 1) + 6.0
        _el, _er, outcome = robot.go_to(int(round(fwd_mm)), int(round(lft_mm)),
                                        args.speed, on_tick=on_tick, timeout_s=timeout_s)
        print(f"[goto_world]   leg outcome={outcome}")
        if aborted["hit"]:
            robot.stop()
            break
        time.sleep(0.4)
        try:
            ex, ey, eyaw = pose_src.read()
        except Exception:
            print("[goto_world] lost robot tag after leg.")
            break
        ferr = math.hypot(tx - ex, ty - ey)
        print(f"[goto_world]   now @ ({ex:+.1f},{ey:+.1f}) error={ferr:.1f}cm")
        if ferr <= args.arrive:
            arrived = True
            break
        if ferr > prev_err - 0.5:      # no meaningful progress → stop correcting
            print(f"[goto_world]   no further progress ({ferr:.1f} ≥ "
                  f"{prev_err:.1f}cm) — stopping corrections.")
            break
        prev_err = ferr

    # --- Report --------------------------------------------------------------
    ferr = math.hypot(tx - ex, ty - ey)
    print(f"[goto_world] FINAL @ ({ex:+.1f},{ey:+.1f})cm yaw={math.degrees(eyaw):+.0f}°  "
          f"error={ferr:.1f}cm  (arrive {args.arrive:.0f}cm)  "
          f"{'ARRIVED' if arrived else 'NOT-ARRIVED'} in {attempt} pass(es)")

    # Stand check: wheels turned but the robot didn't translate ⇒ on a stand.
    if enc0 is not None and not aborted["hit"]:
        try:
            el2, er2 = robot.refresh().encoders or enc0
            wheel_cm = (abs(el2 - enc0[0]) + abs(er2 - enc0[1])) / 2.0 / 10.0
            cam_cm = math.hypot(ex - sx, ey - sy)
            if wheel_cm > 10.0 and cam_cm < wheel_cm * 0.3:
                print(f"[goto_world] WARNING: wheels drove ~{wheel_cm:.0f}cm but the "
                      f"camera shows only ~{cam_cm:.0f}cm — robot may be on a STAND.")
        except Exception:
            pass

    # Machine-parseable summary line (consumed by the tour harness for the CSV).
    print(f"[goto_world] RESULT start=({sx:.1f},{sy:.1f},{math.degrees(syaw):.0f}) "
          f"end=({ex:.1f},{ey:.1f},{math.degrees(eyaw):.0f}) "
          f"target=({tx:.1f},{ty:.1f}) error={ferr:.1f} "
          f"arrived={arrived} passes={attempt}")

    # --- Draw the two trajectories on AprilCamView --------------------------
    # RED  = onboard odometry (firmware fused pose); GREEN = camera ground truth.
    # Their divergence shows how well the onboard pose tracks reality.
    if not args.no_paths:
        try:
            if onboard_pts:
                pf.add_path("goto_onboard", onboard_pts, symbol="filled_circle",
                            color=(255, 80, 80), size_cm=0.4)
            if camera_pts:
                pf.add_path("goto_camera", camera_pts, symbol="filled_circle",
                            color=(80, 230, 120), size_cm=0.4)
            print(f"[goto_world] AprilCamView paths: onboard={len(onboard_pts)}pts (RED), "
                  f"camera={len(camera_pts)}pts (GREEN)")
        except Exception as exc:
            print(f"[goto_world] path draw failed: {exc}")

    _cleanup(tr, pf)
    return 0 if arrived else 1


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
