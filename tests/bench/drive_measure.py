#!/usr/bin/env python3
"""drive_measure.py — slow straight drive, compare encoder vs OTOS vs camera.

The simplest possible calibration check:
  1. Connect via the rogo path (which pushes the robot's stored calibration —
     data/robots/tovez.json: OTOS linear/angular scalars — onto the firmware).
  2. Read the robot's world position from the camera (truth).
  3. Drive FORWARD slowly for a fixed time (capped so it cannot leave the
     field), streaming encoder totals and the OTOS pose.
  4. Stop, read the camera again.
  5. Report how far each source thinks it went:
       - camera   (ground truth, cm)
       - encoders (cumulative wheel mm → cm)
       - OTOS     (optical-flow odometry displacement, cm)
     and the scale factors that would make OTOS / encoders match the camera.

Usage:
    uv run python tests/bench/drive_measure.py [--speed 50] [--secs 10]
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

DEFAULT_PLAYFIELD = "/Volumes/Proj/proj/RobotProjects/AprilTags/data/aprilcam/playfield.json"
ROBOT_TAG_ID = 100


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=None)
    p.add_argument("--speed", type=int, default=50, help="forward wheel speed mm/s (default 50)")
    p.add_argument("--secs", type=float, default=10.0, help="drive duration s (default 10)")
    p.add_argument("--margin", type=float, default=14.0,
                   help="keep the robot this many cm inside the ArUco-corner extent")
    p.add_argument("--playfield", default=DEFAULT_PLAYFIELD)
    return p.parse_args()


def read_cam_pose(dc, cam, timeout_s=2.0):
    """Return (x_cm, y_cm, yaw_rad) for the robot tag, or None."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        tf = dc.get_tags(cam)
        for t in tf.tags:
            if t.id == ROBOT_TAG_ID and t.world_xy is not None and t.yaw is not None:
                return float(t.world_xy[0]), float(t.world_xy[1]), float(t.yaw)
        time.sleep(0.03)
    return None


def main() -> int:
    import json
    args = _parse_args()

    # --- geofence from the playfield (so an open-loop drive can't leave it) ---
    with open(args.playfield) as f:
        pf = json.load(f)
    xs = [float(u["x"]) for u in pf["aruco_tags"]]
    ys = [float(u["y"]) for u in pf["aruco_tags"]]
    xlo, xhi = min(xs) + args.margin, max(xs) - args.margin
    ylo, yhi = min(ys) + args.margin, max(ys) - args.margin

    # --- daemon (camera) ---
    print("Connecting to aprilcam daemon …")
    from aprilcam.config import Config
    from aprilcam.client.control import DaemonControl
    dc = DaemonControl.connect_default(Config.load())
    cams = dc.list_cameras()
    cam = cams[0] if cams else dc.open_camera(index=1)
    print(f"  camera: {cam}")

    # --- robot (rogo path pushes stored calibration) ---
    print("Connecting to robot (pushes stored calibration) …")
    from robot_radio.io.cli import _make_robot
    from robot_radio.robot.nezha import Nezha
    from robot_radio.robot.protocol import parse_tlm

    class _A:
        port = args.port
        verbose = False
    robot, conn, _ = _make_robot(_A())
    if not isinstance(robot, Nezha):
        print("ERROR: need a Nezha robot"); return 2
    proto = robot._proto

    # Confirm the calibration that is now on the firmware.
    ol = proto.otos_get_linear_scalar()
    oa = proto.otos_get_angular_scalar()
    print(f"  firmware OTOS scalars: OL={ol}  OA={oa}")

    try:
        with BenchRun(proto, max_seconds=int(args.secs) + 30):
            # --- where are we (camera truth) ---
            p0 = read_cam_pose(dc, cam, timeout_s=3.0)
            if p0 is None:
                print("ERROR: camera cannot see the robot tag"); return 2
            cx0, cy0, cyaw0 = p0
            fwd = cyaw0     # tag orientation IS the forward direction (0=east, CCW+)

            # --- cap the drive so the forward endpoint stays inside the geofence ---
            # distance to each fence wall along +fwd:
            def wall_dist():
                d = float("inf")
                cosf, sinf = math.cos(fwd), math.sin(fwd)
                if cosf > 1e-6:
                    d = min(d, (xhi - cx0) / cosf)
                elif cosf < -1e-6:
                    d = min(d, (xlo - cx0) / cosf)
                if sinf > 1e-6:
                    d = min(d, (yhi - cy0) / sinf)
                elif sinf < -1e-6:
                    d = min(d, (ylo - cy0) / sinf)
                return d
            max_cm = wall_dist()
            req_cm = args.speed / 10.0 * args.secs
            if max_cm < 15.0:
                print(f"  robot at ({cx0:+.1f},{cy0:+.1f}) faces a near wall "
                      f"({max_cm:.0f}cm of room) — turn it toward open field and retry.")
                return 1
            drive_cm = min(req_cm, max_cm - 5.0)
            secs = drive_cm / (args.speed / 10.0)
            print(f"  start (camera): ({cx0:+.1f}, {cy0:+.1f})cm  facing {math.degrees(fwd):+.0f}°")
            print(f"  forward room to fence: {max_cm:.0f}cm → driving {drive_cm:.0f}cm "
                  f"(~{secs:.1f}s) at {args.speed}mm/s")

            # --- prepare telemetry + clean serial ---
            proto.send("STOP", 200)
            proto.send("STREAM 0", 200)
            time.sleep(0.05)
            proto.send("SET sTimeout=10000", 300)
            proto.zero_encoders()
            proto.stream(50)

            enc = (0, 0)
            otos0 = None
            otosL = None
            cam_trace = [(cx0, cy0)]

            # --- drive forward, keepalive S every 150 ms ---
            t0 = time.monotonic()
            last_send = 0.0
            last_cam = 0.0
            aborted = None
            while time.monotonic() - t0 < secs:
                now = time.monotonic()
                if now - last_send >= 0.15:
                    proto.drive(args.speed, args.speed)
                    last_send = now
                for line in proto.read_lines(duration_ms=25):
                    if "EVT safety_stop" in line:
                        proto.drive(args.speed, args.speed)  # re-arm
                    tlm = parse_tlm(line)
                    if tlm is None:
                        continue
                    if tlm.enc is not None:
                        enc = tlm.enc
                    if tlm.pose is not None:
                        otosL = (tlm.pose[0], tlm.pose[1])
                        if otos0 is None:
                            otos0 = otosL
                # best-effort live camera bound check
                if now - last_cam >= 0.2:
                    last_cam = now
                    cp = read_cam_pose(dc, cam, timeout_s=0.08)
                    if cp is not None:
                        cam_trace.append((cp[0], cp[1]))
                        if not (xlo <= cp[0] <= xhi and ylo <= cp[1] <= yhi):
                            aborted = "geofence"
                            break

            # --- stop ---
            for _ in range(4):
                proto.stop()
                time.sleep(0.05)
            proto.stream(0)
            if aborted:
                print(f"  ** stopped early: {aborted} **")

            # --- final camera read (stationary = reliable) ---
            time.sleep(0.3)
            p1 = read_cam_pose(dc, cam, timeout_s=3.0)
            if p1 is None:
                print("ERROR: lost camera at end"); return 2
            cx1, cy1, _ = p1
            cam_trace.append((cx1, cy1))

            # --- distances ---
            cam_cm = math.hypot(cx1 - cx0, cy1 - cy0)
            enc_cm = (abs(enc[0]) + abs(enc[1])) / 2.0 / 10.0
            if otos0 is not None and otosL is not None:
                otos_cm = math.hypot(otosL[0] - otos0[0], otosL[1] - otos0[1]) / 10.0
            else:
                otos_cm = float("nan")
            move_dir = math.degrees(math.atan2(cy1 - cy0, cx1 - cx0))
            dir_err = abs((move_dir - math.degrees(fwd) + 180) % 360 - 180)

            print("\n========= RESULT =========")
            print(f"  camera (truth):   {cam_cm:6.1f} cm   "
                  f"(moved {move_dir:+.0f}°, {dir_err:.0f}° off 'forward')")
            print(f"  encoders:         {enc_cm:6.1f} cm   "
                  f"(L={enc[0]}mm R={enc[1]}mm)")
            print(f"  OTOS odometry:    {otos_cm:6.1f} cm")
            if cam_cm > 1.0:
                print("  --- scale factors to match camera ---")
                if enc_cm > 1.0:
                    print(f"  encoder: camera/enc = {cam_cm/enc_cm:.4f}  "
                          f"(mm_per_wheel_deg ×{cam_cm/enc_cm:.4f})")
                if otos_cm > 1.0 and ol is not None:
                    new_ol = (ol / 100.0 if abs(ol) > 5 else ol)  # OL is int8-ish
                    print(f"  OTOS: camera/otos = {cam_cm/otos_cm:.4f}  "
                          f"→ otos_linear_scale ×{cam_cm/otos_cm:.4f} "
                          f"(currently 1.05 in tovez.json)")
            print("==========================")
    finally:
        try:
            for _ in range(3):
                proto.stop(); time.sleep(0.04)
            proto.stream(0)
        except Exception:
            pass
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
