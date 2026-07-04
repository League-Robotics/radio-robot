#!/usr/bin/env python3
"""turn_to.py — turn the robot to an ABSOLUTE WORLD heading (degrees, camera
frame: 0deg = +x/east, CCW positive).

Design: the robot's onboard odometry heading is relative to its power-on/OTOS
origin, so an "absolute" onboard angle is not a world angle. So we:

  1. Read the robot's true world pose ONCE from the overhead camera (AprilTag
     100) and SEED the robot's onboard pose with it (SI command -> Odometry::
     setPose + OTOS re-anchor).  After this, the onboard heading IS the world
     heading.
  2. TURN to the requested absolute world heading using the robot's ODOMETER
     (the firmware TURN/heading controller).  The camera is NOT used to drive
     the turn — the robot runs on its own.
  3. (Optional) read the camera again afterwards to REPORT the world error.
     This is verification only; it does not affect control.

    uv run python tests/_infra/calibrate/turn_to.py --to 90
    uv run python tests/_infra/calibrate/turn_to.py --to -45 --eps 0.5 -v
    uv run python tests/_infra/calibrate/turn_to.py --to 0 --no-verify
"""
from __future__ import annotations

import argparse
import math
import statistics
import sys
import time

from robot_radio.robot.nezha import Nezha
from robot_radio.robot.protocol import NezhaProtocol
from robot_radio.io.serial_conn import SerialConnection, list_serial_ports

ROBOT_TAG = 100


class _Cam:
    """Overhead camera ground truth from the aprilcam daemon, with retry."""

    def __init__(self, tag_id: int = ROBOT_TAG):
        self.tag_id = tag_id
        from aprilcam.client.control import DaemonControl
        from aprilcam.config import Config
        self.dc = DaemonControl.connect_default(Config.load())
        cams = self.dc.list_cameras()
        if not cams:
            raise SystemExit("aprilcam: no cameras open")
        c0 = cams[0]
        self.cam = c0 if isinstance(c0, str) else getattr(c0, "id", c0)

    def pose(self, samples: int = 6, settle: float = 0.05):
        """Median (x_cm, y_cm, yaw_rad) of the robot tag, or None if unseen."""
        xs, ys, yaws = [], [], []
        attempts = 0
        while len(xs) < samples and attempts < samples * 4:
            attempts += 1
            try:
                tf = self.dc.get_tags(self.cam)
                for t in tf.tags:
                    if t.id == self.tag_id and getattr(t, "world_xy", None) is not None:
                        xs.append(float(t.world_xy[0]))
                        ys.append(float(t.world_xy[1]))
                        yaws.append(float(t.yaw))
            except Exception:
                pass
            time.sleep(settle)
        if not xs:
            return None
        cy = math.atan2(statistics.fmean(math.sin(v) for v in yaws),
                        statistics.fmean(math.cos(v) for v in yaws))
        return (statistics.median(xs), statistics.median(ys), cy)

    def close(self):
        try:
            self.dc.close()
        except Exception:
            pass


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--to", type=float, required=True,
                    help="absolute WORLD target heading in degrees "
                         "(0=+x/east, CCW+; wrapped to ±180)")
    ap.add_argument("--eps", type=float, default=1.0,
                    help="arrival tolerance in degrees (default 1.0)")
    ap.add_argument("--rate", type=float, default=None,
                    help="yaw-rate ceiling in deg/s for this run (SET yawRateMax; "
                         "default leaves the robot's config, ~35). Reverts on reboot.")
    ap.add_argument("--accel", type=float, default=None,
                    help="yaw acceleration limit in deg/s^2 (SET yawAccMax)")
    ap.add_argument("--port", default=None)
    ap.add_argument("--no-verify", action="store_true",
                    help="skip the post-turn camera verification read")
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="print serial traffic: '>>' sent, '<<' received")
    a = ap.parse_args(argv)

    wrap = lambda d: (d + 18000) % 36000 - 18000   # centidegrees, ±180°

    port = a.port or (list_serial_ports() or [None])[0]
    if port is None:
        print("no serial port"); return 2
    tx = (lambda s: print(f"  >> {s}", flush=True)) if a.verbose else None
    rx = (lambda s: print(f"  << {s}", flush=True)) if a.verbose else None

    cam = _Cam()
    conn = SerialConnection(port=port, on_send=tx, on_recv=rx); conn.connect()
    proto = NezhaProtocol(conn); robot = Nezha(proto)
    print("connected:", robot.connect().get("fw"))

    # Optional: raise the yaw-rate/accel ceilings for this run (live, reverts on
    # reboot). Faster turns slip/overshoot more, so accuracy may drop.
    cfg = {}
    if a.rate is not None:
        cfg["yawRateMax"] = a.rate
    if a.accel is not None:
        cfg["yawAccMax"] = a.accel
    if cfg:
        robot.set_config(**cfg)
        print("set        : " + ", ".join(f"{k}={v}" for k, v in cfg.items()))

    def _snap(tries=6):
        """SNAP with retry (relay framing can corrupt a lone SNAP)."""
        for _ in range(tries):
            t = proto.snap()
            if t and t.pose:
                return t
            time.sleep(0.3)
        return None

    try:
        # 1. Seed onboard pose from the camera (world fix) -----------------
        cpose = cam.pose()
        if cpose is None:
            print("camera: AprilTag 100 not seen — cannot seed pose."); return 1
        x_cm, y_cm, yaw_rad = cpose
        print(f"camera fix : x={x_cm:.1f}cm y={y_cm:.1f}cm "
              f"heading={math.degrees(yaw_rad):+.1f}° (world)")
        robot.update_world_pose(x_cm, y_cm, yaw_rad)   # SI -> seed odometry
        time.sleep(0.4)
        t0 = _snap()
        h0 = t0.pose[2] if (t0 and t0.pose) else None
        if h0 is None:
            print("no onboard heading snap after seeding"); return 1
        print(f"seeded     : onboard heading now {h0/100:+.1f}° "
              f"(should match camera {math.degrees(yaw_rad):+.1f}°)")

        # 2. TURN to absolute world heading on ODOMETRY -------------------
        tgt = int(round(wrap(a.to * 100)))
        eps_cdeg = max(1, int(round(a.eps * 100)))
        print(f"turning    : -> {tgt/100:+.1f}° world (Δ {wrap(tgt - h0)/100:+.1f}°, "
              f"eps {eps_cdeg/100:.2f}°), on odometry")
        proto.turn(tgt, eps=eps_cdeg)
        proto.wait_for_evt_done("TURN", 12000)
        proto.stop(); time.sleep(0.8)

        t1 = _snap()
        h1 = t1.pose[2] if (t1 and t1.pose) else None
        if h1 is not None:
            print(f"onboard    : heading {h1/100:+.1f}°  "
                  f"(odometry err {wrap(h1 - tgt)/100:+.2f}° from target)")

        # 3. Camera verification (report only; not used for control) ------
        if not a.no_verify:
            cpose2 = cam.pose()
            if cpose2 is not None:
                world = math.degrees(cpose2[2])
                err = wrap(int(round(world * 100)) - tgt) / 100.0
                print(f"camera vfy : world heading {world:+.1f}°  "
                      f"err {err:+.2f}° from target {tgt/100:+.1f}°")
        return 0
    finally:
        try: proto.stop()
        except Exception: pass
        conn.disconnect()
        cam.close()


if __name__ == "__main__":
    sys.exit(main())
