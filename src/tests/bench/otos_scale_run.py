#!/usr/bin/env python3
"""One long straight leg with the OTOS scale set to 1.0, for tape calibration.

Sets linear_scale to unity (so the chip reports its RAW optical measurement),
seeds the world pose from the camera, drives one long leg, and prints what
each of the three references says the robot travelled:

    commanded  -- what we asked the planner for
    OTOS       -- raw optical flow, scale 1.0
    camera     -- the overhead playfield fix

The fourth reference is a tape measure, read by a human. With all four, the
scale error can be attributed to the sensor that actually has it instead of
being split between them by assumption.

    uv run python src/tests/bench/otos_scale_run.py --distance 1000
"""
from __future__ import annotations

import argparse
import math
import time

RELAY_PORT = "/dev/cu.usbmodem214102"
CAMERA = "arducam-ov9782-usb-camera"
ROBOT_TAG = 100

CRUISE = 150.0        # [mm/s]
PIVOT_OMEGA = 1.6     # [rad/s]
YAW_SIGN = +1.0       # 2026-08-13: commanded omega is now REP-103 CCW-positive,
                      # i.e. it INCREASES camera yaw. Was -1.0, a workaround for
                      # tovez's swapped drive-motor labelling (port 1 was its RIGHT
                      # wheel); fixed at source via motors.left_port/right_port.
SETTLE = 1.6          # [s] rest dwell before every fix

FENCE_X, FENCE_Y = 640.0, 420.0   # [mm] inside the 671/446 table half-widths


def camera_pose(dc, tries: int = 30):
    """(x_mm, y_mm, yaw_rad) of the robot's centre of rotation, or None."""
    for _ in range(tries):
        tf = dc.get_tags(CAMERA)
        for t in getattr(tf, "tags", []):
            if t.id == ROBOT_TAG and getattr(t, "world_xy", None) is not None:
                return (float(t.world_xy[0]) * 10.0,
                        float(t.world_xy[1]) * 10.0,
                        float(t.yaw))
        time.sleep(0.12)
    return None


class Robot:
    def __init__(self, port: str):
        from robot_radio.io.serial_conn import SerialConnection
        from robot_radio.robot.protocol import NezhaProtocol
        self.conn = SerialConnection(port=port, mode="relay")
        self.conn.connect()
        self.p = NezhaProtocol(self.conn)
        self._id = int(time.time()) % 600000 + 400000

    def pose(self, tries: int = 8):
        """OTOS (x, y, heading) -- replies claiming no OTOS carry no pose."""
        for _ in range(tries):
            for line in self.conn.send_cleartext("POSE", read_timeout=700):
                if not line.startswith("POSE:"):
                    continue
                f = line[5:].split(":")
                if len(f) >= 7 and f[6] == "1":
                    return (float(f[0]), float(f[1]), float(f[2]) / 1000.0)
        return None

    def set_linear_scale(self, value: float):
        """Push the scalar and read it back -- an ack is not evidence."""
        from robot_radio.robot.pb2 import robot_config_pb2 as pb
        self.p.set_config_field(pb.OTOS, "linear_scale", value)
        time.sleep(0.8)
        for _ in range(4):
            snap = self.p.get_config_snapshot(pb.OTOS)
            if snap is not None:
                return snap.values.linear_scale
            time.sleep(0.4)
        return None

    def seed(self, x: float, y: float, heading: float) -> bool:
        self.conn.send_cleartext(f"SEED:{x:.1f},{y:.1f},{heading:.5f}",
                                 read_timeout=900)
        time.sleep(0.6)
        got = self.pose()
        return got is not None and math.hypot(got[0] - x, got[1] - y) < 20.0

    def pivot(self, degrees: float) -> None:
        self._id += 1
        omega = math.copysign(PIVOT_OMEGA, degrees * YAW_SIGN)
        self.p.move_twist(0.0, 0.0, omega, stop_angle=math.radians(abs(degrees)),
                          timeout=16000.0, move_id=self._id)
        time.sleep(abs(degrees) / math.degrees(PIVOT_OMEGA) + 2.5)

    def straight(self, distance: float) -> None:  # [mm]
        self._id += 1
        self.p.move_twist(CRUISE, 0.0, 0.0, stop_distance=abs(distance),
                          timeout=40000.0, move_id=self._id)
        time.sleep(abs(distance) / CRUISE + 3.0)

    def close(self) -> None:
        try:
            self.p.estop()
        except Exception:
            pass
        try:
            self.conn.disconnect()
        except Exception:
            pass


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--distance", type=float, default=1000.0, help="[mm]")
    ap.add_argument("--scale", type=float, default=1.0, help="OTOS linear_scale")
    ap.add_argument("--pivot-first", type=float, default=0.0,
                    help="[deg] turn this much before the leg")
    ap.add_argument("--port", default=RELAY_PORT)
    args = ap.parse_args()

    from aprilcam.config import Config
    from aprilcam.client.control import DaemonControl
    dc = DaemonControl.connect_default(Config.load())

    bot = Robot(args.port)
    try:
        readback = bot.set_linear_scale(args.scale)
        print(f"linear_scale pushed {args.scale} -> robot reports {readback}")
        if readback is None or abs(readback - args.scale) > 0.002:
            print("scale did not take -- stopping rather than measuring a lie")
            return 1

        if args.pivot_first:
            print(f"pivoting {args.pivot_first:.0f}deg first ...")
            bot.pivot(args.pivot_first)
            time.sleep(SETTLE)

        start_cam = camera_pose(dc)
        if start_cam is None:
            print("camera cannot see tag 100 -- are the lights on?")
            return 1
        if not bot.seed(*start_cam):
            print("seed did not read back -- stopping")
            return 1
        time.sleep(0.5)
        start_otos = bot.pose()

        end_x = start_cam[0] + args.distance * math.cos(start_cam[2])
        end_y = start_cam[1] + args.distance * math.sin(start_cam[2])
        print(f"start   camera ({start_cam[0]:.1f}, {start_cam[1]:.1f}) "
              f"heading {math.degrees(start_cam[2]):.1f}deg")
        print(f"        otos   ({start_otos[0]:.1f}, {start_otos[1]:.1f})")
        print(f"predicted end   ({end_x:.1f}, {end_y:.1f})")
        if abs(end_x) > FENCE_X or abs(end_y) > FENCE_Y:
            print(f"that leg exits the fence (+-{FENCE_X:.0f}, +-{FENCE_Y:.0f}) -- refusing")
            return 1

        print(f"\ndriving {args.distance:.0f}mm ...")
        bot.straight(args.distance)
        time.sleep(SETTLE)

        end_cam = camera_pose(dc)
        end_otos = bot.pose()
        if end_cam is None or end_otos is None:
            print("lost a fix at the end of the leg")
            return 1

        cam_d = math.dist(start_cam[:2], end_cam[:2])
        otos_d = math.dist(start_otos[:2], end_otos[:2])

        print(f"\nend     camera ({end_cam[0]:.1f}, {end_cam[1]:.1f}) "
              f"heading {math.degrees(end_cam[2]):.1f}deg")
        print(f"        otos   ({end_otos[0]:.1f}, {end_otos[1]:.1f})")
        print()
        print(f"  commanded : {args.distance:8.1f} mm")
        print(f"  OTOS      : {otos_d:8.1f} mm   (linear_scale {args.scale})")
        print(f"  camera    : {cam_d:8.1f} mm")
        print(f"  TAPE      :      ?   mm   <-- measure this")
        print()
        print(f"  otos/camera        = {otos_d / cam_d:.4f}")
        print(f"  camera/commanded   = {cam_d / args.distance:.4f}")
        print(f"  otos/commanded     = {otos_d / args.distance:.4f}")
        print()
        print("  linear_scale to make OTOS match, once tape is known:")
        print(f"    vs camera : {args.scale * cam_d / otos_d:.4f}")
        print(f"    vs tape   : {args.scale:.4f} * tape_mm / {otos_d:.1f}")
    finally:
        bot.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
