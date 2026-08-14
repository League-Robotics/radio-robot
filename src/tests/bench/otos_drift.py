#!/usr/bin/env python3
"""Measure how well the OTOS holds world position against camera truth.

Seeds the chip once from the overhead camera, then drives a square and takes
BOTH fixes -- camera and OTOS -- at every segment boundary, at rest. The
divergence between them over the run is the drift, and it is the number that
decides whether the OTOS can carry navigation between camera fixes.

The OTOS is seeded ONCE, at the start. Nothing re-seeds it mid-run: that is
the point. A sensor that needs frequent camera updates has not replaced the
camera, it has just added a second one.

    uv run python src/tests/bench/otos_drift.py --legs 4 --leg 250
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
SETTLE = 1.4          # [s] rest dwell before every fix

FENCE_X, FENCE_Y = 620.0, 400.0   # [mm] inside the 671/446 table half-widths


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class Robot:
    """tovez over the radio relay: cleartext POSE/SEED plus binary Moves."""

    def __init__(self, port: str):
        from robot_radio.io.serial_conn import SerialConnection
        from robot_radio.robot.protocol import NezhaProtocol
        self.conn = SerialConnection(port=port, mode="relay")
        self.conn.connect()
        self.p = NezhaProtocol(self.conn)
        self._id = int(time.time()) % 600000 + 300000

    def _next_id(self) -> int:
        self._id += 1
        return self._id

    def pose(self, tries: int = 6):
        """(otos_x, otos_y, otos_heading, enc_x, enc_y, enc_heading, fresh).

        The trailing field is otos_present. A reply that says the OTOS is
        absent carries no pose, so it is not one -- keep reading.
        """
        for _ in range(tries):
            for line in self.conn.send_cleartext("POSE", read_timeout=700):
                if not line.startswith("POSE:"):
                    continue
                f = line[5:].split(":")
                if len(f) >= 7 and f[6] == "1":
                    return (float(f[0]), float(f[1]), float(f[2]) / 1000.0,
                            float(f[3]), float(f[4]), float(f[5]) / 1000.0, True)
        return None

    def seed(self, x: float, y: float, heading: float) -> bool:
        """Seed the chip, then confirm by reading it back.

        The ack says the command parsed; only the read-back says the chip
        actually holds the value (.claude/rules/configuration-discipline.md).
        """
        self.conn.send_cleartext(f"SEED:{x:.1f},{y:.1f},{heading:.5f}",
                                 read_timeout=900)
        time.sleep(0.5)
        got = self.pose()
        if got is None:
            return False
        return math.hypot(got[0] - x, got[1] - y) < 20.0

    def straight(self, distance: float) -> None:  # [mm]
        self.p.move_twist(CRUISE, 0.0, 0.0, stop_distance=abs(distance),
                          timeout=25000.0, move_id=self._next_id())
        time.sleep(abs(distance) / CRUISE + 2.0)

    def pivot(self, degrees: float) -> None:
        omega = math.copysign(PIVOT_OMEGA, degrees * YAW_SIGN)
        self.p.move_twist(0.0, 0.0, omega, stop_angle=math.radians(abs(degrees)),
                          timeout=14000.0, move_id=self._next_id())
        time.sleep(abs(degrees) / math.degrees(PIVOT_OMEGA) + 2.0)

    def halt(self) -> None:
        try:
            self.p.estop()
        except Exception:
            pass

    def close(self) -> None:
        self.halt()
        try:
            self.conn.disconnect()
        except Exception:
            pass


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


def fix(dc, bot, label: str):
    """One rest-to-rest boundary fix from both sources."""
    time.sleep(SETTLE)
    cam = camera_pose(dc)
    otos = bot.pose()
    if cam is None or otos is None:
        print(f"  {label:<10} FIX FAILED  cam={cam is not None} otos={otos is not None}")
        return None
    dx, dy = otos[0] - cam[0], otos[1] - cam[1]
    err = math.hypot(dx, dy)
    dh = math.degrees(wrap(otos[2] - cam[2]))
    print(f"  {label:<10} cam=({cam[0]:7.1f},{cam[1]:7.1f}) {math.degrees(cam[2]):7.1f}deg | "
          f"otos=({otos[0]:7.1f},{otos[1]:7.1f}) {math.degrees(otos[2]):7.1f}deg | "
          f"gap={err:6.1f}mm {dh:6.1f}deg")
    return {"label": label, "cam": cam, "otos": otos, "err": err, "dheading": dh}


def inside_fence(x: float, y: float) -> bool:
    return abs(x) < FENCE_X and abs(y) < FENCE_Y


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--legs", type=int, default=4)
    ap.add_argument("--leg", type=float, default=250.0, help="[mm] leg length")
    ap.add_argument("--turn", type=float, default=90.0, help="[deg] corner turn")
    ap.add_argument("--port", default=RELAY_PORT)
    args = ap.parse_args()

    from aprilcam.config import Config
    from aprilcam.client.control import DaemonControl
    dc = DaemonControl.connect_default(Config.load())

    bot = Robot(args.port)
    samples = []
    try:
        start = camera_pose(dc)
        if start is None:
            print("camera cannot see tag 100 -- are the lights on?")
            return 1
        print(f"seeding OTOS from camera: ({start[0]:.1f}, {start[1]:.1f}) "
              f"{math.degrees(start[2]):.1f}deg")
        if not bot.seed(*start):
            print("SEED was not acknowledged")
            return 1
        time.sleep(0.8)

        samples.append(fix(dc, bot, "start"))

        for i in range(args.legs):
            here = camera_pose(dc)
            if here is None:
                print("  lost the tag -- stopping")
                break
            ahead = (here[0] + args.leg * math.cos(here[2]),
                     here[1] + args.leg * math.sin(here[2]))
            if not inside_fence(*ahead):
                print(f"  leg {i+1} would exit the fence at "
                      f"({ahead[0]:.0f},{ahead[1]:.0f}) -- turning instead")
            else:
                bot.straight(args.leg)
                samples.append(fix(dc, bot, f"leg{i+1}"))
            bot.pivot(args.turn)
            samples.append(fix(dc, bot, f"turn{i+1}"))
    finally:
        bot.close()

    good = [s for s in samples if s]
    if len(good) < 2:
        print("not enough fixes to report")
        return 1

    print()
    print(f"OTOS-vs-camera gap: start {good[0]['err']:.1f}mm -> "
          f"end {good[-1]['err']:.1f}mm   (max {max(s['err'] for s in good):.1f}mm)")
    print(f"heading gap:        start {good[0]['dheading']:.1f}deg -> "
          f"end {good[-1]['dheading']:.1f}deg")
    travelled = sum(
        math.dist(good[i - 1]["cam"][:2], good[i]["cam"][:2])
        for i in range(1, len(good)))
    print(f"path length (camera): {travelled:.0f}mm over {len(good)} fixes")
    if travelled > 0:
        print(f"drift rate: {good[-1]['err'] / travelled * 100.0:.2f}mm per 100mm travelled")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
