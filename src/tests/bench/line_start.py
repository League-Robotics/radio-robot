#!/usr/bin/env python
"""Drive the robot back to the start of the line course and square it up.

This is a POSITIONING move, not line following. It is the one part of the
line-follow workflow that legitimately uses the camera: line_follow.py needs
the robot already straddling the line, and a reflectance array cannot find a
line it cannot see. Once this hands off, the camera is done.

It follows .claude/rules/playfield-testing.md's positioning procedure --
camera fix, seed, capped legs with a re-fix between hops, fence checked
inside every move -- rather than driving the whole way in one open leg.

WHERE THE LINE IS (surveyed 2026-08-08 via pixel_to_world against the
calibrated playfield homography, spot-checked by confirming the robot tag's
own pixel projected to within 1mm of the world_xy get_tags reported):

    the course's top segment runs at y = +16cm, from its east end at
    x = +57.5cm westward past x = +19cm where it starts to curve north.

The array sits ARRAY_LEVER (96mm) FORWARD of the centre of rotation, and
tag 100 reports the CENTRE, so a robot centred at x facing west has its
array at x - 9.6cm. TARGET is chosen so both the centre and the array land
on the straight part of the top segment.

Usage:
    uv run python src/tests/bench/line_start.py --port /dev/cu.usbmodemXXXX

Then hand off to the follower, which needs no camera at all:
    uv run python src/tests/bench/line_follow.py --port ... --speed 70
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time

sys.path.insert(0, "src/tests/bench")

from field_map import (OUT, camera_fix, connect_camera, halt_verified,  # noqa: E402
                       read_tag, seed_retry, undo_parallax, wait_for_goto, wrap)
from line_follow import LINE_THRESHOLD, describe, line_error  # noqa: E402
from robot_radio.robot.protocol import TLMFrame  # noqa: E402

TARGET = (52.0, 16.2)       # [cm] on the top segment, room to run west
HEADING = math.pi           # [rad] west, along the course
HOP = 22.0                  # [cm] max distance per leg -- re-fix between
# [cm] hand off from GO_TO to the fine approach. This must be WIDER than
# Motion::Navigator's own arrival tolerance, or GO_TO reports arrival and
# stops moving while still short -- see fine_approach's docstring.
ARRIVE = 12.0
FENCE = (62.0, 42.0)        # [cm] positioning fence, checked inside each move
# [cm] lateral steps to try if the array does not see the line on arrival.
# The line is ~5cm wide, so a miss means the y estimate is off by more than
# half that; searching +/-4.5cm in 1.5cm steps covers the plausible error.
HUNT = (0.0, 1.5, -1.5, 3.0, -3.0, 4.5, -4.5)


def fix(dc, cal):
    raw = read_tag(dc)
    if raw is None:
        return None
    x, y = undo_parallax(raw[0], raw[1], cal["k"], cal["cx"], cal["cy"])
    return (x, y, raw[2])


def read_array(bot, seconds=1.2):
    """Median-ish look at the line array while parked."""
    bot.conn.drain_binary_tlm()
    rows = []
    t0 = time.time()
    while time.time() - t0 < seconds:
        for env in bot.conn.drain_binary_tlm():
            t = getattr(env, "tlm", None)
            if t is None:
                continue
            f = TLMFrame.from_pb2(t)
            if f.line_present and f.line:
                rows.append(tuple(f.line))
        time.sleep(0.02)
    if not rows:
        return None
    return tuple(sorted(r[i] for r in rows)[len(rows) // 2] for i in range(4))


def fine_approach(bot, dc, cal, tx, ty, speed=70.0):
    """Close the last few cm with an aimed, distance-bounded straight leg.

    GO_TO cannot do this. Motion::Navigator's arrival tolerance is wider than
    the remaining distance, so it reports arrival and never moves -- measured
    here as six identical camera fixes 6.7cm from the target, and previously
    as the same stall in field_map's recover_to_centre. A Move bounded by
    stop_distance has no such tolerance: it runs the arc length it is given.
    """
    from field_map import align_heading

    for _ in range(4):
        here = fix(dc, cal)
        if here is None:
            return False
        dx, dy = tx - here[0], ty - here[1]
        dist = math.hypot(dx, dy)
        print(f"    fine: at ({here[0]:+.1f},{here[1]:+.1f})cm, {dist:.1f}cm out")
        if dist <= 1.2:
            return True
        if not align_heading(bot, math.atan2(dy, dx), tol=0.06,
                             fix_fn=lambda: fix(dc, cal)):
            print("    could not square up onto the bearing")
            return False
        halt_verified(bot)
        mid = bot._next_id()
        bot.p.move_twist(speed, 0.0, 0.0, stop_distance=dist * 10.0,
                         timeout=dist * 10.0 / speed * 3000.0, move_id=mid)
        done, breach = wait_for_goto(bot, mid, timeout=20.0, fence=FENCE,
                                     hard_fence=FENCE)
        halt_verified(bot)
        if breach is not None:
            print(f"    FENCE breach at ({breach[0]:+.1f},{breach[1]:+.1f})cm")
            return False
        if not done:
            print("    leg never completed")
            return False
    return True


def drive_to(bot, dc, cal, tx, ty, speed):
    """Hop toward (tx,ty), re-fixing between legs. True if it arrived."""
    from goto_otos import FRAME_WORLD

    for hop in range(8):
        here = fix(dc, cal)
        if here is None:
            print("  camera cannot see tag 100 -- stopping rather than "
                  "driving unsupervised")
            return False
        dx, dy = tx - here[0], ty - here[1]
        dist = math.hypot(dx, dy)
        print(f"  at ({here[0]:+.1f},{here[1]:+.1f})cm, {dist:.1f}cm to go")
        if dist <= ARRIVE:
            return True
        if not seed_retry(bot, here[0], here[1], here[2]):
            print("  SEED never read back -- refusing to GO_TO on a pose the "
                  "robot may not have")
            return False
        f = min(1.0, HOP / dist)
        wx, wy = here[0] + dx * f, here[1] + dy * f
        _c, gid = bot.goto_wire(wx * 10.0, wy * 10.0, FRAME_WORLD,
                                speed=speed, timeout=40000.0)
        done, breach = wait_for_goto(bot, gid, timeout=40.0, fence=FENCE,
                                     hard_fence=FENCE)
        if breach is not None:
            print(f"  FENCE breach at ({breach[0]:+.1f},{breach[1]:+.1f})cm")
            return False
        if not done:
            print("  GO_TO never completed")
            return False
        time.sleep(0.5)
    return False


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True)
    ap.add_argument("--speed", type=float, default=120.0, help="[mm/s]")
    args = ap.parse_args()

    from goto_otos import Robot

    cal = json.loads((OUT / "field_parallax.json").read_text())
    dc = connect_camera()
    bot = Robot(args.port)   # connects and enables telemetry in __init__
    try:
        print(f"driving to the line start {TARGET} facing west")
        for k, dy in enumerate(HUNT):
            tx, ty = TARGET[0], TARGET[1] + dy
            if k:
                print(f"array did not see the line -- trying y {ty:+.1f}cm "
                      f"({k}/{len(HUNT) - 1})")
            if not drive_to(bot, dc, cal, tx, ty, args.speed):
                return 2
            halt_verified(bot)
            if not fine_approach(bot, dc, cal, tx, ty):
                return 2

            here = fix(dc, cal)
            if here is not None:
                err = wrap(HEADING - here[2])
                print(f"  heading off by {math.degrees(err):+.1f} deg")
                from field_map import align_heading
                align_heading(bot, HEADING, tol=0.05, fix_fn=lambda: fix(dc, cal))
                halt_verified(bot)

            vals = read_array(bot)
            if vals is None:
                print("  no line telemetry")
                return 2
            e = line_error(vals)
            print(f"  array reads {describe(vals)}  {vals}  "
                  f"err={'LOST' if e is None else f'{e:+.1f}mm'}")
            if e is not None:
                here = fix(dc, cal)
                print(f"\nON THE LINE at ({here[0]:+.1f},{here[1]:+.1f})cm, "
                      f"heading {math.degrees(here[2]):+.1f} deg.")
                print("hand off:  uv run python src/tests/bench/line_follow.py "
                      f"--port {args.port} --speed 70")
                return 0

        print("\ncould not find the line anywhere in the search band -- "
              "place the robot on it by hand")
        return 1
    finally:
        halt_verified(bot)
        bot.close()


if __name__ == "__main__":
    raise SystemExit(main())
