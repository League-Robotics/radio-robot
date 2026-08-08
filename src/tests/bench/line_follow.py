#!/usr/bin/env python
"""Follow the line on the KIPR line mat using ONLY the reflectance array.

Stakeholder directive 2026-08-08: "your goal here is to follow the line
without using the camera. You're just using the reflectance sensors."

Nothing here uses the camera. Steering, corner recovery, crossing rejection
and the off-mat stop all come from the four line channels alone. A camera
safety halt is available behind --fence but is OFF by default and never
steers -- see that flag's help for why it is opt-in (it false-halted a good
run when tag 1 left the frame and world coordinates silently re-based).

CALIBRATION (measured on tovez 2026-08-08, robot parked straddling the line
with the middle two sensors on it and the outer two off -- stakeholder-
confirmed ground truth for that pose):

    ch1 (leftmost)   71-89     OFF
    ch2             255        ON
    ch3             255        ON
    ch4 (rightmost)  39-93     OFF

On-line saturates the ADC at 255; off-line never exceeded 93 over 184
samples. A threshold anywhere in 100..240 separates them, so LINE_THRESHOLD
is set mid-gap at 170 and no per-run calibration sweep is needed. Note the
polarity: HIGH means DARK means ON the line (the array reads inverted --
see field_map.py's render comment, and the mat scan where bare wood read
~240 against white vinyl at ~22).

GEOMETRY: the four channels sit at body y = +32/+8/-8/-32 mm
(data/robots/tovez.json perception.line_array, channel order VERIFIED
2026-08-08: ch1 leftmost, ch4 rightmost). Body y is LEFT-positive, so a
positive error means the line is to the robot's LEFT.

CONTROL: proportional on the coverage-weighted centroid of the four
channels, with the forward speed cut as the error grows -- slow down for
corners, run out on the straights -- and omega bounded by the array lever
arm (see ARRAY_LEVER). When every channel reads bare mat the line has been
lost, and the recovery rotates in place toward the side it was last seen.

CROSSING LINES: the mat carries lines that cross the course -- coloured
markers ("Line A"/"Line B"/"Line D") and several black ones near the finish.
A reflectance array CANNOT tell a black crossing line from the brown course
line: both are dark, and the sensor has no colour information at all. The
discriminator is therefore geometric, not chromatic: how WIDE the dark
region under the array is. The course line is ~50mm wide against a 64mm
array span (MEASURED -- see CROSS_LIT; an earlier guess of ~36mm off a
camera frame was wrong and cost a whole run), so it lights up to three
channels and never four. All four lit is a crossing. On a crossing the
follower holds its last good steering and drives straight through rather
than steering on it; steering on a crossing is exactly how a line follower
turns down the wrong line. The same rule covers the coloured markers for
free, and it is bounded in time so that sitting on a large dark area stops
the robot instead of latching a stale command.

Usage:
    uv run python src/tests/bench/line_follow.py --port /dev/cu.usbmodemXXXX
    uv run python src/tests/bench/line_follow.py --port ... --speed 90
    uv run python src/tests/bench/line_follow.py --port ... --report  # sensors only, no motion

The robot does NOT need a healthy gyro for this: the follower never reads
heading or pose, so a poisoned OTOS bias (see gyro_recal.py) cannot make it
drive crooked. It only matters if you arm --fence, whose pose would then be
fiction.

Start the robot straddling the line -- middle two channels dark, outer two
on bare mat -- pointing the way you want it to go. The follower has no idea
which end of the line is the start.
"""
from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys
import time

sys.path.insert(0, "src/tests/bench")

from robot_radio.io.serial_conn import SerialConnection  # noqa: E402
from robot_radio.robot.protocol import NezhaProtocol, TLMFrame  # noqa: E402

# [counts] a channel reading at or below this is looking at bare white mat.
# Measured floor is 0-15; anything above is partial or full line coverage.
OFF_LEVEL = 40
# [counts] fully-lit threshold, used only for counting how WIDE a reading is
# (the crossing test), never for steering.
LINE_THRESHOLD = 170

# [mm] lateral offset of each channel from the centre line, ch1 first.
# Left-positive, matching the robot JSON and body-frame convention.
CHANNEL_Y = (32.0, 8.0, -8.0, -32.0)

# [mm] the line array sits this far FORWARD of the centre of rotation
# (tovez.json perception.line_array.x), and MIN_RADIUS is the tightest turn
# the follower will ask for. Bounding omega at speed/MIN_RADIUS keeps the
# array from being swept sideways off the line faster than the robot drives
# forward -- outrunning its own sensor is what produced 32% blind samples in
# the first run. 90mm is tighter than the hairpin's centreline radius.
ARRAY_LEVER = 96.0
MIN_RADIUS = 90.0

# [counts] minimum total coverage across all four channels before the
# centroid is believed. Without this, two channels grazing the line at 50
# counts each produce a confident-looking error that flips sign on noise --
# observed jumping +32.0 to -32.0 between consecutive samples, which is the
# full width of the array and physically impossible in 40ms.
MIN_SIGNAL = 60.0

# Steering.
KP = 0.030
KD = 0.004              # small: the error is quantised and D amplifies steps

# Forward speed shaping: full speed centred, SLOW_FLOOR of it at max error.
# The floor cannot go low: omega is capped at speed/MIN_RADIUS, so cutting
# the speed also cuts the turn authority, and the FIRST version of this cut
# it hardest exactly when the error was largest -- the robot sat pinned at
# err=+32mm for seven consecutive samples, turning at 0.29 rad/s, unable to
# recover the line it could still see.
SLOW_FLOOR = 0.70

# Lost-line recovery, in two phases. Never drive FORWARD while blind -- that
# is how the robot leaves the course entirely.
#
# Phase 1, REVERSE (first LOST_BACK_TIME): retrace. The line is almost always
# BEHIND the array, because the way it gets lost is the robot driving past a
# corner too sharp to turn into. Rotating in place cannot reach it: the array
# sweeps a circle of radius ARRAY_LEVER (96mm), so a line further back than
# that is unreachable no matter how long the robot spins. Backing up along
# the path just travelled puts the array over the line again directly.
#
# Phase 2, ROTATE: only if reversing did not find it, sweep toward the side
# the line was last seen. Measured 2026-08-08 with rotation alone: two
# separate 5.6s blind stretches in one 75s run, 23% of all samples.
LOST_BACK_TIME = 0.9    # [s]
LOST_BACK_SPEED = 0.45  # fraction of nominal, negative direction
LOST_OMEGA = 0.45       # [rad/s]
# [s] of continuous loss before stopping. This is a SPACE budget as much as a
# time one: 0.9s reversing plus ~2.6s of rotation sweeps the array through
# roughly 120 degrees, which recovers a line the robot overshot at a corner.
# Anything longer is no longer searching, it is wandering -- and at the END OF
# THE COURSE there is no line left to find, so a generous budget just drives
# the robot off the mat looking for it (measured 2026-08-08: with a 6.0s
# budget the robot ran past the finish and off the paper's edge). Reaching
# this limit after a stretch of good tracking means the line ended.
LOST_GIVE_UP = 3.5

# Crossing rejection. MEASURED 2026-08-08 over a 1279-sample run: the course
# line routinely lights THREE channels (`###.` was 18.5% of all samples, the
# second most common reading), because it is ~50mm wide against a 64mm array
# span -- not the ~36mm this file first assumed. Only FOUR lit channels is
# geometrically impossible for the course line, and in that run all-four
# occurred in exactly one episode. So the crossing test is all four, and it
# is bounded: a genuine crossing is swept in a fraction of a second, whereas
# sustained all-four means the robot is parked on something big and dark
# (the thick start bar, or bare wood off the mat edge, which reads ~240) and
# holding a stale steering command there is a runaway, not a recovery.
CROSS_LIT = 4
CROSS_HOLD = 0.15       # [s] latch past the trailing edge
CROSS_MAX = 1.2         # [s] beyond this it is not a crossing -- stop

# Command cadence. Each twist carries stop_time as its own deadman, so a
# host that dies or a dropped command stops the robot rather than leaving a
# twist running.
CMD_PERIOD = 0.09       # [s]
CMD_STOP_TIME = 400.0   # [ms]

# Camera SAFETY fence only -- never steering. Generous: it exists to stop a
# runaway leaving the table, not to keep the robot on the line.
FENCE_X, FENCE_Y = 62.0, 40.0   # [cm]


def read_line(frame):
    """Four raw channels, or None if this frame carries no line reading."""
    if not frame.line_present or not frame.line:
        return None
    return tuple(frame.line)


def line_error(vals):
    """Centroid of the line under the array [mm], left-positive; None if lost.

    Weighted by how much of each channel the line covers, not by a bare
    on/off test. A fully-covered channel saturates at 255 and a bare-mat
    channel floors near 0, but a channel the line PARTLY covers reads in
    between, and that intermediate value says where the edge falls between
    two channels. The first version of this used a binary centroid, which
    could only ever produce seven distinct values -- the resulting staircase
    was what made the derivative term explode (an 8.2 rad/s command was
    observed on a single-step error change).
    """
    w = [max(0.0, float(v) - OFF_LEVEL) for v in vals]
    total = sum(w)
    if total < MIN_SIGNAL:
        return None
    return sum(y * wi for y, wi in zip(CHANNEL_Y, w)) / total


def describe(vals):
    return "".join("#" if v >= LINE_THRESHOLD else "." for v in vals)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True)
    ap.add_argument("--speed", type=float, default=110.0, help="[mm/s] nominal")
    ap.add_argument("--seconds", type=float, default=180.0, help="[s] run limit")
    ap.add_argument("--report", action="store_true",
                    help="print sensor state continuously and DO NOT drive")
    ap.add_argument("--fence", action="store_true",
                    help="arm the camera safety halt. OFF by default -- it "
                         "false-halted a good run on 2026-08-08 when tag 1 "
                         "went out of view and the daemon silently reframed "
                         "world coordinates to a corner origin, moving the "
                         "fence out from under the robot. The reflectance "
                         "off-mat guard (sustained all-four-dark = bare "
                         "wood) does the same job and cannot be blinded by "
                         "a lost tag.")
    args = ap.parse_args()

    fix_fn = None
    if args.fence and not args.report:
        try:
            from field_map import OUT, connect_camera, read_tag, undo_parallax
            cal = json.loads((OUT / "field_parallax.json").read_text())
            K, CX, CY = cal["k"], cal["cx"], cal["cy"]
            dc = connect_camera()

            def fix_fn():  # noqa: F811
                raw = read_tag(dc)
                if raw is None:
                    return None
                x, y = undo_parallax(raw[0], raw[1], K, CX, CY)
                return (x, y, raw[2])
            print("camera safety fence armed (halt only -- never steers)")
        except Exception as e:
            print(f"camera fence unavailable ({e}) -- continuing without it")
            fix_fn = None

    out = pathlib.Path(__file__).resolve().parent / "output"
    out.mkdir(exist_ok=True)
    csv = None if args.report else (out / "line_follow.csv").open("w")
    if csv is not None:
        csv.write("t,ch1,ch2,ch3,ch4,state,err,speed,omega\n")

    conn = SerialConnection(port=args.port)
    conn.connect(skip_ping=False)
    p = NezhaProtocol(conn)
    try:
        conn.send_cleartext("TLM:ON")
        time.sleep(0.6)
        conn.drain_binary_tlm()

        last_err = 0.0
        last_omega = 0.0
        last_seen_sign = -1.0     # which way the line went when last seen
        lost_since = None
        cross_until = 0.0
        cross_start = 0.0
        track_time = 0.0
        crossings = 0
        cross_log: list[tuple[float, str]] = []
        last_cmd = 0.0
        last_cam = 0.0
        t0 = time.time()
        printed = 0.0
        last_frame = time.time()
        frames = 0

        while time.time() - t0 < args.seconds:
            vals = None
            for env in conn.drain_binary_tlm():
                t = getattr(env, "tlm", None)
                if t is None:
                    continue
                v = read_line(TLMFrame.from_pb2(t))
                if v is not None:
                    vals = v
                    frames += 1
            if vals is None:
                time.sleep(0.01)
                continue

            now = time.time()
            dt = now - last_frame
            nlit = sum(1 for v in vals if v >= LINE_THRESHOLD)
            err = line_error(vals)

            if nlit >= CROSS_LIT:
                if cross_until < now:      # rising edge of a new crossing
                    crossings += 1
                    cross_start = now
                    cross_log.append((now - t0, describe(vals)))
                cross_until = now + CROSS_HOLD
            crossing = now < cross_until

            if crossing and now - cross_start > CROSS_MAX:
                print(f"\nall four channels dark for {now - cross_start:.1f}s "
                      f"-- this is not a crossing, the robot is parked on "
                      f"something wide and dark. Stopping.")
                break

            if crossing:
                # Another line is over the array. Its centroid says nothing
                # about where OUR line goes, so ignore it and hold course.
                state = "CROSS"
                omega = last_omega
                speed = args.speed
            elif err is None:
                state = "LOST"
                if lost_since is None:
                    lost_since = now
                elif now - lost_since > LOST_GIVE_UP:
                    # Reaching the search budget after a good run of tracking
                    # means the line ran out under us -- the end of the course
                    # -- not that the follower lost a line that is still there.
                    if track_time > 5.0:
                        print(f"\nno line found in {LOST_GIVE_UP:.1f}s of "
                              f"searching, after {track_time:.0f}s of "
                              f"tracking -- the line ended. Stopping here.")
                    else:
                        print(f"\nline lost for {LOST_GIVE_UP:.1f}s and the "
                              f"search did not recover it -- stopping")
                    break
                if now - lost_since < LOST_BACK_TIME:
                    # Retrace: the line is behind us, not beside us.
                    speed = -args.speed * LOST_BACK_SPEED
                    omega = 0.0
                else:
                    omega = LOST_OMEGA * last_seen_sign
                    speed = 0.0
            else:
                state = "TRACK"
                track_time += min(dt, 0.5)
                lost_since = None
                if abs(err) > 1e-6:
                    last_seen_sign = -1.0 if err > 0 else 1.0
                d = (err - last_err) / CMD_PERIOD
                last_err = err
                # Positive omega turns the robot RIGHT (decreases heading,
                # .claude/rules/playfield-testing.md); a positive error means
                # the line is to the LEFT -- hence the negation.
                omega = -(KP * err + KD * d)
                frac = min(abs(err) / max(CHANNEL_Y), 1.0)
                speed = args.speed * (1.0 - (1.0 - SLOW_FLOOR) * frac)
                # The array is ARRAY_LEVER ahead of the centre of rotation,
                # so a turn sweeps it sideways at ARRAY_LEVER*omega. Once
                # that exceeds the forward speed the robot is outrunning its
                # own sensor -- it steers the line out from under the array
                # and goes blind, which is what produced 32% `....` readings
                # at omega=1.1 with v=85. Capping the ratio at 1 bounds the
                # turn radius at ARRAY_LEVER (96mm), still tighter than the
                # hairpin needs.
                omega_max = speed / MIN_RADIUS
                omega = max(-omega_max, min(omega_max, omega))
                last_omega = omega

            last_frame = now
            if csv is not None:
                csv.write(f"{now - t0:.3f},{vals[0]},{vals[1]},{vals[2]},"
                          f"{vals[3]},{state},"
                          f"{'' if err is None else f'{err:.1f}'},"
                          f"{speed:.0f},{omega:.3f}\n")

            if now - printed > 0.4:
                printed = now
                e_txt = "  --  " if err is None else f"{err:+6.1f}"
                print(f"  {describe(vals)}  {state:5s} err={e_txt}mm  "
                      f"v={speed:5.0f}  omega={omega:+5.2f}  "
                      f"crossings={crossings}", end="\r", flush=True)

            if args.report:
                time.sleep(0.05)
                continue

            if fix_fn is not None and now - last_cam > 0.5:
                last_cam = now
                c = fix_fn()
                if c is not None and (abs(c[0]) > FENCE_X or abs(c[1]) > FENCE_Y):
                    print(f"\nSAFETY HALT: robot at ({c[0]:+.1f},{c[1]:+.1f})cm")
                    break

            if now - last_cmd >= CMD_PERIOD:
                last_cmd = now
                p.move_twist(speed, 0.0, omega, stop_time=CMD_STOP_TIME,
                             timeout=CMD_STOP_TIME * 3, move_id=0)
            time.sleep(0.01)

        print(f"\nran {time.time() - t0:.0f}s, {frames} line frames, "
              f"{crossings} crossings")
        for t, pat in cross_log:
            print(f"    crossing at t={t:6.1f}s  first pattern {pat}")
    finally:
        if not args.report:
            for _ in range(3):
                try:
                    p.estop()
                except Exception:
                    pass
                time.sleep(0.25)
        if csv is not None:
            csv.close()
            print(f"wrote {out / 'line_follow.csv'}")
        conn.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
