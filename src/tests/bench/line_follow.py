#!/usr/bin/env python
"""Follow the line on the KIPR line mat using ONLY the reflectance array.

Stakeholder directive 2026-08-08: "your goal here is to follow the line
without using the camera. You're just using the reflectance sensors."

Nothing here uses the camera. Steering, corner recovery, crossing rejection
and the off-mat stop all come from the four line channels alone. A camera
safety halt is available behind --fence but is OFF by default and never
steers -- see that flag's help for why it is opt-in (it false-halted a good
run when tag 1 left the frame and world coordinates silently re-based).

--map is the same idiom applied to logging rather than safety: it reads tag
100 every iteration (never feeds the controller) so each line sample gets a
time-aligned body pose, then at the end projects each channel out to its own
world position via the array's lever arm (field_map.lever_arm) and renders
the whole run over a real playfield photo. Also opt-in, also never steers.

PATH ANTICIPATION (on by default): every run integrates distance travelled
from frame.twist[0] (fused body velocity, always on the wire -- not the
camera) and, on a run that reaches a real finish, records which stretches of
that distance needed |omega| >= KINK_OMEGA into output/course_profile.json.
The NEXT run loads that file and, on approaching or inside a recorded zone,
preemptively cuts speed and boosts KP -- braking for a known kink before the
reactive error signal alone would, and tracking it tighter once there. This
is still reflectance-only steering: distance only says "where along the
course am I", never "where is the line". Disable saving with --no-record if
you want to test profile-driven behaviour without further changing the file.

FINISH DETECTION: the same sustained-all-four-dark stop that always halted
the follower now checks distance against a previously recorded run's total
distance (FINISH_DISTANCE_FRAC). Past that point it is reported as the
finish crossbar; short of it, the message stays the generic "parked on
something wide and dark" caution, since that combination is more likely a
real obstruction than the course ending early. No profile yet means no
comparison is possible, so the first-ever run always gets the generic
message even when it did in fact just finish.

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
# (tovez.json perception.line_array.x).
ARRAY_LEVER = 96.0

# [rad/s] hard ceiling on omega, INDEPENDENT of forward speed.
#
# This replaces an earlier speed/MIN_RADIUS cap that bounded the turn RADIUS
# at 90mm. That cap was derived from the wrong constraint -- it required the
# array's sideways sweep (ARRAY_LEVER * omega) to stay under the forward
# speed, which is a per-DISTANCE test. The real limit is per-SAMPLE: the line
# must not leave the array between two readings. At ~28Hz the array sweeps
# ARRAY_LEVER * omega * 0.036s, so even omega = 1.2 moves it only ~4mm
# against a 32mm half-width -- a wide margin.
#
# The wrong cap had a nasty second-order effect, because omega_max scaled
# with a speed that itself fell with error: turn authority was LOWEST exactly
# where the error was largest. MEASURED 2026-08-08: the follower tracked the
# top waves and the hairpin for 31.6 SECONDS unbroken, then lost the line in
# the sharp zigzags near the finish, whose corners are tighter than 90mm.
OMEGA_MAX = 1.2

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
# Now that omega is capped independently of speed (OMEGA_MAX), the floor can
# go low again -- slowing hard into a corner no longer costs turn authority,
# it buys reaction time. At err=32mm and speed 60 this drives 27mm/s with up
# to 1.2 rad/s available, a 22mm turn radius: tight enough for the zigzags.
SLOW_FLOOR = 0.45

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

# Path anticipation: "record the path, anticipate the kinks." Distance is
# integrated from frame.twist[0] -- the firmware's fused body-frame forward
# velocity ([mm/s], always on the wire, no presence gate -- see TLMFrame's
# docstring) -- NEVER the camera. This stays inside "reflectance sensors
# steer, nothing else does": distance only clocks how far along the course
# we are; the lateral correction is still driven by line_error() alone.
PROFILE_PATH = pathlib.Path(__file__).resolve().parent / "output" / "course_profile.json"
KINK_OMEGA = 0.5        # [rad/s] a TRACK sample at/above this is a kink sample
KINK_MARGIN = 90.0      # [mm] recorded zones are extended this far on both
                        # ends, so a later run starts slowing BEFORE the turn.
                        # Widened from 60mm once runs got fast enough that a
                        # fixed REACTION TIME covers more ground before a
                        # correction lands -- more lead-in buys back that time.
KINK_GAP_MERGE = 20.0   # [mm] kink samples closer than this become one zone.
                        # MEASURED 2026-08-08: 80mm merged the whole densely
                        # zigzagged back half of the course (932-2425mm) into
                        # ONE zone at peak severity: correct that it is all
                        # rough, but useless for "anticipate THIS kink" and it
                        # would slow/boost-gain the entire second half. 20mm
                        # still merges genuinely adjacent kinks (e.g. the
                        # S-wave's alternating corners) without swallowing the
                        # straight stretches between distinct zigzags.
KINK_SLOW = 0.55        # fraction of speed cut inside a zone, scaled by how
                        # severe that zone's recorded peak omega was
KINK_KP_BOOST = 1.0     # fractional KP increase inside a zone, same scaling.
                        # MEASURED 2026-08-08: 0.35/0.6 held a clean 140mm/s
                        # run but let 180mm/s lose the line 4 times (recovered
                        # every time, but that is luck, not margin) --
                        # steeper cut/boost buys the tracking loop more grip
                        # through a corner specifically, leaving the straights
                        # to carry the speed increase instead.

# A sustained all-four-dark past this fraction of a previously recorded run's
# total distance is the finish crossbar, not an obstruction. With no profile
# yet recorded, there is nothing to compare against, so it stays a generic
# stop -- see the CROSS_MAX branch below.
FINISH_DISTANCE_FRAC = 0.85
# [mm] once within this much of the recorded finish, start braking -- so by
# the time the array actually reaches the crossbar the robot is already
# slow, rather than arriving at full speed and needing the whole CROSS_MAX
# dwell (which drives THROUGH it at args.speed) to decide it should stop.
FINISH_APPROACH = 150.0
FINISH_MIN_FRAC = 0.35  # fraction of speed left at the very end of the ramp

# A run only updates the recorded profile if its worst single LOST episode
# stayed under this. MEASURED 2026-08-08: a single-sample 0.12s blip that
# recovered immediately is normal noise, not a bad map -- blocking on ANY
# loss at all (the first version of this gate) discarded good runs for a
# reading that never actually threw the distance/omega history off.
MAX_CLEAN_LOST = 0.5    # [s]


def load_profile():
    """Load the recorded course profile, or None if there isn't one yet."""
    try:
        return json.loads(PROFILE_PATH.read_text())
    except (FileNotFoundError, ValueError):
        return None


def zone_at(profile, distance):
    """Peak |omega| of the recorded kink zone(s) containing `distance`
    (already inflated by KINK_MARGIN when built), or None. Zones legitimately
    overlap in a densely-curvy stretch (each is margin-extended from its own
    corner) -- taking the max over every match is the conservative choice, a
    nearby sharper corner should never be masked by a milder one that happens
    to be listed first."""
    if profile is None:
        return None
    hits = [z["peak_omega"] for z in profile.get("kinks", ())
            if z["start"] <= distance <= z["end"]]
    return max(hits) if hits else None


def build_profile(samples):
    """samples: [(distance, omega), ...] from TRACK-state control decisions
    on a run that reached a real finish. Extract kink zones (contiguous
    stretches of |omega| >= KINK_OMEGA, margin-extended and merged) plus the
    total distance travelled, so the NEXT run can anticipate them."""
    total = samples[-1][0] if samples else 0.0
    raw = []
    cur = None
    for dist, omega in samples:
        if abs(omega) >= KINK_OMEGA:
            if cur is None:
                cur = {"start": dist, "end": dist, "peak_omega": abs(omega)}
            else:
                cur["end"] = dist
                cur["peak_omega"] = max(cur["peak_omega"], abs(omega))
        elif cur is not None:
            raw.append(cur)
            cur = None
    if cur is not None:
        raw.append(cur)

    # Merge on RAW gaps first. Applying KINK_MARGIN before merging (the
    # first version of this) silently inflated the effective merge distance
    # by 2x KINK_MARGIN regardless of KINK_GAP_MERGE -- MEASURED 2026-08-08:
    # widening KINK_MARGIN 60->90mm for more brake lookahead had the side
    # effect of re-collapsing the whole back half into one zone again, the
    # exact bug KINK_GAP_MERGE was already dropped 80->20mm to fix. Margin is
    # a lookahead distance for braking, not a merge distance -- apply it
    # once, after merging is decided on the real gap between corners.
    merged = []
    for z in raw:
        if merged and z["start"] - merged[-1]["end"] <= KINK_GAP_MERGE:
            merged[-1]["end"] = z["end"]
            merged[-1]["peak_omega"] = max(merged[-1]["peak_omega"], z["peak_omega"])
        else:
            merged.append(dict(z))

    zones = [{"start": z["start"] - KINK_MARGIN, "end": z["end"] + KINK_MARGIN,
             "peak_omega": z["peak_omega"]} for z in merged]
    return {"total_distance": total, "kinks": zones}


def save_profile(profile):
    PROFILE_PATH.parent.mkdir(exist_ok=True)
    PROFILE_PATH.write_text(json.dumps(profile, indent=2))


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
    ap.add_argument("--speed", type=float, default=140.0, help="[mm/s] nominal")
    ap.add_argument("--seconds", type=float, default=180.0, help="[s] run limit")
    ap.add_argument("--no-record", action="store_true",
                    help="don't update output/course_profile.json even on a "
                         "clean finish. The recorded profile still loads and "
                         "drives kink anticipation as usual -- this only "
                         "stops this run's own data from changing the file.")
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
    ap.add_argument("--map", action="store_true",
                    help="log tag-100 pose alongside every line sample and "
                         "render an end-of-run PNG: body-centre trail plus "
                         "all four channels' lever-arm-projected world "
                         "positions, coloured by reading, over a real "
                         "playfield photo. Camera is read-only here -- it "
                         "never steers, same as --fence. Off by default "
                         "(extra camera round-trips every iteration).")
    args = ap.parse_args()

    fix_fn = None
    dc = None
    if (args.fence or args.map) and not args.report:
        try:
            from field_map import CAMERA, OUT, connect_camera, read_tag, undo_parallax
            cal = json.loads((OUT / "field_parallax.json").read_text())
            K, CX, CY = cal["k"], cal["cx"], cal["cy"]
            dc = connect_camera()

            def fix_fn():  # noqa: F811
                raw = read_tag(dc)
                if raw is None:
                    return None
                x, y = undo_parallax(raw[0], raw[1], K, CX, CY)
                return (x, y, raw[2])
            if args.fence:
                print("camera safety fence armed (halt only -- never steers)")
            if args.map:
                print("camera pose logging armed for --map (never steers)")
        except Exception as e:
            print(f"camera unavailable ({e}) -- continuing without it")
            fix_fn = None
            dc = None

    bg = None  # (rgb ndarray, extent, origin='upper') for the --map overlay
    if args.map and dc is not None:
        try:
            tag_frame = dc.get_tags(CAMERA)
            raw_bgr = dc.capture_frame(CAMERA)
            from robot_radio.testgui.operations import _deskew_bgr_ndarray
            result = _deskew_bgr_ndarray(raw_bgr, tag_frame)
            if result is not None:
                deskewed_bgr, origin_x, origin_y = result
                fw = float(getattr(tag_frame, "field_width_cm"))
                fh = float(getattr(tag_frame, "field_height_cm"))
                rgb = deskewed_bgr[:, :, ::-1]
                extent = (-origin_x, fw - origin_x, origin_y - fh, origin_y)
                bg = (rgb, extent)
                print(f"playfield photo captured: {fw:.1f}x{fh:.1f}cm")
            else:
                print("could not deskew the playfield photo -- map will have no background")
        except Exception as e:
            print(f"playfield photo unavailable ({e}) -- map will have no background")

    profile = load_profile()
    if profile is not None:
        print(f"loaded course profile: {profile['total_distance']:.0f}mm, "
              f"{len(profile['kinks'])} kink zone(s)")

    out = pathlib.Path(__file__).resolve().parent / "output"
    out.mkdir(exist_ok=True)
    csv = None if args.report else (out / "line_follow.csv").open("w")
    if csv is not None:
        csv.write("t,dist,ch1,ch2,ch3,ch4,state,err,speed,omega,cam_x,cam_y,cam_heading\n")

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
        lost_episodes = 0        # informational -- see worst_lost for the save gate
        worst_lost = 0.0         # [s] longest single LOST episode this run
        cross_until = 0.0
        cross_start = 0.0
        track_time = 0.0
        crossings = 0
        cross_log: list[tuple[float, str]] = []
        last_cmd = 0.0
        t0 = time.time()
        printed = 0.0
        last_frame = time.time()
        frames = 0
        distance = 0.0    # [mm] integrated from frame.twist[0], never the camera
        dist_samples: list[tuple[float, float]] = []  # (distance, omega) while TRACKing
        finished = False  # a real end-of-course stop, not a Ctrl-C/fence/failure
        cam_x = cam_y = cam_heading = None
        map_rows: list[tuple[float, float, float, tuple]] = []
        if fix_fn is not None:
            from field_map import camera_fix
            seed = camera_fix(fix_fn, samples=5)
            if seed is not None:
                cam_x, cam_y, cam_heading = seed

        while time.time() - t0 < args.seconds:
            vals = None
            twist_v = None
            for env in conn.drain_binary_tlm():
                t = getattr(env, "tlm", None)
                if t is None:
                    continue
                fr = TLMFrame.from_pb2(t)
                v = read_line(fr)
                if v is not None:
                    vals = v
                    frames += 1
                if fr.twist is not None:
                    twist_v = fr.twist[0]
            if vals is None:
                time.sleep(0.01)
                continue

            now = time.time()
            dt = now - last_frame
            if twist_v is not None:
                distance += twist_v * dt
            nlit = sum(1 for v in vals if v >= LINE_THRESHOLD)
            err = line_error(vals)

            near_finish = (profile is not None and
                          distance >= profile["total_distance"] * FINISH_DISTANCE_FRAC)

            if nlit >= CROSS_LIT:
                if cross_until < now:      # rising edge of a new crossing
                    crossings += 1
                    cross_start = now
                    cross_log.append((now - t0, describe(vals)))
                    # Near the recorded finish, this dark reading almost
                    # certainly IS the crossbar -- stop on this first sample
                    # rather than waiting through CROSS_HOLD/CROSS_MAX, which
                    # holds course at full args.speed and drives the robot
                    # THROUGH the bar before ever deciding to stop. The
                    # FINISH_APPROACH ramp below has already been braking for
                    # the last stretch, so this stop lands close to the bar
                    # rather than well past it.
                    if near_finish:
                        print(f"\nFINISH LINE reached at distance {distance:.0f}mm "
                              f"(recorded course "
                              f"{profile['total_distance']:.0f}mm) -- stopping.")
                        finished = True
                        break
                cross_until = now + CROSS_HOLD
            crossing = now < cross_until

            if crossing and now - cross_start > CROSS_MAX:
                # Reaching here means near_finish was false at the rising
                # edge above (that branch already stopped on the first dark
                # sample when it was true) -- so this sustained darkness is
                # NOT where a prior run's finish was, or there is no profile
                # yet to compare against. Every such stop this session has
                # still turned out to be the real end of the course, so it
                # is still worth recording, just without positive
                # confirmation -- hence the more cautious wording.
                finished = True
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
                    lost_episodes += 1
                elif now - lost_since > LOST_GIVE_UP:
                    # Reaching the search budget after a good run of tracking
                    # means the line ran out under us -- the end of the course
                    # -- not that the follower lost a line that is still there.
                    if track_time > 5.0:
                        print(f"\nno line found in {LOST_GIVE_UP:.1f}s of "
                              f"searching, after {track_time:.0f}s of "
                              f"tracking -- the line ended. Stopping here.")
                        finished = True
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
                if lost_since is not None:
                    worst_lost = max(worst_lost, now - lost_since)
                lost_since = None
                if abs(err) > 1e-6:
                    last_seen_sign = -1.0 if err > 0 else 1.0
                # Dividing by the FIXED CMD_PERIOD (not actual dt) is
                # deliberate and already what KD=0.004 was tuned against --
                # true samples arrive faster than CMD_PERIOD (~35ms vs 90ms),
                # so switching to real dt would make the D-term ~2.5x
                # stronger for every normal sample, not just this case.
                # MEASURED 2026-08-08: telemetry can gap for SECONDS (relay/
                # radio dropout -- drain_binary_tlm() is a plain non-blocking
                # queue drain, never this loop). Dividing (err-last_err) by
                # 0.09s after a multi-second gap manufactures a huge,
                # meaningless derivative right when tracking resumes, which
                # is a real contributor to losing the line right after a
                # dropout. Neutralise only that pathological case; leave the
                # tuned constant alone everywhere else.
                d = 0.0 if dt > 0.3 else (err - last_err) / CMD_PERIOD
                last_err = err
                # A recorded kink zone (build_profile, from a PRIOR run that
                # reached a real finish) says a turn is here or coming up
                # within KINK_MARGIN -- tighten the gain and cut speed ahead
                # of the reactive error signal alone, scaled by how sharp
                # that zone was last time.
                severity = zone_at(profile, distance)
                sev_frac = 0.0 if severity is None else min(severity / OMEGA_MAX, 1.0)
                kp = KP * (1.0 + KINK_KP_BOOST * sev_frac)
                # Positive omega turns the robot RIGHT (decreases heading,
                # .claude/rules/playfield-testing.md); a positive error means
                # the line is to the LEFT -- hence the negation.
                omega = -(kp * err + KD * d)
                frac = min(abs(err) / max(CHANNEL_Y), 1.0)
                speed = args.speed * (1.0 - (1.0 - SLOW_FLOOR) * frac)
                speed *= 1.0 - KINK_SLOW * sev_frac
                if profile is not None:
                    # Brake on the approach to the recorded finish so the
                    # robot is already slow by the time the array reaches
                    # the crossbar, rather than arriving at args.speed.
                    to_go = profile["total_distance"] - distance
                    if to_go < FINISH_APPROACH:
                        ramp = max(0.0, min(1.0, to_go / FINISH_APPROACH))
                        speed *= FINISH_MIN_FRAC + (1.0 - FINISH_MIN_FRAC) * ramp
                # The array is ARRAY_LEVER ahead of the centre of rotation,
                # so a turn sweeps it sideways at ARRAY_LEVER*omega. Once
                # that exceeds the forward speed the robot is outrunning its
                # own sensor -- it steers the line out from under the array
                # and goes blind, which is what produced 32% `....` readings
                # at omega=1.1 with v=85. Capping the ratio at 1 bounds the
                # turn radius at ARRAY_LEVER (96mm), still tighter than the
                # hairpin needs.
                omega = max(-OMEGA_MAX, min(OMEGA_MAX, omega))
                last_omega = omega
                dist_samples.append((distance, omega))

            last_frame = now
            if fix_fn is not None:
                c = fix_fn()
                if c is not None:
                    cam_x, cam_y, cam_heading = c
                if args.fence and c is not None and (
                        abs(c[0]) > FENCE_X or abs(c[1]) > FENCE_Y):
                    print(f"\nSAFETY HALT: robot at ({c[0]:+.1f},{c[1]:+.1f})cm")
                    break
                if args.map and cam_x is not None:
                    map_rows.append((cam_x, cam_y, cam_heading, vals))
            if csv is not None:
                cam_txt = (",,," if cam_x is None else
                           f",{cam_x:.2f},{cam_y:.2f},{cam_heading:.4f}")
                csv.write(f"{now - t0:.3f},{distance:.1f},{vals[0]},{vals[1]},"
                          f"{vals[2]},{vals[3]},{state},"
                          f"{'' if err is None else f'{err:.1f}'},"
                          f"{speed:.0f},{omega:.3f}{cam_txt}\n")

            if now - printed > 0.4:
                printed = now
                e_txt = "  --  " if err is None else f"{err:+6.1f}"
                print(f"  {describe(vals)}  {state:5s} err={e_txt}mm  "
                      f"v={speed:5.0f}  omega={omega:+5.2f}  "
                      f"crossings={crossings}", end="\r", flush=True)

            if args.report:
                time.sleep(0.05)
                continue

            if now - last_cmd >= CMD_PERIOD:
                last_cmd = now
                p.move_twist(speed, 0.0, omega, stop_time=CMD_STOP_TIME,
                             timeout=CMD_STOP_TIME * 3, move_id=0)
            time.sleep(0.01)

        print(f"\nran {time.time() - t0:.0f}s, {frames} line frames, "
              f"{crossings} crossings, {lost_episodes} lost episode(s) "
              f"(worst {worst_lost:.2f}s), distance {distance:.0f}mm")
        for t, pat in cross_log:
            print(f"    crossing at t={t:6.1f}s  first pattern {pat}")
        if finished and not args.no_record and dist_samples and worst_lost <= MAX_CLEAN_LOST:
            new_profile = build_profile(dist_samples)
            save_profile(new_profile)
            print(f"updated {PROFILE_PATH.name}: "
                  f"{new_profile['total_distance']:.0f}mm, "
                  f"{len(new_profile['kinks'])} kink zone(s)")
        elif finished and worst_lost > MAX_CLEAN_LOST:
            print(f"NOT updating {PROFILE_PATH.name}: a {worst_lost:.1f}s lost "
                  f"episode this run -- that distance/omega history isn't a "
                  f"clean map of the course, it would teach the next run the "
                  f"WRONG place to brake. The existing file is untouched.")
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

    if args.map and map_rows:
        try:
            render_map(out, bg, map_rows)
        except Exception as e:
            print(f"could not render map plot ({e})")
    return 0


def render_map(out, bg, map_rows):
    """Overlay body-centre trail + per-channel lever-arm positions on the
    playfield photo captured at run start. Never called during the drive --
    pure post-processing of what was already logged."""
    from field_map import lever_arm
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(12, 8))
    if bg is not None:
        rgb, extent = bg
        ax.imshow(rgb, extent=extent, origin="upper")
    else:
        ax.set_facecolor("#dddddd")

    xs = [r[0] for r in map_rows]
    ys = [r[1] for r in map_rows]
    ax.plot(xs, ys, "-", color="deepskyblue", linewidth=1.0, alpha=0.9,
            label="body centre (tag 100)", zorder=3)

    markers = ("o", "s", "^", "D")
    sc = None
    for ch in range(4):
        px, py, vv = [], [], []
        for (x, y, h, vals) in map_rows:
            wx, wy = lever_arm(x, y, h, ARRAY_LEVER, CHANNEL_Y[ch])
            px.append(wx)
            py.append(wy)
            vv.append(vals[ch])
        sc = ax.scatter(px, py, c=vv, cmap="gray_r", vmin=0, vmax=255,
                        s=8, marker=markers[ch], edgecolors="black",
                        linewidths=0.3, label=f"ch{ch + 1}", zorder=4)

    ax.legend(loc="upper right", fontsize=8)
    ax.set_xlabel("x [cm]")
    ax.set_ylabel("y [cm]")
    ax.set_title("Line-follow run: sensor lever-arm positions over the playfield")
    ax.set_aspect("equal")
    if sc is not None:
        cb = fig.colorbar(sc, ax=ax, shrink=0.6)
        cb.set_label("raw channel reading (HIGH = dark = ON line)")
    plot_path = out / "line_follow_map.png"
    fig.savefig(plot_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {plot_path}")


if __name__ == "__main__":
    raise SystemExit(main())
