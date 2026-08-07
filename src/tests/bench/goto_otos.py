#!/usr/bin/env python3
"""Drive to world (or robot-frame) coordinates by sending GO_TO.

135-006 port: this script used to drive its own arc-solve/replan loop --
read OTOS pose, solve a tangent arc, send it as a replaceable twist Move,
re-solve before that arc finished. That whole policy (TURN_FIRST pivot
threshold, fine-approach terminal commit, the arc solver itself) has moved
INTO the firmware (Motion::Navigator, tickets 002-004): one `GO_TO` wire
command now drives a target to completion on its own, re-solving against
live OTOS pose every cycle without any further host involvement. This
script is now a thin driver and scorer, not a policy owner: it sends ONE
`GO_TO` per waypoint and watches telemetry (the ack ring, encoders, OTOS
pose, the host-side geofence) to confirm it landed correctly.

The camera is still used ONCE, to seed the world pose (or pass --seed to
skip it entirely on a bench/stand session with no playfield camera in
view -- see main()). After that the robot navigates on its own OTOS; the
camera is re-read only at the end of each leg, to score the result.

Two frames have to be reconciled first, which is the whole reason this needs
saying out loud:

  * targets (the flat tags 12/14/15/16 on the orange dots) sit on the
    playfield plane and the camera reports them TRUE.
  * the robot's own tag 100 sits 117mm above that plane and the camera
    inflates its displacement by 1.1212 (2026-08-05, tape-measured; see
    .claude/rules/ and the tovez.json linear_scale note).

So the robot's camera fix is divided by that factor and the targets are not.
Get this wrong and the robot chases coordinates in a frame it is not in --
which is exactly what made earlier camera-driven tours fight themselves.

Two target frames, both resolved by the FIRMWARE now (GoTo.frame,
envelope.proto -- 0=WORLD, 1=ROBOT):

    goto W  -- a point in WORLD coordinates (the playfield frame)
    goto R  -- a point in the ROBOT's own frame at the moment the GO_TO is
               ACCEPTED: +x straight ahead, +y to the left. Resolved to
               world once, firmware-side, at acceptance (App::RobotLoop::
               handleGoto()) -- not by this script, and not continuously,
               so the target does not chase the robot as it turns.

    uv run python src/tests/bench/goto_otos.py W 300 0
    uv run python src/tests/bench/goto_otos.py R 400 0        # 400mm ahead
    uv run python src/tests/bench/goto_otos.py R 0 300        # 300mm to port
    uv run python src/tests/bench/goto_otos.py NE SE SW NW

    # bench/stand session, no playfield camera -- seed the OTOS directly:
    uv run python src/tests/bench/goto_otos.py R 300 0 --seed 0 0 0 \\
        --port /dev/cu.usbmodem2121102
"""
from __future__ import annotations

import argparse
import math
import time

# RELAY_PORT -- name kept for camera_map.py's own `from goto_otos import
# ... RELAY_PORT` (that script uses it as its own --port default too).
# Historically a relay dongle's port; nothing here requires a relay any
# more -- Robot.__init__() auto-detects direct vs. relay from the
# connecting device's own boot banner (SerialConnection's mode=None path),
# so the SAME script now works unmodified over a direct-USB port (this
# ticket's own tovez bench session) or through a relay (playfield use).
RELAY_PORT = "/dev/cu.usbmodem214102"
CAMERA = "arducam-ov9782-usb-camera"
ROBOT_TAG = 100

# Camera overstates the ELEVATED robot tag's radius from the field origin by
# this much. Flat tags are unaffected. Remove once AprilCam corrects for tag
# height -- verified against tape at 989.7/990.0mm on two 990mm legs.
CAM_TAG_INFLATION = 1.1212

# GoTo.frame (envelope.proto) -- 0=WORLD (OTOS/SEED frame), 1=ROBOT
# (resolved once at acceptance, firmware-side). Matches
# App::RobotLoop::handleGoto()'s own goTo.frame check exactly.
FRAME_WORLD = 0
FRAME_ROBOT = 1

# Per-waypoint host-side patience AND the wire-level GoTo.timeout backstop
# (converted to ms in goto_wire()) -- everything else the old script's own
# CRUISE/APPROACH/SLOW_RADIUS/PIVOT_OMEGA/YAW_SIGN/TURN_FIRST/FINE_TURN/
# ARRIVE/REPLAN/FINE_RADIUS/MAX_ARC constants used to tune is now
# Motion::Navigator policy, sourced from the robot's own `navigator` config
# group (data/robots/tovez.json) -- speed/arrive default to 0 in goto_wire()
# below, which falls open to that config (NavigatorLimits::speed /
# ::defaultArrivalTolerance), not a value this script re-invents.
TIMEOUT = 45.0          # [s] per waypoint

FENCE_X, FENCE_Y = 600.0, 390.0   # [mm] TRUE coordinates

# The orange dots: exactly +-500mm x, +-300mm y (stakeholder, 2026-08-05).
# Confirmed by laying flat tags 12/14/15/16 on them and reading (+-50, +-30)cm
# back from the camera -- that check was to validate the DOTS, and the dots
# won; the tags themselves are not placed accurately enough to survey from.
DOTS = {"NW": (-500.0, 300.0), "NE": (500.0, 300.0),
        "SW": (-500.0, -300.0), "SE": (500.0, -300.0)}


class Robot:
    def __init__(self, port: str):
        from robot_radio.io.serial_conn import SerialConnection
        from robot_radio.robot.protocol import NezhaProtocol
        # mode=None -- auto-detect direct-USB vs. relay from the connecting
        # device's own boot banner (SerialConnection.connect()'s "3a/3b"
        # classify). The old hardcoded mode="relay" forced the relay
        # !ECHO OFF/!MODE RAW250/!GO control-plane handshake unconditionally
        # -- fine through an actual RADIOBRIDGE dongle, but wrong (and, per
        # that handshake's own "# entering data plane" wait, likely to hang)
        # against a direct-USB NEZHA2 banner, which is exactly what this
        # ticket's own tovez bench session is (.claude/rules/
        # hardware-bench-testing.md: "Transport this session: DIRECT SERIAL
        # ONLY"). Auto-detect makes this ONE script correct for both.
        self.conn = SerialConnection(port=port, mode=None)
        self.conn.connect()
        self.p = NezhaProtocol(self.conn)
        self._id = int(time.time()) % 500000 + 100000
        self.conn.send_cleartext("TLM:ON")
        time.sleep(0.4)
        self.conn.drain_binary_tlm()

    def _next_id(self) -> int:
        self._id += 1
        return self._id

    def pose(self, window: float = 0.12):
        """Newest OTOS pose from telemetry: (x, y, heading). ~20Hz, no polling."""
        time.sleep(window)
        best = None
        for env in self.conn.drain_binary_tlm():
            t = getattr(env, "tlm", None)
            if t is None or not (t.flags & 0x02):   # no OTOS -> no pose
                continue
            if best is None or t.now > best.now:
                best = t
        if best is None:
            return None
        return (float(best.otos.x), float(best.otos.y),
                float(best.otos.heading) * 0.001)

    def pose_blocking(self, timeout: float = 4.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            p = self.pose()
            if p is not None:
                return p
        return None

    def seed(self, x: float, y: float, heading: float,  # [mm] [mm] [rad]
             timeout: float = 5.0) -> bool:              # [s]
        """SEED the OTOS world pose, verified against FRESH telemetry.

        Two bugs fixed here, both measured on the playfield 2026-08-06:

        1. HEADING MUST BE WRAPPED to [-pi, pi]. ``Devices::Otos::setPose()``
           rotates the configured lever arm (tovez ``otos.offset_x`` =
           -47.7mm) by whatever heading it is handed, so an out-of-range
           angle rotates that correction the wrong way and lands the
           POSITION seed up to ~2x the lever arm off. The camera can
           legitimately report an unwrapped yaw -- a raw -5.94995 rad
           camera yaw put x 91mm wrong while y was fine.

        2. THE READ-BACK MUST WAIT FOR A FRESH FRAME. The old check slept
           0.5s then took whatever ``drain_binary_tlm()`` returned, which is
           a backlog that can predate the seed -- producing false "seed did
           not read back" aborts on a seed that actually landed. Draining
           FIRST, then waiting for a frame that shows the seeded pose, fixes
           it. The ``POSE`` verb is NOT an alternative: its reply is
           unreliable while binary telemetry streams (returns empty).
        """
        heading = math.atan2(math.sin(heading), math.cos(heading))
        self.conn.drain_binary_tlm()          # discard everything pre-seed
        self.conn.send_cleartext(f"SEED:{x:.1f},{y:.1f},{heading:.5f}",
                                 read_timeout=1500)
        deadline = time.time() + timeout
        while time.time() < deadline:
            for env in self.conn.drain_binary_tlm():
                t = getattr(env, "tlm", None)
                if t is None or not (t.flags & 0x02):   # no OTOS -> no pose
                    continue
                if math.hypot(float(t.otos.x) - x,
                              float(t.otos.y) - y) < 25.0:
                    return True
            time.sleep(0.08)
        return False

    def goto_wire(self, x: float, y: float, frame: int, *,
                  speed: float = 0.0,    # [mm/s] 0 = NavigatorLimits::speed config default
                  arrive: float = 0.0,   # [mm] 0 = NavigatorLimits::defaultArrivalTolerance
                  timeout: float = 45000.0,  # [ms] REQUIRED whole-goto backstop
                  ) -> tuple[int, int]:
        """Send ONE GO_TO via ``NezhaProtocol.go_to()`` (135-007).

        135-006 introduced this method as a TEMPORARY DUPLICATION (building
        and firing the ``CommandEnvelope.go_to`` envelope itself, since
        ``NezhaProtocol`` had no ``go_to()`` wrapper yet); 135-007 added that
        wrapper (mirroring ``move_twist()``/``move_wheels()``), so this
        method is now a thin pass-through -- kept only so this script's own
        `goto()` call site and its ``(corr_id, goto_id)`` return shape
        (needed to match the later COMPLETION ack) do not need to change.

        Returns ``(corr_id, goto_id)``: ``corr_id`` is the envelope id
        ``go_to()`` auto-assigns via ``send_envelope_fast()``, which the
        ENQUEUE ack (``Telemetry.acks``) echoes; ``goto_id`` is this call's
        own explicit ``GoTo.id``, echoed by the single COMPLETION ack when
        the goto ends (Done or Aborted) -- two distinct keys, exactly like
        ``move_twist()``'s ``corr_id`` vs. its ``move_id``.
        """
        goto_id = self._next_id()
        corr_id = self.p.go_to(x, y, frame=frame, speed=speed, arrive=arrive,
                               timeout=timeout, goto_id=goto_id)
        return corr_id, goto_id

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


def camera_fixes(dc, tries: int = 12, want_tags: int = 4):
    """(robot_pose_true, {tag_id: (x,y)}) -- robot de-inflated, tags as-is.

    Accumulates across frames. A single frame routinely misses tags, so
    returning on the first robot sighting loses the targets entirely.
    """
    robot, tags = None, {}
    for _ in range(tries):
        tf = dc.get_tags(CAMERA)
        for t in getattr(tf, "tags", []):
            xy = getattr(t, "world_xy", None)
            if xy is None:
                continue
            if t.id == ROBOT_TAG:
                robot = (float(xy[0]) * 10.0 / CAM_TAG_INFLATION,
                         float(xy[1]) * 10.0 / CAM_TAG_INFLATION,
                         float(t.yaw))
            elif t.id != 1:
                tags[t.id] = (float(xy[0]) * 10.0, float(xy[1]) * 10.0)
        if robot is not None and len(tags) >= want_tags:
            break
        time.sleep(0.12)
    return robot, tags


def inside_fence(x, y):
    return abs(x) < FENCE_X and abs(y) < FENCE_Y


def goto(bot, dc, target, label, frame=FRAME_WORLD, *,
         speed: float = 0.0, arrive: float = 0.0, timeout: float = TIMEOUT):
    """Send ONE GO_TO and monitor it to completion via telemetry.

    ``dc`` is accepted but UNUSED (kept for call-site compatibility --
    ``camera_map.py`` calls ``goto(bot, dc, target, label)`` positionally;
    the original script also never used it inside the loop body). All
    arc-solve/replan/pivot POLICY now lives in firmware (Motion::Navigator,
    tickets 002-004): this function's whole job is to send the command and
    WATCH -- the ack ring (exactly one enqueue ack, exactly one completion
    ack, zero spurious entries -- ticket 004's "Landmine 1"), encoder
    motion, and the host-side geofence (this project's own "never drive
    blind" rule, `.claude/rules/playfield-testing.md` -- a firmware bug
    must not be allowed to drive the robot off the fenced area just
    because the firmware is now the one steering).

    Returns True iff the goto's own completion ack reports OK and no
    spurious ack ring entries were observed; False on a fence trip, a
    timeout waiting for the completion ack, or a non-OK completion.
    """
    from robot_radio.robot.protocol import TLMFrame

    frame_name = "ROBOT" if frame == FRAME_ROBOT else "WORLD"
    print(f"  -> {label} ({target[0]:.0f}, {target[1]:.0f}) [{frame_name}]")

    corr_id, goto_id = bot.goto_wire(target[0], target[1], frame,
                                     speed=speed, arrive=arrive,
                                     timeout=timeout * 1000.0)

    started = time.time()
    deadline = started + timeout + 5.0  # host patience beyond the wire's own backstop
    enqueue = None
    completion = None
    spurious = []
    enc_start = enc_last = None
    otos_last = None

    # Telemetry.acks is a PERSISTENT ring snapshot (App::Telemetry::
    # pushAckRing()/assembleFrame(), telemetry.cpp): every frame
    # re-serializes whatever currently sits in the depth-4 ring, so the
    # SAME ack entry legitimately repeats across many consecutive frames
    # until a NEW ack() call evicts it -- an at-least-once redundant
    # broadcast (protection against one dropped/corrupted frame), not a
    # re-fire. 135-006 found this the hard way: a naive "first sighting
    # wins, every repeat is spurious" reader reported ~900 spurious acks
    # on ONE 45s goto that idled in the ack ring the whole time. The
    # correct read: a corr_id never sent by US (Landmine 1 -- an internal
    # segment leaking its own completion under some other id), or the
    # SAME corr_id reappearing with DIFFERENT content (a genuine
    # double-fire), is spurious; an exact repeat of an
    # already-accounted-for entry is not.
    seen = {}  # corr_id -> the AckEntry already accounted for (enqueue or completion)

    while completion is None and time.time() < deadline:
        for env in bot.conn.drain_binary_tlm():
            t = getattr(env, "tlm", None)
            if t is None:
                continue
            tf = TLMFrame.from_pb2(t)
            if enc_start is None:
                enc_start = tf.enc
            enc_last = tf.enc
            if tf.otos is not None:
                otos_last = tf.otos
            for ack in tf.acks:
                prior = seen.get(ack.corr_id)
                if prior is not None:
                    if prior.ok != ack.ok or prior.err_code != ack.err_code:
                        spurious.append(ack)  # same corr_id, different outcome -- a real double-fire
                    continue  # exact repeat of an already-accounted-for entry -- expected, not spurious
                seen[ack.corr_id] = ack
                if ack.corr_id == goto_id:
                    completion = ack
                elif ack.corr_id == corr_id:
                    enqueue = ack
                else:
                    spurious.append(ack)  # an ack for a corr_id we never sent -- Landmine 1
            if otos_last is not None and not inside_fence(otos_last[0], otos_last[1]):
                bot.halt()
                print(f"     FENCE: OTOS at ({otos_last[0]:.0f},{otos_last[1]:.0f}) -- halted")
                return False
        if completion is None:
            time.sleep(0.05)

    elapsed = time.time() - started
    if completion is None:
        bot.halt()
        print(f"     TIMEOUT after {elapsed:.1f}s waiting for the completion ack "
              f"(enqueue ack: {'OK' if enqueue and enqueue.ok else enqueue})")
        return False

    print(f"     enqueue ack: {'OK' if enqueue and enqueue.ok else enqueue}; "
          f"completion ack: {'OK' if completion.ok else f'err={completion.err_code}'}; "
          f"{elapsed:.1f}s; enc {enc_start} -> {enc_last}; otos {otos_last}")
    if spurious:
        print(f"     WARNING: {len(spurious)} SPURIOUS ack ring entries: {spurious}")
    return completion.ok and not spurious


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("target", nargs="+", help="'x y' pairs, or NE/NW/SE/SW dots")
    ap.add_argument("--port", default=RELAY_PORT)
    ap.add_argument("--seed", nargs=3, type=float, metavar=("X", "Y", "HEADING_DEG"),
                    help="skip the camera fix and seed the OTOS at this world pose "
                         "directly -- for a bench/stand session with no playfield "
                         "camera in view (135-006); CLI shape otherwise unchanged")
    args = ap.parse_args()

    dc = None
    if args.seed is not None:
        x, y, heading_deg = args.seed
        start = (x, y, math.radians(heading_deg))
        print(f"--seed given: skipping the camera, seeding OTOS at "
              f"({x:.1f}, {y:.1f}) {heading_deg:.1f}deg directly")
    else:
        from aprilcam.config import Config
        from aprilcam.client.control import DaemonControl
        dc = DaemonControl.connect_default(Config.load())
        start, _tags = camera_fixes(dc)
        if start is None:
            print("camera cannot see tag 100 -- are the lights on? (or pass "
                  "--seed X Y HEADING_DEG on a bench/stand session with no camera)")
            return 1

    # The dots are whatever flat tags are on the field, named by quadrant.
    dots = dict(DOTS)
    print(f"dots: { {k: (round(v[0]), round(v[1])) for k, v in sorted(dots.items())} }")

    # A waypoint is (label, target, frame). A ROBOT-frame target is resolved
    # to world coordinates by the FIRMWARE at acceptance (App::RobotLoop::
    # handleGoto()), not here: "400mm ahead" means ahead of wherever the
    # robot is when that GO_TO is accepted, which this script does not need
    # to compute -- it only estimates it, below, for a pre-flight fence
    # check and a diagnostic print.
    waypoints = []
    toks = list(args.target)
    while toks:
        t = toks.pop(0)
        key = t.upper()
        if key in dots:
            waypoints.append((key, dots[key], FRAME_WORLD))
        elif key in ("R", "W") and len(toks) >= 2:
            x = float(toks.pop(0))
            y = float(toks.pop(0))
            frame = FRAME_ROBOT if key == "R" else FRAME_WORLD
            waypoints.append((f"{key}({x:.0f},{y:.0f})", (x, y), frame))
        elif toks:
            y = toks.pop(0)
            waypoints.append((f"({t},{y})", (float(t), float(y)), FRAME_WORLD))
        else:
            print(f"unknown target {t!r}; dots: {sorted(dots)}, or R/W <x> <y>")
            return 1

    bot = Robot(args.port)
    try:
        print(f"seeding OTOS: ({start[0]:.1f}, {start[1]:.1f}) "
              f"{math.degrees(start[2]):.1f}deg")
        if not bot.seed(*start):
            print("seed did not read back -- stopping")
            return 1

        results = []
        for label, target, frame in waypoints:
            if frame == FRAME_WORLD:
                score_target = target
                if not inside_fence(*target):
                    print(f"  -> {label} is outside the fence -- skipping")
                    continue
            else:
                here = bot.pose_blocking()
                if here is None:
                    print(f"  -> {label}: no OTOS pose to pre-check against")
                    break
                c, s_ = math.cos(here[2]), math.sin(here[2])
                score_target = (here[0] + target[0] * c - target[1] * s_,
                                here[1] + target[0] * s_ + target[1] * c)
                print(f"  {label} estimated world "
                      f"({score_target[0]:.0f}, {score_target[1]:.0f}) -- "
                      f"the firmware resolves this authoritatively at acceptance")
                if not inside_fence(*score_target):
                    print(f"  -> {label} estimated outside the fence -- skipping")
                    continue

            ok = goto(bot, dc, target, label, frame)
            time.sleep(1.2)
            if dc is not None:
                cam, _ = camera_fixes(dc)
                otos = bot.pose_blocking()
                if cam and otos:
                    cam_err = math.dist(cam[:2], score_target)
                    drift = math.dist(cam[:2], otos[:2])
                    results.append((label, cam_err, drift))
                    print(f"     camera says {cam_err:.1f}mm from target; "
                          f"OTOS-vs-camera {drift:.1f}mm")
            if not ok:
                break

        if results:
            print()
            print(f"{'waypoint':<10} {'camera err':>11} {'otos-cam':>10}")
            for label, err, drift in results:
                print(f"{label:<10} {err:10.1f}mm {drift:9.1f}mm")
            errs = sorted(r[1] for r in results)
            print(f"median camera error: {errs[len(errs)//2]:.1f}mm over "
                  f"{len(results)} waypoints, ONE camera fix at the start")
    finally:
        bot.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
