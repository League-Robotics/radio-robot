#!/usr/bin/env python3
"""square_tour.py -- the square tour, one script, two backends. THE gate on
the command-ingestion rework (command-ingestion-ring-buffered-comms-
subsystem-routing-two-stops.md, "Verification"):

    uv run python src/tests/bench/square_tour.py --sim
    uv run python src/tests/bench/square_tour.py --port /dev/cu.usbmodem2121102

Evolved from the tagged, working ``wheels_square_tour.py`` (`wheel-layer-v1`)
-- same 8 segments, same per-run self-calibration prelude, same two-panel
chart and printed summary. Two things changed:

1. Segments are driven by the NEW ``WHEELS`` verb (``NezhaProtocol.
   wheels()``) instead of a wheels-velocity ``MOVE``. Under the reworked
   routing a ``MOVE`` -- any ``MOVE`` -- goes to ``Motion::Planner``; the
   dumb, time-bounded, planner-free wheel command this tour is built on is
   ``WHEELS``, routed straight to ``App::Drive``.
2. It runs against the SIM as well as hardware. ``NezhaProtocol`` drives a
   ``SimLoop`` through ``SimConfigConn`` (``robot_radio.io.sim_config``)
   exactly as it drives a ``SerialConnection``, so the tour LOGIC is
   written once; only the backend's notion of "let time pass" and "where is
   the robot really" differ, and those are the two methods ``_Backend``
   below abstracts.

The motion path itself is unchanged by that rework, so a regression here
means the new COMMAND path broke something -- which is exactly why this is
the gate.

Ground truth:
  --sim   ``SimLoop.get_true_pose()`` -- the plant's own truth, no odometry
          in the loop at all.
  --port  encoder odometry, integrated host-side from the telemetry
          positions (the same integration ``wheels_square_tour.py`` did).

Exits nonzero if the tour does not complete (a segment never acked) or if
closure/heading fall outside the stated bounds. Reference result on the
`wheel-layer-v1` hardware run: heading 359.9/360 deg, closure 19 mm.
"""
from __future__ import annotations

import argparse
import math
import pathlib
import signal
import sys
import time

from robot_radio.field import Geofence, GeofenceViolation, checkPlayfieldLights

_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]

TRACK = 128.0          # [mm] PHYSICAL wheel separation (caliper-measured)

# Two different tracks for two different jobs -- conflating them was itself
# a defect (2026-07-29 finding, see below).
#
# EFFECTIVE_TRACK (trackwidth / rotational_slip) is for GENERATING a
# command: a skid-steer robot scrubs its wheels sideways through a pivot and
# rotates LESS than ideal kinematics predict for a given wheel-speed
# difference, so the commanded pivot arc (TURN_ARC) has to be inflated by
# 1/slip to actually deliver 90 degrees.
#
# PHYSICAL_TRACK (the caliper-measured 128 mm, from the robot config's
# `trackwidth`) is for INTERPRETING encoders after the fact: the encoders
# report how far the wheels actually turned, and converting that back to an
# actual rotation is plain differential-drive kinematics -- the physical
# track the wheels really pivot about, not the inflated one used to
# compensate for scrub when the wheel speeds were chosen. Scrub already
# happened by the time the encoder counted it; dividing by the
# slip-inflated track a SECOND time double-counts it and under-reports the
# true rotation.
#
# This was gotten backwards previously: EFFECTIVE_TRACK was used for BOTH
# jobs, on the theory that "every angle here... is wheel-difference over
# track." MEASURED 2026-07-29 (playfield run): encoders (via
# EFFECTIVE_TRACK) reported heading 322.8 deg / closure 226 mm for a run the
# camera measured at 368.9 deg / 71.7 mm -- the encoder-interpreted heading
# was suppressed by the same 1/slip factor that inflated the command, i.e.
# double-corrected. PHYSICAL_TRACK is the fix for every encoder-INTERPRETING
# use (headingDeg, integrateEncoderPath, the moving prelude's test-pivot
# measurement); EFFECTIVE_TRACK stays for TURN_ARC, the only
# command-GENERATING use. Both set from the robot config at backend init so
# sim (slip 1.0 -> both equal 128) is unchanged.
PHYSICAL_TRACK = TRACK
EFFECTIVE_TRACK = TRACK


def _set_effective_track(robot_config) -> None:
    """Derive PHYSICAL_TRACK and EFFECTIVE_TRACK from the active robot
    config, once. See the module-level comment above these globals for
    which one each downstream use needs."""
    global PHYSICAL_TRACK, EFFECTIVE_TRACK, TURN_ARC
    PHYSICAL_TRACK = robot_config.trackwidth or TRACK
    slip = getattr(robot_config.geometry, "rotational_slip", None) or 1.0
    EFFECTIVE_TRACK = PHYSICAL_TRACK / slip if slip > 0 else PHYSICAL_TRACK
    TURN_ARC = EFFECTIVE_TRACK * math.pi / 4.0
LEG = 500.0            # [mm] default leg length; overridden by --leg
CRUISE = 150.0         # [mm/s] leg speed, both wheels
TURN_SPEED = 120.0     # [mm/s] pivot wheel speed (above the crawl boundary)
TURN_ARC = TRACK * math.pi / 4.0  # [mm] per-wheel travel for a 90 deg pivot
                                  # (recomputed by _set_effective_track())
BOOT_WAIT = 6.0        # [s] hardware only -- the sim boots synchronously
TAU = 0.23             # [s] plant time constant (spin-up transient)
DWELL = 0.10           # [s] armor reversal dwell (sign-flip wheel holds 0)
CYCLE_S = 0.04         # [s] one SimLoop.step() -- App::RobotLoop::kCycle

# Two dwells for two different jobs -- conflating them into one SEGMENT_REST
# was itself a defect (2026-07-30 finding, out-of-process, stakeholder-
# instrumented): a 1.63 s per-segment gap on hardware decomposed as
#   SEGMENT_REST (was 0.9 s) + planner settle 0.52 s + command latency
#   0.21 s + poll granularity <=0.10 s.
#
# `_awaitMove` observes the planner's own `active` flag fall only ~0.52 s
# AFTER the wheels actually stopped (motion ended 5.61 s, `active` fell
# 6.13 s in the instrumented run) -- by the time any dwell here even starts,
# the robot has already been at rest for about half a second. The OLD
# SEGMENT_REST (0.9 s) was stacking a SECOND dwell on top of a settle that
# had already happened; its own docstring ("rest after each segment, so the
# next starts from rest") was satisfied before it ran.
#
# INTER_SEGMENT_DWELL is the short one: safe to shrink to near-zero because
# the planner settle above already covers "start the next segment from
# rest". Used whenever nothing downstream needs the chassis motionless for
# a camera read -- sim, bench, and any hardware run with no geofence/camera
# armed (--no-geofence, or a bench script with no camera at all).
INTER_SEGMENT_DWELL = 0.1  # [s] short inter-segment gap, no camera fix follows

# CAMERA_FIX_DWELL preserves the OLD SEGMENT_REST value and behavior
# unchanged: `.claude/rules/playfield-testing.md` MANDATES a camera pose
# fix at every segment boundary "at REST (after the settle dwell, so there
# is no velocity lag)" -- this is a playfield-rule requirement, not a
# preference, and `Tour.captureFix()`'s own docstring already assumes the
# caller has settled first. There is also a recorded ~1.2 s post-STOP
# settle figure elsewhere in the bench notes (chassis rocking can outlast
# the planner's own 0.52 s `active`-flag settle), so this dwell stays
# conservative rather than being cut to match INTER_SEGMENT_DWELL -- a
# camera fix taken while the chassis is still rocking is worse than no fix.
CAMERA_FIX_DWELL = 0.9  # [s] dwell before a camera fix capture, at rest

# Segment CHAINING (out-of-process, 2026-07-30, stakeholder-directed): the
# third and structural latency fix, after the dead-wait fix and the dwell
# split above. Measured remaining per-boundary cost, ~0.8s x 8 segments:
# planner settle 0.52s + command latency 0.21s (send -> `active` rises) +
# 0.1s dwell. Motion::Planner already has the machinery for this (a 5-deep
# queue, boundaryLambda()/shapesCompatible() hand-off, carryPath_/
# carryHeading_ zero-leak ledger, activateNext() same-tick pop) -- it was
# simply unused by this script, which sent one segment, waited for FULL
# completion (drain to idle), dwelled, then sent the next.
#
# _runChained() (Tour) keeps CHAIN_DEPTH MOVEs enqueued in the firmware's
# own queue at once so the queued successor activates on the very next
# firmware tick after the active move's stop condition fires, instead of
# waiting for a host round trip. THREE constraints make this correct
# rather than silently wrong:
#
#   1. `replace` must be passed EXPLICITLY as False on every chained
#      enqueue. move_twist()/move_wheels()/move() all default
#      `replace=True`; the default would flush the queue and preempt the
#      move already running -- the opposite of chaining.
#   2. Completion must be detected from the ACK RING (a completion ack
#      keyed by Move.id, err always 0), not the telemetry `active` flag's
#      rise-then-fall edge `_awaitMove()` uses for the sequential path.
#      Under chaining `active` stays asserted continuously across the
#      whole chain, so that edge never fires and a wait keyed on it would
#      burn its full timeout every time -- reintroducing the exact
#      dead-wait bug the first out-of-process fix on this file already
#      killed once.
#   3. Move ids must be unique and monotonic for the whole run (already
#      true here -- segments()'s own _ID_BASE scheme). The firmware dedups
#      an enqueue on Move.id in a 16-slot ring BEFORE honoring `replace`;
#      reusing an id silently vanishes a move while still acking `err=0`.
#
# Chaining does NOT make the square flow through corners at speed: a
# square alternates Linear legs with Angular turns, and shapesCompatible()
# (planner.cpp) refuses an at-speed hand-off across that axis change -- the
# robot still comes to rest at every corner. What chaining removes is the
# host round trip, the INTER_SEGMENT_DWELL, and most of the planner-settle
# drain between segments, not the stop itself.
#
# Chaining requires a camera fix is NEVER due mid-chain: `.claude/rules/
# playfield-testing.md` mandates a camera pose fix at every segment
# boundary AT REST, and captureFix()'s own docstring already assumes the
# caller has settled first -- a queued successor activating the instant
# the prior move completes gives the chassis no such rest. So chaining
# applies ONLY to the no-camera path (sim, bench, --no-geofence); a
# geofence-armed tour always runs _runSequential() regardless of the
# --sequential-segments flag below. See Tour._canChain().
#
# CHAIN_DEPTH is 1, NOT the 2-3 this rewrite set out to use -- a measured
# planner surprise, not a design choice. At CHAIN_DEPTH=2 (the next
# segment enqueued WHILE the current one is still active, so it sits in
# Motion::Planner's pending queue for the current move's whole remaining
# run) the sim tour's heading landed at 322 deg / closure 246 mm --
# WORSE than sequential's 361/6 -- and it compounded: turn 1 measured
# ~79 deg of its own 90 deg target (already short, but matching an
# isolated turn's own shortfall -- see below), turns 2-4 each measured
# only ~69 deg, a deficit that GREW turn over turn rather than the
# carry ledger's own documented intent ("per-landing residuals cancel
# instead of accumulating"). At CHAIN_DEPTH=1 (the next segment is only
# enqueued once the CURRENT one's own completion ack has already been
# observed -- so nothing ever sits pending while an INCOMPATIBLE-shape
# move is still active) heading/closure came back to 361/5.8, matching
# sequential almost exactly, and the SAME isolated-turn shortfall
# pattern reappeared WITHOUT compounding (~79 deg per turn, corrected
# back by ~8 deg of heading-hold during the following leg, stable
# across all four corners) -- strong evidence the compounding at depth 2
# was specifically carryValid_ persisting continuously across MULTIPLE
# hops (never invalidated, since Planner::activateNext() only drops it
# when the pending queue is briefly empty), not a bug in this script's
# own enqueue/ack bookkeeping. Diagnosing further would mean reading
# (or instrumenting) planner.cpp's own carry-adoption arithmetic across
# a live chain, which is firmware work -- out of this OOP fix's own
# host-Python-only scope. CHAIN_DEPTH stays a named, easily-bumped
# constant so a future ticket that actually chases this down can raise
# it once the planner side is understood; shipping depth 2-3 today would
# violate this rewrite's own acceptance bar ("a chained tour that closes
# worse than sequential is a failure, not a tradeoff").
#
# Depth 1 still buys real time: it removes the 0.1s INTER_SEGMENT_DWELL
# at every boundary (a script-level constant, so this part transfers
# 1:1 to hardware -- 8 segments x 0.1s = 0.8s) and detects completion
# from the ack ring instead of `_awaitMove()`'s `active`-edge wait, which
# should also shrink whatever share of the hardware-measured 0.52s
# planner-settle lag was `active`-flag-observation latency rather than
# real plant coast. It does NOT hide the ~0.21s command round trip (the
# next move is still sent only after this one's completion is observed).
#
# MEASURED, sim, true firmware time (not `Tour.elapsed` -- see below):
# sequential 21.16s vs chained(depth=1) 20.04s, a 1.12s reduction (~5%)
# for this 8-segment/~2m square -- consistent with the 0.8s of dwell
# removal plus a small (~0.3s) completion-detection-latency saving, NOT
# with the full ~5-6s a naive hardware extrapolation would suggest. Sim
# is not a reliable stand-in for the hardware number: the sim plant has
# far less coast/settle lag than the hardware-measured 0.52s figure this
# module's own history cites, and an in-process sim command has no real
# serial/radio round trip to hide at all (the ~0.21s figure is a
# measured HARDWARE latency with no sim analogue). The 0.8s of dwell
# removal is the one component sim and hardware should agree on; the
# rest genuinely needs the stakeholder's own bench comparison (this
# ticket's own "Do NOT run against hardware" boundary) to know for real.
#
# `Tour.elapsed` itself is NOT a reliable timing metric to compare
# across paths: `_awaitMove()`'s 0.1s poll slices into three
# SimBackend.advance() calls (0.04+0.04+0.02s) whose last slice rounds
# under Python's round-half-to-even to 0 steps, floored back up to 1 by
# `max(1, ...)` -- so every `_awaitMove()` poll silently steps 0.12s of
# REAL sim firmware time while crediting only 0.1s to `self.elapsed`,
# a small drift that adds up over a full tour (measured: sequential's
# own `self.elapsed` undercounts its OWN true firmware time by ~3.4s
# over this same tour). `_awaitAck()`'s CYCLE_S-granularity poll has no
# such rounding error. Comparing `tour.elapsed` between the two paths
# directly would have UNDERSTATED sequential's real duration and made
# chaining look better than it measured -- true firmware time (counting
# actual SimBackend.advance() calls) is the only apples-to-apples sim
# metric, which is what the 21.16s/20.04s figures above use.
CHAIN_DEPTH = 1  # [in-flight MOVEs] this active + this many queued ahead of
                 # it. See the comment above for why this is 1, not the
                 # 2-3 originally intended -- Motion::Planner's 5-deep
                 # queue (1 active + 4 pending, kQueueDepth) has ample
                 # headroom; the limit here is measured accuracy, not
                 # queue capacity.
ERR_FULL = 4     # envelope.proto ErrCode.ERR_FULL -- "destination queue full"

DEFAULT_PORT = "/dev/cu.usbmodem2121102"
DEFAULT_ROBOT_JSON = _REPO_ROOT / "data" / "robots" / "tovez_nocal.json"

# Pass bounds. Deliberately looser than the `wheel-layer-v1` reference run
# (359.9/360 deg, 19 mm): this gate is asking "did the command path break
# the tour", not "is the tour as good as it has ever been" -- a tight bound
# would make it a flaky motion-accuracy test instead.
DEFAULT_MAX_CLOSURE = 60.0    # [mm]
DEFAULT_HEADING_TOLERANCE = 15.0  # [deg] around 360

# Per-wheel command correction (measured steady / commanded), measured by
# the warm-up segment at the start of EVERY run -- the plant's gain wanders
# with temperature/usage state, so baked constants overcorrect within a
# session. Filled in by the calibration prelude.
CAL = {"L": 1.0, "R": 1.0}


def meanFactor(duration):  # [s] mean/steady speed ratio incl. spin-up
    if duration <= 0:
        return 1.0
    return 1.0 - (TAU / duration) * (1.0 - math.exp(-duration / TAU))


def durationFor(distance, steady, reversal):  # [mm] [mm/s] -> [s]
    # From-rest segment: the commanded window covers (distance - coast); the
    # post-expiry coast (~steady * TAU) delivers the rest.
    target = max(distance - steady * TAU, distance * 0.3)
    d = target / steady
    for _ in range(6):  # fixed point: duration -> transient factor
        eff = d - (DWELL if reversal else 0.0)
        d = target / (steady * meanFactor(eff)) + (DWELL if reversal else 0.0)
    return d


# Move ids must be unique PER RUN, not fixed constants.
#
# The firmware keeps a 16-slot ring of accepted move ids so a retried enqueue
# whose ack was lost is not executed twice (robot_loop.cpp alreadyAccepted()).
# It ACKS a duplicate id and discards it. The robot does not reboot between
# runs over the radio, so fixed ids meant the second and later runs had their
# segments silently skipped -- proven on hardware: the same move_id sent
# twice moved 50.7 mm then 0.1 mm, both acked.
#
# Skipping the TURNS turns four legs into one 2 m straight line, which is how
# the robot ended up driving off the table. The script could not detect it:
# an ack is indistinguishable from "already accepted".
#
# Base is derived from the clock so consecutive runs never collide, and kept
# stable within a run so sendVerified()'s own retry-on-lost-ack still works.
_ID_BASE = 10000 + (int(time.time()) % 50000) * 10


def segments() -> "list[tuple[float, float, float, int]]":
    """(vLeft, vRight, duration_ms, move_id) per segment. Commands are
    calibration-corrected so the MEASURED wheel speeds hit the targets;
    durations account for the spin-up transient (and the reversal dwell on
    the sign-flipping wheel during turns)."""
    legMs = durationFor(LEG, CRUISE, reversal=False) * 1000.0
    turnMs = durationFor(TURN_ARC, TURN_SPEED, reversal=True) * 1000.0
    legL = CRUISE / CAL["L"]
    legR = CRUISE / CAL["R"]
    turnL = -TURN_SPEED / CAL["L"]
    turnR = TURN_SPEED / CAL["R"]
    out = []
    for i in range(4):
        out.append((legL, legR, legMs, _ID_BASE + 1 + 2 * i))
        out.append((turnL, turnR, turnMs, _ID_BASE + 2 + 2 * i))
    return out


# ---------------------------------------------------------------------------
# Backends -- everything that genuinely differs between sim and hardware.
# ---------------------------------------------------------------------------


class _Backend:
    """A connected robot, real or simulated.

    `proto` is a plain ``NezhaProtocol`` on both -- that is the whole point
    of this split: the tour's command traffic is backend-agnostic, and only
    "let `seconds` of robot time pass" and "where is the robot really"
    need a backend-specific answer.
    """

    label = "?"

    def advance(self, seconds: float) -> "list":
        """Let `seconds` of ROBOT time pass, draining and returning every
        telemetry frame that arrived during it. This is the ONLY telemetry
        consumer in the script -- ack matching reads these same frames (see
        `Tour.sendVerified`) rather than draining a second time, which on a
        single-consumer queue would starve one of the two readers."""
        raise NotImplementedError

    def truePose(self) -> "tuple[float, float, float] | None":
        """Ground truth (x [mm], y [mm], heading [rad]), or None when the
        backend has none and the caller must integrate odometry itself."""
        return None

    def enableTelemetry(self) -> None:
        """Ensure unsolicited telemetry is flowing before any mode's FIRST
        telemetry read. No-op by default: only HardwareBackend needs this
        -- kAuto (the default emit policy) keeps a parked REAL robot
        silent until something moves it or asks explicitly (see that
        override's own docstring for the measured finding). SimBackend
        inherits this no-op rather than calling ``proto.tlmOn()`` itself:
        the sim has no emit-policy state machine to switch (it always
        emits), and ``SimConfigConn`` does not implement ``send_fast()``
        (only ``send_envelope``/``send_envelope_fast``, for CONFIG
        traffic) -- calling ``tlmOn()`` against it would raise
        AttributeError, not silently no-op."""
        return

    def disableTelemetry(self) -> None:
        """Undo enableTelemetry() on the way out. No-op by default; see
        that method's docstring."""
        return

    def close(self) -> None:
        raise NotImplementedError


class SimBackend(_Backend):
    label = "sim"

    def __init__(self, robot_json: "str | pathlib.Path", realTime: bool = False) -> None:
        from robot_radio.config.robot_config import load_robot_config
        from robot_radio.io.sim_config import SimConfigConn
        from robot_radio.io.sim_loop import SimLoop
        from robot_radio.robot.protocol import NezhaProtocol

        robot_config = load_robot_config(robot_json)
        _set_effective_track(robot_config)
        track = robot_config.trackwidth if robot_config.trackwidth is not None else TRACK
        self._sim = SimLoop(track_width=track)
        self._realTime = realTime
        # Deterministic manual stepping, no tick thread (the DEFAULT): this
        # script is the single consumer of the sim's own telemetry queue,
        # and real-time thread scheduling would only add jitter to a
        # measurement that does not need wall-clock realism at all.
        #
        # realTime=True (--mode goto only, 127-007): `pathplan.planner.
        # gotoWorld()`'s own `_advance()` polls telemetry on WALL-CLOCK
        # time and never calls `SimLoop.step()` itself (matches
        # `test_pathplan_goto_convergence.py`'s own established pattern,
        # 127-006) -- it expects the sim to be advancing in the
        # background on its own tick thread. Manual per-slice stepping
        # (this class's default, used by every other mode in this script)
        # would leave the sim frozen while `gotoWorld()`'s poll loop spins
        # forever waiting for telemetry that never arrives.
        self._sim.connect(start_tick_thread=realTime)
        self._sim.configure_from_robot(robot_config)
        self.proto = NezhaProtocol(SimConfigConn(self._sim))

    def advance(self, seconds: float) -> "list":
        if self._realTime:
            # Wall-clock poll, mirroring HardwareBackend.advance() -- the
            # tick thread drains the C ABI into the queue itself each
            # iteration (see the manual-stepping branch's own comment
            # below for why that matters), so no manual step()/drain call
            # is needed or correct here.
            frames = []
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                frames.extend(self._sim.read_pending_binary_tlm_frames())
                time.sleep(0.005)
            return frames
        frames = []
        for _ in range(max(1, int(round(seconds / CYCLE_S)))):
            self._sim.step(1)
            # _drain_tlm_into_queue() explicitly: that pull off the C ABI
            # is normally the TICK THREAD's per-iteration job, and with no
            # tick thread running nothing else does it -- read_pending...()
            # alone would return an eternally empty queue. Same private
            # call turn_prediction_capture.py's own deterministic sink makes.
            self._sim._drain_tlm_into_queue()  # noqa: SLF001
            frames.extend(self._sim.read_pending_binary_tlm_frames())
        return frames

    def truePose(self) -> "tuple[float, float, float]":
        pose = self._sim.get_true_pose()
        return (pose["x"], pose["y"], pose["h"])

    def close(self) -> None:
        self._sim.disconnect()


class HardwareBackend(_Backend):
    label = "hardware"

    def __init__(self, port: str, robot_json: "str | pathlib.Path") -> None:
        import json

        from robot_radio.config.robot_config import load_robot_config
        from robot_radio.io.serial_conn import SerialConnection
        from robot_radio.robot.protocol import NezhaProtocol

        # The hardware backend previously read no robot config at all, so
        # every angle it computed used the physical 128 mm track -- see
        # EFFECTIVE_TRACK. That is the path that actually matters, since the
        # sim robot has no scrub.
        #
        # And it must be the ACTIVE robot, not this script's --robot-json
        # default: that default is tovez_nocal, the deliberately
        # UNCALIBRATED config the sim wants. Loading it on hardware silently
        # gave the host unity scrub (effective track 128 for a robot whose
        # measured effective track is 140.4) while the robot itself ran its
        # own baked tovez calibration -- host and firmware disagreeing about
        # the same robot.
        pointer = _REPO_ROOT / "data" / "robots" / "active_robot.json"
        active = robot_json
        if pointer.exists():
            spec = json.loads(pointer.read_text())
            if "path" in spec:
                active = _REPO_ROOT / spec["path"]
        cfg = load_robot_config(active)
        print(f"hardware robot config: {cfg.robot_name} ({pathlib.Path(active).name})")
        _set_effective_track(cfg)

        self._conn = SerialConnection(port=port)
        self._conn.connect()
        self.proto = NezhaProtocol(self._conn)
        print(f"connected; waiting {BOOT_WAIT:.0f}s for boot")
        time.sleep(BOOT_WAIT)
        self.proto.read_pending_binary_tlm_frames()  # discard the boot burst

        # kAuto (the default emit policy) keeps a parked real robot SILENT
        # until something moves it or asks explicitly -- force streaming-
        # always now, before ANY mode's first telemetry read, rather than
        # per-mode in main(). 'segments' mode never hit this (it sends a
        # Move first, so the robot is already emitting -- moving --  by
        # the time it reads telemetry); '--mode goto' reads telemetry
        # FIRST to seed WorldPose and hit it hard: MEASURED 2026-07-30
        # over the relay, 0 frames in 2s before this call, 67 frames in 3s
        # (~22 Hz) after. Same defect, same fix as
        # move_protocol_bench.py's own proto.tlmOn() call.
        self.enableTelemetry()

    geofence = None

    def enableTelemetry(self) -> None:
        """See `_Backend.enableTelemetry()`'s own docstring for why this
        override exists at all (SimBackend does not need one)."""
        self.proto.tlmOn()

    def disableTelemetry(self) -> None:
        """Undo enableTelemetry() on the way out. Best-effort, like
        estop() in close() below: a broken link on exit must not raise
        on top of -- or mask -- whatever real failure sent us here."""
        try:
            self.proto.tlmOff()
        except Exception:
            pass

    def advance(self, seconds: float) -> "list":
        frames = []
        deadline = time.monotonic() + seconds
        nextCheck = 0.0
        while time.monotonic() < deadline:
            frames.extend(self.proto.read_pending_binary_tlm_frames())
            now = time.monotonic()
            if self.geofence is not None and now >= nextCheck:
                self.geofence.check()      # raises GeofenceViolation
                nextCheck = now + 0.1
            time.sleep(0.005)
        return frames

    def close(self) -> None:
        # estop(), not stop(): "halt now" is ESTOP since the command-
        # ingestion rework -- stop() is now a QUEUED stop and would sit
        # behind anything still pending instead of killing the drivetrain.
        try:
            self.proto.estop()
        finally:
            self.disableTelemetry()
            self._conn.disconnect()


# ---------------------------------------------------------------------------
# The tour
# ---------------------------------------------------------------------------


class Tour:
    def __init__(self, backend: _Backend) -> None:
        self.backend = backend
        self.proto = backend.proto
        self.log = dict(t=[], velL=[], velR=[], posL=[], posR=[], cmdL=[], cmdR=[],
                        trueX=[], trueY=[], trueH=[])
        self.marks: "list[tuple[float, str]]" = []
        self.cmdL = 0.0
        self.cmdR = 0.0
        self.elapsed = 0.0  # [s] script-local clock, so both backends chart alike
        # corr_id/Move.id -> AckEntry, populated by record() from every
        # frame advance() drains. One dict serves BOTH ack kinds an id
        # here can key: an ENQUEUE ack (keyed by the envelope's own
        # corr_id, returned by move_twist()/wheels()) and a COMPLETION ack
        # (keyed by Move.id, this script's own segment moveId) -- the two
        # id spaces never collide (moveId starts at _ID_BASE >= 10000;
        # corr_id is a small per-connection counter), same assumption
        # sendVerified() already relied on before chaining existed.
        self._seenAcks: "dict[int, object]" = {}
        # Camera pose fix at every segment boundary (stakeholder mandate,
        # 2026-07-29, `.claude/rules/playfield-testing.md`) -- one dict per
        # boundary: {"label", "logIndex" (index into self.log at capture
        # time), "camera" ((x_cm, y_cm, yaw_rad) or None)}. Populated only
        # when a camera (backend.geofence) is present -- no-op on sim or
        # --no-geofence, which have no camera to fix against.
        self.fixes: "list[dict]" = []

    def captureFix(self, label: str) -> None:
        """Capture a camera pose fix at a segment boundary, at REST (the
        caller has already run the settle dwell). No-op when there is no
        camera (sim, or --no-geofence)."""
        geofence = getattr(self.backend, "geofence", None)
        if geofence is None:
            return
        self.fixes.append({
            "label": label,
            "logIndex": len(self.log["t"]) - 1,
            "camera": geofence.captureFix(label),
        })

    def restDwell(self) -> float:
        """CAMERA_FIX_DWELL if a camera fix is about to be taken (a geofence
        is armed, i.e. `captureFix()` above will not no-op), else the short
        INTER_SEGMENT_DWELL -- see the module-level comment on those two
        constants for the measured decomposition that justifies the split.
        Every `run()` iteration calls `captureFix()` right after its
        `run*()` call regardless of backend, so this mirrors that same
        no-op condition rather than duplicating a second one."""
        armed = getattr(self.backend, "geofence", None) is not None
        return CAMERA_FIX_DWELL if armed else INTER_SEGMENT_DWELL

    def record(self, frames: "list") -> None:
        truth = self.backend.truePose()
        for f in frames:
            for ack in f.acks:
                self._seenAcks[ack.corr_id] = ack
            if f.enc_left is None:
                continue
            self.log["t"].append(self.elapsed)
            self.log["velL"].append(f.enc_left.velocity)
            self.log["velR"].append(f.enc_right.velocity)
            self.log["posL"].append(f.enc_left.position)
            self.log["posR"].append(f.enc_right.position)
            self.log["cmdL"].append(self.cmdL)
            self.log["cmdR"].append(self.cmdR)
            if truth is not None:
                self.log["trueX"].append(truth[0])
                self.log["trueY"].append(truth[1])
                self.log["trueH"].append(truth[2])

    def advance(self, seconds: float) -> "list":
        """Advance robot time in short slices, sampling ground truth once
        per slice so the sim's truth trace has the same resolution as the
        telemetry it rides alongside. Returns every frame drained across
        all slices (in order), so callers can inspect completion state
        (e.g. `_awaitMove`'s `active` flag) without a second, starving
        drain of the backend's telemetry queue -- see `sendVerified`'s own
        docstring on why there is only ever one consumer.

        Previously returned nothing: every frame still reached `record()`,
        but a caller like `_awaitMove` that needed to SEE the frames (not
        just have them logged) got `None` back on every call, which made
        its early-exit-on-completion branch permanently unreachable and
        left every segment riding out its full timeout budget instead of
        stopping the instant the move actually finished."""
        out = []
        remaining = seconds
        while remaining > 1e-6:
            slice_s = min(CYCLE_S, remaining)
            frames = self.backend.advance(slice_s)
            self.record(frames)
            out.extend(frames)
            self.elapsed += slice_s
            remaining -= slice_s
        return out

    def sendVerified(self, vL: float, vR: float, durationMs: float, moveId: int) -> bool:
        """Send one WHEELS segment and confirm the firmware acked it,
        retrying up to 4 times -- genuine, if modest, inbound loss is why
        the retry loop exists. MEASURED (ticket 135-001, 2026-08-05,
        tovez): 0% loss on direct serial and 3.5% over the radio relay at a
        steady 20Hz/200-cmd rate, with the firmware's own command-ring-full
        fault bit clear throughout on both transports (genuine link loss,
        not firmware backpressure) -- see
        `src/tests/bench/command_loss_bench.py`'s own module docstring and
        this sprint's ticket 001 Completion Notes for the full breakdown.
        This SUPERSEDES the previously-cited, unsourced "~20% of inbound
        command packets... dropped by the USB->UART bridge" figure.

        Ack matching scans the frames `advance()` already drained, not a
        second `wait_for_ack()` drain: on a single-consumer telemetry queue
        two readers starve each other, and on the sim nothing would be
        stepping the loop while `wait_for_ack()` slept.
        """
        for _ in range(4):
            corr = self.proto.wheels(vL, vR, durationMs, move_id=moveId)
            waited = 0.0
            while waited < 0.5:  # [s] ack deadline per attempt
                self.advance(CYCLE_S)
                waited += CYCLE_S
                if corr in self._seenAcks:
                    self.cmdL, self.cmdR = vL, vR
                    return True
        print(f"WARNING: segment {moveId} never acked")
        return False

    turnMode = "move"       # closed-loop corners; see runTurnMove()
    legMode  = "move"       # closed-loop legs;    see runLegMove()
    movingPrelude = False   # stationary prelude is the DEFAULT: it must
                            # not move the robot on the playfield.
    chainSegments = True    # segment chaining is the DEFAULT when eligible
                            # (see _canChain()); --sequential-segments in
                            # main() forces this False for A/B comparison.

    def _legMoveArgs(self) -> "tuple[dict, float]":
        """(move_twist kwargs, timeout [ms]) for a 500 mm DISTANCE-stopped
        leg -- the exact args runLegMove() sends, factored out so the
        chained path (_buildMoveJobs()) builds the identical command."""
        timeout = LEG / CRUISE * 1000.0 * 3.0 + 3000.0
        return dict(v_x=CRUISE, v_y=0.0, omega=0.0, stop_distance=LEG), timeout

    def _turnMoveArgs(self) -> "tuple[dict, float]":
        """(move_twist kwargs, timeout [ms]) for a 90 deg ANGLE-stopped
        corner -- the exact args runTurnMove() sends, factored out so the
        chained path (_buildMoveJobs()) builds the identical command."""
        # vLeft<0 / vRight>0 in the WHEELS form is positive omega; keep the
        # same rotation sense so the square runs the same way round.
        omega = 2.0  # [rad/s]
        rad = math.pi / 2.0
        timeout = rad / omega * 1000.0 * 3.0 + 3000.0
        return dict(v_x=0.0, v_y=0.0, omega=omega, stop_angle=rad), timeout

    def runLegMove(self, label: str, moveId: int) -> bool:
        """A 500 mm leg as a DISTANCE-stopped MOVE, not a timed WHEELS run.

        Measured: a timed WHEELS leg curved 8 cm north over 48 cm east (the
        geofence aborted on it), because equal commanded wheel speeds do not
        produce equal actual speeds and nothing closes the loop. The same
        distance driven as a MOVE held ~1 cm of cross-track over 40 cm
        earlier today, and 99-100% of commanded distance over five legs.
        """
        self.marks.append((self.elapsed, label))
        kwargs, timeout = self._legMoveArgs()
        self.backend.proto.move_twist(timeout=timeout, move_id=moveId, **kwargs)
        self._awaitMove(timeout, label, moveId)
        return True

    def _awaitMove(self, timeout: float, label: str, moveId: int) -> None:
        """Wait for the move's completion signal (telemetry `active` flag
        rising then falling), returning the instant it is observed.
        `timeout` (plus a 2s safety margin) is a BACKSTOP against a lost
        command or a stalled move, not the normal path.

        Previously `self.advance()` returned nothing, so `frames` here was
        always falsy and completion could never be observed -- every
        segment silently rode out this entire backstop deadline on every
        run (fixed by `Tour.advance()` now returning the frames it
        drains). If the backstop still fires, print a LOUD, greppable
        warning distinguishing the two distinct failure modes, since they
        have completely different causes: the move never started
        (`active` never rose -- likely a lost command) vs. it started but
        never finished (`active` rose but never fell -- the move itself
        stalled, or its completion frame was lost)."""
        seen = False
        start = time.monotonic()
        budget = timeout / 1000.0 + 2.0
        deadline = start + budget
        while time.monotonic() < deadline:
            frames = self.advance(0.1)
            if frames:
                if frames[-1].active:
                    seen = True
                elif seen:
                    self.advance(self.restDwell())
                    return
        waited = time.monotonic() - start
        reason = ("started but never completed -- 'active' rose but never "
                  "dropped" if seen else
                  "never started -- 'active' never rose (command likely lost)")
        print(f"TIMEOUT (hey jackass, it timed out): {label} (move {moveId}) "
              f"{reason}; waited {waited:.1f}s of a {budget:.1f}s budget")
        self.advance(self.restDwell())

    def runTurnMove(self, label: str, moveId: int) -> bool:
        """A 90 deg corner as an ANGLE-stopped MOVE, not a timed WHEELS pivot.

        A timed pivot has to PREDICT how far the wheels coast; measured, it
        over-rotates ~14% (sim heading 412.7 for a target 360 once the old
        prelude's duration fudge is removed). The old moving prelude hid that
        behind a per-run scale factor bought with 40 cm of travel.

        An ANGLE stop instead closes the loop on the estimator heading, and
        the firmware now applies the camera-measured affine turn calibration
        (RobotLoop::setRotationCalibration) on top -- 180 deg commands landed
        at 180.3 (sd 1.9) over six runs, and 15..180 deg across four rates
        held to mean +0.64 deg. No duration to guess and nothing to
        pre-measure, so the prelude does not have to move.
        """
        self.marks.append((self.elapsed, label))
        kwargs, timeout = self._turnMoveArgs()
        self.backend.proto.move_twist(timeout=timeout, move_id=moveId, **kwargs)
        self._awaitMove(timeout, label, moveId)
        return True

    def runSegment(self, label: str, vL: float, vR: float, durationMs: float,
                   moveId: int) -> bool:
        self.marks.append((self.elapsed, label))
        if not self.sendVerified(vL, vR, durationMs, moveId):
            return False
        # The ack round trip already consumed part of the window; drive the
        # rest of it, then rest so the next segment starts from rest (the
        # condition the calibration was measured under) -- CAMERA_FIX_DWELL
        # when a fix follows, else the short INTER_SEGMENT_DWELL; see
        # restDwell().
        self.advance(durationMs / 1000.0)
        self.cmdL = self.cmdR = 0.0
        self.advance(self.restDwell())
        return True

    # --- segment chaining (out-of-process, 2026-07-30) -- see the module-
    # level CHAIN_DEPTH comment for the design and its three constraints ---

    def _buildMoveJobs(self, segs) -> "list[tuple[str, dict, float, int]]":
        """One (label, move_twist kwargs, timeout [ms], moveId) job per
        segment in `segs` (run()'s own scaled segment list) -- built from
        _legMoveArgs()/_turnMoveArgs(), the SAME args runLegMove()/
        runTurnMove() send, so a chained tour drives identical commands to
        the sequential one. `segs`' own vL/vR/durationMs fields are the
        WHEELS-shaped values runSegment() needs and go unread here; only
        its (index, moveId) pairing is used, so label numbering matches
        _runSequential()'s exactly."""
        jobs = []
        for idx, (_vL, _vR, _durMs, moveId) in enumerate(segs):
            label = f"leg {idx // 2 + 1}" if idx % 2 == 0 else f"turn {idx // 2 + 1}"
            kwargs, timeout = (self._turnMoveArgs() if idx % 2 == 1
                              else self._legMoveArgs())
            jobs.append((label, kwargs, timeout, moveId))
        return jobs

    def _awaitAck(self, corrId: int, budget: float, label: str, kind: str) -> bool:
        """Wait up to `budget` [s] for `corrId`'s ack to land in
        `self._seenAcks` (populated by record() from every frame
        advance() drains -- see sendVerified()'s own docstring for why
        this is the ONLY telemetry consumer this script ever runs).
        `kind` ("enqueue" or "completion") only labels the timeout
        message, so a chained wait that times out says which ack it was
        waiting on, not just that it timed out."""
        start = time.monotonic()
        while time.monotonic() - start < budget:
            self.advance(CYCLE_S)
            if corrId in self._seenAcks:
                return True
        waited = time.monotonic() - start
        print(f"TIMEOUT (hey jackass, it timed out): {label} {kind} ack for "
              f"id {corrId} never arrived; waited {waited:.1f}s of a "
              f"{budget:.1f}s budget")
        return False

    def _enqueueMove(self, kwargs: dict, timeout: float, moveId: int,
                     replace: bool, label: str) -> bool:
        """Send one planner MOVE (move_twist) with `replace` passed
        EXPLICITLY (constraint 1, module docstring: the wire default
        `True` would flush the queue and preempt whatever chained move is
        already running), confirming the ENQUEUE ack and retrying on a
        lost ack or an error response -- ERR_FULL in particular is handled
        by retrying, never by aborting (this method's own contract; see
        the module docstring's CHAIN_DEPTH comment). `moveId` is the SAME
        id across every retry of this one segment -- safe, since the
        firmware's 16-slot accepted-id ring dedups a retried enqueue whose
        ack was lost (constraint 3) -- but every segment across the whole
        tour still gets its own unique id (segments()'s own _ID_BASE
        scheme)."""
        for attempt in range(6):
            corr = self.backend.proto.move_twist(
                timeout=timeout, replace=replace, move_id=moveId, **kwargs)
            if self._awaitAck(corr, 0.5, label, "enqueue"):
                ack = self._seenAcks.get(corr)
                if ack is not None and ack.ok:
                    return True
                errCode = getattr(ack, "err_code", None)
                full = " (ERR_FULL -- queue full)" if errCode == ERR_FULL else ""
                print(f"{label}: enqueue for move {moveId} err={errCode}{full} "
                      f"(attempt {attempt + 1}/6) -- retrying")
            self.advance(0.1)
        print(f"WARNING: segment {moveId} ({label}) never enqueued after 6 attempts")
        return False

    def _awaitCompletion(self, moveId: int, timeout: float, label: str) -> None:
        """Wait for `moveId`'s COMPLETION ack -- keyed by Move.id, NOT the
        enqueue corr_id (constraint 2, module docstring): under chaining
        the telemetry `active` flag stays asserted continuously across
        the whole chain, so `_awaitMove()`'s active-rise-then-fall edge
        (the sequential path's own completion signal) never fires here.
        `timeout` [ms] is the Move's own safety-backstop timeout; the
        wait budget adds the same 2s margin `_awaitMove()` uses.
        Non-fatal on timeout, matching `_awaitMove()`'s own contract (a
        logged warning, not a tour abort) -- only a failed ENQUEUE
        (`_enqueueMove()`) aborts the tour."""
        budget = timeout / 1000.0 + 2.0
        self._awaitAck(moveId, budget, label, "completion")

    def _canChain(self) -> bool:
        """Eligibility for the chained path (_runChained()): both legs and
        turns must go through the planner (turnMode == legMode == "move"
        -- WHEELS bypasses Motion::Planner entirely, per wheels()'s own
        docstring, so there is no queue to chain through), chaining must
        not be explicitly disabled (self.chainSegments, the A/B flag), and
        no camera fix may be due at any boundary. That third condition is
        equivalent to "no geofence armed": captureFix() no-ops exactly
        when self.backend.geofence is None (its own docstring), and a
        queued successor activates the instant the active move completes
        -- before any dwell could settle the chassis for a fix."""
        return (self.turnMode == "move" and self.legMode == "move"
                and self.chainSegments
                and getattr(self.backend, "geofence", None) is None)

    def _runSequential(self, segs) -> bool:
        """The original send-one -> await-completion -> dwell -> send-next
        per-segment loop (unchanged), kept as the explicit A/B baseline
        against _runChained() and as the REQUIRED path whenever a camera
        fix is due at a boundary -- see _canChain()."""
        for idx, (vL, vR, durMs, moveId) in enumerate(segs):
            label = f"leg {idx // 2 + 1}" if idx % 2 == 0 else f"turn {idx // 2 + 1}"
            isTurn = idx % 2 == 1
            if isTurn and self.turnMode == "move":
                ok = self.runTurnMove(label, moveId)
            elif not isTurn and self.legMode == "move":
                ok = self.runLegMove(label, moveId)
            else:
                ok = self.runSegment(label, vL, vR, durMs, moveId)
            if not ok:
                return False
            self.captureFix(label)
        return True

    def _runChained(self, segs) -> bool:
        """Drive every segment by keeping CHAIN_DEPTH MOVEs enqueued in
        Motion::Planner's own queue at once, instead of
        _runSequential()'s send -> await-completion -> dwell -> send-next
        loop. Only called when _canChain() is true.

        Marks are appended to line up with when each segment actually
        starts, not when it was enqueued: job 0 is marked at send time (it
        starts immediately, replace=True), and job i+1 -- already sitting
        in the queue, one command ahead the whole time -- is marked the
        instant job i's completion ack arrives, since activateNext() pops
        it on the very next firmware tick.

        Chaining does NOT make the square flow through corners at speed:
        shapesCompatible() (planner.cpp) refuses an at-speed hand-off
        across a Linear/Angular axis change, so the robot still comes to
        rest at every corner (see the CHAIN_DEPTH comment above the
        module's constants). What this removes is the INTER_SEGMENT_DWELL
        and completion-detection lag between segments, not the stop
        itself -- and at CHAIN_DEPTH=1 specifically, not the enqueue
        round trip either; see that same comment.
        """
        jobs = self._buildMoveJobs(segs)
        n = len(jobs)
        if n == 0:
            return True

        label0, kwargs0, timeout0, moveId0 = jobs[0]
        self.marks.append((self.elapsed, label0))
        if not self._enqueueMove(kwargs0, timeout0, moveId0, replace=True, label=label0):
            return False
        enqueued = 1
        while enqueued < min(CHAIN_DEPTH, n):
            label, kwargs, timeout, moveId = jobs[enqueued]
            if not self._enqueueMove(kwargs, timeout, moveId, replace=False, label=label):
                return False
            enqueued += 1

        for i in range(n):
            label, _kwargs, timeout, moveId = jobs[i]
            self._awaitCompletion(moveId, timeout, label)
            if i + 1 < n:
                self.marks.append((self.elapsed, jobs[i + 1][0]))
            self.captureFix(label)  # no-op: _canChain() requires no geofence
            if enqueued < n:
                nLabel, nKwargs, nTimeout, nMoveId = jobs[enqueued]
                if not self._enqueueMove(nKwargs, nTimeout, nMoveId, replace=False,
                                         label=nLabel):
                    return False
                enqueued += 1
        return True

    # --- calibration prelude (unchanged in intent from wheel-layer-v1) ---

    def calibrateStationary(self) -> float:
        """The no-motion prelude. Returns the turn scale; fills CAL.

        THE PRELUDE MUST NOT MOVE THE ROBOT (stakeholder, 2026-07-29). The
        old moving prelude drove ~39 cm to measure per-wheel gain and then
        pivoted and un-pivoted to measure a turn scale. On a 89.3 cm field
        that displacement is half the height -- it put the robot into the
        north rail -- and it is also the "turned right, then turned left"
        the stakeholder saw before the tour proper had even started.

        Neither measurement is needed:

        - Per-wheel gain: App::Drive ALREADY inverts
          `actual = gain*commanded + intercept`, per wheel and per direction
          of approach, from the robot JSON's wheel_gain_*/wheel_intercept_*
          (drive.cpp correctedCommand(), seeded in main.cpp). Measuring a
          host-side gain on top of that double-corrects -- it fits the
          RESIDUAL of an already-closed correction, which is why three
          consecutive runs produced L 0.819/0.873/1.200 for one robot. Unity
          is the honest value here.

        - Turn scale: TURN_ARC is now built from the EFFECTIVE track
          (trackwidth / rotational_slip), which is itself camera-measured,
          so the commanded pivot arc is already the right arc for 90 deg.

        Anything this cannot know from configuration is better measured by a
        dedicated calibration run than by stealing 40 cm from every tour.
        """
        CAL["L"] = 1.0
        CAL["R"] = 1.0
        print(f"prelude: stationary (no motion). "
              f"wheel gains from firmware correction, "
              f"turn arc from effective track {EFFECTIVE_TRACK:.1f} mm")
        return 1.0

    def calibrateMoving(self) -> float:
        """The ORIGINAL moving prelude -- retained for the bench, where the
        40 cm of travel it needs is free. Never the default; see
        calibrateStationary() for why it must not run on the playfield."""
        print("calibrating straight...")
        self.sendVerified(CRUISE, CRUISE, 3500.0, _ID_BASE + 98)
        base = len(self.log["t"])
        self.advance(4.2)
        rows = list(zip(self.log["posL"][base:], self.log["posR"][base:]))
        # Per-wheel steady gain from POSITION SLOPES over a window well
        # inside the move -- immune to ack-retry latency and to the
        # coast-down at the end (the bug in the tagged tour's first
        # version, which measured across the whole capture).
        n = len(rows)
        if n < 8:
            print("WARNING: too few frames to calibrate; using unity gains")
            return 1.0
        lo, hi = int(n * 0.3), int(n * 0.85)
        dt = (self.log["t"][base + hi] - self.log["t"][base + lo])  # [s]
        if dt <= 0:
            return 1.0
        CAL["L"] = max(0.5, min(1.5, ((rows[hi][0] - rows[lo][0]) / dt) / CRUISE))
        CAL["R"] = max(0.5, min(1.5, ((rows[hi][1] - rows[lo][1]) / dt) / CRUISE))
        self.cmdL = self.cmdR = 0.0
        self.advance(1.0)
        print(f"cal factors: L {CAL['L']:.3f}  R {CAL['R']:.3f}")

        # One test pivot at the calibrated turn commands and the nominal
        # duration: measured degrees -> turn-duration scale.
        turnMsNominal = durationFor(TURN_ARC, TURN_SPEED, reversal=True) * 1000.0
        tL = -TURN_SPEED / CAL["L"]
        tR = TURN_SPEED / CAL["R"]
        print("calibrating turn...")
        base = len(self.log["t"])
        self.sendVerified(tL, tR, turnMsNominal, _ID_BASE + 97)
        self.advance(turnMsNominal / 1000.0 + 1.5)
        if len(self.log["t"]) - base < 4:
            return 1.0
        dL = self.log["posL"][-1] - self.log["posL"][base]
        dR = self.log["posR"][-1] - self.log["posR"][base]
        # PHYSICAL_TRACK, not EFFECTIVE_TRACK: this INTERPRETS the encoder
        # delta as an actual rotation, the same job headingDeg/
        # integrateEncoderPath do -- see the module-level PHYSICAL_TRACK/
        # EFFECTIVE_TRACK comment.
        degMeasured = abs(math.degrees((dR - dL) / PHYSICAL_TRACK))
        turnScale = 90.0 / degMeasured if degMeasured > 10 else 1.0
        print(f"test pivot: {degMeasured:.1f} deg -> turn duration scale {turnScale:.3f}")
        self.cmdL = self.cmdR = 0.0
        self.advance(1.0)

        # Un-rotate so the tour starts square, with the test pivot undone.
        self.sendVerified(-tL, -tR, turnMsNominal * turnScale, 9596)
        self.advance(turnMsNominal * turnScale / 1000.0 + 1.2)
        self.cmdL = self.cmdR = 0.0
        self.advance(1.0)
        return turnScale

    def run(self) -> bool:
        turnScale = (self.calibrateMoving() if self.movingPrelude
                     else self.calibrateStationary())
        segs = [(vL, vR, dur * (turnScale if idx % 2 == 1 else 1.0), mid)
                for idx, (vL, vR, dur, mid) in enumerate(segments())]
        self._tourStartIndex = len(self.log["t"])
        self.captureFix("start")
        chained = self._canChain()
        print(f"segment chaining: {'ON' if chained else 'off'} "
              f"({'no camera-fix boundaries' if chained else 'sequential -- see Tour._canChain()'})")
        ok = self._runChained(segs) if chained else self._runSequential(segs)
        if not ok:
            return False
        self.advance(1.0)  # final coast-down
        return True


# ---------------------------------------------------------------------------
# Results
# ---------------------------------------------------------------------------


def integrateEncoderPath(log, start: int):
    """Host-side odometry from the encoder positions -- the hardware
    backend's ground truth (and the chart's path on both backends when no
    plant truth is available)."""
    xs, ys = [0.0], [0.0]
    heading = 0.0
    for i in range(start + 1, len(log["t"])):
        dL = log["posL"][i] - log["posL"][i - 1]
        dR = log["posR"][i] - log["posR"][i - 1]
        ds = 0.5 * (dL + dR)
        # PHYSICAL_TRACK: interpreting encoders, not generating a command --
        # see the module-level PHYSICAL_TRACK/EFFECTIVE_TRACK comment.
        dTheta = (dR - dL) / PHYSICAL_TRACK
        heading += dTheta
        xs.append(xs[-1] + ds * math.cos(heading - dTheta / 2.0))
        ys.append(ys[-1] + ds * math.sin(heading - dTheta / 2.0))
    return xs, ys


def reportSegmentFixes(tour: "Tour", log: dict) -> None:
    """Per-segment report from the camera fixes captured at every segment
    boundary (stakeholder mandate, 2026-07-29, `.claude/rules/
    playfield-testing.md`): leg length / turn angle / per-leg cross-track
    from the camera, printed ALONGSIDE the encoder-odometry estimate for
    the SAME segment, plus overall closure computed from the first vs last
    fix. No-op when fewer than 2 fixes were captured (sim, --no-geofence,
    or a tour that aborted before its first boundary)."""
    fixes = tour.fixes
    if len(fixes) < 2:
        return

    print(f"\nper-segment camera vs. encoder-odometry ({len(fixes)} fixes):")
    for prev, cur in zip(fixes, fixes[1:]):
        label = cur["label"]
        isTurn = label.startswith("turn")

        i0, i1 = prev["logIndex"], cur["logIndex"]
        encLength = encAngle = None
        if 0 <= i0 < i1 < len(log["posL"]):
            dL = log["posL"][i1] - log["posL"][i0]
            dR = log["posR"][i1] - log["posR"][i0]
            encLength = 0.5 * (dL + dR)  # [mm]
            # PHYSICAL_TRACK: interpreting encoders -- see the module-level
            # PHYSICAL_TRACK/EFFECTIVE_TRACK comment.
            encAngle = math.degrees((dR - dL) / PHYSICAL_TRACK)  # [deg]

        camPrev, camCur = prev["camera"], cur["camera"]
        if camPrev is None or camCur is None:
            encTxt = (f"length={encLength:.1f}mm heading_delta={encAngle:+.1f}deg"
                      if encLength is not None else "no encoder samples either")
            print(f"  {label}: camera fix missing -- encoder only: {encTxt}")
            continue

        x0, y0, yaw0 = camPrev
        x1, y1, yaw1 = camCur
        dx, dy = x1 - x0, y1 - y0  # [cm]
        camLength = math.hypot(dx, dy) * 10.0  # [mm]
        dyaw = ((yaw1 - yaw0 + math.pi) % (2.0 * math.pi)) - math.pi
        camAngle = math.degrees(dyaw)  # [deg]

        if isTurn:
            encTxt = f"{encAngle:+.1f}deg" if encAngle is not None else "n/a"
            print(f"  {label}: turn angle -- camera {camAngle:+.1f}deg | "
                  f"encoder {encTxt}")
        else:
            crossTrack = (dx * -math.sin(yaw0) + dy * math.cos(yaw0)) * 10.0  # [mm]
            encTxt = f"{encLength:.1f}mm" if encLength is not None else "n/a"
            print(f"  {label}: leg length -- camera {camLength:.1f}mm | "
                  f"encoder {encTxt} | cross-track {crossTrack:+.1f}mm (camera)")

    first, last = fixes[0]["camera"], fixes[-1]["camera"]
    if first is not None and last is not None:
        closureCam = math.hypot(last[0] - first[0], last[1] - first[1]) * 10.0
        print(f"camera closure (first vs last fix): {closureCam:.1f} mm")
    else:
        print("camera closure: unavailable -- start or end fix missing")


# ---------------------------------------------------------------------------
# --mode goto (127-007): the same square, driven as gotoWorld() waypoints
# instead of open-loop WHEELS / closed-loop MOVE segments.
# ---------------------------------------------------------------------------

# GOTO_OVERSHOOT_BOUND / GOTO_STALL_WINDOW / GOTO_STALL_EPS -- RETIRED
# (out-of-process, 2026-07-30). Those three gated a PER-CORNER "did this
# one gotoWorld() call converge cleanly" analysis (`_TruePoseSampler`,
# removed alongside them) under the OLD call-gotoWorld()-once-per-waypoint
# model. Under the pass-through pursuit model (see the module-level
# comment above `pathplan.solver.hasPassedWaypoint()`/
# `advanceWaypointIndex()`), there is no longer a per-waypoint "arrival"
# event for an interior waypoint to measure overshoot/stall against at
# all -- the whole point of pass-through is that the robot never stops
# converging on any of them. `pathplan.planner.followPath()`'s own
# `GotoResult`-shaped return (`success`, `reason`, `iterations`, `sent`,
# `retries`, `forcedResends`, `unacked`) is the replacement diagnostic
# surface -- see `runGotoTour()` below.

# --- rounded-square geometry (out-of-process, 2026-07-30) -----------------
#
# `gotoSquareWaypoints()` used to emit the square's four SHARP vertices.
# From one vertex, the next is 90 deg off the current heading, so EVERY
# corner hits `solveArcToPoint()`'s target-behind guard
# (`SolverLimits.behindAngle`, default 90 deg) -- the guard exists so the
# solver never emits a near-infinite-curvature arc, and a square's own
# corners are exactly that degenerate case. The fix is geometric, not a
# guard change: round each corner into a quarter-circle fillet and sample
# it finely enough that the bearing to the NEXT waypoint never gets
# anywhere near 90 deg.
#
# CORNER_RADIUS is bounded on both sides -- derived, not guessed:
#
#   FLOOR (drivable): entering the fillet from a straight leg means the
#   commanded omega has to move from 0 toward the fillet's own
#   steady-state value, omega = CRUISE / r, without the solver's own
#   curvature slew limit (`solver.MAX_WHEEL_STEP` = 250 mm/s PER SOLVE at
#   the outer loop's ~10 Hz, converted to an omega-step budget via
#   `trackWidth == PHYSICAL_TRACK`) itself rounding the fillet's entry off
#   the intended tangent-circle shape:
#
#     omegaStepBudget = 2 * MAX_WHEEL_STEP / PHYSICAL_TRACK   [rad/s per solve]
#     radiusFloor      = CRUISE / omegaStepBudget
#                      = CRUISE * PHYSICAL_TRACK / (2 * MAX_WHEEL_STEP)
#
#   For this robot (CRUISE=150 mm/s, PHYSICAL_TRACK=128 mm,
#   MAX_WHEEL_STEP=250 mm/s) that is 150*128/500 ~= 38.4 mm -- the radius
#   at which the fillet's steady-state omega equals a SINGLE solve's
#   clamp step. `gotoSquareWaypoints()` recomputes this floor at call time
#   from the ACTUAL `PHYSICAL_TRACK`/`CRUISE` (set from the active robot
#   config by `_set_effective_track()`), rather than baking in this one
#   robot's numbers, and raises rather than silently emitting an
#   undrivable path if a caller's `legLength` forces the radius below it.
#
#   CEILING (recognisably a square): the playfield run this fix targets
#   used a 250 mm leg (the smallest leg this script drives -- see --leg's
#   own help text). Capping the radius at `legLength / 4` leaves at least
#   half of every edge (`leg - 2*r >= leg/2`) genuinely straight, so the
#   path still reads as a square with rounded corners, not a circle.
#
# TARGET_CORNER_RADIUS = 90 mm sits comfortably above the 38.4 mm floor
# (2.3x margin) for the sim-default 500 mm leg; `gotoSquareWaypoints()`
# clamps it down to `legLength / 4` whenever that ceiling is tighter (the
# 250 mm playfield case -> clamped to 62.5 mm, still >= the floor).
TARGET_CORNER_RADIUS = 90.0  # [mm] preferred corner radius; see derivation above

# Each fillet is sampled into CORNER_SEGMENTS_PER_CORNER equal hops of
# (90 / CORNER_SEGMENTS_PER_CORNER) deg apiece. For points equally spaced
# by angle theta around a circle, the direction change between consecutive
# CHORDS equals theta exactly (the exterior angle of an inscribed regular
# polygon) -- so this IS the bearing-to-next-waypoint the guard sees, not
# an approximation of it. 3 segments -> 30 deg per hop, a 3x margin under
# the 90 deg guard (comfortable headroom for the residual heading error a
# real "arrived within tolerance" stop leaves versus the idealized tangent
# chord). The straight run between one corner's last sample and the next
# corner's first sample needs no further subdivision: the robot's heading
# is already aligned with that edge when it leaves the fillet, so the
# bearing to the next corner's first sample is small by construction.
CORNER_SEGMENTS_PER_CORNER = 3

# Arrival tolerance for THIS dense waypoint sequence's TERMINAL waypoint
# ONLY (out-of-process, 2026-07-30): every INTERIOR waypoint is now
# pass-through, never arrival-tested -- see the module-level comment above
# `pathplan.solver.hasPassedWaypoint()`/`advanceWaypointIndex()` and
# `pathplan.planner.followPath()` for the advance rule that replaced
# per-waypoint arrival. The derivation below (why this value, not
# `TERMINATION_TOLERANCE`) predates that change and was written for a
# per-waypoint arrival test; it is kept because the NUMBER is still the
# right one for "how close is close enough to stop" at the end of this
# path, even though it no longer has to fit under every interior chord.
#
# Arrival tolerance for THIS dense waypoint sequence -- deliberately
# smaller than, and decoupled from, `pathplan.planner.TERMINATION_TOLERANCE`
# (100 mm, left UNCHANGED -- it is provisional pending ticket 007's own
# actuation-floor measurement, not this fix's to touch). That default is
# sized for a single far-off goal (keep a >=100 mm carrot ahead of a
# ~150 ms-lag outer loop); every waypoint here is already close by design
# (see CORNER_SEGMENTS_PER_CORNER above), so the risk runs the OTHER way --
# this ticket's own stated trap: a tolerance at or above spacing makes the
# loop "arrive" at a waypoint it never actually approached and skip ahead,
# cutting the path short (measured on this exact script pre-fix: a corner
# "converged" 96.8 mm from a 250 mm target under the 100 mm default).
#
# Must stay comfortably below the SMALLEST chord this module can generate
# -- the clamped-radius (leg/4) case. Chord length for N equal hops of a
# radius-r fillet is `2 * r * sin(pi / (2*N))`; for the 250 mm playfield
# leg (radius clamped to 62.5 mm, N=3) that is 2*62.5*sin(30deg) ~=
# 62.5 mm... using the exact per-hop angle (30 deg): 2*62.5*sin(15deg) ~=
# 32.4 mm. GOTO_PATH_TOLERANCE = 12 mm keeps >=2.7x margin under even that
# worst case (>=3.9x under the 500 mm sim-default leg's 46.6 mm chord).
GOTO_PATH_TOLERANCE = 12.0  # [mm] see derivation above


def _installEstopSignalHandler(backend: "_Backend") -> None:
    """estop() on SIGTERM/SIGINT before the process exits -- mirrors
    `planner_square_tour.py`'s own `_install_estop_signal_handler()`
    (127-002 lesson, hardware incident 2026-07-30: a bare SIGTERM bypasses
    every Python `finally` block, so the ordinary `try/finally: estop()`
    idiom alone left a background batch driving after an external kill --
    SIGINT/Ctrl-C was already safe, since it raises `KeyboardInterrupt`
    and IS caught by `finally`). Defense in depth, not a full guarantee --
    a SIGKILL cannot be caught by any process-level handler; the remaining
    gap is closed procedurally (run goto-mode hardware/playfield tours in
    the foreground, one at a time, never as an unsupervised background
    batch).

    Also calls `backend.disableTelemetry()` right after the estop (out-of-
    process, 2026-07-30): telemetry-off rides the SAME path as the estop
    here rather than fighting it -- both are best-effort (each wrapped in
    its own `try/except`) so a failure in one never suppresses the other,
    and disableTelemetry() is a no-op on sim (this handler is only
    installed for a real backend; see the call site)."""

    def _handler(signum: int, _frame) -> None:
        try:
            backend.proto.estop()
        except Exception:
            pass
        try:
            backend.disableTelemetry()
        except Exception:
            pass
        sys.exit(1)

    signal.signal(signal.SIGTERM, _handler)
    signal.signal(signal.SIGINT, _handler)


def gotoSquareWaypoints(startPose, legLength: float) -> "list[tuple[object, bool]]":
    """A `legLength` x `legLength` square, corners ROUNDED into quarter-
    circle fillets, sampled into dense WORLD-frame waypoints -- the
    goto-mode equivalent of `segments()`'s fixed leg/turn sequence, and
    the fix for the ORIGINAL sharp-vertex version of this function (see
    the module-level comment above `TARGET_CORNER_RADIUS` for why sharp
    vertices hit `solveArcToPoint()`'s target-behind guard at every
    corner). `gotoWorld()` drives to each returned waypoint in turn
    (continuously re-solving the arc, including through the fillets)
    rather than following a scripted wheel-speed sequence.

    Returns `[(Pose, isVertex), ...]`, laid out relative to the robot's
    OWN heading at `startPose` (forward = +x body, left = +y body), same
    CCW winding as the original 4-corner list. `isVertex` is True for
    exactly the LAST sample of each corner's fillet (4 total, one per
    corner) -- the geometrically meaningful boundary nearest that corner,
    and the ONLY points `runGotoTour()` takes a camera fix at (a
    median-of-7 fix at every one of the ~4*CORNER_SEGMENTS_PER_CORNER
    dense samples would be absurdly slow, and none of the intermediate
    fillet samples is a boundary anything downstream cares about).
    `startPose` is a `nav.pose.Pose` (cm, rad); every returned `Pose` has
    heading 0.0 -- `gotoWorld()`/`solveArcToPoint()` both ignore target
    heading (sprint.md's own "Out of Scope: terminal-theta honoring"), so
    there is no meaningful value to put there.

    THE RETURNED LIST IS A POLYLINE, NOT A BAG OF TARGETS (2026-07-31).
    `pathplan.planner.followPath()` follows the polyline THROUGH these
    points -- it projects the robot onto it and steers at a point one
    lookahead distance along it -- so every vertex's own local DIRECTION
    matters, not just its position. Two consequences this function now
    honours, and which the earlier "each point is somewhere to drive to"
    reading let it get wrong:

    * `startPose` is the FIRST element. The robot has to travel ~190 mm
      from where it is to the first fillet, and that stretch is part of
      the intended trajectory; leaving it out started the polyline 190 mm
      away from the robot with nothing describing how to get there.
    * Each fillet emits its INBOUND tangent point (`k = 0`) as its own
      vertex. That point coincides with where the preceding straight run
      was already headed -- which is exactly why it is needed: without it
      the fillet's first chord runs at 45 deg (the mean of the k=1 and k=2
      tangents) straight out of a vertex the robot reaches heading 0 deg,
      a 45 deg kink in a path that is supposed to be smooth. MEASURED
      2026-07-31: that kink alone made `followPath()` command omega
      = +4.9 rad/s (a ~30 mm turn radius at CRUISE) on entry to the first
      corner. With `k = 0` emitted, consecutive chords differ by the
      fillet's own per-hop angle (30 deg at `CORNER_SEGMENTS_PER_CORNER`
      = 3) and the first one is only half that off the inbound straight.
    """
    from robot_radio.nav.pose import Pose
    from robot_radio.pathplan.solver import MAX_WHEEL_STEP
    from robot_radio.pathplan.world_pose import Transform2

    radiusMm = min(TARGET_CORNER_RADIUS, legLength / 4.0)
    radiusFloorMm = CRUISE * PHYSICAL_TRACK / (2.0 * MAX_WHEEL_STEP)
    if radiusMm < radiusFloorMm:
        raise ValueError(
            f"gotoSquareWaypoints: leg {legLength:.0f} mm forces a corner "
            f"radius of {radiusMm:.1f} mm (leg/4, the 'recognisable square' "
            f"ceiling), below the {radiusFloorMm:.1f} mm drivable floor for "
            f"this robot (CRUISE={CRUISE:.0f} mm/s, "
            f"PHYSICAL_TRACK={PHYSICAL_TRACK:.0f} mm) -- pick a longer leg")
    radiusCm = radiusMm / 10.0  # [mm] -> [cm], nav.pose.Pose's own convention

    toWorld = Transform2(x=startPose.x, y=startPose.y, rotation=startPose.heading)
    legCm = legLength / 10.0
    corners = [(legCm, 0.0), (legCm, legCm), (0.0, legCm), (0.0, 0.0)]
    # edgeDirs[i] is the unit direction of the edge ARRIVING at corners[i];
    # edgeDirs[(i+1) % 4] is the unit direction LEAVING it -- both a 90 deg
    # CCW rotation of the previous edge, matching the square's own winding.
    edgeDirs = [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)]

    localPoints: "list[tuple[float, float, bool]]" = []
    for i in range(4):
        cx, cy = corners[i]
        dIn = edgeDirs[i]
        dOut = edgeDirs[(i + 1) % 4]
        # Fillet centre: offset from the sharp corner by radiusCm along
        # BOTH the inbound edge's own perpendicular and the outbound
        # edge's -- for these axis-aligned 90 deg corners that reduces to
        # centre = cornerPoint - radius*dIn + radius*dOut (verified: this
        # point is exactly radiusCm from both tangent points, each
        # perpendicular to its own edge).
        centreX = cx - dIn[0] * radiusCm + dOut[0] * radiusCm
        centreY = cy - dIn[1] * radiusCm + dOut[1] * radiusCm
        # The arc sweeps 90 deg CCW from the tangent point on the inbound
        # edge (angle = atan2(-dOut.y, -dOut.x) from the centre) to the
        # tangent point on the outbound edge. k=0 (the inbound tangent
        # point) IS emitted -- see this function's own docstring for why
        # skipping it put a 45 deg kink in the polyline.
        startAngle = math.atan2(-dOut[1], -dOut[0])
        for k in range(0, CORNER_SEGMENTS_PER_CORNER + 1):
            angle = startAngle + (math.pi / 2.0) * k / CORNER_SEGMENTS_PER_CORNER
            x = centreX + radiusCm * math.cos(angle)
            y = centreY + radiusCm * math.sin(angle)
            isVertex = k == CORNER_SEGMENTS_PER_CORNER
            localPoints.append((x, y, isVertex))

    # The robot's own start is the polyline's first vertex -- see the
    # docstring. isVertex False: it is not a corner, and runGotoTour()
    # already takes its own separate "start" camera fix.
    localPoints.insert(0, (0.0, 0.0, False))
    return [(toWorld.apply(Pose(x=x, y=y, heading=0.0)), isVertex)
            for x, y, isVertex in localPoints]


def reportGotoBoundaryFixes(fixes: "list[dict]") -> None:
    """Per-boundary report from goto mode's own camera fixes (start + one
    per waypoint) -- the goto-mode analogue of `reportSegmentFixes()`, but
    over corner-to-corner hops (position+heading combined in one number,
    since a `gotoWorld()` corner is not split into separate leg/turn stops
    the way the segmented tour's are). No-op with fewer than 2 fixes
    (sim, bench with no camera, or a tour that aborted before boundary 2)."""
    cameraFixes = [f for f in fixes if f["camera"] is not None]
    if len(cameraFixes) < 2:
        return
    print(f"\ngoto-mode per-boundary camera fixes ({len(cameraFixes)}/{len(fixes)} boundaries seen):")
    for prev, cur in zip(cameraFixes, cameraFixes[1:]):
        x0, y0, yaw0 = prev["camera"]
        x1, y1, yaw1 = cur["camera"]
        hop = math.hypot(x1 - x0, y1 - y0) * 10.0  # [mm]
        dyaw = ((yaw1 - yaw0 + math.pi) % (2.0 * math.pi)) - math.pi
        print(f"  {prev['label']} -> {cur['label']}: hop={hop:.1f}mm "
              f"heading_delta={math.degrees(dyaw):+.1f}deg")
    x0, y0, _ = cameraFixes[0]["camera"]
    x1, y1, _ = cameraFixes[-1]["camera"]
    print(f"camera closure (first vs last fix): {math.hypot(x1 - x0, y1 - y0) * 10.0:.1f} mm")


def writeGotoChart(results: "list[dict]", fixes: "list[dict]", waypoints: "list[tuple]",
                   closureWorld: "float | None", out: str, label: str) -> None:
    """Two-panel chart: left is the PATH panel (commanded rounded-square
    waypoints, the WorldPose trace actually driven, and the camera fixes
    taken at the four vertices -- all three overlaid in the SAME relative
    frame so they are comparable at a glance), right is per-corner
    convergence (or a summary table when no ground-truth sampler ran).

    The path panel previously rendered as a tiny strip with most of the
    figure blank: `axPath.set_aspect("equal")` alone (no `box_aspect`)
    only constrains the DATA limits to be square -- with a rectangular
    subplot box (this figure's own `width_ratios=[1, 1.4]` at
    `figsize=(14, 6)`), matplotlib's default `adjustable="box"` then
    SHRINKS the axes' own box to match that data aspect, leaving the
    unused remainder of the allotted subplot area blank around it. Adding
    `axPath.set_box_aspect(1)` fixes this the way matplotlib's own docs
    recommend for "square plot regardless of figure size": it tells the
    layout engine up front to allocate a square box (using the panel's
    full height), so there is no oversized rectangle to shrink out of in
    the first place -- `tight_layout()` sizes the LEFT COLUMN's width to
    match instead of leaving dead margin.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (axPath, axConv) = plt.subplots(
        1, 2, figsize=(14, 6), gridspec_kw={"width_ratios": [1, 1.4]})

    x0 = fixes[0]["pose"].x * 10.0 if fixes and fixes[0]["pose"] is not None else 0.0
    y0 = fixes[0]["pose"].y * 10.0 if fixes and fixes[0]["pose"] is not None else 0.0

    # Commanded path: every rounded-square waypoint this run ASKED
    # gotoWorld() to reach, in order -- the ideal/intended shape.
    # gotoSquareWaypoints() emits the robot's own start as waypoint 0, so
    # the polyline is already complete -- do not prepend it a second time.
    wpXs = [w.x * 10.0 - x0 for w, _isVertex in waypoints]
    wpYs = [w.y * 10.0 - y0 for w, _isVertex in waypoints]
    axPath.plot(wpXs, wpYs, color="#aaaaaa", lw=1.2, ls=":", marker=".",
               ms=4, label="commanded waypoints", zorder=1)

    # Driven path: WorldPose after every waypoint call (dense -- every
    # fillet sample, not just the four vertices) -- what the robot
    # actually reported doing.
    relXs, relYs, annotateLbls, annotateXs, annotateYs = [], [], [], [], []
    for f in fixes:
        if f["pose"] is None:
            continue
        rx, ry = f["pose"].x * 10.0 - x0, f["pose"].y * 10.0 - y0
        relXs.append(rx)
        relYs.append(ry)
        # Annotate only "start" and the four vertex labels -- annotating
        # every dense intermediate fillet sample too would be unreadable.
        if f["camera"] is not None or f["label"] == "start":
            annotateLbls.append(f["label"])
            annotateXs.append(rx)
            annotateYs.append(ry)
    axPath.plot(relXs, relYs, marker="o", ms=3, color="#1f77b4", lw=1.6,
               label="driven (WorldPose)", zorder=2)
    for lbl, x, y in zip(annotateLbls, annotateXs, annotateYs):
        axPath.annotate(lbl, xy=(x, y), fontsize=8, xytext=(4, 4), textcoords="offset points")

    # Camera fixes: the ground truth taken at the four vertices only (see
    # gotoSquareWaypoints()'s own docstring for why not every sample) --
    # plotted alongside the other two so a systematic WorldPose bias shows
    # up directly as a gap between the blue trace and these markers.
    camXs = [f["camera"][0] * 10.0 - x0 for f in fixes if f["camera"] is not None]
    camYs = [f["camera"][1] * 10.0 - y0 for f in fixes if f["camera"] is not None]
    if camXs:
        axPath.plot(camXs, camYs, marker="^", ms=8, mfc="none", mec="#d62728",
                   mew=1.8, ls="none", label="camera fix", zorder=3)

    axPath.set_aspect("equal", adjustable="datalim")
    axPath.set_box_aspect(1)  # square box regardless of figsize/width_ratios -- see docstring
    axPath.margins(0.12)
    axPath.set_xlabel("x [mm]")
    axPath.set_ylabel("y [mm]")
    axPath.set_title("goto-mode path: commanded vs. driven vs. camera")
    axPath.legend(fontsize=7.5, loc="best")
    axPath.grid(True, alpha=0.3)

    hasSamples = any(r.get("samples") for r in results)
    if hasSamples:
        for r in results:
            samples = r.get("samples") or []
            if not samples:
                continue
            axConv.plot([s[0] for s in samples], [s[3] for s in samples], label=r["label"])
        axConv.axhline(0, color="black", lw=0.5)
        axConv.set_xlabel("time within corner call [s]")
        axConv.set_ylabel("distance to target (ground truth) [mm]")
        axConv.set_title("per-corner convergence")
        axConv.legend(fontsize=8)
        axConv.grid(True, alpha=0.25)
    else:
        axConv.axis("off")
        summary = "\n".join(
            f"{r['label']}: success={r['result'].success} "
            f"it={r['result'].iterations} sent={r['result'].sent}"
            for r in results)
        axConv.text(0.02, 0.98, summary, va="top", fontsize=9, family="monospace")
        axConv.set_title("per-corner GotoResult summary")

    title = f"Square tour goto-mode ({label})"
    if closureWorld is not None:
        title += f" -- closure {closureWorld:.0f} mm"
    fig.suptitle(title, fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(out, dpi=130)
    print(f"wrote {out}")


def runGotoTour(backend: "_Backend", args) -> int:
    """--mode goto: the same square, driven as ONE continuous pure-pursuit
    run through the rounded-square waypoint sequence
    (`pathplan.planner.followPath()`, this IS ticket 008's own algorithm,
    built here per stakeholder directive 2026-07-30 -- see the
    module-level comment above `pathplan.solver.hasPassedWaypoint()`/
    `advanceWaypointIndex()` for the arrival-tolerance-floor-vs-chord-
    ceiling conflict this replaces), instead of calling `gotoWorld()` once
    per waypoint and requiring a full stop-and-arrive at every one. Full
    `CRUISE` speed throughout -- no goto-specific speed reduction; slowing
    down was proposed and explicitly rejected (see the module history).

    One `MoveIdAllocator` is shared across the whole call -- required by
    `planner.gotoWorld()`'s own docstring (equally true of `followPath()`,
    which shares its send machinery) for any caller issuing multiple
    sequential Moves in one robot boot session (a fresh allocator risks a
    later, genuinely new move being misread as a duplicate of an earlier
    one that reused the same low id) -- seeded from wall-clock time
    (matching this module's own `_ID_BASE` and 127-002's finding that the
    robot does not necessarily reboot between separate script
    invocations)."""
    from robot_radio.field import captureFixWithRetry
    from robot_radio.pathplan.planner import GiveUpLimits, MoveIdAllocator, followPath
    from robot_radio.pathplan.solver import SolverLimits
    from robot_radio.pathplan.world_pose import WorldPose

    proto = backend.proto
    if not args.sim:
        _installEstopSignalHandler(backend)

    worldPose = WorldPose()
    geofence = getattr(backend, "geofence", None)
    # GOTO_PATH_TOLERANCE (not planner.TERMINATION_TOLERANCE) is the
    # default here -- see that constant's own module-level derivation.
    # Under pass-through this gates ONLY `followPath()`'s terminal-waypoint
    # arrival test (every interior waypoint is pass-through, never
    # arrival-tested). --goto-tolerance still overrides explicitly when
    # given.
    tolerance = args.goto_tolerance if args.goto_tolerance is not None else GOTO_PATH_TOLERANCE

    # Seed WorldPose from one real telemetry frame, then re-anchor:
    #   SIM       -- from SimLoop.get_true_pose(), the sim-tier stand-in
    #                for a startup camera fix (matches
    #                test_pathplan_goto_convergence.py's own established
    #                idiom, 127-006).
    #   PLAYFIELD (geofence present) -- from a real camera fix.
    #   BENCH (hardware, no camera) -- left un-anchored: worldPose() then
    #                reports the raw encoder-integrated pose from boot
    #                (identity T_world_from_odom), which IS "encoder-judged
    #                closure" for a stand run with no camera.
    frames = []
    deadline = time.monotonic() + 5.0
    while not frames and time.monotonic() < deadline:
        frames = backend.advance(CYCLE_S)
    if not frames:
        # HardwareBackend already called enableTelemetry() (TLM:ON) at
        # connect time (see that class's __init__), so a real robot
        # answering at all should be emitting by now -- say so distinctly
        # from the sim message below, which points at a different failure
        # (2026-07-30: the original generic message here sent the
        # stakeholder hunting the wrong cause -- it read like the robot
        # was never asked, not that it was asked and stayed silent).
        if backend.label == "hardware":
            print("FAIL: no telemetry after tlmOn (robot may be off or "
                  "the link is down)")
        else:
            print("FAIL: no telemetry received -- cannot seed WorldPose")
        return 1
    for frame in frames:
        worldPose.ingest(frame)

    startTrue = None
    if args.sim:
        startTrue = backend.truePose()
        worldPose.reanchor((startTrue[0] / 10.0, startTrue[1] / 10.0, startTrue[2]))
    elif geofence is not None:
        fix = captureFixWithRetry(geofence, "start")
        if fix is None:
            print("FAIL: no camera fix at start -- cannot re-anchor WorldPose")
            return 1
        worldPose.reanchor(fix)

    startPose = worldPose.worldPose()
    if startPose is None:
        print("FAIL: WorldPose has no pose after seeding")
        return 1

    # (Pose, isVertex) pairs -- isVertex marks the LAST sample of each
    # corner's fillet, the one point per corner a camera fix is taken at
    # (see gotoSquareWaypoints()'s own docstring).
    waypoints = gotoSquareWaypoints(startPose, args.leg)
    targetPoses = [w for w, _isVertex in waypoints]
    limits = SolverLimits(trackWidth=PHYSICAL_TRACK, speed=CRUISE)
    # Budget covers the WHOLE path now, not one waypoint -- previously each
    # of the 4 corners got its own 400-iteration/30s budget under the old
    # per-corner gotoWorld() loop (summed: ~1600 iterations/120s across the
    # tour). Sized generously above that sum: the pass-through loop no
    # longer pays a stop-and-settle cost at every interior waypoint, so it
    # should finish well under this, but there is no reason to run the
    # budget tight now that it covers 12 waypoints instead of 1.
    giveUp = GiveUpLimits(maxIterations=1600, giveUpTimeout=90.0)
    allocator = MoveIdAllocator(start=_ID_BASE + 1)

    startCamera = captureFixWithRetry(geofence, "start") if geofence is not None else None
    fixes = [dict(label="start", pose=startPose, camera=startCamera)]

    # Labels are derived from the isVertex flags, not from index
    # arithmetic: gotoSquareWaypoints() now emits a leading start vertex
    # and each fillet's own inbound tangent point, so a waypoint's position
    # in the list no longer maps to `idx % CORNER_SEGMENTS_PER_CORNER`.
    cornerCount = 1
    hop = 0
    waypointLabels: "list[str]" = ["path start"]
    for _waypoint, isVertex in waypoints[1:]:
        if isVertex:
            waypointLabels.append(f"corner {cornerCount}")
            cornerCount += 1
            hop = 0
        else:
            hop += 1
            waypointLabels.append(f"corner {cornerCount} hop {hop}")

    def onWaypoint(index: int, pose) -> None:
        """`followPath()`'s own per-crossing hook -- records a chart mark
        (every crossing) and a camera fix (ONLY at the four original
        vertices, matching the OLD per-corner loop's own camera-fix
        policy -- a median-of-7 fix at every one of the
        ~4*CORNER_SEGMENTS_PER_CORNER dense samples would be absurdly slow
        and no intermediate sample is a boundary anything downstream
        cares about) the instant the pass-through advance rule crosses
        each waypoint.

        NOT at rest: `.claude/rules/playfield-testing.md` mandates a
        camera pose fix at every segment boundary "at REST, after the
        settle dwell" -- pass-through, by design, never stops at an
        INTERIOR waypoint (the terminal one does, via `followPath()`'s
        own arrival test, but this hook fires for it at the instant
        arrival is DETECTED, one solve cycle before the `finally`-block
        `estop()` actually lands). A camera fix taken here is therefore a
        MOVING fix on every geofence-armed run -- a known, deliberate gap
        left for whoever runs `--mode goto` on the playfield next (this
        OOP fix's own stated boundary was "do NOT run against hardware";
        reconciling moving-fix-vs-at-rest is 127-008's own territory)."""
        isVertex = waypoints[index][1]
        camera = (captureFixWithRetry(geofence, waypointLabels[index])
                  if geofence is not None and isVertex else None)
        fixes.append(dict(label=waypointLabels[index], pose=pose, camera=camera))

    pathResult = followPath(proto, worldPose, targetPoses, limits, geofence=geofence,
                            tolerance=tolerance, giveUp=giveUp, moveIds=allocator,
                            onWaypoint=onWaypoint)

    print(f"[{backend.label}] goto-mode square (pure-pursuit, full "
          f"{CRUISE:.0f} mm/s): success={pathResult.success} "
          f"reason={pathResult.reason!r}")
    print(f"  waypointsReached={pathResult.waypointsReached}/{len(waypoints)} "
          f"iterations={pathResult.iterations} sent={pathResult.sent} "
          f"retries={pathResult.retries} forcedResends={pathResult.forcedResends} "
          f"unacked={pathResult.unacked}")

    divergence = worldPose.encoderOtosDivergence()
    if divergence is not None:
        print(f"  encoder-vs-OTOS divergence: distance={divergence.distance * 10.0:.1f}mm "
              f"heading={math.degrees(divergence.heading):+.2f}deg")

    # Success implies every waypoint was reached (arrival at the terminal
    # waypoint is only possible once every interior one has been passed --
    # see followPath()'s own docstring); the second clause is a redundant,
    # cheap sanity check on that invariant, not an independent condition.
    completedAll = pathResult.success and pathResult.waypointsReached == len(waypoints)
    finalPose = worldPose.worldPose()
    closureWorld = (math.hypot(finalPose.x - startPose.x, finalPose.y - startPose.y) * 10.0
                    if finalPose is not None else None)
    if closureWorld is not None:
        print(f"  WorldPose closure (start -> end, "
              f"{'plant truth' if args.sim else 'encoder' if geofence is None else 'camera-anchored'}): "
              f"{closureWorld:.1f} mm")

    if args.sim and startTrue is not None:
        endTrue = backend.truePose()
        trueClosure = math.hypot(endTrue[0] - startTrue[0], endTrue[1] - startTrue[1])
        print(f"  ground-truth closure (SimLoop.get_true_pose()): {trueClosure:.1f} mm")

    reportGotoBoundaryFixes(fixes)

    chartPath = args.chart or f"src/tests/bench/square_tour_goto_{backend.label}.png"
    # One entry, not one per corner -- followPath() is a single continuous
    # call now. writeGotoChart()'s summary branch only needs `.success`/
    # `.iterations`/`.sent`, which FollowPathResult already provides
    # directly; no `samples` key means the per-corner convergence trace
    # panel falls back to that summary (see writeGotoChart()'s own
    # docstring) -- a full-path ground-truth trace is not meaningful here
    # (a SQUARE's terminal waypoint sits right next to the start, so
    # "distance to target over time" would spike and fall with no useful
    # per-corner interpretation; the OLD per-corner `_TruePoseSampler` this
    # replaced could do it only because it re-anchored a fresh target for
    # every gotoWorld() call).
    writeGotoChart([dict(label="full path (pure pursuit)", result=pathResult)],
                   fixes, waypoints, closureWorld, chartPath, backend.label)

    if not completedAll:
        print("FAIL: goto-mode square tour did not reach every waypoint")
        return 1
    print("PASS: goto-mode square tour closed")
    return 0


def runActuationFloorMeasurement(backend: "_Backend", args) -> int:
    """--mode actuation-floor (127-007, Decision 4's feedback-loop
    obligation): PLAYFIELD ONLY. Issues a DESCENDING series of small
    Distance-stopped and Angle-stopped `move_twist()` moves, camera-fixing
    immediately before and after each (at rest), to find the smallest
    commanded distance/angle that still reliably produces close-to-
    commanded motion -- the real actuation floor `planner.
    TERMINATION_TOLERANCE` is provisional pending.

    NOT run by this ticket's own pass: the robot was on the STAND
    (wheels unloaded) for this pass, and an unloaded-wheel floor would be
    optimistically wrong -- see this ticket's Completion Notes. Refuses
    to run without a camera geofence (raises `SystemExit`) rather than
    silently substituting a bench measurement.

    Prints raw (commanded, measured) pairs for both distance and angle
    and stops WITHOUT auto-selecting a floor value or writing it into
    `planner.py` -- picking the actual floor from this raw data, and
    updating `TERMINATION_TOLERANCE` with it, is a judgment call for
    whoever reviews the data once it exists (this ticket's own acceptance
    wording: "raw data, not just the final number")."""
    geofence = getattr(backend, "geofence", None)
    if geofence is None:
        raise SystemExit(
            "actuation-floor measurement requires the playfield camera "
            "(geofence) -- refusing to run on the bench/stand, where "
            "unloaded wheels would understate the real floor (see this "
            "ticket's Completion Notes)")
    from robot_radio.field import captureFixWithRetry

    def awaitMoveAndSettle(timeout: float, label: str, moveId: int) -> None:
        """Same wait-for-completion pattern as `Tour._awaitMove` (see that
        method's docstring for the two distinct timeout causes this
        distinguishes) -- duplicated here rather than shared because this
        function reads `backend.advance()` directly (no `Tour`/log in this
        code path), which already returns its drained frames and was never
        subject to the `Tour.advance()` bug fixed above.

        Always dwells CAMERA_FIX_DWELL, not the shorter INTER_SEGMENT_DWELL:
        this whole mode refuses to run without a geofence (see the guard at
        the top of `runActuationFloorMeasurement`), so every call here is
        immediately followed by a `captureFixWithRetry()` -- there is no
        no-camera case to shortcut."""
        seen = False
        start = time.monotonic()
        budget = timeout / 1000.0 + 2.0
        deadline = start + budget
        while time.monotonic() < deadline:
            frames = backend.advance(0.1)
            if frames:
                if frames[-1].active:
                    seen = True
                elif seen:
                    backend.advance(CAMERA_FIX_DWELL)
                    return
        waited = time.monotonic() - start
        reason = ("started but never completed -- 'active' rose but never "
                  "dropped" if seen else
                  "never started -- 'active' never rose (command likely lost)")
        print(f"TIMEOUT (hey jackass, it timed out): {label} (move {moveId}) "
              f"{reason}; waited {waited:.1f}s of a {budget:.1f}s budget")
        backend.advance(CAMERA_FIX_DWELL)

    results = {"distance": [], "angle": []}

    print("measuring minimum reliable move DISTANCE...")
    for d in args.floor_distances:
        before = captureFixWithRetry(geofence, f"dist-{d:.0f}mm-before")
        timeout = max(1500.0, d / CRUISE * 1000.0 * 4.0 + 1000.0)
        moveId = int(time.time() * 1000) % 1_000_000_000
        backend.proto.move_twist(CRUISE, 0.0, 0.0, stop_distance=d, timeout=timeout,
                                 move_id=moveId)
        awaitMoveAndSettle(timeout, f"dist-{d:.0f}mm", moveId)
        after = captureFixWithRetry(geofence, f"dist-{d:.0f}mm-after")
        measured = (math.hypot(after[0] - before[0], after[1] - before[1]) * 10.0
                    if before is not None and after is not None else None)
        print(f"  commanded {d:.0f} mm -> measured "
              f"{'n/a (camera fix missing)' if measured is None else f'{measured:.1f} mm'}")
        results["distance"].append((d, measured))

    print("measuring minimum reliable TURN angle...")
    omega = 2.0  # [rad/s] matches segments()'s own TURN_SPEED regime, well above the crawl boundary
    for a in args.floor_angles:
        rad = math.radians(a)
        before = captureFixWithRetry(geofence, f"angle-{a:.0f}deg-before")
        timeout = max(1500.0, rad / omega * 1000.0 * 4.0 + 1000.0)
        moveId = int(time.time() * 1000) % 1_000_000_000
        backend.proto.move_twist(0.0, 0.0, omega, stop_angle=rad, timeout=timeout,
                                 move_id=moveId)
        awaitMoveAndSettle(timeout, f"angle-{a:.0f}deg", moveId)
        after = captureFixWithRetry(geofence, f"angle-{a:.0f}deg-after")
        measured = None
        if before is not None and after is not None:
            dyaw = ((after[2] - before[2] + math.pi) % (2.0 * math.pi)) - math.pi
            measured = math.degrees(dyaw)
        print(f"  commanded {a:.0f} deg -> measured "
              f"{'n/a (camera fix missing)' if measured is None else f'{measured:+.1f} deg'}")
        results["angle"].append((a, measured))

    print("\nraw data (commanded, measured):")
    print(f"  distance: {results['distance']}")
    print(f"  angle: {results['angle']}")
    print("\nNOT auto-selecting a floor value or writing planner.py's "
          "TERMINATION_TOLERANCE -- see this function's own docstring.")
    return 0


def _floatList(text: str) -> "list[float]":
    """argparse type: 'a,b,c' -> [a, b, c] -- used by --floor-distances/
    --floor-angles (--mode actuation-floor)."""
    return [float(v) for v in text.split(",") if v.strip()]


def main() -> int:
    global LEG
    p = argparse.ArgumentParser(description=__doc__)
    backend_group = p.add_mutually_exclusive_group(required=True)
    backend_group.add_argument("--sim", action="store_true",
                               help="run against SimLoop (no hardware needed)")
    backend_group.add_argument("--port", nargs="?", const=DEFAULT_PORT,
                               help="run against a real robot on this serial port")
    p.add_argument("--robot-json", default=str(DEFAULT_ROBOT_JSON),
                   help="robot config the sim backend configures from")
    p.add_argument("--leg", type=float, default=LEG,
                   help="[mm] length of each side of the square (default "
                        f"{LEG:.0f}). The playfield is 134.3x89.3 cm -- 500mm "
                        "leaves under 10cm of margin per side, 400mm ~24cm.")
    p.add_argument("--max-closure", type=float, default=DEFAULT_MAX_CLOSURE,
                   help="[mm] fail if the tour's end point is farther than this from its start")
    p.add_argument("--heading-tolerance", type=float, default=DEFAULT_HEADING_TOLERANCE,
                   help="[deg] fail if total heading change is farther than this from 360")
    p.add_argument("--no-geofence", action="store_true",
                   help="DISABLE the camera geofence. Do not use on the playfield.")
    p.add_argument("--geofence-margin", type=float, default=12.0,
                   help="[cm] halt this close to the field edge (default 10)")
    p.add_argument("--leg-mode", choices=("move", "wheels"), default="move",
                   help="legs as DISTANCE-stopped MOVEs (default, closed-loop) or "
                        "as timed WHEELS runs (the original; measured 8cm of "
                        "cross-track drift over a 48cm leg)")
    p.add_argument("--turn-mode", choices=("move", "wheels"), default="move",
                   help="corners as ANGLE-stopped MOVEs (default, closed-loop) or "
                        "as timed WHEELS pivots (the original; needs the moving "
                        "prelude's duration scale to be accurate)")
    p.add_argument("--moving-prelude", action="store_true",
                   help="use the ORIGINAL prelude, which drives ~39cm and pivots "
                        "to self-calibrate. Bench only -- it does not fit on the "
                        "playfield and skews the tour's start heading.")
    p.add_argument("--sequential-segments", action="store_true",
                   help="disable segment chaining (out-of-process, 2026-07-30) "
                        "and use the original send-one -> await-completion -> "
                        "dwell -> send-next loop, for A/B comparison against "
                        "the chained default. Chaining only ever applies when "
                        "--leg-mode/--turn-mode are both 'move' (the default) "
                        "and no camera geofence is armed -- a geofence-armed "
                        "tour is ALWAYS sequential regardless of this flag, "
                        "since a camera fix needs the chassis at rest; see "
                        "Tour._canChain().")
    p.add_argument("--chart", default=None,
                   help="output PNG path (default: src/tests/bench/square_tour_<backend>.png)")
    p.add_argument("--mode", choices=("segments", "goto", "actuation-floor"),
                   default="segments",
                   help="'segments' (default): the original 8-segment WHEELS/MOVE "
                        "tour, unchanged. 'goto': drive the same square as one "
                        "continuous pure-pursuit run through a dense rounded-"
                        "corner waypoint sequence at full CRUISE speed "
                        "(pathplan.planner.followPath(), 127-007/127-008). "
                        "'actuation-floor': PLAYFIELD-ONLY minimum-reliable-move "
                        "measurement (127-007); refuses to run without the camera "
                        "geofence.")
    p.add_argument("--goto-tolerance", type=float, default=None,
                   help="[mm] followPath() arrival tolerance for --mode goto's "
                        "TERMINAL waypoint only -- interior waypoints are "
                        "pass-through, never tolerance-tested (default: "
                        "GOTO_PATH_TOLERANCE)")
    p.add_argument("--floor-distances", type=_floatList,
                   default=_floatList("150,100,80,60,50,40,30,20,15,10,5"),
                   help="[mm] comma-separated, descending: commanded distances "
                        "for --mode actuation-floor's distance sweep")
    p.add_argument("--floor-angles", type=_floatList,
                   default=_floatList("45,30,20,15,10,8,6,4,2"),
                   help="[deg] comma-separated, descending: commanded turn "
                        "angles for --mode actuation-floor's angle sweep")
    args = p.parse_args()

    LEG = args.leg

    backend: _Backend = (
        SimBackend(args.robot_json, realTime=(args.mode == "goto")) if args.sim
        else HardwareBackend(args.port, args.robot_json))
    if not args.sim and not args.no_geofence:
        checkPlayfieldLights()
        backend.geofence = Geofence(backend.proto, margin=args.geofence_margin)
        print(f"geofence ARMED: stops the robot within "
              f"{args.geofence_margin:.0f} cm of the field edge, and on tag loss")

    if args.mode == "goto":
        try:
            return runGotoTour(backend, args)
        except GeofenceViolation as exc:
            print(f"ABORTED: {exc}")
            print("motors stopped by the geofence")
            return 2
        finally:
            if getattr(backend, "geofence", None) is not None:
                backend.geofence.close()
            backend.close()

    if args.mode == "actuation-floor":
        try:
            return runActuationFloorMeasurement(backend, args)
        except GeofenceViolation as exc:
            print(f"ABORTED: {exc}")
            print("motors stopped by the geofence")
            return 2
        finally:
            if getattr(backend, "geofence", None) is not None:
                backend.geofence.close()
            backend.close()

    tour = Tour(backend)
    tour.movingPrelude = args.moving_prelude
    tour.turnMode = args.turn_mode
    tour.legMode = args.leg_mode
    tour.chainSegments = not args.sequential_segments
    try:
        completed = tour.run()
    except GeofenceViolation as exc:
        print(f"ABORTED: {exc}")
        print("motors stopped by the geofence")
        return 2
    finally:
        if getattr(backend, "geofence", None) is not None:
            backend.geofence.close()
        backend.close()

    log = tour.log
    if len(log["t"]) < 10:
        print("FAIL: almost no telemetry captured -- the robot never reported anything")
        return 1

    start = getattr(tour, "_tourStartIndex", 0)
    xs, ys = integrateEncoderPath(log, start)
    dTotL = log["posL"][-1] - log["posL"][start]
    dTotR = log["posR"][-1] - log["posR"][start]
    path = 0.5 * (dTotL + dTotR)
    # PHYSICAL_TRACK: interpreting encoders, not generating a command -- see
    # the module-level PHYSICAL_TRACK/EFFECTIVE_TRACK comment. MEASURED
    # 2026-07-29: using EFFECTIVE_TRACK here double-corrected for scrub and
    # reported heading 322.8 deg / closure 226 mm for a run camera truth
    # put at 368.9 deg / 71.7 mm.
    headingDeg = math.degrees((dTotR - dTotL) / PHYSICAL_TRACK)
    closure = math.hypot(xs[-1], ys[-1])

    # Plant truth wins where it exists: on the sim the encoder integration
    # is a derived quantity, but get_true_pose() is the actual answer.
    truthClosure = None
    if log["trueX"]:
        n = min(len(log["trueX"]), len(log["t"]))
        s = min(start, n - 1)
        truthClosure = math.hypot(log["trueX"][-1] - log["trueX"][s],
                                  log["trueY"][-1] - log["trueY"][s])
        xs = [x - log["trueX"][s] for x in log["trueX"][s:]]
        ys = [y - log["trueY"][s] for y in log["trueY"][s:]]
        closure = truthClosure

    print(f"[{backend.label}] path {path:.1f} mm (target {4 * LEG:.0f}), "
          f"heading {headingDeg:.1f} deg (target 360), closure {closure:.1f} mm"
          + (" (plant truth)" if truthClosure is not None else " (encoder odometry)"))

    reportSegmentFixes(tour, log)

    chartPath = args.chart or f"src/tests/bench/square_tour_{backend.label}.png"
    writeChart(log, tour.marks, xs, ys, path, headingDeg, closure, chartPath,
               backend.label)

    failures = []
    if not completed:
        failures.append("the tour did not complete (a segment was never acked)")
    if closure > args.max_closure:
        failures.append(f"closure {closure:.1f} mm exceeds the {args.max_closure:.0f} mm bound")
    if abs(abs(headingDeg) - 360.0) > args.heading_tolerance:
        failures.append(f"heading {headingDeg:.1f} deg is more than "
                        f"{args.heading_tolerance:.0f} deg from 360")
    if failures:
        for f in failures:
            print(f"FAIL: {f}")
        return 1
    print("PASS: square tour closed")
    return 0


def writeChart(log, marks, xs, ys, path, headingDeg, closure, out, label) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (axPath, axVel) = plt.subplots(
        1, 2, figsize=(16, 6.5), gridspec_kw={"width_ratios": [1, 1.6]})

    axPath.plot(xs, ys, color="#1f77b4", lw=1.6)
    axPath.plot([0], [0], marker="o", color="#2ca02c", ms=8, label="start")
    axPath.plot([xs[-1]], [ys[-1]], marker="x", color="#d62728", ms=10,
                mew=2.5, label=f"end (closure {closure:.0f} mm)")
    axPath.set_aspect("equal")
    axPath.set_xlabel("x [mm]")
    axPath.set_ylabel("y [mm]")
    axPath.set_title("Path (plant truth)" if log["trueX"] else "Path (encoder odometry)")
    axPath.legend(fontsize=9, loc="upper left")
    axPath.grid(True, alpha=0.3)

    axVel.plot(log["t"], log["cmdL"], color="#1f77b4", lw=1.0, ls="--",
               alpha=0.7, label="left commanded [mm/s]")
    axVel.plot(log["t"], log["cmdR"], color="#2ca02c", lw=1.0, ls="--",
               alpha=0.7, label="right commanded [mm/s]")
    axVel.plot(log["t"], log["velL"], color="#1f77b4", lw=1.4,
               label="left measured [mm/s]")
    axVel.plot(log["t"], log["velR"], color="#2ca02c", lw=1.4,
               label="right measured [mm/s]")
    for tm, mark in marks:
        axVel.axvline(tm, color="#bbbbbb", lw=0.6)
        axVel.annotate(mark, xy=(tm, 1.0), xycoords=("data", "axes fraction"),
                       fontsize=7.5, rotation=90, va="top", ha="right",
                       color="#888888")
    axVel.axhline(0, color="black", lw=0.5)
    axVel.set_xlabel("time [s]")
    axVel.set_ylabel("wheel speed [mm/s]")
    axVel.set_title("Wheel speeds -- WHEELS verb, open loop")
    axVel.legend(fontsize=8, loc="lower right")
    axVel.grid(True, alpha=0.25)

    fig.suptitle(
        f"Square tour ({label}) -- path {path:.0f}/{4 * LEG:.0f} mm, "
        f"heading {headingDeg:.1f}/360 deg, closure {closure:.0f} mm",
        fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(out, dpi=130)
    print(f"wrote {out}")


if __name__ == "__main__":
    sys.exit(main())
