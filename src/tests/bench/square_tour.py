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
    slip = getattr(robot_config.calibration, "rotational_slip", None) or 1.0
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
SEGMENT_REST = 0.9     # [s] rest after each segment, so the next starts from rest
CYCLE_S = 0.04         # [s] one SimLoop.step() -- App::RobotLoop::kCycle

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

    def close(self) -> None:
        raise NotImplementedError


class SimBackend(_Backend):
    label = "sim"

    def __init__(self, robot_json: "str | pathlib.Path") -> None:
        from robot_radio.config.robot_config import load_robot_config
        from robot_radio.io.sim_config import SimConfigConn
        from robot_radio.io.sim_loop import SimLoop
        from robot_radio.robot.protocol import NezhaProtocol

        robot_config = load_robot_config(robot_json)
        _set_effective_track(robot_config)
        track = robot_config.trackwidth if robot_config.trackwidth is not None else TRACK
        self._sim = SimLoop(track_width=track)
        # Deterministic manual stepping, no tick thread: this script is the
        # single consumer of the sim's own telemetry queue, and real-time
        # thread scheduling would only add jitter to a measurement that
        # does not need wall-clock realism at all.
        self._sim.connect(start_tick_thread=False)
        self._sim.configure_from_robot(robot_config)
        self.proto = NezhaProtocol(SimConfigConn(self._sim))

    def advance(self, seconds: float) -> "list":
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

    geofence = None

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
        self._seenAcks: "set[int]" = set()
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

    def record(self, frames: "list") -> None:
        truth = self.backend.truePose()
        for f in frames:
            for ack in f.acks:
                self._seenAcks.add(ack.corr_id)
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

    def advance(self, seconds: float) -> None:
        """Advance robot time in short slices, sampling ground truth once
        per slice so the sim's truth trace has the same resolution as the
        telemetry it rides alongside."""
        remaining = seconds
        while remaining > 1e-6:
            slice_s = min(CYCLE_S, remaining)
            self.record(self.backend.advance(slice_s))
            self.elapsed += slice_s
            remaining -= slice_s

    def sendVerified(self, vL: float, vR: float, durationMs: float, moveId: int) -> bool:
        """Send one WHEELS segment and confirm the firmware acked it,
        retrying up to 4 times (the DAPLink inbound-loss workaround the
        tagged tour already needed -- ~20% of inbound command packets are
        dropped by the USB->UART bridge before they reach the nRF).

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

    def runLegMove(self, label: str, moveId: int) -> bool:
        """A 500 mm leg as a DISTANCE-stopped MOVE, not a timed WHEELS run.

        Measured: a timed WHEELS leg curved 8 cm north over 48 cm east (the
        geofence aborted on it), because equal commanded wheel speeds do not
        produce equal actual speeds and nothing closes the loop. The same
        distance driven as a MOVE held ~1 cm of cross-track over 40 cm
        earlier today, and 99-100% of commanded distance over five legs.
        """
        import math as _m
        self.marks.append((self.elapsed, label))
        timeout = LEG / CRUISE * 1000.0 * 3.0 + 3000.0
        self.backend.proto.move_twist(CRUISE, 0.0, 0.0, stop_distance=LEG,
                                      timeout=timeout, move_id=moveId)
        self._awaitMove(timeout)
        return True

    def _awaitMove(self, timeout: float) -> None:
        seen = False
        deadline = time.monotonic() + timeout / 1000.0 + 2.0
        while time.monotonic() < deadline:
            frames = self.advance(0.1)
            if frames:
                if frames[-1].active:
                    seen = True
                elif seen:
                    break
        self.advance(SEGMENT_REST)

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
        import math as _m
        self.marks.append((self.elapsed, label))
        # vLeft<0 / vRight>0 in the WHEELS form is positive omega; keep the
        # same rotation sense so the square runs the same way round.
        omega = 2.0                     # [rad/s]
        rad = _m.pi / 2.0
        timeout = rad / omega * 1000.0 * 3.0 + 3000.0
        self.backend.proto.move_twist(0.0, 0.0, omega, stop_angle=rad,
                                      timeout=timeout, move_id=moveId)
        self._awaitMove(timeout)
        return True

    def runSegment(self, label: str, vL: float, vR: float, durationMs: float,
                   moveId: int) -> bool:
        self.marks.append((self.elapsed, label))
        if not self.sendVerified(vL, vR, durationMs, moveId):
            return False
        # The ack round trip already consumed part of the window; drive the
        # rest of it, then rest so the next segment starts from rest (the
        # condition the calibration was measured under).
        self.advance(durationMs / 1000.0)
        self.cmdL = self.cmdR = 0.0
        self.advance(SEGMENT_REST)
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
            # Each run*() above already ends with a settle dwell (SEGMENT_REST
            # or _awaitMove's own advance(SEGMENT_REST)) -- REST, per the
            # stakeholder mandate's own requirement.
            self.captureFix(label)
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
    p.add_argument("--chart", default=None,
                   help="output PNG path (default: src/tests/bench/square_tour_<backend>.png)")
    args = p.parse_args()

    LEG = args.leg

    backend: _Backend = (SimBackend(args.robot_json) if args.sim
                         else HardwareBackend(args.port, args.robot_json))
    tour = Tour(backend)
    if not args.sim and not args.no_geofence:
        checkPlayfieldLights()
        backend.geofence = Geofence(backend.proto, margin=args.geofence_margin)
        print(f"geofence ARMED: stops the robot within "
              f"{args.geofence_margin:.0f} cm of the field edge, and on tag loss")
    tour.movingPrelude = args.moving_prelude
    tour.turnMode = args.turn_mode
    tour.legMode = args.leg_mode
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
