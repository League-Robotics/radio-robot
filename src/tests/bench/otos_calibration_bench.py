#!/usr/bin/env python3
"""src/tests/bench/otos_calibration_bench.py -- sprint 126 (OTOS Telemetry
Bring-Up and Camera Calibration): the ONE shared bench/playfield script this
whole sprint grows one ``--mode`` at a time (sprint.md's Architecture Design
Rationale, "one script with growing modes, not one script per ticket").

Every mode reuses the SAME preflight (``square_tour.py``'s
``checkPlayfieldLights()``/``Geofence`` -- lights check, camera bring-up,
per-segment camera-fix, and the hard geofence that halts the robot near the
field edge or on tag loss) rather than forking new copies of that logic.

Modes so far:

  --mode units       (126-001) Confirm OTOS presence/liveness and settle the
                      reported pose tuple's UNITS (mm vs cm) from one
                      measured straight-line move against camera truth.
  --mode lever-arm    (126-002) Confirm whether the reported pose is the
                      robot CENTRE or the raw CHIP, and whether the
                      configured ``odometry_offset_mm`` lever arm is correct
                      in magnitude AND sign, from one measured in-place
                      rotation against camera truth.
  --mode distance     (126-003) Fit ``calibration.otos_linear_scale`` against
                      camera truth from many straight-line runs of differing
                      lengths, both directions -- fitted scale plus residual
                      mean/spread, not just a single-point check.
  --mode heading      (126-004) Fit ``calibration.otos_angular_scale`` against
                      camera truth from many in-place turns of differing
                      magnitudes, both directions, tether-safe throughout --
                      fitted scale plus residual mean/spread.

All modes are hardware/playfield-only (no ``--sim``) -- there is nothing to
unit-test about a camera-vs-sensor comparison (see the sprint's Test
Strategy). Every connection path calls ``estop()`` (never ``stop()``) in a
``finally`` block, per ``.claude/rules/playfield-testing.md``.

``--mode distance``/``--mode heading`` only MEASURE and REPORT a fitted
scale -- ``otos_linear_scale``/``otos_angular_scale`` are boot-baked
(``gen_boot_config.py`` -> ``OtosBootConfig``), not runtime-settable, so
correcting one is a separate step: edit ``data/robots/tovez.json``, reflash
(``mbdeploy deploy --build``), then re-run a subset (``--lengths``/
``--turn-angles``/``--reps``) to spot-check the correction.

Usage:
    uv run python src/tests/bench/otos_calibration_bench.py --port /dev/cu.usbmodem21141112 --mode units
    uv run python src/tests/bench/otos_calibration_bench.py --port /dev/cu.usbmodem21141112 --mode lever-arm
    uv run python src/tests/bench/otos_calibration_bench.py --port /dev/cu.usbmodem21141112 --mode distance
    uv run python src/tests/bench/otos_calibration_bench.py --port /dev/cu.usbmodem21141112 --mode heading
"""
from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys
import time

_THIS_DIR = pathlib.Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))

# Reuse, not fork (sprint.md's own Architecture Design Rationale, and both
# tickets' own instruction): lights preflight, the hard camera geofence, and
# its per-segment camera-fix helper.
from square_tour import Geofence, GeofenceViolation, checkPlayfieldLights  # noqa: E402

from robot_radio.io.serial_conn import SerialConnection  # noqa: E402
from robot_radio.robot.protocol import NezhaProtocol, TLMFrame  # noqa: E402

_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]

DEFAULT_PORT = "/dev/cu.usbmodem21141112"  # tovez, NEZHA2 direct USB (mbdeploy list)
CONNECT_SETTLE = 1.0           # [s] let the link settle before trusting frames
SEGMENT_REST = 0.9             # [s] settle dwell after a move, before a camera fix
                                # (mirrors square_tour.SEGMENT_REST)
ACK_WAIT = 0.6                 # [s] bound for scanning drained frames for a matching ack

CRUISE = 150.0                  # [mm/s] straight-leg speed, --mode units
LEG = 300.0                     # [mm] default straight-leg distance, --mode units

TURN_OMEGA_MAG = 2.0            # [rad/s] --mode lever-arm's rotation speed
                                 # magnitude. The SIGN is chosen at run time
                                 # from the measured starting heading (see
                                 # pickSafeTurnDirection()), not fixed here --
                                 # a fixed direction is only tether-safe when
                                 # starting near east; this script may be
                                 # invoked from any heading. NEGATIVE
                                 # commanded omega INCREASES camera yaw
                                 # (playfield-testing.md: "positive commanded
                                 # omega DECREASES camera yaw").
TURN_ANGLE_DEG = 90.0            # [deg] --mode lever-arm's rotation magnitude

FIELD_SAFE_MARGIN_CM = 15.0      # [cm] stay at least this far from the field
                                  # edge (the project floor is 12 cm; this
                                  # script uses a slightly larger margin since
                                  # neither move here is coast-calibrated).
SOUTH_HEADING_MARGIN_DEG = 20.0  # [deg] minimum clearance from south
                                  # (-90/270 deg, the tether direction) the
                                  # chosen turn must keep throughout its
                                  # sweep -- a turn that technically avoids
                                  # crossing south but ends or passes within
                                  # a few degrees of it is not what "never
                                  # through south" means in practice.

REPS_PER_COMBO = 3               # default repeats per length/direction (or
                                  # magnitude/direction) combination,
                                  # --mode distance / --mode heading
DISTANCE_LEGS_MM = (150.0, 300.0, 450.0)  # [mm] short/medium/long spread,
                                            # --mode distance -- all well
                                            # inside the field limits even
                                            # doubled by the ping-pong
                                            # forward/backward pattern
TURN_ANGLES_DEG = (15.0, 45.0, 90.0, 135.0)  # [deg] --mode heading spread,
                                               # enough to catch a magnitude-
                                               # dependent error


# ---------------------------------------------------------------------------
# Shared connection / telemetry plumbing -- used by both modes.
# ---------------------------------------------------------------------------


def connectRobot(port: str) -> "tuple[SerialConnection, NezhaProtocol]":
    """Open the robot's own serial link (mode=None auto-detects direct USB
    vs. the radio relay) and hand back a ready-to-drive protocol. Discards
    whatever telemetry was already queued so callers only see fresh
    pushes."""
    conn = SerialConnection(port=port)
    info = conn.connect()
    if info.get("status") != "connected":
        raise SystemExit(f"ERROR: connect failed: {info}")
    proto = NezhaProtocol(conn)
    print(f"connected: port={port} mode={info.get('mode')}")
    # 125-006: a PARKED robot under the default kAuto telemetry-emit policy
    # is correctly silent unless streaming-always is requested (same fix
    # tlm_log.py/twist_drive.py already need) -- both modes here read
    # telemetry continuously (at-rest jitter, a move-completion watch, and
    # (lever-arm) a trajectory spanning a whole turn), so switch to
    # streaming-always for the session; tlmOff() undoes it in each mode's
    # own finally block.
    proto.tlmOn()
    time.sleep(CONNECT_SETTLE)
    proto.read_pending_binary_tlm_frames()
    return conn, proto


def drainFrames(proto: "NezhaProtocol", seconds: float,
                 geofence: "Geofence | None" = None) -> "list[TLMFrame]":
    """Drain telemetry for `seconds` of wall time, checking the geofence on
    the same timebase the robot is moving on (never between segments --
    `.claude/rules/playfield-testing.md`'s "never drive blind"). This is the
    ONLY telemetry consumer both modes use -- ack matching and move-
    completion polling scan these same frames rather than draining a second
    time (square_tour.py's sendVerified() docstring: a single-consumer
    telemetry queue starves whichever of two readers goes second)."""
    frames: "list[TLMFrame]" = []
    deadline = time.monotonic() + seconds
    nextCheck = 0.0
    while time.monotonic() < deadline:
        frames.extend(proto.read_pending_binary_tlm_frames())
        now = time.monotonic()
        if geofence is not None and now >= nextCheck:
            geofence.check()  # raises GeofenceViolation; estop() already sent internally
            nextCheck = now + 0.1
        time.sleep(0.01)
    return frames


def awaitAck(proto: "NezhaProtocol", geofence: "Geofence | None", corrId: int,
             timeout: float = ACK_WAIT):  # [s]
    """Scan drained frames (via drainFrames(), never a second wait_for_ack()
    drain -- see drainFrames()'s own docstring) for an ack matching corrId.
    Returns the AckEntry, or None if the deadline passes with no match."""
    waited = 0.0
    step = 0.02  # [s]
    while waited < timeout:
        for frame in drainFrames(proto, step, geofence):
            for ack in frame.acks:
                if ack.corr_id == corrId:
                    return ack
        waited += step
    return None


def awaitMoveCompletion(proto: "NezhaProtocol", geofence: "Geofence | None",
                        timeoutMs: float) -> None:  # [ms]
    """Block until a Move's `active` flag has risen then fallen (mirrors
    square_tour.py's Tour._awaitMove) -- the Move is genuinely done, not
    just acked."""
    seen = False
    deadline = time.monotonic() + timeoutMs / 1000.0 + 2.0
    while time.monotonic() < deadline:
        frames = drainFrames(proto, 0.1, geofence)
        if frames:
            if frames[-1].active:
                seen = True
            elif seen:
                break


def connectAndArm(args: argparse.Namespace) -> "tuple[SerialConnection, NezhaProtocol, Geofence]":
    """The shared preflight both modes use, unchanged (ticket 002's own
    instruction: reuse ticket 001's helper, do not duplicate it): playfield
    lights check, connect, arm the hard camera geofence."""
    checkPlayfieldLights()
    conn, proto = connectRobot(args.port)
    geofence = Geofence(proto, margin=args.geofence_margin)
    print(f"geofence ARMED: stops the robot within {args.geofence_margin:.0f} cm "
          f"of the field edge, and on tag loss")
    return conn, proto, geofence


def checkLiveness(proto: "NezhaProtocol", geofence: "Geofence | None",
                   seconds: float = 1.0) -> "tuple[bool, list[TLMFrame]]":  # [s]
    """Confirm the robot is genuinely at READY: telemetry is flowing and,
    over every frame in the sample window, otos_present (flags bit 0) /
    conn_left / conn_right all read true. Prints the result (issue
    acceptance criterion 1).

    There is no live boot-time READY event to catch here -- comms_.
    sendReady() (robot_loop.cpp) fires once at boot, and this script
    connects to an ALREADY-booted robot (opening the serial port does not
    reset it -- `.clasi/knowledge/serial-monitor-never-shows-the-banner.md`).
    conn_left=1/conn_right=1/otos_present=1 held steady over a live sample
    IS the practical evidence of READY available to a script that attaches
    after the fact.
    """
    frames = drainFrames(proto, seconds, geofence)
    tlm = [f for f in frames if f.flags is not None]
    if not tlm:
        print("FAIL: no telemetry frames received at all -- is the robot connected "
              "and past boot? See .clasi/knowledge/silent-robot-dead-external-i2c-bus.md")
        return False, []
    allOtos = all(f.otos_present for f in tlm)
    allConnLeft = all(f.conn_left for f in tlm)
    allConnRight = all(f.conn_right for f in tlm)
    print(f"liveness over {len(tlm)} frames: otos_present={allOtos} "
          f"conn_left={allConnLeft} conn_right={allConnRight}")
    ok = allOtos and allConnLeft and allConnRight
    if ok:
        print("PASS: STATUS otos=1, connL=1, connR=1 -- telemetry flags bit 0 set "
              "on a READY robot (issue acceptance criterion 1)")
    else:
        print("FAIL: robot is not fully READY (otos=0 or connL/connR=0 observed) -- "
              "check for a wedged external I2C bus "
              "(.clasi/knowledge/silent-robot-dead-external-i2c-bus.md) BEFORE "
              "concluding the OTOS chip itself is missing; it shares that bus with "
              "the Nezha motor controllers.")
    return ok, tlm


def _resolveRobotJsonPath(robotJsonOverride: "str | None") -> pathlib.Path:
    """Resolve which robot JSON file this script's config reads (and, for
    --mode distance/--mode heading, calibration edits) apply to: an
    explicit --robot-json override, else the active robot pointer
    (data/robots/active_robot.json), else tovez.json. Shared by
    loadConfiguredOffsetMm() and loadCommittedScale() -- one resolution,
    not two copies of it."""
    if robotJsonOverride is not None:
        return pathlib.Path(robotJsonOverride)
    pointer = _REPO_ROOT / "data" / "robots" / "active_robot.json"
    if pointer.exists():
        spec = json.loads(pointer.read_text())
        if "path" in spec:
            return _REPO_ROOT / spec["path"]
    return _REPO_ROOT / "data" / "robots" / "tovez.json"


def loadConfiguredOffsetMm(robotJsonOverride: "str | None"
                           ) -> "tuple[float, float, str]":  # [mm] [mm]
    """The configured odometry_offset_mm lever arm (x, y) -- read from the
    ACTIVE robot config (data/robots/active_robot.json's pointer), the same
    resolution square_tour.py's HardwareBackend uses, unless overridden.
    Returns (x, y, path-used) so the caller can print provenance."""
    from robot_radio.config.robot_config import load_robot_config

    path = _resolveRobotJsonPath(robotJsonOverride)
    cfg = load_robot_config(path)
    offset = cfg.geometry.odometry_offset_mm
    return offset.x, offset.y, str(path)


def loadCommittedScale(path: pathlib.Path, key: str) -> float:
    """Read calibration.<key> (``otos_linear_scale``/``otos_angular_scale``)
    straight from the robot JSON, not through the pydantic RobotConfig --
    these are boot-baked config-file values consumed by
    gen_boot_config.py's OtosBootConfig, not fields the loader models."""
    data = json.loads(path.read_text())
    return float(data["calibration"][key])


def fitScaleThroughOrigin(xs: "list[float]", ys: "list[float]") -> float:
    """Least-squares slope of y = k*x forced through the origin -- physically
    correct here (zero true distance/heading-change must read back as zero
    on both camera and OTOS, so an intercept term would only fit noise):
    k = sum(x*y) / sum(x*x)."""
    sumXY = sum(x * y for x, y in zip(xs, ys))
    sumXX = sum(x * x for x in xs)
    return sumXY / sumXX if sumXX > 1e-9 else float("nan")


def meanAndStdDev(values: "list[float]") -> "tuple[float, float]":
    """Sample mean and sample standard deviation (n-1 denominator; 0.0 for
    a single-element sample rather than a division by zero)."""
    n = len(values)
    mean = sum(values) / n
    if n < 2:
        return mean, 0.0
    variance = sum((v - mean) ** 2 for v in values) / (n - 1)
    return mean, math.sqrt(variance)


# ---------------------------------------------------------------------------
# Timeout safety clamp -- shared by every move this script commands.
#
# 2026-07-29 post-mortem: an earlier run ended with the robot hard against
# the WEST rail after tag 100 was lost mid-move. The fence fired estop()
# correctly on tag loss, but a fence that cannot see the robot cannot bound
# its travel -- the backstop that had to hold was the move's own `timeout`.
# The formula in use then (`dist/speed*3 + 3000`) allowed ~10.5s at
# 120 mm/s for a 300 mm move -- ~126cm of possible worst-case travel, far
# more than the field can absorb. clampTimeoutToClearance() bounds every
# move's timeout so that worst-case travel (timeout x commanded speed)
# cannot exceed the camera-confirmed clearance to the nearest field
# boundary in the direction of travel, computed from the pre-move camera
# fix -- not from the intended stop condition, which is exactly the thing
# that can fail to fire.
# ---------------------------------------------------------------------------


def clearanceAlongDirectionCm(x: float, y: float, dirX: float, dirY: float,
                              marginCm: float) -> float:  # [cm]
    """Distance from (x, y) to the margin-shrunk field boundary, travelling
    along the unit direction (dirX, dirY). `inf` for a degenerate
    (zero-length) direction -- caller only feeds this a real direction when
    there is translational speed to bound."""
    limX = Geofence.HALF_W - marginCm
    limY = Geofence.HALF_H - marginCm
    candidates: "list[float]" = []
    if dirX > 1e-9:
        candidates.append((limX - x) / dirX)
    elif dirX < -1e-9:
        candidates.append((-limX - x) / dirX)
    if dirY > 1e-9:
        candidates.append((limY - y) / dirY)
    elif dirY < -1e-9:
        candidates.append((-limY - y) / dirY)
    if not candidates:
        return float("inf")
    return max(0.0, min(candidates))


def clampTimeoutToClearance(requestedTimeoutMs: float,
                            camPose: "tuple[float, float, float]",
                            vx: float, vy: float,  # [mm/s] commanded body-frame
                            marginCm: float) -> "tuple[float, float]":  # [ms] [mm]
    """Clamp requestedTimeoutMs so that worst-case travel (timeout x
    commanded speed) cannot exceed the camera-confirmed clearance to the
    nearest field boundary in the direction of travel, from camPose (the
    PRE-MOVE camera fix). A pure rotation (vx=vy=0, e.g. --mode lever-arm's
    turn) has zero translational worst case -- the requested timeout passes
    through unchanged and clearance is reported as inf. Returns
    (clampedTimeoutMs, clearanceMm)."""
    speed = math.hypot(vx, vy)  # [mm/s]
    if speed <= 1e-6:
        return requestedTimeoutMs, float("inf")
    x, y, yaw = camPose
    worldDirX = math.cos(yaw) * vx - math.sin(yaw) * vy
    worldDirY = math.sin(yaw) * vx + math.cos(yaw) * vy
    worldDirX, worldDirY = worldDirX / speed, worldDirY / speed
    clearanceCm = clearanceAlongDirectionCm(x, y, worldDirX, worldDirY, marginCm)
    clearanceMm = clearanceCm * 10.0
    maxTimeoutMs = clearanceMm / speed * 1000.0
    return min(requestedTimeoutMs, maxTimeoutMs), clearanceMm


# ---------------------------------------------------------------------------
# --mode units (126-001)
# ---------------------------------------------------------------------------


def legDirectionSafe(camPose: "tuple[float, float, float]", legMm: float,
                     sign: float, marginCm: float) -> bool:  # [mm] [cm]
    """True if a legMm straight move in the given SIGNED direction (body-
    frame forward=+1 / backward=-1), commanded from camPose, keeps the
    robot at least marginCm from the field edge. Shared by
    pickSafeLegDirection() (picks whichever direction is safe) and
    --mode distance (2026-07-30, ticket 126-003 -- needs to validate a
    SPECIFIC planned direction, not just find any safe one, since it must
    cover both directions across the run)."""
    x, y, yaw = camPose
    dxCm = (legMm / 10.0) * math.cos(yaw) * sign
    dyCm = (legMm / 10.0) * math.sin(yaw) * sign
    nx, ny = x + dxCm, y + dyCm
    return (abs(nx) <= Geofence.HALF_W - marginCm
            and abs(ny) <= Geofence.HALF_H - marginCm)


def pickSafeLegDirection(camPose: "tuple[float, float, float]", legMm: float,
                         marginCm: float) -> "float | None":  # [mm] [cm]
    """Choose the sign of v_x (forward=+1 / backward=-1) for a legMm straight
    move that keeps the robot at least marginCm from the field edge, given
    its current camera-measured pose. Returns None if neither direction is
    safe (caller must not move)."""
    if legDirectionSafe(camPose, legMm, 1.0, marginCm):
        return 1.0
    if legDirectionSafe(camPose, legMm, -1.0, marginCm):
        return -1.0
    return None


def runUnitsMode(args: argparse.Namespace) -> int:
    print("=== otos_calibration_bench --mode units (ticket 126-001) ===")
    conn = proto = geofence = None
    try:
        conn, proto, geofence = connectAndArm(args)

        liveOk, _ = checkLiveness(proto, geofence, seconds=1.0)
        if not liveOk:
            return 1

        camBefore = geofence.captureFix("units: rest before move")
        if camBefore is None:
            print("FAIL: camera did not see tag 100 at rest before the move")
            return 1

        restFrames = drainFrames(proto, 1.0, geofence)
        otosRest = [f.otos_reading for f in restFrames if f.otos_reading is not None]
        if len(otosRest) < 3:
            print(f"FAIL: too few OTOS bursts at rest to establish a jitter bound "
                  f"(got {len(otosRest)})")
            return 1
        jitterX = max(o.x for o in otosRest) - min(o.x for o in otosRest)
        jitterY = max(o.y for o in otosRest) - min(o.y for o in otosRest)
        jitterH = max(o.heading for o in otosRest) - min(o.heading for o in otosRest)
        print(f"at-rest OTOS jitter over {len(otosRest)} bursts: "
              f"dx={jitterX:.2f} dy={jitterY:.2f} raw units, "
              f"dheading={math.degrees(jitterH):.3f} deg")
        otosBefore = otosRest[-1]

        sign = pickSafeLegDirection(camBefore, args.leg, args.geofence_margin)
        if sign is None:
            print(f"FAIL: no field-safe straight direction for a {args.leg:.0f} mm leg "
                  f"from current pose x={camBefore[0]:+.1f}cm y={camBefore[1]:+.1f}cm "
                  f"yaw={math.degrees(camBefore[2]):+.1f}deg")
            return 1
        vx = sign * args.cruise
        requestedTimeoutMs = args.leg / args.cruise * 1000.0 * 3.0 + 3000.0  # [ms]
        requiredMs = args.leg / args.cruise * 1000.0  # [ms] bare time to complete the leg
        timeoutMs, clearanceMm = clampTimeoutToClearance(
            requestedTimeoutMs, camBefore, vx, 0.0, args.geofence_margin)
        worstCaseMm = timeoutMs / 1000.0 * abs(vx)
        print(f"timeout safety clamp: requested={requestedTimeoutMs:.0f}ms "
              f"camera-confirmed clearance={clearanceMm:.0f}mm -> clamped={timeoutMs:.0f}ms "
              f"(worst-case travel {worstCaseMm:.0f}mm)")
        if timeoutMs < requiredMs:
            print(f"FAIL: clamped timeout {timeoutMs:.0f}ms is below the {requiredMs:.0f}ms "
                  f"needed to complete the {args.leg:.0f}mm leg at {args.cruise:.0f}mm/s -- "
                  "not enough camera-confirmed clearance from the current pose to run this "
                  "leg safely; reposition or shorten --leg rather than running it")
            return 1

        print(f"commanding straight move: v_x={vx:+.0f} mm/s stop_distance={args.leg:.0f} mm "
              f"timeout={timeoutMs:.0f} ms (direction={'forward' if sign > 0 else 'backward'}, "
              "chosen for field margin; straight moves never wrap the tether)")

        corrId = proto.move_twist(vx, 0.0, 0.0, stop_distance=args.leg,
                                  timeout=timeoutMs, replace=True)
        ack = awaitAck(proto, geofence, corrId)
        moveAcked = ack is not None and ack.ok
        print(f"  enqueue ack: {ack} ({'OK' if moveAcked else 'FAIL'})")

        awaitMoveCompletion(proto, geofence, timeoutMs)
        drainFrames(proto, SEGMENT_REST, geofence)

        camAfter = geofence.captureFix("units: rest after move")
        afterFrames = drainFrames(proto, 1.0, geofence)
        otosAfterList = [f.otos_reading for f in afterFrames if f.otos_reading is not None]
        otosAfter = otosAfterList[-1] if otosAfterList else otosBefore

        if camAfter is None:
            print("FAIL: camera did not see tag 100 at rest after the move")
            return 1

        camDeltaMm = math.hypot(camAfter[0] - camBefore[0], camAfter[1] - camBefore[1]) * 10.0
        otosDeltaRaw = math.hypot(otosAfter.x - otosBefore.x, otosAfter.y - otosBefore.y)
        mmRatio = otosDeltaRaw / camDeltaMm if camDeltaMm > 1e-6 else float("nan")
        cmRatio = (otosDeltaRaw * 10.0) / camDeltaMm if camDeltaMm > 1e-6 else float("nan")

        print(f"\ncamera-measured travel: {camDeltaMm:.1f} mm (commanded leg {args.leg:.0f} mm)")
        print(f"OTOS raw delta magnitude: {otosDeltaRaw:.2f} raw units "
              f"(x: {otosBefore.x:.1f}->{otosAfter.x:.1f}, y: {otosBefore.y:.1f}->{otosAfter.y:.1f})")
        print(f"  mm hypothesis (raw IS mm):        ratio raw/camera_mm       = {mmRatio:.3f}")
        print(f"  cm hypothesis (raw IS cm, x10=mm): ratio (raw*10)/camera_mm = {cmRatio:.3f}")

        mmErr = abs(mmRatio - 1.0)
        cmErr = abs(cmRatio - 1.0)
        unit: "str | None" = None
        if mmErr <= 0.25 and mmErr <= cmErr:
            unit = "mm"
        elif cmErr <= 0.25 and cmErr < mmErr:
            unit = "cm"
        if unit is not None:
            print(f"CONCLUSION: OTOS pose tuple units = {unit} "
                  f"(residual {min(mmErr, cmErr) * 100:.1f}% vs. camera-measured travel, "
                  "measured from this move, not asserted from source)")
        else:
            print("CONCLUSION: AMBIGUOUS -- neither the mm nor the cm hypothesis lands within "
                  "25% of the camera-measured travel; units NOT settled by this run "
                  "(see the ratios above for the actual numbers)")

        movedEnough = otosDeltaRaw > max(3.0 * max(jitterX, jitterY), 1.0)
        print(f"\nmotion-vs-rest: OTOS delta ({otosDeltaRaw:.2f}) "
              f"{'exceeds' if movedEnough else 'does NOT exceed'} 3x the at-rest jitter bound "
              f"({3.0 * max(jitterX, jitterY):.2f}) -- pose "
              f"{'DOES' if movedEnough else 'does NOT'} track motion")

        print("\n=== SUC-001 acceptance ===")
        print("  AC1 presence (otos=1, flags bit0, connL=1, connR=1): PASS")
        print(f"  AC2 liveness+units (tracks motion, holds at rest, units stated): "
              f"{'PASS' if (movedEnough and unit is not None) else 'FAIL'}")
        return 0 if (moveAcked and movedEnough and unit is not None) else 1
    finally:
        if proto is not None:
            try:
                proto.tlmOff()
            except Exception:
                pass
            try:
                proto.estop()
            except Exception:
                pass
        if geofence is not None:
            geofence.close()
        if conn is not None:
            conn.disconnect()


# ---------------------------------------------------------------------------
# --mode distance (126-003)
# ---------------------------------------------------------------------------


def runDistanceMode(args: argparse.Namespace) -> int:
    print("=== otos_calibration_bench --mode distance (ticket 126-003) ===")
    conn = proto = geofence = None
    rows: "list[dict]" = []
    try:
        conn, proto, geofence = connectAndArm(args)

        liveOk, _ = checkLiveness(proto, geofence, seconds=1.0)
        if not liveOk:
            return 1

        # Ping-pong pattern per length: forward, backward, forward, ... --
        # covers both directions of travel (ticket's own requirement) while
        # keeping the robot's position bounded near its start (each
        # backward leg undoes the preceding forward leg), so a long run of
        # many legs never drifts toward an edge the way a one-directional
        # walk would.
        for length in args.lengths:
            for i in range(2 * args.reps):
                plannedSign = 1.0 if i % 2 == 0 else -1.0
                label = f"len={length:.0f}mm {'fwd' if plannedSign > 0 else 'bwd'} rep{i // 2}"

                camBefore = geofence.captureFix(f"distance before: {label}")
                if camBefore is None:
                    print(f"  SKIP {label}: camera did not see tag 100 at rest")
                    continue
                restFrames = drainFrames(proto, 0.4, geofence)
                otosRestList = [f.otos_reading for f in restFrames if f.otos_reading is not None]
                if not otosRestList:
                    print(f"  SKIP {label}: no OTOS reading at rest before the leg")
                    continue
                otosBefore = otosRestList[-1]

                sign = plannedSign
                if not legDirectionSafe(camBefore, length, sign, args.geofence_margin):
                    if legDirectionSafe(camBefore, length, -sign, args.geofence_margin):
                        print(f"  {label}: planned direction unsafe from current pose -- "
                              "using the opposite direction instead")
                        sign = -sign
                    else:
                        print(f"  SKIP {label}: no field-safe direction for a {length:.0f}mm "
                              f"leg from x={camBefore[0]:+.1f}cm y={camBefore[1]:+.1f}cm "
                              f"yaw={math.degrees(camBefore[2]):+.1f}deg -- reposition")
                        continue

                vx = sign * args.cruise
                requestedTimeoutMs = length / args.cruise * 1000.0 * 3.0 + 3000.0  # [ms]
                requiredMs = length / args.cruise * 1000.0  # [ms]
                timeoutMs, clearanceMm = clampTimeoutToClearance(
                    requestedTimeoutMs, camBefore, vx, 0.0, args.geofence_margin)
                if timeoutMs < requiredMs:
                    print(f"  SKIP {label}: clamped timeout {timeoutMs:.0f}ms below the "
                          f"{requiredMs:.0f}ms needed (camera-confirmed clearance "
                          f"{clearanceMm:.0f}mm)")
                    continue

                print(f"commanding {label}: v_x={vx:+.0f} mm/s stop_distance={length:.0f} mm "
                      f"timeout={timeoutMs:.0f} ms")
                corrId = proto.move_twist(vx, 0.0, 0.0, stop_distance=length,
                                          timeout=timeoutMs, replace=True)
                ack = awaitAck(proto, geofence, corrId)
                if ack is None or not ack.ok:
                    print(f"  SKIP {label}: enqueue ack FAILED ({ack})")
                    continue

                awaitMoveCompletion(proto, geofence, timeoutMs)
                drainFrames(proto, SEGMENT_REST, geofence)

                camAfter = geofence.captureFix(f"distance after: {label}")
                afterFrames = drainFrames(proto, 0.4, geofence)
                otosAfterList = [f.otos_reading for f in afterFrames if f.otos_reading is not None]
                if camAfter is None or not otosAfterList:
                    print(f"  SKIP {label}: missing camera or OTOS reading after the leg")
                    continue
                otosAfter = otosAfterList[-1]

                camMm = math.hypot(camAfter[0] - camBefore[0], camAfter[1] - camBefore[1]) * 10.0
                otosMm = math.hypot(otosAfter.x - otosBefore.x, otosAfter.y - otosBefore.y)
                errMm = otosMm - camMm
                errPct = errMm / camMm * 100.0 if camMm > 1e-6 else float("nan")
                rows.append(dict(length=length, sign=sign, camMm=camMm, otosMm=otosMm,
                                  errMm=errMm, errPct=errPct))
                print(f"  {label}: camera={camMm:.1f}mm OTOS={otosMm:.1f}mm "
                      f"err={errMm:+.1f}mm ({errPct:+.1f}%)")

        return _reportDistanceFit(rows, args)
    finally:
        if proto is not None:
            try:
                proto.tlmOff()
            except Exception:
                pass
            try:
                proto.estop()
            except Exception:
                pass
        if geofence is not None:
            geofence.close()
        if conn is not None:
            conn.disconnect()


def _reportDistanceFit(rows: "list[dict]", args: argparse.Namespace) -> int:
    """Fit OTOS distance = k * camera distance (through the origin) across
    every valid run, derive the corrected otos_linear_scale, and print the
    per-run table plus fitted scale, residual mean+spread, and an explicit
    PASS/FAIL against the currently-committed value (ticket's own required
    report shape -- not just a mean, not just a pass/fail verdict)."""
    print(f"\n=== distance-scale fit ({len(rows)} valid runs) ===")
    if len(rows) < 6:
        print(f"FAIL: only {len(rows)} valid runs -- too few to fit a scale reliably "
              "(need at least a handful spanning lengths/directions)")
        return 1

    print(f"{'length_mm':>9} {'dir':>4} {'camera_mm':>10} {'otos_mm':>10} "
          f"{'err_mm':>8} {'err_pct':>8}")
    for r in rows:
        print(f"{r['length']:9.0f} {'fwd' if r['sign'] > 0 else 'bwd':>4} "
              f"{r['camMm']:10.1f} {r['otosMm']:10.1f} {r['errMm']:8.1f} {r['errPct']:8.2f}")

    camList = [r["camMm"] for r in rows]
    otosList = [r["otosMm"] for r in rows]
    slope = fitScaleThroughOrigin(camList, otosList)  # otos_mm ~= slope * camera_mm
    correctionRatio = (1.0 / slope) if slope and not math.isnan(slope) else float("nan")

    path = _resolveRobotJsonPath(args.robot_json)
    committed = loadCommittedScale(path, "otos_linear_scale")
    correctedScale = committed * correctionRatio

    meanErrMm, sdErrMm = meanAndStdDev([r["errMm"] for r in rows])
    meanErrPct, sdErrPct = meanAndStdDev([r["errPct"] for r in rows])
    n = len(rows)
    standardErrorPct = sdErrPct / math.sqrt(n)
    ciLow, ciHigh = meanErrPct - 1.96 * standardErrorPct, meanErrPct + 1.96 * standardErrorPct
    needsCorrection = not (ciLow <= 0.0 <= ciHigh)

    print(f"\nfitted OTOS/camera slope (through origin): {slope:.4f}")
    print(f"committed calibration.otos_linear_scale ({path.name}): {committed:.4f}")
    print(f"corrected otos_linear_scale = committed x (1/slope) = "
          f"{committed:.4f} x {correctionRatio:.4f} = {correctedScale:.4f}")
    print(f"residual: mean={meanErrMm:+.2f}mm sd={sdErrMm:.2f}mm "
          f"({meanErrPct:+.2f}% sd={sdErrPct:.2f}%, n={n})")
    print(f"95% CI of mean residual: [{ciLow:+.2f}%, {ciHigh:+.2f}%]")
    if needsCorrection:
        print(f"FAIL: committed otos_linear_scale={committed:.4f} disagrees with measurement "
              f"(95% CI of the mean residual excludes 0) -- correct to {correctedScale:.4f}")
    else:
        print(f"PASS: committed otos_linear_scale={committed:.4f} is within measured "
              "uncertainty (95% CI of the mean residual includes 0) -- no correction needed")
    return 0


# ---------------------------------------------------------------------------
# --mode lever-arm (126-002)
# ---------------------------------------------------------------------------


def sweepCrossesAngle(startDeg: float, deltaDeg: float, targetDeg: float) -> bool:  # [deg] [deg] [deg]
    """True if a continuous sweep from startDeg by the SIGNED deltaDeg passes
    through targetDeg (mod 360) at some point along the way. Used to enforce
    the tether rule: a single turn must never sweep through south (-90 deg)."""
    d = (targetDeg - startDeg) % 360.0
    if deltaDeg >= 0:
        return 0.0 < d <= deltaDeg
    d -= 360.0
    return deltaDeg <= d < 0.0


def closestApproachToSouthDeg(startDeg: float, deltaDeg: float) -> float:  # [deg] [deg] -> [deg]
    """Minimum angular distance from south (-90 / 270 deg, the tether
    direction) reached anywhere along a continuous, monotonic sweep from
    startDeg by the SIGNED deltaDeg (including both endpoints)."""
    steps = max(2, int(round(abs(deltaDeg))))
    closest = float("inf")
    for i in range(steps + 1):
        angle = startDeg + deltaDeg * (i / steps)
        d = (angle - (-90.0)) % 360.0
        d = min(d, 360.0 - d)
        closest = min(closest, d)
    return closest


def pickSafeTurnDirection(startDeg: float, angleDeg: float, marginDeg: float
                          ) -> "float | None":  # [deg] [deg] [deg] -> signed [deg] or None
    """Choose the signed turn delta (+angleDeg or -angleDeg) from startDeg
    that respects the tether rule: never sweep through south (-90/270 deg),
    and keep at least marginDeg of clearance from south throughout the
    sweep (a turn that technically avoids crossing south but skims within a
    few degrees of it is not tether-safe in practice -- a fixed
    east-starting direction is only safe when the robot actually starts
    near east; from other headings the safe direction can flip). Of the two
    candidate directions, prefers whichever keeps the larger margin.
    Returns None if neither direction clears marginDeg."""
    candidates: "list[tuple[float, float]]" = []
    for delta in (angleDeg, -angleDeg):
        if sweepCrossesAngle(startDeg, delta, -90.0):
            continue
        candidates.append((closestApproachToSouthDeg(startDeg, delta), delta))
    if not candidates:
        return None
    candidates.sort(key=lambda c: -c[0])
    bestMargin, bestDelta = candidates[0]
    if bestMargin < marginDeg:
        return None
    return bestDelta


def fitCircle(points: "list[tuple[float, float]]"
             ) -> "tuple[float, float, float, float] | None":
    """Kasa algebraic circle fit over [(x, y), ...] -- purely geometric, same
    units as the input points. Returns (centreX, centreY, radius,
    residualRms), or None if under-determined (fewer than 3 points, or a
    degenerate/collinear fit)."""
    import numpy as np

    pts = np.array(points, dtype=float)
    if len(pts) < 3:
        return None
    x, y = pts[:, 0], pts[:, 1]
    coeffs = np.column_stack([2.0 * x, 2.0 * y, np.ones_like(x)])
    rhs = x**2 + y**2
    solution, *_ = np.linalg.lstsq(coeffs, rhs, rcond=None)
    centreX, centreY, c = solution
    radiusSquared = c + centreX**2 + centreY**2
    if radiusSquared <= 0:
        return None
    radius = math.sqrt(radiusSquared)
    residual = float(np.sqrt(np.mean((np.hypot(x - centreX, y - centreY) - radius) ** 2)))
    return float(centreX), float(centreY), radius, residual


def runLeverArmMode(args: argparse.Namespace) -> int:
    print("=== otos_calibration_bench --mode lever-arm (ticket 126-002) ===")
    conn = proto = geofence = None
    try:
        conn, proto, geofence = connectAndArm(args)

        liveOk, _ = checkLiveness(proto, geofence, seconds=1.0)
        if not liveOk:
            return 1

        camBefore = geofence.captureFix("lever-arm: rest before turn")
        if camBefore is None:
            print("FAIL: camera did not see tag 100 at rest before the turn")
            return 1
        x0, y0, yaw0 = camBefore
        startYawDeg = math.degrees(yaw0)

        if abs(x0) > Geofence.HALF_W - args.geofence_margin or abs(y0) > Geofence.HALF_H - args.geofence_margin:
            print(f"FAIL: starting pose x={x0:+.1f}cm y={y0:+.1f}cm is already within "
                  f"{args.geofence_margin:.0f}cm of the field edge -- reposition before turning")
            return 1

        # Direction is chosen from the MEASURED starting heading, not fixed:
        # a fixed east->west-through-north direction is only tether-safe
        # when the robot actually starts near east. Whichever direction
        # keeps the larger clearance from south wins.
        deltaDeg = pickSafeTurnDirection(startYawDeg, TURN_ANGLE_DEG, SOUTH_HEADING_MARGIN_DEG)
        if deltaDeg is None:
            print(f"FAIL: no tether-safe {TURN_ANGLE_DEG:.0f} deg turn direction from current "
                  f"heading {startYawDeg:+.1f} deg keeps >= {SOUTH_HEADING_MARGIN_DEG:.0f} deg "
                  "clearance from south (-90/270 deg, tether direction) -- reposition before "
                  "turning")
            return 1
        omega = -TURN_OMEGA_MAG if deltaDeg > 0 else TURN_OMEGA_MAG  # negative omega increases yaw
        southMargin = closestApproachToSouthDeg(startYawDeg, deltaDeg)
        print(f"turn direction: {deltaDeg:+.0f} deg (omega={omega:+.2f} rad/s) chosen from the "
              f"measured start heading {startYawDeg:+.1f} deg for tether safety -- closest "
              f"approach to south during the sweep: {southMargin:.1f} deg "
              f"(>= {SOUTH_HEADING_MARGIN_DEG:.0f} required)")

        angleRad = math.radians(abs(deltaDeg))
        requestedTimeoutMs = abs(angleRad / TURN_OMEGA_MAG) * 1000.0 * 3.0 + 3000.0  # [ms]
        # Same clamp every move in this script goes through (see
        # clampTimeoutToClearance's own docstring). v_x=v_y=0 here -- an
        # in-place rotation has zero commanded translational speed, so the
        # clamp is a documented no-op, not a skipped check.
        timeoutMs, clearanceMm = clampTimeoutToClearance(
            requestedTimeoutMs, camBefore, 0.0, 0.0, args.geofence_margin)
        print(f"timeout safety clamp: requested={requestedTimeoutMs:.0f}ms "
              f"camera-confirmed clearance={clearanceMm} (pure rotation, zero translational "
              f"worst case) -> clamped={timeoutMs:.0f}ms")
        print(f"commanding in-place rotation: omega={omega:+.2f} rad/s "
              f"stop_angle={abs(deltaDeg):.0f} deg timeout={timeoutMs:.0f} ms")

        corrId = proto.move_twist(0.0, 0.0, omega, stop_angle=angleRad,
                                  timeout=timeoutMs, replace=True)
        ack = awaitAck(proto, geofence, corrId)
        moveAcked = ack is not None and ack.ok
        print(f"  enqueue ack: {ack} ({'OK' if moveAcked else 'FAIL'})")

        # Record every OTOS burst spanning the turn -- not just before/after
        # (ticket's own requirement: the frame hypotheses are tested against
        # the TRAJECTORY).
        trace = []
        seen = False
        deadline = time.monotonic() + timeoutMs / 1000.0 + 2.0
        while time.monotonic() < deadline:
            frames = drainFrames(proto, 0.05, geofence)
            for frame in frames:
                if frame.otos_reading is not None:
                    trace.append(frame.otos_reading)
                if frame.active:
                    seen = True
            if frames and not frames[-1].active and seen:
                break
        drainFrames(proto, SEGMENT_REST, geofence)

        camAfter = geofence.captureFix("lever-arm: rest after turn")
        if camAfter is None:
            print("FAIL: camera did not see tag 100 at rest after the turn")
            return 1
        camDeltaYawDeg = math.degrees(
            ((camAfter[2] - camBefore[2] + math.pi) % (2.0 * math.pi)) - math.pi)
        print(f"\ncamera-measured turn: {camDeltaYawDeg:+.1f} deg "
              f"(commanded {TURN_ANGLE_DEG:.0f} deg)")

        if len(trace) < 5:
            print(f"FAIL: only {len(trace)} OTOS bursts captured spanning the turn -- "
                  "too few to test the frame hypotheses")
            return 1
        print(f"\n{len(trace)} OTOS bursts spanning the turn:")
        for i, o in enumerate(trace):
            print(f"  [{i:2d}] x={o.x:+8.2f} y={o.y:+8.2f} heading={math.degrees(o.heading):+7.2f} deg")

        x0r, y0r, h0r = trace[0].x, trace[0].y, trace[0].heading
        maxDevFromStart = max(math.hypot(o.x - x0r, o.y - y0r) for o in trace)

        expectedOffsetMag = math.hypot(args.offset_x, args.offset_y)  # [mm]
        print(f"\ncentre-frame test: max deviation from the trace's own start point = "
              f"{maxDevFromStart:.2f} raw units")

        fit = fitCircle([(o.x, o.y) for o in trace])
        if fit is not None:
            centreX, centreY, radiusFit, residual = fit
            print(f"chip-frame test: fitted circle centre=({centreX:+.2f},{centreY:+.2f}) "
                  f"radius={radiusFit:.2f} raw units, fit residual RMS={residual:.2f} "
                  f"(expected radius if chip-frame ~ |odometry_offset_mm| = {expectedOffsetMag:.2f})")
        else:
            centreX = centreY = radiusFit = None
            print("chip-frame test: circle fit degenerate (too few or collinear points)")

        # CENTRE_BOUND is deliberately well below the ~48mm chip-frame radius
        # so the two hypotheses stay unambiguous at that separation.
        centreBound = max(15.0, expectedOffsetMag * 0.35)
        isCentre = maxDevFromStart <= centreBound
        isChip = (radiusFit is not None
                  and abs(radiusFit - expectedOffsetMag) <= expectedOffsetMag * 0.35
                  and maxDevFromStart > centreBound)

        print("\n=== FRAME DETERMINATION ===")
        frameResult: str
        offsetConfirmed: "bool | None"
        if isCentre and not isChip:
            frameResult = "centre"
            print(f"CENTRE-FRAME: reported (x,y) stayed within {maxDevFromStart:.2f} raw units "
                  f"of its start (bound {centreBound:.1f}) across a {camDeltaYawDeg:+.1f} deg "
                  "camera-measured turn.")
            print("This is consistent with the lever-arm transform being applied AND with "
                  "odometry_offset_mm's magnitude+sign being correct -- a wrong offset would "
                  "leave an uncancelled residual that traces a circle with the turn, which was "
                  "not observed.")
            offsetConfirmed = True
        elif isChip and not isCentre:
            frameResult = "chip"
            bodyX = math.cos(h0r) * (x0r - centreX) + math.sin(h0r) * (y0r - centreY)
            bodyY = -math.sin(h0r) * (x0r - centreX) + math.cos(h0r) * (y0r - centreY)
            print(f"CHIP-FRAME: reported (x,y) traced an arc of radius {radiusFit:.2f} raw units "
                  f"(expected |offset| {expectedOffsetMag:.2f}) -- consistent with the RAW sensor "
                  "pose, not the lever-arm-compensated centre.")
            print(f"fitted offset direction (start-of-turn body frame): x={bodyX:+.2f} y={bodyY:+.2f} "
                  f"vs. configured odometry_offset_mm x={args.offset_x:+.2f} y={args.offset_y:+.2f}")
            magErrPct = abs(radiusFit - expectedOffsetMag) / expectedOffsetMag * 100.0
            signMatch = (bodyX * args.offset_x >= 0.0) and (bodyY * args.offset_y >= 0.0)
            print(f"magnitude error: {magErrPct:.1f}%  sign match (both components): {signMatch}")
            offsetConfirmed = signMatch and magErrPct <= 35.0
        else:
            frameResult = "ambiguous"
            offsetConfirmed = None
            print(f"AMBIGUOUS: max deviation from start {maxDevFromStart:.2f} (centre bound "
                  f"{centreBound:.1f}), fitted radius "
                  f"{'n/a' if radiusFit is None else f'{radiusFit:.2f}'} vs. expected "
                  f"{expectedOffsetMag:.2f} -- neither hypothesis is a clean match. Reporting "
                  "raw numbers only, per this ticket's own mandate not to force a conclusion.")

        print("\n=== SUC-002 acceptance ===")
        print(f"  AC1 frame determined from measurement: "
              f"{'PASS' if frameResult != 'ambiguous' else 'FAIL/AMBIGUOUS'} (frame={frameResult})")
        if offsetConfirmed is True:
            print("  AC2 odometry_offset_mm magnitude+sign: CONFIRMED correct against the "
                  "measured frame")
        elif offsetConfirmed is False:
            print("  AC2 odometry_offset_mm magnitude+sign: MISMATCH found -- reported, NOT "
                  "corrected (out of this sprint's scope, see sprint.md Scope) -- flag as a "
                  "candidate follow-up issue for the stakeholder")
        else:
            print("  AC2 odometry_offset_mm magnitude+sign: UNDETERMINED (frame itself ambiguous)")

        return 0 if (moveAcked and frameResult != "ambiguous") else 1
    finally:
        if proto is not None:
            try:
                proto.tlmOff()
            except Exception:
                pass
            try:
                proto.estop()
            except Exception:
                pass
        if geofence is not None:
            geofence.close()
        if conn is not None:
            conn.disconnect()


# ---------------------------------------------------------------------------
# --mode heading (126-004)
# ---------------------------------------------------------------------------


def isTurnSafe(startDeg: float, deltaDeg: float, marginDeg: float) -> bool:  # [deg] [deg] [deg]
    """Tether-safety check for ONE candidate SIGNED turn: never sweeps
    through south (-90/270 deg) and keeps at least marginDeg clearance from
    it throughout. pickSafeTurnDirection() (126-002) already enforces this
    when it is free to pick whichever of +/-angle is safe; --mode heading
    needs to validate a SPECIFIC planned direction instead, since the
    ticket requires covering BOTH directions across the run, not just
    whichever pickSafeTurnDirection would choose each time."""
    if sweepCrossesAngle(startDeg, deltaDeg, -90.0):
        return False
    return closestApproachToSouthDeg(startDeg, deltaDeg) >= marginDeg


def runHeadingMode(args: argparse.Namespace) -> int:
    print("=== otos_calibration_bench --mode heading (ticket 126-004) ===")
    conn = proto = geofence = None
    rows: "list[dict]" = []
    try:
        conn, proto, geofence = connectAndArm(args)

        liveOk, _ = checkLiveness(proto, geofence, seconds=1.0)
        if not liveOk:
            return 1

        # Ping-pong pattern per magnitude: +angle, -angle, +angle, ... --
        # covers both directions (ticket's own requirement) while keeping
        # net rotation near zero across the whole sequence (tether rule),
        # since each turn is largely undone by the next. netRotationDeg is
        # tracked and printed as a running check on that, not assumed.
        netRotationDeg = 0.0  # [deg] cumulative camera-measured turn, tether-wrap tracker
        for magnitude in args.turn_angles:
            for i in range(2 * args.reps):
                plannedSign = 1.0 if i % 2 == 0 else -1.0
                label = f"mag={magnitude:.0f}deg {'ccw' if plannedSign > 0 else 'cw'} rep{i // 2}"

                camBefore = geofence.captureFix(f"heading before: {label}")
                if camBefore is None:
                    print(f"  SKIP {label}: camera did not see tag 100 at rest")
                    continue
                startYawDeg = math.degrees(camBefore[2])

                restFrames = drainFrames(proto, 0.4, geofence)
                otosRestList = [f.otos_reading for f in restFrames if f.otos_reading is not None]
                if not otosRestList:
                    print(f"  SKIP {label}: no OTOS reading at rest before the turn")
                    continue
                otosBefore = otosRestList[-1]

                deltaDeg = plannedSign * magnitude
                if not isTurnSafe(startYawDeg, deltaDeg, SOUTH_HEADING_MARGIN_DEG):
                    flipped = -deltaDeg
                    if not isTurnSafe(startYawDeg, flipped, SOUTH_HEADING_MARGIN_DEG):
                        print(f"  SKIP {label}: neither direction keeps "
                              f"{SOUTH_HEADING_MARGIN_DEG:.0f}deg clearance from south from "
                              f"heading {startYawDeg:+.1f}deg -- reposition")
                        continue
                    print(f"  {label}: planned direction unsafe from current heading -- "
                          "using the opposite direction instead")
                    deltaDeg = flipped

                # negative commanded omega INCREASES camera yaw
                # (playfield-testing.md: positive omega DECREASES yaw)
                omega = -TURN_OMEGA_MAG if deltaDeg > 0 else TURN_OMEGA_MAG
                southMargin = closestApproachToSouthDeg(startYawDeg, deltaDeg)

                angleRad = math.radians(abs(deltaDeg))
                requestedTimeoutMs = abs(angleRad / TURN_OMEGA_MAG) * 1000.0 * 3.0 + 3000.0  # [ms]
                timeoutMs, _ = clampTimeoutToClearance(
                    requestedTimeoutMs, camBefore, 0.0, 0.0, args.geofence_margin)

                print(f"commanding {label}: delta={deltaDeg:+.0f}deg omega={omega:+.2f}rad/s "
                      f"timeout={timeoutMs:.0f}ms south-clearance={southMargin:.1f}deg "
                      f"(>= {SOUTH_HEADING_MARGIN_DEG:.0f} required)")
                corrId = proto.move_twist(0.0, 0.0, omega, stop_angle=angleRad,
                                          timeout=timeoutMs, replace=True)
                ack = awaitAck(proto, geofence, corrId)
                if ack is None or not ack.ok:
                    print(f"  SKIP {label}: enqueue ack FAILED ({ack})")
                    continue

                awaitMoveCompletion(proto, geofence, timeoutMs)
                drainFrames(proto, SEGMENT_REST, geofence)

                camAfter = geofence.captureFix(f"heading after: {label}")
                afterFrames = drainFrames(proto, 0.4, geofence)
                otosAfterList = [f.otos_reading for f in afterFrames if f.otos_reading is not None]
                if camAfter is None or not otosAfterList:
                    print(f"  SKIP {label}: missing camera or OTOS reading after the turn")
                    continue
                otosAfter = otosAfterList[-1]

                camDeg = math.degrees(
                    ((camAfter[2] - camBefore[2] + math.pi) % (2.0 * math.pi)) - math.pi)
                otosDeg = math.degrees(
                    ((otosAfter.heading - otosBefore.heading + math.pi) % (2.0 * math.pi)) - math.pi)
                netRotationDeg += camDeg
                errDeg = otosDeg - camDeg
                errPct = errDeg / camDeg * 100.0 if abs(camDeg) > 1e-6 else float("nan")
                rows.append(dict(magnitude=magnitude, deltaDeg=deltaDeg, camDeg=camDeg,
                                  otosDeg=otosDeg, errDeg=errDeg, errPct=errPct))
                print(f"  {label}: camera={camDeg:+.1f}deg OTOS={otosDeg:+.1f}deg "
                      f"err={errDeg:+.2f}deg ({errPct:+.1f}%) "
                      f"net-rotation-so-far={netRotationDeg:+.1f}deg")

        print(f"\nnet rotation across the whole sequence: {netRotationDeg:+.1f}deg "
              "(tether-wrap tracker -- should stay near zero)")
        return _reportHeadingFit(rows, args)
    finally:
        if proto is not None:
            try:
                proto.tlmOff()
            except Exception:
                pass
            try:
                proto.estop()
            except Exception:
                pass
        if geofence is not None:
            geofence.close()
        if conn is not None:
            conn.disconnect()


def _reportHeadingFit(rows: "list[dict]", args: argparse.Namespace) -> int:
    """Fit OTOS heading-change = k * camera heading-change (through the
    origin) across every valid turn, derive the corrected
    otos_angular_scale, and print the per-turn table plus fitted scale,
    residual mean+spread, and an explicit PASS/FAIL against the currently-
    committed value."""
    print(f"\n=== heading-scale fit ({len(rows)} valid turns) ===")
    if len(rows) < 6:
        print(f"FAIL: only {len(rows)} valid turns -- too few to fit a scale reliably "
              "(need at least a handful spanning magnitudes/directions)")
        return 1

    print(f"{'mag_deg':>8} {'dir':>4} {'camera_deg':>11} {'otos_deg':>10} "
          f"{'err_deg':>8} {'err_pct':>8}")
    for r in rows:
        print(f"{r['magnitude']:8.0f} {'ccw' if r['deltaDeg'] > 0 else 'cw':>4} "
              f"{r['camDeg']:11.2f} {r['otosDeg']:10.2f} {r['errDeg']:8.2f} {r['errPct']:8.2f}")

    camList = [r["camDeg"] for r in rows]
    otosList = [r["otosDeg"] for r in rows]
    slope = fitScaleThroughOrigin(camList, otosList)  # otos_deg ~= slope * camera_deg
    correctionRatio = (1.0 / slope) if slope and not math.isnan(slope) else float("nan")

    path = _resolveRobotJsonPath(args.robot_json)
    committed = loadCommittedScale(path, "otos_angular_scale")
    correctedScale = committed * correctionRatio

    meanErrDeg, sdErrDeg = meanAndStdDev([r["errDeg"] for r in rows])
    meanErrPct, sdErrPct = meanAndStdDev([r["errPct"] for r in rows])
    n = len(rows)
    standardErrorPct = sdErrPct / math.sqrt(n)
    ciLow, ciHigh = meanErrPct - 1.96 * standardErrorPct, meanErrPct + 1.96 * standardErrorPct
    needsCorrection = not (ciLow <= 0.0 <= ciHigh)

    print(f"\nfitted OTOS/camera slope (through origin): {slope:.4f}")
    print(f"committed calibration.otos_angular_scale ({path.name}): {committed:.4f}")
    print(f"corrected otos_angular_scale = committed x (1/slope) = "
          f"{committed:.4f} x {correctionRatio:.4f} = {correctedScale:.4f}")
    print(f"residual: mean={meanErrDeg:+.3f}deg sd={sdErrDeg:.3f}deg "
          f"({meanErrPct:+.2f}% sd={sdErrPct:.2f}%, n={n})")
    print(f"95% CI of mean residual: [{ciLow:+.2f}%, {ciHigh:+.2f}%]")
    if needsCorrection:
        print(f"FAIL: committed otos_angular_scale={committed:.4f} disagrees with measurement "
              f"(95% CI of the mean residual excludes 0) -- correct to {correctedScale:.4f}")
    else:
        print(f"PASS: committed otos_angular_scale={committed:.4f} is within measured "
              "uncertainty (95% CI of the mean residual includes 0) -- no correction needed")
    return 0


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def buildArgParser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=DEFAULT_PORT,
                   help="the robot's own serial port (mbdeploy list's NEZHA2 row), "
                        "not the radio relay")
    p.add_argument("--mode", required=True,
                   choices=("units", "lever-arm", "distance", "heading"),
                   help="which ticket's measurement to run")
    p.add_argument("--leg", type=float, default=LEG,  # [mm]
                   help="straight-leg distance for --mode units")
    p.add_argument("--cruise", type=float, default=CRUISE,  # [mm/s]
                   help="straight-leg speed for --mode units / --mode distance")
    p.add_argument("--geofence-margin", type=float, default=FIELD_SAFE_MARGIN_CM,  # [cm]
                   help="stay this far from the field edge")
    p.add_argument("--robot-json", default=None,
                   help="override the robot config read for the configured "
                        "odometry_offset_mm / calibration scales (default: "
                        "the active robot, data/robots/active_robot.json)")
    p.add_argument("--lengths", type=lambda s: [float(v) for v in s.split(",") if v.strip()],
                   default=list(DISTANCE_LEGS_MM),  # [mm]
                   help="comma-separated straight-leg lengths, --mode distance "
                        f"(default {','.join(str(int(v)) for v in DISTANCE_LEGS_MM)})")
    p.add_argument("--turn-angles", type=lambda s: [float(v) for v in s.split(",") if v.strip()],
                   default=list(TURN_ANGLES_DEG),  # [deg]
                   help="comma-separated turn magnitudes, --mode heading "
                        f"(default {','.join(str(int(v)) for v in TURN_ANGLES_DEG)})")
    p.add_argument("--reps", type=int, default=REPS_PER_COMBO,
                   help="repeats per direction, --mode distance / --mode heading "
                        "(each rep is one leg or turn per direction, ping-ponged)")
    return p


def main() -> int:
    args = buildArgParser().parse_args()
    offsetX, offsetY, robotJsonPath = loadConfiguredOffsetMm(args.robot_json)
    args.offset_x, args.offset_y = offsetX, offsetY
    print(f"configured odometry_offset_mm: x={offsetX:+.2f} y={offsetY:+.2f} mm "
          f"(from {robotJsonPath})")
    try:
        if args.mode == "units":
            return runUnitsMode(args)
        if args.mode == "lever-arm":
            return runLeverArmMode(args)
        if args.mode == "distance":
            return runDistanceMode(args)
        return runHeadingMode(args)
    except GeofenceViolation as exc:
        print(f"ABORTED: {exc}")
        print("motors stopped by the geofence")
        return 2


if __name__ == "__main__":
    sys.exit(main())
