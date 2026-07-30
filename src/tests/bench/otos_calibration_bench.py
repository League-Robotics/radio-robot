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

MIN_TOUR_LEG_MM = 90.0           # [mm] shortest leg worth commanding for
                                  # criterion 2b's liveness tour (126-006
                                  # bench-gate fix) -- below this a "leg" is
                                  # too short to be a meaningful OTOS-vs-
                                  # camera liveness measurement, so geometry
                                  # that can't fit even this much falls back
                                  # to reorienting rather than clamping
                                  # further.

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


def planTourLegLength(camPose: "tuple[float, float, float]", legMm: float,
                      marginCm: float, minLegMm: float
                      ) -> "tuple[float, float] | None":  # [mm] [cm] [mm] -> (sign, [mm]) | None
    """126-006 bench-gate fix: criterion 2b's tour used to give up on a leg
    entirely whenever neither the FULL-length forward nor backward move fit
    the field -- a fixed-length leg routinely fails this depending on where
    an earlier turn left the robot, and used to cascade into skipping every
    later segment even though a SHORTER leg in the roomier direction would
    have fit fine. This is a tour-planner limitation, not a sensor problem.

    Picks whichever direction (forward=+1/backward=-1) has more camera-
    confirmed clearance along the current heading and shortens legMm to fit
    that clearance -- exactly parallel to clampTimeoutToClearance()'s own
    clamp-not-abandon pattern -- down to minLegMm. Returns None only if even
    minLegMm does not fit in EITHER direction; the caller should then try
    reorienting rather than give up outright."""
    x, y, yaw = camPose
    dirX, dirY = math.cos(yaw), math.sin(yaw)
    forwardMm = clearanceAlongDirectionCm(x, y, dirX, dirY, marginCm) * 10.0
    backwardMm = clearanceAlongDirectionCm(x, y, -dirX, -dirY, marginCm) * 10.0
    sign, clearanceMm = (1.0, forwardMm) if forwardMm >= backwardMm else (-1.0, backwardMm)
    if clearanceMm < minLegMm:
        return None
    return sign, min(legMm, clearanceMm)


def measureUnitsOnce(proto: "NezhaProtocol", geofence: "Geofence", legMm: float,
                     cruise: float, marginCm: float) -> "dict":  # [mm] [mm/s] [cm]
    """Core one-shot measurement behind --mode units (ticket 126-001): one
    straight-line move, OTOS-vs-camera units conclusion, at-rest jitter,
    motion-tracking check.

    Extracted (126-006) so --mode acceptance's criterion 2a can drive the
    EXACT SAME measurement runUnitsMode() uses, instead of --mode
    acceptance reimplementing it (the ticket's own reuse requirement) --
    runUnitsMode() below is now a thin connect/report wrapper around this.
    Caller must already hold an armed connection (connectAndArm()); this
    function does not connect, disconnect, or estop.

    Returns a dict with at least ``ok`` (bool). On an early failure (no
    camera fix, no OTOS bursts, no safe direction, clamp leaves too little
    time), returns ``{"ok": False, "error": "..."}`` and nothing else. On
    a completed measurement, additionally carries ``moveAcked``,
    ``movedEnough``, ``unit`` (``"mm"``/``"cm"``/``None``), ``camDeltaMm``,
    ``otosDeltaRaw``, ``mmRatio``, ``cmRatio``, ``jitterX``, ``jitterY``,
    ``jitterH`` -- ``ok`` is ``moveAcked and movedEnough and unit is not
    None``, matching the pre-126-006 inline logic exactly.
    """
    result: "dict" = {"ok": False}

    camBefore = geofence.captureFix("units: rest before move")
    if camBefore is None:
        result["error"] = "camera did not see tag 100 at rest before the move"
        return result

    restFrames = drainFrames(proto, 1.0, geofence)
    otosRest = [f.otos_reading for f in restFrames if f.otos_reading is not None]
    if len(otosRest) < 3:
        result["error"] = (f"too few OTOS bursts at rest to establish a jitter bound "
                            f"(got {len(otosRest)})")
        return result
    jitterX = max(o.x for o in otosRest) - min(o.x for o in otosRest)
    jitterY = max(o.y for o in otosRest) - min(o.y for o in otosRest)
    jitterH = max(o.heading for o in otosRest) - min(o.heading for o in otosRest)
    print(f"at-rest OTOS jitter over {len(otosRest)} bursts: "
          f"dx={jitterX:.2f} dy={jitterY:.2f} raw units, "
          f"dheading={math.degrees(jitterH):.3f} deg")
    otosBefore = otosRest[-1]

    sign = pickSafeLegDirection(camBefore, legMm, marginCm)
    if sign is None:
        result["error"] = (f"no field-safe straight direction for a {legMm:.0f} mm leg "
                            f"from current pose x={camBefore[0]:+.1f}cm y={camBefore[1]:+.1f}cm "
                            f"yaw={math.degrees(camBefore[2]):+.1f}deg")
        return result
    vx = sign * cruise
    requestedTimeoutMs = legMm / cruise * 1000.0 * 3.0 + 3000.0  # [ms]
    requiredMs = legMm / cruise * 1000.0  # [ms] bare time to complete the leg
    timeoutMs, clearanceMm = clampTimeoutToClearance(
        requestedTimeoutMs, camBefore, vx, 0.0, marginCm)
    worstCaseMm = timeoutMs / 1000.0 * abs(vx)
    print(f"timeout safety clamp: requested={requestedTimeoutMs:.0f}ms "
          f"camera-confirmed clearance={clearanceMm:.0f}mm -> clamped={timeoutMs:.0f}ms "
          f"(worst-case travel {worstCaseMm:.0f}mm)")
    if timeoutMs < requiredMs:
        result["error"] = (f"clamped timeout {timeoutMs:.0f}ms is below the {requiredMs:.0f}ms "
                            f"needed to complete the {legMm:.0f}mm leg at {cruise:.0f}mm/s -- "
                            "not enough camera-confirmed clearance from the current pose to run "
                            "this leg safely; reposition or shorten the leg rather than running it")
        return result

    print(f"commanding straight move: v_x={vx:+.0f} mm/s stop_distance={legMm:.0f} mm "
          f"timeout={timeoutMs:.0f} ms (direction={'forward' if sign > 0 else 'backward'}, "
          "chosen for field margin; straight moves never wrap the tether)")

    corrId = proto.move_twist(vx, 0.0, 0.0, stop_distance=legMm,
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
        result["error"] = "camera did not see tag 100 at rest after the move"
        return result

    camDeltaMm = math.hypot(camAfter[0] - camBefore[0], camAfter[1] - camBefore[1]) * 10.0
    otosDeltaRaw = math.hypot(otosAfter.x - otosBefore.x, otosAfter.y - otosBefore.y)
    mmRatio = otosDeltaRaw / camDeltaMm if camDeltaMm > 1e-6 else float("nan")
    cmRatio = (otosDeltaRaw * 10.0) / camDeltaMm if camDeltaMm > 1e-6 else float("nan")

    print(f"\ncamera-measured travel: {camDeltaMm:.1f} mm (commanded leg {legMm:.0f} mm)")
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

    result.update(dict(
        moveAcked=moveAcked, movedEnough=movedEnough, unit=unit,
        camDeltaMm=camDeltaMm, otosDeltaRaw=otosDeltaRaw, mmRatio=mmRatio, cmRatio=cmRatio,
        jitterX=jitterX, jitterY=jitterY, jitterH=jitterH,
    ))
    result["ok"] = bool(moveAcked and movedEnough and unit is not None)
    return result


def runUnitsMode(args: argparse.Namespace) -> int:
    print("=== otos_calibration_bench --mode units (ticket 126-001) ===")
    conn = proto = geofence = None
    try:
        conn, proto, geofence = connectAndArm(args)

        liveOk, _ = checkLiveness(proto, geofence, seconds=1.0)
        if not liveOk:
            return 1

        report = measureUnitsOnce(proto, geofence, args.leg, args.cruise, args.geofence_margin)
        if "error" in report:
            print(f"FAIL: {report['error']}")
            return 1

        print("\n=== SUC-001 acceptance ===")
        print("  AC1 presence (otos=1, flags bit0, connL=1, connR=1): PASS")
        print(f"  AC2 liveness+units (tracks motion, holds at rest, units stated): "
              f"{'PASS' if report['ok'] else 'FAIL'}")
        return 0 if report["ok"] else 1
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


def sweepMovesAwayFromSouth(startDeg: float, deltaDeg: float) -> bool:  # [deg] [deg] -> bool
    """True if a continuous sweep from startDeg by the SIGNED deltaDeg never
    comes closer to south (-90/270 deg, the tether direction) than the
    STARTING point itself -- i.e. angular distance from south is
    monotonically non-decreasing along the whole sweep.

    2026-07-30 bug fix (126-006 bench gate): the pre-fix guard only ever
    looked at the sweep's MINIMUM clearance from south
    (closestApproachToSouthDeg()) and rejected the turn outright if that
    minimum fell under SOUTH_HEADING_MARGIN_DEG -- but when the robot
    already STARTS inside the margin (e.g. left there by an earlier
    criterion's tour), the sweep's minimum is bounded below by the
    start's own clearance no matter which way it turns, so BOTH
    directions got rejected even though one of them strictly increases
    clearance and is exactly the recovery manoeuvre needed. A sweep that
    only moves away from south can only IMPROVE the tether situation, so
    it is safe regardless of how close the start already is -- it is only
    a sweep that APPROACHES or CROSSES south that must still respect the
    margin floor (see pickSafeTurnDirection() below)."""
    startDist = closestApproachToSouthDeg(startDeg, 0.0)
    sweepMin = closestApproachToSouthDeg(startDeg, deltaDeg)
    return sweepMin >= startDist - 1e-6


def pickSafeTurnDirection(startDeg: float, angleDeg: float, marginDeg: float
                          ) -> "float | None":  # [deg] [deg] [deg] -> signed [deg] or None
    """Choose the signed turn delta (+angleDeg or -angleDeg) from startDeg
    that respects the tether rule: never sweep through south (-90/270 deg),
    and either (a) the sweep only moves AWAY from south throughout (always
    safe, regardless of the start's own clearance -- see
    sweepMovesAwayFromSouth()), or (b) it keeps at least marginDeg of
    clearance from south throughout the sweep (a turn that technically
    avoids crossing south but skims within a few degrees of it is not
    tether-safe in practice -- a fixed east-starting direction is only safe
    when the robot actually starts near east; from other headings the safe
    direction can flip). Of the qualifying candidate directions, prefers
    whichever keeps the larger margin. Returns None if neither direction
    qualifies."""
    candidates: "list[tuple[float, float]]" = []
    for delta in (angleDeg, -angleDeg):
        if sweepCrossesAngle(startDeg, delta, -90.0):
            continue
        margin = closestApproachToSouthDeg(startDeg, delta)
        if sweepMovesAwayFromSouth(startDeg, delta) or margin >= marginDeg:
            candidates.append((margin, delta))
    if not candidates:
        return None
    candidates.sort(key=lambda c: -c[0])
    return candidates[0][1]


def pickSafeTurnToHeading(startDeg: float, targetDeg: float, marginDeg: float
                          ) -> "float | None":  # [deg] [deg] [deg] -> signed [deg] or None
    """Like pickSafeTurnDirection(), but aims at a specific targetDeg
    heading (mod 360) instead of a fixed +-angleDeg magnitude -- used by
    reorientForLegClearance() (126-006 bench-gate fix) to turn toward
    field-centre clearance rather than a fixed rotation amount. Tries the
    shorter sweep direction first, falls back to the longer way around, and
    only accepts a direction that respects the tether rule (never through
    south, and either monotonically away from south or keeping >= marginDeg
    clearance throughout -- see sweepMovesAwayFromSouth()/
    pickSafeTurnDirection()). Returns None if neither direction qualifies."""
    shortDelta = ((targetDeg - startDeg + 180.0) % 360.0) - 180.0
    longDelta = shortDelta - 360.0 if shortDelta >= 0.0 else shortDelta + 360.0
    candidates: "list[tuple[float, float]]" = []
    for delta in (shortDelta, longDelta):
        if abs(delta) < 1e-6:
            continue
        if sweepCrossesAngle(startDeg, delta, -90.0):
            continue
        margin = closestApproachToSouthDeg(startDeg, delta)
        if sweepMovesAwayFromSouth(startDeg, delta) or margin >= marginDeg:
            candidates.append((abs(delta), delta))
    if not candidates:
        return None
    candidates.sort(key=lambda c: c[0])
    return candidates[0][1]


def pickRecoveryTurnDelta(startDeg: float, targetClearanceDeg: float
                          ) -> "tuple[float, float] | None":
    # [deg] [deg] -> (signed [deg], resulting clearance [deg]) or None
    """The MINIMAL safe rotation from startDeg that reaches at least
    targetClearanceDeg of clearance from south -- the smallest recovery
    turn, not a fixed absolute compass heading. Returns None if startDeg
    is already at/beyond targetClearanceDeg (caller should treat that as
    "no turn needed", not a failure) or if no safe direction exists.

    Deliberately does NOT aim for a fixed heading like east or north: from
    a start already close to south, the shortest safe path to a FIXED
    heading can be forced the long way around (100+ deg) depending which
    side of south the fixed heading falls on, while the shortest path to
    merely CLEARING the margin is always small and on the "away" side --
    exactly the recovery manoeuvre described by the 126-006 bench-gate
    fix (rotating further away from south, not aiming for a specific
    compass point)."""
    startClearance = closestApproachToSouthDeg(startDeg, 0.0)
    if startClearance >= targetClearanceDeg:
        return None
    neededDeg = targetClearanceDeg - startClearance
    deltaDeg = pickSafeTurnDirection(startDeg, neededDeg, targetClearanceDeg)
    if deltaDeg is None:
        return None
    endClearance = closestApproachToSouthDeg(startDeg + deltaDeg, 0.0)
    return deltaDeg, endClearance


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


def measureFrameOnce(proto: "NezhaProtocol", geofence: "Geofence", offsetX: float,
                     offsetY: float, turnAngleDeg: float, marginCm: float) -> "dict":
                     # [mm] [mm] [deg] [cm]
    """Core one-shot measurement behind --mode lever-arm (ticket 126-002):
    one in-place rotation, OTOS trajectory spanning it, centre-frame vs.
    chip-frame hypothesis test, lever-arm magnitude+sign confirmation.

    Extracted (126-006) so --mode acceptance's criterion 3 can drive the
    EXACT SAME measurement runLeverArmMode() uses, instead of --mode
    acceptance reimplementing it. Caller must already hold an armed
    connection; this function does not connect, disconnect, or estop.

    Returns a dict with at least ``ok`` (bool). On an early failure,
    returns ``{"ok": False, "error": "..."}``. On a completed measurement,
    additionally carries ``moveAcked``, ``frameResult``
    (``"centre"``/``"chip"``/``"ambiguous"``), ``offsetConfirmed``
    (``bool | None``), ``camDeltaYawDeg``, ``camDeltaMm``, ``otosDeltaMm``,
    ``deviationFromCamera``, ``maxDevFromStart``, ``centreBound``,
    ``radiusFit``, ``expectedOffsetMag`` -- ``ok`` is ``moveAcked and
    frameResult != "ambiguous"``.

    The centre-frame discriminator (126-006 fix) compares OTOS-reported
    translation over the turn against the CAMERA-MEASURED centre
    translation over the same turn, not against an idealised zero -- see
    the inline comment ahead of ``centreBound`` below for why the
    zero-referenced version (pre-126-006) was wrong and camera-disproven.
    """
    result: "dict" = {"ok": False}

    camBefore = geofence.captureFix("lever-arm: rest before turn")
    if camBefore is None:
        result["error"] = "camera did not see tag 100 at rest before the turn"
        return result
    x0, y0, yaw0 = camBefore
    startYawDeg = math.degrees(yaw0)

    if abs(x0) > Geofence.HALF_W - marginCm or abs(y0) > Geofence.HALF_H - marginCm:
        result["error"] = (f"starting pose x={x0:+.1f}cm y={y0:+.1f}cm is already within "
                            f"{marginCm:.0f}cm of the field edge -- reposition before turning")
        return result

    # Direction is chosen from the MEASURED starting heading, not fixed:
    # a fixed east->west-through-north direction is only tether-safe
    # when the robot actually starts near east. Whichever direction
    # keeps the larger clearance from south wins.
    deltaDeg = pickSafeTurnDirection(startYawDeg, turnAngleDeg, SOUTH_HEADING_MARGIN_DEG)
    if deltaDeg is None:
        result["error"] = (f"no tether-safe {turnAngleDeg:.0f} deg turn direction from current "
                            f"heading {startYawDeg:+.1f} deg keeps >= {SOUTH_HEADING_MARGIN_DEG:.0f} "
                            "deg clearance from south (-90/270 deg, tether direction) -- "
                            "reposition before turning")
        return result
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
        requestedTimeoutMs, camBefore, 0.0, 0.0, marginCm)
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
        result["error"] = "camera did not see tag 100 at rest after the turn"
        return result
    camDeltaYawDeg = math.degrees(
        ((camAfter[2] - camBefore[2] + math.pi) % (2.0 * math.pi)) - math.pi)
    camDeltaMm = math.hypot((camAfter[0] - camBefore[0]) * 10.0,
                            (camAfter[1] - camBefore[1]) * 10.0)  # [mm] centre translation, camera truth
    print(f"\ncamera-measured turn: {camDeltaYawDeg:+.1f} deg "
          f"(commanded {turnAngleDeg:.0f} deg), camera-measured CENTRE translation over the "
          f"turn: {camDeltaMm:.1f} mm")

    if len(trace) < 5:
        result["error"] = f"only {len(trace)} OTOS bursts captured spanning the turn -- too few to test the frame hypotheses"
        return result
    print(f"\n{len(trace)} OTOS bursts spanning the turn:")
    for i, o in enumerate(trace):
        print(f"  [{i:2d}] x={o.x:+8.2f} y={o.y:+8.2f} heading={math.degrees(o.heading):+7.2f} deg")

    x0r, y0r, h0r = trace[0].x, trace[0].y, trace[0].heading
    maxDevFromStart = max(math.hypot(o.x - x0r, o.y - y0r) for o in trace)
    otosDeltaMm = math.hypot(trace[-1].x - x0r, trace[-1].y - y0r)  # [mm] endpoint-to-endpoint,
    # matching what the camera fix pair actually brackets (rest-before vs. rest-after)

    expectedOffsetMag = math.hypot(offsetX, offsetY)  # [mm]
    print(f"\nOTOS-reported CENTRE translation over the turn (trace start -> trace end) = "
          f"{otosDeltaMm:.2f} raw units (max deviation from the trace's own start point at "
          f"any point during the turn = {maxDevFromStart:.2f}, reported for diagnostics only)")

    fit = fitCircle([(o.x, o.y) for o in trace])
    if fit is not None:
        centreX, centreY, radiusFit, residual = fit
        print(f"chip-frame test: fitted circle centre=({centreX:+.2f},{centreY:+.2f}) "
              f"radius={radiusFit:.2f} raw units, fit residual RMS={residual:.2f} "
              f"(expected radius if chip-frame ~ |odometry_offset_mm| = {expectedOffsetMag:.2f})")
    else:
        centreX = centreY = radiusFit = None
        print("chip-frame test: circle fit degenerate (too few or collinear points)")

    # --- why this is no longer "deviation from zero" ---
    # This discriminator used to be `maxDevFromStart <= centreBound` with
    # centreBound = max(15.0, expectedOffsetMag * 0.35) (~16.7mm for this
    # robot's ~47.83mm |odometry_offset_mm|) -- i.e. it asserted the robot
    # CENTRE translates by approximately ZERO during an in-place turn.
    # That premise is false and camera-verified false: a same-session
    # camera-vs-camera measurement across one 90deg in-place turn showed
    # the physical centre itself moves ~11.7mm (before x=+4.9cm y=+11.0cm
    # -> after x=+5.9cm y=+10.4cm) while the OTOS-reported centre moved
    # ~19.0mm (start x=+219,y=+5 -> end x=+225,y=-13, in the OTOS's own raw
    # units, which ticket 126-001 established track camera mm ~1:1) over
    # the SAME turn -- the robot scrubs about a pivot that is not its
    # geometric centre, so a real, nonzero, camera-confirmed translation is
    # the CORRECT expectation under the centre-frame hypothesis, not a test
    # failure. Comparing OTOS translation against an idealised zero was
    # therefore testing pivot quality (how cleanly the robot spins in
    # place), not reference frame -- the thing this criterion is actually
    # supposed to discriminate. DO NOT reinstate a zero-referenced bound.
    #
    # The fix: compare OTOS-reported translation against the CAMERA-
    # MEASURED centre translation over the same turn, not against zero.
    # The two hypotheses stay just as separable at this offset/turn size:
    #   CENTRE frame -- OTOS translation tracks camera translation; both
    #     driven by the same physical scrub, same order of magnitude
    #     (measured instance: 19.0mm OTOS vs. 11.7mm camera).
    #   CHIP frame -- the fitted circle radius lands near
    #     |odometry_offset_mm| (~47.83mm here) regardless of how little the
    #     centre itself moved (measured instance: fitted radius 27.51mm,
    #     42% off |offset| -- nowhere near it).
    # centreBound keeps the SAME margin the old bound used
    # (max(15.0, expectedOffsetMag * 0.35), deliberately well below the
    # chip-frame radius so the two hypotheses stay unambiguous) but now
    # applies it to the RESIDUAL after subtracting the camera's own
    # measured translation, not to the raw OTOS translation itself.
    centreBound = max(15.0, expectedOffsetMag * 0.35)
    deviationFromCamera = abs(otosDeltaMm - camDeltaMm)  # [mm]
    isCentre = deviationFromCamera <= centreBound
    isChip = (radiusFit is not None
              and abs(radiusFit - expectedOffsetMag) <= expectedOffsetMag * 0.35
              and deviationFromCamera > centreBound)

    print(f"\ncentre-frame test: |OTOS translation - camera translation| = "
          f"{deviationFromCamera:.2f} raw units (bound {centreBound:.1f})")

    print("\n=== FRAME DETERMINATION ===")
    frameResult: str
    offsetConfirmed: "bool | None"
    if isCentre and not isChip:
        frameResult = "centre"
        print(f"CENTRE-FRAME: OTOS-reported translation ({otosDeltaMm:.2f} raw units) tracked "
              f"the camera-measured centre translation ({camDeltaMm:.2f} mm) within "
              f"{deviationFromCamera:.2f} (bound {centreBound:.1f}) across a "
              f"{camDeltaYawDeg:+.1f} deg camera-measured turn.")
        print("This is consistent with the lever-arm transform being applied AND with "
              "odometry_offset_mm's magnitude+sign being correct -- a wrong offset would "
              "leave an uncancelled residual that traces a circle of radius ~|offset| with "
              "the turn (a much larger, offset-scaled translation independent of the "
              "camera's own measured centre motion), which was not observed.")
        offsetConfirmed = True
    elif isChip and not isCentre:
        frameResult = "chip"
        bodyX = math.cos(h0r) * (x0r - centreX) + math.sin(h0r) * (y0r - centreY)
        bodyY = -math.sin(h0r) * (x0r - centreX) + math.cos(h0r) * (y0r - centreY)
        print(f"CHIP-FRAME: reported (x,y) traced an arc of radius {radiusFit:.2f} raw units "
              f"(expected |offset| {expectedOffsetMag:.2f}) -- consistent with the RAW sensor "
              "pose, not the lever-arm-compensated centre.")
        print(f"fitted offset direction (start-of-turn body frame): x={bodyX:+.2f} y={bodyY:+.2f} "
              f"vs. configured odometry_offset_mm x={offsetX:+.2f} y={offsetY:+.2f}")
        magErrPct = abs(radiusFit - expectedOffsetMag) / expectedOffsetMag * 100.0
        signMatch = (bodyX * offsetX >= 0.0) and (bodyY * offsetY >= 0.0)
        print(f"magnitude error: {magErrPct:.1f}%  sign match (both components): {signMatch}")
        offsetConfirmed = signMatch and magErrPct <= 35.0
    else:
        frameResult = "ambiguous"
        offsetConfirmed = None
        print(f"AMBIGUOUS: |OTOS - camera| translation deviation {deviationFromCamera:.2f} "
              f"(centre bound {centreBound:.1f}), fitted radius "
              f"{'n/a' if radiusFit is None else f'{radiusFit:.2f}'} vs. expected "
              f"{expectedOffsetMag:.2f} -- neither hypothesis is a clean match. Reporting "
              "raw numbers only, per this ticket's own mandate not to force a conclusion.")

    result.update(dict(
        moveAcked=moveAcked, frameResult=frameResult, offsetConfirmed=offsetConfirmed,
        camDeltaYawDeg=camDeltaYawDeg, camDeltaMm=camDeltaMm, otosDeltaMm=otosDeltaMm,
        deviationFromCamera=deviationFromCamera, maxDevFromStart=maxDevFromStart,
        centreBound=centreBound, radiusFit=radiusFit, expectedOffsetMag=expectedOffsetMag,
    ))
    result["ok"] = bool(moveAcked and frameResult != "ambiguous")
    return result


def runLeverArmMode(args: argparse.Namespace) -> int:
    print("=== otos_calibration_bench --mode lever-arm (ticket 126-002) ===")
    conn = proto = geofence = None
    try:
        conn, proto, geofence = connectAndArm(args)

        liveOk, _ = checkLiveness(proto, geofence, seconds=1.0)
        if not liveOk:
            return 1

        report = measureFrameOnce(proto, geofence, args.offset_x, args.offset_y,
                                  TURN_ANGLE_DEG, args.geofence_margin)
        if "error" in report:
            print(f"FAIL: {report['error']}")
            return 1

        frameResult = report["frameResult"]
        offsetConfirmed = report["offsetConfirmed"]
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

        return 0 if report["ok"] else 1
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
# --mode acceptance (126-006) -- the sprint's closing demonstration.
#
# Assembles and RE-RUNS tickets 001-004's own measurements (via the shared
# measureUnitsOnce()/measureFrameOnce() cores above, and a fresh multi-leg
# tour) rather than reimplementing them, plus the two checks the issue asks
# for that no single earlier ticket covers: a full-tour liveness check
# (criterion 2's "survives a full tour" clause) and the sim-suite + fusion-
# weights confirmation (criterion 7).
# ---------------------------------------------------------------------------


def awaitMoveCompletionCollecting(proto: "NezhaProtocol", geofence: "Geofence | None",
                                  timeoutMs: float) -> "list[TLMFrame]":  # [ms]
    """Like awaitMoveCompletion() but returns every frame drained across
    the wait, so a caller can inspect state CONTINUOUSLY through the move
    (e.g. otos_present, to catch a mid-move dropout) rather than only at
    the boundary. Added for --mode acceptance's tour-liveness check
    (126-006) -- the other modes only need completion, not the frames in
    between, so awaitMoveCompletion() itself is unchanged."""
    collected: "list[TLMFrame]" = []
    seen = False
    deadline = time.monotonic() + timeoutMs / 1000.0 + 2.0
    while time.monotonic() < deadline:
        frames = drainFrames(proto, 0.1, geofence)
        collected.extend(frames)
        if frames:
            if frames[-1].active:
                seen = True
            elif seen:
                break
    return collected


def _lastOtos(frames: "list[TLMFrame]"):
    readings = [f.otos_reading for f in frames if f.otos_reading is not None]
    return readings[-1] if readings else None


def reorientForLegClearance(proto: "NezhaProtocol", geofence: "Geofence",
                            camBefore: "tuple[float, float, float]",
                            marginCm: float) -> "dict":
    """126-006 bench-gate fix, step 3 of criterion 2b's leg-clamp
    (planTourLegLength()): when even MIN_TOUR_LEG_MM does not fit in
    either direction from camBefore, turn toward the field centre -- the
    heading with the most room in general -- and let the caller retry
    planTourLegLength() once from the new pose, rather than giving up on
    the rest of the tour. Tether-safe: reuses pickSafeTurnToHeading()'s
    south-margin guard, the same one every other turn in this script
    respects. The tether guard stays active even though the bench is
    currently running untethered over the radio relay (2026-07-30) -- see
    this ticket's own instruction not to relax it.

    Returns dict(ok=True, camAfter=<pose>) on success, or
    dict(ok=False, error=<str>) -- the caller must report the real reason,
    never silently fold this into a generic 'no camera fix' message."""
    x, y, yaw = camBefore
    targetDeg = math.degrees(math.atan2(-y, -x))  # heading toward field centre (0, 0)
    startYawDeg = math.degrees(yaw)
    deltaDeg = pickSafeTurnToHeading(startYawDeg, targetDeg, SOUTH_HEADING_MARGIN_DEG)
    if deltaDeg is None:
        return dict(ok=False, error=(
            f"no tether-safe turn from heading {startYawDeg:+.1f}deg toward the field-centre "
            f"heading {targetDeg:+.1f}deg"))

    omega = -TURN_OMEGA_MAG if deltaDeg > 0 else TURN_OMEGA_MAG  # negative omega increases yaw
    angleRad = math.radians(abs(deltaDeg))
    requestedTimeoutMs = abs(angleRad / TURN_OMEGA_MAG) * 1000.0 * 3.0 + 3000.0  # [ms]
    timeoutMs, _ = clampTimeoutToClearance(requestedTimeoutMs, camBefore, 0.0, 0.0, marginCm)
    print(f"  reorienting for leg clearance: turning {deltaDeg:+.1f}deg from "
          f"{startYawDeg:+.1f}deg toward {targetDeg:+.1f}deg (field centre) -- "
          f"omega={omega:+.2f}rad/s timeout={timeoutMs:.0f}ms")
    corrId = proto.move_twist(0.0, 0.0, omega, stop_angle=angleRad,
                              timeout=timeoutMs, replace=True)
    ack = awaitAck(proto, geofence, corrId)
    if ack is None or not ack.ok:
        return dict(ok=False, error=f"reorientation enqueue ack FAILED ({ack})")

    awaitMoveCompletion(proto, geofence, timeoutMs)
    drainFrames(proto, SEGMENT_REST, geofence)
    camAfter = geofence.captureFix("acceptance: tour leg-clearance reorientation")
    if camAfter is None:
        return dict(ok=False, error="camera did not see tag 100 after the leg-clearance "
                    "reorientation")
    print(f"  reorientation complete: now at {math.degrees(camAfter[2]):+.1f}deg")
    return dict(ok=True, camAfter=camAfter)


def runTourLiveness(proto: "NezhaProtocol", geofence: "Geofence",
                    args: argparse.Namespace) -> "dict":
    """Criterion 2's multi-leg liveness check: a short, tether-safe 4-leg/
    4-turn path -- 'comparable' to square_tour.py's own 4-leg/4-turn shape
    (this ticket's own wording), but with ALTERNATING turn direction
    (+90, -90, +90, -90 by default) instead of four same-direction turns.
    Four same-direction 90deg turns sum to a full 360deg wind of the
    tether even though camera yaw returns to the same value (yaw is
    periodic; the physical cable is not) -- alternating keeps net rotation
    near zero throughout, per the issue's own tether constraint.

    Camera-fixed AND OTOS-sampled at every one of the 9 segment boundaries
    (start + 8 segments), per `.claude/rules/playfield-testing.md`'s
    mandatory per-boundary convention. otos_present is checked on EVERY
    frame collected during EVERY segment's move (not just at the
    boundaries) via awaitMoveCompletionCollecting(), so a mid-move dropout
    is caught even if the pose has recovered by the next boundary fix --
    this is what proves "survives a full tour" rather than merely
    "reads fine at rest between moves".

    Returns a dict with ``ok`` (bool), ``camPoints``/``otosPoints``
    (parallel lists, one entry per boundary INCLUDING the start, ``None``
    for a boundary that could not be fixed), ``labels``,
    ``allOtosPresent``, ``segmentsOk`` (boundaries with both a camera AND
    an OTOS reading), and ``endAgreementMm`` (closing disagreement between
    the camera-measured and OTOS-reported net displacement over the whole
    tour, or ``None`` if the start/end boundary was not fixed)."""
    result: "dict" = dict(ok=False, camPoints=[], otosPoints=[], labels=[])
    legMm = args.leg
    turnDeg = TURN_ANGLE_DEG
    cruise = args.cruise
    margin = args.geofence_margin

    camFix = captureFixWithRetry(geofence, "tour: start")
    otosFix = _lastOtos(drainFrames(proto, 0.3, geofence))
    result["camPoints"].append(camFix)
    result["otosPoints"].append(otosFix)
    result["labels"].append("start")
    if camFix is None:
        result["error"] = "camera did not see tag 100 at the tour's starting rest pose"
        return result

    allOtosPresent = True
    segmentsOk = 1 if otosFix is not None else 0
    turnSignPreference = 1.0

    def _recordBoundary(label: str) -> None:
        drainFrames(proto, SEGMENT_REST, geofence)
        cam = captureFixWithRetry(geofence, f"tour after: {label}")
        otos = _lastOtos(drainFrames(proto, 0.3, geofence))
        result["camPoints"].append(cam)
        result["otosPoints"].append(otos)
        result["labels"].append(label)
        return cam, otos

    for i in range(4):
        # -- leg i+1 --
        # 126-006 bench-gate fix: a FIXED-length leg routinely fails to fit
        # the field in either direction depending on where the previous
        # turn left the robot (a tour-planner limitation, not a sensor
        # problem -- see planTourLegLength()'s own docstring). Clamp the
        # leg to whatever fits first; only reorient (once) if even the
        # MIN_TOUR_LEG_MM floor doesn't fit either way; only then skip.
        label = f"leg {i + 1}"
        legSkipReason = None
        camBefore = result["camPoints"][-1]
        planned = None
        if camBefore is None:
            legSkipReason = "no camera fix at the previous boundary to plan from"
            print(f"  SKIP {label}: {legSkipReason}")
        else:
            planned = planTourLegLength(camBefore, legMm, margin, MIN_TOUR_LEG_MM)
            if planned is None:
                print(f"  {label}: no direction fits even the {MIN_TOUR_LEG_MM:.0f}mm minimum "
                      f"leg from ({camBefore[0]:+.1f},{camBefore[1]:+.1f})cm -- reorienting "
                      "toward field-centre clearance and retrying once")
                reorient = reorientForLegClearance(proto, geofence, camBefore, margin)
                if not reorient["ok"]:
                    legSkipReason = (f"no field-safe geometry from current pose, and "
                                      f"reorientation failed ({reorient['error']})")
                    print(f"  SKIP {label}: {legSkipReason}")
                    camBefore = None
                else:
                    camBefore = reorient["camAfter"]
                    planned = planTourLegLength(camBefore, legMm, margin, MIN_TOUR_LEG_MM)
                    if planned is None:
                        legSkipReason = (f"no field-safe geometry even after reorienting to "
                                          f"{math.degrees(camBefore[2]):+.1f}deg")
                        print(f"  SKIP {label}: {legSkipReason}")
                        camBefore = None

        if camBefore is None or planned is None:
            result["camPoints"].append(None)
            result["otosPoints"].append(None)
            result["labels"].append(label)
        else:
            sign, useLegMm = planned
            if useLegMm < legMm - 1e-6:
                print(f"  {label}: clamped leg length {legMm:.0f}mm -> {useLegMm:.0f}mm to fit "
                      f"within {margin:.0f}cm field clearance")
            vx = sign * cruise
            requestedTimeoutMs = useLegMm / cruise * 1000.0 * 3.0 + 3000.0  # [ms]
            requiredMs = useLegMm / cruise * 1000.0  # [ms]
            timeoutMs, _ = clampTimeoutToClearance(requestedTimeoutMs, camBefore, vx, 0.0, margin)
            if timeoutMs < requiredMs:
                legSkipReason = "clamped timeout below the distance required"
                print(f"  SKIP {label}: {legSkipReason}")
                result["camPoints"].append(None)
                result["otosPoints"].append(None)
                result["labels"].append(label)
            else:
                print(f"commanding {label}: v_x={vx:+.0f} mm/s stop_distance={useLegMm:.0f} mm "
                      f"timeout={timeoutMs:.0f} ms")
                corrId = proto.move_twist(vx, 0.0, 0.0, stop_distance=useLegMm,
                                          timeout=timeoutMs, replace=True)
                ack = awaitAck(proto, geofence, corrId)
                if ack is None or not ack.ok:
                    legSkipReason = f"enqueue ack FAILED ({ack})"
                    print(f"  SKIP {label}: {legSkipReason}")
                    result["camPoints"].append(None)
                    result["otosPoints"].append(None)
                    result["labels"].append(label)
                else:
                    frames = awaitMoveCompletionCollecting(proto, geofence, timeoutMs)
                    tlm = [f for f in frames if f.flags is not None]
                    if tlm and not all(f.otos_present for f in tlm):
                        allOtosPresent = False
                        print(f"  WARNING {label}: otos_present dropped during this segment")
                    cam, otos = _recordBoundary(label)
                    if cam is not None and otos is not None:
                        segmentsOk += 1
                    elif cam is None:
                        legSkipReason = "camera did not confirm a fix after the leg's move"

        # -- turn i+1 (alternating direction) --
        label = f"turn {i + 1}"
        camBeforeTurn = result["camPoints"][-1]
        if camBeforeTurn is None:
            legLabel = f"leg {i + 1}"
            reason = legSkipReason or "no camera fix at the previous boundary"
            print(f"  SKIP {label}: cannot plan the turn -- {legLabel} did not leave a valid "
                  f"pose ({reason})")
            result["camPoints"].append(None)
            result["otosPoints"].append(None)
            result["labels"].append(label)
            turnSignPreference = -turnSignPreference
            continue

        startYawDeg = math.degrees(camBeforeTurn[2])
        preferredDelta = turnSignPreference * turnDeg
        if isTurnSafe(startYawDeg, preferredDelta, SOUTH_HEADING_MARGIN_DEG):
            deltaDeg = preferredDelta
        elif isTurnSafe(startYawDeg, -preferredDelta, SOUTH_HEADING_MARGIN_DEG):
            print(f"  {label}: the alternating direction is unsafe from the current heading -- "
                  "using the opposite direction instead")
            deltaDeg = -preferredDelta
        else:
            print(f"  SKIP {label}: neither direction keeps {SOUTH_HEADING_MARGIN_DEG:.0f}deg "
                  f"clearance from south from heading {startYawDeg:+.1f}deg")
            result["camPoints"].append(None)
            result["otosPoints"].append(None)
            result["labels"].append(label)
            turnSignPreference = -turnSignPreference
            continue

        omega = -TURN_OMEGA_MAG if deltaDeg > 0 else TURN_OMEGA_MAG
        southMargin = closestApproachToSouthDeg(startYawDeg, deltaDeg)
        angleRad = math.radians(abs(deltaDeg))
        requestedTimeoutMs = abs(angleRad / TURN_OMEGA_MAG) * 1000.0 * 3.0 + 3000.0  # [ms]
        timeoutMs, _ = clampTimeoutToClearance(requestedTimeoutMs, camBeforeTurn, 0.0, 0.0, margin)
        print(f"commanding {label}: delta={deltaDeg:+.0f}deg omega={omega:+.2f}rad/s "
              f"timeout={timeoutMs:.0f}ms south-clearance={southMargin:.1f}deg "
              f"(>= {SOUTH_HEADING_MARGIN_DEG:.0f} required)")
        corrId = proto.move_twist(0.0, 0.0, omega, stop_angle=angleRad,
                                  timeout=timeoutMs, replace=True)
        ack = awaitAck(proto, geofence, corrId)
        if ack is None or not ack.ok:
            print(f"  SKIP {label}: enqueue ack FAILED ({ack})")
            result["camPoints"].append(None)
            result["otosPoints"].append(None)
            result["labels"].append(label)
        else:
            frames = awaitMoveCompletionCollecting(proto, geofence, timeoutMs)
            tlm = [f for f in frames if f.flags is not None]
            if tlm and not all(f.otos_present for f in tlm):
                allOtosPresent = False
                print(f"  WARNING {label}: otos_present dropped during this segment")
            cam, otos = _recordBoundary(label)
            if cam is not None and otos is not None:
                segmentsOk += 1
        turnSignPreference = -turnSignPreference

    print(f"\ntour boundaries: {sum(1 for c in result['camPoints'] if c is not None)}/"
          f"{len(result['camPoints'])} camera fixes, "
          f"{sum(1 for o in result['otosPoints'] if o is not None)}/"
          f"{len(result['otosPoints'])} OTOS readings, otos_present held throughout every "
          f"completed segment: {allOtosPresent}")

    endAgreementMm, lastMutualIdx, totalBoundaries = tourEndAgreement(
        result["camPoints"], result["otosPoints"], camFix, otosFix)
    if endAgreementMm is not None:
        camEnd = result["camPoints"][lastMutualIdx]
        otosEnd = result["otosPoints"][lastMutualIdx]
        camDx = (camEnd[0] - camFix[0]) * 10.0  # [mm]
        camDy = (camEnd[1] - camFix[1]) * 10.0  # [mm]
        otosDx = otosEnd.x - otosFix.x
        otosDy = otosEnd.y - otosFix.y
        print(f"end-of-tour agreement (through boundary '{result['labels'][lastMutualIdx]}', "
              f"spanning {lastMutualIdx + 1}/{totalBoundaries} boundaries -- the LAST boundary "
              "with BOTH a camera fix and an OTOS reading at the SAME point in the tour, never "
              "mixed across boundaries): camera net displacement="
              f"({camDx:+.1f},{camDy:+.1f})mm OTOS net displacement=({otosDx:+.1f},{otosDy:+.1f})mm "
              f"disagreement={endAgreementMm:.1f}mm")
    else:
        print(f"end-of-tour agreement: INSUFFICIENT CAMERA COVERAGE -- no boundary beyond the "
              f"start ({totalBoundaries} boundaries total) has both a camera fix and an OTOS "
              "reading at the same point in the tour -- refusing to compute a camera-vs-OTOS "
              "disagreement number across partial/mismatched paths (it would look like a huge "
              "sensor error but be a bookkeeping artifact)")

    result["allOtosPresent"] = allOtosPresent
    result["segmentsOk"] = segmentsOk
    result["endAgreementMm"] = endAgreementMm
    # >= 6/9 boundaries (start + at least 5 of 8 segments) with both a
    # camera and an OTOS reading, AND otos_present held throughout every
    # segment that did complete -- a handful of field-safety skips (a
    # planned direction going unsafe mid-tour) is expected and does not
    # itself indicate an OTOS problem; a DROPOUT does.
    result["ok"] = bool(allOtosPresent and segmentsOk >= 6)
    return result


# Distance-calibration fit data -- MEASURED 2026-07-30, ticket 126-003,
# hardware run 2 (`--lengths 150,300,350`, tovez.json's then-committed
# otos_linear_scale=1.067 was still live at measurement time), 18/18 valid
# legs. Restated here (not re-measured this run) per this ticket's own
# instruction -- copied verbatim from 126-003's completion notes.
DISTANCE_CAL_ROWS_126_003 = [  # (length_mm, camera_mm, otos_mm)
    (150, 137.3, 143.1), (150, 136.5, 143.0), (150, 136.3, 141.4),
    (150, 136.5, 143.6), (150, 136.4, 140.7), (150, 138.2, 145.2),
    (300, 272.6, 280.9), (300, 279.0, 289.1), (300, 277.3, 286.3),
    (300, 277.5, 291.4), (300, 278.2, 289.0), (300, 281.0, 291.6),
    (350, 325.0, 335.4), (350, 195.5, 200.3), (350, 275.9, 285.4),
    (350, 342.0, 357.4), (350, 307.7, 318.7), (350, 340.9, 356.2),
]
DISTANCE_CAL_FITTED_SLOPE = 1.0384      # OTOS/camera, through origin, n=18
DISTANCE_CAL_OLD_COMMITTED = 1.067      # otos_linear_scale live at measurement time
DISTANCE_CAL_PRE_RESIDUAL = dict(meanPct=3.91, sdPct=0.79, ciLow=3.55, ciHigh=4.27, n=18)
DISTANCE_CAL_EXPECTED_CORRECTED = round(DISTANCE_CAL_OLD_COMMITTED / DISTANCE_CAL_FITTED_SLOPE, 4)
# Post-reflash spot-check (126-003, `--lengths 300 --reps 3`), 6/6 valid.
DISTANCE_CAL_POST_RESIDUAL = dict(meanPct=0.37, sdPct=0.34, ciLow=0.10, ciHigh=0.65, n=6)

# Heading-calibration fit data -- MEASURED 2026-07-30, ticket 126-004,
# hardware run (`--turn-angles 15,45,90,135 --reps 3`), 24/24 valid turns.
# Restated here (not re-measured this run) -- copied verbatim from
# 126-004's completion notes.
HEADING_CAL_ROWS_126_004 = [  # (magnitude_deg, camera_deg, otos_deg)
    (15, 13.42, 14.15), (15, -12.60, -14.61), (15, 11.92, 13.69),
    (15, -14.99, -14.67), (15, 15.24, 14.32), (15, -11.78, -13.52),
    (45, 41.71, 43.37), (45, -41.36, -43.89), (45, 44.93, 44.00),
    (45, -44.44, -43.32), (45, 46.20, 45.49), (45, -43.94, -43.32),
    (90, 85.26, 84.17), (90, -84.41, -83.77), (90, 85.95, 86.29),
    (90, -83.09, -85.31), (90, 87.94, 88.12), (90, -86.94, -85.77),
    (135, -131.41, -131.49), (135, 131.37, 128.97), (135, -128.54, -128.92),
    (135, 129.52, 128.92), (135, -129.23, -127.88), (135, 130.36, 131.84),
]
HEADING_CAL_FITTED_SLOPE = 0.9984       # OTOS/camera, through origin, n=24
HEADING_CAL_COMMITTED = 0.987           # unchanged -- measurement confirmed it correct
HEADING_CAL_RESIDUAL = dict(meanPct=1.82, sdPct=5.80, ciLow=-0.50, ciHigh=4.14, n=24)


def restateDistanceCalibration(robotJsonOverride: "str | None") -> "dict":
    """Criterion 4: restate 126-003's fitted distance scale and residual,
    and confirm the live `tovez.json` carries the corrected value."""
    path = _resolveRobotJsonPath(robotJsonOverride)
    committed = loadCommittedScale(path, "otos_linear_scale")
    r = DISTANCE_CAL_POST_RESIDUAL
    print(f"fitted OTOS/camera slope (126-003, n=18): {DISTANCE_CAL_FITTED_SLOPE:.4f}")
    print(f"pre-correction residual vs. then-committed {DISTANCE_CAL_OLD_COMMITTED}: "
          f"mean={DISTANCE_CAL_PRE_RESIDUAL['meanPct']:+.2f}% sd={DISTANCE_CAL_PRE_RESIDUAL['sdPct']:.2f}% "
          f"(95% CI [{DISTANCE_CAL_PRE_RESIDUAL['ciLow']:+.2f}%, {DISTANCE_CAL_PRE_RESIDUAL['ciHigh']:+.2f}%], "
          f"n={DISTANCE_CAL_PRE_RESIDUAL['n']}) -- excludes 0, correction applied")
    print(f"corrected otos_linear_scale = {DISTANCE_CAL_OLD_COMMITTED} x "
          f"(1/{DISTANCE_CAL_FITTED_SLOPE}) = {DISTANCE_CAL_EXPECTED_CORRECTED:.4f}")
    print(f"live committed calibration.otos_linear_scale ({path.name}): {committed:.4f}")
    print(f"post-reflash spot-check residual (n={r['n']}): mean={r['meanPct']:+.2f}% "
          f"sd={r['sdPct']:.2f}% (95% CI [{r['ciLow']:+.2f}%, {r['ciHigh']:+.2f}%])")
    ok = abs(committed - DISTANCE_CAL_EXPECTED_CORRECTED) < 0.001
    print(f"  {'PASS' if ok else 'FAIL'}: live tovez.json otos_linear_scale "
          f"{'matches' if ok else 'does NOT match'} the 126-003 correction "
          f"(expected {DISTANCE_CAL_EXPECTED_CORRECTED:.4f})")
    return dict(ok=ok, committed=committed, fittedSlope=DISTANCE_CAL_FITTED_SLOPE,
                preResidual=DISTANCE_CAL_PRE_RESIDUAL, postResidual=r, path=str(path))


def restateHeadingCalibration(robotJsonOverride: "str | None") -> "dict":
    """Criterion 5: restate 126-004's fitted heading scale and residual,
    and confirm the live `tovez.json` still carries the committed value
    unchanged (measurement CONFIRMED it correct -- no correction was
    made)."""
    path = _resolveRobotJsonPath(robotJsonOverride)
    committed = loadCommittedScale(path, "otos_angular_scale")
    r = HEADING_CAL_RESIDUAL
    print(f"fitted OTOS/camera slope (126-004, n=24): {HEADING_CAL_FITTED_SLOPE:.4f}")
    print(f"committed calibration.otos_angular_scale ({path.name}): {committed:.4f}")
    print(f"residual vs. committed {HEADING_CAL_COMMITTED}: mean={r['meanPct']:+.2f}% "
          f"sd={r['sdPct']:.2f}% (95% CI [{r['ciLow']:+.2f}%, {r['ciHigh']:+.2f}%], n={r['n']}) "
          "-- includes 0, no correction needed")
    ok = abs(committed - HEADING_CAL_COMMITTED) < 0.001
    print(f"  {'PASS' if ok else 'FAIL'}: live tovez.json otos_angular_scale "
          f"{'is unchanged at' if ok else 'does NOT match the expected'} "
          f"{HEADING_CAL_COMMITTED:.4f} (confirmed correct, not corrected)")
    return dict(ok=ok, committed=committed, fittedSlope=HEADING_CAL_FITTED_SLOPE,
                residual=r, path=str(path))


def printResidualSummaryTable(distSummary: dict, headSummary: dict) -> None:
    """Criterion 6: distance and heading residual mean/spread, side by
    side, so a later fusion decision has one table to weigh rather than
    two separately-printed reports."""
    print("\n=== residual summary (criterion 6) ===")
    print(f"{'measurement':>12} {'n':>4} {'mean':>9} {'sd':>8} {'95% CI':>20}")
    d = distSummary["postResidual"]
    print(f"{'distance':>12} {d['n']:4d} {d['meanPct']:+8.2f}% {d['sdPct']:7.2f}% "
          f"[{d['ciLow']:+.2f}%, {d['ciHigh']:+.2f}%]")
    h = headSummary["residual"]
    print(f"{'heading':>12} {h['n']:4d} {h['meanPct']:+8.2f}% {h['sdPct']:7.2f}% "
          f"[{h['ciLow']:+.2f}%, {h['ciHigh']:+.2f}%]")
    print("(distance row is the POST-REFLASH spot-check, n=6, the number that reflects the "
          "live otos_linear_scale=1.0275; heading row is the full n=24 fit, since no "
          "correction/reflash changed otos_angular_scale)")


def checkSimSuiteAndFusionWeights(robotJsonOverride: "str | None") -> "dict":
    """Criterion 7: run the sim suite fresh (not restated -- this is cheap
    enough, and stale, to always re-run) and confirm the estimator's OTOS
    fusion weights are still the untouched 0.0/0.0 sentinel."""
    import subprocess

    print("\n--- Criterion 7: sim suite + fusion weights untouched ---")
    proc = subprocess.run(
        ["uv", "run", "python", "-m", "pytest", "src/tests/sim", "-q"],
        cwd=str(_REPO_ROOT), capture_output=True, text=True, timeout=600)
    tail = "\n".join(proc.stdout.strip().splitlines()[-6:])
    print(tail)
    simOk = proc.returncode == 0
    print(f"sim suite: {'PASS' if simOk else 'FAIL'} (exit code {proc.returncode})")

    path = _resolveRobotJsonPath(robotJsonOverride)
    data = json.loads(path.read_text())
    weightHeading = data.get("estimator", {}).get("weight_heading_otos")
    weightOmega = data.get("estimator", {}).get("weight_omega_otos")
    weightsOk = (weightHeading == 0.0 and weightOmega == 0.0)
    print(f"estimator.weight_heading_otos={weightHeading} weight_omega_otos={weightOmega} "
          f"({path.name}): {'PASS -- both 0.0, untouched' if weightsOk else 'FAIL -- expected both 0.0'}")
    return dict(ok=bool(simOk and weightsOk), simOk=simOk, weightsOk=weightsOk,
                weightHeading=weightHeading, weightOmega=weightOmega, simTail=tail)


def writeAcceptanceChart(distSummary: dict, headSummary: dict, tourResult: dict,
                         out: str) -> None:
    """Criterion 9: one chart, three panels -- distance calibration fit,
    heading calibration fit, and the full-tour trajectory overlay
    (OTOS-reported path vs. camera-measured path), matching the project's
    existing bench-script chart convention (square_tour.py's/
    speed_map.py's PNG output pattern: matplotlib Agg backend, one
    fig.savefig() call, printed confirmation of the path written)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (axDist, axHead, axTour) = plt.subplots(1, 3, figsize=(19, 6.0))

    # --- panel 1: distance calibration fit ---
    camMm = [r[1] for r in DISTANCE_CAL_ROWS_126_003]
    otosMm = [r[2] for r in DISTANCE_CAL_ROWS_126_003]
    axDist.scatter(camMm, otosMm, s=22, color="#1f77b4", alpha=0.75, label="126-003 runs (n=18)")
    xs = [0.0, max(camMm) * 1.05]
    axDist.plot(xs, [x * DISTANCE_CAL_FITTED_SLOPE for x in xs], color="#d62728", lw=1.5,
               label=f"fit slope={DISTANCE_CAL_FITTED_SLOPE:.4f}")
    axDist.plot(xs, xs, color="#888888", lw=1.0, ls="--", label="ideal (slope=1)")
    axDist.set_xlabel("camera distance [mm]")
    axDist.set_ylabel("OTOS distance [mm]")
    axDist.set_title(f"Distance calibration (126-003)\ncorrected otos_linear_scale="
                     f"{DISTANCE_CAL_EXPECTED_CORRECTED:.4f} (post-reflash residual "
                     f"{DISTANCE_CAL_POST_RESIDUAL['meanPct']:+.2f}%)")
    axDist.legend(fontsize=8, loc="upper left")
    axDist.grid(True, alpha=0.3)

    # --- panel 2: heading calibration fit ---
    camDeg = [r[1] for r in HEADING_CAL_ROWS_126_004]
    otosDeg = [r[2] for r in HEADING_CAL_ROWS_126_004]
    axHead.scatter(camDeg, otosDeg, s=22, color="#2ca02c", alpha=0.75, label="126-004 turns (n=24)")
    lo, hi = min(camDeg) * 1.1, max(camDeg) * 1.1
    axHead.plot([lo, hi], [lo * HEADING_CAL_FITTED_SLOPE, hi * HEADING_CAL_FITTED_SLOPE],
               color="#d62728", lw=1.5, label=f"fit slope={HEADING_CAL_FITTED_SLOPE:.4f}")
    axHead.plot([lo, hi], [lo, hi], color="#888888", lw=1.0, ls="--", label="ideal (slope=1)")
    axHead.set_xlabel("camera heading change [deg]")
    axHead.set_ylabel("OTOS heading change [deg]")
    axHead.set_title(f"Heading calibration (126-004)\notos_angular_scale={HEADING_CAL_COMMITTED:.4f} "
                     f"unchanged (residual {HEADING_CAL_RESIDUAL['meanPct']:+.2f}%)")
    axHead.legend(fontsize=8, loc="upper left")
    axHead.grid(True, alpha=0.3)

    # --- panel 3: full-tour trajectory overlay ---
    camPoints = tourResult.get("camPoints", [])
    otosPoints = tourResult.get("otosPoints", [])
    idx0 = next((i for i in range(len(camPoints))
                if camPoints[i] is not None and otosPoints[i] is not None), None)
    camXs = [c[0] * 10.0 for c in camPoints if c is not None]  # [mm]
    camYs = [c[1] * 10.0 for c in camPoints if c is not None]
    if idx0 is not None:
        cx0, cy0, cyaw0 = camPoints[idx0]
        ox0, oy0, oh0 = otosPoints[idx0].x, otosPoints[idx0].y, otosPoints[idx0].heading
        # Align the OTOS trace to the camera (world) frame at the first
        # mutually-fixed boundary -- same origin AND heading -- so the
        # overlay compares TRAJECTORY SHAPE from a shared reference, not an
        # independently-established absolute frame mapping (that mapping
        # was never measured; ticket 002 established CENTRE-vs-CHIP frame,
        # not axis alignment to the camera's world frame).
        rot = cyaw0 - oh0
        cosR, sinR = math.cos(rot), math.sin(rot)
        alignedXs, alignedYs = [], []
        for o in otosPoints:
            if o is None:
                continue
            dx, dy = o.x - ox0, o.y - oy0
            alignedXs.append(cx0 * 10.0 + (dx * cosR - dy * sinR))
            alignedYs.append(cy0 * 10.0 + (dx * sinR + dy * cosR))
        axTour.plot(alignedXs, alignedYs, color="#ff7f0e", lw=1.6, marker="o", ms=4,
                   label="OTOS-reported path (aligned at start)")
    axTour.plot(camXs, camYs, color="#1f77b4", lw=1.6, marker="s", ms=4,
               label="camera-measured path")
    if camXs:
        axTour.plot([camXs[0]], [camYs[0]], marker="*", color="#2ca02c", ms=14, label="start")
    axTour.set_aspect("equal")
    axTour.set_xlabel("x [mm]")
    axTour.set_ylabel("y [mm]")
    agreement = tourResult.get("endAgreementMm")
    agreementTxt = f"{agreement:.0f}mm" if agreement is not None else "n/a"
    axTour.set_title(f"Full-tour liveness (126-006)\nend-of-tour disagreement: {agreementTxt}")
    axTour.legend(fontsize=8, loc="best")
    axTour.grid(True, alpha=0.3)

    fig.suptitle("OTOS bring-up acceptance (sprint 126) -- distance/heading calibration fits "
                "+ full-tour OTOS-vs-camera trajectory overlay", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(out, dpi=130)
    print(f"wrote {out}")


REORIENT_CLEARANCE_DEG = 45.0  # [deg] target clearance from south for criterion 3's
                                 # reorientation -- comfortably more than
                                 # SOUTH_HEADING_MARGIN_DEG (20deg), and enough
                                 # headroom that the subsequent TURN_ANGLE_DEG
                                 # (90deg) measurement turn can itself find a
                                 # tether-safe direction without re-triggering
                                 # this same guard.


def reorientForFrameCheck(proto: "NezhaProtocol", geofence: "Geofence",
                          fieldMarginCm: float) -> "dict":
    """--mode acceptance's criterion 3 (126-006 fix): reorient to a
    KNOWN-GOOD clearance from south (>= REORIENT_CLEARANCE_DEG, not a
    fixed compass heading -- see pickRecoveryTurnDelta()'s own docstring
    for why a fixed target can force an unnecessarily huge sweep) BEFORE
    measureFrameOnce()'s own measurement turn, so criterion 3 is
    SELF-CONTAINED and repeatable regardless of where an earlier criterion
    (2b's multi-leg tour) happened to leave the robot, rather than
    silently depending on that leftover heading or silently skipping when
    it turns out to be unsafe. Uses the corrected tether-safety guard
    (pickSafeTurnDirection() / sweepMovesAwayFromSouth()), which allows a
    sweep that only moves AWAY from south even when the start heading is
    already inside the margin of it -- exactly the situation 2b's tour can
    leave behind, and exactly the manoeuvre needed to recover from it.

    Returns a dict with ``ok`` (bool) and, on failure, ``error`` -- the
    caller must report the reason and FAIL criterion 3, never silently
    skip it."""
    camBefore = geofence.captureFix("acceptance: reorient before criterion 3")
    if camBefore is None:
        return dict(ok=False, error="camera did not see tag 100 to plan the "
                    "criterion-3 reorientation")
    startYawDeg = math.degrees(camBefore[2])

    recovery = pickRecoveryTurnDelta(startYawDeg, REORIENT_CLEARANCE_DEG)
    if recovery is None:
        startClearance = closestApproachToSouthDeg(startYawDeg, 0.0)
        if startClearance >= REORIENT_CLEARANCE_DEG:
            print(f"criterion 3 reorientation: already at {startYawDeg:+.1f}deg "
                  f"({startClearance:.1f}deg clear of south, >= {REORIENT_CLEARANCE_DEG:.0f} "
                  "required) -- no turn needed")
            return dict(ok=True)
        return dict(ok=False, error=(
            f"no tether-safe recovery turn from heading {startYawDeg:+.1f}deg "
            f"({startClearance:.1f}deg clear of south) reaches {REORIENT_CLEARANCE_DEG:.0f}deg "
            "clearance -- reposition before criterion 3 can proceed"))
    deltaDeg, endClearance = recovery

    omega = -TURN_OMEGA_MAG if deltaDeg > 0 else TURN_OMEGA_MAG  # negative omega increases yaw
    angleRad = math.radians(abs(deltaDeg))
    requestedTimeoutMs = abs(angleRad / TURN_OMEGA_MAG) * 1000.0 * 3.0 + 3000.0  # [ms]
    timeoutMs, clearanceMm = clampTimeoutToClearance(
        requestedTimeoutMs, camBefore, 0.0, 0.0, fieldMarginCm)
    print(f"criterion 3 reorientation: turning {deltaDeg:+.1f}deg from {startYawDeg:+.1f}deg "
          f"(recovery manoeuvre -- moves AWAY from south) -- omega={omega:+.2f}rad/s "
          f"timeout={timeoutMs:.0f}ms resulting south-clearance={endClearance:.1f}deg "
          f"(target >= {REORIENT_CLEARANCE_DEG:.0f})")
    corrId = proto.move_twist(0.0, 0.0, omega, stop_angle=angleRad,
                              timeout=timeoutMs, replace=True)
    ack = awaitAck(proto, geofence, corrId)
    if ack is None or not ack.ok:
        return dict(ok=False, error=f"criterion-3 reorientation enqueue ack FAILED ({ack})")

    awaitMoveCompletion(proto, geofence, timeoutMs)
    drainFrames(proto, SEGMENT_REST, geofence)
    camAfter = geofence.captureFix("acceptance: reorient after criterion 3")
    if camAfter is None:
        return dict(ok=False, error="camera did not see tag 100 after the criterion-3 "
                    "reorientation")
    print(f"criterion 3 reorientation complete: now at {math.degrees(camAfter[2]):+.1f}deg")
    return dict(ok=True)


# ---------------------------------------------------------------------------
# Criterion 2b tag-dropout retry + honest partial-coverage reporting
# (126-006 bench-gate fix, 2026-07-30). Tag 100 has repeatedly dropped out
# mid-tour and come back on its own -- most often traced to the playfield
# Shelly light relay (192.168.1.122) switching itself off. A dropout must
# be RETRIED, not treated as instant, permanent loss; and an end-of-tour
# camera-vs-OTOS agreement figure must never be computed by mixing a
# camera fix from one boundary with an OTOS reading from another, or by
# comparing a partial camera path against a full OTOS path -- either is a
# meaningless number that LOOKS like a huge sensor error but is really a
# bookkeeping artifact.
# ---------------------------------------------------------------------------


def _playfieldLightsOn() -> "bool | None":  # None if the relay could not be reached
    """Query the playfield Shelly relay's light-switch state directly
    (square_tour.checkPlayfieldLights()'s own URL) -- returns True/False,
    or None if the relay is unreachable (a different network problem, not
    this script's to diagnose)."""
    import json
    import urllib.error
    import urllib.request

    from square_tour import PLAYFIELD_LIGHTS_URL

    try:
        with urllib.request.urlopen(PLAYFIELD_LIGHTS_URL, timeout=2.0) as resp:
            status = json.loads(resp.read())
        return bool(status.get("output"))
    except (urllib.error.URLError, OSError, ValueError) as exc:
        print(f"  WARNING: could not reach the playfield lights relay: {exc!r}")
        return None


def _turnPlayfieldLightsOn() -> None:
    """Best-effort: switch the playfield Shelly relay's light channel on.
    Never raises -- a failed light-on attempt just means the subsequent
    captureFix() retries are less likely to recover, which is already
    handled by giving up honestly after the retry window."""
    import urllib.error
    import urllib.request

    url = "http://192.168.1.122/rpc/Switch.Set?id=0&on=true"
    try:
        urllib.request.urlopen(url, timeout=2.0)
        print("  turned playfield lights ON (Shelly 192.168.1.122)")
    except (urllib.error.URLError, OSError) as exc:
        print(f"  WARNING: failed to turn playfield lights on: {exc!r}")


def captureFixWithRetry(geofence: "Geofence", label: str,
                        retrySeconds: float = 5.0) -> "tuple[float, float, float] | None":
    # [s]
    """Like Geofence.captureFix(), but on losing tag 100 -- a dropout that
    has repeatedly happened and self-recovered during this sprint's bench
    sessions -- checks whether the playfield Shelly lights have switched
    themselves off (a known, observed cause) and turns them back on, then
    RETRIES the fix for up to retrySeconds before giving up. Never masks a
    genuine, lasting loss: if the tag still is not seen after the retry
    window, returns None exactly like captureFix() does, and the caller
    must treat that as a real failure, not silently skip past it."""
    fix = geofence.captureFix(label)
    if fix is not None:
        return fix

    print(f"  '{label}': tag 100 lost -- checking playfield lights before retrying")
    lightsOn = _playfieldLightsOn()
    if lightsOn is False:
        print("  playfield lights are OFF -- turning them on (known dropout cause)")
        _turnPlayfieldLightsOn()
        time.sleep(1.0)  # let the light + camera exposure settle

    deadline = time.monotonic() + retrySeconds
    attempt = 0
    while time.monotonic() < deadline:
        attempt += 1
        time.sleep(0.5)
        fix = geofence.captureFix(f"{label} (retry {attempt})")
        if fix is not None:
            print(f"  '{label}': tag 100 recovered on retry {attempt}")
            return fix
    print(f"  '{label}': tag 100 still not seen after {retrySeconds:.0f}s of retrying -- "
          "giving up")
    return None


def tourEndAgreement(camPoints: "list", otosPoints: "list",
                     camStart: "tuple[float, float, float] | None",
                     otosStart) -> "tuple[float | None, int, int]":  # [mm], index, count
    """Criterion 2b's end-of-tour camera-vs-OTOS disagreement, scoped
    honestly: walk BACKWARD from the last boundary to the LAST index where
    BOTH a camera fix AND an OTOS reading are present at THAT SAME
    boundary, and compare against the start.

    This replaces the pre-126-006 logic, which searched camPoints and
    otosPoints for their last non-None entry INDEPENDENTLY -- two
    reversed searches that can each land on a DIFFERENT boundary index
    when the two signals drop out at different points in the tour. That
    produced a distance between two different moments in time and printed
    it as "disagreement", which is meaningless -- it looks exactly like a
    huge sensor error but is really a bookkeeping artifact of comparing a
    partial camera path against a full OTOS path.

    Returns (endAgreementMm | None, lastMutualIndex, totalBoundaries).
    endAgreementMm is None (insufficient camera coverage) if no boundary
    beyond the start has both a camera fix and an OTOS reading at the same
    index -- the caller must report that honestly, not print a number."""
    n = len(camPoints)
    lastMutual = None
    for i in range(n - 1, 0, -1):  # never re-select index 0 (the start) as "the end"
        if camPoints[i] is not None and otosPoints[i] is not None:
            lastMutual = i
            break
    if lastMutual is None or camStart is None or otosStart is None:
        return None, (lastMutual if lastMutual is not None else 0), n

    camEnd = camPoints[lastMutual]
    otosEnd = otosPoints[lastMutual]
    camDx = (camEnd[0] - camStart[0]) * 10.0  # [mm]
    camDy = (camEnd[1] - camStart[1]) * 10.0  # [mm]
    otosDx = otosEnd.x - otosStart.x
    otosDy = otosEnd.y - otosStart.y
    agreementMm = math.hypot(camDx - otosDx, camDy - otosDy)
    return agreementMm, lastMutual, n


def runAcceptanceMode(args: argparse.Namespace) -> int:
    print("=== otos_calibration_bench --mode acceptance (ticket 126-006) ===")
    print("Runs all seven of the issue's acceptance criteria in one session, reusing "
          "tickets 001-004's own measurement cores rather than reimplementing them.")
    conn = proto = geofence = None
    results: "dict[str, bool]" = {}
    tourResult: "dict" = dict(camPoints=[], otosPoints=[], labels=[])
    try:
        conn, proto, geofence = connectAndArm(args)

        print("\n--- Criterion 1: presence (STATUS otos=1, flags bit0, READY connL/connR) ---")
        liveOk, _ = checkLiveness(proto, geofence, seconds=1.0)
        results["1_presence"] = liveOk
        if not liveOk:
            print("ABORT: robot is not READY -- cannot safely continue the acceptance run")
            return 1

        print("\n--- Criterion 2a: liveness -- single-move units/liveness re-check "
              "(ticket 001's own measurement) ---")
        unitsReport = measureUnitsOnce(proto, geofence, args.leg, args.cruise, args.geofence_margin)
        if "error" in unitsReport:
            print(f"FAIL: {unitsReport['error']}")
        results["2a_units_liveness"] = bool(unitsReport.get("ok"))

        print("\n--- Criterion 2b: liveness -- multi-leg tour, per-boundary camera + OTOS fixes ---")
        tourResult = runTourLiveness(proto, geofence, args)
        if "error" in tourResult:
            print(f"FAIL: {tourResult['error']}")
        results["2b_tour_liveness"] = bool(tourResult.get("ok"))

        print("\n--- Criterion 3: lever-arm / frame -- fresh rotation re-check "
              "(ticket 002's own measurement) ---")
        print("reorienting to a known-good heading first -- criterion 3 must be "
              "self-contained, not dependent on wherever criterion 2b's tour left the robot")
        reorient = reorientForFrameCheck(proto, geofence, args.geofence_margin)
        if not reorient.get("ok"):
            print(f"FAIL: criterion 3 reorientation failed: {reorient.get('error')}")
            results["3_frame"] = False
        else:
            frameReport = measureFrameOnce(proto, geofence, args.offset_x, args.offset_y,
                                           TURN_ANGLE_DEG, args.geofence_margin)
            if "error" in frameReport:
                print(f"FAIL: {frameReport['error']}")
            results["3_frame"] = bool(frameReport.get("ok"))

        print("\n--- Criterion 4: distance calibration -- RESTATED fit from ticket 126-003 "
              "(n=18 straight-line runs, NOT re-measured this session -- re-running that "
              "sweep is expensive and out of this ticket's scope; only the live tovez.json "
              "otos_linear_scale value below is freshly read THIS run) ---")
        distSummary = restateDistanceCalibration(args.robot_json)
        results["4_distance_cal"] = bool(distSummary.get("ok"))

        print("\n--- Criterion 5: heading calibration -- RESTATED fit from ticket 126-004 "
              "(n=24 in-place turns, NOT re-measured this session; only the live "
              "tovez.json otos_angular_scale value below is freshly read THIS run) ---")
        headSummary = restateHeadingCalibration(args.robot_json)
        results["5_heading_cal"] = bool(headSummary.get("ok"))

        printResidualSummaryTable(distSummary, headSummary)
        results["6_residuals_stated"] = True  # the table above IS this criterion

        simSummary = checkSimSuiteAndFusionWeights(args.robot_json)
        results["7_sim_and_fusion"] = bool(simSummary.get("ok"))

        chartPath = args.chart or "src/tests/bench/otos_calibration_bench_acceptance.png"
        writeAcceptanceChart(distSummary, headSummary, tourResult, chartPath)

        print("\n=== CONSOLIDATED ACCEPTANCE REPORT (issue otos-telemetry-bring-up-and-"
              "camera-calibration.md) ===")
        labels = {
            "1_presence": "1. Presence (otos=1, flags bit0, READY connL/connR)",
            "2a_units_liveness": "2a. Liveness -- single-move units/liveness re-check",
            "2b_tour_liveness": "2b. Liveness -- multi-leg tour (no dropout, per-boundary fixes)",
            "3_frame": "3. Lever-arm / frame -- centre-frame + offset confirmed",
            "4_distance_cal": "4. Distance calibration -- otos_linear_scale corrected + verified",
            "5_heading_cal": "5. Heading calibration -- otos_angular_scale confirmed correct",
            "6_residuals_stated": "6. Residuals stated (summary table above)",
            "7_sim_and_fusion": "7. Sim suite passes + fusion weights untouched (0.0/0.0)",
        }
        # Explicit provenance per criterion -- a reader must not mistake a
        # quotation of an earlier ticket's fit for a fresh measurement this
        # run. Criteria 4/5's fitted slopes/residuals are RESTATED from
        # tickets 126-003/004 (re-running those n=18/n=24 sweeps is
        # expensive and out of this ticket's scope) -- only their live
        # tovez.json value checks are fresh. Criterion 6 is derived
        # entirely from 4/5's restated numbers. Everything else in this
        # run (1, 2a, 2b, 3, 7) is a fresh measurement taken THIS session.
        provenance = {
            "1_presence": "FRESH -- measured this run",
            "2a_units_liveness": "FRESH -- measured this run",
            "2b_tour_liveness": "FRESH -- measured this run",
            "3_frame": "FRESH -- measured this run",
            "4_distance_cal": "RESTATED fit (126-003, n=18); live tovez.json value check is FRESH",
            "5_heading_cal": "RESTATED fit (126-004, n=24); live tovez.json value check is FRESH",
            "6_residuals_stated": "RESTATED -- derived from criteria 4/5's restated fits",
            "7_sim_and_fusion": "FRESH -- pytest run + live tovez.json grep, this run",
        }
        allPass = True
        for key, label in labels.items():
            ok = results.get(key, False)
            allPass = allPass and ok
            print(f"  [{'PASS' if ok else 'FAIL'}] {label}")
            print(f"        provenance: {provenance[key]}")
        print(f"\nchart: {chartPath}")
        print(f"\n{'ALL SEVEN CRITERIA PASS' if allPass else 'ONE OR MORE CRITERIA FAILED'}")
        return 0 if allPass else 1
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
# CLI
# ---------------------------------------------------------------------------


def buildArgParser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=DEFAULT_PORT,
                   help="the robot's own serial port (mbdeploy list's NEZHA2 row), "
                        "not the radio relay")
    p.add_argument("--mode", required=True,
                   choices=("units", "lever-arm", "distance", "heading", "acceptance"),
                   help="which ticket's measurement to run ('acceptance' runs all seven "
                        "of the issue's acceptance criteria, ticket 126-006)")
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
    p.add_argument("--chart", default=None,
                   help="output PNG path, --mode acceptance (default: "
                        "src/tests/bench/otos_calibration_bench_acceptance.png)")
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
        if args.mode == "heading":
            return runHeadingMode(args)
        return runAcceptanceMode(args)
    except GeofenceViolation as exc:
        print(f"ABORTED: {exc}")
        print("motors stopped by the geofence")
        return 2


if __name__ == "__main__":
    sys.exit(main())
