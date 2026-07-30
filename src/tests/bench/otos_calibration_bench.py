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

Both modes are hardware/playfield-only (no ``--sim``) -- there is nothing to
unit-test about a camera-vs-sensor comparison (see the sprint's Test
Strategy). Every connection path calls ``estop()`` (never ``stop()``) in a
``finally`` block, per ``.claude/rules/playfield-testing.md``.

Usage:
    uv run python src/tests/bench/otos_calibration_bench.py --port /dev/cu.usbmodem21141112 --mode units
    uv run python src/tests/bench/otos_calibration_bench.py --port /dev/cu.usbmodem21141112 --mode lever-arm
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


def loadConfiguredOffsetMm(robotJsonOverride: "str | None"
                           ) -> "tuple[float, float, str]":  # [mm] [mm]
    """The configured odometry_offset_mm lever arm (x, y) -- read from the
    ACTIVE robot config (data/robots/active_robot.json's pointer), the same
    resolution square_tour.py's HardwareBackend uses, unless overridden.
    Returns (x, y, path-used) so the caller can print provenance."""
    from robot_radio.config.robot_config import load_robot_config

    path: "pathlib.Path | str | None" = robotJsonOverride
    if path is None:
        pointer = _REPO_ROOT / "data" / "robots" / "active_robot.json"
        if pointer.exists():
            spec = json.loads(pointer.read_text())
            if "path" in spec:
                path = _REPO_ROOT / spec["path"]
    if path is None:
        path = _REPO_ROOT / "data" / "robots" / "tovez.json"
    cfg = load_robot_config(path)
    offset = cfg.geometry.odometry_offset_mm
    return offset.x, offset.y, str(path)


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


def pickSafeLegDirection(camPose: "tuple[float, float, float]", legMm: float,
                         marginCm: float) -> "float | None":  # [mm] [cm]
    """Choose the sign of v_x (forward=+1 / backward=-1) for a legMm straight
    move that keeps the robot at least marginCm from the field edge, given
    its current camera-measured pose. Returns None if neither direction is
    safe (caller must not move)."""
    x, y, yaw = camPose
    dxCm = (legMm / 10.0) * math.cos(yaw)
    dyCm = (legMm / 10.0) * math.sin(yaw)

    def safe(sign: float) -> bool:
        nx, ny = x + sign * dxCm, y + sign * dyCm
        return (abs(nx) <= Geofence.HALF_W - marginCm
                and abs(ny) <= Geofence.HALF_H - marginCm)

    if safe(1.0):
        return 1.0
    if safe(-1.0):
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
# CLI
# ---------------------------------------------------------------------------


def buildArgParser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=DEFAULT_PORT,
                   help="the robot's own serial port (mbdeploy list's NEZHA2 row), "
                        "not the radio relay")
    p.add_argument("--mode", required=True, choices=("units", "lever-arm"),
                   help="which ticket's measurement to run")
    p.add_argument("--leg", type=float, default=LEG,  # [mm]
                   help="straight-leg distance for --mode units")
    p.add_argument("--cruise", type=float, default=CRUISE,  # [mm/s]
                   help="straight-leg speed for --mode units")
    p.add_argument("--geofence-margin", type=float, default=FIELD_SAFE_MARGIN_CM,  # [cm]
                   help="stay this far from the field edge")
    p.add_argument("--robot-json", default=None,
                   help="override the robot config read for the configured "
                        "odometry_offset_mm (default: the active robot, "
                        "data/robots/active_robot.json)")
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
        return runLeverArmMode(args)
    except GeofenceViolation as exc:
        print(f"ABORTED: {exc}")
        print("motors stopped by the geofence")
        return 2


if __name__ == "__main__":
    sys.exit(main())
