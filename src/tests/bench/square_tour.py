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

_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]

TRACK = 128.0          # [mm] PHYSICAL wheel separation (caliper-measured)

# The track this script must use for ANGLE math is the EFFECTIVE one:
# trackwidth / rotational_slip. A skid-steer robot scrubs its wheels
# sideways through a pivot and rotates LESS than ideal kinematics predict,
# and every angle here -- the commanded pivot arc, the self-calibration
# measurement, and the reported heading -- is wheel-difference over track.
#
# Using the physical 128 over-reported rotation by 1/slip (9.7% on tovez).
# That corrupted BOTH ends of the measurement: the pivot self-calibration
# read 93.6 deg for a true 85.3, then SHORTENED already-short turns by
# 0.962, and the final heading came out 383.6 for a true ~345. Set from the
# robot config at backend init so sim (slip 1.0 -> 128) is unchanged.
EFFECTIVE_TRACK = TRACK


def _set_effective_track(robot_config) -> None:
    """Derive EFFECTIVE_TRACK from the active robot config, once."""
    global EFFECTIVE_TRACK, TURN_ARC
    tw = robot_config.trackwidth or TRACK
    slip = getattr(robot_config.calibration, "rotational_slip", None) or 1.0
    EFFECTIVE_TRACK = tw / slip if slip > 0 else tw
    TURN_ARC = EFFECTIVE_TRACK * math.pi / 4.0
LEG = 500.0            # [mm]
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
        out.append((legL, legR, legMs, 9601 + 2 * i))
        out.append((turnL, turnR, turnMs, 9602 + 2 * i))
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
        from robot_radio.config.robot_config import load_robot_config
        from robot_radio.io.serial_conn import SerialConnection
        from robot_radio.robot.protocol import NezhaProtocol

        # The hardware backend previously read no robot config at all, so
        # every angle it computed used the physical 128 mm track -- see
        # EFFECTIVE_TRACK. That is the path that actually matters, since the
        # sim robot has no scrub.
        _set_effective_track(load_robot_config(robot_json))

        self._conn = SerialConnection(port=port)
        self._conn.connect()
        self.proto = NezhaProtocol(self._conn)
        print(f"connected; waiting {BOOT_WAIT:.0f}s for boot")
        time.sleep(BOOT_WAIT)
        self.proto.read_pending_binary_tlm_frames()  # discard the boot burst

    def advance(self, seconds: float) -> "list":
        frames = []
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            frames.extend(self.proto.read_pending_binary_tlm_frames())
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

    def calibrate(self) -> float:
        """Measure this run's per-wheel gain and turn-duration scale.
        Returns the turn scale; fills CAL as a side effect."""
        print("calibrating straight...")
        self.sendVerified(CRUISE, CRUISE, 3500.0, 9598)
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
        self.sendVerified(tL, tR, turnMsNominal, 9597)
        self.advance(turnMsNominal / 1000.0 + 1.5)
        if len(self.log["t"]) - base < 4:
            return 1.0
        dL = self.log["posL"][-1] - self.log["posL"][base]
        dR = self.log["posR"][-1] - self.log["posR"][base]
        degMeasured = abs(math.degrees((dR - dL) / EFFECTIVE_TRACK))
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
        turnScale = self.calibrate()
        segs = [(vL, vR, dur * (turnScale if idx % 2 == 1 else 1.0), mid)
                for idx, (vL, vR, dur, mid) in enumerate(segments())]
        self._tourStartIndex = len(self.log["t"])
        for idx, (vL, vR, durMs, moveId) in enumerate(segs):
            label = f"leg {idx // 2 + 1}" if idx % 2 == 0 else f"turn {idx // 2 + 1}"
            if not self.runSegment(label, vL, vR, durMs, moveId):
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
        dTheta = (dR - dL) / EFFECTIVE_TRACK
        heading += dTheta
        xs.append(xs[-1] + ds * math.cos(heading - dTheta / 2.0))
        ys.append(ys[-1] + ds * math.sin(heading - dTheta / 2.0))
    return xs, ys


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    backend_group = p.add_mutually_exclusive_group(required=True)
    backend_group.add_argument("--sim", action="store_true",
                               help="run against SimLoop (no hardware needed)")
    backend_group.add_argument("--port", nargs="?", const=DEFAULT_PORT,
                               help="run against a real robot on this serial port")
    p.add_argument("--robot-json", default=str(DEFAULT_ROBOT_JSON),
                   help="robot config the sim backend configures from")
    p.add_argument("--max-closure", type=float, default=DEFAULT_MAX_CLOSURE,
                   help="[mm] fail if the tour's end point is farther than this from its start")
    p.add_argument("--heading-tolerance", type=float, default=DEFAULT_HEADING_TOLERANCE,
                   help="[deg] fail if total heading change is farther than this from 360")
    p.add_argument("--chart", default=None,
                   help="output PNG path (default: src/tests/bench/square_tour_<backend>.png)")
    args = p.parse_args()

    backend: _Backend = (SimBackend(args.robot_json) if args.sim
                         else HardwareBackend(args.port, args.robot_json))
    tour = Tour(backend)
    try:
        completed = tour.run()
    finally:
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
    headingDeg = math.degrees((dTotR - dTotL) / EFFECTIVE_TRACK)
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
