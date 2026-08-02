#!/usr/bin/env python3
"""hil_drive.py -- hardware-in-the-loop drive of the REAL robot through the
REAL Motion::Planner (libmotionplanner via ctypes).

The planner runs HERE on the host, exactly as designed: each control
interval a RobotState is built from the robot's own telemetry (encoder
position/velocity with their own sample timestamps, robot-clock time
base), plannerTick() computes, plannerUpdate() stages the wheel velocity
targets, and those targets bridge to the robot as short time-bounded
``move_wheels(replace=True)`` reissues (the same reissue idiom
move_soak.py uses; the bounded stop_time means host silence stops the
robot on its own). Completion comes from the planner's TickResult, and
accuracy is judged by the robot's OWN encoders against the commanded
targets.

Scenarios: a distance leg, a rotation, then a distance->rotation->distance
chain. HITL: robot on the stand, wheels free.

    uv run python src/tests/bench/hil_drive.py \
        --port /dev/cu.usbmodem2121102

130-007: the ``--duty`` mode (host-side closed loop through the planner's
M4 PID, firmware PID reduced to pure kff via CONFIG) is DROPPED outright --
``Motion::WheelPid``/``Planner::stageDuty()`` are deleted with the rest of
the parked duty stage (bench-duty-readers-see-zero-after-stageduty-park.md's
own suggested resolution: drop the duty-read mode rather than leave it
silently reading zero). It was already measured UNSTABLE at bench gains
2026-07-26 (the serial transport adds ~6 control cycles of dead time, and
the integrator winds across it -- runaway legs, sign reversals; see the
motion-planner issue's HIL findings) and never the default. The remaining
mode -- bridging velocity targets to the firmware's own PID -- is the
stable, known-good configuration and is now the ONLY mode.
"""

import argparse
import ctypes
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))  # 128-010: planner_harness.py is co-located
from planner_harness import (  # noqa: E402  (path bootstrap above)
    KIND_ANGLE, KIND_DISTANCE, VELOCITY_TWIST, Move, PlannerLimits,
    RobotState, TickResult, loadLibrary)

from robot_radio.io.serial_conn import SerialConnection  # noqa: E402
from robot_radio.robot.protocol import NezhaProtocol  # noqa: E402

PERIOD = 0.05          # [s] host control interval, mirrors the loop's own 50 ms
BRIDGE_HOLD = 300.0    # [ms] each reissued move_wheels' own stop_time backstop
BRIDGE_TIMEOUT = 1000.0  # [ms] its timeout backstop


def hilLimits() -> PlannerLimits:
    limits = PlannerLimits()
    limits.ceilings.vMax = 300.0          # [mm/s] gentle bench ceiling
    limits.ceilings.aMax = 200.0          # [mm/s^2]
    limits.ceilings.aDecel = 120.0   # measured: bridge braking authority ~4x weaker than sim plant        # [mm/s^2]
    limits.ceilings.omegaMax = 2.0        # [rad/s]
    limits.ceilings.alphaMax = 2.5        # [rad/s^2]
    limits.ceilings.alphaDecel = 1.2  # measured: single-turn trace, 76 deg overshoot at 5.0      # [rad/s^2]
    limits.plant.trackWidth = 128.0    # [mm] tovez.json drivetrain trackwidth
    limits.plant.controlPeriod = 50.0  # [ms]
    limits.plant.actuationDelay = 150.0  # [ms] velocity-mode value (run 2, known
                                         # good): transport + firmware PID plant lag
    limits.plant.velocityFilterWeight = 0.4  # real encoder velocity is noisy
    limits.tracking.headingHoldGain = 1.2  # [1/s] straight legs crabbed ~-10 deg uncorrected (run 1)
    # 130-009: requireSettle/settleWindow (the settle-confirm defer path,
    # already dissolved by 130-008) and otosStaleness/headingOtosWeight
    # (the OTOS heading blend, out of scope this sprint) are DELETED from
    # PlannerLimits outright -- there is nothing left here to set for
    # either. The M4 duty stage (velKff/velKp/velKi/velIMax's one-time
    # destination) is deleted the same way (130-007/130-009).
    return limits


class HilSession:
    def __init__(self, port: str):
        self.lib = loadLibrary()
        limits = hilLimits()
        self.planner = self.lib.plannerCreate(ctypes.byref(limits))
        self.conn = SerialConnection(port=port)
        info = self.conn.connect()
        if info.get("status") != "connected":
            raise ConnectionError(f"connect failed: {info}")
        self.proto = NezhaProtocol(self.conn)
        self.state = RobotState()
        self.lastFrame = None

    def close(self):
        try:
            self.proto.estop()
        finally:
            self.conn.disconnect()
            self.lib.plannerDestroy(self.planner)

    def ingestTelemetry(self) -> bool:
        """Fold the newest telemetry frame into the RobotState. Returns
        True when a frame with encoder readings was seen."""
        frames = self.proto.read_pending_binary_tlm_frames()
        for f in frames:
            if f.enc_left is None or f.t is None:
                continue
            self.lastFrame = f
        f = self.lastFrame
        if f is None:
            return False
        self.state.time.cycleStart = int(f.t) & 0xFFFFFFFF
        for wheel, reading in ((self.state.wheelLeft, f.enc_left),
                               (self.state.wheelRight, f.enc_right)):
            wheel.position = reading.position
            wheel.velocity = reading.velocity
            wheel.sampleTime = (int(f.t) - int(reading.age)) & 0xFFFFFFFF
            wheel.connected = True
        if f.otos_reading is not None and f.otos_present:
            o = f.otos_reading
            self.state.otos.present = True
            self.state.otos.heading = o.heading
            self.state.otos.omega = o.omega
            self.state.otos.sampleTime = self.state.time.cycleStart
        else:
            self.state.otos.present = False
        return True

    def submit(self, move: Move) -> bool:
        return self.lib.plannerMove(self.planner, ctypes.byref(move), False)

    def runUntilComplete(self, label: str, maxSeconds: float,
                         stopAtEnd: bool = True) -> dict:
        """Drive the planner<->robot loop until the planner reports the
        active Move complete (or deadline). stopAtEnd=False is the
        mid-chain shape: return on the completion event itself WITHOUT
        stopping the robot, so a same-axis carry survives the hand-off
        (the planner keeps commanding through the boundary)."""
        result = TickResult()
        t0 = time.monotonic()
        ticks = 0
        completed = None
        encStart = None
        while time.monotonic() - t0 < maxSeconds:
            loopStart = time.monotonic()
            if self.ingestTelemetry():
                if encStart is None:
                    encStart = (self.state.wheelLeft.position,
                                self.state.wheelRight.position)
                self.lib.plannerTick(self.planner, ctypes.byref(self.state),
                                     ctypes.byref(result))
                self.lib.plannerUpdate(self.planner, ctypes.byref(self.state))
                ticks += 1
                vL = self.state.wheelLeft.cmdVelocity
                vR = self.state.wheelRight.cmdVelocity
                if abs(vL) > 0.5 or abs(vR) > 0.5:
                    self.proto.move_wheels(v_left=vL, v_right=vR,
                                           stop_time=BRIDGE_HOLD,
                                           timeout=BRIDGE_TIMEOUT,
                                           replace=True)
                elif stopAtEnd:
                    self.proto.estop()
                if result.completed and completed is None:
                    completed = dict(moveId=result.moveId,
                                     timedOut=result.timedOut,
                                     settled=result.settled,
                                     atTick=ticks)
                    if not stopAtEnd:
                        break  # hand-off: caller reports; planner carries on
                if completed is not None and abs(vL) < 0.5 and abs(vR) < 0.5:
                    break
            elapsed = time.monotonic() - loopStart
            time.sleep(max(0.0, PERIOD - elapsed))
        if stopAtEnd:
            self.proto.estop()
            time.sleep(0.3)
            self.ingestTelemetry()
        encEnd = (self.state.wheelLeft.position,
                  self.state.wheelRight.position)
        stats = dict(label=label, ticks=ticks, completed=completed,
                     encStart=encStart, encEnd=encEnd)
        return stats


def distanceMove(moveId: int, threshold: float, v_x: float) -> Move:
    return Move(id=moveId, kind=KIND_DISTANCE, threshold=threshold,
                timeout=30000.0, velocityKind=VELOCITY_TWIST, v_x=v_x)


def angleMove(moveId: int, threshold: float, omega: float) -> Move:
    return Move(id=moveId, kind=KIND_ANGLE, threshold=threshold,
                timeout=30000.0, velocityKind=VELOCITY_TWIST, omega=omega)


def report(stats: dict, expectLeft: float, expectRight: float,
           track: float) -> None:
    dL = stats["encEnd"][0] - stats["encStart"][0]
    dR = stats["encEnd"][1] - stats["encStart"][1]
    path = 0.5 * (dL + dR)
    headingDeg = math.degrees((dR - dL) / track)
    done = stats["completed"]
    doneTxt = ("timedOut" if done and done["timedOut"] else
               f"completed@tick{done['atTick']}" if done else "NO COMPLETION")
    print(f"  {stats['label']}: {doneTxt}, settled={done and done['settled']}")
    print(f"    encoders dL={dL:+.1f} dR={dR:+.1f} mm -> "
          f"path={path:+.1f} mm, heading={headingDeg:+.2f} deg "
          f"(expected dL={expectLeft:+.0f} dR={expectRight:+.0f})")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    args = parser.parse_args()

    session = HilSession(args.port)
    track = 128.0
    quarter = math.pi / 2
    try:
        print("=== HIL: real robot driven by Motion::Planner (host) ===")

        print("-- scenario 1: distance 400 mm @ 150 mm/s")
        assert session.submit(distanceMove(1, 400.0, 150.0))
        s = session.runUntilComplete("distance-400", 15.0)
        report(s, 400.0, 400.0, track)

        print("-- scenario 2: rotate +90 deg @ 1.5 rad/s")
        assert session.submit(angleMove(2, quarter, 1.5))
        s = session.runUntilComplete("turn-90", 15.0)
        wheel = quarter * track / 2.0
        report(s, -wheel, +wheel, track)

        print("-- scenario 3: chain 300 mm -> +90 deg -> 300 mm")
        assert session.submit(distanceMove(3, 300.0, 150.0))
        assert session.submit(angleMove(4, quarter, 1.5))
        assert session.submit(distanceMove(5, 300.0, 150.0))
        s = session.runUntilComplete("chain-leg1", 12.0, stopAtEnd=False)
        report(s, 300.0, 300.0, track)
        s = session.runUntilComplete("chain-leg2", 12.0, stopAtEnd=False)
        report(s, -wheel, +wheel, track)
        s = session.runUntilComplete("chain-leg3", 12.0)
        report(s, 300.0, 300.0, track)

        print("=== HIL run complete ===")
    finally:
        session.close()


if __name__ == "__main__":
    main()
