#!/usr/bin/env python3
"""wheel_frozen_bench.py — bench verification of the 129-002 wheel-frozen
fault flag (`telemetry.h`'s `kFlagFaultWheelFrozenLeft`/`Right`, bits
19/20; decoded host-side as `TLMFrame.fault_wheel_frozen_left`/
`fault_wheel_frozen_right`, `robot_radio.robot.protocol.wheel_frozen_reason()`).

129-002's acceptance criteria require two bench checks against real
hardware (NOT the TestGUI — this script is the "short bench script"
the ticket calls for):

  1. **Positive case** (`--stall-wheel left|right`): commands ONLY the
     named wheel to a nonzero target speed for an extended window. This
     is a GATED, motion-qualified detector
     (`Hardware::MotorArmor::wedgeSuspect()`) — it only latches if the
     wheel is commanded to move AND its encoder genuinely does not
     advance for `kWedgeThreshold` (10) consecutive ~50 ms cycles
     (~0.5 s). On the stand the wheels spin FREE (no ground contact, no
     robot weight) — closed-loop velocity control means the PID will
     keep raising duty until the wheel actually turns, so this mode can
     only produce a real, sustained "commanded but frozen" reading if
     something is physically resisting the named wheel while this
     script runs (an operator's hand, gloved, clear of the drivetrain
     pinch points per `.claude/rules/hardware-bench-testing.md`'s safety
     note) — this script has no way to apply that resistance itself. Run
     it, then physically hold the named wheel stationary a moment after
     the move starts; the script reports when (if) the flag sets and how
     long that took, and does NOT claim a stall occurred if the flag
     never sets.

  2. **Negative case** (default, no `--stall-wheel`): drives a full
     `--leg-mm` (default 700 mm) straight leg at speed and confirms
     NEITHER wheel-frozen flag is ever observed. This is the
     higher-priority check per the ticket's own acceptance text — "a
     false positive here is worse than no flag at all" — and needs no
     physical intervention.

Robot is mounted on a stand with wheels off the ground (see
`.claude/rules/hardware-bench-testing.md`), so it is safe to spin freely.

Usage:
    uv run python src/tests/bench/wheel_frozen_bench.py --port /dev/cu.usbmodem2121102
    uv run python src/tests/bench/wheel_frozen_bench.py --port ... --stall-wheel left
"""
from __future__ import annotations

import argparse
import sys
import time

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol, wheel_frozen_reason

DEFAULT_PORT = "/dev/cu.usbmodem2121102"
ACK_TIMEOUT = 500  # [ms] wait_for_ack() bound for each command's ack


def _args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=DEFAULT_PORT)
    p.add_argument("--leg-mm", type=float, default=700.0,  # [mm]
                   help="negative-case straight-leg distance")
    p.add_argument("--speed", type=float, default=200.0,  # [mm/s]
                   help="per-wheel commanded speed")
    p.add_argument("--stall-wheel", choices=["left", "right"], default=None,
                   help="positive case: command only this wheel and watch "
                        "for its fault_wheel_frozen_* flag; requires an "
                        "operator to physically resist the wheel by hand "
                        "for the flag to have any chance of setting")
    p.add_argument("--stall-duration", type=float, default=3000.0,  # [ms]
                   help="positive-case commanded window")
    return p.parse_args()


class Result:
    def __init__(self) -> None:
        self.checks: list[tuple[str, bool, str]] = []

    def record(self, name: str, ok: bool, detail: str = "") -> None:
        self.checks.append((name, ok, detail))
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))

    def ok(self) -> bool:
        passed = sum(1 for _, k, _ in self.checks if k)
        print(f"\n==== {passed}/{len(self.checks)} checks passed ====")
        return passed == len(self.checks)


def _drain_and_seed(proto: NezhaProtocol) -> None:
    """Discard stale queued frames, then force one fresh frame so callers
    have a real baseline (mirrors twist_drive.py's own seeding, needed
    because a PARKED robot in kAuto mode is correctly silent otherwise —
    telemetry-emit-policy-rebuild-spec.md Part 3)."""
    proto.read_pending_binary_tlm_frames()
    proto.tlmNow()
    deadline = time.monotonic() + 0.5
    while time.monotonic() < deadline:
        if proto.read_pending_binary_tlm_frames():
            return
        time.sleep(0.02)


def run_negative_case(proto: NezhaProtocol, result: Result, leg_mm: float, speed: float) -> None:
    """Drive a full healthy leg; confirm neither wheel-frozen flag is ever
    observed while it runs."""
    _drain_and_seed(proto)

    timeout = (leg_mm / speed) * 1000.0 + 2000.0  # [ms] generous margin
    corr_id = proto.move_wheels(v_left=speed, v_right=speed,
                                stop_distance=leg_mm, timeout=timeout, replace=True)
    ack = proto.wait_for_ack(corr_id, timeout=ACK_TIMEOUT)
    result.record("negative case: leg move() ack confirmed", ack is not None and ack.ok,
                   f"ack={ack}")

    saw_left = False
    saw_right = False
    frames_seen = 0
    enc_first = None
    enc_last = None
    deadline = time.monotonic() + (timeout / 1000.0) + 1.0
    active_seen = False
    while time.monotonic() < deadline:
        for frame in proto.read_pending_binary_tlm_frames():
            frames_seen += 1
            if frame.enc is not None:
                if enc_first is None:
                    enc_first = frame.enc
                enc_last = frame.enc
            if frame.active:
                active_seen = True
            reason = wheel_frozen_reason(frame)
            if reason is not None:
                print(f"  !! wheel_frozen_reason={reason} at t={frame.t}")
            saw_left = saw_left or bool(frame.fault_wheel_frozen_left)
            saw_right = saw_right or bool(frame.fault_wheel_frozen_right)
        if active_seen and not proto_active_or_recent(proto):
            pass  # completion is polled via the ack ring in the real tour runner;
                   # this bench script just watches the clock + flags directly.
        time.sleep(0.02)

    result.record("negative case: motion observed (active flag seen)", active_seen)
    result.record("negative case: encoders advanced",
                   enc_first is not None and enc_last is not None and enc_last != enc_first,
                   f"enc_first={enc_first} enc_last={enc_last}")
    result.record("negative case: fault_wheel_frozen_left never set", not saw_left)
    result.record("negative case: fault_wheel_frozen_right never set", not saw_right)
    print(f"  ({frames_seen} telemetry frames observed)")


def proto_active_or_recent(_proto: NezhaProtocol) -> bool:
    # Placeholder hook kept simple: this script polls on a wall-clock
    # deadline sized to the move's own timeout rather than tracking
    # per-frame completion acks — sufficient for a flag-observation bench
    # check, unlike planner.tour's own real completion-event polling.
    return False


def run_positive_case(proto: NezhaProtocol, result: Result, wheel: str,
                       speed: float, duration: float) -> None:
    """Command only the named wheel; report when (if) its fault_wheel_frozen_*
    flag sets. Does NOT itself apply any resistance to the wheel -- see this
    file's own module docstring."""
    _drain_and_seed(proto)

    v_left = speed if wheel == "left" else 0.0
    v_right = speed if wheel == "right" else 0.0
    start = time.monotonic()
    corr_id = proto.move_wheels(v_left=v_left, v_right=v_right,
                                stop_time=duration, timeout=duration + 1000.0, replace=True)
    ack = proto.wait_for_ack(corr_id, timeout=ACK_TIMEOUT)
    result.record("positive case: move() ack confirmed", ack is not None and ack.ok,
                   f"ack={ack}")

    print(f"  >> commanding {wheel} wheel at {speed} mm/s for {duration:.0f} ms -- "
          f"physically hold that wheel now if verifying the stall path")

    set_at = None
    deadline = start + (duration / 1000.0) + 1.0
    while time.monotonic() < deadline:
        for frame in proto.read_pending_binary_tlm_frames():
            reason = wheel_frozen_reason(frame)
            flagged = (frame.fault_wheel_frozen_left if wheel == "left"
                       else frame.fault_wheel_frozen_right)
            if flagged and set_at is None:
                set_at = time.monotonic() - start
                print(f"  !! {wheel} wheel-frozen flag SET at t+{set_at:.2f}s "
                      f"(wheel_frozen_reason={reason})")
        time.sleep(0.02)

    if set_at is not None:
        result.record(f"positive case: {wheel} wheel-frozen flag set within ~0.5s",
                       set_at <= 1.0, f"set_at={set_at:.2f}s")
    else:
        print(f"  ({wheel} wheel-frozen flag never set during this {duration:.0f} ms window -- "
              "either the wheel was never actually held stationary, or the gate did not "
              "latch; this script cannot apply physical resistance itself, only observe)")
        result.record(f"positive case: {wheel} wheel-frozen flag observed", False,
                       "not set -- see console note above")


def main() -> int:
    args = _args()
    result = Result()

    conn = SerialConnection(port=args.port)
    info = conn.connect()
    if info.get("status") != "connected":
        print(f"ERROR: connect failed: {info}")
        return 2
    proto = NezhaProtocol(conn)
    result.record("connect()", True, f"mode={info.get('mode')}")

    try:
        if args.stall_wheel is not None:
            run_positive_case(proto, result, args.stall_wheel, args.speed, args.stall_duration)
        else:
            run_negative_case(proto, result, args.leg_mm, args.speed)
    finally:
        try:
            proto.estop()
        except Exception:
            pass
        conn.disconnect()

    return 0 if result.ok() else 1


if __name__ == "__main__":
    sys.exit(main())
