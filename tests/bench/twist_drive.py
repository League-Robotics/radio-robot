#!/usr/bin/env python3
"""twist_drive.py — bench verification of the P4 minimal host slice
(103-009): `NezhaProtocol.twist()`/`stop()` + the ack-ring matcher
(`wait_for_ack()`), against the real single-loop firmware.

Robot is mounted on a stand with the wheels off the ground (see
`.claude/rules/hardware-bench-testing.md`), so it is safe to spin the wheels
freely.

What this script proves, in order:
  1. `connect()` — the real serial/relay link comes up (HELLO/PING banner
     classification, still text-plane, unaffected by the P4 binary prune).
  2. `twist()` sends a `CommandEnvelope{twist: Twist{v_x, omega, duration}}`
     and gets its outcome confirmed via `wait_for_ack()` — the ack rides
     the next `Telemetry` push's ack ring, NOT a synchronous per-command
     reply (Decision 2's "telemetry-only return path"; see `protocol.py`'s
     `NezhaProtocol.twist()`/`wait_for_ack()` docstrings for why).
  3. Telemetry pushes show the encoders actually moving while the twist's
     `duration` window is armed — the real point of a bench gate: not just
     "the ack came back", but "the wheels actually turned".
  4. `stop()` halts the drivetrain and its own ack is confirmed the same
     way.

No "arm telemetry" step: the P4 firmware pushes `Telemetry` UNCONDITIONALLY
at all times (~25 Hz primary / 5 Hz secondary) — there is no `STREAM` verb
left to arm (that whole arm was pruned at 103-001/103-003). This script
just drains whatever the firmware is already pushing via
`read_pending_binary_tlm_frames()`.

Usage:
    uv run python tests/bench/twist_drive.py
    uv run python tests/bench/twist_drive.py --port /dev/cu.usbmodem2121102
    uv run python tests/bench/twist_drive.py --v-x 150 --omega 0 --duration 800
"""
from __future__ import annotations

import argparse
import sys
import time

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol

DEFAULT_PORT = "/dev/cu.usbmodem2121102"
ACK_TIMEOUT = 500  # [ms] wait_for_ack() bound for each command's ack


def _args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=DEFAULT_PORT)
    p.add_argument("--v-x", type=float, default=150.0,  # [mm/s]
                   help="body-frame forward velocity")
    p.add_argument("--omega", type=float, default=0.0,  # [rad/s]
                   help="body-frame yaw rate")
    p.add_argument("--duration", type=float, default=800.0,  # [ms]
                   help="deadman arm window for the twist command")
    p.add_argument("--watch", type=float, default=0.6,  # [s]
                   help="how long to watch telemetry for encoder movement "
                        "after the twist's ack lands")
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


def main() -> int:
    args = _args()
    result = Result()

    conn = SerialConnection(port=args.port)   # mode=None -> auto-detect direct vs relay
    info = conn.connect()
    if info.get("status") != "connected":
        print(f"ERROR: connect failed: {info}")
        return 2
    proto = NezhaProtocol(conn)
    result.record("connect()", True, f"mode={info.get('mode')}")

    try:
        # Drain any frames queued before this run started so the "encoders
        # moving" watch window below only sees fresh pushes.
        proto.read_pending_binary_tlm_frames()

        enc_before = None
        for frame in proto.read_pending_binary_tlm_frames():
            if frame.enc is not None:
                enc_before = frame.enc
        if enc_before is None:
            # No frame arrived in the drain above (telemetry is push-only,
            # not request/reply) — give the firmware one push cycle.
            time.sleep(0.1)
            for frame in proto.read_pending_binary_tlm_frames():
                if frame.enc is not None:
                    enc_before = frame.enc

        # --- twist() -----------------------------------------------------
        corr_id = proto.twist(v_x=args.v_x, omega=args.omega, duration=args.duration)
        result.record("twist() returns a corr_id", corr_id != 0, f"corr_id={corr_id}")

        ack = proto.wait_for_ack(corr_id, timeout=ACK_TIMEOUT)
        result.record("twist() ack confirmed via ack ring", ack is not None and ack.ok,
                       f"ack={ack}")

        # --- watch telemetry for encoder movement -------------------------
        deadline = time.monotonic() + args.watch
        enc_after = enc_before
        moved = False
        while time.monotonic() < deadline:
            for frame in proto.read_pending_binary_tlm_frames():
                if frame.enc is not None:
                    enc_after = frame.enc
                    if enc_before is not None and enc_after != enc_before:
                        moved = True
            time.sleep(0.02)
        result.record("encoders moving during twist()", moved,
                       f"before={enc_before} after={enc_after}")

        # --- stop() --------------------------------------------------------
        stop_corr_id = proto.stop()
        result.record("stop() returns a corr_id", stop_corr_id != 0,
                       f"corr_id={stop_corr_id}")

        stop_ack = proto.wait_for_ack(stop_corr_id, timeout=ACK_TIMEOUT)
        result.record("stop() ack confirmed via ack ring",
                       stop_ack is not None and stop_ack.ok, f"ack={stop_ack}")
    finally:
        # Guaranteed stop: motors must never be left running on an
        # exception or Ctrl-C, even if a check above already stopped them.
        try:
            proto.stop()
        except Exception:
            pass
        conn.disconnect()

    return 0 if result.ok() else 1


if __name__ == "__main__":
    sys.exit(main())
