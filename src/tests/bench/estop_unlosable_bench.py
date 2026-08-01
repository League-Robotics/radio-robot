#!/usr/bin/env python3
"""estop_unlosable_bench.py -- 129-001 (issue 07, the 2026-07-31 runaway)
bench acceptance: `estop()` must stop the wheels EVERY time, consecutively,
without a power cycle.

Root cause this proves fixed: `Devices::NezhaMotor::writeRawDuty()` used to
suppress a duty write equal to the last one it ATTEMPTED
(`pct == lastWrittenPct_`), but the Nezha brick physically LATCHES its last
commanded speed and does not reset on an nRF52 reset -- only power does. One
lost zero write was permanent: the host believed the stop landed, the wheel
kept spinning, and every LATER `estop()` was suppressed as a no-op because
`lastWrittenPct_` already claimed 0. This is why the ticket's own acceptance
requires 10 CONSECUTIVE trials, not one -- a single pass proves nothing
(exactly the shape of the 2026-07-31 incident: 13 estops, one WHEELS(0,0),
and a reset all failed).

Per trial:
  1. `move_wheels(150, 150, stop_time=<long>, timeout=<long>)` -- drive both
     wheels at 150 mm/s for a window long enough that `estop()` below always
     lands mid-leg, never after the Move's own natural completion.
  2. Wait until telemetry confirms the wheels are actually moving (encoders
     climbing, `flags` bit 2 / `frame.active` set).
  3. `estop()`, timestamped at the moment the wire write happens (NOT the
     ack -- the ack rides the NEXT telemetry push, and 129-001's fix is
     about the wheels actually stopping, not about ack latency).
  4. Poll telemetry for `--settle-window` (default 1.0s) tracking the last
     tick where either encoder still changed -- that must fall within
     `--stop-bound` (0.15s, the ticket's own acceptance number) of the
     estop() call.
  5. Continue polling for the REMAINDER of a full 3s hold (the ticket's own
     "stay stopped for 3s" number) with nothing else commanding, confirming
     NEITHER encoder changes again.

Ten trials run back-to-back on the SAME connection, SAME robot boot -- no
reconnect, no power cycle -- which is the whole point: the defect needed a
LOST write to manifest, and only a lost write on some EARLIER trial could
have suppressed a LATER trial's stop.

Usage:
    uv run python src/tests/bench/estop_unlosable_bench.py
    uv run python src/tests/bench/estop_unlosable_bench.py --port /dev/cu.usbmodem2121102
    uv run python src/tests/bench/estop_unlosable_bench.py --trials 10
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
    p.add_argument("--trials", type=int, default=10)
    p.add_argument("--speed", type=float, default=150.0,  # [mm/s]
                   help="per-wheel commanded speed")
    p.add_argument("--leg-duration", type=float, default=2000.0,  # [ms]
                   help="Move's own time stop condition -- long enough that "
                        "estop() always lands mid-leg")
    p.add_argument("--mid-leg-delay", type=float, default=0.4,  # [s]
                   help="how long to drive before estop(), well inside "
                        "--leg-duration")
    p.add_argument("--stop-bound", type=float, default=0.15,  # [s]
                   help="ticket acceptance: encoders must stop advancing "
                        "within this long of estop()")
    p.add_argument("--hold", type=float, default=3.0,  # [s]
                   help="ticket acceptance: encoders must stay stopped this "
                        "long after estop(), with nothing else commanding")
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


def _drain(proto: NezhaProtocol, into: list) -> None:
    into.extend(proto.read_pending_binary_tlm_frames())


def _wait_for_baseline_enc(proto: NezhaProtocol, deadline: float):
    """Seed enc via a forced frame (bare TLM, honored in every mode -- see
    twist_drive.py's own note on why a parked robot needs this rather than
    passively waiting for an ambient push)."""
    proto.tlmNow()
    enc = None
    while enc is None and time.monotonic() < deadline:
        for frame in proto.read_pending_binary_tlm_frames():
            if frame.enc is not None:
                enc = frame.enc
        if enc is None:
            time.sleep(0.02)
    return enc


def run_trial(proto: NezhaProtocol, trial: int, args: argparse.Namespace, result: Result) -> None:
    label = f"trial {trial}"

    # Fresh baseline -- drain stale frames, then force one so enc_before is
    # never left at a pre-move value.
    _drain(proto, [])
    enc_before = _wait_for_baseline_enc(proto, time.monotonic() + 0.5)
    result.record(f"{label}: baseline enc seeded", enc_before is not None,
                  f"enc_before={enc_before}")

    timeout = args.leg_duration + 1000.0  # [ms] generous backstop past the leg itself
    corr_id = proto.move_wheels(args.speed, args.speed,
                                stop_time=args.leg_duration, timeout=timeout,
                                replace=True)
    move_ack = proto.wait_for_ack(corr_id, timeout=ACK_TIMEOUT)
    result.record(f"{label}: move_wheels() ack confirmed",
                  move_ack is not None and move_ack.ok, f"ack={move_ack}")

    # Confirm the wheels are ACTUALLY moving before estop()ing -- an
    # estop() that "succeeds" against a wheel that was never driven proves
    # nothing about the write-on-change defect.
    moving = False
    enc_moving = enc_before
    deadline = time.monotonic() + args.mid_leg_delay
    while time.monotonic() < deadline:
        for frame in proto.read_pending_binary_tlm_frames():
            if frame.enc is not None:
                enc_moving = frame.enc
                if enc_before is not None and enc_moving != enc_before:
                    moving = True
        time.sleep(0.02)
    result.record(f"{label}: wheels genuinely moving before estop()", moving,
                  f"enc_before={enc_before} enc_at_estop={enc_moving}")

    # --- The moment of truth: estop() mid-leg. -----------------------------
    estop_sent_at = time.monotonic()
    stop_corr_id = proto.estop()
    stop_ack = proto.wait_for_ack(stop_corr_id, timeout=ACK_TIMEOUT)
    result.record(f"{label}: estop() ack confirmed",
                  stop_ack is not None and stop_ack.ok, f"ack={stop_ack}")

    # Track the last tick either encoder still changed, across the full
    # hold window (settle bound + the remaining hold), and confirm no
    # further change after that.
    last_enc = enc_moving
    last_change_at = estop_sent_at
    hold_deadline = estop_sent_at + args.hold
    # Record EVERY observed change with its own relative timestamp -- NOT
    # just the latest one -- so "stopped quickly" and "stayed stopped" can
    # be checked as two INDEPENDENT facts below rather than one collapsing
    # into a restatement of the other (a change seen only once, early, must
    # make both checks pass; a change seen again late must fail the second
    # check regardless of how fast the first one was).
    changes: list[tuple[float, tuple[int, int]]] = []
    while time.monotonic() < hold_deadline:
        for frame in proto.read_pending_binary_tlm_frames():
            if frame.enc is None:
                continue
            if frame.enc != last_enc:
                t_rel = (frame.recvTime if frame.recvTime is not None else time.monotonic()) - estop_sent_at
                changes.append((t_rel, frame.enc))
                last_enc = frame.enc
        time.sleep(0.02)

    settle_time = changes[-1][0] if changes else 0.0  # [s] time of the LAST observed change
    result.record(f"{label}: encoders stop advancing within {args.stop_bound}s",
                  settle_time <= args.stop_bound,
                  f"settle_time={settle_time:.3f}s enc_final={last_enc}")

    # "Stays stopped" is checked against a GENEROUS, fixed coast-down grace
    # (double the stop-bound, or at least 0.3s) -- independent of how fast
    # the initial coast actually was above -- so a slow-but-honest coast
    # doesn't make this check spuriously fail, while a LATER relapse (the
    # original defect's own failure shape: an estop that "took" briefly
    # then let the wheel resume) still trips it.
    coast_grace = max(2.0 * args.stop_bound, 0.3)  # [s]
    relapses = [(t, enc) for (t, enc) in changes if t > coast_grace]
    result.record(f"{label}: encoders stay stopped for the full {args.hold}s hold "
                  f"(no change after the {coast_grace:.2f}s coast-down grace)",
                  len(relapses) == 0,
                  f"relapses={relapses}" if relapses else f"held from {settle_time:.3f}s to {args.hold}s")


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
        for trial in range(1, args.trials + 1):
            print(f"\n--- trial {trial}/{args.trials} ---")
            run_trial(proto, trial, args, result)
    finally:
        # Guaranteed stop: motors must never be left running on an
        # exception or Ctrl-C, even if a check above already stopped them.
        try:
            proto.estop()
        except Exception:
            pass
        conn.disconnect()

    return 0 if result.ok() else 1


if __name__ == "__main__":
    sys.exit(main())
