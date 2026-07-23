#!/usr/bin/env python3
"""ack_ring_rapid_fire_bench.py -- ticket 120's own acceptance scenario for
the bounded ack ring (`Telemetry.acks`, depth 4,
bench-single-ack-slot-observability-collapses-at-40ms.md): fire N (>= 5,
the queue's own 5-deep `ERR_FULL` ceiling -- 1 active + 4 pending,
protocol-v4.md Sec 5.1) `move_twist()` enqueues BACK-TO-BACK, with no
inter-send wait at all, then confirm every single one's enqueue ack
surfaces somewhere in the captured telemetry stream's ack ring -- the
exact rapid-fire burst the pre-120 single ack slot lost (bench measurement:
`move_protocol_bench.py` was 31/43, every FAIL a missed transient ack).

Kept as its own script (not folded into `move_protocol_bench.py`) so that
file's own 43-check count stays exactly what the ticket's own before/after
framing (31/43 -> 43/43) refers to -- this is an ADDITIONAL acceptance
proof, not a replacement for any of those 43 checks.

Robot is mounted on a stand with the wheels off the ground (see
`.claude/rules/hardware-bench-testing.md`), so it is safe to spin the
wheels freely -- every Move here commands a small velocity (60 mm/s) with
a generous TIME stop condition, cleaned up by STOP at the end.

Usage:
    uv run python src/tests/bench/ack_ring_rapid_fire_bench.py
    uv run python src/tests/bench/ack_ring_rapid_fire_bench.py --port /dev/cu.usbmodem2121102
    uv run python src/tests/bench/ack_ring_rapid_fire_bench.py -n 8
"""
from __future__ import annotations

import argparse
import sys
import time

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import AckEntry, NezhaProtocol, TLMFrame

DEFAULT_PORT = "/dev/cu.usbmodem2121102"
DEFAULT_N = 5  # the MoveQueue's own 5-deep ERR_FULL ceiling (1 active + 4 pending)
CAPTURE_WINDOW_S = 1.5  # [s] how long to drain telemetry after firing all N enqueues


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


def _watch(proto: NezhaProtocol, duration: float) -> list[TLMFrame]:  # [s]
    """Drain telemetry for `duration` seconds, collecting every frame.
    Poll period well under the 40ms primary cycle so no frame waits a full
    poll behind -- every frame's own ack ring snapshot gets captured, not
    just whichever frame happens to be current at a coarser poll rate."""
    frames: list[TLMFrame] = []
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        frames.extend(proto.read_pending_binary_tlm_frames())
        time.sleep(0.01)
    return frames


def _find_ack_entry(frames: list[TLMFrame], corr_id: int) -> "AckEntry | None":
    """Scan every frame's bounded ack ring (`TLMFrame.acks`, 120) across
    `frames` for the first entry matching `corr_id` -- the UNION of every
    captured frame's own ring snapshot, not just the latest one, so an ack
    that was present in an EARLIER frame but has since been evicted by
    later pushes is still found (as long as it was captured before being
    evicted -- see this script's own module docstring)."""
    for f in frames:
        for entry in f.acks:
            if entry.corr_id == corr_id:
                return entry
    return None


def scenario_rapid_fire_n_enqueue(proto: NezhaProtocol, result: Result, n: int) -> None:
    """Fire `n` back-to-back MOVE enqueues (no wait between sends), then
    confirm every single one's enqueue ack is observable via the ack
    ring."""
    # Drain any stale frames queued before this run started.
    proto.read_pending_binary_tlm_frames()

    move_id_base = 9500
    corrs: list[int] = []
    send_started = time.monotonic()
    for i in range(n):
        # First enqueue is replace=True (starts the active Move); the rest
        # are replace=False (fill the pending queue) -- exactly the
        # ERR_FULL-ceiling shape at n=5. A small commanded velocity (safe
        # on the stand) with a generous TIME stop condition -- this
        # scenario is about ACK OBSERVABILITY, not motion accuracy.
        corr = proto.move_twist(
            v_x=60.0, v_y=0.0, omega=0.0,
            stop_time=5000.0, timeout=6000.0,
            replace=(i == 0), move_id=move_id_base + i,
        )
        corrs.append(corr)
    send_elapsed_ms = (time.monotonic() - send_started) * 1000.0
    print(f"  sent {n} back-to-back move_twist() enqueues in {send_elapsed_ms:.1f}ms "
          f"(corr_ids={corrs})")

    frames = _watch(proto, CAPTURE_WINDOW_S)
    print(f"  captured {len(frames)} telemetry frames over {CAPTURE_WINDOW_S}s")

    all_ok = True
    for i, corr in enumerate(corrs):
        entry = _find_ack_entry(frames, corr)
        ok = entry is not None and entry.ok
        all_ok = all_ok and ok
        result.record(f"rapid-fire: enqueue #{i + 1}/{n} (corr_id={corr}) ack observed via ring",
                      ok, f"entry={entry}")

    result.record(f"rapid-fire: ALL {n} back-to-back enqueue acks surfaced via the ack ring",
                  all_ok, f"n={n} send_window={send_elapsed_ms:.1f}ms")

    # Clean up -- STOP flushes the active Move and every pending slot.
    stop_corr = proto.stop()
    stop_ack = proto.wait_for_ack(stop_corr, timeout=500)
    result.record("rapid-fire: STOP cleanup ack ok", stop_ack is not None and stop_ack.ok,
                  f"ack={stop_ack}")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=DEFAULT_PORT)
    p.add_argument("-n", type=int, default=DEFAULT_N,
                    help=f"number of back-to-back enqueues (default {DEFAULT_N}, "
                         "the queue's own ERR_FULL ceiling)")
    args = p.parse_args()

    conn = SerialConnection(port=args.port)
    info = conn.connect()
    if info.get("status") != "connected":
        print(f"ERROR: connect failed: {info}")
        return 2
    proto = NezhaProtocol(conn)
    print(f"connected: port={args.port} mode={info.get('mode')}")

    result = Result()
    try:
        scenario_rapid_fire_n_enqueue(proto, result, args.n)
    finally:
        try:
            proto.stop()
        except Exception:
            pass
        conn.disconnect()

    return 0 if result.ok() else 1


if __name__ == "__main__":
    sys.exit(main())
