#!/usr/bin/env python3
"""tests/bench/relay_telemetry_rate.py — sprint 102 ticket 001 P0 spike.

Measures whether binary PUSH telemetry (armed via NezhaProtocol.stream(),
CommandEnvelope{stream: StreamControl{period, binary: true}}) survives
sustained delivery at a ~30 Hz target rate, direct over USB and through the
radio relay's `!GO` data plane, against CURRENT (pre-single-loop) firmware.
This is a read-only diagnostic: it arms/disarms telemetry only, never issues
a motion command, and leaves the robot on the stand with motors neutralized
throughout.

Uses the canonical SerialConnection + NezhaProtocol stack (never hand-rolled
lock-step pyserial) — relay handshake (!ECHO OFF / !MODE RAW250 / !GO) is
handled automatically by SerialConnection.connect() when it classifies the
port as a RADIOBRIDGE. The capture loop polls
NezhaProtocol.read_pending_binary_tlm_frames() (non-blocking drain) every
--poll-interval seconds rather than one long blocking read, because the
underlying queue is bounded (256 frames, drop-oldest) — see serial_conn.py's
_TLM_QUEUE_DEPTH.

Statistics computed from the received TLMFrame.seq (D10 uint16 sequence
counter, shared by STREAM/SNAP, wraps at 65535):
  - delivered frame count, wall-clock elapsed, frames/sec
  - drop rate (reusing protocol.tlm_drop_rate(), correct uint16-wrap math)
  - gap-run-length histogram + longest single gap
  - burst-vs-uniform classification: are missing frames concentrated in a
    few poll windows (burst, e.g. a relay stall) or spread one-or-two-per-
    many windows (uniform sparse loss, e.g. the single-slot RX reassembly
    buffer dropping a message here and there)?
  - malformed *B frame count: instrumented via a local (script-side only,
    no production file touched) wrap of SerialConnection._handle_binary_reply
    that re-attempts the same base64/protobuf decode it performs and counts
    decode exceptions BEFORE they are swallowed. This is the only way to
    observe them at all — a malformed frame is otherwise silently dropped
    inside the reader thread and would appear only as an ordinary seq gap.

Usage (ports are bench-specific — always confirm with `mbdeploy list`'s ROLE
column rather than trusting a stale example; the relay and robot enumerate as
separate /dev/cu.usbmodem* ports that can change across power cycles):
    uv run python tests/bench/relay_telemetry_rate.py \\
        --port /dev/cu.usbmodem2121102 --label direct-usb --duration 240

    uv run python tests/bench/relay_telemetry_rate.py \\
        --port /dev/cu.usbmodem2121302 --label relay --duration 240

Prints a human-readable report to stdout; --json-out additionally writes a
machine-readable summary.

Sprint 102 ticket 001 results (2026-07-14, robot v0.20260714.2): both
captures sustained ~26.8 fps at a 33 ms armed period over a 240 s window —
direct USB at 0.00% drop, the relay at 0.031% drop (2 isolated single-frame
gaps, uniform/sparse). See
clasi/sprints/102-single-loop-firmware-spikes-archive-and-delete-to-stub-p0-p2/spike-001-relay-telemetry.md
for the full writeup and
.clasi/knowledge/2026-06-12-relay-go-data-plane-and-docs.md for the
retraction of the "async STREAM frames dropped by the bridge" claim.
"""
from __future__ import annotations

import argparse
import base64
import json
import sys
import time
from collections import Counter
from dataclasses import asdict, dataclass, field

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.pb2 import envelope_pb2
from robot_radio.robot.protocol import NezhaProtocol, TLMFrame, tlm_drop_rate


def instrument_malformed_counter(conn: SerialConnection) -> dict:
    """Wrap ``conn._handle_binary_reply`` to count ``*B`` decode failures.

    Script-side only — does not modify serial_conn.py. Re-performs the same
    base64-decode + ReplyEnvelope.FromString() that the real handler does,
    purely to observe whether it would have raised, then always calls the
    real (unmodified) handler so behavior is unchanged. Returns a dict the
    caller can read live: {"malformed": <count>}.
    """
    original = conn._handle_binary_reply
    counts = {"malformed": 0}
    prefix = "*B"

    def wrapped(text: str) -> None:
        if text.startswith(prefix):
            try:
                raw_bytes = base64.b64decode(text[len(prefix):])
                envelope_pb2.ReplyEnvelope.FromString(raw_bytes)
            except Exception:
                counts["malformed"] += 1
        return original(text)

    conn._handle_binary_reply = wrapped  # type: ignore[method-assign]
    return counts


@dataclass
class GapEvent:
    missing_count: int    # number of consecutive missing seq values
    poll_time: float       # [s] wall-clock offset (from capture start) when the gap was observed


@dataclass
class CaptureResult:
    label: str
    port: str
    period: int              # [ms] armed STREAM period
    duration: float           # [s] wall-clock capture window (arm to disarm)
    delivered: int            # frames actually received
    expected_from_seq: int    # frames implied by the first/last seq span
    delivery_rate: float      # [fps] delivered / duration
    drop_rate: float          # fraction of expected-from-seq frames missing
    longest_gap: int          # largest single run of consecutive missing seq
    gap_events: list          # list[GapEvent] — every run of >=1 missing frame
    malformed: int            # *B frames that failed base64/protobuf decode
    connect_info: dict        # SerialConnection.connect() return value
    ping_ok: bool             # sanity PING succeeded before arming stream


def analyze(frames_with_time: "list[tuple[float, TLMFrame]]") -> dict:
    """Compute delivered/expected/drop/gap statistics from a capture."""
    seq_series = [(t, f.seq) for t, f in frames_with_time if f.seq is not None]
    delivered = len(frames_with_time)
    if len(seq_series) < 2:
        return {
            "delivered": delivered,
            "expected_from_seq": delivered,
            "drop_rate": 0.0,
            "longest_gap": 0,
            "gap_events": [],
        }

    span = 0
    gap_events: list[GapEvent] = []
    for i in range(1, len(seq_series)):
        prev_t, prev_seq = seq_series[i - 1]
        cur_t, cur_seq = seq_series[i]
        gap = (cur_seq - prev_seq) & 0xFFFF  # uint16 wrap-safe
        span += gap
        if gap > 1:
            gap_events.append(GapEvent(missing_count=gap - 1, poll_time=cur_t))

    return {
        "delivered": delivered,
        "expected_from_seq": span + 1,
        "drop_rate": tlm_drop_rate([f for _, f in frames_with_time]),
        "longest_gap": max((g.missing_count for g in gap_events), default=0),
        "gap_events": gap_events,
    }


def classify_gap_pattern(gap_events: "list[GapEvent]", duration: float) -> str:
    """Burst vs uniform: bucket gap events into 1-second windows and compare
    how many distinct windows contain loss against how many total missing
    frames there are. Concentrated in a few windows => burst; one-per-many
    windows, evenly spread => uniform sparse loss."""
    if not gap_events:
        return "none (zero gaps observed)"
    buckets = Counter(int(g.poll_time) for g in gap_events)
    total_missing = sum(g.missing_count for g in gap_events)
    windows_with_loss = len(buckets)
    max_in_one_window = max(buckets.values())
    span_windows = max(1, int(duration))
    if max_in_one_window >= max(3, total_missing * 0.5):
        return (f"burst — {max_in_one_window}/{total_missing} missing frames "
                f"landed in a single ~1s window "
                f"({windows_with_loss} of {span_windows} windows had any loss)")
    return (f"uniform/sparse — {total_missing} missing frames spread across "
            f"{windows_with_loss} of {span_windows} 1s windows "
            f"(max {max_in_one_window} in any one window)")


def run_capture(port: str, label: str, duration: float, period: int,
                 poll_interval: float) -> CaptureResult:
    conn = SerialConnection(port=port)
    malformed_counts = instrument_malformed_counter(conn)

    info = conn.connect()
    if info.get("status") not in ("connected", "already_connected"):
        raise RuntimeError(f"connect() failed for {port}: {info}")

    proto = NezhaProtocol(conn)

    # Sanity round-trip before arming the stream (per ticket: confirm we are
    # actually talking to the robot on this path, not just that the port
    # opened).
    ping_result = proto.ping()
    ping_ok = ping_result is not None
    print(f"[{label}] connect: {info.get('mode')!r} ping={ping_result} "
          f"announcement={info.get('announcement')}")

    # Drop any stale frames left over from a previous session before arming.
    proto.read_pending_binary_tlm_frames()

    frames_with_time: list[tuple[float, TLMFrame]] = []
    start = time.monotonic()
    proto.stream(period)
    try:
        next_poll = start
        while True:
            now = time.monotonic()
            elapsed = now - start
            if elapsed >= duration:
                break
            sleep_for = max(0.0, next_poll + poll_interval - now)
            time.sleep(sleep_for)
            next_poll += poll_interval
            batch = proto.read_pending_binary_tlm_frames()
            poll_time = time.monotonic() - start
            for frame in batch:
                frames_with_time.append((poll_time, frame))
            if int(poll_time) % 30 == 0:
                print(f"[{label}] t={poll_time:6.1f}s frames_so_far={len(frames_with_time)} "
                      f"malformed_so_far={malformed_counts['malformed']}")
    finally:
        proto.stream(0)
        # Final drain — catch anything queued between the last poll and disarm.
        tail = proto.read_pending_binary_tlm_frames()
        tail_time = time.monotonic() - start
        for frame in tail:
            frames_with_time.append((tail_time, frame))

    actual_duration = time.monotonic() - start
    stats = analyze(frames_with_time)

    return CaptureResult(
        label=label,
        port=port,
        period=period,
        duration=actual_duration,
        delivered=stats["delivered"],
        expected_from_seq=stats["expected_from_seq"],
        delivery_rate=stats["delivered"] / actual_duration if actual_duration else 0.0,
        drop_rate=stats["drop_rate"],
        longest_gap=stats["longest_gap"],
        gap_events=stats["gap_events"],
        malformed=malformed_counts["malformed"],
        connect_info={k: v for k, v in info.items() if k != "lines"},
        ping_ok=ping_ok,
    )


def report(result: CaptureResult) -> None:
    print()
    print(f"=== {result.label} ({result.port}) ===")
    print(f"  period armed        : {result.period} ms (~{1000.0 / result.period:.1f} Hz target)")
    print(f"  capture duration     : {result.duration:.1f} s")
    print(f"  ping sanity check    : {'OK' if result.ping_ok else 'FAILED'}")
    print(f"  delivered frames     : {result.delivered}")
    print(f"  expected (seq span)  : {result.expected_from_seq}")
    print(f"  frames/sec delivered : {result.delivery_rate:.2f}")
    print(f"  drop rate            : {result.drop_rate * 100:.2f}%")
    print(f"  longest gap          : {result.longest_gap} consecutive missing frames")
    print(f"  gap pattern          : {classify_gap_pattern(result.gap_events, result.duration)}")
    print(f"  malformed *B frames  : {result.malformed}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--label", default="capture")
    parser.add_argument("--duration", type=float, default=240.0)      # [s]
    parser.add_argument("--period", type=int, default=33)             # [ms] ~30 Hz
    parser.add_argument("--poll-interval", type=float, default=1.0)   # [s]
    parser.add_argument("--json-out", default=None)
    args = parser.parse_args()

    result = run_capture(args.port, args.label, args.duration, args.period,
                          args.poll_interval)
    report(result)

    if args.json_out:
        payload = asdict(result)
        with open(args.json_out, "w") as f:
            json.dump(payload, f, indent=2, default=str)
        print(f"\nwrote {args.json_out}")

    return 0 if result.ping_ok else 2


if __name__ == "__main__":
    sys.exit(main())
