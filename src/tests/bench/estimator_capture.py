#!/usr/bin/env python3
"""src/tests/bench/estimator_capture.py -- sprint 117 ticket 006: drive a
scripted, varied MOVE-pattern sequence (both directions, steps, reversals,
pivots; straights and turns -- the same pattern shape ticket 005's sim-system
scenario and the stakeholder's own validation methodology use) while
capturing the TLM stream to CSV, reusing ``tlm_log.py``'s existing
``stream_to_csv()``/``frame_to_row()``/``CSV_FIELDNAMES`` machinery verbatim
-- this module adds ONLY the MOVE-pattern-driving loop on top, per this
ticket's own Implementation Plan ("reuse, don't reimplement, the
frame-to-row/CSV logic sprint 115 already built and tested").

Works against a real ``NezhaProtocol`` connection (serial or ``--relay``)
**and** a ``robot_radio.io.sim_loop.SimLoop`` instance -- the read side
(``stream_to_csv()``'s own ``FrameSource`` protocol,
``read_pending_binary_tlm_frames() -> list[TLMFrame]``) is ALREADY satisfied
identically by both (see ``tlm_log.py``'s own module docstring); this module
only adds the analogous duck-typed dispatch on the WRITE side (issuing MOVE
commands), in ``_drive_segment()`` below.

Why the write side needs its own dispatch (no single shared method name)
--------------------------------------------------------------------------
``NezhaProtocol.move_twist(v_x, v_y, omega, *, stop_time, timeout, replace,
move_id)`` (the current, post-116-001 MOVE protocol) and
``SimLoop.twist(v_x, omega, duration)`` (a ``TwistTransport``-shaped
behavior-preserving translation onto the SAME protocol, ``sim_ctypes.cpp``'s
``sim_inject_twist()`` -> ``SimHarness::injectMove(..., kTime, ...)``) do NOT
share a method name or signature. ``SimLoop`` ALSO exposes a ``.move()``
method, but it is STALE dead code left over from BEFORE the 116-001 MOVE
protocol cutover -- it builds ``envelope_pb2.Move(distance=..., delta_
heading=..., v_max=..., omega=..., time=..., replace=..., id=...)``, and the
CURRENT ``Move`` message (``src/protos/envelope.proto``) has no
``delta_heading``/``v_max`` fields at all (verified directly: constructing
that call raises ``ValueError: Protocol message Move has no "delta_heading"
field.``) -- calling ``SimLoop.move()`` today crashes immediately. This
module therefore drives BOTH sources through their own bounded-TWIST
entry point (``move_twist()``/``twist()``), dispatched by ``hasattr()`` in
``_drive_segment()``, and never touches ``SimLoop.move()``.

Usage:
    # sim-mode capture (works today even with the bench motor bus down --
    # see this sprint's own bench-gate contingency, ticket 008)
    uv run python src/tests/bench/estimator_capture.py --sim

    # real hardware (serial or --relay)
    uv run python src/tests/bench/estimator_capture.py --port /dev/cu.usbmodem2121102
    uv run python src/tests/bench/estimator_capture.py --relay --port /dev/cu.usbmodem2121302
"""
from __future__ import annotations

import argparse
import pathlib
import sys
import threading
import time
from dataclasses import dataclass
from typing import Sequence

_BENCH_DIR = pathlib.Path(__file__).resolve().parent
if str(_BENCH_DIR) not in sys.path:
    sys.path.insert(0, str(_BENCH_DIR))

from tlm_log import stream_to_csv  # noqa: E402  (path must be set up before this import)

_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
DEFAULT_PORT = "/dev/cu.usbmodem2121102"
DEFAULT_CSV = _REPO_ROOT / "src" / "tests" / "bench" / "out" / "estimator_capture.csv"

# The same fixture robot config test_sim_loop.py/test_tour_closure_gate.py/
# test_turn_error_characterization.py already use for a configured SimLoop
# (data/robots/tovez_nocal.json) -- see _run_sim()'s own comment for why a
# SimLoop MUST be configured before it will accept any MOVE at all
# (sprint 114's fail-closed configuration-completeness gate).
DEFAULT_ROBOT_JSON = _REPO_ROOT / "data" / "robots" / "tovez_nocal.json"

_MOVE_TIMEOUT_MARGIN = 1000.0  # [ms] safety-backstop margin over a segment's own duration


@dataclass(frozen=True)
class MoveSegment:
    """One scripted twist leg: (label, v_x, omega, duration). Mirrors
    ``Move``'s own TWIST velocity variant with a TIME stop condition -- the
    simplest Move shape both ``NezhaProtocol.move_twist()`` and
    ``SimLoop.twist()`` already support identically (see this module's own
    header for why the WHEELS variant / DISTANCE / ANGLE stop conditions are
    out of scope here: this capture script only needs varied v_x/omega over
    time, not to exercise every Move stop-condition kind -- that is ticket
    005's/116-008's own job)."""

    label: str
    v_x: float      # [mm/s]
    omega: float    # [rad/s]
    duration: float  # [ms]


# The stakeholder's own validation-methodology pattern set (also mirrored by
# ticket 005's sim-system scenario, state_estimator_tracking_harness.cpp):
# both directions, steps, a reversal, pivots (both directions), then a
# chained sequence of short legs mixing straights and a turn.
DEFAULT_PATTERN: "tuple[MoveSegment, ...]" = (
    MoveSegment("forward_step", v_x=150.0, omega=0.0, duration=1500.0),
    MoveSegment("forward_reversal", v_x=-150.0, omega=0.0, duration=1500.0),
    MoveSegment("pivot_ccw", v_x=0.0, omega=1.0, duration=1200.0),
    MoveSegment("pivot_cw", v_x=0.0, omega=-1.0, duration=1200.0),
    MoveSegment("chain_step_a", v_x=100.0, omega=0.0, duration=500.0),
    MoveSegment("chain_step_b", v_x=-100.0, omega=0.0, duration=500.0),
    MoveSegment("chain_turn", v_x=0.0, omega=-1.0, duration=500.0),
    MoveSegment("chain_fast", v_x=200.0, omega=0.0, duration=500.0),
)


_next_move_id_lock = threading.Lock()
_next_move_id_counter = 6000  # well above any corr_id a session's own connection assigns


def _next_move_id() -> int:
    global _next_move_id_counter
    with _next_move_id_lock:
        _next_move_id_counter += 1
        return _next_move_id_counter


def _drive_segment(source, segment: MoveSegment) -> None:
    """Issue *segment* as a bounded, TIME-stop, ``replace=True`` twist Move
    against *source* -- dispatched on which move-issuing method *source*
    actually exposes. See this module's own header for why there is no
    single shared method/signature (unlike the read side's
    ``read_pending_binary_tlm_frames()``, which both objects already satisfy
    identically)."""
    if hasattr(source, "move_twist"):
        # A real NezhaProtocol (or a duck-typed stand-in exposing the same
        # method) -- fire-and-poll, matching every other bench script's own
        # convention (twist_drive.py/move_protocol_bench.py): this call
        # never blocks on the enqueue ack.
        source.move_twist(v_x=segment.v_x, v_y=0.0, omega=segment.omega,
                          stop_time=segment.duration,
                          timeout=segment.duration + _MOVE_TIMEOUT_MARGIN,
                          replace=True, move_id=_next_move_id())
    elif hasattr(source, "twist"):
        # A SimLoop (or duck-typed stand-in) -- see this module's own header
        # for why SimLoop.move() is never used instead.
        source.twist(segment.v_x, segment.omega, segment.duration)
    else:
        raise TypeError(
            f"{source!r} exposes neither move_twist() (NezhaProtocol) nor twist() (SimLoop) "
            "-- cannot drive a MOVE pattern against it")


def drive_pattern(source, pattern: "Sequence[MoveSegment]" = DEFAULT_PATTERN,
                  settle: float = 0.05) -> "list[tuple[str, float]]":  # [s]
    """Issue every segment in *pattern* back-to-back on the CALLING thread,
    sleeping out each segment's own commanded duration (plus a small settle
    margin) before issuing the next. Ends with an explicit ``stop()``.

    Returns ``[(label, elapsed_s_at_segment_start), ...]`` -- a host
    WALL-CLOCK schedule (``time.monotonic()``-based), for a human/caller to
    eyeball roughly where each phase landed. This is NOT the robot's own
    clock -- a later analysis (ticket 007's notebook) that needs to group
    residuals by pattern phase should derive phase boundaries from the
    CAPTURED CSV's own ``now``/``enc_left_time``/`otos_time`` columns (the
    robot's own clock domain, self-consistent with the rest of the ZOH math
    -- see ``src/tests/tools/one_step_ahead.py``'s own ``group_rms_by_
    phase()``), not from this wall-clock schedule, which is only ever an
    approximate cross-check.
    """
    schedule: "list[tuple[str, float]]" = []
    start = time.monotonic()
    try:
        for segment in pattern:
            schedule.append((segment.label, time.monotonic() - start))
            _drive_segment(source, segment)
            time.sleep(segment.duration / 1000.0 + settle)
    finally:
        # Guaranteed stop: motors must never be left running on an
        # exception or Ctrl-C (hardware-bench-testing.md).
        try:
            source.stop()
        except Exception:
            pass
    return schedule


def capture_with_pattern(source, csv_path: "str | pathlib.Path",
                         pattern: "Sequence[MoveSegment]" = DEFAULT_PATTERN,
                         settle: float = 0.05,
                         end_margin: float = 2.0) -> "tuple[int, list[tuple[str, float]]]":  # [s]
    """Drive *pattern* against *source* while concurrently capturing its TLM
    stream to *csv_path* via ``tlm_log.py``'s own, UNMODIFIED
    ``stream_to_csv()`` -- run on a background thread for the pattern's own
    total duration (plus *end_margin* seconds of trailing capture, so the
    last segment's own settle/completion telemetry lands in the CSV too),
    while ``drive_pattern()`` issues the scripted commands on the calling
    thread. Both ``NezhaProtocol``/``SerialConnection`` and ``SimLoop``
    already run their own background reader/tick thread feeding a thread-safe
    queue ``read_pending_binary_tlm_frames()`` drains -- concurrently issuing
    commands on the calling thread while ``stream_to_csv()`` polls on a
    second thread is safe against either (no shared mutable state between
    the two operations on either class).

    Returns ``(row_count, schedule)`` -- ``row_count`` is ``stream_to_csv()``'s
    own return value; ``schedule`` is ``drive_pattern()``'s own wall-clock
    schedule (see that function's docstring).
    """
    total_duration = sum(seg.duration for seg in pattern) / 1000.0
    total_duration += settle * len(pattern) + end_margin

    row_count_holder: "list[int]" = []

    def _capture() -> None:
        row_count_holder.append(stream_to_csv(source, csv_path, total_duration))

    capture_thread = threading.Thread(target=_capture, name="estimator-capture-tlm", daemon=True)
    capture_thread.start()
    schedule = drive_pattern(source, pattern, settle=settle)
    capture_thread.join(timeout=total_duration + 10.0)

    row_count = row_count_holder[0] if row_count_holder else 0
    return row_count, schedule


def _args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--sim", action="store_true",
                   help="capture against a SimLoop instance instead of real hardware")
    p.add_argument("--port", default=DEFAULT_PORT, help="serial port (ignored with --sim)")
    p.add_argument("--relay", action="store_true",
                   help="port is a radio relay dongle (ignored with --sim)")
    p.add_argument("--robot-json", default=str(DEFAULT_ROBOT_JSON),
                   help=f"robot config to configure the SimLoop from (--sim only, "
                        f"default {DEFAULT_ROBOT_JSON})")
    p.add_argument("--csv", default=str(DEFAULT_CSV), help=f"output CSV path (default {DEFAULT_CSV})")
    return p.parse_args()


def _run_sim(args: argparse.Namespace) -> int:
    from robot_radio.config.robot_config import load_robot_config
    from robot_radio.io.sim_loop import SimLoop

    # A freshly-constructed SimLoop's own RobotLoop starts UNCONFIGURED
    # (App::RobotLoop::isConfigured() == false) -- sprint 114's fail-closed
    # configuration-completeness gate makes handleMove() refuse EVERY Move
    # with ERR_NOT_CONFIGURED until configure_from_robot() has run (see that
    # method's own docstring, sim_loop.py). Skipping this call doesn't raise
    # or warn -- move_twist()/twist() are fire-and-poll, so a capture run
    # against an unconfigured sim silently produces a flat, all-zero
    # trace (discovered exactly this way verifying this ticket's own sim
    # capture -- see this ticket's completion notes). ``SimLoop(track_width=
    # ...)`` is constructed from the SAME robot config's own trackwidth so
    # SimPlant's OtosPlant and the firmware's own App::Odometry describe the
    # same physical wheelbase (sim_plant.h's own "MUST match" comment).
    robot_config = load_robot_config(args.robot_json)
    track_width = robot_config.trackwidth if robot_config.trackwidth is not None else 128.0

    sim = SimLoop(track_width=track_width)
    sim.connect()
    sim.configure_from_robot(robot_config)
    print(f"sim connected: firmware={sim.firmware_version()} track_width={track_width} "
          f"robot={args.robot_json}")
    try:
        # Discard anything queued before this run started (boot frames),
        # matching tlm_log.py's own main() convention.
        sim.read_pending_binary_tlm_frames()
        print(f"capturing sim pattern ({len(DEFAULT_PATTERN)} segments) -> {args.csv}")
        row_count, schedule = capture_with_pattern(sim, args.csv)
    finally:
        try:
            sim.stop()
        except Exception:
            pass
        sim.disconnect()

    for label, elapsed in schedule:
        print(f"  t={elapsed:6.2f}s  segment={label}")
    print(f"wrote {row_count} rows to {args.csv}")
    return 0 if row_count > 0 else 1


def _run_hardware(args: argparse.Namespace) -> int:
    from robot_radio.io.serial_conn import SerialConnection
    from robot_radio.robot.protocol import NezhaProtocol

    mode = "relay" if args.relay else None  # None -> SerialConnection auto-detects
    conn = SerialConnection(port=args.port, mode=mode)
    info = conn.connect()
    if info.get("status") != "connected":
        print(f"ERROR: connect failed: {info}")
        return 2
    proto = NezhaProtocol(conn)
    print(f"connected: port={args.port} mode={info.get('mode')}")

    row_count, schedule = 0, []
    try:
        proto.read_pending_binary_tlm_frames()
        print(f"capturing hardware pattern ({len(DEFAULT_PATTERN)} segments) -> {args.csv}")
        row_count, schedule = capture_with_pattern(proto, args.csv)
    finally:
        try:
            proto.stop()
        except Exception:
            pass
        conn.disconnect()

    for label, elapsed in schedule:
        print(f"  t={elapsed:6.2f}s  segment={label}")
    print(f"wrote {row_count} rows to {args.csv}")
    return 0 if row_count > 0 else 1


def main() -> int:
    args = _args()
    return _run_sim(args) if args.sim else _run_hardware(args)


if __name__ == "__main__":
    sys.exit(main())
