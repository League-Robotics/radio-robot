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

================================================================================
PHASE 2 -- THE FOUR STOP TAILS (133-003)
================================================================================

Sprint 133 needs a different question answered by the same rig: not "did
the estop land" but **how much distance does each way of stopping actually
cost, after the moment motion was last commanded?** The source issue's
headline number is 936 mm of continued travel with no decay, measured on
`vevov` with a host that commanded a stop ONCE and then went quiet.

Four tails, each measured the same way (`--tails`, default all four):

    silent    the host issues one bounded `wheels()` and then STOPS TALKING.
              Nothing halts the robot but the command's own duration
              expiring inside Core::DifferentialDrive -- the exact case that produced
              936 mm. "Silent" means no COMMANDS; telemetry is still read,
              which is passive and cannot influence the robot.
    estop     one `estop()`, mid-leg.
    wheels0   one `wheels(0, 0, ...)`, mid-leg. The other single-write halt,
              and the one the 2026-07-31 incident also lost.
    stream    `wheels(0, 0, ...)` re-armed every `--tick` for the whole tail
              -- the belt-and-braces halt. Repetition is what USED to be
              required to stop the wheels, so this is the control case:
              whatever distance shows up here is honest coast, and every
              other tail is measured against it.

THE MEASUREMENT BUG THIS CARRIES A FIX FOR. An earlier harness anchored
its tail at the BASELINE frame -- captured during the leading settle window,
before the leg even started -- which charges the whole commanded leg to the
tail and inflates every figure by roughly 150 mm. Travel here is measured
from the **commanded-zero transition**:

    estop / wheels0   the monotonic instant of the WIRE WRITE (never the
                      ack -- the ack rides the next telemetry push)
    stream            the instant of the FIRST zero write
    silent            the frame where `active` (flags bit 2) goes False --
                      i.e. when the ROBOT stopped commanding motion. There
                      is no wire write to timestamp in this tail, so the
                      robot's own transition is the only honest anchor.

The encoder anchor is the last frame at or before that instant, and its
AGE is reported on every trial, because a one-frame-old anchor over-counts
by up to a frame period of real travel (~7.5 mm at 150 mm/s) and that
residual should be visible rather than folded silently into the result.

Usage:
    uv run python src/tests/bench/estop_unlosable_bench.py
    uv run python src/tests/bench/estop_unlosable_bench.py --port /dev/cu.usbmodem2121102
    uv run python src/tests/bench/estop_unlosable_bench.py --trials 10

    # just the tails, three trials of each
    uv run python src/tests/bench/estop_unlosable_bench.py --trials 0 --tail-trials 3

    # one tail, verbosely
    uv run python src/tests/bench/estop_unlosable_bench.py --trials 0 --tails silent
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol

DEFAULT_PORT = "/dev/cu.usbmodem2121102"
ACK_TIMEOUT = 500  # [ms] wait_for_ack() bound for each command's ack

# The four tails, in the order they are run. `stream` runs LAST on purpose:
# it is the control case, and running it after the three single-write tails
# means its number is taken on a rig that has already been asked to stop
# three times without repetition -- which is exactly the state the original
# defect needed to manifest.
TAIL_NAMES = ("silent", "estop", "wheels0", "stream")


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
    # --- phase 2: the four stop tails (133-003) ---
    p.add_argument("--tails", default=",".join(TAIL_NAMES),
                   help=f"comma-separated tails to measure, from "
                        f"{{{','.join(TAIL_NAMES)}}} (default: all four). "
                        f"Empty string skips phase 2 entirely.")
    p.add_argument("--tail-trials", type=int, default=3,
                   help="trials per tail. More than one because a single "
                        "clean stop proves nothing -- the defect this "
                        "measures needed a LOST write on some earlier trial")
    p.add_argument("--tail-window", type=float, default=6.0,  # [s]
                   help="how long to keep watching after the commanded-zero "
                        "transition. The source issue's capture ENDED while "
                        "the robot was still moving at 8 s, so this wants to "
                        "be long enough to see a non-decaying runaway rather "
                        "than just the top of one")
    p.add_argument("--tail-bound", type=float, default=60.0,  # [mm]
                   help="PASS bound on travel after the commanded-zero "
                        "transition. A safety bound, not a precision one: "
                        "honest coast at 150 mm/s is tens of mm, and the "
                        "defect this catches measured 936 mm")
    p.add_argument("--tail-tick", type=float, default=0.1,  # [s]
                   help="re-arm interval for the `stream` tail's zeros")
    args = p.parse_args()
    # Validate HERE, not inside run_tails(): that runs after connect, so a
    # typo would only surface once a robot was already spun up and driving.
    unknown = [name for name in _wanted_tails(args) if name not in TAIL_NAMES]
    if unknown:
        p.error(f"unknown tail(s) {unknown}; choose from {list(TAIL_NAMES)}")
    return args


def _wanted_tails(args: argparse.Namespace) -> "list[str]":
    return [name.strip() for name in args.tails.split(",") if name.strip()]


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


# ---------------------------------------------------------------------------
# Phase 2 -- the four stop tails (133-003)
# ---------------------------------------------------------------------------

def _positions(frame) -> "tuple[float, float] | None":
    """Per-wheel accumulated position [mm] off one frame, from the
    float-valued `EncoderReading`s rather than `TLMFrame.enc`'s rounded
    integer pair. The tail figures are reported to a tenth of a millimetre;
    take the precision that is free."""
    left, right = frame.enc_left, frame.enc_right
    if left is None or right is None:
        return None
    return (float(left.position), float(right.position))


class TailCapture:
    """Every frame seen during one tail trial, with its host receive time.

    Frames are collected first and interpreted afterwards, deliberately: the
    commanded-zero transition for the `silent` tail is only knowable from the
    frames themselves, so the anchor cannot be chosen while polling."""

    def __init__(self) -> None:
        self.frames: "list[tuple[float, object]]" = []  # (recvTime [s], frame)

    def drain(self, proto: NezhaProtocol) -> None:
        now = time.monotonic()
        for frame in proto.read_pending_binary_tlm_frames():
            received = frame.recvTime if frame.recvTime is not None else now
            self.frames.append((float(received), frame))

    def anchor(self, at: float) -> "tuple[float, tuple[float, float]] | None":
        """The (recvTime, positions) pair to measure travel FROM, given a
        commanded-zero instant `at`.

        Prefers the last frame at or before `at` -- conservative, it can
        only over-count -- and falls back to the first frame after when the
        transition happened before any frame arrived. Returns None when no
        frame in this capture carries positions at all."""
        with_positions = [(t, _positions(f)) for t, f in self.frames]
        with_positions = [(t, p) for t, p in with_positions if p is not None]
        if not with_positions:
            return None
        before = [(t, p) for t, p in with_positions if t <= at]
        if before:
            return before[-1]
        return with_positions[0]

    def active_fell(self) -> "float | None":
        """recvTime of the first frame where `active` reads False AFTER it
        has read True -- the robot's own "I stopped commanding motion"
        transition. None when the leg was never observed active (which is a
        finding, not a detail: it means the drive command never took)."""
        seen_active = False
        for received, frame in self.frames:
            if frame.active is None:
                continue
            if frame.active:
                seen_active = True
            elif seen_active:
                return received
        return None

    def travel_after(self, anchor_positions: "tuple[float, float]") -> "tuple[float, float]":
        """Signed travel [mm] per wheel from `anchor_positions` to the last
        frame that carried positions."""
        final = None
        for _, frame in self.frames:
            positions = _positions(frame)
            if positions is not None:
                final = positions
        if final is None:
            return (0.0, 0.0)
        return (final[0] - anchor_positions[0], final[1] - anchor_positions[1])

    def settled_at(self, at: float, *, quiet: float = 0.5) -> "float | None":
        """Seconds from `at` to the LAST frame whose positions differed from
        the previous one -- i.e. how long the wheels kept turning.

        `quiet` [mm] is the change below which two consecutive readings count
        as the same position: an encoder at rest still dithers by well under
        a millimetre, and calling that motion would make every tail read as
        never settling."""
        last_change = at
        previous = None
        for received, frame in self.frames:
            positions = _positions(frame)
            if positions is None or received < at:
                if positions is not None:
                    previous = positions
                continue
            if previous is not None and (
                    abs(positions[0] - previous[0]) > quiet
                    or abs(positions[1] - previous[1]) > quiet):
                last_change = received
            previous = positions
        return last_change - at


def run_tail_trial(proto: NezhaProtocol, tail: str, trial: int,
                   args: argparse.Namespace, result: Result) -> "float | None":
    """Measure ONE stop tail once. Returns the travel [mm] accumulated after
    the commanded-zero transition (the larger of the two wheels), or None
    when the trial could not be measured at all."""
    label = f"{tail} tail, trial {trial}"

    # `wheels()` for every tail, not `move_wheels()`: it routes straight to
    # Core::DifferentialDrive with no planner, no ramps and no odometry stop condition,
    # which is the path the runaway was measured on. The planner's own queue
    # would add a second thing that could stop the robot and make the tail
    # unattributable.
    capture = TailCapture()
    proto.read_pending_binary_tlm_frames()          # discard anything stale
    leg = args.leg_duration                          # [ms]
    command_sent_at = time.monotonic()
    proto.wheels(args.speed, args.speed, leg)

    # Drive until well inside the leg, confirming the wheels really turn --
    # a tail measured on a robot that never moved proves nothing.
    deadline = time.monotonic() + args.mid_leg_delay
    while time.monotonic() < deadline:
        capture.drain(proto)
        time.sleep(0.02)
    start_anchor = capture.anchor(command_sent_at)
    moved = capture.travel_after(start_anchor[1]) if start_anchor else (0.0, 0.0)
    result.record(f"{label}: wheels genuinely moving before the halt",
                  max(abs(moved[0]), abs(moved[1])) > 1.0,
                  f"travelled L={moved[0]:.1f} R={moved[1]:.1f} mm")

    # --- the halt, and its commanded-zero instant ------------------------
    #
    # Timestamped around the WIRE WRITE, never the ack: the ack rides the
    # next telemetry push, and what is being measured is when the robot was
    # last asked to move, not when it got round to saying so.
    commanded_zero_at: "float | None" = None
    anchor_note = ""
    if tail == "estop":
        commanded_zero_at = time.monotonic()
        proto.estop()
    elif tail == "wheels0":
        commanded_zero_at = time.monotonic()
        proto.wheels(0.0, 0.0, args.tail_window * 1000.0)
    elif tail == "stream":
        commanded_zero_at = time.monotonic()
        proto.wheels(0.0, 0.0, args.tail_tick * 4000.0)
    elif tail == "silent":
        pass  # nothing is sent at all; the anchor comes from the frames
    else:
        raise ValueError(f"unknown tail {tail!r}")

    # --- watch ------------------------------------------------------------
    watch_until = time.monotonic() + args.tail_window
    next_zero = time.monotonic() + args.tail_tick
    while time.monotonic() < watch_until:
        capture.drain(proto)
        if tail == "stream" and time.monotonic() >= next_zero:
            # The ONLY tail that keeps talking. Its whole point is that
            # repetition is what used to be required to stop the wheels.
            proto.wheels(0.0, 0.0, args.tail_tick * 4000.0)
            next_zero = time.monotonic() + args.tail_tick
        time.sleep(0.02)
    capture.drain(proto)

    if tail == "silent":
        commanded_zero_at = capture.active_fell()
        if commanded_zero_at is None:
            # Fall back to the deadline the host ASKED for, and say so. A
            # silently substituted anchor is how a tail figure ends up
            # attributed to the wrong instant, which is the exact bug this
            # harness carries a fix for.
            commanded_zero_at = command_sent_at + leg / 1000.0
            anchor_note = ("`active` never fell in telemetry; anchored on the "
                           "commanded deadline instead -- treat as approximate")

    anchor = capture.anchor(commanded_zero_at)
    if anchor is None:
        result.record(f"{label}: tail measurable", False,
                      "no telemetry frame carried encoder positions")
        return None
    anchor_at, anchor_positions = anchor
    anchor_age = commanded_zero_at - anchor_at  # [s]

    travel_left, travel_right = capture.travel_after(anchor_positions)
    travel = max(abs(travel_left), abs(travel_right))  # [mm]
    settled = capture.settled_at(commanded_zero_at)     # [s]

    detail = (f"travel after commanded zero: L={travel_left:+.1f} "
              f"R={travel_right:+.1f} mm (worst {travel:.1f}); "
              f"wheels kept turning {settled:.2f}s; "
              f"anchor {anchor_age * 1000:.0f} ms old; "
              f"{len(capture.frames)} frames")
    if anchor_note:
        detail += f"; NOTE: {anchor_note}"
    result.record(f"{label}: travel after commanded zero <= {args.tail_bound:.0f} mm",
                  travel <= args.tail_bound, detail)

    # Leave the robot stopped between trials whatever the tail was, so the
    # NEXT trial's baseline is not contaminated by this one's residual
    # motion. This is cleanup, not part of the measurement -- it happens
    # strictly after the watch window closed.
    proto.estop()
    time.sleep(0.5)
    proto.read_pending_binary_tlm_frames()
    return travel


def run_tails(proto: NezhaProtocol, args: argparse.Namespace,
              result: Result) -> None:
    # Run in the canonical order regardless of how they were spelled on the
    # command line -- see TAIL_NAMES for why `stream` goes last. Names are
    # already validated by _args().
    wanted = _wanted_tails(args)
    ordered = [name for name in TAIL_NAMES if name in wanted]

    summary: "dict[str, list[float]]" = {}
    for tail in ordered:
        print(f"\n--- tail: {tail} ---")
        travels = []
        for trial in range(1, args.tail_trials + 1):
            travel = run_tail_trial(proto, tail, trial, args, result)
            if travel is not None:
                travels.append(travel)
        summary[tail] = travels

    print(f"\n  === stop tails (travel after the COMMANDED-ZERO transition, "
          f"not the baseline frame) ===")
    print(f"  {'tail':>8}  {'trials':>6}  {'median':>9}  {'worst':>9}  bound "
          f"{args.tail_bound:.0f} mm")
    for tail in ordered:
        travels = summary[tail]
        if not travels:
            print(f"  {tail:>8}  {'0':>6}  {'--':>9}  {'--':>9}  NOT MEASURED")
            continue
        print(f"  {tail:>8}  {len(travels):>6}  "
              f"{statistics.median(travels):>8.1f}mm  {max(travels):>8.1f}mm  "
              f"{'PASS' if max(travels) <= args.tail_bound else 'FAIL'}")


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

    # Telemetry is silent at IDLE by design, and a stop tail is measured
    # almost entirely at idle -- so without this every tail would go blind
    # at exactly the moment it starts mattering. TLM:ON forces a per-cycle
    # push regardless of the idle gate. It also strictly improves phase 1
    # (more frames to observe the settle in), and it costs nothing on the
    # command plane, so the `silent` tail stays genuinely silent.
    proto.tlmOn()
    time.sleep(1.0)
    proto.read_pending_binary_tlm_frames()

    try:
        for trial in range(1, args.trials + 1):
            print(f"\n--- trial {trial}/{args.trials} ---")
            run_trial(proto, trial, args, result)
        if args.tails.strip():
            run_tails(proto, args, result)
    finally:
        # Guaranteed stop: motors must never be left running on an
        # exception or Ctrl-C, even if a check above already stopped them.
        # estop(), never stop() -- a planned stop queues behind whatever is
        # in flight and would narrate a runaway rather than end one
        # (`.claude/rules/playfield-testing.md`, measured: 39.8 cm vs 2.9).
        #
        # Sent THREE times: this whole harness exists because a single zero
        # write can be lost on the bus and the Nezha brick latches its last
        # commanded speed. In the one path where a lost write means walking
        # away from a spinning robot, repetition is cheap.
        for _ in range(3):
            try:
                proto.estop()
            except Exception:
                pass
            time.sleep(0.1)
        try:
            proto.tlmOff()
        except Exception:
            pass
        conn.disconnect()

    return 0 if result.ok() else 1


if __name__ == "__main__":
    sys.exit(main())
