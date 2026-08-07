#!/usr/bin/env python3
"""move_protocol_bench.py -- ticket 116-010's own bench-gate script for the
MOVE-protocol queue/stop-condition/response surface (`App::MoveQueue`,
`Motion::StopCondition`, `RobotLoop::handleMove()`/`handleStop()`/
`handleConfig()`) that `twist_drive.py` (110-007's single-shot smoke test)
does not exercise: distance/angle stop conditions, the `wheels` velocity
variant, chaining (`replace=False`), `replace=True` mid-motion preemption,
the 5-deep `ERR_FULL` queue limit, the no-deadman empty-queue drain, the
`timeout` safety-backstop fault flag, `STOP` mid-motion (including a
flushed pending queue), and a `CONFIG` patch arriving mid-`Move`.

Mirrors `twist_drive.py`'s own `Result`-based PASS/FAIL reporting shape so
a human (or the next agent) can read the transcript directly. Every
scenario is a bare function taking `(proto, result)`; `main()` runs them
in sequence over ONE connection, draining and re-draining
`read_pending_binary_tlm_frames()` throughout (telemetry is always-on, no
STREAM arm to arm -- see `protocol.py`'s own module docstring).

Distance/angle stop conditions and the "encoders track sign/magnitude"
checks in this file depend on a LIVE motor bus (`Telemetry.flags` bits
3/4, `conn_left`/`conn_right`) -- `App::Odometry::pathLength()`/`theta()`
are both derived from real encoder deltas, so a disconnected bus makes a
DISTANCE/ANGLE `Move` fall back to ending via its `timeout` backstop
instead of its nominal stop condition (never a false pass: this script
checks the ACTUAL completion evidence -- `fault_move_timeout` vs. a
measured pose delta -- not just "did an ack arrive"). See this ticket's
own completion notes / the bench checklist doc
(`docs/bench-checklists/sprint-116-move-protocol.md`) for whether the bus
was live during a given run.

Usage:
    uv run python src/tests/bench/move_protocol_bench.py
    uv run python src/tests/bench/move_protocol_bench.py --port /dev/cu.usbmodem2121102
"""
from __future__ import annotations

import argparse
import math
import sys
import time

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol, TLMFrame

DEFAULT_PORT = "/dev/cu.usbmodem2121102"
ACK_TIMEOUT = 500  # [ms] wait_for_ack() bound for each command's enqueue ack

# Move.id values start well above any corr_id this session's SerialConnection
# will ever assign (a small monotonic counter starting at 1) so a completion
# ack (keyed by Move.id) is never confused with an enqueue ack (keyed by the
# envelope's own corr_id) -- see protocol-v4.md Sec 7.2.
_NEXT_MOVE_ID = 9000


def _next_move_id() -> int:
    global _NEXT_MOVE_ID
    _NEXT_MOVE_ID += 1
    return _NEXT_MOVE_ID


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


def _drain(proto: NezhaProtocol) -> list[TLMFrame]:
    return proto.read_pending_binary_tlm_frames()


def _wait_for_boot_ready(connect_info: dict) -> bool:
    """Report whether the robot was observed ready before its first real
    command may be sent -- 125-001
    (bench-move-commands-intermittently-never-reach-firmware.md).

    Root cause: `SerialConnection.connect()`'s own HELLO-classify + PING
    readiness poll can return (and, measured against real hardware, often
    does -- device-probe retries push `RobotLoop::boot()` past 1s) well
    BEFORE `Preamble::done()` actually fires -- HELLO/PING both answer
    straight out of `boot()`'s own `comms_.pump()` loop, independent of
    whether any device has resolved yet. A `Move` sent in that window now
    gets an EXPLICIT `ERR_NOT_CONFIGURED` ack (125-001's own firmware fix,
    `RobotLoop::rejectDuringBoot()` -- never a silent drop any more), but it
    still does not EXECUTE: firmware deliberately does not act on a command
    before every device is probed (executing early against a not-yet-
    resolved motor/OTOS would be the genuinely unsafe alternative, not the
    fix).

    125-005 (telemetry-emit-policy-rebuild-spec.md Part 7): this used to
    poll telemetry for `kFlagEventBootReady` (flags bit 11), which 125-002
    DELETED outright (the bit never sets any more, under any TLM mode) --
    that whole flags-word signal is gone, not just harder to observe under
    `kAuto`. Readiness is now `SerialConnection.connect()`'s own job: it
    already waits for the unsolicited `READY` cleartext line (falling back
    to the legacy flags-word bit for pre-READY firmware) before returning,
    and reports the outcome as `info["ready"]` -- see that method's own
    docstring. This function is kept as the call site's existing entry
    point, now just reading that field rather than polling telemetry itself
    (connect() already did the waiting, before this is ever called).
    """
    return bool(connect_info.get("ready"))


def _watch(proto: NezhaProtocol, duration: float,  # [s]
           on_frame=None) -> list[TLMFrame]:
    """Drain telemetry for `duration` seconds, collecting every frame (and
    calling `on_frame(frame)` for each, if given). Poll period well under
    the 20ms primary cycle so no frame waits a full poll behind."""
    frames: list[TLMFrame] = []
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        for f in proto.read_pending_binary_tlm_frames():
            frames.append(f)
            if on_frame is not None:
                on_frame(f)
        time.sleep(0.01)
    return frames


def _find_ack_entry(frames: list[TLMFrame], corr_id: int):
    """Scan the bounded ack ring (`TLMFrame.acks`, 120) across `frames` for
    the first entry matching `corr_id` -- ring-based, NOT the single
    freshest-ack scalar slot (`ack_fresh`/`ack_corr`/`ack_err`), which this
    script used to scan directly (a manual re-implementation of the exact
    single-slot race this ticket's ack ring fixes -- see protocol-v4.md
    Sec 7.1/7.2). Returns the matched `AckEntry` (its own `.ok`/`.err_code`
    are ALWAYS the right ones for `corr_id`, even when the frame's own
    scalar slot has since moved on to a different, later command), or
    `None` if `corr_id` never appears in any frame's ring."""
    for f in frames:
        for entry in f.acks:
            if entry.corr_id == corr_id:
                return entry
    return None


def _find_completion_ack(frames: list[TLMFrame], move_id: int) -> TLMFrame | None:
    """Return the first frame whose ack ring (120) carries `move_id`'s
    completion ack -- see protocol-v4.md Sec 7.2. Ring-based (scans every
    entry in every frame's bounded ack ring), not the single freshest-ack
    scalar slot the pre-ring implementation scanned -- a completion ack
    pushed onto the ring but later superseded as "the freshest ack" by a
    different command's own ack is still found here. Returns the FRAME
    (for positional/timing reasoning against the rest of the stream, e.g.
    "no idle gap after this frame") -- use `_find_ack_entry()` instead when
    the ack's own outcome (`.ok`/`.err_code`) is what's needed, since this
    frame's own scalar `ack_corr`/`ack_err` may belong to a DIFFERENT,
    later command by the time it's read."""
    for f in frames:
        if any(entry.corr_id == move_id for entry in f.acks):
            return f
    return None


def _last_pose(frames: list[TLMFrame]) -> tuple[int, int, int] | None:
    for f in reversed(frames):
        if f.pose is not None:
            return f.pose
    return None


# ---------------------------------------------------------------------------
# Scenarios
# ---------------------------------------------------------------------------

def scenario_distance_stop(proto: NezhaProtocol, result: Result) -> None:
    """DISTANCE-stop MOVE: measure actual traveled distance (pose delta)
    against the commanded threshold. A disconnected motor bus makes this
    end via `timeout` instead (pathLength() never advances) -- reported as
    a FAIL with the exact evidence, not silently passed on ack alone."""
    _drain(proto)
    before = _last_pose(_watch(proto, 0.1)) or (0, 0, 0)
    move_id = _next_move_id()
    corr = proto.move_wheels(v_left=150.0, v_right=150.0, stop_distance=200.0,
                             timeout=3000.0, replace=True, move_id=move_id)
    ack = proto.wait_for_ack(corr, timeout=ACK_TIMEOUT)
    result.record("distance MOVE enqueue ack ok", ack is not None and ack.ok, f"ack={ack}")

    frames = _watch(proto, 3.5)
    completion = _find_completion_ack(frames, move_id)
    after = _last_pose(frames) or before
    dx, dy = after[0] - before[0], after[1] - before[1]
    traveled = math.hypot(dx, dy)  # [mm]
    timed_out = any(f.fault_move_timeout for f in frames)
    result.record("distance MOVE completion ack observed (Move.id)", completion is not None,
                  f"move_id={move_id}")
    result.record("distance MOVE ended via its own stop condition, not timeout",
                  not timed_out, f"fault_move_timeout seen={timed_out}")
    result.record("distance MOVE traveled ~200mm (+/-20% tolerance)",
                  180.0 <= traveled <= 240.0,
                  f"before={before[:2]} after={after[:2]} traveled={traveled:.1f}mm")
    proto.estop()


def scenario_angle_stop(proto: NezhaProtocol, result: Result) -> None:
    """ANGLE-stop MOVE (pivot via move_wheels(+v,-v)): measure actual
    heading change (pose theta delta, centidegrees) against the commanded
    threshold. Same live-bus caveat as scenario_distance_stop()."""
    _drain(proto)
    before = _last_pose(_watch(proto, 0.1)) or (0, 0, 0)
    move_id = _next_move_id()
    stop_angle_rad = 0.5  # [rad] ~28.6 deg
    corr = proto.move_wheels(v_left=-120.0, v_right=120.0, stop_angle=stop_angle_rad,
                             timeout=3000.0, replace=True, move_id=move_id)
    ack = proto.wait_for_ack(corr, timeout=ACK_TIMEOUT)
    result.record("angle MOVE enqueue ack ok", ack is not None and ack.ok, f"ack={ack}")

    frames = _watch(proto, 3.5)
    completion = _find_completion_ack(frames, move_id)
    after = _last_pose(frames) or before
    dtheta_cdeg = after[2] - before[2]
    dtheta_rad = math.radians(dtheta_cdeg / 100.0)
    timed_out = any(f.fault_move_timeout for f in frames)
    result.record("angle MOVE completion ack observed (Move.id)", completion is not None,
                  f"move_id={move_id}")
    result.record("angle MOVE ended via its own stop condition, not timeout",
                  not timed_out, f"fault_move_timeout seen={timed_out}")
    result.record("angle MOVE rotated ~0.5rad (+/-25% tolerance)",
                  0.375 <= abs(dtheta_rad) <= 0.625,
                  f"before_theta_cdeg={before[2]} after_theta_cdeg={after[2]} "
                  f"dtheta={dtheta_rad:.3f}rad")
    proto.estop()


def scenario_wheels_variant_signs(proto: NezhaProtocol, result: Result) -> None:
    """MoveWheels(v_left, v_right) with opposite signs: confirm both wheels'
    encoder deltas move in OPPOSITE directions (a differential pivot),
    matching the sim harness's own "wheels variant drives correct signs"
    scenario."""
    _drain(proto)
    enc_before = None
    for f in _watch(proto, 0.1):
        if f.enc is not None:
            enc_before = f.enc
    move_id = _next_move_id()
    corr = proto.move_wheels(v_left=150.0, v_right=-150.0, stop_time=600.0,
                             timeout=1500.0, replace=True, move_id=move_id)
    ack = proto.wait_for_ack(corr, timeout=ACK_TIMEOUT)
    result.record("wheels MOVE enqueue ack ok", ack is not None and ack.ok, f"ack={ack}")

    enc_after = enc_before
    for f in _watch(proto, 0.9):
        if f.enc is not None:
            enc_after = f.enc
    if enc_before is not None and enc_after is not None:
        d_left = enc_after[0] - enc_before[0]
        d_right = enc_after[1] - enc_before[1]
        opposite = (d_left > 0 and d_right < 0) or (d_left < 0 and d_right > 0)
        result.record("wheels MOVE drove the two wheels in opposite directions",
                      opposite, f"before={enc_before} after={enc_after} "
                                f"d_left={d_left} d_right={d_right}")
    else:
        result.record("wheels MOVE drove the two wheels in opposite directions",
                      False, "no encoder frames observed")
    proto.estop()


def scenario_chaining_seamless(proto: NezhaProtocol, result: Result) -> None:
    """Chain B (replace=False) behind active A: confirm A's own completion
    ack lands, B activates, `active` stays continuously True across the
    handoff (no intervening idle cycle), and B's own completion ack lands
    afterward."""
    _drain(proto)
    move_a = _next_move_id()
    move_b = _next_move_id()

    corr_a = proto.move_twist(v_x=150.0, v_y=0.0, omega=0.0, stop_time=1000.0,
                              timeout=2000.0, replace=True, move_id=move_a)
    ack_a = proto.wait_for_ack(corr_a, timeout=ACK_TIMEOUT)
    result.record("chain: Move A enqueue ack ok", ack_a is not None and ack_a.ok, f"ack={ack_a}")

    corr_b = proto.move_twist(v_x=150.0, v_y=0.0, omega=0.0, stop_time=700.0,
                              timeout=2000.0, replace=False, move_id=move_b)
    ack_b = proto.wait_for_ack(corr_b, timeout=ACK_TIMEOUT)
    result.record("chain: Move B enqueue ack ok (queued behind A)",
                  ack_b is not None and ack_b.ok, f"ack={ack_b}")

    active_gap_seen = [False]
    saw_active = [False]

    def _watch_active(f: TLMFrame) -> None:
        if f.active is False and saw_active[0]:
            # only counts as a "gap" if it happens BEFORE B's own completion
            active_gap_seen[0] = active_gap_seen[0] or True
        if f.active:
            saw_active[0] = True

    frames = _watch(proto, 2.2, on_frame=lambda f: None)
    ack_complete_a = _find_completion_ack(frames, move_a)
    ack_complete_b = _find_completion_ack(frames, move_b)
    # Seamless handoff: between A's completion frame and B's activation there
    # must be no frame reporting active=False.
    idx_a = frames.index(ack_complete_a) if ack_complete_a in frames else None
    gap = False
    if idx_a is not None:
        for f in frames[idx_a:idx_a + 3]:
            if f.active is False and f is not ack_complete_a:
                gap = True
    result.record("chain: Move A completion ack observed", ack_complete_a is not None,
                  f"move_id={move_a}")
    result.record("chain: Move B completion ack observed (seamless handoff)",
                  ack_complete_b is not None, f"move_id={move_b}")
    result.record("chain: no idle gap between A ending and B activating", not gap,
                  f"gap_detected={gap}")
    proto.estop()


def scenario_replace_preempts(proto: NezhaProtocol, result: Result) -> None:
    """replace=True mid-motion: A is long-running; B (replace=True) preempts
    it before A's own stop condition fires. A's completion ack must NEVER
    appear (flushed, not completed -- protocol-v4.md Sec 5.3); B's own
    completion ack must appear when B ends."""
    _drain(proto)
    move_a = _next_move_id()
    move_b = _next_move_id()

    corr_a = proto.move_twist(v_x=150.0, v_y=0.0, omega=0.0, stop_time=3000.0,
                              timeout=4000.0, replace=True, move_id=move_a)
    ack_a = proto.wait_for_ack(corr_a, timeout=ACK_TIMEOUT)
    result.record("preempt: Move A enqueue ack ok", ack_a is not None and ack_a.ok, f"ack={ack_a}")

    _watch(proto, 0.5)  # let A run for a bit

    corr_b = proto.move_twist(v_x=150.0, v_y=0.0, omega=0.0, stop_time=600.0,
                              timeout=1500.0, replace=True, move_id=move_b)
    ack_b = proto.wait_for_ack(corr_b, timeout=ACK_TIMEOUT)
    result.record("preempt: Move B (replace=True) enqueue ack ok",
                  ack_b is not None and ack_b.ok, f"ack={ack_b}")

    frames = _watch(proto, 1.2)
    ack_complete_a = _find_completion_ack(frames, move_a)
    ack_complete_b = _find_completion_ack(frames, move_b)
    result.record("preempt: Move A completion ack NEVER appears (flushed, not completed)",
                  ack_complete_a is None, f"move_id={move_a}")
    result.record("preempt: Move B completion ack observed", ack_complete_b is not None,
                  f"move_id={move_b}")
    proto.estop()


# ---------------------------------------------------------------------------
# 127-001: replace-preemption characterization, five cases (sprint 127's
# design issue investigation, "Finding 1" -- clasi/sprints/127-host-side-
# path-planner-goto-and-path-following/issues/sprint-127-host-side-path-
# planner-goto-path-following.md). These five scenarios run on the STAND
# (wheels free), NOT the playfield -- see the ticket's own "Safety
# obligations" section. Every scenario below calls estop() (never stop()) on
# exit, in a way that survives an early return, matching the rest of this
# file's convention. Bench-side measurement uses `TLMFrame.vel` (ALWAYS
# present, ACTUAL measured per-wheel velocity, mm/s) as the discontinuity
# signal, not `TLMFrame.cmd_vel` (a "permanent gap, TelemetrySecondary only"
# field per protocol.py's own TLMFrame docstring -- not reliably present in
# every short post-replace window) -- this is a deliberate, DIFFERENT
# measurement from the sim-tier harness's own `Planner::commandedLeft()/
# commandedRight()` (the raw profiled COMMAND, before PID/actuation lag):
# the sim tier isolates the profiler's own math (Edge B/Edge A's actual
# mechanism); this bench tier observes what the real plant's closed loop
# does in response, PID lag and all. Both numbers matter and are reported
# separately -- see the ticket's own Completion Notes for how they compare.
BENCH_TRACK_WIDTH_MM = 128.0  # [mm] data/robots/tovez.json's own caliper-measured trackwidth


def _last_vel(frames: list[TLMFrame]) -> tuple[int, int] | None:
    for f in reversed(frames):
        if f.vel is not None:
            return f.vel
    return None


def _watch_bracket(proto: NezhaProtocol, pre: float, send_fn, post: float) -> list[TLMFrame]:  # [s] [s]
    """Drain telemetry for `pre` seconds, call `send_fn()` (the replace
    itself), then keep draining for `post` more seconds -- returning ONE
    chronologically continuous frame list spanning the replace instant.

    This exists because a "watch a window, take the LAST frame" sample
    (this file's older `_last_vel(_watch(...))` idiom, still used by
    `scenario_distance_stop`/etc. for steady-state poses) DILUTES a
    single-tick transient: telemetry lands roughly every ~44ms (close to
    but not locked to the firmware's own 40ms control cycle), so a 0.2-0.3s
    "before"/"after" window spans 5-7 cycles -- by the time the LAST frame
    in the "after" window is read, the wheel-velocity PID has already
    closed most of the gap toward its new target, so "before vs. last-after"
    understates the actual step at replace. `_max_consecutive_vel_step()`
    below instead scans EVERY chronologically adjacent frame pair in the
    bracketed sequence and reports the single largest one -- the closest a
    ~25Hz wire observer can get to "the commanded discontinuity at the
    instant of replace," the same concept the sim-tier harness measures
    directly off `Planner::commandedLeft()/commandedRight()`."""
    frames: list[TLMFrame] = []
    deadline = time.monotonic() + pre
    while time.monotonic() < deadline:
        frames.extend(proto.read_pending_binary_tlm_frames())
        time.sleep(0.01)
    send_fn()
    deadline = time.monotonic() + post
    while time.monotonic() < deadline:
        frames.extend(proto.read_pending_binary_tlm_frames())
        time.sleep(0.01)
    return frames


def _max_consecutive_vel_step(frames: list[TLMFrame]) -> tuple[int, tuple[int, int] | None, tuple[int, int] | None]:
    """Scan `frames` (assumed chronologically ordered) for the largest
    per-wheel step between any two CONSECUTIVE `vel` readings -- see
    `_watch_bracket()`'s own docstring for why this, not a "first vs last"
    diff, is the right bench analog of a single-tick command discontinuity.
    Returns (max_step, frame_pair_before, frame_pair_after); (0, None, None)
    if fewer than two `vel`-bearing frames were observed."""
    max_step = 0
    prev: tuple[int, int] | None = None
    pair: tuple[tuple[int, int] | None, tuple[int, int] | None] = (None, None)
    for f in frames:
        if f.vel is None:
            continue
        if prev is not None:
            step = max(abs(f.vel[0] - prev[0]), abs(f.vel[1] - prev[1]))
            if step > max_step:
                max_step = step
                pair = (prev, f.vel)
        prev = f.vel
    return max_step, pair[0], pair[1]


def scenario_replace_same_curvature_at_speed(proto: NezhaProtocol, result: Result) -> None:
    """Case 1 (baseline): replace a Linear move with another Linear move at
    the SAME unitLeft/unitRight ratio (a straight, v_x=150) while at speed.
    Expect profile continuity -- the measured per-wheel velocity step across
    the replace instant should stay within ordinary PID/measurement noise,
    not show a step matching Case 2's Edge B magnitude. Uses
    `_watch_bracket()`/`_max_consecutive_vel_step()` (not a "before vs.
    long-window-later" diff) so the measurement isn't diluted by the
    wheel-velocity PID closing the gap before the "after" sample is taken."""
    _drain(proto)
    move_a = _next_move_id()
    corr_a = proto.move_twist(v_x=150.0, v_y=0.0, omega=0.0, stop_time=3000.0,
                              timeout=4000.0, replace=True, move_id=move_a)
    ack_a = proto.wait_for_ack(corr_a, timeout=ACK_TIMEOUT)
    result.record("case1: Move A enqueue ack ok", ack_a is not None and ack_a.ok, f"ack={ack_a}")

    _watch(proto, 1.0)  # let A reach cruise

    move_b = _next_move_id()

    def _send_b() -> None:
        corr_b = proto.move_twist(v_x=150.0, v_y=0.0, omega=0.0, stop_time=1500.0,
                                  timeout=2500.0, replace=True, move_id=move_b)
        ack_b = proto.wait_for_ack(corr_b, timeout=ACK_TIMEOUT)
        result.record("case1: Move B (same curvature, replace=True) enqueue ack ok",
                      ack_b is not None and ack_b.ok, f"ack={ack_b}")

    frames = _watch_bracket(proto, pre=0.3, send_fn=_send_b, post=0.5)
    step, before, after = _max_consecutive_vel_step(frames)
    print(f"  CASE1_SAME_CURVATURE_STEP_MM_S={step}  (largest consecutive-frame step: {before}->{after}, "
          f"{len(frames)} frames bracketed)")
    result.record("case1: largest consecutive-frame velocity step across the replace stays small "
                   "(<=60mm/s, generous over real PID/actuation noise)",
                  before is not None and step <= 60, f"step={step}mm/s")
    proto.estop()


def scenario_replace_edge_b_curvature_step(proto: NezhaProtocol, result: Result) -> None:
    """Case 2 (Edge B) -- THE bench-side counterpart of the number ticket
    005 consumes. Replace a straight (axisPerLambda=1.0) with a tight arc
    (unitLeft=-0.5, unitRight=1.0, axisPerLambda=0.25) at the SAME
    dominant-wheel peak speed (150mm/s) while cruising. Measures the
    largest per-wheel velocity step between any two chronologically
    adjacent telemetry frames bracketing the replace instant (see
    `_watch_bracket()`'s own docstring for why this, not a "before vs.
    long-window-later" diff, is the right bench analog of the sim-tier
    harness's raw-command discontinuity measurement)."""
    _drain(proto)
    move_a = _next_move_id()
    corr_a = proto.move_twist(v_x=150.0, v_y=0.0, omega=0.0, stop_time=3000.0,
                              timeout=4000.0, replace=True, move_id=move_a)
    ack_a = proto.wait_for_ack(corr_a, timeout=ACK_TIMEOUT)
    result.record("case2 (Edge B): straight Move A enqueue ack ok", ack_a is not None and ack_a.ok,
                  f"ack={ack_a}")

    _watch(proto, 1.0)  # let A reach cruise

    # vRight=150 (dominant, peak), vLeft=-75 -- unitLeft=-0.5, unitRight=1.0.
    v_right = 150.0
    v_left = -0.5 * 150.0
    arc_v_x = 0.5 * (v_left + v_right)
    arc_omega = (v_right - v_left) / BENCH_TRACK_WIDTH_MM
    move_b = _next_move_id()

    def _send_b() -> None:
        corr_b = proto.move_twist(v_x=arc_v_x, v_y=0.0, omega=arc_omega, stop_distance=100.0,
                                  timeout=2500.0, replace=True, move_id=move_b)
        ack_b = proto.wait_for_ack(corr_b, timeout=ACK_TIMEOUT)
        result.record("case2 (Edge B): tight-arc Move B (replace=True) enqueue ack ok",
                      ack_b is not None and ack_b.ok, f"ack={ack_b}")

    frames = _watch_bracket(proto, pre=0.3, send_fn=_send_b, post=0.5)
    step, before, after = _max_consecutive_vel_step(frames)
    # ===== THE labeled number: bench-measured counterpart to ticket 005's own Edge B figure. =====
    print(f"  CASE2_EDGE_B_BENCH_STEP_MM_S={step}  (largest consecutive-frame step: {before}->{after}, "
          f"{len(frames)} frames bracketed)")
    # =================================================================================================
    # No pass/fail threshold on the SIZE (same rationale as the sim
    # harness's own Case 2) -- just confirm it is a real measurement.
    result.record("case2 (Edge B): a real velocity step was measured across the replace",
                  before is not None, f"step={step}mm/s frames={len(frames)}")
    proto.estop()


def scenario_replace_edge_a_axis_change_at_speed(proto: NezhaProtocol, result: Result) -> None:
    """Case 3 (Edge A) -- replace a Distance-stopped (Linear axis) move with
    an Angle-stopped (Angular axis) move while at speed. The sim-tier
    harness shows the trim/PID integrator carries across this replace
    (unlike an axis change from rest, Case 4) -- this bench scenario
    measures the resulting transient wheel-speed error against the robot's
    OWN live-configured PID/trim gains (data/robots/tovez.json), not an
    injected test gain."""
    _drain(proto)
    move_a = _next_move_id()
    corr_a = proto.move_twist(v_x=150.0, v_y=0.0, omega=0.0, stop_time=3000.0,
                              timeout=4000.0, replace=True, move_id=move_a)
    ack_a = proto.wait_for_ack(corr_a, timeout=ACK_TIMEOUT)
    result.record("case3 (Edge A): straight Move A enqueue ack ok", ack_a is not None and ack_a.ok,
                  f"ack={ack_a}")

    _watch(proto, 1.5)  # let A settle at cruise long enough for any trim integral to build

    move_b = _next_move_id()

    def _send_b() -> None:
        corr_b = proto.move_twist(v_x=0.0, v_y=0.0, omega=2.0, stop_angle=3.14159265,
                                  timeout=3000.0, replace=True, move_id=move_b)
        ack_b = proto.wait_for_ack(corr_b, timeout=ACK_TIMEOUT)
        result.record("case3 (Edge A): Angle Move B (replace=True, axis change) enqueue ack ok",
                      ack_b is not None and ack_b.ok, f"ack={ack_b}")

    # The commanded per-wheel speed the new turn itself asks for --
    # omega * trackWidth/2 -- is the yardstick the sim harness's own verdict
    # uses; recompute it here for a directly comparable bench verdict. NOTE:
    # this is also, unavoidably, close to the ordinary ramp-from-zero
    # magnitude every fresh axis change shows (the sim harness's own case 4
    # finding: profileVelocity_ resets to 0 on ANY axis change, at speed or
    # from rest) -- see this scenario's own printed comparison against
    # scenario_replace_axis_change_from_rest's identical metric below,
    # which is what actually isolates the mid-motion-specific carryover the
    # sim's trim-integral check found (not observable directly over the
    # wire -- no cmd_vel-equivalent trim telemetry field exists).
    commanded_wheel_speed = 2.0 * BENCH_TRACK_WIDTH_MM * 0.5

    frames = _watch_bracket(proto, pre=0.2, send_fn=_send_b, post=0.6)
    step, before, after = _max_consecutive_vel_step(frames)
    peak_dev = 0.0
    for f in frames:
        if f.vel is None:
            continue
        dev = max(abs(abs(f.vel[0]) - commanded_wheel_speed), abs(abs(f.vel[1]) - commanded_wheel_speed))
        peak_dev = max(peak_dev, dev)

    # ===== THE labeled numbers: Case 3's bench-measured transient wheel-speed error. =====
    hazardous = peak_dev > commanded_wheel_speed
    print(f"  CASE3_EDGE_A_BENCH_TRANSIENT_ERROR_MM_S={peak_dev}  vs commanded_wheel_speed="
          f"{commanded_wheel_speed}  VERDICT={'HAZARDOUS' if hazardous else 'BENIGN'}")
    print(f"  CASE3_EDGE_A_BENCH_MAX_CONSECUTIVE_STEP_MM_S={step}  ({before}->{after}, "
          f"{len(frames)} frames bracketed)")
    # =========================================================================================
    result.record("case3 (Edge A): transient wheel-speed error measured and printed",
                  len(frames) > 0, f"peak_dev={peak_dev}mm/s frames={len(frames)}")
    proto.estop()


def scenario_replace_axis_change_from_rest(proto: NezhaProtocol, result: Result) -> None:
    """Case 4 (sanity) -- same shape as Case 3 but Move A is allowed to
    COMPLETE and drain to rest first (the sanctioned "stop, then turn"
    path), rather than being replaced mid-motion. Expect the same
    measurement as Case 3 to come back materially smaller -- the sim-tier
    harness's own structural finding (coming to rest resets the trim
    integrator; a mid-motion replace does not) predicts this."""
    _drain(proto)
    move_a = _next_move_id()
    corr_a = proto.move_twist(v_x=150.0, v_y=0.0, omega=0.0, stop_time=600.0,
                              timeout=1500.0, replace=True, move_id=move_a)
    ack_a = proto.wait_for_ack(corr_a, timeout=ACK_TIMEOUT)
    result.record("case4: short straight Move A enqueue ack ok", ack_a is not None and ack_a.ok,
                  f"ack={ack_a}")

    frames_a = _watch(proto, 1.2)
    completion_a = _find_completion_ack(frames_a, move_a)
    result.record("case4: Move A completes on its own before the axis change",
                  completion_a is not None, f"move_id={move_a}")
    _watch(proto, 0.3)  # a few more settle cycles

    move_b = _next_move_id()

    def _send_b() -> None:
        corr_b = proto.move_twist(v_x=0.0, v_y=0.0, omega=2.0, stop_angle=3.14159265,
                                  timeout=3000.0, replace=False, move_id=move_b)  # from rest -- replace doesn't matter
        ack_b = proto.wait_for_ack(corr_b, timeout=ACK_TIMEOUT)
        result.record("case4: Angle Move B (from rest) enqueue ack ok", ack_b is not None and ack_b.ok,
                      f"ack={ack_b}")

    commanded_wheel_speed = 2.0 * BENCH_TRACK_WIDTH_MM * 0.5
    frames = _watch_bracket(proto, pre=0.2, send_fn=_send_b, post=0.6)
    step, before, after = _max_consecutive_vel_step(frames)
    peak_dev = 0.0
    for f in frames:
        if f.vel is None:
            continue
        dev = max(abs(abs(f.vel[0]) - commanded_wheel_speed), abs(abs(f.vel[1]) - commanded_wheel_speed))
        peak_dev = max(peak_dev, dev)

    print(f"  CASE4_FROM_REST_BENCH_TRANSIENT_ERROR_MM_S={peak_dev}  vs commanded_wheel_speed="
          f"{commanded_wheel_speed}  (compare against CASE3_EDGE_A_BENCH_TRANSIENT_ERROR_MM_S above)")
    print(f"  CASE4_FROM_REST_BENCH_MAX_CONSECUTIVE_STEP_MM_S={step}  ({before}->{after}, "
          f"{len(frames)} frames bracketed; compare against CASE3_EDGE_A_BENCH_MAX_CONSECUTIVE_STEP_MM_S)")
    result.record("case4: transient wheel-speed error measured and printed", len(frames) > 0,
                  f"peak_dev={peak_dev}mm/s frames={len(frames)}")
    proto.estop()


def scenario_replace_high_rate(proto: NezhaProtocol, result: Result) -> None:
    """Case 5 -- issue replace=True twists at ~20Hz (faster than the
    ~1-tick activation latency) for >=5s. Measures the largest step between
    consecutive OBSERVED per-wheel velocities, and confirms every single
    enqueue ack lands OK (never ERR_FULL) -- the wire-visible evidence that
    the planner queue never grows past 1 (active-only, no pending pile-up)
    even under replacements arriving faster than they drain."""
    _drain(proto)
    max_step = 0
    all_acked_ok = True
    prev_vel: tuple[int, int] | None = None
    deadline = time.monotonic() + 5.5  # >= 5s
    move_ids: list[int] = []
    i = 0
    while time.monotonic() < deadline:
        omega = 0.3 * math.sin(i * 0.2)
        move_id = _next_move_id()
        move_ids.append(move_id)
        corr = proto.move_twist(v_x=150.0, v_y=0.0, omega=omega, stop_time=5000.0,
                                timeout=6000.0, replace=True, move_id=move_id)
        ack = proto.wait_for_ack(corr, timeout=ACK_TIMEOUT)
        if ack is None or not ack.ok:
            all_acked_ok = False
        for f in _watch(proto, 0.05):  # ~20Hz cadence
            if f.vel is not None:
                if prev_vel is not None:
                    step = max(abs(f.vel[0] - prev_vel[0]), abs(f.vel[1] - prev_vel[1]))
                    max_step = max(max_step, step)
                prev_vel = f.vel
        i += 1

    print(f"  CASE5_HIGH_RATE_BENCH_MAX_STEP_MM_S={max_step}  "
          f"(ran {i} replacements over {time.monotonic() - (deadline - 5.5):.1f}s at ~20Hz)")
    result.record("case5: every replace=True enqueue acked OK (never ERR_FULL) across the 20Hz run",
                  all_acked_ok, f"replacements_sent={i}")
    proto.estop()


def scenario_err_full(proto: NezhaProtocol, result: Result) -> None:
    """1 active + 4 pending; a 5th pending enqueue (replace=False) must be
    rejected ERR_FULL (envelope.proto ErrCode value 4)."""
    _drain(proto)
    active_id = _next_move_id()
    corr = proto.move_twist(v_x=60.0, v_y=0.0, omega=0.0, stop_time=6000.0,
                            timeout=7000.0, replace=True, move_id=active_id)
    ack = proto.wait_for_ack(corr, timeout=ACK_TIMEOUT)
    result.record("ERR_FULL: active Move enqueue ack ok", ack is not None and ack.ok, f"ack={ack}")

    pending_ids = []
    all_pending_ok = True
    for i in range(4):
        pid = _next_move_id()
        pending_ids.append(pid)
        corr = proto.move_twist(v_x=60.0, v_y=0.0, omega=0.0, stop_time=200.0,
                                timeout=1000.0, replace=False, move_id=pid)
        ack = proto.wait_for_ack(corr, timeout=ACK_TIMEOUT)
        ok = ack is not None and ack.ok
        all_pending_ok = all_pending_ok and ok
        result.record(f"ERR_FULL: pending Move #{i + 1}/4 enqueue ack ok", ok, f"ack={ack}")

    fifth_id = _next_move_id()
    corr = proto.move_twist(v_x=60.0, v_y=0.0, omega=0.0, stop_time=200.0,
                            timeout=1000.0, replace=False, move_id=fifth_id)
    ack = proto.wait_for_ack(corr, timeout=ACK_TIMEOUT)
    result.record("ERR_FULL: 5th pending Move rejected with ERR_FULL (err_code=4)",
                  ack is not None and not ack.ok and ack.err_code == 4, f"ack={ack}")

    # Clean up immediately -- no need to let the whole 6s+4x200ms chain play
    # out; STOP flushes the active Move and every pending slot at once.
    stop_corr = proto.estop()
    stop_ack = proto.wait_for_ack(stop_corr, timeout=ACK_TIMEOUT)
    result.record("ERR_FULL: STOP cleanup ack ok", stop_ack is not None and stop_ack.ok,
                  f"ack={stop_ack}")


def scenario_empty_queue_drain_no_traffic(proto: NezhaProtocol, result: Result) -> None:
    """A single short Move, no follow-up command at all: confirm the queue
    drains to empty (active flag False, completion ack observed), then the
    robot stays stopped with ZERO further host traffic for several seconds
    -- the no-deadman structural safety property (SUC-053)."""
    _drain(proto)
    move_id = _next_move_id()
    corr = proto.move_twist(v_x=150.0, v_y=0.0, omega=0.0, stop_time=400.0,
                            timeout=1000.0, replace=True, move_id=move_id)
    ack = proto.wait_for_ack(corr, timeout=ACK_TIMEOUT)
    result.record("drain: Move enqueue ack ok", ack is not None and ack.ok, f"ack={ack}")

    frames = _watch(proto, 1.0)
    completion = _find_completion_ack(frames, move_id)
    # Anchor the settle check to the completion-ack FRAME itself (and every
    # frame captured after it), not to an arbitrary wall-clock window edge --
    # a fixed-duration watch window can close a frame or two "behind" the
    # robot's own clock under host-side scheduling jitter, which would
    # otherwise read as a false failure even though the robot's own active
    # flag already settled.
    settled = None
    if completion is not None:
        idx = frames.index(completion)
        settled = all(f.active is False for f in frames[idx:])
    result.record("drain: Move completion ack observed", completion is not None,
                  f"move_id={move_id}")
    result.record("drain: active flag false at and after completion (queue empty -> Drive::stop())",
                  settled is True, f"settled={settled}")

    # No further host command sent AT ALL during this window -- the whole
    # point of the check.
    enc_before_silence = None
    for f in reversed(frames):
        if f.enc is not None:
            enc_before_silence = f.enc
            break
    silent_frames = _watch(proto, 2.5)
    enc_after_silence = enc_before_silence
    ever_active_again = False
    for f in silent_frames:
        if f.enc is not None:
            enc_after_silence = f.enc
        if f.active:
            ever_active_again = True
    result.record("drain: motors stay stopped with zero further host traffic",
                  not ever_active_again,
                  f"enc_before={enc_before_silence} enc_after={enc_after_silence} "
                  f"frames_observed={len(silent_frames)}")


def scenario_timeout_fault(proto: NezhaProtocol, result: Result) -> None:
    """A DISTANCE-stop MOVE with commanded velocity 0 can never progress --
    the required `timeout` backstop must fire instead. Safe by construction
    (nothing is ever commanded to move, so this needs no live motor bus and
    does NOT physically obstruct the wheels)."""
    _drain(proto)
    move_id = _next_move_id()
    corr = proto.move_wheels(v_left=0.0, v_right=0.0, stop_distance=100.0,
                             timeout=1200.0, replace=True, move_id=move_id)
    ack = proto.wait_for_ack(corr, timeout=ACK_TIMEOUT)
    result.record("timeout-fault: zero-velocity distance MOVE enqueue ack ok",
                  ack is not None and ack.ok, f"ack={ack}")

    frames = _watch(proto, 1.8)
    completion = _find_completion_ack(frames, move_id)
    completion_entry = _find_ack_entry(frames, move_id)
    timed_out = any(f.fault_move_timeout for f in frames)
    completion_ack_err = completion_entry.err_code if completion_entry is not None else None
    result.record("timeout-fault: completion ack observed (Move.id)", completion is not None,
                  f"move_id={move_id}")
    result.record("timeout-fault: kFlagFaultMoveTimeout (flags bit 15) set", timed_out,
                  f"seen={timed_out}")
    result.record("timeout-fault: completion ack_err == 0 (AS-BUILT: timeout signaled via "
                  "flags bit, not ack_err)", completion_ack_err == 0,
                  f"ack_err={completion_ack_err}")


def scenario_stop_mid_motion(proto: NezhaProtocol, result: Result) -> None:
    """STOP while a Move is active AND a Move is pending behind it: confirm
    immediate halt (velocity trends to ~0 within ~1-2 frames) and the
    pending Move is flushed (never activates)."""
    _drain(proto)
    move_a = _next_move_id()
    move_b = _next_move_id()

    corr_a = proto.move_twist(v_x=150.0, v_y=0.0, omega=0.0, stop_time=3000.0,
                              timeout=4000.0, replace=True, move_id=move_a)
    ack_a = proto.wait_for_ack(corr_a, timeout=ACK_TIMEOUT)
    result.record("stop-mid-motion: Move A enqueue ack ok", ack_a is not None and ack_a.ok,
                  f"ack={ack_a}")
    corr_b = proto.move_twist(v_x=150.0, v_y=0.0, omega=0.0, stop_time=600.0,
                              timeout=1500.0, replace=False, move_id=move_b)
    ack_b = proto.wait_for_ack(corr_b, timeout=ACK_TIMEOUT)
    result.record("stop-mid-motion: Move B (pending) enqueue ack ok",
                  ack_b is not None and ack_b.ok, f"ack={ack_b}")

    _watch(proto, 0.4)  # let A actually be underway

    stop_sent_at = time.monotonic()
    stop_corr = proto.estop()
    stop_frames: list[TLMFrame] = []

    def _record(f: TLMFrame) -> None:
        stop_frames.append(f)

    frames_after = _watch(proto, 1.0, on_frame=_record)
    stop_ack = _find_ack_entry(frames_after, stop_corr)
    result.record("stop-mid-motion: STOP command ack ok", stop_ack is not None and stop_ack.ok,
                  f"ack={stop_ack}")

    # Anchor to the STOP ack's own frame (and everything captured after it),
    # not an arbitrary wall-clock point relative to when stop() was called --
    # see scenario_empty_queue_drain_no_traffic()'s identical rationale.
    active_after_stop = None
    stop_ack_frame = next((f for f in frames_after if any(e.corr_id == stop_corr for e in f.acks)), None)
    if stop_ack_frame is not None:
        idx = frames_after.index(stop_ack_frame)
        active_after_stop = any(f.active for f in frames_after[idx:])
    completion_b = _find_completion_ack(frames_after, move_b)
    result.record("stop-mid-motion: active flag never True again after STOP's own ack frame",
                  active_after_stop is False, f"elapsed_since_stop={time.monotonic() - stop_sent_at:.2f}s")
    result.record("stop-mid-motion: pending Move B never activates (flushed by STOP)",
                  completion_b is None, f"move_id={move_b}")


def scenario_config_mid_move(proto: NezhaProtocol, result: Result) -> None:
    """A CONFIG patch arriving mid-Move must not disturb the active Move's
    staged velocity or its own completion -- protocol-v4.md Sec 6."""
    _drain(proto)
    move_id = _next_move_id()
    corr_move = proto.move_twist(v_x=150.0, v_y=0.0, omega=0.0, stop_time=1500.0,
                                 timeout=2500.0, replace=True, move_id=move_id)
    ack_move = proto.wait_for_ack(corr_move, timeout=ACK_TIMEOUT)
    result.record("config-mid-move: Move enqueue ack ok", ack_move is not None and ack_move.ok,
                  f"ack={ack_move}")

    # Sprint 132-012 deleted NezhaProtocol.config(**{"pid.kp": ...}) in favour
    # of the schema-addressed set_config_field(target, field_name, value),
    # which returns its AckEntry directly rather than a corr_id to wait on.
    # This scenario still called the deleted method and died on an
    # AttributeError before reaching any of its own assertions.
    from robot_radio.robot.pb2 import robot_config_pb2 as pb

    _watch(proto, 0.3)
    ack_cfg = proto.set_config_field(pb.WHEEL_CONTROL, "pid_kp", 0.0025)
    result.record("config-mid-move: CONFIG patch ack ok (not ERR_UNIMPLEMENTED)",
                  ack_cfg is not None and ack_cfg.ok, f"ack={ack_cfg}")

    frames = _watch(proto, 1.5)
    completion = _find_completion_ack(frames, move_id)
    result.record("config-mid-move: the SAME Move still completes normally afterward",
                  completion is not None, f"move_id={move_id}")

    # Restore the default kp before continuing (sprint 115 checklist
    # precedent -- don't leave the robot detuned for later scripts).
    restore_ack = proto.set_config_field(pb.WHEEL_CONTROL, "pid_kp", 0.002)
    result.record("config-mid-move: restored default pid.kp=0.002",
                  restore_ack is not None and restore_ack.ok, f"ack={restore_ack}")


SCENARIOS = [
    scenario_distance_stop,
    scenario_angle_stop,
    scenario_wheels_variant_signs,
    scenario_chaining_seamless,
    scenario_replace_preempts,
    scenario_replace_same_curvature_at_speed,
    scenario_replace_edge_b_curvature_step,
    scenario_replace_edge_a_axis_change_at_speed,
    scenario_replace_axis_change_from_rest,
    scenario_replace_high_rate,
    scenario_err_full,
    scenario_empty_queue_drain_no_traffic,
    scenario_timeout_fault,
    scenario_stop_mid_motion,
    scenario_config_mid_move,
]


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=DEFAULT_PORT)
    args = p.parse_args()

    conn = SerialConnection(port=args.port)
    info = conn.connect()
    if info.get("status") != "connected":
        print(f"ERROR: connect failed: {info}")
        return 2
    proto = NezhaProtocol(conn)
    print(f"connected: port={args.port} mode={info.get('mode')}")

    # 125-001: wait for the robot to actually be ready before sending the
    # first real command -- connect() itself only proves the link answers
    # HELLO/PING, not that RobotLoop::boot() has finished probing every
    # device. 125-005: readiness is now connect()'s own job (the READY
    # line); see _wait_for_boot_ready()'s own doc comment.
    if _wait_for_boot_ready(info):
        print("boot_ready observed -- robot is ready for the first MOVE")
    else:
        print("WARNING: boot-ready (READY line) not observed by connect() -- "
              "proceeding anyway (scenario_distance_stop's own checks will "
              "surface whatever the robot actually does)")

    # kAuto (the default emit policy) keeps a parked robot quiet -- switch
    # to streaming-always so _watch()/_drain() actually see frames even
    # before the first Move starts (127-001 bench pass, 2026-07-30: a
    # parked robot on the stand answered HELLO/PING/READY fine but emitted
    # zero unsolicited telemetry until commanded).
    proto.tlmOn()

    result = Result()
    try:
        for scenario in SCENARIOS:
            print(f"\n--- {scenario.__name__} ---")
            scenario(proto, result)
    finally:
        try:
            proto.estop()
        except Exception:
            pass
        try:
            proto.tlmOff()
        except Exception:
            pass
        conn.disconnect()

    return 0 if result.ok() else 1


if __name__ == "__main__":
    sys.exit(main())
