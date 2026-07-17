---
id: '003'
title: Fix Sim speed-up factor stutter/breakage at 10x/20x
status: done
use-cases:
- SUC-003
depends-on: []
github-issue: ''
issue: testgui-speedup-factor-broken-at-high-values.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Fix Sim speed-up factor stutter/breakage at 10x/20x

## Description

The TestGUI's Sim speed-up selector offers five multipliers (1×, 2×, 5×,
10×, 20× — `__main__.py`'s `sim_speed_combo`). 2× and 5× work; 10× is
"herky-jerky" (doesn't actually run faster, just stutters); 20× is
broken (doesn't work at all).

**Sprint-planning-time finding — a hypothesis, not a confirmed
diagnosis.** Reading `src/host/robot_radio/io/sim_loop.py`'s
`_tick_loop()`: each iteration steps `cycles = max(1, int(speed_factor))`
sim cycles via one `sim_step()` call, then paces the WHOLE iteration to
fit a single `_CYCLE_DURATION_S` (50 ms) budget regardless of `cycles` —
per that method's own comment, once N cycles' compute exceeds the 50 ms
budget the loop intentionally "free-runs" with no sleep, which is stated
as the desired behavior for a fast tour. That pacing model looks correct
and intentional for the PHYSICS side. The more likely culprit is on the
CONSUMPTION side: at `cycles=10`/`cycles=20`, one iteration now delivers
up to 10-20 telemetry frames in a burst (`_drain_tlm_into_queue()` →
`on_telemetry`) instead of the usual one — and this project has a
documented gotcha (`pyside-queuedconnection-bare-function` memory note)
about `QueuedConnection` cross-thread delivery to the GUI: a bare-function
target runs on the WORKER thread, not the GUI thread, and even a correct
bound-method `QObject`-bridge target processes each queued frame as a
SEPARATE delivery — a burst of 10-20 in rapid succession could plausibly
overwhelm or serialize badly against the GUI's own redraw/event-loop
budget. **This is a hypothesis to confirm, not a diagnosis to implement
against directly.**

Per ticket 109-009's own precedent (six real bugs found by building a
real, deterministic reproduction harness before fixing anything, after a
plausible-but-wrong hypothesis would have wasted the fix): build a
deterministic (non-wall-clock-paced) test harness FIRST, using it to
either confirm the burst-delivery hypothesis above or find the real
mechanism, THEN fix.

## Acceptance Criteria

- [x] A deterministic sim-stepping test harness exists (modeled on ticket
      109-009's own `SimLoop`/`sim_ctypes.cpp` pattern — explicit
      `step(cycles)` calls, `start_tick_thread=False` or equivalent, no
      wall-clock racing) that can drive the sim at each of the five
      offered multipliers and measure actual cycles-advanced and/or
      frames-delivered per unit of test time.
- [x] The harness is used to confirm (or refute) the burst-telemetry-vs-
      `QueuedConnection`-bridge hypothesis above; the ticket records
      which mechanism was found to be the real cause, with evidence (not
      an assumption carried over from planning).
- [x] The real cause (whatever it turns out to be) is fixed such that all
      five offered multipliers (1×, 2×, 5×, 10×, 20×) advance the
      simulation smoothly and proportionally, with no stutter and no
      breakage at 10×/20×.
- [x] The deterministic harness becomes a permanent regression test —
      it must fail if the stutter/breakage is reintroduced.
- [x] No change to the underlying physics/trajectory integration — per
      the existing tooltip's own promise ("physics integration step is
      unchanged — trajectories are identical at every speed"), only the
      pacing/delivery mechanism is in scope.
- [x] Full `src/tests/testgui/` suite (and any `sim_loop.py`-adjacent
      tests) stays green.

## Findings (2026-07-17)

**Part A — burst-delivery premise, confirmed against the real compiled sim.**
Using `SimLoop.connect(start_tick_thread=False)` + explicit `step(cycles)`
calls (no wall-clock racing — new `test_sim_speed_factor.py`), one
iteration's worth of stepping at `cycles=N` delivers a burst of TLM frames
that grows with `N` (empirically ~0.8 frame/cycle at this firmware's own
STREAM period, e.g. 20 cycles → 16 frames in one drain), confirmed strictly
larger at 10x/20x than at 2x/5x. This matches `_tick_loop()`'s own per-
iteration `cycles = max(1, int(speed_factor))` step exactly — the burst-
delivery half of the sprint-planning hypothesis is real.

**Part B — the actual GUI-side mechanism (refined from the hypothesis).**
The hypothesis named "`QueuedConnection` bridge throughput" generically;
reading `__main__.py`'s `_TelemetryBridge.on_frame_ready` (the actual
consumer) found the PRECISE mechanism: every queued `TLMFrame` in a drained
burst triggered its own `canvas_ctrl.refresh()` call, called INSIDE the
per-frame while-loop, not once per drain. `canvas.py`'s
`CanvasController._update_traces()` (called by `refresh()`) REBUILDS all
four trace `QPainterPath`s from scratch every call — cost scales with the
total accumulated trace length, not O(1). Benchmarked directly (2000
accumulated trace points, matching a realistic mid-session length): one
`refresh()` call costs ~0.7-0.8ms; 16 back-to-back calls (the measured 20x
burst size) cost ~13ms — i.e. at 20x the GUI thread was asked to spend
~13ms on REDUNDANT, immediately-discarded intermediate redraws inside a
single ~50ms tick-thread iteration, and this cost only grows as a session's
trace history grows. At 2x/5x the burst (2-4 frames) makes this
imperceptible; at 10x/20x it is not — this exactly explains the reported
"herky-jerky at 10x / broken at 20x" shape. `sim_loop.py`'s own pacing
(`_tick_loop()`) is untouched and confirmed correct, matching the sprint-
planning-time reading. The rolling strip charts (110-002) and the trace
recorder's idle-freeze gate (110-001) both key off `time.monotonic()`
wall-clock timestamps, not sim time — verified this is the CORRECT choice
(an operator watching the screen cares about real elapsed wall time,
which is exactly what a fast-forwarded session should compress into), not
a second time-base bug.

**Fix.** `__main__.py::_TelemetryBridge.on_frame_ready` now feeds every
drained frame into `trace_model`/`graph_panel` (cheap, dirty-flag-gated
accumulation, unchanged), but calls `canvas_ctrl.refresh()` AT MOST ONCE
per drain — after the loop, using the last frame's state — instead of once
per frame. Final on-screen state after a burst is identical (drawn from the
same last-frame data); only the N-1 discarded intermediate redraws are
removed. No sim/physics change.

**Tests**: `src/tests/testgui/test_sim_speed_factor.py` (new — the
deterministic harness, Part A) and `src/tests/testgui/test_telemetry_gating.py`
(updated — its established closure-mirroring pattern now asserts exactly
one `refresh()` call per drained burst regardless of frame count; the
prior version asserted N calls for N frames, i.e. the bug itself).

## Testing

- **Existing tests to run**: `uv run python -m pytest` (full suite,
  especially any existing `sim_loop.py`/`SimTransport`/speed-factor
  tests); ticket 109-009's own `test_tour_closure_gate.py` (must remain
  green — this ticket must not regress tour-closure behavior while
  fixing speed-factor pacing).
- **New tests to write**: the deterministic multi-speed-factor harness
  itself (parameterized over 1×/2×/5×/10×/20×, asserting cycles-advanced
  scales with the selected factor); a telemetry-burst-delivery test if
  the hypothesis is confirmed (assert all N frames from a burst are
  correctly delivered/rendered, not dropped or serialized into a stutter).
- **Verification command**: `uv run python -m pytest src/tests/testgui/
  -k "speed_factor or sim_loop or speedup"`.

## Implementation Plan

**Approach**: Diagnose with a deterministic harness before touching any
production code — the sprint-planning-time hypothesis (burst delivery vs.
`QueuedConnection` bridge throughput) is plausible but unconfirmed; do
not implement a fix for it without first proving it's the real mechanism,
per ticket 109-009's own documented lesson about guessing at timing bugs.

**Files to investigate first**:
- `src/host/robot_radio/io/sim_loop.py` (`_tick_loop()`, `_drain_tlm_
  into_queue()`, `set_speed_factor()`)
- `src/host/robot_radio/testgui/transport.py` (`SimTransport`'s own
  `_tick_loop`/telemetry consumption, `set_speed_factor()`)
- Whichever `QueuedConnection` GUI-thread bridge currently receives
  `on_telemetry` deliveries (`__main__.py` — see the
  `pyside-queuedconnection-bare-function` project memory note for the
  established pattern and its own known failure modes)

**Files to modify**: determined by the harness's findings — not
predictable at planning time.

**Testing plan**: as above — harness first, confirm mechanism, fix,
permanent regression test.

**Documentation updates**: none in `src/firm/` (host-only ticket, no
wire-schema change).
