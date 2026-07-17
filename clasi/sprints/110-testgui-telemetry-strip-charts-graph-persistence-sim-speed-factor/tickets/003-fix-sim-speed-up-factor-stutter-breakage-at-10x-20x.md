---
id: '003'
title: Fix Sim speed-up factor stutter/breakage at 10x/20x
status: open
use-cases: [SUC-003]
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

- [ ] A deterministic sim-stepping test harness exists (modeled on ticket
      109-009's own `SimLoop`/`sim_ctypes.cpp` pattern — explicit
      `step(cycles)` calls, `start_tick_thread=False` or equivalent, no
      wall-clock racing) that can drive the sim at each of the five
      offered multipliers and measure actual cycles-advanced and/or
      frames-delivered per unit of test time.
- [ ] The harness is used to confirm (or refute) the burst-telemetry-vs-
      `QueuedConnection`-bridge hypothesis above; the ticket records
      which mechanism was found to be the real cause, with evidence (not
      an assumption carried over from planning).
- [ ] The real cause (whatever it turns out to be) is fixed such that all
      five offered multipliers (1×, 2×, 5×, 10×, 20×) advance the
      simulation smoothly and proportionally, with no stutter and no
      breakage at 10×/20×.
- [ ] The deterministic harness becomes a permanent regression test —
      it must fail if the stutter/breakage is reintroduced.
- [ ] No change to the underlying physics/trajectory integration — per
      the existing tooltip's own promise ("physics integration step is
      unchanged — trajectories are identical at every speed"), only the
      pacing/delivery mechanism is in scope.
- [ ] Full `src/tests/testgui/` suite (and any `sim_loop.py`-adjacent
      tests) stays green.

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
