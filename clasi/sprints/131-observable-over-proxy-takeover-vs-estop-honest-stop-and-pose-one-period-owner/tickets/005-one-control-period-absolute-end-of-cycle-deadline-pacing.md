---
id: '005'
title: 'One control period: absolute end-of-cycle deadline pacing'
status: open
use-cases: [SUC-131-005]
depends-on: ['004']
github-issue: ''
issue: A-nominal-50ms-vs-delivered-54ms.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# One control period: absolute end-of-cycle deadline pacing

## Description

Measured 130-011: the loop delivers 54.000 ms +/- 0.006 idle against
`kCycle = 50`. `cycleBusy` is only 21-23 ms, so this is not budget
overrun — it is structural: the four `runAndWait()` gaps
(`robot_loop.cpp:512-534`) sum to `kCycle`, but real work runs between the
marks and each block's `fiber_sleep` rounds up to a whole ms. This is not
new — `kCycle = 40` delivered 44 ms, the same +4. `App::Drive` integrates
with measured dt (`drive.cpp:298`, `state.time.cyclePeriod`);
`Motion::Planner`'s profile math and `cmdAccel` use the baked
`limits_.plant.controlPeriod` (`planner.cpp:1347`, currently fed from
`data/robots/*.json` and a stale "measured 47ms" rationale left over from
the 40ms-nominal generation, `boot_wiring.h:36`). That is a systematic ~8%
disagreement between two halves of the same control loop, and 130-011
bench-reproduced 8% as enough to flip `applyHeadingHold()` unstable.

Pick the honest fix: make the loop's final pacing block target an
ABSOLUTE end-of-cycle deadline (`state_.time.cycleStart + kCycle`) rather
than a relative gap from its own entry mark, so jitter/rounding
accumulated by the three earlier blocks is absorbed into the final wait
instead of compounding across all four. This was chosen over re-baking
`controlPeriod` to whatever the loop currently, empirically delivers —
that is literally the mechanism that produced this defect twice already
(the 40ms generation baked "47ms," and when nominal moved to 50ms nobody
re-measured it). The absolute-deadline approach is self-correcting under
jitter and is verifiable in a host-build unit test with an injected
jittery fake clock — no hardware required. Then correct
`data/robots/*.json`'s `controlPeriod` to 50 (matching `kCycle`) and the
stale "47 ms" comments (`main.cpp:80`, `boot_wiring.h:36`,
`robot_loop.h:49-57`) to state the new, true invariant, plus the
`Motion::Planner` comment near `update()`'s `dt` computation
(`planner.cpp` ~1347) noting the two now agree by construction.

**Explicitly declared bench debt**: this ticket proves the pacing logic
is self-correcting in a host-build unit test with a simulated jittery
clock. It does NOT re-measure the PHYSICAL robot's delivered period —
`tovez` is unreachable this sprint. Re-running
`src/tests/bench/planner_square_tour.py` on real hardware once the bench
is available is a required follow-up, not part of this ticket's
acceptance.

## Acceptance Criteria

- [ ] `App::RobotLoop::cycle()`'s final pacing block's wait targets
      `state_.time.cycleStart + kCycle` (an absolute deadline), not a
      relative gap from that block's own entry `markTime()`.
- [ ] Host-build unit test: an injected fake `Clock`/`Sleeper` simulating
      variable per-block overruns and whole-millisecond sleep rounding
      (matching the vendor-bus-clearance profile `robot_loop.h`'s own
      doc comments describe) shows the MEAN measured inter-cycle-start
      period converging to `kCycle` (50 ms) over many simulated cycles,
      not `kCycle` + a fixed offset.
- [ ] The existing "exactly four `sleepMillis()` calls per cycle"
      schedule-shape assertion (`sim_api_harness.cpp` or equivalent)
      still holds.
- [ ] `data/robots/*.json`'s `controlPeriod` field is corrected to 50 on
      every robot profile.
- [ ] Stale comments corrected: `main.cpp:80`'s "~47 ms budget",
      `boot_wiring.h:36`'s "measured 47ms instead of the nominal 40",
      `robot_loop.h:49-57`'s `kCycle` doc comment, all restated to
      reflect the new true invariant (delivered == nominal by
      construction).
- [ ] `Motion::Planner`'s comment near `update()`'s
      `dt = limits_.plant.controlPeriod * 0.001f` (`planner.cpp` ~1347) is
      updated to note `controlPeriod` and `App::Drive`'s measured
      `state.time.cyclePeriod` now agree by construction, rather than
      describing the two as an accepted disagreement.
- [ ] Explicitly declared in the ticket's own completion notes (not
      silently assumed): the PHYSICAL robot's delivered period is NOT
      re-measured this ticket — flagged as a required follow-up
      (`src/tests/bench/planner_square_tour.py` once `tovez` is
      reachable).
- [ ] No regression: the sim (which already steps at exactly `kCycle` per
      `SimHarness::step()`) shows unchanged behavior. Full sim suite
      stays green.

## Testing

- **Existing tests to run**: the schedule-shape assertion
  (`sim_api_harness.cpp` or equivalent), full `src/tests/sim` suite.
- **New tests to write**: host-build unit test with an injected fake
  `Clock`/`Sleeper` pair returning variable/jittery elapsed times per
  call, asserting the mean measured inter-cycle-start period over N
  simulated cycles.
- **Verification command**: `uv run python -m pytest src/tests/sim`; the
  new host-build unit test per its own build target.

## Implementation Plan

**Approach**: In `robot_loop.cpp`'s `cycle()`, replace the final
`runAndWait(kPace, [&]{ ... })` call's implicit `sleepUntil(mark, kPace)`
(where `mark` is that block's own entry `markTime()`) with an explicit
sequence: run the body, then call `sleepUntil(state_.time.cycleStart,
kCycle)` directly instead of going through the generic `runAndWait()`
helper for this one block (or add a small `runAndWaitUntil(uint32_t
deadlineMark, uint32_t deadlineGap, Body body)` variant used only here).
Verify the count of pacing/sleep calls per cycle is unchanged (still
exactly four). Add a host-build-only test harness (fake `Devices::Clock`/
`Devices::Sleeper` pair returning jittery elapsed times) and assert the
mean period over N simulated cycles.

**Files to modify**:
- `src/firm/app/robot_loop.cpp` (final pacing block)
- `src/firm/app/robot_loop.h` (if a new private helper is added)
- `data/robots/*.json` (`controlPeriod` value)
- `src/firm/main.cpp` (stale comment)
- `src/firm/app/boot_wiring.h` (stale comment)
- `src/motion/planner/planner.cpp` (comment only, near `update()`'s `dt`)

**Testing plan**: as listed above.

**Documentation updates**: all the stale-comment fixes listed in
Acceptance Criteria are this ticket's own deliverables, not incidental
cleanup — verify each cited line is actually corrected, not just the
code.
