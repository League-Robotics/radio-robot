---
id: '005'
title: 'One control period: absolute end-of-cycle deadline pacing'
status: done
use-cases:
- SUC-131-005
depends-on:
- '004'
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

- [x] `App::RobotLoop::cycle()`'s final pacing block's wait targets
      `state_.time.cycleStart + kCycle` (an absolute deadline), not a
      relative gap from that block's own entry `markTime()`. Implemented
      via a new private `runAndWaitUntil(deadlineMark, deadlineGap, body)`
      helper (mirrors `runAndWait()` but takes the deadline mark as a
      caller-supplied argument instead of re-deriving it from `markTime()`
      at entry); the call site is
      `runAndWaitUntil(state_.time.cycleStart, kCycle, [&] {...})`,
      replacing `runAndWait(kPace, [&] {...})`. The now-unused `kPace`
      constant is deleted (robot_loop.cpp); `kWindows`/its static_assert
      stay (still a real invariant: the three settle/clear windows must
      leave room in the budget).
- [x] Host-build unit test: an injected fake `Clock`/`Sleeper` simulating
      variable per-block overruns and whole-millisecond sleep rounding
      shows the MEAN measured inter-cycle-start period converging to
      `kCycle` (50 ms) over many simulated cycles, not `kCycle` + a fixed
      offset. New `src/tests/sim/unit/app_robot_loop_pacing_harness.cpp`
      (+ `test_app_robot_loop_pacing.py`): a `JitterySleeper` wraps a
      `TestSim::SimClock` and advances it by `requested + overrun` on
      every `sleepMillis()` call, `overrun` cycling through a period-7
      pattern `{1,0,2,1,3,0,1}` ms (period 7 chosen deliberately NOT a
      divisor of 4, so which of the cycle's 4 pacing calls absorbs how
      much genuinely varies cycle to cycle rather than degenerating into
      one repeated constant). Built directly on `App::composeRobot()`
      (not `TestSim::SimHarness`, whose own SimClock/SimSleeper pair is
      fixed/non-substitutable and never advances mid-cycle -- no seam to
      inject jitter through). Result: mean 51.146 ms, range
      [50.000, 53.000] ms over 199 samples, both within the asserted
      +/-2 ms band around kCycle=50. Verified this is a REAL regression
      test, not a tautology: pointed the identical harness at a scratch
      copy of the OLD relative-gap call
      (`runAndWait(kCycle-2*kSettle-kClear, ...)`) under the SAME
      injected jitter and got mean 54.568 ms (range [52,56]) -- matching
      130-011's own hardware measurement (54.000 ms +/- 0.006) almost
      exactly -- and the new test's own convergence check correctly
      FAILED against it.
- [x] The existing "exactly four `sleepMillis()` calls per cycle"
      schedule-shape assertion (`sim_api_harness.cpp` or equivalent)
      still holds. The COUNT (4 calls, 0 yields) is unchanged and still
      asserted. The scenario's OLD value assertion
      (`lastSleepMillis == kPace == 38`) does NOT hold any more under
      `TestSim::SimHarness`'s own fake clock, for a harness-specific
      reason unrelated to the fix: that clock never advances BETWEEN a
      cycle's four pacing blocks (only `SimHarness::step()`'s own single
      bulk advance, before `cycle()`), so an absolute deadline computed
      from cycleStart always sees zero elapsed time and requests the
      full kCycle=50, not kCycle-minus-windows. Updated the scenario's
      expected value to `kCycle` (50) with an extensive comment
      explaining why, and pointed to the new host-build jitter test as
      where the real convergence-to-kCycle claim is actually proven
      (this fake clock structurally cannot prove it). Verified via
      direct harness run before/after: the scenario failed with
      `lastSleepMillis=50ms` against the OLD test expectations
      immediately after the production fix landed, passes now that the
      test's own expectations are updated to match.
- [x] `data/robots/*.json`'s `controlPeriod` field is corrected to 50 on
      every robot profile. Verified: `togov.json`, `tovez_nocal.json`,
      `tovez.json` all already carry `control_period: 50.0` -- no
      numeric change was needed (a prior ticket already set the value;
      only the surrounding rationale text was stale). `tovez.json`'s own
      `_timing_note` (the one that documented the 54ms defect as an
      unfixed follow-up) is appended with a `FIX (131-005 ...)` note
      describing the actual root cause and fix, rather than leaving the
      note saying "flagged as a follow-up, not fixed here" while the
      defect it describes is in fact now fixed.
- [x] Stale comments corrected: `main.cpp:80`'s "~47 ms budget",
      `boot_wiring.h:36`'s "measured 47ms instead of the nominal 40",
      `robot_loop.h:49-57`'s `kCycle` doc comment, all restated to
      reflect the new true invariant. `robot_loop.h`'s comment
      specifically no longer claims "the delivered period is exactly
      50ms and stable" (measurably false per 130-011's own 54.000ms
      finding) -- it now states the full history (130-007's raise,
      130-011's re-measurement finding the SAME defect reopened at the
      new nominal, 131-005's fix and its host-build-only proof) and
      explicitly flags the physical-hardware re-measurement as
      outstanding. Also swept two directly-adjacent stale spots found
      while touching this neighborhood (not explicitly named in the
      ticket, but the same claim in the same immediate area): the
      `PlannerBootConfig::controlPeriod` field doc comment
      (`boot_config.h`, was "the MEASURED delivered cycle period, not a
      nominal") and `robot_config.schema.json`'s `control_period`
      description (claimed "kCycle=50 is now delivered exactly" --
      exactly the claim 130-011 falsified). Left `src/firm/app/DESIGN.md`
      and `src/motion/DESIGN.md`'s several "`kPace` block" labels alone
      (out of the ticket's stated file scope, and none of them assert a
      false timing claim -- they use "kPace block" purely as a stable
      name for "the trailing pacing block", which still exists and
      still runs the same work, only its own wait's arithmetic changed)
      -- flagged here rather than silently left.
- [x] `Motion::Planner`'s comment near `update()`'s
      `dt = limits_.plant.controlPeriod * 0.001f` (`planner.cpp` ~1347) is
      updated to note `controlPeriod` and `App::Drive`'s measured
      `state.time.cyclePeriod` now agree by construction, rather than
      describing the two as an accepted disagreement.
- [x] Explicitly declared in the ticket's own completion notes (not
      silently assumed): the PHYSICAL robot's delivered period is NOT
      re-measured this ticket — flagged as a required follow-up
      (`src/tests/bench/planner_square_tour.py` once `tovez` is
      reachable). See Completion Notes below.
- [x] No regression: the sim (which already steps at exactly `kCycle` per
      `SimHarness::step()`) shows unchanged behavior. Full sim suite
      stays green. Verified: `uv run python -m pytest src/tests/sim` --
      461 passed, 1 xfailed, 2 xpassed, 0 failed at baseline; 462
      passed (1 new test added by this ticket), 1 xfailed, 2 xpassed, 0
      failed after -- byte-identical otherwise to the team-lead's stated
      `0edefd06` baseline.

## Completion Notes

**The decision made** (per the ticket's own framing, and matching its
title): option 1 -- make the loop deliver the nominal, via an absolute
end-of-cycle deadline on the trailing pacing block -- not option 2
(re-baking `controlPeriod` to whatever the loop empirically delivers).
`data/robots/*.json`'s `controlPeriod` was already `50.0` on every
profile (a prior ticket had set the VALUE; only the rationale text
around it was stale/wrong), so this ticket's real work was entirely in
`robot_loop.{h,cpp}`'s pacing mechanism plus the stale-comment sweep --
there is exactly one owner of "how long is a cycle" now: `kCycle`
itself, read directly by the pacing deadline, matching the value baked
into the robot JSON, matching the planner's `controlPeriod`.

**What was verified**: the sim tier (`src/tests/sim`, 462/1x/2x/0f,
unchanged from baseline plus this ticket's own new test), the
turn-accuracy non-regression gate
(`test_tour_closure_gate.py::test_tour_1_and_tour_2_ninety_degree_turns_land_within_the_shaped_band`),
and, new, a host-build unit test proving the pacing fix's own claim
under injected jitter, cross-checked against the OLD scheme to confirm
it is a genuine regression test (see the second AC's own notes above
for the exact before/after numbers).

**Turn gate result (unregressed)**: TOUR_1/ideal turn 2 = -2.567 deg,
turn 4 = -13.080 deg -- BYTE-IDENTICAL to the team-lead's stated
tickets-001/002 baseline. This test is EXPECTED to still fail overall
(turn 4's -13.080 deg exceeds the scenario's own 8 deg "shaped-band"
tolerance) -- that failure is turn 4's pre-existing `decelLatched`
defect (ticket 006's own territory,
`A-tour2-146-degree-turn-still-undershoots-after-130-010.md`), already
present at the stated `0edefd06` baseline and explicitly on the "not
yours, do not fix" list. This ticket changed NOTHING about either
number, which is exactly the expected outcome: the sim always steps at
exactly `kCycle` regardless of which pacing scheme computes firmware's
own trailing-block wait (`SimHarness::step()` advances virtual time by
a fixed `kCycleDtUs` before `cycle()` runs, independent of what
`cycle()` itself does internally) -- so this ticket's fix is, by
construction, INVISIBLE to every sim-tier numeric behavior. That is
also precisely why a host-build unit test with an injected jittery
clock (new, this ticket) was necessary to prove the fix at all; the sim
tier alone could not have shown it either way.

**What is deferred, explicitly, not silently assumed**: the PHYSICAL
robot's own delivered period is NOT re-measured by this ticket --
`tovez` has been wedged/unreachable since 2026-08-01 (four dead-I2C
wedges, needs a physical power cycle). Re-running
`src/tests/bench/planner_square_tour.py` once the bench is available
again is a required follow-up, not part of this ticket's own acceptance
(matching the sprint's own "Hardware bench debt" section, which called
this out for ticket 005 specifically before implementation began).

**For ticket 006** (`decelLatched` release + large-angle scenarios):
this ticket did NOT touch `planner.cpp`'s `decelLatched` logic or any
of its profile-math VALUES -- only a comment near `update()`'s `dt`
computation (an unconditionally-true statement about `controlPeriod`
vs. `state.time.cyclePeriod` now agreeing by construction; no behavior
change). The turn-4 -13.080 deg residual ticket 006 is meant to close
is present, unchanged, and confirmed reproducible on this exact commit
-- the number above is ticket 006's own starting baseline, not
something this ticket perturbed. Nothing else in this ticket's diff
touches `src/motion/planner/` beyond that one comment.

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
