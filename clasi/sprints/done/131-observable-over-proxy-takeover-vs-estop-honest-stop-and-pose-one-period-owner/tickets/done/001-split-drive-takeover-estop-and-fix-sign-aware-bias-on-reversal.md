---
id: '001'
title: Split Drive::takeover()/estop() and fix sign-aware bias on reversal
status: done
use-cases:
- SUC-131-001
depends-on: []
github-issue: ''
issue: A-move-takeover-wipes-the-controllers-learned-state.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Split Drive::takeover()/estop() and fix sign-aware bias on reversal

## Description

`App::RobotLoop::handleMove()` (`robot_loop.cpp:227`) calls `drive_.estop()`
on every accepted MOVE so the planner can take over motion. Since 130-004,
`Drive::estop()` also zeroes both PID integrators, both Stage C biases, and
the deficit latch — so every planner Move cold-starts adaptation, and Stage
C (tauAdapt = 30s) can never converge on the path it exists to serve.

Introduce `Drive::takeover()`: zero targets, disarm the WHEELS command —
KEEP Stage B's integrators, Stage C's bias, and the deficit latch, because
ownership changed but the plant did not. Point `robot_loop.cpp:227` at
`takeover()`. `Drive::estop()` keeps its current full-reset behavior,
reserved for the ESTOP verb (`handleEstop()`) and other genuine panic
paths.

**Mandatory pairing — do not land the takeover/estop split without the
bias fix below.** Once takeover stops resetting the bias every leg, a
forward-learned bias becomes long-lived enough to be exercised across a
direction reversal for the first time. Today `correctedCommand()`
(`drive.cpp:112-123`) applies `bias` as a body-frame-fixed additive term
(`copysign(magnitude, desired) + bias`): a forward-learned positive bias
REDUCES a reverse command's magnitude instead of boosting it, and perturbs
pivots until re-learned at tau = 30s (review C3). This is latent today only
because takeover's `estop()` call resets the bias so often. Fixing takeover
alone, without this, makes C3 live and worse than before. Fix
`correctedCommand()` so `bias` is applied as a magnitude-domain correction
that follows the CURRENT commanded direction (`desired`'s sign), not a
fixed positive offset — this preserves 130-004's one-bias-per-wheel model
(no per-direction split; the physical droop is a property of load, not
which side of a ramp it arrived from) while making the correction help
regardless of direction. Both changes land in this one ticket.

## Acceptance Criteria

- [x] `Drive::takeover()` exists, is public, zeroes `targetLeft_`/
      `targetRight_`, sets `commandActive_ = false` — and does NOT touch
      `pidIntegralLeft_`/`Right_`, `biasLeft_`/`Right_`,
      `deficitSinceLeft_`/`Right_`, `deficitLeft_`/`Right_`, or
      `stopEnforceCountdown_`.
- [x] `App::RobotLoop::handleMove()`'s call at the "the planner takes over
      motion" comment (`robot_loop.cpp:227`) now calls `drive_.takeover()`,
      not `drive_.estop()`.
- [x] `Drive::estop()`'s behavior is unchanged (still zeroes both biases,
      both PID integrators, and the deficit latch); its doc comment states
      it is reserved for the ESTOP verb and genuine panic paths, not
      ownership handover.
- [x] `correctedCommand()`'s bias term is applied direction-relative
      (magnitude domain, following `desired`'s sign) rather than as a
      fixed additive term. Verified by a new test: converge a positive
      bias under sustained forward motion, then command sustained reverse
      motion — the reverse command's magnitude is NOT reduced relative to
      what it would be with bias == 0 (the fix must not make reverse
      magnitude worse than uncorrected).
- [x] New test: a converged Stage C bias survives a `takeover()` call —
      `biasLeft()`/`biasRight()` read the same value immediately before and
      after the call.
- [x] New test: `estop()` still zeroes both biases, both PID integrators,
      and the deficit latch in one call — a regression guard so takeover
      and estop cannot be silently re-merged later.
- [x] Sim test: a chained multi-leg Move sequence (2+ legs, equivalent to
      a tour) shows bias NOT resetting to 0 between legs.
- [x] Full sim suite stays green (460 passed / 0 failed, or the baseline
      at ticket start if it has moved); `handleWheels()`/`handleEstop()`
      call sites and their existing tests are unaffected (this ticket does
      not touch them).

## Testing

- **Existing tests to run**: firmware/sim unit tests covering
  `App::Drive` (the Stage A/B/C harness), `App::RobotLoop`'s command
  routing tests, full `src/tests/sim` suite.
- **New tests to write**:
  - Firmware/sim unit test: `takeover()` preserves bias/integrators/
    deficit latch; `estop()` still resets all of them (both in one file,
    asserting the distinct post-conditions side by side).
  - Firmware/sim unit test: reversal-after-converged-forward-bias does not
    reduce reverse magnitude.
  - Sim test: bias persists across a chained-leg tour (no reset between
    legs).
- **Verification command**: `uv run python -m pytest src/tests/sim` (host
  build); firmware-side harness per its own build target.

## Implementation Plan

**Approach**: Add `void takeover();` to `Drive`'s public interface
(`drive.h`), documented alongside `estop()` to make the ownership-vs-safety
distinction explicit at the declaration site. Implement in `drive.cpp` by
factoring the "zero targets, disarm, arm the stop-reassertion window"
subset of `estop()`'s current body into `takeover()`; have `estop()` call
`takeover()` for that shared part, then additionally reset Stage B/C state
and the deficit latch. Change the one call site in
`robot_loop.cpp:227`. For the bias fix, change `correctedCommand()`'s
return expression so `bias` is applied as a signed magnitude following
`desired`'s direction instead of a raw additive term.

**Files to modify**:
- `src/firm/app/drive.h` — add `takeover()` declaration + doc comment;
  update `estop()`'s doc comment to narrow its purpose.
- `src/firm/app/drive.cpp` — implement `takeover()`; refactor `estop()` to
  build on it; fix `correctedCommand()`'s bias application.
- `src/firm/app/robot_loop.cpp` — one call-site change (`drive_.estop()`
  -> `drive_.takeover()` in `handleMove()`), with the adjacent comment
  updated to say learned state is preserved.
- Existing `App::Drive` test harness (confirm exact file, e.g.
  `src/tests/sim/unit/app_drive_harness.cpp` or equivalent) — extend with
  the new scenarios above.

**Testing plan**: as listed above — firmware-level unit tests for the two
verbs' distinct post-conditions and the reversal-bias behavior; a sim-tier
chained-leg test for bias persistence.

**Documentation updates**: `drive.h`'s file header (the responsibilities/
LOAD-BEARING narrative) gets a short addendum describing the takeover/
estop split and why, matching sprint.md's Design Rationale Decision 1 and
Decision 2.

## Completion Notes

**Implemented** (one commit, per the ticket's mandatory pairing):

- `src/firm/app/drive.h` / `drive.cpp`: added `Drive::takeover()` (zero
  `targetLeft_`/`targetRight_`, `commandActive_ = false`; leaves
  `pidIntegralLeft_`/`Right_`, `biasLeft_`/`Right_`,
  `deficitSinceLeft_`/`Right_`, `deficitLeft_`/`Right_`, and
  `stopEnforceCountdown_` completely untouched). `estop()` now calls
  `takeover()` for the shared "zero targets, disarm" half, then adds the
  stop-reassertion-window arm and the full Stage B/C/deficit reset on top
  — the two verbs cannot silently re-merge without an editor deleting a
  visible, commented block. `correctedCommand()`'s bias term now folds into
  the magnitude (`magnitude + bias`, clamped to never go negative) before
  `copysign()` restores `desired`'s sign, instead of being added after
  `copysign()` — a bias learned under sustained motion in one direction now
  boosts a command in the OTHER direction by the same physical amount
  instead of subtracting from it.
- `src/firm/app/robot_loop.cpp`: `handleMove()`'s call at the "planner
  takes over motion" site now calls `drive_.takeover()`, not
  `drive_.estop()`; the adjacent dedup-early-return comment and a new
  comment above the call site were updated to match.
- `src/tests/sim/unit/app_drive_harness.cpp`: four new scenarios —
  (1) `scenarioTakeoverPreservesLearnedStateEstopFullyResets` — one
  scenario, both verbs' post-conditions asserted side by side (bias, Stage
  B integrator via a fresh-Drive comparison, and the deficit latch); (2)
  `scenarioTakeoverDoesNotTouchStopReassertionCountdown` — indirect proof
  via `tick()`'s own re-assertion write count, since
  `stopEnforceCountdown_` has no public accessor; (3)
  `scenarioReversalAfterConvergedForwardBiasDoesNotReduceMagnitude` —
  converges a positive bias forward, freezes Stage C, then confirms a
  reverse command's magnitude is boosted (not reduced) relative to a
  bias==0 baseline Drive; (4)
  `scenarioBiasPersistsAcrossChainedTakeoverBoundaries` — bias survives
  THREE legs / two `takeover()` boundaries at the `Drive`-controller level.
- `src/tests/sim/system/sim_api_harness.cpp`: one new scenario,
  `scenarioBiasPersistsAcrossChainedMoveLegs`, exercising the SAME property
  through the real production call site — `TestSim::SimHarness`, a
  deliberately-wrong (80%) duty-per-speed calibration on top of the real
  `WheelPlant` physics so Stage C has a genuine error to close, then three
  chained real `MOVE` commands over the wire, asserting `sim.drive().
  biasLeft()` stays away from 0 at both leg boundaries.

**Verified**:

- `uv run python -m pytest src/tests/sim/unit/test_app_drive.py` and
  `uv run python -m pytest src/tests/sim/system/test_sim_api.py` run
  directly, both green, confirming the four/one new scenarios above pass
  (and fail loudly when deliberately broken during development — the
  Stage-B-integrator comparison in scenario (1) was caught and fixed this
  way: the naive "p-term-only" expectation didn't account for `pidMax`
  clamping, which the fresh-Drive comparison approach resolved correctly).
- Full suite baseline at ticket start: **463 collected**
  (`pytest --collect-only`). Full suite after this change: **460 passed, 1
  xfailed, 2 xpassed** (668.89s) — 463 total, matching the baseline count
  exactly, no regressions. The 1 xfailed / 2 xpassed are pre-existing,
  unrelated to this ticket (`clasi/issues/
  B-app-robot-loop-harness-never-compiled.md` — `app_robot_loop_harness.cpp`
  predates this ticket and does not compile against the current
  `App::RobotLoop` constructor shape; not touched here).
- `handleWheels()`/`handleEstop()` call sites are untouched (confirmed by
  `grep`: the only `drive_.estop()` call sites in `src/firm` remaining are
  `robot_loop.cpp:293` inside `handleEstop()`; `handleWheels()` calls
  `planner_.estop()`, a different class, unaffected by this ticket).

**Not verified / explicitly deferred**: hardware/bench acceptance.
`tovez` has been wedged since 2026-08-01 (per sprint.md's own "Hardware
bench debt" section and `.claude/rules/hardware-bench-testing.md`) and was
not reachable during this ticket. Every claim above is sim-tier only —
the real firmware source (`drive.cpp`/`drive.h`/`robot_loop.cpp`) compiled
and run through the sim composition root (`TestSim::SimHarness`,
130-002), not a physical stand run. No bench result is claimed or implied.
The ARM cross-build (`just build`) was not separately invoked for this
ticket — the host-build sim-tier compile already builds the exact changed
files against the same headers the ARM build compiles (`-DHOST_BUILD`,
per `test_app_drive.py`'s own header comment); a full ARM build was judged
redundant for a change with no ARM-only code path (no `#ifdef
HOST_BUILD`/vendor-SDK touch in this ticket's diff).

**Note for ticket 002** (commanded-zero through Stage B): confirmed by
direct inspection of `tick()` that `correctedCommand()`'s `desired ==
0.0f` early return is a **Stage-A-only** guard — it zeroes both the
calibration map's output AND the bias term together, but has no effect on
Stage B at all. Stage B's `fastPid()` call in `tick()` is a fully separate
computation: `errLeft = speedLeft - state.wheelLeft.velocity` (where
`speedLeft` is the floored commanded speed, genuinely 0.0f when commanded
is 0, via `applySpeedFloor()`'s own `commanded == 0.0f` guard) is nonzero
whenever the wheel hasn't yet physically coasted to rest, and nothing
gates `fastPid()` on "commanded speed is exactly zero" the way
`correctedCommand()` gates Stage A. `steadyLeft` (`|cmdAccel| < aSteady`)
can be true immediately after the commanded speed reaches 0 (once the
deceleration itself ends, `cmdAccel` returns to ~0), so the integrator
resumes winding against the residual coast-down error rather than freezing
or zeroing. The resulting nonzero `pidLeft` is added to `correctedLeft`
(0) before the `dutyPerSpeed` multiply, so `dutyLeft` can be measurably
nonzero even though the commanded speed is exactly 0 — and because
`dutyLeft != 0.0f`, the bottom-of-`tick()` `commandedStop` test
(`dutyLeft == 0.0f && dutyRight == 0.0f`) is also false, so the
quiet-at-zero shortcut never even considers this a stop. These are
genuinely two separate paths today: this ticket's bias fix lives entirely
inside `correctedCommand()` (Stage A) and does not touch or interact with
Stage B's gap in any way — ticket 002 will need to add its OWN explicit
commanded-zero guard to `fastPid()`/its call site (and decide whether to
skip it outright or freeze the integrator), since nothing here provides
that protection.
