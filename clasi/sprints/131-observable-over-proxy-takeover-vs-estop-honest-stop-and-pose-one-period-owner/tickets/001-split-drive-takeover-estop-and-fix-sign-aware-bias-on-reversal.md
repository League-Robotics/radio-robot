---
id: '001'
title: Split Drive::takeover()/estop() and fix sign-aware bias on reversal
status: open
use-cases: [SUC-131-001]
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

- [ ] `Drive::takeover()` exists, is public, zeroes `targetLeft_`/
      `targetRight_`, sets `commandActive_ = false` — and does NOT touch
      `pidIntegralLeft_`/`Right_`, `biasLeft_`/`Right_`,
      `deficitSinceLeft_`/`Right_`, `deficitLeft_`/`Right_`, or
      `stopEnforceCountdown_`.
- [ ] `App::RobotLoop::handleMove()`'s call at the "the planner takes over
      motion" comment (`robot_loop.cpp:227`) now calls `drive_.takeover()`,
      not `drive_.estop()`.
- [ ] `Drive::estop()`'s behavior is unchanged (still zeroes both biases,
      both PID integrators, and the deficit latch); its doc comment states
      it is reserved for the ESTOP verb and genuine panic paths, not
      ownership handover.
- [ ] `correctedCommand()`'s bias term is applied direction-relative
      (magnitude domain, following `desired`'s sign) rather than as a
      fixed additive term. Verified by a new test: converge a positive
      bias under sustained forward motion, then command sustained reverse
      motion — the reverse command's magnitude is NOT reduced relative to
      what it would be with bias == 0 (the fix must not make reverse
      magnitude worse than uncorrected).
- [ ] New test: a converged Stage C bias survives a `takeover()` call —
      `biasLeft()`/`biasRight()` read the same value immediately before and
      after the call.
- [ ] New test: `estop()` still zeroes both biases, both PID integrators,
      and the deficit latch in one call — a regression guard so takeover
      and estop cannot be silently re-merged later.
- [ ] Sim test: a chained multi-leg Move sequence (2+ legs, equivalent to
      a tour) shows bias NOT resetting to 0 between legs.
- [ ] Full sim suite stays green (460 passed / 0 failed, or the baseline
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
