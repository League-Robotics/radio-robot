---
id: '004'
title: Delete the terminal patch stack; unify completion; bump heading_kp default
status: open
use-cases:
- SUC-005
- SUC-006
- SUC-007
depends-on:
- '003'
github-issue: ''
issue: motion-control-terminal-blips-reconciled-fix-plan.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Delete the terminal patch stack; unify completion; bump heading_kp default

## Description

Issue step 6 — the final ticket of this sprint. Deletes the remaining
terminal patch stack in `Motion::Executor`/`App::Pilot`: `kStraightLeadBias`/
`kStraightLeadSlope` (straight-lead padding), the `pendingLinearRetarget_`
terminal top-up + its cross-bias nudge, the same-sign `pendingOvershoot_`
carry between chained kArc commands, `App::Pilot`'s min-speed floor, the
EMA (`dwellRateFilt_`) and leaky-counter (`dwellHeldMs_`) dwell machinery,
and `terminal_lead`/`thetaErrLead` — and replaces the ad hoc completion
dispatch with one unified rule: `done = (t >= duration + margin) AND
(|s_err| < distance_tol) AND (|theta_err| < heading_dwell_tol)`, held for
`arrive_dwell`, one `stopTimeBackstopMs()` timeout. This is safe only now
that tickets 002/003 have landed the principled replacements (accel
feedforward, bounded position trim) the deleted patches were compensating
for the absence of — see sprint Architecture "Why" and Design Rationale
Decision 1 for the full dependency reasoning. Bumps `HEADING_KP_DEFAULT`
from 3.0 to 6.0 in `gen_boot_config.py` so the deadband inequality
(`heading_kp * heading_dwell_tol >= omega_deadband`) holds without the
deleted min-speed floor — see sprint Architecture Design Rationale
Decision 5 for the full numeric derivation to start from and re-verify.
**Two existing mechanisms must survive this ticket unchanged, not be
folded away by the completion-logic rewrite**: the 40mm gross-divergence
reanchor (`checkDivergence()`'s `pendingLinearReanchor_` tier — a
mid-command recovery mechanism, distinct from the between-command
`pendingOvershoot_` carry this ticket deletes) and the 109-009
boundary-velocity-carry chained-pivot dwell-skip exception
(`carryingRotationalVelocity`, tested via `withinTol OR crossedTarget`, no
hold — see `motion/DESIGN.md` §2c's own "109-009 revision" write-up for
why this exception exists and what regressed before it was added).

## Acceptance Criteria

- [ ] `kStraightLeadBias`, `kStraightLeadSlope`, and the straight-lead
      padding block in `plan()`'s kArc branch are deleted.
- [ ] `pendingLinearRetarget_`, its terminal top-up trigger in `tick()`,
      its cross-bias epsilon nudge in `plan()`, and
      `kTopUpMeasuredRestVelocity` are deleted.
- [ ] `pendingOvershoot_` and its same-sign-carry logic in `activate()`/
      `completeActive()` are deleted.
- [ ] **Guardrail — 40mm reanchor preserved**: `checkDivergence()`'s
      `pendingLinearReanchor_` tier (the 40mm gross-divergence recovery) is
      explicitly NOT touched by this ticket. Verify by diff that this
      branch of `checkDivergence()` is unchanged (only the unrelated,
      between-command `pendingOvershoot_` carry is deleted).
- [ ] `App::Pilot::tick()`'s min-speed floor block (`minSpeed_`, the
      `if (minSpeed_ > 0.0f && ...)` block) is deleted; the `minSpeed_`
      member and its `configureHeading()` assignment are deleted.
- [ ] `dwellRateFilt_` (EMA) and `dwellHeldMs_`'s leaky-counter logic, and
      the `withinRate`/`crossedTarget`/`carryingRotationalVelocity`
      dispatch tree, are replaced by the unified completion rule below —
      EXCEPT the `carryingRotationalVelocity` distinction and its
      `withinTol OR crossedTarget` no-hold test, which are preserved
      VERBATIM for a command carrying a nonzero `exitVelocity_` into a
      compatible successor (the 109-009 exception).
- [ ] **Guardrail — 109-009 exception preserved**: do not regress the
      chained-pivot dwell-skip exception. Sprint 111's harness has NO
      coverage of chained pivot->pivot (its same-boot scenario alternates
      straight/pivot with no chaining — see sprint Architecture Open
      Questions) — verify this exception by code inspection AND a new,
      targeted unit/system scenario (e.g. two same-sign chained pivots via
      `injectMove(..., replace=false)` back-to-back), not by the harness
      alone.
- [ ] `terminal_lead`/`thetaErrLead` are deleted; the dwell tolerance test
      uses the raw `thetaErr` already computed in `tick()`.
- [ ] Unified completion rule implemented for the "not carrying" branch
      (terminal, or chained into an incompatible/non-pivot successor):
      `t >= duration + margin AND |s_err| < distance_tol AND |theta_err| <
      heading_dwell_tol`, held for `arrive_dwell`, single
      `stopTimeBackstopMs()` timeout.
- [ ] `HEADING_KP_DEFAULT` bumped 3.0 -> 6.0 in `gen_boot_config.py`, with
      a comment citing the deadband-inequality derivation AND this
      ticket's own empirical re-verification of the actual
      `v_deadband`/`trackWidth`/`heading_dwell_tol` in force at
      implementation time (do not cite the architecture doc's numbers
      unchecked).
- [ ] `test_straight_terminal_bounds` flips from `xfail` to passing.
- [ ] `test_pivot_single_lobe_left` flips from `xfail` to passing.
- [ ] `test_pivot_single_lobe_right` flips from `xfail` to passing.
- [ ] `test_pivot_lobes_opposite_sign` flips from `xfail` to passing.
- [ ] Every other harness check (all currently-passing ones, plus ticket
      001's two prior flips) stays green.
- [ ] `test_same_boot_all_moves_completed` stays passing (40 consecutive
      moves, no stale-executor-state fault introduced by the
      completion-rule rewrite).
- [ ] **Guardrail (SUC-007)**: `git diff --stat` shows no changes to
      `src/firm/app/robot_loop.cpp`; `grep 'runAndWait\|sleepUntil'
      src/firm/app/robot_loop.cpp` output is byte-for-byte unchanged from
      before this ticket.
- [ ] **Guardrail (SUC-007, 087-009 non-regression)**: no `JerkTrajectory`
      solve is newly seeded from measured state — the unified completion
      rule is a MEASURED-STATE READ (comparison against `distance_tol`/
      `heading_dwell_tol`), never a trigger for a new solve seeded from
      that measured state near the target.
- [ ] `motion/DESIGN.md` §2c (dwell completion, distance completion +
      overshoot carry, turn-error characterization locus 3) and
      `app/DESIGN.md` (Pilot's min-speed floor mention) are updated to
      reflect the deletions — do not leave a stale doc describing removed
      machinery as if it still exists.
- [ ] `uv run python -m pytest` is green end to end: baseline was 1224
      passed / 18 xfailed / 2 xpassed / 0 failed; this ticket's completion
      notes should show 6 fewer xfails and 6 more passed (the full
      straight+pivot flip set across tickets 001+004), same xpassed/failed
      counts, 0 failed.
- [ ] Ticket frontmatter `completes_issue: false` (already set) — steps
      8-9-10 of the driving issue remain open for future arc sprints; this
      ticket does not complete the issue.

## Implementation Plan

- **Approach**: deletion-heavy. Implement the unified rule as one small
  function/block replacing the current `headingContent`/
  `carryingRotationalVelocity` dispatch tree, explicitly keeping the
  carrying-velocity branch as a DISTINCT, still-present code path (not
  accidentally folded into the unified rule's hold logic).
- **Files to modify**: `src/firm/motion/executor.h`/`.cpp` (all
  deletions + unified rule), `src/firm/app/pilot.h`/`.cpp` (min-speed
  floor deletion), `src/scripts/gen_boot_config.py` (`HEADING_KP_DEFAULT`
  bump), `src/tests/sim/system/test_behavior_lock.py` (4 `xfail`
  removals), `src/firm/motion/DESIGN.md`, `src/firm/app/DESIGN.md`.
- **Documentation updates**: as listed above; this ticket's completion
  notes must record the actual expected-xfail-count delta and confirm it
  against the sprint's stated baseline.

## Testing

- **Existing tests to run**: full `test_behavior_lock.py -v -s` first,
  then the full `uv run python -m pytest`.
- **New tests to write**: a dedicated same-boot/chained-pivot scenario for
  the 109-009 exception (the harness's own same-boot scenario does not
  chain commands — see the guardrail bullet above).
- **Verification command**: `uv run python -m pytest`.
