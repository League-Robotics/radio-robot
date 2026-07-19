---
id: '001'
title: Delete plan_lead / pivot-overshoot-lead peek sampling
status: open
use-cases:
- SUC-005
- SUC-006
- SUC-007
depends-on: []
github-issue: ''
issue: motion-control-terminal-blips-reconciled-fix-plan.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Delete plan_lead / pivot-overshoot-lead peek sampling

## Description

Issue step 3 (`clasi/issues/motion-control-terminal-blips-reconciled-fix-plan.md`).
`Motion::Executor::tick()` currently samples the dominant channel's velocity
reference via `JerkTrajectory::peek(elapsed + lead)` rather than
`sample(elapsed)` — a time-shifted "lead" intended to anticipate actuation
lag but which (finding F2) evaluates the reference at `2t` during the ramp-in,
doubling commanded acceleration and quadrupling commanded jerk right at Move
activation. This ticket deletes that peek-ahead machinery for BOTH the
linear channel (`plan_lead`) and the rotational channel (`plan_lead` plus the
pivot-only extra lead, `kPivotOvershootLeadSlope`), restoring honest
same-instant sampling. This is a pure deletion — no new control-law code is
added by this ticket (feedforward/feedback additions are tickets 002/003).

## Acceptance Criteria

- [ ] `out.v` (kArc) and `omegaFf` (kPivot / heading-bearing kArc) in
      `Executor::tick()` are computed from `linSample`/`rotSample` (the
      same-instant `sample()` result already computed earlier in the
      function), never from a `peek(elapsed + lead)` call.
- [ ] The `linLead`/`rotLead`/`rotTargetLead` local computation block and the
      `kPivotOvershootLeadSlope` constant are deleted from `executor.cpp`.
- [ ] `Executor`'s `planLeadS_` member and its `configure()` assignment
      (`planLeadS_ = config.plan_lead`) are deleted. `plan_lead` itself
      remains a DECLARED (not `reserved`) `PlannerConfig` field — do not
      touch `planner.proto` in this ticket (see sprint Architecture Design
      Rationale Decision 7; schema cleanup is a future, separate ticket).
- [ ] `test_straight_ramp_bounds` flips from `xfail` to passing (remove its
      `@pytest.mark.xfail` decorator in `test_behavior_lock.py`).
- [ ] `test_pivot_ramp_bounds` flips from `xfail` to passing (remove its
      `@pytest.mark.xfail` decorator).
- [ ] No other harness check regresses: `test_straight_single_lobe_left`,
      `test_straight_single_lobe_right`,
      `test_straight_no_command_after_terminal_zero`,
      `test_straight_shelf_collapsed`, `test_pivot_terminal_bounds`,
      `test_pivot_no_command_after_terminal_zero`,
      `test_pivot_shelf_collapsed`, `test_same_boot_all_moves_completed`,
      `test_behavior_lock_harness_compiles_and_runs` all still pass.
- [ ] **Guardrail (SUC-007)**: `git diff --stat` shows no changes to
      `src/firm/app/robot_loop.cpp` — this ticket does not touch cycle
      order or request/collect sequencing.
- [ ] **Guardrail (SUC-007)**: no `JerkTrajectory` solve call
      (`solveToRest`/`solveToState`/`solveToVelocity`/`retarget`/`reanchor`)
      is added, removed in a way that changes its seeding, or newly seeded
      from measured state by this ticket — this is a sampling-only change
      (`sample()`/`peek()` call sites), not a solve-path change. The 40mm
      gross-divergence reanchor (`checkDivergence()`'s
      `pendingLinearReanchor_` path) is untouched.
- [ ] `motion/DESIGN.md` §2c's "Turn-error characterization (109-010)"
      locus-2 write-up is updated to note `plan_lead` (locus 2) is
      deleted/dead; loci 1 (`heading_lead_bias`) and 3 (`terminal_lead`) are
      explicitly out of this ticket's scope (locus 3 is ticket 004's scope;
      locus 1 is out of scope for the whole sprint — see sprint.md's Out of
      Scope).
- [ ] `uv run python -m pytest` is green (pass or pre-existing/expected
      xfail) end to end.

## Implementation Plan

- **Approach**: in `Executor::tick()`'s kArc/kPivot branch, replace the
  `linSampleLead`/`rotSampleLead` peek results (used for `out.v`/`omegaFf`)
  with the already-computed `linSample`/`rotSample` (`sample(elapsed)`
  results). Delete the lead-ramp-in computation block and the
  `kPivotOvershootLeadSlope` constant. Delete `planLeadS_` and its
  `configure()` line. Leave `terminalLeadS_`/`terminal_lead` and
  `thetaErrLead` untouched (ticket 004's scope) — this ticket only touches
  the VELOCITY-reference sampling locus (locus 2), not the completion-decision
  locus (locus 3).
- **Files to modify**: `src/firm/motion/executor.h` (remove `planLeadS_`
  field; update the file-header's lead-compensation description),
  `src/firm/motion/executor.cpp` (delete the peek-ahead block and
  `kPivotOvershootLeadSlope`; delete `configure()`'s `planLeadS_` line),
  `src/firm/motion/DESIGN.md` (locus-2 entry), `src/tests/sim/system/
  test_behavior_lock.py` (remove 2 `xfail` markers).
- **Documentation updates**: `motion/DESIGN.md` §2c locus-2 write-up.

## Testing

- **Existing tests to run**: `uv run python -m pytest
  src/tests/sim/system/test_behavior_lock.py -v -s` first (fast iteration
  against the harness), then the full `uv run python -m pytest`.
- **New tests to write**: none — this ticket is verified entirely by
  flipping sprint 111's existing harness `xfail` markers; no new test file.
- **Verification command**: `uv run python -m pytest` (canonical command —
  bare `uv run pytest` hits a known collection error).
