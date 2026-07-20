---
id: '001'
title: Delete plan_lead / pivot-overshoot-lead peek sampling
status: done
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

- [x] `out.v` (kArc) and `omegaFf` (kPivot / heading-bearing kArc) in
      `Executor::tick()` are computed from `linSample`/`rotSample` (the
      same-instant `sample()` result already computed earlier in the
      function), never from a `peek(elapsed + lead)` call.
- [x] The `linLead`/`rotLead`/`rotTargetLead` local computation block and the
      `kPivotOvershootLeadSlope` constant are deleted from `executor.cpp`.
- [x] `Executor`'s `planLeadS_` member and its `configure()` assignment
      (`planLeadS_ = config.plan_lead`) are deleted. `plan_lead` itself
      remains a DECLARED (not `reserved`) `PlannerConfig` field — do not
      touch `planner.proto` in this ticket (see sprint Architecture Design
      Rationale Decision 7; schema cleanup is a future, separate ticket).
- [x] `test_straight_ramp_bounds` flips from `xfail` to passing (remove its
      `@pytest.mark.xfail` decorator in `test_behavior_lock.py`). Required
      an in-scope, stakeholder-approved harness-grading fix in addition to
      the production deletion above — see Completion Notes.
- [ ] `test_pivot_ramp_bounds` flips from `xfail` to passing (remove its
      `@pytest.mark.xfail` decorator). **NOT achieved** — stays `xfail`
      with an updated, evidence-based reason. See Completion Notes for the
      full finding and why this box is deliberately left unchecked.
- [x] No other harness check regresses: `test_straight_single_lobe_left`,
      `test_straight_single_lobe_right`,
      `test_straight_no_command_after_terminal_zero`,
      `test_straight_shelf_collapsed`, `test_pivot_terminal_bounds`,
      `test_pivot_no_command_after_terminal_zero`,
      `test_pivot_shelf_collapsed`, `test_same_boot_all_moves_completed`,
      `test_behavior_lock_harness_compiles_and_runs` all still pass.
- [x] **Guardrail (SUC-007)**: `git diff --stat` shows no changes to
      `src/firm/app/robot_loop.cpp` — this ticket does not touch cycle
      order or request/collect sequencing.
- [x] **Guardrail (SUC-007)**: no `JerkTrajectory` solve call
      (`solveToRest`/`solveToState`/`solveToVelocity`/`retarget`/`reanchor`)
      is added, removed in a way that changes its seeding, or newly seeded
      from measured state by this ticket — this is a sampling-only change
      (`sample()`/`peek()` call sites), not a solve-path change. The 40mm
      gross-divergence reanchor (`checkDivergence()`'s
      `pendingLinearReanchor_` path) is untouched.
- [x] `motion/DESIGN.md` §2c's "Turn-error characterization (109-010)"
      locus-2 write-up is updated to note `plan_lead` (locus 2) is
      deleted/dead; loci 1 (`heading_lead_bias`) and 3 (`terminal_lead`) are
      explicitly out of this ticket's scope (locus 3 is ticket 004's scope;
      locus 1 is out of scope for the whole sprint — see sprint.md's Out of
      Scope).
- [x] `uv run python -m pytest` is green (pass or pre-existing/expected
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

## Completion Notes

**What was deleted (executor.h/executor.cpp, as planned).** In
`Motion::Executor::tick()`'s kArc/kPivot branch, `out.v`/`omegaFf` are now
computed directly from `linSample`/`rotSample` (`sample(elapsed)`, already
computed earlier in the function) instead of `linSampleLead`/
`rotSampleLead` (`peek(elapsed + lead)`). Deleted: the `linLead`/`rotLead`/
`rotTargetLead` lead-ramp-in computation block, the `kPivotOvershootLeadSlope`
constant, the `Executor::planLeadS_` member, and its `configure()`
assignment (`planLeadS_ = config.plan_lead`). `terminalLeadS_`/
`terminal_lead`/`thetaErrLead` are untouched (ticket 004's scope).
`plan_lead` remains a declared (not `reserved`) `PlannerConfig` field —
`planner.proto` was not touched. `motion/DESIGN.md` §2c's locus-2 write-up
is updated to document the deletion and explicitly scope loci 1/3 out.
`git diff --stat` confirms zero changes to `src/firm/app/robot_loop.cpp`.

**First pass (Motion::Executor-only) found the deletion correct but
insufficient to flip either xfail.** Verified by a direct, controlled A/B
rebuild against the pre-fix `executor.cpp` (same harness, same config):
the deletion is real and root-caused — it cuts the F2 jerk-warp doubling
exactly as designed (straight commanded-reference accel spike ~4040mm/s^2
-> clean; pivot's own commanded accel spike ~3722mm/s^2 -> clean). But
`test_behavior_lock.py`'s `ramp_bounds` checks were, at that point, still
differentiating the DECODED/MEASURED wheel-velocity trace
(`behavior_lock_harness.cpp`'s original design) rather than the commanded
setpoint — and that measured trace still exceeded bound
(straight ~1635mm/s^2 vs 1350mm/s^2 bound), because it also captures the
downstream velocity-PID/actuation-lag tracking response to a freshly-
nonzero setpoint (a real, separate, ~130ms plant-lag phenomenon, unrelated
to and unfixable by any `Motion::Executor` sampling change). Confirmed via
direct instrumentation (`SimHarness::driveTargetVelLeft/Right()`) that the
COMMANDED reference was already perfectly clean at that point — the
residual was entirely on the measured side.

**Stakeholder decision (mid-ticket, reviews Sec5.3 "differentiate the
emitted setpoints"): fix the harness to grade the commanded setpoint,
expanding this ticket's scope.** `behavior_lock_harness.cpp`'s
`ramp_bounds`/`terminal_bounds`/`single_lobe_*`/`lobes_opposite_sign`
checks now differentiate `SimHarness::driveTargetVelLeft/Right()` (the
same commanded-PID-target signal `measureShelfCycles()` already used) via
a new `Sample::cmdLeft`/`cmdRight` field pair, captured every cycle
alongside the existing decoded telemetry timestamp. `checkNoCommandAfterTerminalZero`
(and therefore `test_*_no_command_after_terminal_zero`) and
`measureShelfCycles`/`runShelfScenario` (`test_*_shelf_collapsed`) are
UNCHANGED — both stay on their pre-existing (measured / already-commanded)
signals, per instruction. This is a test-harness-only change; no firmware
outside `Motion::Executor` was touched.

**Re-verifying every affected marker against the commanded signal
honestly (not all flipped the way expected):**

- `test_straight_ramp_bounds` — **flips to a plain pass.** Confirmed clean
  on the commanded signal (no xfail needed).
- `test_straight_terminal_bounds` — **also flips to a plain pass**, an
  honest, verified-real early side effect of the same two changes (not
  something this ticket set out to fix on purpose — sprint.md assigns
  this flip to ticket 004). Left un-xfailed per "set each marker to match
  reality" rather than held stale for ticket 004 to re-discover.
- `test_pivot_ramp_bounds` — **does NOT flip. Stays `xfail`, with an
  updated, evidence-based reason.** This is a genuine deviation from the
  dispatching instruction's own stated expectation ("ramp_bounds (straight
  + pivot) should now PASS... this is ticket 001's acceptance"), made
  under that same instruction's explicit override ("Whatever the actual
  result, set each marker to match reality — never leave a marker that
  XPASSes-strict or a removed marker that FAILs"). Root cause, confirmed:
  unlike the straight leg, the pivot is heading-bearing
  (`deltaHeading != 0`), so its COMMANDED setpoint is not
  `Motion::Executor`'s `omegaFf` alone — `App::Pilot::tick()` adds the
  heading PD correction (`heading_kp * (thetaRef - thetaMeasLead)`) on top
  before `Drive::setTwist()`, and that PD term reacts every cycle to the
  OTOS-measured heading, which is not a smooth reference the way
  `Executor`'s own `sample()` output is. On the commanded signal,
  post-112-001: the activation-region ACCEL violation is gone entirely
  (was ~3722mm/s^2 vs 1728mm/s^2 bound pre-fix), but a JERK violation
  remains one derivative up (~13824mm/s^3 vs 6912mm/s^3 bound) — real
  progress (down from ~24852mm/s^3 measured pre-harness-fix), but not
  clean. This is an `App::Pilot`-level (heading PD × measured-heading)
  finding, structurally outside this ticket's `Motion::Executor`-only,
  pure-deletion scope — not attributable to any single already-planned
  future ticket, so no specific ticket number is claimed for it here;
  flagged for follow-up (a candidate: a `heading_lead_bias`/measured-
  heading-rate smoothing investigation, or bounding the PD term's own
  rate of change, neither of which exists as a ticket today).
- `test_pivot_single_lobe_left`/`_right`, `test_pivot_lobes_opposite_sign`
  — still fail on the commanded signal (5 lobes, not 1), reasons updated
  to note the commanded-signal grading; kept `xfail`, expected to flip
  when ticket 004's terminal-patch-stack deletion lands (and/or the same
  `App::Pilot` heading-PD dynamic above).

**Suite state (final).** `uv run python -m pytest
src/tests/sim/system/test_behavior_lock.py -v`: **11 passed, 4 xfailed, 0
xpassed, 0 failed.** Full suite: `uv run python -m pytest` → **1226
passed, 16 xfailed, 2 xpassed, 0 failed** (sprint baseline was 1224 passed
/ 18 xfailed / 2 xpassed / 0 failed — the +2 passed / -2 xfailed delta is
exactly `test_straight_ramp_bounds` and `test_straight_terminal_bounds`
flipping to plain passes; xpassed count is unchanged, confirming no new
stale-xfail marker was introduced elsewhere).

**Files touched beyond the original plan**: `src/tests/sim/system/
behavior_lock_harness.cpp` (the commanded-vs-measured grading fix,
stakeholder-authorized mid-ticket expansion — not in the ticket's
original "Files to modify" list above).
