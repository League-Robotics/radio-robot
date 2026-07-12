---
id: '005'
title: 'Policy: envelopes + terminal machine + MotionPlan::step() composition'
status: open
use-cases: [SUC-005, SUC-006, SUC-007]
depends-on: ['004']
github-issue: ''
issue: motion-stack-v2-a-self-contained-stateless-motion-control-subsystem.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Policy: envelopes + terminal machine + MotionPlan::step() composition

## Description

Implement `source/drive/policy.{h,cpp}` (replan envelope evaluation, the
sustain/rate-limit/N-max replan state machine, the terminal settle
machine, the flying-handoff envelope check, and pose-fix step
absorption/bypass) and compose it with ticket 003's reference sample and
ticket 004's tracker cascade into `MotionPlan::step()`, completing the
`StepState`/`StepInput`/`StepOutput`/`Status` contract from the issue's
`motion_plan.h` sketch.

**This is the highest-risk ticket in the sprint.** The terminal machine
governs exactly the kind of small terminal correction this project has
been burned by before — see `.clasi/knowledge/
encoder-wedge-boundary-latch.md` and `.clasi/knowledge/
wedge-latch-terminology-and-repro.md` before writing any terminal-machine
code. Every numeric constant and branch condition in this ticket's
acceptance criteria is a direct TRANSCRIPTION from the issue's "Control
laws and numbers" section — never re-derive one from first principles.

## Acceptance Criteria

- [ ] `StepState{dwellStart, sustainStart, lastReplan, replanCount,
      settling}` matches the issue's five-scalar sketch exactly — no
      additional mutable field anywhere in `source/drive/` (this is the
      subsystem's ONE statelessness residue; review-verify no other file
      in `source/drive/` gained a mutable member this ticket).
- [ ] Replan envelopes implement the issue's table exactly: `e_along`
      envelope = `40mm + 0.25s*|v_ref|`; `e_cross` envelope = `35mm`
      flat; `e_theta` envelope = `0.15rad + 0.20s*|omega_ref|`;
      trim-saturated trigger = saturated AND outside envelope; sustain
      `200ms`, rate limit `>=300ms`, N-max `3` -> `ABORT_REPLAN_LIMIT`.
- [ ] Terminal machine implements the issue's spec exactly: `t >= T_plan`
      -> `SETTLING` (omega trims off; along walk-in banded one-sided:
      inside tolerance -> literal `0.0f` + 150ms dwell; outside ->
      `clamp(k_s*e_along, 50, 100)` mm/s, NEVER negative; overshot ->
      `0.0f` and complete). Completion: `|e_along| <= 10-15mm AND |v_hat|
      <= 15mm/s` held 150ms -> the emitted setpoint snaps to a literal
      `0.0f`. Timeout `T_plan + 1.5s` -> complete-with-warning within 2x
      tolerance, else `ABORT_TIMEOUT`.
- [ ] Flying handoff implements the issue's spec: exhausted AND `e_cross
      <= 30mm`, `|e_theta| <= 5deg`, `e_along <= 0.14*vExit + 40mm` ->
      `DONE_HANDOFF`. Envelope violated -> `REPLAN_DUE` (same pure
      replan, never new geometry). Document (in this ticket's own code
      comments, not a separate design doc) the next-plan seeding
      contract — `entrySpeed = vExit` from the REFERENCE, `a = 0` — even
      though the actual `replan()`/next-`plan()` call is the caller's
      job (ticket 007): this ticket only emits the correct `Status` at
      the right time.
- [ ] Pose-fix step handling: `StepInput.poseStep`/`poseStepTheta`
      `<= 30mm/3deg` is absorbed by the ordinary trim law and resets the
      sustain timer; `> 30mm/3deg` bypasses the sustain filter and emits
      `REPLAN_DUE` immediately (rate limit + N-max still apply); a step
      arriving while `Status::SETTLING`'s dwell is counting does NOT
      reset or extend the dwell (the segment completes on its pre-step
      basis and reports honestly).
- [ ] `MotionPlan::step(const StepInput&, StepState*) const` composes:
      reference sample (ticket 003) -> tracker cascade (ticket 004) ->
      policy evaluation (this ticket) -> `StepOutput{command, status,
      record}`. `step()` NEVER calls `replan()` itself — it only emits
      `REPLAN_DUE` (the issue's explicit rule; grep/review-verify no
      `replan(` call exists inside `step()`'s call graph).
- [ ] `TrackRecord` carries every field from the issue's sketch (`in`,
      `ref`, `eAlong`/`eCross`/`eTheta`, `vTrim`/`omegaTrim`, `vCmd`/
      `omegaCmd`, `wheelLeft`/`wheelRight`, `trimSaturated`, `status`) —
      this is both the wire trace payload (ticket 009) and the tier-0
      replay payload (bit-exact replay from `TrackRecord.in` is SUC-002's
      requirement, verified in ticket 006).
- [ ] Purity/determinism test: the same `(plan, StepInput, StepState)`
      fed to `step()` twice produces byte-identical `StepOutput` and
      resulting `StepState`.
- [ ] Closed-loop scenario tests (against ticket 004's tracker + a plant
      stub, same "may be superseded by ticket 006" allowance as ticket
      004): (a) held short of tolerance does NOT report `DONE_STOP`
      before the dwell holds; (b) an overshot approach completes at a
      literal `0.0f`, never negative; (c) a pathological non-convergent
      plant produces `ABORT_TIMEOUT` at `T_plan + 1.5s`, not an infinite
      `SETTLING`; (d) two chained arc segments show no velocity
      discontinuity at the handoff boundary; (e) a handoff attempted
      outside the envelope emits `REPLAN_DUE`, never a silent
      `DONE_HANDOFF`; (f) a small (`<=30mm`) injected `poseStep` does not
      trigger `REPLAN_DUE` and resets the sustain timer; (g) a large
      (`>30mm`) injected `poseStep` triggers `REPLAN_DUE` on the same
      tick, bypassing sustain; (h) a `poseStep` injected during
      `SETTLING`'s dwell does not reset or extend the dwell.
- [ ] `uv run python -m pytest` passes.

## Testing

- **Existing tests to run**: `uv run python -m pytest`; ticket 002's grep
  isolation test; ticket 003/004's harnesses (must stay passing).
- **New tests to write**: the purity/determinism test; all eight
  closed-loop scenarios (a)-(h) above; a dedicated no-reversal regression
  test on the terminal machine specifically (never a negative wheel
  command anywhere in `SETTLING`).
- **Verification command**: `uv run pytest`

## Implementation Plan

**Approach**: implement and test the terminal machine and its
interaction with the one-sided clamp FIRST, in isolation, before wiring
the full `step()` composition — this is the ticket's highest-risk
surface. Re-read the issue's "Control laws and numbers" and "Terminal
(stop segments)" sections verbatim immediately before writing code.

**Files to create/modify**:
- `source/drive/policy.h`, `source/drive/policy.cpp`
- `source/drive/motion_plan.cpp` gains `step()`'s real composition body
  (the declaration and stub from ticket 003 are replaced)
- `tests/sim/unit/drive_policy_harness.cpp` + `test_drive_policy.py`
- Tier-0 Python scenario tests for (a)-(h) above (location: under
  `tests/_infra/drive/` if ticket 006 has landed by the time this ticket
  executes — check ticket sequencing before assuming; otherwise a
  temporary location under `tests/sim/unit/`, explicitly flagged in
  completion notes for ticket 006 to reconcile)

**Testing plan**: purity/determinism test; the eight closed-loop
scenarios; a dedicated terminal-machine no-reversal regression test.

**Documentation updates**: none.
