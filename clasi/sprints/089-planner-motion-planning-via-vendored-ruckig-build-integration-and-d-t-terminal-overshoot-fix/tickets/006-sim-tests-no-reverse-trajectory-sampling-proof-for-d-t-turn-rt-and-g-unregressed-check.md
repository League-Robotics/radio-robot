---
id: '006'
title: 'Sim tests: no-reverse trajectory-sampling proof for D/T/TURN/RT and G-unregressed
  check'
status: open
use-cases:
- SUC-002
- SUC-003
- SUC-004
- SUC-005
- SUC-006
depends-on:
- '004'
- '005'
github-issue: ''
issue: planner-motion-planning-via-vendored-ruckig.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sim tests: no-reverse trajectory-sampling proof for D/T/TURN/RT and G-unregressed check

## Description

Tickets 002-005 each added narrowly-scoped tests alongside their own
implementation (the wrapper's own unit tests, and per-goal-kind Planner
tests). This ticket is the CONSOLIDATION pass: a coherent, end-to-end sim
test suite proving the sprint's actual acceptance bar across every migrated
goal kind together, plus the explicit regression guard for `GOTO_GOAL`
(the one goal kind that did NOT migrate). This is what `test_ruckig_smoke
.py` proved for a hand-built `InputParameter` in isolation — this ticket
proves the same property against the REAL goal-staging path
(`Planner::apply()`/`tick()`), for every migrated wire verb, and is the
gate before the bench pass (ticket 007) — the sim cannot itself prove
no-reverse on real hardware, but a sim regression here must be caught
before spending bench time.

## Implementation Plan

**Approach**:
1. Consolidate/organize the Planner-level trajectory-sampling tests tickets
   003-005 introduced into a clear, complete suite (do not just duplicate
   them — refactor into one well-organized file/fixture set if that reads
   better than what accumulated ticket-by-ticket). Cover, for EACH of `D`,
   `T`, `TURN`, `RT`: the full commanded trace across the whole goal
   (ramp-up/solve, cruise where applicable, stop-triggered or natural
   convergence to rest) never reverses sign relative to the commanded
   direction.
2. Add the explicit `GOTO_GOAL` regression guard (SUC-006): confirm `G`'s
   existing sim test suite (`PRE_ROTATE`/`PURSUE` behavior,
   `pursueSteer()`/`enterPursue()` code paths) has NOT changed pass status
   from pre-sprint — a clean diff-style check (list of test names +
   pass/fail) is acceptable, not necessarily new test content.
3. Run the FULL existing regression surface and record status changes
   explicitly (not just "still green"): `test_motion_commands*.py`,
   `test_motion_commands_arc_turn.py`, `test_motion_overshoot_regression.py`,
   `tests/sim/system/test_tour_geometry.py`. Confirm the two documented RT
   `xfail`s are either still `xfail` or have flipped to passing (not
   silently deleted or skipped).
4. Confirm test coverage traces to every SUC in `usecases.md` this ticket
   claims (SUC-002 through SUC-006) — cross-check acceptance criteria
   phrase-by-phrase, not just informally.
5. **[Revision 2, post-stakeholder-design-discussion] Divergence-replan
   coverage** (architecture-update.md Decision 10): the sim's idealized
   plant tracks the commanded plan closely enough that it is UNLIKELY to
   produce natural divergence beyond threshold on its own — do not rely on
   the plant to exercise this path. Cover it either with (a) a sim
   plant-lag injection knob (if one exists or is cheap to add — check
   `tests/sim`'s existing plant-model knobs before adding a new one) that
   deliberately desyncs measured from commanded, or (b) unit-level
   Planner-tier tests that call `tick()` with SYNTHETIC observations
   (hand-constructed `MotorState`/`PoseEstimate` values showing more
   remaining than the plan expects) rather than driving them through the
   full plant simulation — (b) is likely the lower-risk, more direct option
   given how narrowly-scoped the trigger's own logic is (a threshold
   comparison plus a guarded call), and mirrors this ticket's own existing
   pattern of testing the PLAN via direct Planner-level calls rather than
   only through full plant convergence (architecture-update.md Step 2, row
   6's rationale). Either way, confirm: (i) a replan fires when a
   synthetic/injected divergence crosses the threshold with the stop
   condition not yet fired; (ii) no replan fires once the stop condition has
   fired; (iii) no replan is requested when the (projected) measured
   remaining is `<= 0` (the no-reverse guard); (iv) the existing no-reverse
   trace assertion (this ticket's own core property — `D`/`T`/`TURN`/`RT`
   never reverse) ALSO holds across traces that include one or more
   replans, not just the un-replanned baseline case.

**Files to modify/create**: consolidate/organize files under
`tests/sim/unit/` and `tests/sim/system/` — exact file layout is an
implementation decision, but must not silently drop coverage tickets
003-005 already added.

**Testing plan**: this ticket IS test content — no separate "testing plan"
beyond running the full suite and confirming the consolidated result.

**Documentation updates**: none expected beyond test docstrings.

## Acceptance Criteria

- [ ] A Planner-level test for `D` samples the full commanded velocity
      trace and asserts it is `>= 0` throughout, arriving at rest at the
      commanded distance (SUC-002).
- [ ] A Planner-level test for `T` (and, at minimum, a spot-check for bare
      `S`/`R`) samples the full commanded trace across cruise AND the
      stop-triggered re-solve, asserting `>= 0` throughout (SUC-003).
- [ ] A Planner-level test for `TURN` and for `RT` samples the full
      commanded rotational trace and asserts no reversal relative to the
      commanded turn direction (SUC-005).
- [ ] `j_max`/`yaw_jerk_max` sentinel-mapping coverage from ticket 002 is
      confirmed present and passing (SUC-004) — not re-implemented, just
      verified in scope.
- [ ] `GOTO_GOAL`'s existing sim test suite is confirmed unchanged in
      pass/fail status from pre-sprint, with an explicit before/after list
      recorded in this ticket's completion notes (SUC-006).
- [ ] `test_motion_commands_arc_turn.py`'s two documented `xfail`s: status
      (still `xfail` or flipped to pass) is explicitly recorded, not just
      "suite is green."
- [ ] Full `uv run pytest` is green; no test silently deleted or skipped to
      achieve this.
- [ ] **[Revision 2]** Divergence-replan trigger coverage exists for at
      least `D` and one of `TURN`/`RT` (sim-plant-lag injection or
      synthetic-observation Planner-tier tests, whichever this ticket's
      execution chooses — documented explicitly, not silently omitted).
- [ ] **[Revision 2]** The no-reverse assertion is confirmed to hold across
      a trace that includes at least one replan, not only the un-replanned
      baseline trace.
- [ ] **[Revision 2]** A test confirms no replan fires once the goal's stop
      condition has fired, and no replan is requested when projected
      measured remaining is `<= 0`.

## Testing

- **Existing tests to run**: the full suite — this ticket's job IS running
  and consolidating it.
- **New tests to write**: consolidated `D`/`T`/`TURN`/`RT` trajectory-
  sampling coverage (may reorganize, not just append to, tickets 003-005's
  test additions) plus the explicit `G` before/after status check.
  **[Revision 2]** Plus divergence-replan coverage (injected-lag or
  synthetic-observation based) for at least `D` and one of `TURN`/`RT`.
- **Verification command**: `uv run pytest tests/sim` (full sim suite),
  with results recorded in this ticket's completion notes.
