---
id: '006'
title: 'Sim tests: no-reverse trajectory-sampling proof for D/T/TURN/RT and G-unregressed
  check'
status: done
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

- [x] A Planner-level test for `D` samples the full commanded velocity
      trace and asserts it is `>= 0` throughout, arriving at rest at the
      commanded distance (SUC-002).
- [x] A Planner-level test for `T` (and, at minimum, a spot-check for bare
      `S`/`R`) samples the full commanded trace across cruise AND the
      stop-triggered re-solve, asserting `>= 0` throughout (SUC-003).
- [x] A Planner-level test for `TURN` and for `RT` samples the full
      commanded rotational trace and asserts no reversal relative to the
      commanded turn direction (SUC-005).
- [x] `j_max`/`yaw_jerk_max` sentinel-mapping coverage from ticket 002 is
      confirmed present and passing (SUC-004) — not re-implemented, just
      verified in scope.
- [x] `GOTO_GOAL`'s existing sim test suite is confirmed unchanged in
      pass/fail status from pre-sprint, with an explicit before/after list
      recorded in this ticket's completion notes (SUC-006).
- [x] `test_motion_commands_arc_turn.py`'s two documented `xfail`s: status
      (still `xfail` or flipped to pass) is explicitly recorded, not just
      "suite is green."
- [x] Full `uv run pytest` is green; no test silently deleted or skipped to
      achieve this.
- [x] **[Revision 2]** Divergence-replan trigger coverage exists for at
      least `D` and one of `TURN`/`RT` (sim-plant-lag injection or
      synthetic-observation Planner-tier tests, whichever this ticket's
      execution chooses — documented explicitly, not silently omitted).
- [x] **[Revision 2]** The no-reverse assertion is confirmed to hold across
      a trace that includes at least one replan, not only the un-replanned
      baseline trace.
- [x] **[Revision 2]** A test confirms no replan fires once the goal's stop
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

## Completion Notes

**Finding on entry: this ticket's own acceptance bar was already ~90% built
by tickets 003–005.** Each of those tickets landed its own no-reverse
trace-sampling scenarios (`D`: 5c; `T`: 6b; `TURN`: 7b; `RT`: 8b) AND its own
Revision 2 divergence-replan coverage — via option (b), unit-level
Planner-tier calls with synthetic/hand-constructed `MotorState`/
`PoseEstimate` observations (`scenarioDistanceGoalDivergenceReplanCorrects
LaggingPlant`/`GrossDivergenceReanchorsAfterStall`/`GuardSkipsNearTarget
BackwardReplan` for `D`; the `Turn`/`Rotation` analogs for `TURN`/`RT`) —
directly into `tests/sim/unit/planner_harness.cpp`, run by
`test_planner.py`. Rather than duplicate or rewrite that already-coherent,
already-well-organized suite, this ticket's own work was: (1) verify it by
running the full regression surface end to end, (2) cross-check every
acceptance-criteria phrase against what already exists, (3) fill the two
gaps that check found, (4) produce the explicit `G` before/after list and
xfail-status record the ticket itself requires.

**Gaps found and filled (2 new scenarios' worth of content in
`planner_harness.cpp` / `test_planner.py`'s docstring, no `source/`
change):**

1. **AC2's "at minimum, a spot-check for bare `S`/`R`" clause** had no
   direct Planner-level test driving `VELOCITY` (bare `R`'s own internal
   goal kind, `motion_commands.cpp::handleR`) through cruise AND a
   stop-triggered decel with an explicit no-reverse assertion — only `T`
   (`TIMED`) had the full trace test (6b); `VELOCITY`'s existing coverage
   (4c) was cruise-only (no stop, so no decel phase) and `STREAM`/bare-`S`'s
   (11b) was a re-target proof, not a decel proof. Added
   `scenarioVelocityGoalWithStopRuckigTraceNeverReversesThroughCruiseAndDecel`
   (4d): a `VELOCITY` goal with a caller `stop=t:300`, mirroring 6b's own
   assertion style (both channels, `>= 0` throughout, completes via
   `reason=time`).
2. **Guard 1 (stop-not-fired) had only CODE-level enforcement**
   (`planner.cpp`'s `tick()`: `if (!stopping_) { maybeReplan*(...); }`) and
   no dedicated EXTERNAL-observable scenario proving it the way guard 2
   (near-target no-reverse, 5f/7f) already had. Added
   `scenarioDistanceGoalGuardSkipsReplanOnceStopHasFired` (5g) and
   `scenarioTurnGoalGuardSkipsReplanOnceStopHasFired` (7g): each arms a
   SMOOTH stop-triggered decel, then feeds a synthetic observation showing
   a huge divergence (well past `kGrossDivergenceThreshold`/
   `kRotGrossDivergenceThreshold`) AFTER the stop has fired, and asserts the
   commanded output is unaffected (matches an undisturbed control run at the
   identical tick) — proving guard 1 blocks the replan regardless of the
   observation.
   - **Mutation-tested during authoring, not just written and trusted**: I
     built a scratch copy of `planner.cpp` with the `if (!stopping_)` gate
     removed, recompiled the harness against it, and confirmed both new
     scenarios correctly FAIL against that mutant (v_x flips 100→0 for `D`;
     omega flips -1.0→0 for `TURN`) while passing against the real code —
     i.e. these are not vacuously-passing assertions. This also surfaced and
     fixed two real test-design pitfalls before they shipped: (a) for `D`, a
     small synthetic snap (50mm) does NOT discriminate — once
     `armDistanceStopDecel()`'s own velocity-control re-solve is active,
     `linear_`'s position tracking is not meaningful, so the plan-remaining
     estimate stays large regardless of the observation; only a mid-range
     snap (300mm) reliably crosses the GROSS threshold. (b) for `TURN`, a
     100ms-later probe (mirroring `D`'s own timing) does NOT discriminate
     either — `yaw_acc_max=100rad/s²` makes the forced decel so fast
     (~20ms) that both the guarded and unguarded builds have already fully
     converged to `omega=0` by then; the probe had to land ~10ms after
     arming, INSIDE that short window, and the comparison tolerance had to
     be tightened from 1.0 to 0.3 (the mutant's own diff of exactly 1.0
     landed ON a 1.0 tolerance boundary in an early draft and silently did
     not fail — `>` not `>=`). Both pitfalls, and the fixes, are documented
     inline at each scenario.

**AC-by-AC disposition (phrase-by-phrase, all traced to `usecases.md`
SUC-002–SUC-006):**

- **AC1 (`D`, SUC-002)**: `scenarioDistanceGoalRuckigTraceNeverReverses`
  (planner_harness.cpp, run by `test_planner.py`) — full commanded velocity
  trace `>= 0`, arrives at rest at the commanded 500mm, completes via
  `reason=dist`. Pre-existing (089-003), re-verified.
- **AC2 (`T`/`S`/`R`, SUC-003)**:
  `scenarioTimedGoalBothChannelsRuckigTraceNeverReverseAndCompleteViaTime`
  (`T`, both channels, cruise+decel, pre-existing 089-004) +
  `scenarioVelocityGoalWithStopRuckigTraceNeverReversesThroughCruiseAndDecel`
  (bare `R`'s own goal kind, NEW this ticket, item 1 above).
  `test_motion_overshoot_regression.py`'s `D`/`T`-class bars
  (`test_d_200_200_500_stops_within_tight_tolerance_of_commanded_distance`)
  re-run green, unchanged tolerance (1.5%/7.5mm).
- **AC3 (`TURN`/`RT`, SUC-005)**: `scenarioTurnGoalRuckigTraceNeverReverses`
  / `scenarioRotationGoalRuckigTraceNeverReverses` — full rotational trace,
  no reversal relative to the commanded turn direction, both pre-existing
  (089-005), re-verified.
- **AC4 (`j_max`/`yaw_jerk_max`, SUC-004)**: confirmed present and passing,
  NOT re-implemented — `tests/sim/unit/test_jerk_trajectory.py` ->
  `jerk_trajectory_harness.cpp`'s `scenarioJerkSentinelMapsToInfinity`
  (ticket 002). Verified via `uv run pytest tests/sim/unit/
  test_jerk_trajectory.py -v`: 2 passed.
- **AC5 (`GOTO_GOAL`, SUC-006)**: see the explicit before/after list below.
- **AC6 (`test_motion_commands_arc_turn.py` xfails)**: see the xfail
  disposition below.
- **AC7 (full suite green)**: `uv run python -m pytest tests/sim` — **308
  passed, 2 xfailed, 0 failed in 164.53s (2:44)**. Zero tests deleted or
  skipped; the 2 `xfail`s are the pre-existing, unrelated
  `test_tour_geometry.py` pair (see below), not silently introduced by this
  ticket.
- **AC8 (Revision 2, divergence-replan coverage for `D` + one of
  `TURN`/`RT`)**: option (b) chosen (already chosen by tickets 003/005, not
  re-decided here) — synthetic-observation Planner-tier `tick()` calls, not
  a sim-plant-lag knob. Checked `tests/sim`'s existing plant-model knobs
  before accepting that choice (per this ticket's own instruction): option
  (a) DOES exist and is reachable —
  `tests/_infra/sim/firmware.py`'s `Sim.set_enc_slip(side, fraction)` and
  `Sim.set_motor_lag(side, tau)` inject exactly this kind of tracking lag
  into the FULL plant simulation, via the wire-level `sim` fixture (used by
  `test_stiction_and_motor_lag.py`/`test_otos_error_injection.py`, among
  others) — so the earlier architecture-doc framing ("the sim's idealized
  plant tracks too well to produce natural divergence without an injected-
  lag knob") is confirmed accurate on the knob's EXISTENCE, but option (b)
  was still the right call to keep: those knobs operate through the whole
  `Drivetrain`/`Hal::MotorVelocityPid`/encoder pipeline, so hitting a
  PRECISE, repeatable divergence value at an exact tick (needed to place a
  test right at, or just under/over, `kDivergenceThreshold`/
  `kGrossDivergenceThreshold`/`kMinReplanInterval`'s own boundaries, the way
  5f/7f/5g/7g all do) would require iteratively tuning a slip-fraction/
  lag-tau pair against the REAL Ruckig-solved dynamics with no direct lever
  on the resulting divergence number — whereas option (b) sets the exact
  observation value directly, isolating the trigger's own guard logic from
  plant-model noise. This matches the architecture doc's own recommendation
  ("(b) is likely the lower-risk, more direct option") and is why this
  ticket did not switch mechanisms during consolidation. Coverage exists
  for BOTH `D` (`scenarioDistanceGoalDivergenceReplanCorrectsLaggingPlant` /
  `GrossDivergenceReanchorsAfterStall`) AND `TURN`+`RT`
  (`scenarioTurnGoalDivergenceReplanCorrectsLaggingPlant` /
  `GrossDivergenceReanchorsAfterStall`, `scenarioRotationGoalDivergence
  ReplanCorrectsLaggingPlant`) — exceeding the "at least `D` and one of
  `TURN`/`RT`" bar.
- **AC9 (Revision 2, no-reverse across a replanned trace)**: the same
  lagging-plant/stalled-plant scenarios above assert `checkFalse(everReversed,
  ...)` across their WHOLE trace, which includes one or more retarget()/
  reanchor() replans by construction (that is the scenario's own point) —
  not just the un-replanned baseline (5c/7b/8b cover that baseline
  separately).
- **AC10 (Revision 2, guard 1 + guard 2)**: guard 2 (no-reverse-target,
  projected remaining `<= 0`) was already covered
  (`scenarioDistanceGoalGuardSkipsNearTargetBackwardReplan` /
  `scenarioTurnGoalGuardSkipsNearTargetBackwardReplan`, 089-003/005). Guard
  1 (stop-not-fired) was NOT independently covered by an external-observable
  scenario before this ticket — added this ticket (item 2 above), and
  mutation-tested to confirm real detection power.

**`GOTO_GOAL` (SUC-006) explicit before/after list.** `G`'s own code path
(`Motion::VelocityRamp`, `pursueSteer()`/`enterPursue()`/`PRE_ROTATE`/
`PURSUE`) is confirmed BYTE-FOR-BYTE unchanged across every commit of this
sprint (`git diff b90feeed..HEAD -- source/motion/velocity_ramp.cpp` is
empty; `velocity_ramp.h`'s only diff is a doc-comment sentence narrowing
"sole caller" from `TURN`/`ROTATION`/`GOTO_GOAL` to `GOTO_GOAL` alone, no
behavior change). `tests/sim/unit/test_motion_commands_goto.py` itself has
**zero diff** since its creation in ticket 084-004
(`git diff b90feeed..HEAD -- tests/sim/unit/test_motion_commands_goto.py`
is empty) — not touched by any ticket in this sprint. Before/after, run
fresh as part of this ticket's own verification:

| Test | Before (pre-089, unchanged code) | After (this ticket's run) |
|---|---|---|
| `test_g_reaches_relative_target_and_emits_done_reason_pos` | PASS | PASS |
| `test_g_pre_rotates_when_bearing_exceeds_the_gate` | PASS | PASS |
| `test_g_does_not_pre_rotate_when_bearing_is_within_the_gate` | PASS | PASS |
| `test_g_short_distance_within_arrive_tol_completes_almost_immediately` | PASS | PASS |
| `test_g_distance_just_over_arrive_tol_requires_travel_before_completing` | PASS | PASS |
| `test_g_range_validation` | PASS | PASS |
| `test_g_too_few_args_rejected_with_badarg` | PASS | PASS |
| `scenarioGotoGoalPursuesDirectlyWhenBearingWithinGate` (planner_harness.cpp) | PASS | PASS |
| `scenarioGotoGoalPreRotatesThenPursuesAndArrives` (planner_harness.cpp) | PASS | PASS |

All 9 unchanged, all still passing — no diff-style status change in either
direction. (The two `planner_harness.cpp` scenario BODIES are themselves
confirmed unchanged by this sprint via `git diff b90feeed..HEAD --
tests/sim/unit/planner_harness.cpp`, which shows only unchanged context
lines around their two `main()` call sites, not a modification to the
functions themselves.)

**`test_motion_commands_arc_turn.py`'s two documented `xfail`s — explicit
status.** BOTH FLIPPED TO PASSING, by ticket 089-005 (before this ticket
started, re-confirmed by this ticket's own run, not silently re-asserted):

- `test_rt_rotates_about_90_degrees_and_emits_done_rot` — no `@pytest.mark.
  xfail` decorator present; runs as a normal (non-xfail) PASS, measuring
  95.70687deg (+5.71deg over the commanded 90deg), inside the file's
  existing ±7deg bound.
- `test_rt_negative_relangle_rotates_the_opposite_direction` — same
  disposition, symmetric (-95.70687deg).

Grepped the file directly (`grep -n "@pytest.mark.xfail"
tests/sim/unit/test_motion_commands_arc_turn.py`) to confirm zero
`xfail` decorators remain anywhere in it — not just these two tests.
Separately, and NOT to be confused with the two tests above:
`tests/sim/system/test_tour_geometry.py` carries its OWN, DIFFERENT pair of
`@pytest.mark.xfail(strict=True)` tests
(`test_tour1_every_leg_matches_commanded_geometry_and_settles_cleanly` /
`test_tour2_...`), STILL `xfail` after this ticket's own fresh run (not
flipped) — these were already `xfail` before 089-005 and remain so; ticket
005's own completion notes record they were confirmed byte-identical
before/after 089-005 via a direct git-stash A/B comparison, and this
ticket's own targeted run (`uv run pytest tests/sim/system/
test_tour_geometry.py -rA`) reproduces the SAME two `XFAIL`s (not `XPASS`),
consistent with that record — the closed-form dead-time fix that unblocked
the `D`-leg tolerance is a no-op on the RT-leg heading check at this
config's `omega`/`yaw_acc_max` (per that test's own xfail reason text), so
`RT`'s improved sim accuracy (±5.71deg vs the file's own ±7deg bound) is
not, by itself, enough to clear the Tour's own ±8deg per-leg bound in this
particular multi-leg sequenced scenario.

**Test results.** Full `uv run python -m pytest tests/sim`:

```
308 passed, 2 xfailed in 164.53s (0:02:44)
```

Identical pass/xfail COUNT to ticket 005's own recorded "308 passed, 2
xfailed, 0 failed" — expected and correct, not a red flag: this ticket's 3
new C++-level scenarios (4d, 5g, 7g — the latter two mutation-tested) all
live inside `planner_harness.cpp`, exercised by the SAME single pytest-level
test (`test_planner.py::test_planner_harness_compiles_and_passes`) tickets
003–005 already used — adding scenarios inside that one compiled binary does
not change the pytest COLLECTION count, only what that one test's own
binary proves internally (confirmed directly: running the compiled harness
binary stand-alone prints one `---` line per scenario, including the 3 new
ones, all passing).

**Deviations from the ticket text.** None of substance. The ticket's own
"Files to modify/create" left the exact file layout as an implementation
decision; given tickets 003–005 had already built a coherent, well-commented
consolidation-ready suite (each ticket's own header comment in
`planner_harness.cpp` already narrates the sprint-wide history), a full
reorganization into a different file/fixture layout was judged to add
churn/regression-risk without improving clarity, so this ticket's own work
was additive (2 new scenarios, ~90 lines) plus verification/documentation,
not a rewrite — consistent with the ticket's own "do not just duplicate
them" instruction (nothing WAS duplicated; the 2 additions are the only
genuinely new content) and its "exact file layout is an implementation
decision" latitude.
