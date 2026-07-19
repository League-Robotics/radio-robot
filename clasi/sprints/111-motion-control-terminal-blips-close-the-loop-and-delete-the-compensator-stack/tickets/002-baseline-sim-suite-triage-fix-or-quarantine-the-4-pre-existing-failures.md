---
id: '002'
title: 'Baseline sim-suite triage: fix or quarantine the 4 pre-existing failures'
status: open
use-cases: [SUC-004]
depends-on: ['001']
github-issue: ''
issue: motion-control-terminal-blips-reconciled-fix-plan.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Baseline sim-suite triage: fix or quarantine the 4 pre-existing failures

## Description

Master currently carries 4 failing sim tests, introduced by already-merged
`pid-debugging` WIP (a motor-interface refactor + a sim-behavior change +
an in-place cycle-order/request-collect reorder in `robot_loop.cpp` — all
three named in the driving issue's own §5 corrections section).
`close_sprint` runs `uv run pytest` — this sprint (and the rest of the
arc) cannot close until the suite is green. For EACH of the 4 tests:
confirm (do not assume) whether the failure traces to the in-place
`robot_loop.cpp` reorder; fix at the root cause if not, or quarantine
(`xfail`/`skip`, cited) only if it does. Do not blanket-quarantine all 4
under a convenient "it's just WIP" umbrella — a failure not actually
coupled to the reorder is a real, fixable regression, and quarantining it
without justification would hide it.

## Pre-gathered evidence (starting hypotheses — VERIFY before acting)

Captured via an actual `uv run pytest` run during sprint planning. Each
disposition below is a hypothesis grounded in real repro output, not a
final answer — confirm before fixing or quarantining.

1. **`src/tests/sim/plant/test_plant.py::test_plant_harness_compiles_and_passes`**
   — fails: "OtosPlant's simulated heading tracks Odometry's own heading
   closely (same wheel positions, same kinematics) — expected 2.34615,
   got 1.65345 (tol 0.05)" during a pivot scenario.
   `plant_harness.cpp`'s own source list (`_HARNESS_SRC`, `_WHEEL_PLANT_SRC`,
   `_OTOS_PLANT_SRC`, `_SIM_PLANT_SRC`, `_ODOMETRY_SRC`, `_VELOCITY_PID_SRC`,
   `_NEZHA_MOTOR_SRC`, `_OTOS_SRC`, `_BODY_KINEMATICS_SRC`) never compiles
   or links `robot_loop.cpp`/`pilot.cpp` — this test **cannot structurally
   be coupled** to the cycle-order reorder (that code isn't in the binary).
   **Hypothesis: FIX** — a real plant/kinematics inconsistency, most likely
   from the motor-interface refactor (commit `5f5a2ba7`). Investigate
   `OtosPlant`'s heading computation vs. `Odometry::integrate()`'s for a
   scale/sign/trackwidth convention drift between the two.
2. **`src/tests/sim/system/test_sim_api.py::test_sim_api_harness_compiles_and_passes`**
   — fails a hardcoded timing assertion expecting `kPace=28ms`/
   `virtualCycleMillis=40ms` (i.e. `kCycle=40`, `kSettle=4`). The ACTUAL
   current `robot_loop.cpp` constants are `kCycle=20`, `kSettle=0`,
   `kClear=0` (so `kPace=20`) — independently confirmed by the driving
   issue's own §5: "Cycle time is 20ms, not 50ms... robot_loop.cpp:25 now
   has kCycle = 20." This is a stale hardcoded TEST expectation, not a
   coupling to the live reorder (the reorder is about call ORDER within
   `cycle()`, not the `kCycle` numeric value).
   **Hypothesis: FIX** — update `sim_api_harness.cpp`'s (or
   `test_sim_api.py`'s) hardcoded expected `kPace`/`virtualCycleMillis`/
   sleep-count numbers to match `kCycle=20`/`kSettle=0`/`kClear=0`. Note
   while fixing: `src/firm/app/telemetry.h`'s `kPrimaryPeriod` is still
   `40` — `robot_loop.cpp`'s own doc comment claims "kCycle matches
   Telemetry::kPrimaryPeriod by construction," which is currently FALSE
   (20 != 40). Do not silently "fix" this mismatch as part of this
   ticket (it is outside this sprint's scope — no ticket here touches
   `robot_loop.cpp`'s timing constants or `telemetry.h`); file a fresh
   `clasi/issues/` entry noting the drift for a future sprint instead.
3. **`src/tests/sim/unit/test_app_robot_loop.py::test_app_robot_loop_harness_compiles_and_passes`**
   — fails `ScriptedI2CBus` "no script under-run: motor/otos (cycles)"
   and "...(config-dispatch cycles)" — in exactly the two scenarios that
   exercise `cycle()` (the "(boot)" scenarios pass). `robot_loop.cpp`
   carries an explicit in-code comment: "NOTE! These requests and collects
   have been reordered for testing and development and will need to be
   reverted to their original positions before running on hardware."
   This harness DOES link `robot_loop.cpp` in full.
   **Hypothesis: QUARANTINE** — `xfail`/`skip` the two failing scenarios
   (or the whole test if scenarios cannot be split) with a comment citing
   `clasi/issues/cycle-order-reorder-experiment-ab-before-hardware.md`.
   Do NOT fix by rewriting `app_robot_loop_harness.cpp`'s own
   `ScriptedI2CBus` script to match the reordered sequence — that would
   bake today's temporary experiment into a permanent test fixture,
   working against the deferred issue's own intent to A/B-compare before
   hardware.
4. **`src/tests/sim/system/test_profiled_motion_sim.py::test_profiled_turn_leg_sim_ramp_shape_and_heading_target`**
   — fails a cruise-plateau shape assertion; the printed `velL`/`velR`
   trace visibly oscillates between roughly half and full commanded speed
   almost every sample (e.g. `-6.3, -81.8, -38.1, -76.1, -52.2, -33.8,
   -63.4, ...`) — a signature consistent with a stale/alternating encoder
   read. `profiled_motion_harness.cpp` DOES link the full `robot_loop.cpp`/
   `pilot.cpp` graph (its own `_APP_SOURCES` list includes both).
   **Hypothesis: plausible reorder coupling, NOT yet confirmed** — this
   is the one case genuinely requiring investigation before disposing.
   Confirm by comparing the oscillation pattern against what the
   request/collect reorder in `robot_loop.cpp` (the same "NOTE!" comment
   as case 3) would predict (e.g. temporarily reverting the reorder
   locally — NOT committing that revert, this sprint must not touch
   `robot_loop.cpp` — and re-running the test to see if the oscillation
   disappears is a legitimate diagnostic step, distinct from landing the
   revert). If confirmed reorder-coupled: quarantine, citing the same
   deferred issue as case 3. If NOT confirmed (e.g. the oscillation
   persists with the reorder reverted locally): this is a genuine,
   separate regression — fix at its root cause instead, and say so
   explicitly in the ticket's own completion notes rather than
   quarantining by default.

## Acceptance Criteria

- [ ] Each of the 4 tests has an individually investigated and justified
      disposition (fixed-with-root-cause-stated, or
      quarantined-with-citation) recorded in this ticket's own completion
      notes — not a blanket disposition applied to all 4.
- [ ] No test is quarantined without concrete, checked evidence it traces
      to the `robot_loop.cpp` reorder (does the harness even link
      `robot_loop.cpp`; does the failure signature match a
      bus-transaction-order or stale-read symptom) — "lives in the same
      test tier" or "was probably WIP" is not sufficient justification.
- [ ] Any quarantine (`xfail`/`skip`) carries a comment citing
      `clasi/issues/cycle-order-reorder-experiment-ab-before-hardware.md`.
- [ ] `robot_loop.cpp`'s cycle order and request/collect sequencing are
      NOT modified by this ticket (a local, uncommitted revert purely as
      a diagnostic step to confirm case 4's hypothesis is fine; landing
      that revert is not).
- [ ] The `kCycle`/`kPrimaryPeriod` mismatch discovered while fixing case
      2 is flagged (a fresh `clasi/issues/` entry or a note in this
      ticket's completion notes) but NOT fixed in this ticket.
- [ ] `uv run pytest` exits 0 (green: pass or intentional, cited
      xfail/skip) across the full suite, including ticket 001's new
      harness.

## Implementation Plan

**Approach**: per-test investigate-then-act, using the pre-gathered
evidence above as a starting point, not a final verdict. For each of the
4 tests: reproduce the failure, confirm or refute the hypothesis above,
then apply exactly one of (a) a root-cause fix or (b) a cited quarantine.

**Files likely to modify** (confirm exact scope during investigation):
- `src/tests/sim/plant/test_plant.py` and/or `src/tests/sim/plant/
  plant_harness.cpp` and/or the plant sources it links
  (`src/tests/sim/plant/otos_plant.{h,cpp}`, `src/firm/app/odometry.{h,cpp}`)
  — case 1.
- `src/tests/sim/system/test_sim_api.py` and/or `src/tests/sim/system/
  sim_api_harness.cpp` — case 2 (test-only fix, no firmware change).
- `src/tests/sim/unit/test_app_robot_loop.py` and/or `src/tests/sim/unit/
  app_robot_loop_harness.cpp` — case 3 (xfail/skip the two `cycle()`
  scenarios, cited).
- `src/tests/sim/system/test_profiled_motion_sim.py` and/or
  `src/tests/sim/system/profiled_motion_harness.cpp` — case 4 (disposition
  depends on investigation).

**Testing plan**: after each test's own fix/quarantine, run that test in
isolation to confirm the intended outcome, then run the full suite.

**Documentation updates**: none required in `src/firm/*/DESIGN.md` (no
firmware behavior changes are expected from this ticket — case 1's fix,
if a firmware bug is found, is the only case where a DESIGN.md touch-up
might be warranted; note it in that case's own completion notes).

## Testing

- **Existing tests to run**: `uv run pytest` (full suite).
- **New tests to write**: none — this ticket fixes/quarantines existing
  tests, it does not add new ones.
- **Verification command**: `uv run pytest` exits 0.
