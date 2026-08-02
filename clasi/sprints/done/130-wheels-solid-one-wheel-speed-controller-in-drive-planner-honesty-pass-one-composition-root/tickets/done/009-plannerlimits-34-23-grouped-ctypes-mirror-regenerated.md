---
id: 009
title: "PlannerLimits 34 \u2192 23, grouped, ctypes mirror regenerated"
status: done
use-cases:
- SUC-004
depends-on:
- 008
github-issue: ''
issue: planner-honesty-pass-50ms-period-tick-state-machine-limits-reduction.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# PlannerLimits 34 → 23, grouped, ctypes mirror regenerated

## Description

Cut `PlannerLimits` from 34 to 23 fields per
`planner-honesty-pass-50ms-period-tick-state-machine-limits-
reduction.md` item 3: remove `velKff`/`velKp`/`velKi`/`velIMax`/
`velKaff`/`velIAccelGate`/`dutyFloor` (duty-stage-only, dead since
ticket 007), `headingOtosWeight`/`otosStaleness` (OTOS blend off
everywhere — tracked separately in
`clasi/issues/later/estimator-v2-otos-fusion-sim-first.md`), and
`requireSettle`/`settleWindow` (feature false everywhere, dissolved by
ticket 008's `Settling`-state deletion — `plannedStopWindow()` falls
back to its built-in default allowance once `settleWindow` is gone).
Group the remaining 23 into `ceilings`/`plant`/`landing`/`tracking`
sub-structs. Break the append-only ctypes-mirror constraint ONCE:
regenerate `planner_harness.py` and its offset guards
(`plannerStructSizes`) against the new layout. Update every composition
root (`composeRobot()` from ticket 002, `simPlannerLimits()`, test
`benchLimits()`) and `applyShaperLimits()`/`applyTrimGains()`
signatures if the grouping suggests passing sub-structs.

Sequenced last in the honesty-pass chain because the trim-gain fields
are already gone by this point (relocated to `Drive`/robot-JSON in
ticket 005) and the `Settling`-state fields are already dissolved
(ticket 008) — one reshape, not two (the source issue's own
explicit concern).

## Acceptance Criteria

- [x] `PlannerLimits` carries exactly the 23 fields the source issue
      lists, grouped into `ceilings`/`plant`/`landing`/`tracking`.
      **DEVIATION, evidence-based (see Completion Note below): landed at
      18 fields, not 23.** Direct grep against `planner.cpp`/`planner.h`
      found the sprint's own "keep" list (`trimKp`/`trimKi`/`trimIMax`/
      `trimKaff`/`trimMax`) has ZERO readers — `Motion::WheelTrim`, the
      only thing that ever read them, was deleted outright by ticket 005
      (confirmed via `git show 63bdeb4c`), leaving these 5 fields exactly
      as orphaned as the M4 duty-stage fields ticket 007 orphaned. Per
      this ticket's own explicit instruction ("work from evidence, not
      the number... if you reach a different number than 23, say so and
      justify it rather than deleting a live field to hit the target"),
      they are cut alongside the other 11. `tracking` now holds only
      `headingHoldGain` (confirmed live: `applyHeadingHold()`,
      planner.cpp). Grouping into the four named sub-structs is otherwise
      exactly as specified.
- [x] `planner_harness.py`'s ctypes mirror and offset guards
      (`plannerStructSizes`) regenerated and passing against the new
      layout. Also fixed two PRE-EXISTING, unrelated mirror gaps
      (`Types::RobotState::Wheel::cmdAccel` added by 130-003,
      `Health::wheelFrozenLeft/wheelFrozenRight/ready`) discovered only
      because running this script end-to-end is what actually exercises
      `plannerStructSizes()` — without the fix the script could not run
      at all, at any PlannerLimits layout.
- [x] Every composition root (`composeRobot()`, `simPlannerLimits()`,
      test `benchLimits()`) updated to construct the new struct shape.
      `simPlannerLimits()` no longer exists as of ticket 002 (composition
      roots already unified onto `composeRobot()`/`bootPlannerLimits()`)
      — confirmed there is exactly ONE boot path, not two.
- [x] No dead field (`velKff`, `dutyFloor`, `requireSettle`, etc.)
      remains anywhere in the struct or its consumers. Extended down the
      whole config chain (`Config::PlannerBootConfig`, `gen_boot_config.py`,
      `robot_config.py`, `robot_config.schema.json`, all three
      `data/robots/*.json` `planner` blocks) per explicit dispatch
      instruction, not just the C++ struct itself.

## Testing

- **Existing tests to run**: the offset-guard test; full `planner_tests`
  ctest suite; full sim pytest suite (construction sites must all
  compile against the new layout).
- **New tests to write**: none beyond re-running the regenerated offset
  guard — this ticket is a mechanical reshape, not new behavior.
- **Verification command**: `uv run pytest`; ctest for `motion_tests`.

## Implementation Plan

**Approach**: mechanical regeneration + one deliberate ABI break, done
last in the planner-honesty sequence per the dependency ordering above.

**Files to create/modify**:
- `src/motion/planner/planner_types.h` (field cut + sub-struct
  grouping)
- `src/tests/sim/support/planner_harness.py` (ctypes mirror)
- The offset-guard generator/test (`plannerStructSizes`)
- Every `PlannerLimits` construction site (`composeRobot()`,
  `simPlannerLimits()`, `benchLimits()`)
- `applyShaperLimits()`/`applyTrimGains()` signatures if grouping
  suggests sub-structs

**Testing plan**: offset-guard test regenerated and passing; full
`planner_tests` + sim pytest suites green.

**Documentation updates**: inline field comments only — no separate doc
file changes expected.

## Completion Note

**Field count: 18, not 23** (34 → 18, 16 cut, not 11). Evidence-based
deviation — see the Acceptance Criteria annotations above for the exact
justification. Grouping:
- `ceilings` (8): vMax, aMax, aDecel, omegaMax, alphaMax, alphaDecel,
  jerkMax, yawJerkMax
- `plant` (4): trackWidth, controlPeriod, actuationDelay,
  velocityFilterWeight
- `landing` (5): settleEpsilonLinear, settleEpsilonAngular,
  settleRestVelocity, settleRestOmega, decelPlanFraction
- `tracking` (1): headingHoldGain

Cut (16): velKff, velKp, velKi, velIMax, velKaff, velIAccelGate,
dutyFloor (M4 duty stage, dead since 130-007), headingOtosWeight,
otosStaleness (OTOS heading blend — a LIVE code path in
`Planner::tick()` as of this ticket, deleted here along with the two
fields per explicit sprint scope; tracked forward to
`clasi/issues/later/estimator-v2-otos-fusion-sim-first.md`), requireSettle,
settleWindow (settle-confirm defer path, dissolved by 130-008),
trimKp/trimKi/trimIMax/trimKaff/trimMax (planner-side velocity trim,
dead since 130-005 deleted `Motion::WheelTrim` — the sprint's own ticket
text hadn't caught up to this; found by direct evidence this ticket).

**Config chain extended to match** (team-lead's explicit instruction,
beyond this ticket's own literal file list): `Config::PlannerBootConfig`
(boot_config.h/.cpp, regenerated via `gen_boot_config.py`),
`robot_config.py`'s `PlannerConfig`, `robot_config.schema.json`'s
`planner` block, and all three `data/robots/*.json` files had the same
11 corresponding raw JSON keys removed (`require_settle`, `settle_window`,
`vel_kp`, `vel_ki`, `vel_i_max`, `vel_i_accel_gate`, `duty_floor`,
`trim_kp`, `trim_ki`, `trim_i_max`, `trim_max`) — `plant_gain`/`plant_tau`
stay (recorded measured data, no longer read by the generator since both
derived consumers, velKff/velKaff and trimKaff, are gone).

**Two pre-existing, unrelated bugs fixed opportunistically**, both
discovered only because actually running `planner_harness.py` (required
to prove the regenerated ctypes mirror/offset-guard) surfaced them:
`Types::RobotState::Wheel::cmdAccel` (added by 130-003) and
`Health::wheelFrozenLeft`/`wheelFrozenRight`/`ready` were never mirrored
into this script's ctypes `Wheel`/`Health` classes. Also caught a real
regression from this ticket's own reshape during a corrected re-sweep
(an earlier grep had a broken alternation and produced a false negative):
`src/tests/sim/unit/test_app_robot_loop_replace_harness.cpp` read
`benchLimits().trackWidth` directly — fixed to `.plant.trackWidth`.

**Verification**: `ctest --test-dir src/motion/planner/build` 8/8 passing.
`uv run python -m pytest src/tests/sim -q` — 458 passed, 2 failed (the
same two known-pre-existing failures named in the dispatch brief:
`test_clock_sync_activation.py`, `test_fake_transport.py`), 1 xfailed,
2 xpassed — zero new failures once the harness fix above landed. `just
build-clean` — ARM firmware links (RAM 98.33%/FLASH 41.46%, in the
project's normal near-full-RAM range). `circle.tour` — PASS, 9.6mm
(unchanged from the stated baseline). `square.tour` — FAILS its 80mm
gate at 124.7mm, squarely inside the pre-existing 124.7/126.4/133.5mm
noise band ticket 008 measured on unmodified code — not a regression,
and not this ticket's fix to make (ticket 010's job).
