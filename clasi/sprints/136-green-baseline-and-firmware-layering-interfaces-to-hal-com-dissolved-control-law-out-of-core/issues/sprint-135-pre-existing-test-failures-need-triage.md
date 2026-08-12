---
status: in-progress
priority: medium
sprint: '136'
tickets:
- 136-001
- 136-002
---

# Sprint 135 bare-`pytest` run surfaced failures beyond ticket 004's own scope — captured, not resolved

## Description

Sprint 135 ticket 004 (GO_TO wire arm, RobotLoop routing, NavigatorLimits
config — commit `c7455c4f`) is done and independently verified
(`planner_tests` 8/8, standalone navigator ctest 2/2). While closing the
ticket, a bare `uv run python -m pytest` (the widest possible run, not the
narrower suites the ticket's own testing plan named) surfaced additional
failures that ticket 004 did not introduce and is not the right place to
fix. This issue exists so that work is not lost or silently re-discovered
by the next sprint. **Nothing here has been root-caused to the point of a
confirmed fix; do not treat any "likely" statement below as verified.**

Three items are already handled and are NOT part of this issue (see
ticket 004's own Completion Notes for the full reasoning):

- `test_app_preamble.py::test_app_preamble_harness_compiles_and_passes` —
  pre-existing, reproduced identically (same failure text) against a
  `git stash`-clean baseline; already flagged pre-existing in ticket
  008's own Completion Notes too.
- `test_devices_otos.py::test_devices_otos_harness_compiles_and_passes` —
  pre-existing, confirmed via ticket 008's own stash comparison.
- `test_calibration_kwargs.py::test_calibration_commands_tovez_json_snapshot`
  — pre-existing stale snapshot; `tovez.json`'s current
  `linear_scale = 1.0188` is the correct, tape-measured value (see the
  `otos-is-accurate-tape-calibrated` project memory), the test's own
  hardcoded snapshot still expects the superseded `1.0275`. Ticket 004's
  diff to `tovez.json` is a pure 11-line addition (`git show c7455c4f --
  data/robots/tovez.json`); nothing else in that file was touched.

## The failures, by group

### Group 1 — `gen_boot_config` parity tests (7 tests, 3 files)

```
src/tests/unit/test_gen_boot_config_otos.py::test_otos_boot_config_values_reads_tovez_json
src/tests/unit/test_gen_boot_config_otos.py::test_generate_emits_default_otos_boot_config_additively
src/tests/unit/test_gen_boot_config_planner.py::test_planner_config_for_config_reads_tovez_json
src/tests/unit/test_gen_boot_config_planner.py::test_generate_emits_default_planner_limits_byte_identical_to_pre_ticket_literals
src/tests/unit/test_gen_boot_config_robot_groups.py::test_default_drive_group_matches_tovez_json
src/tests/unit/test_gen_boot_config_robot_groups.py::test_default_wheel_control_group_matches_tovez_json
src/tests/unit/test_gen_boot_config_robot_groups.py::test_default_otos_group_matches_tovez_json
```

Three of these seven — `test_planner_config_for_config_reads_tovez_json`,
`test_generate_emits_default_planner_limits_byte_identical_to_pre_ticket_literals`,
`test_default_drive_group_matches_tovez_json` — are **already tracked** at
`clasi/issues/later/B-gen-boot-config-parity-tests-encode-superseded-literals.md`:
stale hardcoded literals in change-detector tests that fail every time
`tovez.json` is legitimately re-measured. That issue's proposed fix
(assert against values read from the JSON at test time, or delete a
one-time refactor guard that has served its purpose) applies unchanged.

The other four — `test_otos_boot_config_values_reads_tovez_json` (stale
`1.0275` vs the current, correct `1.0188` — same root cause as the
`test_calibration_kwargs.py` snapshot above) and three whose assertions
fail because the expected substring (`cfg.linearScale = ...`,
`cfg.headingHoldGain = ...` restated in a second test, `cfg.v_min = ...`,
`cfg.linear_scale = ...`) is not found in `generate()`'s output at
all — are **not yet in that tracked issue** and should be folded into it
or triaged alongside it. A prior investigation this sprint flagged that
some of these may involve a deeper `gen_boot_config.py` generation-order
issue rather than a simple stale-literal problem — that suspicion is
UNCONFIRMED; someone needs to actually read `gen_boot_config.py`'s
`generate()` and check whether the OTOS/Planner/WheelControl groups it
emits still match what these tests expect it to emit, and whether the
answer is "update the test" or "the generator regressed."

**Do not conflate this with ticket 004's own scope.** Ticket 004 added a
`Navigator` config group and `defaultNavigatorGroup()` following the
`Planner` group's exact pattern; it did not touch the Planner/Drive/
WheelControl/Otos group generators themselves.

### Group 2 — turn-accuracy / tour-completion tests (possibly a recurrence of a known issue)

```
src/tests/testgui/test_tour_closure_gate.py (turn-accuracy miss, ~14 degrees observed this sprint)
src/tests/testgui/test_otos_calibration_convergence.py
src/tests/testgui/test_camera_combo.py (2 tests)
src/tests/testgui/test_gui_button_acceptance.py (3 tests, including a
    "tour_btn_tour_1 must be enabled before this test" PRECONDITION failure)
```

`clasi/issues/later/A-seven-untriaged-failing-tests-poison-every-no-regressions-claim.md`
already names `test_gui_button_acceptance.py::test_tour_1_runs_to_completion`,
`test_tour_2_runs_to_completion`, and
`test_tour_closure_gate.py::test_tour_1_and_tour_2_ninety_degree_turns_land_within_the_shaped_band`
as a recurring, previously-unresolved turn-undershoot family (that issue's
own working theory: `decelLatched`/shaping-band territory, sibling to
`A-tour2-146-degree-turn-still-undershoots-after-130-010`). It is
plausible — NOT confirmed — that what surfaced this sprint is the same
family resurfacing, possibly interacting with sprint 135 ticket 008's OTOS
sign fix (`57b01f32`, "sim OTOS packs the hardware-mounted heading sign").
It is equally plausible this is a distinct, new regression. Distinguishing
the two requires a clean-branch (pre-135) comparison run that has not been
done — this sprint's own investigation started that comparison and did not
finish it before time ran out.

The `tour_btn_tour_1 must be enabled before this test` failure in
`test_gui_button_acceptance.py` is a **precondition** failure, not a
turn-accuracy failure — worth a closer look on its own before assuming it
is the same family as the other two. It may be a simple test-ordering/
fixture-state issue (a button staying disabled because some earlier
state never got set) rather than anything to do with turn accuracy at
all.

## Cause

Not established. Two independent hypotheses are live and neither has been
tested to conclusion:

1. Group 1 (`gen_boot_config`): stale golden literals in change-detector
   tests that were never updated to track legitimate `tovez.json`
   re-measurement (confirmed mechanism for 3 of 7; unconfirmed for the
   other 4, which may be a real generator defect).
2. Group 2 (turn accuracy / tour completion): a known, previously-flagged,
   never-fully-triaged turn-undershoot family, possibly compounded by this
   sprint's OTOS sign-fix ticket. Needs a clean pre-135-branch comparison
   to attribute.

## Proposed fix

- Group 1: fold the 4 newly-identified failures into
  `B-gen-boot-config-parity-tests-encode-superseded-literals.md` (or this
  issue, if that one is closed by the time this is picked up) and resolve
  all 7 together — decide per test whether it defends a property (rewrite
  to read `tovez.json` at test time) or a one-time refactor guard that has
  outlived its purpose (delete).
- Group 2: run `test_tour_closure_gate.py` and the two
  `test_gui_button_acceptance.py` tour tests against a clean checkout of
  `master` before sprint 135 started, and against the tip of
  `sprint/135-go-to-navigator-in-the-motion-library`, to split "pre-existing"
  from "introduced by 135." Investigate the `tour_btn_tour_1` precondition
  failure separately and first — it is cheaper to diagnose and may be
  unrelated to turn accuracy entirely.

## Verification

- All 7 Group-1 tests pass without re-encoding a value that will break on
  the next legitimate re-measurement of `tovez.json`.
- Group 2's three-way attribution (pre-existing / 135 ticket 008 / new)
  is established and recorded, and the `tour_btn_tour_1` precondition
  failure has a named root cause.
- A subsequent bare `uv run python -m pytest` run's failure count only
  includes items tracked in an open issue.

## Related

- `clasi/issues/later/B-gen-boot-config-parity-tests-encode-superseded-literals.md`
- `clasi/issues/later/A-seven-untriaged-failing-tests-poison-every-no-regressions-claim.md`
- `.clasi/knowledge/otos-is-accurate-tape-calibrated.md` (project memory) —
  `linear_scale = 1.0188` is the current correct, tape-measured value.
- Sprint 135 ticket 004 — the ticket whose closure surfaced this list;
  see its Completion Notes for the full final-suite count and the two
  failures it separately confirmed pre-existing (`test_devices_otos.py`,
  `test_calibration_kwargs.py`).
- Sprint 135 ticket 008 (commit `57b01f32`) — OTOS heading-sign fix, the
  most likely interaction point for Group 2 if this turns out not to be
  purely pre-existing.
