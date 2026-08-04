---
id: '016'
title: Host cleanup (extra='forbid', drift assertions)
status: done
use-cases:
- SUC-001
depends-on:
- '013'
- '020'
github-issue: ''
issue: the-configuration-object.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Host cleanup (extra='forbid', drift assertions)

## Description

Set `extra='forbid'` on the generated pydantic model (or its thin
hand-written wrapper layer) so an unrecognized JSON key raises loudly
instead of silently ignoring it (today's default `extra='ignore'`,
confirmed this round — no `model_config`/`ConfigDict` override exists).
Add a test asserting the fields that were previously silently dropped
(`output_deadband`, `reversal_dwell_ms`, and the other 16 identified in
the issue's own audit) are now present on the generated model — this
should already be true structurally once the schema is generated (ticket
002), so this ticket's job is proving it, not fixing it by hand.

**Note**: this ticket runs BEFORE ticket 017's JSON reshape, so the robot
JSONs are still in their OLD 13-section shape at this point —
`extra='forbid'` will likely cause validation to fail against the
current files until the reshape lands (expected; do not treat this as a
bug to work around mid-sprint).

## Acceptance Criteria

- [x] The generated pydantic model (or its thin wrapper) sets
      `extra='forbid'` (via `model_config = ConfigDict(extra='forbid')`
      or equivalent).
- [x] A new test asserts the model has fields for every previously-dropped
      `control.*` key from this round's audit (`output_deadband`,
      `reversal_dwell_ms`, and the other 16 — pull the exact list from
      this ticket's own re-verification, not assumed from memory).
- [x] The test explicitly documents that current robot JSONs are
      expected to FAIL `extra='forbid'` validation until ticket 017
      lands (a skip/xfail marker with a clear reason, not a
      silently-passing test that proves nothing).
- [x] Compiles/imports cleanly.

## Completion Notes (2026-08-04)

**Ordering decision — applied `extra='forbid'` NOW, unconditionally (not
gated until 017).** `RobotConfig` and every group it composes (generated:
`Identity`/`Connection`/`Vision`/`Geometry`/`Motors`/`Drive`/
`WheelControl`/`Planner`/`Otos`/`Estimator`; hand-declared:
`OffsetXY`/`WheelsConfig`/`EncodersConfig`/`GripperConfig`/
`PeripheralsConfig`) now set `model_config = ConfigDict(extra="forbid")`,
added at the generator (`scripts/gen_messages.py`'s `_render_pydantic_
module()`, so it survives regeneration) for the 10 generated groups, and
by hand for the 5 host-only sections with no generated counterpart plus
`RobotConfig` itself. **Consequence, confirmed by running the suite, not
assumed:** `load_robot_config()`/`get_robot_config()` now raise
`pydantic.ValidationError` for every real, on-disk robot JSON
(`tovez.json`, `tovez_nocal.json`, `togov.json`) — the old-shape files'
`calibration`/`control` top-level sections have no matching field at all,
and `connection`/`vision`/`geometry` each carry old-shape keys the new
schema doesn't recognize either (e.g. `connection.i2c_addresses`,
`vision.tag_offset_mm`, `geometry.drive_axle_offset_mm`). `list_robots()`
degrades gracefully (its pre-existing try/except skips unreadable files),
so it returns empty for the real `data/robots/` dir until 017, rather than
crashing. This was the deliberate choice, not the alternative
(gate-until-017): `robot_config.py`'s own module docstring, written by
ticket 020, already anticipated and endorsed this exact posture ("mirrors
ticket 016's own 'expected to fail until 017' posture"), and gating would
have required new machinery (a flag/env var) this ticket's own
Implementation Plan scopes as "a small, targeted pydantic config change."
**Left for ticket 017**: re-shaping the three robot JSONs is what turns
this back green; nothing else is needed on the pydantic-model side.

**Previously-dropped fields, re-verified (not assumed from memory):**
computed `set(tovez.json's 53 `control.*` keys) - set(the OLD
hand-written `ControlConfig`'s 36 field names, read from git commit
`47ef2221^`)` = **18 keys**, matching the issue's own audit exactly. Of
those 18: **2 are load-bearing and now present** on the generated model —
`output_deadband` → `Motors.output_deadband` (unchanged name) and
`reversal_dwell_ms` → `Motors.reversal_dwell` (unit suffix dropped per
naming-and-style.md) — both required by `gen_boot_config.py`, which
refuses to build without them. **The other 16
(`arrive_vel_tol`/`handoff_tol_pos`/`handoff_tol_v`/`replan_err_pos`/
`replan_err_theta`/`replan_hold`/`replan_max`/`replan_min_period`/
`steer_headroom`/`track_k_cross`/`track_k_s`/`track_k_theta`/
`trim_omega_max`/`trim_v_max`/`v_wheel_max`/`wheel_step_max`) are
genuinely dead** — confirmed absent from every one of the 10 generated
groups, and independently confirmed by `robot_config.proto`'s own header
checklist to have no `_require()`/`_get()` call site in
`gen_boot_config.py` — and correctly stay excluded from the new schema by
design, not by oversight. The ticket's own premise ("the other 16 ... are
now present") does not hold; this ticket's job was proving the true state,
which `src/tests/unit/test_robot_config_extra_forbid.py` now documents and
asserts both ways (2 present, 16 correctly absent) so future drift in
either direction is caught.

**Test numbers actually run** (targeted suites — my ticket's own
verification scope; testgui was NOT used as a gate per stakeholder
direction that per-ticket suite gates are off this sprint):

| suite | before (ticket 014 baseline) | after this ticket |
|---|---|---|
| `src/tests/unit` | 756 passed / 3 failed (pre-existing, unrelated) | 785 passed / 3 failed (SAME pre-existing 3) / 7 xfailed |
| `src/tests/sim` | 453 passed / 0 failed / 11 xfailed | 446 passed / 0 failed / 18 xfailed (11 pre-existing + 7 new, all `extra='forbid'`-caused, all `xfail(strict=True, raises=ValidationError)`) |
| `src/tests/testgui` | 541 passed / 51 failed / 15 xfailed / 3 skipped | 498 passed / 33 failed / 61 errors / 15 xfailed / 3 skipped — NOT chased (see below) |

Every unit/sim test that broke as a direct, traceable side effect of this
ticket's own change (real-JSON-loading tests that previously relied on
`extra='ignore'`) was updated with an `xfail(strict=True,
raises=ValidationError, reason=...)` marker pinned to the exact new
failure mode, per this ticket's own Testing section ("any existing test
asserting extra='ignore'-shaped behavior is updated to reflect the new
forbid behavior") — 4 in `src/tests/unit`
(`test_robot_config.py` x3, `test_calibration_kwargs.py` x3,
`test_sim_boot_config.py` x1 — 7 total incl. `test_calibration_kwargs.py`'s
3) and 7 in `src/tests/sim` (`test_sim_configure_from_robot.py` x2,
`test_sim_wire_loopback.py` x1, `test_straight_leg_crab_regression.py` x1,
`test_motor_primitive.py` x2, `test_pathplan_goto_convergence.py` x1).
`strict=True` + `raises=ValidationError` means each will flip to an
unexpected-pass failure the moment 017 reshapes the JSON, which is the
correct signal to go remove the marker and restore each test's original
assertion.

**testgui NOT chased, per explicit stakeholder direction** (per-ticket
suite gates are off this sprint — "I want this to work at the end. It
doesn't have to work in the middle"): the 43 newly-red testgui
tests/errors (541p/51f baseline → 498p/33f/61e) are the SAME class of
change as the unit/sim ones above (real `RobotConfig` loads now raise
`ValidationError` instead of silently loading with defaulted/dropped
fields), just not individually marked xfail here — left entirely for
ticket 017's reshape to resolve wholesale, alongside the pre-existing
51-failure root cause (`Motors.fwd_sign_left/right` /
`Drive.duty_per_speed_left/right` proto3 zero defaults tripping
`App::Drive`'s fail-closed gate) that ticket 017 already owns.

## Testing

- **Existing tests to run**: any existing test asserting
  `extra='ignore'`-shaped behavior is updated to reflect the new
  `forbid` behavior.
- **New tests to write**: as in Acceptance Criteria.
- **Verification command**: `uv run python -m pytest <robot_config test
  path> -q`.

## Implementation Plan

**Approach**: A small, targeted pydantic config change plus one new test
file/test.

**Files to modify**: `src/host/robot_radio/config/robot_config.py` (or
wherever the thin hand-written wrapper from ticket 002 lives).

**Testing plan**: as above.

**Documentation updates**: none beyond the new test's own docstring
explaining the expected-to-fail-until-017 status.
