---
id: '001'
title: Fix the config-generation test cluster (stall-key gap, stale literals, verb/field-number
  lag)
status: done
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: sprint-135-pre-existing-test-failures-need-triage.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Fix the config-generation test cluster (stall-key gap, stale literals, verb/field-number lag)

## Description

This is the first ticket of Half A (the trustworthy-baseline gate that must
complete before any Half B layering ticket starts — see sprint.md's hard
gate). It fixes the two *root-caused* failure clusters identified during
sprint planning by direct investigation (not estimation):

**Cluster 1 — missing config keys.** `data/robots/tovez_nocal.json` and
`data/robots/togov.json` are both missing `wheel_control.stall_speed` /
`stall_demand` / `stall_window`, which `gen_boot_config.py` has required,
fail-closed, since the 2026-08-08 stall-detection directive (the same gap
`.claude/rules/hardware-bench-testing.md` already documents for `gopiv.json`,
fixed there 2026-08-09 by adding the three keys at `0.0`, the documented
inert state). This single gap cascades: in
`src/tests/sim/unit/test_gen_boot_config_required_keys.py`, the shared
`_complete_cfg()` fixture (built from `tovez_nocal.json`) fails on the
missing key *before* any parametrized case reaches its own intended
assertion, producing 37 failures from one root cause (confirmed by direct
`pytest` run during planning: `37 failed, 34 passed`, every failure citing
`wheel_control.stall_speed missing`). The same gap produces 5 more failures
in `src/tests/unit/test_pos_err_max_config_surface.py` (2 `[togov.json]`/
`[tovez_nocal.json]` parametrizations of `test_bake_path_emits_the_files_
own_pos_err_max`, 2 of `test_read_back_equals_file_for_the_whole_wheel_
control_group`, plus `test_no_wheel_control_field_was_renumbered_or_reused`,
which is *also* independently stale — see Cluster 3).

**Cluster 2 — stale hardcoded literals.** 9 tests across
`test_gen_boot_config_otos.py` (2), `test_gen_boot_config_planner.py` (2),
`test_gen_boot_config_robot_groups.py` (4), and
`test_calibration_kwargs.py` (1) assert generated boot config against
literal values (`cfg.linear_scale = 1.0275f`, `cfg.v_min = 99.7f`, ...)
that `tovez.json`'s legitimate re-measurement history has since superseded
(current values: `1.0188`, `20.0`, ...). This is exactly the pattern
`clasi/issues/later/B-gen-boot-config-parity-tests-encode-superseded-
literals.md` already names for 3 of these 9 — this ticket resolves all 9
together and closes that issue.

**Cluster 3 — independent test/doc lag (not config-related).**
`test_command_registry.py::test_verb_inventory_matches_the_issue_spec`
fails because the `CALIBRATE` verb (added in commit `3a05bab9`, "feat:
CALIBRATE — recalibrate the OTOS gyro on demand") was never back-filled
into the test's expected verb set or into `docs/protocol-v5.md`'s §4 verb
table. `test_pos_err_max_config_surface.py::test_no_wheel_control_field_
was_renumbered_or_reused` fails because its hardcoded expected
field-number dict doesn't know about wire field numbers 13-15
(`stall_speed`/`stall_demand`/`stall_window`, the same feature that added
Cluster 1's keys) — this is a legitimate addition the test needs to
acknowledge, not a renumbering defect (the existing 12 field numbers are
unchanged).

Root-cause all three clusters at the source rather than patching symptoms:
fix the two JSON files once, and make each affected test assert a
*property* (read the value from JSON / the current verb table / the current
field-number set at test time) rather than a pinned snapshot, per
`B-gen-boot-config-parity-tests-encode-superseded-literals.md`'s own
proposed fix. Where a test's only job was ever a one-time refactor guard
whose historical moment has passed (the clearest candidate:
`test_generate_emits_default_planner_limits_byte_identical_to_pre_ticket_
literals` — its own name says what it is), delete it rather than
re-pinning it to a new snapshot that will just go stale again.

**Do not touch anything under Half B's scope** (no file moves, no
namespace renames) — this ticket is config-JSON and Python-test changes
only.

## Acceptance Criteria

- [ ] `data/robots/tovez_nocal.json` gains `wheel_control.stall_speed` /
      `stall_demand` / `stall_window` = `0.0`, with a `_stall_note`
      matching `gopiv.json`'s 2026-08-09 precedent (documented inert
      state, not a fabricated calibration).
- [ ] `data/robots/togov.json` gains the same three keys at `0.0`, in the
      same documented-inert posture as its existing `wheel_control` keys
      (see its own `_wheel_controller_note`) — do not borrow `tovez`'s
      numbers.
- [ ] `src/tests/sim/unit/test_gen_boot_config_required_keys.py` passes in
      full (0 failed) after the JSON fix.
- [ ] `src/tests/unit/test_pos_err_max_config_surface.py` passes in full
      (0 failed).
- [ ] All 9 stale-literal tests (`test_gen_boot_config_otos.py` x2,
      `test_gen_boot_config_planner.py` x2,
      `test_gen_boot_config_robot_groups.py` x4,
      `test_calibration_kwargs.py::test_calibration_commands_tovez_json_
      snapshot`) are each either rewritten to assert a value read from the
      JSON/model at test time, or deleted as a spent one-time refactor
      guard (document which, per test, in Completion Notes).
      `test_calibration_commands_tovez_nocal_json_snapshot` must keep
      passing unmodified in intent (confirm the stall-key JSON addition
      doesn't change `calibration_commands()`'s output — `stall_*` is not
      part of the OI/OL/OA calibration-kwargs surface).
- [ ] `test_command_registry.py::test_verb_inventory_matches_the_issue_
      spec` updated to include `CALIBRATE` in its expected binary verb
      set; `docs/protocol-v5.md`'s §4 table updated to list `CALIBRATE`.
- [ ] `test_pos_err_max_config_surface.py::test_no_wheel_control_field_
      was_renumbered_or_reused` updated to include `stall_speed: 13`,
      `stall_demand: 14`, `stall_window: 15` in its expected dict,
      preserving the existing 12 entries unchanged.
- [ ] `clasi/issues/later/B-gen-boot-config-parity-tests-encode-
      superseded-literals.md` is resolved per its own Verification
      section and moved to done.
- [ ] Every edited test file's own doc-comment/docstring provenance trail
      (this suite's established convention) gets a dated note explaining
      the change — no silent edits.

## Testing

- **Existing tests to run**: `uv run python -m pytest
  src/tests/sim/unit/test_gen_boot_config_required_keys.py
  src/tests/unit/test_gen_boot_config_otos.py
  src/tests/unit/test_gen_boot_config_planner.py
  src/tests/unit/test_gen_boot_config_robot_groups.py
  src/tests/unit/test_calibration_kwargs.py
  src/tests/unit/test_command_registry.py
  src/tests/unit/test_pos_err_max_config_surface.py -q`
- **New tests to write**: none required structurally — this ticket
  rewrites existing tests' assertions to defend properties instead of
  snapshots. If a test is deleted, record which and why in Completion
  Notes (traceable, not silent).
- **Verification command**: the file list above. Ticket 002 runs the full
  three-suite sweep next — this ticket's job is the two root-caused
  clusters plus the two independent singletons, not the whole suite.
