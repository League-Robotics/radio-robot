---
id: '001'
title: 'Wire/config schema: motion, planner, envelope, config protos + tovez.json
  generators'
status: done
use-cases:
- SUC-011
depends-on: []
github-issue: ''
issue: motion-stack-v2-a-self-contained-stateless-motion-control-subsystem.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Wire/config schema: motion, planner, envelope, config protos + tovez.json generators

## Description

Foundation ticket for the whole sprint. Declares the wire and boot-config
contract for arc/pivot segments (`MotionSegment`'s new fields), the
`Drive::Limits` source (`PlannerConfig` fields 15-31), and plan
interpretability (`PlanDumpRequest`/`PlanRecord`/`MotionTrace`), plus
`EventNotify`'s real body — before any C++ consumer of any of it exists.
Follows the "declare the arm/field now, implement it live in a later
ticket" pattern this project already used across sprints 095/096/098
(`envelope.proto`'s own header comments document this precedent
extensively — read them before touching `CommandEnvelope`/
`ReplyEnvelope`).

Full field-number and design context: `clasi/issues/
motion-stack-v2-a-self-contained-stateless-motion-control-subsystem.md`
("Wire schema & config" section) and this sprint's `architecture-update.md`
(M1, Decision 2, Decision 4). Read both before starting — Decision 2 in
particular explains why `PlannerConfig.v_wheel_max`/`steer_headroom`
(new fields 15/16) are DELIBERATELY separate from `DrivetrainConfig`'s
existing `v_wheel_max`/`steer_headroom` (fields 10/11) — do not "fix" this
apparent duplication by consolidating them.

## Acceptance Criteria

- [x] `protos/motion.proto`: `MotionSegment` gains `arc_length`(14, `[mm]`
      signed), `delta_heading`(15, `[rad]`), `exit_speed`(16, `[mm/s]`),
      `primitive`(17, `bool`); firmware will reject `primitive=false`
      after cutover (ticket 007) — not enforced yet by this ticket, just
      declared.
- [x] `protos/planner.proto`: `PlannerConfig` gains fields 15-31 exactly
      per the issue's list: `v_wheel_max`, `steer_headroom`,
      `wheel_step_max`, `track_k_s`, `track_k_theta`, `track_k_cross`,
      `trim_v_max`, `trim_omega_max`, `replan_err_pos`, `replan_err_theta`,
      `replan_hold`, `replan_min_period`, `replan_max`,
      `handoff_tol_pos`, `handoff_tol_v`, `arrive_vel_tol`,
      `arrive_dwell` — 17 fields, numbers 15 through 31 inclusive. Each
      field's doc comment carries a `// [unit]` tag (no units in the
      field name itself, per `.claude/rules/naming-and-style.md`).
- [x] `protos/config.proto`: `PlannerConfigPatch` grows to cover the new
      live-tunable subset of fields 15-31 (`optional float`, matching the
      existing 3-field pattern). Run `scripts/gen_messages.py` and
      capture the `kMaxEncodedSize` report for `ConfigDelta`/
      `ConfigSnapshot`/`ReplyEnvelope`/`CommandEnvelope` BEFORE and AFTER
      the growth (paste both into completion notes). If any exceeds
      186B, split `PlannerConfigPatch` into a second `Patch` message
      selected by a new `CONFIG_PLANNER_TRACK` `ConfigTarget` value
      (architecture-update.md Decision 4's specified fallback — do not
      invent a different split mechanism).
- [x] `protos/envelope.proto`: new `PlanDumpRequest` (`CommandEnvelope.cmd`
      arm 18), `PlanRecord` (`ReplyEnvelope.body` arm 10, ~85B:
      goal/anchor/v_eff/duration/exit_speed/entry_speed/replan_count),
      `MotionTrace` (`ReplyEnvelope.body` arm 11, ~90-120B, a serialized
      `TrackRecord`) declared. `BinaryChannel` replies `ERR_UNIMPLEMENTED`
      for both until ticket 009 makes them live (matching the config/
      get/stream and pose/otos precedent already in this file's header
      comments). `EventNotify` (existing empty placeholder, arm 6) gets a
      real body: `seg_seq`, `status`, `e_final_pos`, `e_final_theta`.
- [x] `protos/telemetry.proto` is NOT touched by this ticket — verify by
      diff that `message Telemetry` gains no new field (`MotionTrace` is
      a separate reply arm, never a `Telemetry` extension; the existing
      ~166B budget stays untouched).
- [x] `data/robots/tovez.json` gains new `control.*`/`geometry.*` keys for
      every new `PlannerConfig` tunable, seeded with the issue's starting
      values (`track_k_s`≈2.0, `track_k_theta`≈6.0, `track_k_cross`≈1.5e-5,
      `trim_v_max`≈120, `trim_omega_max`≈1.0/2.0, plus the replan/handoff/
      arrive envelope defaults from the issue's tables), each annotated
      "starting values, not yet bench-tuned" (mirroring the existing
      `_heading_gains_note` convention exactly).
- [x] `scripts/gen_boot_config.py` gains a mapping for the new
      `PlannerConfig` fields, mirroring `heading_gains_for_config()`'s
      exact shape (read from `data/robots/*.json`'s `control`/`geometry`
      block, falling back to a firmware default when absent).
- [x] `scripts/check_config_sync.py`'s `PATCH_TO_PYDANTIC` map gains an
      entry for every new `PlannerConfigPatch` field (allowlisted `[]`
      where no host pydantic field exists yet, matching the
      `ekf_r_otos_*`/`heading_kp` precedent exactly — never a silent
      omission). `python scripts/check_config_sync.py` exits 0.
- [x] `uv run python -m pytest` passes (existing
      `test_gen_boot_config_planner.py`-style tests extended for the new
      fields).

## Testing

- **Existing tests to run**: `uv run python -m pytest` (full sim suite);
  `python scripts/check_config_sync.py`.
- **New tests to write**: extend `tests/sim/unit/
  test_gen_boot_config_planner.py`-style coverage for the new
  `PlannerConfig` fields' JSON-to-boot-config mapping (present-in-JSON
  and absent-from-JSON/fallback-default cases).
- **Verification command**: `uv run pytest`

## Implementation Plan

**Approach**: Follow the exact pattern sprint 098 used for
`heading_kp`/`heading_kd` (`planner.proto` field growth ->
`gen_boot_config.py` -> `tovez.json` -> `check_config_sync.py`), scaled
to 17 fields plus three new envelope/reply arms and one placeholder
resolution (`EventNotify`). Read `protos/envelope.proto`'s header comment
block on the declare-then-implement pattern before touching
`CommandEnvelope`/`ReplyEnvelope` — do not implement any live
`BinaryChannel` behavior for the new arms in this ticket (that is ticket
009's job); this ticket is schema-only.

**Files to create/modify**:
- `protos/motion.proto`, `protos/planner.proto`, `protos/config.proto`,
  `protos/envelope.proto`
- `source/messages/*.h` (regenerated via `scripts/gen_messages.py` — do
  not hand-edit, per `.claude/rules/coding-standards.md`)
- `data/robots/tovez.json`
- `scripts/gen_boot_config.py`
- `scripts/check_config_sync.py`, `scripts/config_sync_allowlist.json`
  (if new allowlist entries are needed)
- `tests/sim/unit/test_gen_boot_config_planner.py` (extend)

**Testing plan**: run `scripts/gen_messages.py` and record the
`kMaxEncodedSize` report for every `ReplyEnvelope`/`CommandEnvelope` arm
in completion notes; run `scripts/check_config_sync.py`; run
`uv run python -m pytest`.

**Documentation updates**: note in completion notes that
`docs/protocol-v3.md` (or whichever protocol doc is current) goes stale
the moment this schema lands and needs a follow-up edit
(architecture-update.md Open Question 4) — do NOT edit the doc in this
ticket (out of scope, flagged for the team-lead to schedule).
