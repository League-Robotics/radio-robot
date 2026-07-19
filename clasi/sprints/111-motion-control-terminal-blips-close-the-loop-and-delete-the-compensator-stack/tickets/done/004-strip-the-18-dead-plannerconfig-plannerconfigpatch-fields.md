---
id: '004'
title: Strip the 18 dead PlannerConfig/PlannerConfigPatch fields
status: in-progress
use-cases:
- SUC-003
depends-on:
- '001'
- '002'
github-issue: ''
issue: motion-control-terminal-blips-reconciled-fix-plan.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Strip the 18 dead PlannerConfig/PlannerConfigPatch fields

## Description

Step 7 of the driving issue. Remove the 18 `PlannerConfig` fields with no
live firmware consumer, through `protos/` + the code generator — never a
hand-edit to generated code (`messages/DESIGN.md`'s own invariant:
"Generated files are never hand-edited... Wire schema changes go through
protos/ + the generator").

Deadness independently re-verified during sprint planning (grep across
`src/firm/` found zero consumers outside the generated struct declaration
and the `Pilot::applyPlannerPatch()` merge arms; grep across
`src/scripts`/`src/host`/`data` found zero host-side callers setting any
of the 18 field names):

- **2 fields on `PlannerConfig` only** (never in `PlannerConfigPatch`):
  `arrive_tol` (field 8), `turn_in_place_gate` (field 9).
  `src/firm/config/boot_config.h`'s own comment already documents these
  as "stay unset (0.0f default) — unused by any current consumer."
- **16 fields on BOTH `PlannerConfig` and `PlannerConfigPatch`**:
  `v_wheel_max` (15/4), `steer_headroom` (16/5), `wheel_step_max` (17/6),
  `track_k_s` (18/7), `track_k_theta` (19/8), `track_k_cross` (20/9),
  `trim_v_max` (21/10), `trim_omega_max` (22/11), `replan_err_pos`
  (23/12), `replan_err_theta` (24/13), `replan_hold` (25/14),
  `replan_min_period` (26/15), `replan_max` (27/16), `handoff_tol_pos`
  (28/17), `handoff_tol_v` (29/18), `arrive_vel_tol` (30/19) — numbers
  given as `PlannerConfig field / PlannerConfigPatch field`.

**Do NOT remove** (confirmed live): `a_max`, `a_decel`, `v_body_max`,
`yaw_rate_max`, `yaw_acc_max`, `j_max`, `yaw_jerk_max`, `min_speed`,
`heading_kp`, `heading_kd`, `arrive_dwell`, `heading_source`,
`heading_dwell_tol`, `heading_dwell_rate`, `heading_lead_bias`,
`plan_lead`, `terminal_lead`. `arrive_dwell` in particular LOOKS similar
to the dead `arrive_vel_tol`/`arrive_tol` but is genuinely live (reused
by the dwell-completion gate, `Motion::Executor`) — do not confuse the
two.

`completes_issue: false` — the driving issue is an explicit multi-sprint
arc umbrella (its own header: "The work is a multi-sprint arc and should
be roadmap-planned up front"). Sprint 111 implements only steps 0/1/7 of
the issue's 10-step plan; steps 2-6/8-9 remain for later arc sprints. All
four tickets in this sprint (001-004) carry `completes_issue: false` so
the issue is NOT archived when this sprint's tickets reach `done` — it
stays `in-progress` until whichever later arc sprint's own final ticket
sets `completes_issue: true`.

## Acceptance Criteria

- [x] `src/protos/planner.proto`: the 2 `PlannerConfig`-only fields
      (`arrive_tol`=8, `turn_in_place_gate`=9) and the 16 shared fields
      (15-30) are removed; their field numbers are added to a `reserved`
      statement on the `PlannerConfig` message (alongside the existing
      `reserved 10, 11;`).
- [x] `src/protos/config.proto`: the 16 corresponding `PlannerConfigPatch`
      fields (numbers 4-19) are removed; their numbers are `reserved` on
      that message. `PlannerConfigPatch`'s remaining fields
      (`min_speed`=1, `heading_kp`=2, `heading_kd`=3, `arrive_dwell`=20)
      are untouched.
- [x] `scripts/gen_messages.py` is re-run (via `python build.py` or the
      project's own codegen step) to regenerate `src/firm/messages/
      planner.h`, `src/firm/messages/config.h`, `src/firm/messages/
      wire.{h,cpp}`, `src/firm/messages/layout_checks.{h,cpp}`. No
      generated file is hand-edited.
- [x] `src/firm/app/pilot.h`'s `Pilot::applyPlannerPatch()` — the 16
      merge lines corresponding to the removed `PlannerConfigPatch`
      fields are deleted. The 4 remaining merge lines (`min_speed`,
      `heading_kp`, `heading_kd`, `arrive_dwell`) are unchanged. The
      method's own doc comment (which currently lists all 20 patch
      fields it curates) is updated to list only the 4 that remain.
- [x] `src/firm/config/boot_config.h`/`.cpp`'s comments referencing
      `arrive_tol`/`turn_in_place_gate` as "left unset, no consumer" are
      updated (those fields no longer exist to leave unset) — or removed
      if the sentence no longer has anything to say.
- [x] `src/firm/config/DESIGN.md`'s Open Questions entry ("Several
      PlannerConfig fields (arrive_tol, turn_in_place_gate) are left
      permanently unset... Revisit if a consumer ever needs them") is
      removed or updated to reflect that the fields no longer exist.
- [x] The full `-DHOST_BUILD` sim build compiles with the regenerated
      headers (confirmed by running the sim test suite, which compiles
      fresh harnesses against `messages/`).
- [x] A `PlannerConfigPatch` round-trip test (existing or newly
      exercised via the sim suite) confirms `min_speed`/`heading_kp`/
      `heading_kd`/`arrive_dwell` still merge correctly after the field
      removal.
- [x] `uv run pytest` is green.

## Implementation Plan

**Approach**: schema-first removal through the generator, never a direct
edit to generated `.h`/`.cpp` files.

1. Edit `src/protos/planner.proto`: delete the 18 field declarations
   (and their doc comments where they exist solely to describe a
   now-removed field), add their numbers to `PlannerConfig`'s `reserved`
   statement.
2. Edit `src/protos/config.proto`: delete the 16 `PlannerConfigPatch`
   field declarations, add their numbers to that message's `reserved`
   statement.
3. Regenerate: run the project's codegen step (`python build.py`, which
   invokes `scripts/gen_messages.py` — confirm the exact invocation from
   `justfile`/`build.py` rather than calling the generator script
   directly with guessed arguments).
4. Edit `src/firm/app/pilot.h`: delete the 16 `if (patch.<field>.has)
   merged.<field> = patch.<field>.val;` lines in `applyPlannerPatch()`
   for the removed fields; update the method's doc comment.
5. Edit `src/firm/config/boot_config.h`/`.cpp` and `src/firm/config/
   DESIGN.md`: update/remove the stale "arrive_tol/turn_in_place_gate
   unused" comments.
6. Full-text grep `src/firm/` and `src/tests/` once more after the
   regeneration for any of the 18 field names to confirm zero remaining
   references (catches a test that might reference one of these fields
   in a config round-trip test, which planning-time grep may not have
   covered exhaustively under `src/tests/`).

**Files to modify**:
- `src/protos/planner.proto`
- `src/protos/config.proto`
- Generated (via codegen, not hand-edited): `src/firm/messages/
  planner.h`, `src/firm/messages/config.h`, `src/firm/messages/
  wire.{h,cpp}`, `src/firm/messages/layout_checks.{h,cpp}`
- `src/firm/app/pilot.h`
- `src/firm/config/boot_config.h`, `src/firm/config/boot_config.cpp`
- `src/firm/config/DESIGN.md`

**Testing plan**: full sim suite compile-and-run (the regenerated
`messages/` headers are exercised by every sim harness that includes
them); a targeted check that a `PlannerConfigPatch` wire message with
only the 4 live fields still round-trips through `handleConfig()`'s
PLANNER arm correctly (existing coverage, if any, or a small addition to
an existing config-patch test).

**Documentation updates**: `src/firm/app/pilot.h`'s `applyPlannerPatch()`
doc comment; `src/firm/config/boot_config.h`'s `defaultPlannerConfig()`
doc comment; `src/firm/config/DESIGN.md`'s Open Questions section.

## Testing

- **Existing tests to run**: full `uv run pytest` suite (the schema
  change is exercised transitively by every sim harness that includes
  `messages/planner.h`/`config.h`).
- **New tests to write**: none required, unless the investigation in
  step 6 above finds a test that needs updating to drop a reference to
  a removed field.
- **Verification command**: `uv run pytest`.

## Completion Notes

**Removed (18 fields, all reserved, none renumbered):**
- `PlannerConfig` (`planner.proto`): `arrive_tol`=8, `turn_in_place_gate`=9,
  `v_wheel_max`=15, `steer_headroom`=16, `wheel_step_max`=17,
  `track_k_s`=18, `track_k_theta`=19, `track_k_cross`=20, `trim_v_max`=21,
  `trim_omega_max`=22, `replan_err_pos`=23, `replan_err_theta`=24,
  `replan_hold`=25, `replan_min_period`=26, `replan_max`=27,
  `handoff_tol_pos`=28, `handoff_tol_v`=29, `arrive_vel_tol`=30. New
  `reserved` statement: `reserved 8, 9, 10, 11, 15, 16, 17, 18, 19, 20, 21,
  22, 23, 24, 25, 26, 27, 28, 29, 30;` (extends the pre-existing
  `reserved 10, 11;`). `arrive_dwell`=31 and fields 32-37
  (`heading_source`/`heading_dwell_tol`/`heading_dwell_rate`/
  `heading_lead_bias`/`plan_lead`/`terminal_lead`) are untouched.
- `PlannerConfigPatch` (`config.proto`): the 16 shared fields, numbers
  4-19. New `reserved 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18,
  19;`. Remaining fields unchanged: `min_speed`=1, `heading_kp`=2,
  `heading_kd`=3, `arrive_dwell`=20.

**Regenerated (never hand-edited):** `src/firm/messages/{common,
communicator,config,drivetrain,envelope,gripper,motion,motor,odometer,
planner,ports,sensors,telemetry}.h`, `wire.{h,cpp}`,
`layout_checks.{h,cpp}` via `uv run python src/scripts/gen_messages.py`;
`src/host/robot_radio/robot/pb2/*.py` via `gen_pb2.py`;
`src/firm/config/boot_config.cpp` via `gen_boot_config.py`. Used the
minimal per-script invocations (not full `python build.py`) per the
ticket's own guidance, to avoid unrelated `build/` churn.

**Beyond the ticket's listed files** (required for the schema change to
actually compile/pass, discovered during implementation — all through
generators or direct source edits, never a generated-file hand-edit):
- `scripts/gen_boot_config.py`: `defaultPlannerConfig()` used to emit
  `cfg.setVWheelMax(...)` etc. for all 16 dead fields (baked at boot from
  `data/robots/tovez.json`'s `control.*` keys) — those setters no longer
  exist on the regenerated `msg::PlannerConfig`, so the generator would
  fail to produce compilable output. Removed the 16 `*_DEFAULT` constants
  and `drive_limits_for_config()`; replaced with a small
  `arrive_dwell_for_config()` (mirrors `heading_dwell_for_config()`'s
  shape) so `arrive_dwell` — the one live survivor — still bakes from
  `control.arrive_dwell` correctly.
- `src/tests/sim/unit/test_gen_boot_config_planner.py` (pytest-collected,
  `src/tests/sim/`): directly tested `drive_limits_for_config()` and the
  16 dead setters (`_DRIVE_LIMIT_FIELDS`). Replaced with equivalent
  coverage for `arrive_dwell_for_config()` alone (reads-JSON,
  falls-back-to-default, reads-arbitrary-value, and `generate()`-emission
  tests, with/without a robot config).
- `src/tests/sim/unit/test_wire_differential.py` (pytest-collected):
  `expected_planner_patch` pinned all 20 old field numbers against the
  live `pb2` descriptor — trimmed to the 4 remaining. Also **added**
  `arrive_dwell` to `test_direction_a_config_planner`'s round-trip
  (previously only `min_speed`/`heading_kp`/`heading_kd`) and to
  `wire_differential_harness.cpp`'s PLANNER decode-print arm, so all 4
  live `PlannerConfigPatch` fields are now genuinely exercised
  host-encode → firmware-decode through the real regenerated
  `wire.cpp`/`config.h` — this is acceptance criterion 8's
  "small addition to an existing config-patch test."
- `src/scripts/check_config_sync.py`'s `PATCH_TO_PYDANTIC` map and
  `src/scripts/config_sync_allowlist.json`: removed the 16 now-stale
  `PlannerConfigPatch.<dead-field>` entries (would not have failed
  `test_check_config_sync.py` — the lint only iterates the live `pb2`
  descriptor — but left as pure dead-key drift otherwise).
- `src/scripts/gen_messages.py`'s `_INVENTORY_MAP` (feeds the optional
  `--emit-inventory` traceability doc, not part of the pytest gate):
  removed the 2 stale `PlannerConfig.arrive_tol`/`turn_in_place_gate`
  entries.

**Out of scope, pre-existing, left untouched:** `src/tests/bench/
rig_drive.py` and `src/tests/notebooks/{drive_plan_table,
plan_dump_trace_overlay}.ipynb` reference field names like `v_wheel_max`/
`trim_v_max` — these are a *different*, host-only Python `Limits`
dataclass from a `src/tests/_infra/drive/drive.py` module that does not
exist in this tree (confirmed: `import drive` fails with
`ModuleNotFoundError`), unrelated to the wire `msg::PlannerConfig`/
`PlannerConfigPatch` this ticket touches. Pre-existing breakage, not
pytest-collected (`bench/`/`notebooks/` are HITL tools, not in
`testpaths`), not caused by or fixable within this ticket's scope.
`data/robots/tovez.json`'s `control.*` keys for the 16 dead fields (and
its `_drive_limits_note`) are likewise left as inert, unread data —
out of the ticket's grep scope (`src/firm/`, `src/tests/`) and a
calibration artifact, not source. `src/tests/sim/parked-094/unit/
planner_harness.cpp` references `arrive_tol`/`turn_in_place_gate`
directly but lives in a `norecursedirs`-excluded parked tree (same
status as `tests_old`/`source_old`) — never touched.

**Grep-clean confirmation:** repo-wide grep for all 18 removed field
names across `src/firm/` and `src/tests/` (excluding the parked/notebook/
bench paths above) turns up only this ticket's own historical
"removed as dead in 111-004" documentation comments (`pilot.h`,
`boot_config.{h,cpp}`, `DESIGN.md`, the two updated test files) and the
unrelated `DrivetrainConfig.v_wheel_max`/`steer_headroom` fields (a
genuinely separate, deliberately-duplicated pair on a different message,
per `planner.proto`'s own Decision-2 history — untouched).

**Verification:** rebuilt the host sim library (`just build-sim`) so the
`-DHOST_BUILD` sim build — including `Pilot::applyPlannerPatch()` and
every regenerated `messages/` header — was actually recompiled and
exercised, not run against a stale pre-existing `.dylib`. Final
`uv run python -m pytest`: **1224 passed, 18 xfailed, 2 xpassed in
323.80s** — exactly matching the ticket's stated baseline, zero new
failures.

**Flagged for follow-up (not this ticket):** `config_sync_allowlist.json`'s
kept `PlannerConfigPatch.arrive_dwell` justification (carried over
verbatim) says live tuning "goes through `set_config(arriveDwell=...)`
directly" — `protocol.py`'s `_PLANNER_KEYS` map was checked and does
**not** actually register an `arriveDwell` wire key (only `minSpeed`/
`headingKp`/`headingKd`). This inaccuracy predates this ticket (not
introduced by it) and is out of scope for a dead-field-removal ticket;
noted here rather than silently left for a future reader to re-discover.
