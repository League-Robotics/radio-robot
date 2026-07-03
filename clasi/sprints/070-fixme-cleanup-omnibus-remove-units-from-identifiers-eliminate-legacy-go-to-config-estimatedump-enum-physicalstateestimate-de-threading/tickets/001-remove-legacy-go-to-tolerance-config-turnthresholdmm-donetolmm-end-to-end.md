---
id: '001'
title: Remove legacy go-to tolerance config (turnThresholdMm/doneTolMm) end-to-end
status: open
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: fixme-cleanup-legacy-config-and-estimatedump-enum.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Remove legacy go-to tolerance config (turnThresholdMm/doneTolMm) end-to-end

## Description

`Config.h:125`'s "FIXME Eliminate legacies" marks `turnThresholdMm`/`doneTolMm`
as dead: live go-to gating runs on `turnInPlaceGate`/`arriveTolMm` (confirmed
by sprint 067's audit and re-confirmed this sprint by direct grep — zero hits
for `getTurnThreshold()`/`getDoneTol()`). These two fields and their
`turnThr`/`doneTol` SET/GET keys were deliberately *retained* in sprint 011
"because removing them would break existing calibration scripts" — this
ticket implements **Decision 4** (approved at the stakeholder gate): reverse
that call and remove the fields, defaults, registry keys, and message
projection end-to-end. This is the sprint's one deliberate, flagged
wire-behavior change: `SET turnThr=`/`SET doneTol=` go from
silently-accepted-and-ignored to `ERR badkey`. See
`architecture-update.md` Step 5 ("Legacy go-to config removal") and Decision 4
for full detail; see `usecases.md` SUC-001 for the acceptance contract.

## Acceptance Criteria

- [ ] `source/types/Config.h` — the "Go-to tolerances (legacy...)" block
      (`turnThresholdMm`, `doneTolMm`, and their FIXME comment, ~lines
      124-126) is deleted.
- [ ] `source/robot/DefaultConfig.cpp:96-97` — the two default assignments
      are deleted.
- [ ] `source/robot/ConfigRegistry.cpp:66-67` — the `turnThr`/`doneTol`
      registry rows are deleted. `SET turnThr=1` / `SET doneTol=1` now reply
      `ERR badkey`; `GET` dumps no longer list either key.
- [ ] `protos/planner.proto` — `turn_threshold`/`done_tol` fields 10/11 are
      deleted from `message PlannerConfig`, replaced with `reserved 10, 11;`.
- [ ] `scripts/gen_messages.py` — the two `("PlannerConfig", "...")` field-map
      rows are deleted; `source/messages/planner.h` is regenerated (drops the
      two fields/getters/setters).
- [ ] `source/superstructure/PlannerConfig.cpp:39-40` (and `.h`) — the
      `cfg.setTurnThreshold(...)`/`cfg.setDoneTol(...)` forwarding calls and
      any doc-comment mentions of the two fields are deleted.
- [ ] `source/superstructure/Planner.cpp:643-644` — the planner-side
      application of the two fields is deleted.
- [ ] `tests/_infra/default_config_golden.json` regenerated (drops the two
      lines).
- [ ] `tests/simulation/unit/test_config_registry.py` —
      `test_legacy_turnThr_still_present`/`test_legacy_doneTol_still_present`
      rewritten to assert `ERR badkey` instead of presence; any
      `("turnThr", "float_as_int")`/`("doneTol", ...)` table rows and
      `DEFAULT_GET_LINE`/expected-dump-string fixtures updated to drop the two
      keys; any hardcoded total-key-count test (e.g. `test_full_get_36_keys`)
      renamed/renumbered to the new real count.
- [ ] `docs/protocol-v2.md` — `turnThr`/`doneTol` removed from the Named Key
      Table and the two example `CFG ...` dump lines; the stale `G`-command
      prose (currently describes pre-rotate/done gating in terms of
      `turnThr`/`doneTol`, which have not actually gated `G` since sprint 011
      introduced `turnInPlaceGate`/`arriveTolMm`) corrected to name the fields
      that are actually live.
- [ ] `docs/design/message-inventory.md` regenerated (drops the two rows).
- [ ] `docs/overview.md`, `docs/architecture.md` — one-line mentions of
      `doneTol`/`turnThresholdMm`/`doneTolMm` updated to the live equivalents.
- [ ] `grep -rn "turnThresholdMm\|doneTolMm\|\"turnThr\"\|\"doneTol\""
      source/` (excluding comments/docs already updated above) returns
      nothing.
- [ ] Full test suite green; TLM output byte-identical (this ticket touches
      no TLM field).

## Testing

- **Existing tests to run**: `tests/simulation/unit/test_config_registry.py`;
  full default suite (`uv run python -m pytest`) — baseline is 2612 passed, 0
  failed (minus the two rewritten legacy-key tests, which change assertion
  but not count).
- **New tests to write**: none required beyond rewriting the two existing
  legacy-key tests to assert `ERR badkey`. If the total-registered-key-count
  test exists, update its expected count.
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: Delete the field/default/registry-key/message-projection/
planner-application chain in one pass, then regenerate the two golden
fixtures (`default_config_golden.json`, `source/messages/planner.h` via
`gen_messages.py`) and update the two existing regression tests to assert the
new `ERR badkey` behavior instead of presence. This is a pure leaf-node
deletion — nothing else in the config graph depends on this subtree (the only
other consumer, `Planner::_planCfg`, was already confirmed dead by sprint
067).

**Files to modify**:
- `source/types/Config.h` (~lines 124-126)
- `source/robot/DefaultConfig.cpp:96-97`
- `source/robot/ConfigRegistry.cpp:66-67`
- `protos/planner.proto`
- `scripts/gen_messages.py`
- `source/messages/planner.h` (regenerated, not hand-edited)
- `source/superstructure/PlannerConfig.h`, `PlannerConfig.cpp:39-40`
- `source/superstructure/Planner.cpp:643-644`
- `tests/_infra/default_config_golden.json` (regenerated)
- `tests/simulation/unit/test_config_registry.py`
- `docs/protocol-v2.md`, `docs/design/message-inventory.md` (regenerated),
  `docs/overview.md`, `docs/architecture.md`

**Testing plan**: run `tests/simulation/unit/test_config_registry.py` in
isolation first to confirm the rewritten assertions pass, then the full
suite (`uv run python -m pytest`) to confirm no other test references either
key or field. A `--clean` sim build is required before running tests, since
`Config.h`/`ConfigRegistry.cpp` are ARM-target-and-sim-shared source and
`gen_messages.py` must be re-run before the build (project knowledge: stale
incremental builds on `/Volumes` — build banners lie).

**Documentation updates**: `docs/protocol-v2.md` (Named Key Table + example
dump lines + `G`-command prose correction), `docs/design/message-inventory.md`
(regenerated), `docs/overview.md`, `docs/architecture.md`.
