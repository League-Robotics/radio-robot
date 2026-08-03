---
id: '013'
title: Delete the *ConfigPatch surface, PatchKind, merge accumulator; reshape persisted
  tuning
status: open
use-cases:
- SUC-001
- SUC-005
depends-on:
- '009'
- '010'
- '011'
- '012'
github-issue: ''
issue: the-configuration-object.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Delete the *ConfigPatch surface, PatchKind, merge accumulator; reshape persisted tuning

## Description

Delete `DrivetrainConfigPatch`/`MotorConfigPatch`/`OtosConfigPatch`/
`EstimatorConfigPatch` (`config.proto`), `ConfigDelta::PatchKind` and its
union (`envelope.h`), and the merge accumulator
(`mergeMotorGainsPatch`/`mergeOtosPatch`, `configurator.cpp`) — the whole
surface the new group/field wire arms (tickets 008-012) replace.
`Configurator::persistedTuning_` (currently
`msg::MotorConfigPatch`/`OtosConfigPatch`-shaped) becomes a snapshot of
`Config::Robot`'s live-reappliable groups (`DRIVE`/`WHEEL_CONTROL`/
`MOTORS`-travel_calib/`OTOS`/`ESTIMATOR` per ticket 008's table),
following today's persistence PRECEDENT — Motor gains/travel_calib and
OTOS scale/offset persist; `EstimatorConfigPatch` never has
(`config.proto`'s own comment: "a reboot always reverts to the baked
JSON default"). Document explicitly which groups persist under the new
shape rather than silently expanding or contracting the set (see
sprint.md Out of Scope).

**This is the point in the sprint where mid-sprint breakage is expected**:
any host code not yet migrated (ticket 014's job) will fail to build/run
against the deleted patch types until that ticket lands. That is
accepted, per stakeholder direction — do not add a compatibility shim to
keep it working in between.

## Acceptance Criteria

- [ ] `DrivetrainConfigPatch`, `MotorConfigPatch`, `OtosConfigPatch`,
      `EstimatorConfigPatch` no longer exist in `config.proto` (or the
      file itself is deleted if nothing else lives in it — confirm
      during implementation).
- [ ] `ConfigDelta::PatchKind` and its union no longer exist in
      `envelope.h`.
- [ ] `mergeMotorGainsPatch`/`mergeOtosPatch` and
      `Configurator::apply(const msg::CommandEnvelope&)` (the old
      patch-based entry point) are deleted.
- [ ] `Configurator::persistedTuning_` is reshaped to a
      `Config::Robot`-groups-shaped snapshot; the persistence scope
      (which groups persist) is documented in `configurator.h`'s file
      header, following today's precedent (Motor/Otos persist,
      Estimator does not) unless this ticket's own finding says
      otherwise — in which case the finding and its reasoning are
      recorded in the ticket's completion notes.
- [ ] A repo-wide grep for `ConfigPatch`/`PatchKind` inside `src/firm/`
      returns nothing outside git history.
- [ ] Compiles under `HOST_BUILD` (firmware side only — host-side
      breakage until ticket 014 is expected and NOT this ticket's
      problem to fix).

## Testing

- **Existing tests to run**: firmware-side tests referencing the OLD
  patch types are expected to need updating or deletion as part of this
  ticket (they test a surface that no longer exists) — update/delete
  them, do not leave them referencing dead types.
- **New tests to write**: a persistence round-trip test for the reshaped
  `persistedTuning_` (save, reload, confirm the same live-reappliable
  groups come back).
- **Verification command**: `uv run python -m pytest <firmware-side sim
  test paths, host-side EXCLUDED this ticket> -q`.

## Implementation Plan

**Approach**: Delete the old types and the old `Configurator::apply()`
entry point; reshape `persistedTuning_`'s storage type and its save/load
serialization to match `Config::Robot`'s live-reappliable groups.

**Files to modify**: `src/protos/config.proto` (or delete),
`src/firm/messages/envelope.h`, `src/firm/app/configurator.{h,cpp}`,
`src/firm/config/persisted_tuning.{h,cpp}`.

**Files to delete**: any firmware test file that exists solely to test
the deleted patch surface (confirm none is testing something else
incidentally before deleting).

**Testing plan**: as above.

**Documentation updates**: `configurator.h`'s file header rewritten to
describe the new group/field-based ownership model in full (superseding
its current "three things it used to own" description, which describes
the old patch-merge design).
