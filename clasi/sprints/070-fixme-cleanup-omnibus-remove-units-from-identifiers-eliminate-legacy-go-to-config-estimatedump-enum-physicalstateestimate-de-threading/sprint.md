---
id: "070"
title: "FIXME cleanup omnibus: remove units from identifiers, eliminate legacy go-to config, EstimateDump enum, PhysicalStateEstimate de-threading"
status: roadmap
branch: sprint/070-fixme-cleanup-omnibus-remove-units-from-identifiers-eliminate-legacy-go-to-config-estimatedump-enum-physicalstateestimate-de-threading
use-cases: []
issues:
- remove-units-from-identifier-names.md
- fixme-cleanup-legacy-config-and-estimatedump-enum.md
- physicalstateestimate-remove-hardwarestate-param-threading.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 070: FIXME cleanup omnibus

## Goals

Sweep up the remaining code-quality FIXME issues into clean, behavior-preserving
refactors: remove unit suffixes from identifier names (units move to a standard
line comment), eliminate the legacy go-to tolerance config fields and convert
`EstimateDump.source` to an enum, and de-thread `HardwareState` out of
`PhysicalStateEstimate`'s per-call signatures. All three are pure refactors —
no behavioral change; the full test suite and TLM output stay identical.

## Issues

1. **remove-units-from-identifier-names.md** — codebase-wide rename: ~60+
   unit-suffixed C++ identifiers (`Mm`, `Mms`, `Deg`, `Dps`, `Ms`, `Us`, `Pct`,
   `Hz`) + host Python (`_mm`, `_ms`, `read_ms` ×121, …). Units documented in a
   standard `// [mm/s]` line comment. Wire/protocol names renamed in lock-step
   on both sides or explicitly excluded (documented).
2. **fixme-cleanup-legacy-config-and-estimatedump-enum.md** — remove
   `turnThresholdMm`/`doneTolMm` end-to-end (Config, defaults, ConfigRegistry
   keys, PlannerConfig/Planner plumbing; record the wire-compat decision);
   `EstimateDump.source` string → `enum class EstimateSource`; delete the
   now-resolved FIXME comments so `grep -ri FIXME source/` is clean.
3. **physicalstateestimate-remove-hardwarestate-param-threading.md** — stop
   passing `HardwareState&` on every method; make inputs (encoder readings) and
   config (trackwidth, rotational slip, EKF noise — with runtime setters) and
   the three pose-estimate outputs explicit; unify `setCtx` as the single
   injection point.

## Planning considerations (for detail phase)

- **Sequencing / likely split.** These interact on the same files
  (`source/types/Config.h`, `source/robot/DefaultConfig.cpp`,
  `source/state/*`). Recommended order: (1) legacy-field removal +
  EstimateDump enum + PhysicalStateEstimate de-threading FIRST (they shrink and
  reshape the surface), THEN (2) the units rename LAST as a mechanical sweep
  over the reduced surface — so the rename doesn't have to touch fields that are
  about to be deleted. The units rename is large enough it may warrant its own
  sprint (split at detail-planning if the ticket sequence isn't clean).
- **Behavior-preserving contract.** No wire/TLM/SET-key change unless renamed
  in lock-step both sides; every rename ticket ends on a green full suite with
  byte-identical TLM output.
- **Consumes prior sprints:** 067 made `SET` propagate live (the
  PhysicalStateEstimate `configure()` setters must preserve that); 068's
  three-pose TLM output must stay byte-identical; 069's `SIMSET`/config keys
  must not be broken by renames.

## Definition of Ready

- [ ] Working tree quiesced — stakeholder's in-flight source edits (Config.h,
      DefaultConfig.cpp, config regen, packaging) committed, so this sprint
      branches from a clean, stable base (these refactors touch the same files).
- [ ] Detail planning complete (usecases, architecture, tickets)
- [ ] Architecture review passed
- [ ] Stakeholder approved

## Tickets

| # | Title | Depends On |
|---|-------|------------|

(Filled at detail-planning.)
