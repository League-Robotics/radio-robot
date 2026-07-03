---
status: pending
sprint: '070'
---

# FIXME cleanup: eliminate legacy go-to config fields; EstimateDump source string → enum

## Description

Sweep of all live `FIXME` markers in the codebase (2026-07-02). Most are
stakeholder markers about unit suffixes in names — those are catalogued as
explicit references in remove-units-from-identifier-names.md and are NOT
duplicated here. This issue covers the remaining two, plus removal of all
the FIXME comments themselves once their substance is tracked (same
pattern as sprint 008's source-fixme-cleanup omnibus).

## 1. Eliminate legacy go-to tolerance config fields

`source/types/Config.h:124` — "FIXME Eliminate legacies", sitting on the
"Go-to tolerances (legacy, retained for backward compatibility)" block:
`turnThresholdMm` and `doneTolMm`.

Remaining uses:

- `source/robot/DefaultConfig.cpp:96-97` — defaults (50.0 / 5.0)
- `source/robot/ConfigRegistry.cpp:66-67` — SET/GET keys `turnThr`,
  `doneTol`
- `source/superstructure/PlannerConfig.cpp:39-40` — forwarded into the
  planner config
- `source/superstructure/Planner.cpp:643-644` — planner applies them

Decide whether anything still meaningfully consumes these (vs. the
sprint-011 pose-control tunables like `arriveTolMm` that superseded them)
and remove the fields, their defaults, their registry keys, and the
planner plumbing. If a host tool or stored robot config still sends
`turnThr`/`doneTol`, either drop with a deprecation note or keep the wire
keys as accepted-but-ignored for one release — note which in the ticket.

## 2. EstimateDump.source should be an enum

`source/state/EstimateDump.h:23` — `const char* source; // "enc",
"otos", "fuse" FIXME should be an enum.`

Replace the string tag with an `enum class EstimateSource { Encoder,
Optical, Fused }` (or equivalent); keep the wire/telemetry text ("EST enc
…" lines) produced by a single to-string mapping at the emit point so the
TLM format is unchanged.

## 3. Remove the FIXME comments

Once items 1-2 land here and the units renames land under
remove-units-from-identifier-names.md, delete the corresponding FIXME
comments so `grep -ri FIXME source/` comes back clean. (The two mentions
in `StopCondition.cpp:20` / `ColorUtil.cpp:4` are historical references
to an already-resolved FIXME, not live markers — reword or leave.)

## Acceptance criteria

- `turnThresholdMm`/`doneTolMm` removed end-to-end (Config struct,
  defaults, ConfigRegistry keys, PlannerConfig/Planner plumbing), with
  the wire-compat decision recorded.
- `EstimateDump::source` is an enum; TLM `EST enc/otos/fuse` output is
  byte-identical.
- No live `FIXME` markers remain in `source/` whose substance isn't
  tracked in an issue.
- Build green; host + sim tests pass.
