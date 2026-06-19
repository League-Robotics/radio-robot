---
id: 038
title: "Phase 0 \u2014 Test tiers and safety-net canaries"
status: done
branch: sprint/038-phase-0-test-tiers-and-safety-net-canaries
use-cases: []
issues:
- migrate-radio-robot-c-to-the-frc-elite-architecture-c-codal-adaptation.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 038: Phase 0 — Test tiers and safety-net canaries

## Goals

Stand up the three-tier test directory structure (simulation / bench / field — §7) and
relocate the existing host suite into `tests/simulation/`. Add safety-net canaries that
must stay green across every subsequent migration phase. `source/` is untouched.

Depends on: nothing (first phase).

## Problem

The seven-phase FRC Elite Architecture migration needs a safety net before any source
moves. Today the host tests live in a flat `tests/` layout with no tier separation, and
there are no automated gates for calibration regression, vendor confinement, or
telemetry-frame stability. Without these canaries, a source reorganization could silently
break behavior or calibration and only be caught at field time.

## Solution

Reorganize `tests/` into the §7 tier layout (`simulation/{unit,system}`,
`bench/{unit,system}`, `field/{unit,system}`, `_infra/`). Move the existing 1954+ host
tests into `simulation/` (path updates only — no logic changes). Add three canary tests:

1. **Vendor-confinement grep gate** — asserts no `MicroBit`/`I2CBus`/Nezha/`int16…Raw`
   types appear above `source/io/` in the source tree. Establishes the Phase 0 baseline.
2. **`defaultRobotConfig()` field-pin** — captures a hash/snapshot of every field in
   `defaultRobotConfig()` and asserts it is unchanged. Gates all subsequent phases
   against calibration drift.
3. **Golden-TLM frame canary** — runs a fixed deterministic command sequence through
   the sim and asserts the telemetry frame bytes are unchanged. Baseline for behavior
   preservation.

Record the calibration baseline and bench smoke baseline in `_infra/`.

## Key Deliverables

- `tests/` reorganized into the §7 tier layout; all existing tests still pass at their
  new paths.
- `simulation/` tier is the canonical always-run gate: `uv run --with pytest python -m pytest -q`
  from `tests/simulation/` (or equivalent) passes with ≥ 1954 tests green.
- Vendor-confinement grep gate in place (Phase 0 baseline — no tightening yet).
- `defaultRobotConfig()` field-pin test in place and green.
- Golden-TLM frame canary in place and green.
- `_infra/` contains sim build helpers, testkit, `calibrate/`, and tools (moved, not
  rewritten).

## Scope

### In Scope

- `tests/` directory reorganization (§7 layout).
- Moving existing test files to new paths; updating imports and `conftest.py` references.
- Three new canary tests (vendor grep gate, field-pin, golden-TLM).
- `_infra/` scaffold: sim build (`sim_api/firmware.py/CMake`), testkit helpers,
  `calibrate/`, tools.
- CMake / `build.py` path updates needed to keep the sim build green.

### Out of Scope

- Any changes to `source/` (zero source moves in Phase 0).
- Tightening the vendor-confinement grep gate (Phase A will ratchet it).
- New simulation logic, PhysicsWorld, or observation models (Phase B).
- Bench-tier or field-tier test content beyond what already exists.

## Architecture Notes

- This phase is `tests/` reorg only. The §7 tier layout is the target for all seven
  phases; establishing it now means each subsequent phase drops tests into the right
  place without further restructuring.
- The canaries are the migration's regression harness. Their baselines are set in this
  phase and must not change unless a phase explicitly adjusts them (with documented
  rationale).
- `loopTickOnce` is shared firmware↔sim and must remain so; no changes here.
- Zero heap, single-threaded, deterministic constraints are unaffected (source
  untouched).

## Definition of Done (Phase 0 — from issue §7 / Migration sequence)

- [ ] `tests/` reorganized into `simulation/{unit,system}`, `bench/{unit,system}`,
      `field/{unit,system}`, `_infra/`.
- [ ] All existing host tests pass at new paths (simulation tier ≥ 1954 tests green).
  Canonical command: `uv run --with pytest python -m pytest -q`
- [ ] Vendor-confinement grep gate passes (Phase 0 baseline established).
- [ ] `defaultRobotConfig()` field-pin diff is empty (calibration unchanged).
- [ ] Golden-TLM frame canary passes (behavior baseline locked).
- [ ] `source/` has zero modifications (no behavior changes, no source moves).
- [ ] No new heap allocation or fibers introduced.

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Scaffold tier dirs and move sim infra to _infra/sim/ | — |
| 002 | Move simulation tests to simulation/unit/ and repoint pyproject testpaths | 001 |
| 003 | Move bench/calibrate/tools/system into bench/ _infra/ and field/system/ | 002 |
| 004 | Vendor-confinement grep-gate canary with Phase 0 baseline | 003 |
| 005 | defaultRobotConfig() field-pin canary with golden snapshot | 003 |
| 006 | Golden-TLM frame canary with committed capture | 003 |
| 007 | Coverage harness under _infra/ (cmake --coverage + gcovr) | 003 |

Tickets execute serially in the order listed. Tickets 004–007 all depend on 003
and may be executed in any order among themselves.
