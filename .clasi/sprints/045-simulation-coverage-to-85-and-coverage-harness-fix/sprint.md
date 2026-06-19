---
id: '045'
title: Simulation coverage to 85% and coverage-harness fix
status: planning-docs
branch: sprint/045-simulation-coverage-to-85-and-coverage-harness-fix
use-cases: ["SUC-001", "SUC-002", "SUC-003", "SUC-004", "SUC-005", "SUC-006"]
issues: []
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 045: Simulation coverage to 85% and coverage-harness fix

## Goals

1. Fix the stale `tests/_infra/coverage.sh` harness so it runs cleanly and reports overall + simulatable-code coverage.
2. Add targeted simulation tests to raise coverage of simulatable firmware logic from 74.6% to ≥85%.

## Problem

After the FRC Elite migration (Phases 0–F, sprints 038–044), measured host-build line coverage is 74.6% (3879/5200 lines). The coverage harness (`coverage.sh`) is broken: it references the old `source/control/EKF.cpp` path (EKF moved to `source/state/` in Phase C), and its `gcovr --root source` invocation does not match the confirmed-working pattern. Large blocks of simulatable logic (MotorController PI loop, StopCondition kinds, MotionCommandHandlers edge paths, EKF correction branches, etc.) are uncovered by the existing test suite.

## Solution

- **Harness fix**: Rewrite `coverage.sh` to use a fresh build directory, `--root .`, `--filter 'source/'`, `--gcov-ignore-errors=source_not_found`, per-file table output, optional `--fail-under N`, and a "simulatable-code" coverage line that excludes the CODAL-only file set.
- **Test additions**: Targeted simulation tests (pytest, `tests/simulation/unit/` or `tests/simulation/system/`) exercising the reachable-but-uncovered logic in `MotorController`, `StopCondition`, `MotionCommandHandlers`, `RatioPidController`, `Drive`, `PhysicsWorld`, `Odometry`, `EKF`, `SystemCommands`, and `OtosCommands`. No production source changes. No deletions or weakening of existing assertions.

## Success Criteria

- `coverage.sh` runs to completion with no errors.
- Either: (a) overall `source/` line coverage ≥85%, OR (b) simulatable-code coverage ≥85% with a clear exclusion report documenting each CODAL-only file and why it is excluded.
- Full suite green (all 2015+ tests pass), 0 errors.
- Golden-TLM byte-exact, field-pin, and vendor grep gate all green.

## Scope

### In Scope

- `tests/_infra/coverage.sh` — rewrite for correct gcovr invocation.
- New `tests/simulation/unit/` test files for: MotorController inner-loop + wedge paths, StopCondition ROTATION/COLOR/LINE_ANY/HEADING/SENSOR C++ binary paths, RatioPidController (via Sim if reachable), MotionCommandHandlers verb error/edge/fallback paths, Drive subsystem paths, PhysicsWorld dynamics-error/slip paths, Odometry branch coverage, EKF correction/gating branches, testable SystemCommands and OtosCommands paths.
- Documentation of the CODAL-only exclusion set.

### Out of Scope

- Production source changes (`source/`).
- Hardware-only code paths: `DebugCommandable.cpp` `#ifndef HOST_BUILD` I2C handlers, `PortController.cpp` / `ServoController.cpp` hardware-op paths, `SystemCommands.cpp` RESET/hardware-query paths, `BenchOtosSensor.cpp`, `source/io/real/*` device drivers, `main.cpp`, `LoopScheduler`, `WedgeTest`, `Icons`.
- Bench or field tier tests.
- ARM firmware build.

## Test Strategy

All new tests are pytest tests in the simulation tier (`tests/simulation/unit/` or `tests/simulation/system/`). They use the existing `sim` fixture (ctypes `Sim` wrapper built from `tests/_infra/sim/`). Some tests use the Phase-B plant/truth/estimation_error ABI for whole-robot assertions. Tests exercise the C++ binary directly — not pure-Python algorithm mirrors — so they contribute to gcovr line coverage. Ticket sequencing keeps the suite green at every step.

## Architecture Notes

This is a test-additive sprint. No production module changes. The architecture update is light: new test modules only, plus harness fix. The CODAL-only exclusion set is documented in the architecture update and propagated to the harness script.

## GitHub Issues

(none)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [ ] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [ ] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Fix coverage.sh harness: correct gcovr invocation, per-file table, simulatable-code report | — |
| 002 | MotorController and Drive coverage: PI+FF inner loop, ZOH velocity, wedge-detector, RatioPidController audit | 001 |
| 003 | StopCondition C++ binary paths and MotionCommandHandlers edge/fallback branch coverage | 001 |
| 004 | EKF gating, Odometry wedge-suppress, PhysicsWorld slip paths, SystemCommands and OtosCommands coverage | 001 |
| 005 | Measure final coverage: run harness, verify ≥85% simulatable-code gate, report exclusion set | 001, 002, 003, 004 |

Tickets execute serially in the order listed. T002, T003, and T004 depend on T001 (harness must work to verify coverage gains) but are otherwise independent of each other and may be executed in any order among themselves. T005 is the final gate and depends on all prior tickets.
