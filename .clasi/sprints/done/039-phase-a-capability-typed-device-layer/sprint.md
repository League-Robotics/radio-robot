---
id: 039
title: "Phase A \u2014 Capability-typed device layer"
status: done
branch: sprint/039-phase-a-capability-typed-device-layer
use-cases: []
issues:
- migrate-radio-robot-c-to-the-frc-elite-architecture-c-codal-adaptation.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 039: Phase A — Capability-typed device layer

## Goals

Introduce capability-typed device interfaces (§1), seal the vendor-confinement leaks in
`MotorController`, move the Nezha split-phase state machine into the `Motor` impl, rename
`hal/` → `io/`, and introduce `ROBOT_RUN_MODE`. All bodies moved verbatim; no behavior
changes.

Depends on: Sprint 038 (Phase 0) — canaries must be green before source moves begin.

## Problem

Today's device interfaces are named by device (`IMotor`, `IOtosSensor`, `IServo`) rather
than by capability, and vendor types leak above the HAL boundary: `MotorController.h`
includes `MicroBit.h` and names `I2CBus`; `IOtosSensor` exposes `int16` LSBs and `cfg`
refs; the split-phase motor state machine lives in `Robot::controlCollectSplitPhase`
rather than in the `Motor` impl where it belongs. These leaks will block the seam
architecture in all later phases.

## Solution

Following §1 of the issue:

- Add `source/io/capability/` headers: `IVelocityMotor.h`, `IPositionMotor.h`,
  `IOdometer.h`, `ILineSensor.h`, `IColorSensor.h`, `IPortIO.h`, `IBusDiagnostics.h`.
  Initially as `using` aliases over current `I*` headers so callers compile unchanged.
- Seal `MotorController.h`: drop `#include "MicroBit.h"` and `I2CBus*`; expose bus
  diagnostics upward only as `IBusDiagnostics` via a `MotorBusDiagnostics` adapter.
- Move Nezha split-phase request/collect state machine from `Robot::controlCollectSplitPhase`
  into `Motor` impl, driven by `Hardware::tick(now)`. `positionMm()` becomes a cheap
  accessor. I2C bytes on the wire unchanged.
- Split `IMotor` → `IVelocityMotor` + `IPositionMotor` (one `Motor` impl exposes both
  via `asPositionMotor()` accessor — RTTI-free). Fold `IServo` into `IPositionMotor`
  (`Servo` impl implements `IPositionMotor` only).
- Rename `IOtosSensor` → `IOdometer`; seal the LSB/`cfg` leaks (raw register access
  stays, but as `IOdometer` methods — `int16` LSBs never cross the interface).
- Rename `source/hal/` → `source/io/`; establish `io/real/` and `io/sim/` subdirs
  (populated progressively).
- Introduce `ROBOT_RUN_MODE` (`REAL`|`SIM`|`REPLAY`) in CMake, replacing the
  `list(FILTER … EXCLUDE REGEX)` mock-exclusion hack.
- Add alias shims (e.g. `using IMotor = IVelocityMotor;`) so old consumers compile
  green during transition; delete old `I*` headers in Phase F.

## Key Deliverables

- `source/io/capability/` with all 7 capability interface headers.
- `MotorController.h` no longer includes `MicroBit.h` or names `I2CBus`.
- `IBusDiagnostics` interface + `MotorBusDiagnostics` adapter in place.
- Split-phase state machine in `Motor` impl; `positionMm()` is a cheap accessor.
- `IVelocityMotor`, `IPositionMotor`, `IOdometer` live at `source/io/capability/`.
- `IServo` folded into `IPositionMotor`; `Servo` impl updated.
- `source/io/` directory layout established (`real/`, `sim/` subdirs scaffolded).
- `ROBOT_RUN_MODE` in CMake; old `list(FILTER … EXCLUDE REGEX)` removed.
- Vendor-confinement grep gate passes with ratcheted scope (no vendor types above
  `source/io/` — Phase A baseline).
- All Phase 0 canaries still green.

## Scope

### In Scope

- `source/io/capability/` interface headers (§1 taxonomy).
- `MotorController` vendor-leak seal (`MicroBit.h`, `I2CBus`).
- `IBusDiagnostics` + `MotorBusDiagnostics` adapter.
- Split-phase move into `Motor` impl.
- `IMotor` → `IVelocityMotor` + `IPositionMotor` split; `IServo` fold.
- `IOtosSensor` → `IOdometer` rename and LSB/`cfg` seal.
- `hal/` → `io/` rename; `io/real/` and `io/sim/` subdir scaffold.
- `ROBOT_RUN_MODE` CMake variable.
- Alias shims to keep existing consumers compiling.
- Ratchet vendor-confinement grep gate to Phase A baseline.

### Out of Scope

- `PhysicsWorld` or observation-model changes (Phase B).
- `PhysicalStateEstimate` / EKF consolidation (Phase C).
- `Superstructure` / `Goal` enum (Phase D).
- Subsystem wrapping with `periodic()` / `updateInputs()` (Phase E).
- TLM reader repoint / `RobotState.h` split / old header deletion (Phase F).

## Architecture Notes

- Secondary-capability discovery is RTTI-free: `virtual IPositionMotor* asPositionMotor()
  { return nullptr; }` accessor pattern (firmware likely `-fno-rtti`).
- `RobotConfig&` stays as an impl member; only removed from public read signatures.
- `OtosPose`/`OtosVelocity` → `Pose2D`/`BodyTwist` with `using` aliases to avoid TLM/
  test churn.
- Alias shims (`using IMotor = IVelocityMotor;`) are temporary scaffolding, deleted in
  Phase F.
- I2C bytes on the wire are unchanged; the split-phase timing moves into the impl but
  the protocol is identical.

## Definition of Done (Phase A — from issue §1 / Migration sequence)

- [ ] `source/io/capability/` contains all 7 capability interface headers.
- [ ] `MotorController.h` has no `#include "MicroBit.h"` and no `I2CBus` references.
- [ ] `IBusDiagnostics` interface + `MotorBusDiagnostics` adapter compile and link.
- [ ] Split-phase state machine lives in `Motor` impl; `Robot::controlCollectSplitPhase`
      removed.
- [ ] `IVelocityMotor`, `IPositionMotor`, `IOdometer` are the canonical interfaces.
- [ ] `IServo` removed; `Servo` impl derives from `IPositionMotor`.
- [ ] `source/io/` layout in place; `source/hal/` deleted (or aliased away).
- [ ] `ROBOT_RUN_MODE` drives the CMake build; old filter removed.
- [ ] Vendor-confinement grep gate passes (Phase A scope: no vendor types above
      `source/io/`).
- [ ] Simulation tier green (≥ 1954 tests): `uv run --with pytest python -m pytest -q`
- [ ] `defaultRobotConfig()` field-pin diff empty.
- [ ] Golden-TLM frame canary unchanged.
- [ ] No new heap allocation or fibers introduced.

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 039-001 | Capability interface headers + IBusDiagnostics + seal MotorController vendor leaks | — |
| 039-002 | Move Nezha split-phase state machine into Motor impl; add positionMm/velocityMmps accessors | 039-001 |
| 039-003 | Split IMotor into IVelocityMotor and IPositionMotor; fold IServo into IPositionMotor | 039-001, 039-002 |
| 039-004 | Rename IOtosSensor to IOdometer; seal RobotConfig from public signatures; Pose2D/BodyTwist aliases | 039-001, 039-003 |
| 039-005 | Rename hal/ to io/; establish real/ sim/ subdirs; ROBOT_RUN_MODE CMake; ReplayHAL stub | 039-001, 039-002, 039-003, 039-004 |

Tickets execute serially in the order listed.
