---
id: '041'
title: "Phase C \u2014 PhysicalStateEstimate seam"
status: roadmap
branch: sprint/041-phase-c-physicalstateestimate-seam
use-cases: []
issues:
- migrate-radio-robot-c-to-the-frc-elite-architecture-c-codal-adaptation.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 041: Phase C — PhysicalStateEstimate seam

## Goals

Consolidate `Odometry` + `EKF` into a single belief object `PhysicalStateEstimate` (§3),
named as the estimate dual of Phase B's `PhysicsWorld`. Composition-first: bodies moved
verbatim, no numerics changed. Strip `Commandable` from the estimator; move OTOS-tuning
verb handlers to an app-layer handler set.

Depends on: Sprint 040 (Phase B) — `PhysicsWorld` / observation models must exist so
the estimator seam has a clean plant to run against in tests.

## Problem

`Odometry` and `EKF` are separate objects; callers reach into both independently.
`Odometry` carries a `Commandable` interface that couples the estimator to the command-
dispatch layer. There is no single named "belief" object — the fused pose is scattered
across `HardwareState` fields. OTOS-tuning verbs (`OI`/`OZ`/`OR`/`OV`/`OL`/`OA`/`OP`)
are handled inside the estimator even though they are app-layer concerns.

## Solution

Following §3 of the issue:

- Create `source/state/PhysicalStateEstimate.{h,cpp}` by composition: wraps `Odometry`
  and `EKF` (bodies verbatim; no numeric changes).
- Move `EKF.{h,cpp}` under `source/state/`.
- Observations-in API: `addOdometryObservation` (= `Odometry::predict`),
  `addOtosObservation` (= `correctEKF`), `addCameraObservation`/`resetPose`
  (= `setPose`, the `SI` verb re-anchor). Observation structs are plain PODs.
- Belief-out API: `getPose()`, `getVelocity()`.
- Dependency rule: `PhysicalStateEstimate` imports `<stdint.h>`/`<math.h>`/`EKF`/PODs
  only — no CODAL, no device handles, no `Protocol.h`, no `Commandable`.
- OTOS-tuning verbs (`OI`/`OZ`/`OR`/`OV`/`OL`/`OA`/`OP`) move to an app-layer handler
  set (new `source/app/OtosCommands.*`).
- Transition safety: keep publishing the fused pose back into `HardwareState.pose*` /
  `fused*` fields so existing readers (`buildTlmFrame`, `MotionController::getPoseFloat`)
  work byte-identically. Readers repointed to `getPose()` in Phase F.

## Key Deliverables

- `source/state/PhysicalStateEstimate.{h,cpp}` with observations-in / belief-out API.
- `source/state/EKF.{h,cpp}` (moved from `source/control/`).
- `Commandable` removed from the estimator.
- `source/app/OtosCommands.*` handling the 7 OTOS-tuning verbs.
- `HardwareState.pose*`/`fused*` still populated each cycle (back-compat, until Phase F).
- `test_ekf.py` and `test_otos_fusion.py` pass unchanged; behavior-preservation fences
  (`test_033_005_wedge_hardening.py`, `test_goto_bounds.py`, etc.) still green.

## Scope

### In Scope

- `PhysicalStateEstimate` class creation by composition.
- `EKF` move to `source/state/`.
- Observations-in API (three observation methods).
- `Commandable` removal from estimator.
- OTOS-tuning verb migration to `source/app/OtosCommands.*`.
- Back-compat pose mirroring into `HardwareState` (unchanged until Phase F).

### Out of Scope

- Repointing TLM readers from `HardwareState` to `getPose()` (Phase F).
- `RobotState.h` split (Phase F).
- `Superstructure` / `Goal` enum (Phase D).
- Subsystem wrapping (Phase E).

## Architecture Notes

- Composition-first: `PhysicalStateEstimate` wraps existing objects by value; no numeric
  changes. Correctness is guaranteed by the unchanged algorithms.
- Dependency rule is strict: the estimator must not import device headers or
  `Protocol.h`. Enforced by the vendor-confinement grep gate (extended to `source/state/`).
- `PhysicalStateEstimate` is the belief's canonical name. "RobotState" blob name is not
  reused (retired in Phase F).
- Observation structs are plain PODs so the estimator is testable without device mocks.

## Definition of Done (Phase C — from issue §3 / Migration sequence)

- [ ] `source/state/PhysicalStateEstimate.{h,cpp}` compiles; EKF is at
      `source/state/EKF.{h,cpp}`.
- [ ] Observations-in API: `addOdometryObservation`, `addOtosObservation`,
      `addCameraObservation`/`resetPose`.
- [ ] Belief-out API: `getPose()`, `getVelocity()`.
- [ ] `PhysicalStateEstimate` has no CODAL / device / Protocol / Commandable imports.
- [ ] OTOS-tuning verbs handled in `source/app/OtosCommands.*`.
- [ ] `HardwareState.pose*`/`fused*` still populated each cycle (back-compat).
- [ ] `test_ekf.py`, `test_otos_fusion.py`, all behavior-preservation fences still green.
- [ ] Simulation tier green (≥ 1954 tests): `uv run --with pytest python -m pytest -q`
- [ ] `defaultRobotConfig()` field-pin diff empty.
- [ ] Golden-TLM frame canary unchanged.
- [ ] Vendor-confinement grep gate passes (Phase C scope).
- [ ] No new heap allocation or fibers introduced.

## Tickets

| # | Title | Depends On |
|---|-------|------------|

Tickets execute serially in the order listed.
