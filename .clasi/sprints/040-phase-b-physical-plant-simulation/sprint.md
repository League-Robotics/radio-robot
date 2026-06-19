---
id: '040'
title: "Phase B \u2014 Physical-plant simulation"
status: planning-docs
branch: sprint/040-phase-b-physical-plant-simulation
use-cases: []
issues:
- migrate-radio-robot-c-to-the-frc-elite-architecture-c-codal-adaptation.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 040: Phase B — Physical-plant simulation

## Goals

Replace the welded mock-sim with a first-class, settable physical-plant simulation (§2):
`PhysicsWorld` as the single ground truth, per-capability observation models that layer
error on top, and `SimHardware` owning the plant. Migrate the ~25 sim-driven tests via
aliases. Bodies moved verbatim; no behavior changes.

Depends on: Sprint 039 (Phase A) — capability-typed interfaces must exist before
observation models can implement them.

## Problem

Today `MockMotor`, `ExactPoseTracker`, and `BenchOtosSensor` fuse plant truth, slip, and
sensor error into single objects. This means tests cannot isolate plant-only, observation-
only, or estimator-only behavior; `sim_set_enc_l/r` lies (sets commanded rather than true
travel); `BenchOtos` integrates commanded velocity while `ExactPose` uses true velocity,
so the two can disagree. The canonical midpoint-arc integration is triplicated. There is
no way to assert estimate-vs-truth.

## Solution

Following §2 of the issue:

- **`PhysicsWorld`** (`source/io/sim/PhysicsWorld.{h,cpp}`) — single ground truth.
  Owns true chassis pose, true per-wheel travel/velocity, true sensor values, OTOS mount
  transform, dynamics-error extras. Two ways in: `setActuators(pwmL,pwmR)`+`update(dt)`
  (evolve) and `setTruePose`/`setTrueWheelTravel`/… (set truth directly for isolation
  tests). Canonical midpoint-arc integration (consolidates the three current copies).
  Slip moves here (chassis-integration step), validated numerically against the
  `sim_field_profile` fixture.
- **Observation models** (`SimMotor`, `SimOdometer`, `SimLineSensor`, `SimColorSensor`,
  `SimPortIO`) — own the error, read `const PhysicsWorld&`, implement the capability
  interfaces. Every error setter defaults to no-op (fresh sensor is perfect).
- **`SimHardware`** — owns the plant, constructs each observation model against it; its
  `tick(now,cmds)` is the one ordered `plant.update(dt)`.
- **Control law on one side (Case B):** per-wheel PI+FF stays in `MotorController`
  above the device line; `SimMotor` only stores PWM — no second controller.
- **`WorldView` adapter** + `sim_get_true_*` ABI → `sim.estimation_error()` makes
  estimate-vs-truth assertable.
- **ABI / tests:** `sim_api.cpp`/`firmware.py` surface kept back-compat; `sim_set_enc_l/r`
  fixed to set true travel; `sim_get_exact_pose_*` aliased to `sim_get_true_pose_*`;
  new settable-truth + error-layer + `sim_set_perfect()` hooks. ~45 pure-Python mirror
  tests untouched. Deterministic stepped time + fixed-seed LCG preserved.
- New plant-correctness isolation tests per §2 verification matrix.

## Key Deliverables

- `source/io/sim/PhysicsWorld.{h,cpp}` with settable truth + `update(dt)`.
- `SimMotor`, `SimOdometer`, `SimLineSensor`, `SimColorSensor`, `SimPortIO` observation
  models in `source/io/sim/`.
- `SimHardware` replaces `MockHAL` as the SIM-mode Hardware impl.
- `WorldView` adapter; `sim_get_true_*` ABI additions.
- `sim_set_enc_l/r` fixed (sets true travel, not commanded).
- `sim_field_profile` slip behavior numerically preserved.
- `estimate-vs-truth` assertable via `sim.estimation_error()`.
- New isolation tests (plant-only, observation-only, estimator-only, whole-robot) in
  `tests/simulation/`.
- All ~25 existing sim-driven tests pass (via aliases or minor repoints).
- All Phase 0 canaries still green; vendor-confinement gate passes.

## Scope

### In Scope

- `PhysicsWorld` plant creation and integration.
- Observation model impls (`Sim*`).
- `SimHardware` factory.
- `WorldView` adapter + `sim_get_true_*` ABI.
- `sim_set_enc_l/r` fix.
- `sim_field_profile` slip migration to plant.
- New plant-correctness isolation tests.
- Migration of ~25 existing sim-driven tests (path/alias updates).

### Out of Scope

- `PhysicalStateEstimate` / EKF consolidation (Phase C).
- `Superstructure` / `Goal` enum (Phase D).
- Subsystem wrapping (Phase E).
- TLM reader repoint / cleanup (Phase F).
- Bench-tier or field-tier test content beyond what already exists.

## Architecture Notes

- `PhysicsWorld` is the single source of truth; observation models read `const
  PhysicsWorld&` and never write to it.
- Control law (PI+FF) stays above the device line; `SimMotor` is not a second controller.
- Deterministic stepped time and fixed-seed LCG are preserved for reproducible tests.
- `MockHAL`/`MockMotor`/`ExactPoseTracker` are superseded by `SimHardware`/`SimMotor`/
  `PhysicsWorld`; old files deleted once tests pass.
- Zero-heap, single-threaded constraints preserved; `SimHardware` uses value-member
  ownership matching the real HAL pattern.

## Definition of Done (Phase B — from issue §2 / Migration sequence)

- [ ] `PhysicsWorld` exists with settable truth + `update(dt)`.
- [ ] All observation models (`Sim*`) implement their capability interfaces.
- [ ] `SimHardware` constructs plant + models; `tick(now,cmds)` drives `plant.update(dt)`.
- [ ] `sim_set_enc_l/r` sets true wheel travel (bug fixed).
- [ ] `sim_field_profile` slip behavior numerically verified against Phase 0 baseline.
- [ ] `estimate-vs-truth` assertable: `sim.estimation_error()` works.
- [ ] Isolation tests pass: plant-only, observation-only, estimator-only, whole-robot.
- [ ] All ~25 existing sim-driven tests pass.
- [ ] Simulation tier green (≥ 1954 tests): `uv run --with pytest python -m pytest -q`
- [ ] `defaultRobotConfig()` field-pin diff empty.
- [ ] Golden-TLM frame canary unchanged.
- [ ] Vendor-confinement grep gate passes (Phase B scope).
- [ ] No new heap allocation or fibers introduced.

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 040-001 | PhysicsWorld — ground-truth plant with canonical midpoint-arc integration | — |
| 040-002 | Observation models and SimHardware — replace MockHAL with clean split | 040-001 |
| 040-003 | WorldView adapter, sim_get_true_* ABI, fix sim_set_enc, estimation_error | 040-002 |
| 040-004 | Retire Mock* files and delete obsolete sim objects | 040-003 |
| 040-005 | Isolation test matrix — plant, observation, estimator, whole-robot | 040-004 |

Tickets execute serially in the order listed.
