---
id: '044'
title: "Phase F \u2014 Logging and rename/cleanup"
status: planning-docs
branch: sprint/044-phase-f-logging-and-rename-cleanup
use-cases: []
issues:
- migrate-radio-robot-c-to-the-frc-elite-architecture-c-codal-adaptation.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 044: Phase F — Logging and rename/cleanup

## Goals

Complete the FRC Elite Architecture migration (§6): repoint TLM readers to
`estimate.getPose()`, stop mirroring pose into `HardwareState`, split `RobotState.h` →
`source/types/Inputs.h`, retire the "RobotState" blob name, delete the old `I*` alias
shim headers, finalize the REPLAY stub, and enforce the logging contract. Final
verification that all canaries pass and the grep gate is clean. This phase closes the
migration.

Depends on: Sprint 043 (Phase E) — subsystems with `updateInputs()` must exist before
the logging contract can be enforced.

## Problem

After Phases A–E, several transitional scaffolds remain: TLM readers still read pose from
`HardwareState` fields (not `estimate.getPose()`); `HardwareState` still has the back-
compat pose mirror; the "RobotState" name is still in use; old `I*` alias shims are still
present; the REPLAY mode is a stub that has never been exercised; and the "every subsystem
writes its inputs slice in `updateInputs`, no subsystem prints" logging contract is not
yet enforced. The alias shims from Phase A and the back-compat mirror from Phase C are
both dead weight at this point.

## Solution

Following §6 of the issue:

- **TLM repoint:** update `buildTlmFrame` / `telemetryEmit` / `MotionController::getPoseFloat`
  and all remaining `HardwareState.pose*`/`fused*` readers to call `estimate.getPose()`
  directly.
- **Stop mirroring:** remove the back-compat pose-mirror write into `HardwareState` that
  was added in Phase C.
- **`RobotState.h` split:** extract the inputs/logging struct to `source/types/Inputs.h`
  (the `HardwareState` blob becomes `Inputs`); retire the "RobotState" name. The `types/`
  directory (also housing `Config.h`, `Protocol.h`, `CommandTypes.h`) is finalized.
- **Delete old alias shims:** remove `using IMotor = IVelocityMotor;` and the other
  transition shims introduced in Phase A. Delete deprecated `I*` headers.
- **Logging contract enforcement:** verify every subsystem writes its inputs slice in
  `updateInputs()` and no subsystem prints during the loop tick. Add a lint check or
  canary if feasible.
- **REPLAY stub exercise:** exercise `RobotMode::REPLAY` + `ReplayHAL` (no-op feed
  impls) — verify it compiles, links, and the stub is exercised by at least one test.
- **Final verification:** vendor-confinement grep returns zero hits above `source/io/`;
  the four-file device quartet exists per capability; the three seams are findable;
  `defaultRobotConfig()` field-pin diff empty; golden-TLM canary passes; all behavior-
  preservation fences pass.

## Key Deliverables

- `estimate.getPose()` / `estimate.getVelocity()` are the canonical pose sources for TLM
  and `MotionController`.
- `HardwareState` no longer has the back-compat pose mirror.
- `source/types/Inputs.h` exists; `HardwareState` blob renamed to `Inputs`.
- "RobotState" name fully retired from the source tree.
- Old `I*` alias shim headers deleted.
- `RobotMode::REPLAY` + `ReplayHAL` stub compiles and is exercised.
- Vendor-confinement grep: zero hits above `source/io/`.
- Final canary sweep: all three canaries green, all behavior fences green.
- The three seams are identifiable in the directory layout.

## Scope

### In Scope

- TLM reader repoint to `estimate.getPose()`.
- Back-compat pose mirror removal from `HardwareState`.
- `RobotState.h` → `source/types/Inputs.h` rename/split.
- "RobotState" name retirement throughout source tree.
- Old `I*` alias shim header deletion.
- Logging contract enforcement check (`updateInputs` convention).
- `ReplayHAL` stub exercise (compile + minimal test).
- Final vendor-confinement grep sweep (must return zero above `source/io/`).
- Final canary sweep + architecture verification.

### Out of Scope

- REPLAY mode full implementation (TLM-log replay pipeline — deferred per issue §6).
- New behavior, sensor improvements, EKF tuning — this entire migration is structural
  only; no behavioral work is folded in.
- PathPlanner, advanced state machines, or anything not in the §1–§6 scope.

## Architecture Notes

- This phase finalizes the §5 directory layout: `source/types/` houses `Config.h`,
  `Protocol.h`, `CommandTypes.h`, and the new `Inputs.h`.
- TLM is the inputs-struct log: `buildTlmFrame`/`telemetryEmit` over the `Inputs` struct
  (not a separate logging layer). The REPLAY-fed TLM log is the AdvantageKit-replay
  analogue — seam cut, impl deferred.
- After this phase the vendor-confinement grep gate is definitive and can be enforced
  permanently in CI.
- After this phase the codebase fully embodies the FRC Elite Architecture as adapted for
  C++/CODAL firmware: three named seams, capability-typed IO, first-class plant sim,
  belief object, thin Superstructure, cooperative periodic, inputs-struct logging.

## Definition of Done (Phase F — from issue §6 / Migration sequence + Verification)

- [ ] `estimate.getPose()` / `getVelocity()` called by TLM and `MotionController`
      (no reads from `HardwareState.pose*`/`fused*`).
- [ ] Back-compat pose mirror removed from `HardwareState`.
- [ ] `source/types/Inputs.h` exists; "RobotState" blob name retired everywhere.
- [ ] Old `I*` alias shim headers deleted; no dangling `using` aliases in headers.
- [ ] `RobotMode::REPLAY` + `ReplayHAL` stub compiles and is exercised by a test.
- [ ] Logging contract: every subsystem writes inputs slice in `updateInputs()`.
- [ ] Vendor-confinement grep returns zero hits above `source/io/` (final).
- [ ] Four-file device quartet exists per capability in `source/io/`.
- [ ] Three seams findable: `source/io/capability/`, `source/state/`, `source/superstructure/`.
- [ ] All behavior-preservation fences green: `test_033_005_wedge_hardening.py`,
      `test_goto_bounds.py`, `test_incident_scenarios.py`, `test_ekf*.py`,
      `test_otos_fusion.py`, `test_watchdog_exemption.py`, `sim_field_profile`.
- [ ] Simulation tier green (≥ 1954 tests): `uv run --with pytest python -m pytest -q`
- [ ] `defaultRobotConfig()` field-pin diff empty.
- [ ] Golden-TLM frame canary unchanged.
- [ ] No new heap allocation or fibers introduced.
- [ ] `python3 build.py --clean` succeeds for both REAL and SIM `ROBOT_RUN_MODE`.

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 044-001 | Repoint TLM and MotionController pose reads to PhysicalStateEstimate seam | — |
| 044-002 | Move RobotState.h to source/types/Inputs.h and retire RobotState name | 044-001 |
| 044-003 | Resolve DebugCommandable I2CBus leak via IBusDiagnostics+IRawBusAccess; empty vendor baseline | 044-002 |
| 044-004 | Delete alias shims, finalize REPLAY stub, and add seam-presence + logging-contract tests | 044-001, 044-002, 044-003 |

Tickets execute serially in the order listed.
