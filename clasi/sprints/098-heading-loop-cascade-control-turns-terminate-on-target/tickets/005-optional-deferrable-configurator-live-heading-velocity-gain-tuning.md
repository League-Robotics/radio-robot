---
id: '005'
title: '[OPTIONAL/DEFERRABLE] Configurator live heading/velocity gain tuning'
status: in-progress
use-cases:
- SUC-003
depends-on:
- '003'
github-issue: ''
issue:
- heading-loop-cascade-control-turns-terminate-on-target.md
- real-robot-motion-calibration-undershoot.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# [OPTIONAL/DEFERRABLE] Configurator live heading/velocity gain tuning

## ⚠️ OPTIONAL/DEFERRABLE — skip if the overnight run's risk budget is spent

The mandatory path (001→002→003→006) already satisfies the sprint's
acceptance criterion WITHOUT this ticket. Independent of ticket 004 — skip
either, both, or neither without affecting the other. If skipped, ticket
006 notes the deferral and closes the sprint with reflash-based tuning
(ticket 003's own method) as the only tuning path, exactly as sprints
093-097 already operate today.

## Description

Wire a minimal `Rt::Configurator` into `main.cpp`'s live loop so a binary
`SET` config delta actually reaches the running `Drivetrain`/`Hardware`,
cutting heading/velocity gain-tuning iteration from a reflash (~5 minutes)
to a live `SET` (seconds). Additive only — boot config still applies once,
directly, at construction, exactly as today; this does NOT reintroduce
093/094-era full runtime config authority.

Reference: `architecture-update.md` M7, SUC-003. `real-robot-motion-
calibration-undershoot.md`'s "Also discovered" section is the origin of
this gap: binary `SET` already acks into `bb.configIn` (ticket 096), but
nothing has drained it since 093/094 removed runtime config authority.

Depends on 003 — tune against the bench-verified Stage 1 baseline, not a
moving target.

## Acceptance Criteria

- [x] `main.cpp` constructs one `Rt::Configurator`, seeded from the SAME
      boot `msg::DrivetrainConfig`/`msg::PlannerConfig` values already
      passed directly to `drivetrain.configure()`/
      `drivetrain.configureMotion()` at construction — boot behavior is
      PROVABLY unchanged (a freshly booted robot with no `SET` ever sent
      behaves identically to today).
- [x] `main.cpp`'s loop calls `configurator.applyOne(bb)` once per pass
      (mirroring the pre-093/094 pattern) — placed so it drains at most one
      `bb.configIn` delta per pass, matching `Configurator::applyOne()`'s
      own documented one-delta-per-call contract.
- [x] `Rt::Configurator::applyOne()`'s existing `kPlanner` case gains ONE
      new line: `drivetrain_.configureMotion(plannerConfig_);` immediately
      after the `foldPlanner(...)` call, alongside the existing
      `bb.plannerConfig = plannerConfig_;` publish — today that case only
      folds+publishes (a residue of ticket 094-002 relocating
      `Subsystems::Planner` out of `source/`); `Subsystems::Drivetrain` is
      the correct live target now (the Configurator already holds a
      `Drivetrain&`).
- [x] `kMotor`/`kDrivetrain`/`kOdometer`'s existing, already-correct
      fold-and-apply paths are UNCHANGED — this ticket touches the
      `kPlanner` case only.
- [x] SIM ACCEPTANCE: a new scenario drives a `SET`-equivalent config
      delta for `heading_kp` mid-session (via `bb.configIn`/whatever the
      sim harness's existing config-delta injection surface is) and
      confirms the VERY NEXT segment's commanded twist reflects the new
      gain — no restart, no reflash-equivalent.
- [x] Full `uv run python -m pytest` stays green, no regression.
- [ ] HARDWARE ACCEPTANCE: a bench session sends a live `SET` for
      `heading_kp` (or `heading_kd`) over serial/relay and confirms (via
      `TLM`/a subsequent `turn_sweep.py` cell) the change took effect
      WITHOUT a reflash. **NOT YET DONE — reserved for the team-lead's
      hardware pass.** See "Implementation Notes" below for a real gap
      this bullet must ALSO confirm/route around: the binary wire `config`
      command cannot carry `heading_kp`/`heading_kd` today (only
      `min_speed` is wire-settable for `kPlanner` — `msg::PlannerConfigPatch`,
      `protos/config.proto`, only declares that one field). A hardware `SET
      heading_kp=...` will need that wire-schema extension FIRST (out of
      this ticket's stated file scope — see note) before it can reach
      `bb.configIn` at all.

## Testing

- **Existing tests to run**: full `uv run python -m pytest`.
- **New tests to write**: the live-`SET`-changes-live-behavior sim
  scenario itemized above.
- **Verification command**: `uv run python -m pytest`; a bench
  `SET heading_kp=<value>` followed by an immediate re-run of one
  `turn_sweep.py` cell as the hardware confirmation.

## Implementation Plan

**Approach**: Construct-and-tick the existing `Rt::Configurator` class
(already fully implemented, just never instantiated in `main.cpp` since
093/094) plus the one-line `kPlanner` fix.

**Files to modify**: `source/main.cpp`, `source/runtime/configurator.cpp`.

**Files to create**: none.

**Testing plan**: as above.

**Documentation updates**: none required structurally.

## Implementation Notes (programmer, post-implementation)

The `Rt::Configurator` shape the architecture/ticket describes was verified
CORRECT by direct read (`source/runtime/configurator.{h,cpp}`): constructor
signature, `applyOne(bb)`'s one-delta-per-call contract, and the `kPlanner`
case's fold-only (no live-apply) shape all matched exactly. Three places
needed judgment calls or turned out to differ from what the ticket assumed:

1. **`PlannerConfigField`/`foldPlanner()` did not yet cover `heading_kp`/
   `heading_kd` at all** (verified: `source/runtime/commands.h`'s
   `PlannerConfigField` enum stopped at `kMinSpeed`, and
   `configurator.cpp`'s `foldPlanner()` had no matching fold lines) — a real
   gap beyond the ticket's stated one-line `kPlanner` fix, since without a
   mask bit + fold line, a `heading_kp` `Rt::ConfigDelta` folds to nothing
   and the SIM ACCEPTANCE criterion is unsatisfiable. Added `kHeadingKp`/
   `kHeadingKd` to the enum and two fold lines to `foldPlanner()` — a
   mechanical, minimal extension of the SAME existing pattern (every other
   `PlannerConfig` field already has exactly this), confined to the
   `kPlanner` fold path only (`kMotor`/`kDrivetrain`/`kOdometer` untouched,
   per the ticket's own boundary). Documented inline at both sites.
2. **The binary wire `config`/`SET` command cannot carry `heading_kp`/
   `heading_kd` at all, independent of (1)** — verified:
   `commands/binary_channel.cpp`'s `handleConfigPlanner()` only forwards
   `msg::PlannerConfigPatch.min_speed`; that generated wire-message type
   (`protos/config.proto`) declares only that one field. This is a SEPARATE,
   larger gap (a `protos/config.proto` schema change + regen + a
   `binary_channel.cpp` edit) that this ticket's stated file scope
   (`main.cpp`, `configurator.cpp`) does not cover and this implementation
   did NOT attempt. The SIM ACCEPTANCE scenario therefore injects the
   `Rt::ConfigDelta` directly via `bb.configIn.post()` — exactly what the
   ticket text sanctions ("via `bb.configIn`/whatever the sim harness's
   existing config-delta injection surface is") and exactly how scenarios
   1-9 in `configurator_harness.cpp` already inject every other target's
   deltas — rather than round-tripping through the wire `config` command.
   **This means the HARDWARE ACCEPTANCE bullet cannot be satisfied with a
   real `SET heading_kp=...` over serial/relay as written today** — the
   wire schema has no field to carry it. Flagged here for the team-lead to
   triage (scope call: extend this ticket, open a follow-on ticket/issue, or
   accept reflash-only tuning per the ticket's own "OPTIONAL/DEFERRABLE...
   exactly as sprints 093-097 already operate today" fallback) — no issue
   file created by this implementation pass; that decision belongs to the
   team-lead, not the ticket's implementer.
3. **`main.cpp` had zero `Subsystems::PoseEstimator` instance** — `Rt::
   Configurator`'s constructor requires one (a `kDrivetrain`-scoped delta
   re-propagates to it). Added `static Subsystems::PoseEstimator
   poseEstimator;`, constructed but never ticked (Stage 2/M6's OTOS wiring,
   ticket 098-004, is independent and not landed on this branch) — inert,
   since `PoseEstimator` holds no hardware reference. Mirrors `tests/_infra/
   sim/sim_api.cpp`'s own `SimHandle` (096-004), which added the identical
   instance for the identical reason.
4. **Loop placement**: `configurator.applyOne(bb)` is called right after
   `router.route()` and before `tickTelemetry()`/`hardware.tick()`/
   `drivetrain.tick()` — so a delta routed THIS pass is already live before
   this SAME pass's `drivetrain.tick()` runs (one tick sooner than draining
   after the commit step). Documented inline in `main.cpp` at the call site
   and in the file's header comment.
5. Also added `configurator.publish(bb)` at boot (right after the existing
   `bb.drivetrainConfig = dtConfig;` seed) — mirrors `sim_api.cpp`'s own
   boot sequence, fills in `bb.motorConfig[]`/`bb.plannerConfig`/
   `bb.odometerConfig` (previously always zero-valued in `main.cpp`, never
   set by anything) with the real boot values. Pure telemetry/`GET`-
   visibility fix (`publish()` never calls any subsystem's `configure()`),
   not a control-loop behavior change.

Verification: `just build-sim` and `just build-clean` both succeed;
`uv run python -m pytest tests/sim tests/unit` is 896 passed before and
after (the new scenario is scenario 10 inside `configurator_harness.cpp`,
exercised by the existing single `test_configurator.py` pytest wrapper, so
the collected-test COUNT is unchanged by design — the scenario itself was
directly compiled/run standalone and confirmed passing, then reconfirmed via
the pytest wrapper).
