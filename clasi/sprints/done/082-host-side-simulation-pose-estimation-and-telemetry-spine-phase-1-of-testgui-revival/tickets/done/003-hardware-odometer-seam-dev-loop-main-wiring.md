---
id: '003'
title: Hardware::odometer() seam + dev-loop/main wiring
status: done
use-cases:
- SUC-003
depends-on:
- '002'
github-issue: ''
issue: plan-revive-testgui-against-the-new-tree-simulator.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Hardware::odometer() seam + dev-loop/main wiring

## Description

Give `devLoopTick` a uniform way to reach whichever concrete `Hal::Odometer`
leaf (if any) the active `Subsystems::Hardware` owner has, and wire
`Subsystems::PoseEstimator` (ticket 002) into the shared dev-loop body so
every pass -- ARM firmware and host sim alike -- advances the pose estimate
exactly once.

**Note on timing**: by the time this ticket executes, sprint 081 will have
merged (082's hard execution gate). `source/subsystems/hardware.h`,
`sim_hardware.{h,cpp}`, `nezha_hardware.{h,cpp}`, `dev_loop.{h,cpp}`, and
`main.cpp` referenced below are 081's *final, merged* versions -- not the
in-progress files visible during 082's planning. Re-read each file fresh at
ticket-execution time rather than assuming this ticket's description is
byte-exact against whatever revision existed during planning.

**Stakeholder decision (2026-07-05, recorded on this sprint's
`stakeholder_approval` gate): accept sim-only OTOS/fused pose for 082. Do
NOT add a real-hardware OTOS I2C driver in this ticket or anywhere in this
sprint** -- `Subsystems::NezhaHardware` inheriting the `nullptr` default
(below) and never being touched is the intended, approved end state for 082.
A real-hardware `Hal::Odometer` leaf is deferred to its own later sprint,
not scheduled by this document.

## Acceptance Criteria

- [x] `Subsystems::Hardware` (`source/subsystems/hardware.h`) gains
      `virtual Hal::Odometer* odometer() { return nullptr; }` -- a defaulted,
      non-pure virtual (matching that class's existing `begin()` no-op
      precedent), NOT a pure virtual every owner must implement.
- [x] `Subsystems::SimHardware::odometer()` overrides it, returning
      `&odometer_` (the already-existing `Hal::SimOdometer` member from
      sprint 081 ticket 003) -- a one-line addition. (Required one
      accompanying rename: the class already had a concrete, non-virtual
      `Hal::SimOdometer& odometer()` test/ctypes accessor that collides with
      the new virtual `Hal::Odometer* odometer()` seam -- C++ cannot overload
      on return type alone. Renamed the pre-existing accessor to
      `simOdometer()`, the same "sim-prefixed concrete twin" pattern already
      used for `motor()`/`simMotor()` in the same class; updated its 8 call
      sites in `tests/_infra/sim/sim_api.cpp` and 3 in
      `tests/sim/unit/sim_hardware_harness.cpp`.)
- [x] `Subsystems::NezhaHardware` requires **zero** source changes -- it
      compiles and links unchanged, inheriting the `nullptr` default. This is
      verified explicitly (diff shows no touch to `nezha_hardware.{h,cpp}`),
      not merely assumed. Confirmed via `git diff --stat -- source/subsystems/nezha_hardware.h source/subsystems/nezha_hardware.cpp` (empty output).
- [x] `source/dev_loop.h`'s `DevLoop` struct gains a
      `Subsystems::PoseEstimator* poseEstimator = nullptr;` field.
- [x] `devLoopTick()` gains exactly one new step: after the SECOND
      `hardware.tick(now)` slice (freshest encoder reads) and before the
      watchdog check, it:
      1. Queries `drivetrain.ports()` **unconditionally** (not only inside
         `if (drivetrain.active())` as today) to get the bound wheel pair.
      2. Reads `hardware.motor(p.left).state()` / `.state()` for the right
         port (`msg::MotorState`, matching what `Drivetrain::tick()` already
         consumes).
      3. Calls `hardware.odometer()`; if non-null, calls its `tick(now)` and
         samples `pose()` into a local `msg::PoseEstimate`.
      4. Calls `loop.poseEstimator->tick(now, leftObs, rightObs,
         odometer present ? &sampledPose : nullptr)` **exactly once** per
         `devLoopTick()` call.
- [x] A standalone harness (matching `tests/sim/unit/*_harness.cpp`'s
      ad hoc-compile convention) proves `poseEstimator->tick()` is invoked
      exactly once per `devLoopTick()` pass, not twice -- the same class of
      double-integration hazard sprint 081's `SimHardware` dt=0 guard
      documents for `MotorVelocityPid::compute()`. This is the single most
      important correctness check in this ticket; do not treat it as
      optional. `tests/sim/unit/dev_loop_pose_estimator_harness.cpp`
      (+ `test_dev_loop_pose_estimator.py` wrapper): proves this via exact
      (bit-for-bit) agreement, across many passes, between the REAL
      `devLoopTick()` and an independently hand-driven once-per-pass
      reference pipeline -- across drivetrain-inactive, drivetrain-active,
      and rebound-port-pair scenarios. See the harness's own file header for
      why a literal "same-instant duplicate call" mutant is a proven
      mathematical no-op for this exact accumulator+EKF (not evidence of a
      test gap).
- [x] `source/main.cpp` constructs a `Subsystems::PoseEstimator`, calls its
      `configure()` with the same `msg::DrivetrainConfig` already built for
      `drivetrain.configure(dtConfig)` (one shared boot-config source, no
      duplicated values), and wires `&poseEstimator` into `DevLoop`.
- [x] Hardware bench smoke (`.claude/rules/hardware-bench-testing.md`): ARM
      build behavior for existing verbs is unaffected -- `PING`/`DEV` family
      round-trip identically before and after this ticket's two new
      `devLoopTick` steps land. Record the actual command transcript.
      **Deferred to ticket 005 per team-lead direction**: 003's and 004's
      bench smokes are being consolidated into ticket 005's single HITL
      session rather than run twice.

## Implementation Plan

### Approach

1. Re-read (fresh, post-081-merge) `source/subsystems/hardware.h`,
   `sim_hardware.{h,cpp}`, `dev_loop.{h,cpp}`, `main.cpp` in full before
   editing -- confirm the exact current shape of `devLoopTick()`'s two-slice
   hardware-tick sequence and where the watchdog check sits, since this
   ticket's new step must be inserted at a precise point in an existing,
   carefully-ordered function.
2. Add `Subsystems::Hardware::odometer()` (defaulted `nullptr`) to
   `hardware.h`; add the one-line override to `sim_hardware.h`/`.cpp`.
3. Add the `PoseEstimator*` field to `DevLoop`; add the new tick step to
   `devLoopTick()` exactly once, positioned per Acceptance Criteria.
4. Update `main.cpp`'s construction/wiring block (mirroring how it already
   constructs and configures `drivetrain`).
5. Write the standalone once-per-pass harness FIRST (before declaring the
   wiring done) -- this is the ticket's highest-risk correctness property.

### Files to create

- `tests/sim/unit/dev_loop_pose_estimator_harness.cpp` (or fold into an
  existing dev-loop harness if one already exists post-081 -- check before
  creating a new file).

### Files to modify

- `source/subsystems/hardware.h` -- add `odometer()`.
- `source/subsystems/sim_hardware.h` / `.cpp` -- add the override.
- `source/dev_loop.h` / `.cpp` -- add the `PoseEstimator*` field and the new
  tick step.
- `source/main.cpp` -- construct/configure/wire `PoseEstimator`.
- `source/subsystems/nezha_hardware.{h,cpp}` -- explicitly **not** modified;
  confirm this in the ticket's own PR/diff review.

**Discovered during execution, not in the plan above, both forced by
`devLoopTick()`'s own new unconditional `loop.poseEstimator->tick(...)`
call:**

- `tests/_infra/sim/sim_api.cpp` -- `SimHandle` is the OTHER production
  caller of the shared `devLoopTick()` (besides `main.cpp`); without wiring a
  real `Subsystems::PoseEstimator` into its own `DevLoop loop` the same way
  `main.cpp` now does, every `sim_tick()`/`sim_command()` call would
  null-deref on `loop.poseEstimator->tick(...)`, crashing the entire
  `tests/sim` pytest gate. Added a `Subsystems::PoseEstimator poseEstimator`
  member, `configure()`d from the same `dtConfig` `drivetrain.configure()`
  already takes, wired via `loop.poseEstimator = &poseEstimator;`.
- `tests/_infra/sim/sim_api.cpp` / `tests/sim/unit/sim_hardware_harness.cpp`
  -- `Subsystems::SimHardware` already had a concrete, non-virtual
  `Hal::SimOdometer& odometer()` test/ctypes accessor predating this ticket
  (sprint 081-003); it collides with the new virtual `Hal::Odometer*
  odometer()` override (C++ cannot overload on return type alone). Renamed
  the pre-existing accessor to `simOdometer()` (matching the class's own
  `motor()`/`simMotor()` precedent for the identical duality) and updated its
  8 + 3 call sites respectively.
- `tests/_infra/sim/CMakeLists.txt` -- added `source/subsystems/
  pose_estimator.cpp` and `source/estimation/ekf_tiny.cpp` to
  `FIRMWARE_SOURCES` (now referenced transitively via `dev_loop.cpp` ->
  `pose_estimator.h`) and `libraries/tinyekf` to the include path
  (`ekf_tiny.h`'s `tinyekf.h`), per the ticket's own "Build/CMake" note.

### Testing plan

- New standalone harness proving exactly-once-per-pass invocation (see
  Acceptance Criteria) -- the ticket's primary test.
- Hardware bench smoke per `.claude/rules/hardware-bench-testing.md`:
  deploy (`mbdeploy deploy --build`), confirm `PING`/`DEV M`/`DEV DT` round-
  trip unchanged, encoders still increment on `DEV DT VW`/`WHEELS` commands.
- Do not yet write sim ground-truth-tracking tests for `pose=`/`encpose=` --
  those need the telemetry surface (ticket 004) to read the values out over
  the wire; this ticket's tests are internal/host-level only.

### Documentation updates

- None wire-visible yet (no new verb). Update `source/subsystems/hardware.h`'s
  own file-header comment to mention the new `odometer()` seam and its
  defaulted-nullptr rationale, matching that file's existing documentation
  style for `begin()`.
