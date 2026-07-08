---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 090 Use Cases

This is a behavior-preserving internal-quality sprint: no new or changed
end-user-visible behavior, so no top-level `docs/usecases.md` UC is added
or modified. Each SUC below names the closest existing UC its underlying
mechanism serves (so ticket→UC traceability is not orphaned) and is
verified by regression, not by new wire-observable acceptance criteria.
"Actor" for every SUC below is the firmware maintainer/reviewer and the
`tests/sim` regression suite standing in for them — not a robot operator.

## SUC-001: Drivetrain resolves its own bound motor-observation ports
Parent: UC-001 (Drive Robot at Continuous Speed) — also serves UC-002/003
(every drive verb reads the same two bound-port observations)

- **Actor**: Firmware maintainer; `tests/sim` regression suite.
- **Preconditions**: `Drivetrain` is configured with a bound left/right
  port pair (`DEV DT PORTS`, or the boot default); `Hardware::tick()` has
  published this pass's per-port `MotorState` observations.
- **Main Flow**:
  1. The loop passes the WHOLE per-port motor-observation array
     (`bb.motors`) to `Drivetrain::tick()` — no port arithmetic in the loop.
  2. `Drivetrain::tick()` resolves its own bound pair's two observation
     cells internally, using the SAME `ports_` it already owns.
  3. `Drivetrain::tick()` asserts the bound ports are in range before
     indexing (a range check that did not exist before this sprint).
  4. Drive/ratio-governor behavior proceeds identically to before.
- **Postconditions**: `bb.motor` is renamed `bb.motors`; no caller outside
  `Drivetrain` performs `port - 1` indexing into it for Drivetrain's own
  bound pair; an out-of-range bound port asserts instead of silently
  reading adjacent memory.
- **Acceptance Criteria**:
  - [ ] `tests/sim` is green, unchanged pass/fail set.
  - [ ] No wire-observable difference in driven speed/ratio-governor
        behavior for any drive verb.
  - [ ] Every repo-wide reference to `bb.motor` (not only the ones the
        source issue names) compiles against the renamed `bb.motors`.

## SUC-002: Odometer owns reset translation and per-pass fusability
Parent: UC-007 (Set Odometry from External Source)

- **Actor**: Firmware maintainer; `tests/sim` SI/OZ/OR/OV regression tests.
- **Preconditions**: A reset action (`OI`/`OZ`/`OR`/`OV`/`SI`) has been
  posted to `bb.otosCommandIn`/`bb.otosSetPoseIn` this pass.
- **Main Flow**:
  1. The loop still drains the reset mailboxes and applies them to the
     odometer (unchanged — the loop legitimately knows a reset happened
     because it just applied one).
  2. The odometer itself, not the loop, performs the
     `SetPose → Pose2D → OdometerCommand` translation
     (`applySetPose()`).
  3. The loop asks the odometer whether this pass's own OTOS reading is
     fusable (`fusableThisPass()`) instead of computing a local bool.
  4. `PoseEstimator::tick()` is fed `nullptr` instead of `&bb.otos` for
     exactly the one pass a reset landed, exactly as before.
- **Postconditions**: The EKF never fuses a stale, pre-reset OTOS reading
  against a freshly `setPose()`'d belief. Fusion resumes with zero
  innovation on the very next pass.
- **Acceptance Criteria**:
  - [ ] `tests/sim`'s SI/OZ/OR/OV regression tests are run and green
        BOTH before and after this change (not inferred from reading the
        diff).
  - [ ] `encoderPose()`/`fusedPose()` values after an SI/OZ/OR/OV are
        bit-for-bit identical to pre-sprint behavior for the same script.
  - [ ] `bb.otosValid`'s externally-observable value (there is no wire
        surface for it today — confirmed by repo-wide grep) is unaffected.

## SUC-003: No caller branches on odometer-presence nullability
Parent: UC-012 (Initialize and Read OTOS Sensor)

- **Actor**: Firmware maintainer; `tests/sim` regression suite;
  `otos_commands_harness.cpp`/`test_otos_commands_nodev.py`.
- **Preconditions**: `Hardware::odometer()` is called from `main_loop.cpp`,
  `main.cpp`, or `configurator.cpp`.
- **Main Flow**:
  1. `Hardware::odometer()` returns a valid `Hal::Odometer&` unconditionally
     (a `NullOdometer` when no real device is bound) — never `nullptr`.
  2. Every caller drains/applies/ticks the odometer unconditionally; no
     `if (odometer != nullptr)` branch remains at any of the three call
     sites this sprint touches.
  3. `NullOdometer::fusableThisPass()` unconditionally reports `false`, so
     `bb.otosValid` (and, for `main.cpp`'s boot snapshot, `bb.otosPresent`)
     fall out of the SAME query rather than a separate pointer check.
- **Postconditions**: Both concrete `Hardware` owners already override
  `odometer()` to non-null as of ticket 086-006/081-003 (confirmed by
  direct code read — this refactor removes now-dead defensive branches,
  it does not change any currently-reachable production behavior).
- **Acceptance Criteria**:
  - [ ] `tests/sim` is green, including `otos_commands_harness.cpp` (still
        asserts OK, not `ERR nodev`, for every OTOS verb against real
        `NezhaHardware`/`SimHardware`).
  - [ ] `main.cpp`'s `bb.otosPresent` boot snapshot and
        `configurator.cpp`'s odometer-config null-guard are updated to the
        non-nullable contract (grep-verified — the source issue's own
        Scope section does not list these two files).

## SUC-004: One wire-layer authority formats every EVT
Parent: UC-004 (Stop Robot Immediately) — also serves UC-001/002/003/015's
own "done `<verb>`" completion events

- **Actor**: Firmware maintainer; any wire client reading `EVT` lines.
- **Preconditions**: A subsystem (Planner) or the loop itself (safety
  watchdog fire, stream-watchdog-fired safety stop) has something to
  report.
- **Main Flow**:
  1. The producer (Planner, or the loop for its own two loop-originated
     events) builds a typed `msg::Event` — no wire text, no `snprintf`.
  2. Whatever drains the event (the loop's routing step) hands it to
     `CommandProcessor::emitEvent()`.
  3. `emitEvent()` — the ONE place `EVT` grammar is assembled — writes the
     exact same wire text this sprint started with: `EVT dev_watchdog`,
     `EVT safety_stop reason=watchdog`, `EVT done <verb> [#<corr> ]reason=
     <token>`.
- **Postconditions**: `main_loop.cpp` contains zero `snprintf` calls
  building `EVT` text; `motionVerbForMode()`/the `activeModeBeforeTick`
  local are removed (their job moves into Planner + `msg::Event`).
  `MainLoop::activeVelocityVerb_` is RETAINED — it independently gates the
  stream-watchdog's S-vs-R distinction, an unrelated loop-owned concern
  this sprint does not touch.
- **Acceptance Criteria**:
  - [ ] `tests/sim` is green; every existing EVT-format assertion
        (watchdog-fire, motion-done reason tokens, safety_stop) produces
        byte-identical wire text before and after.
  - [ ] Planner never calls a `CommandProcessor` method and never builds a
        wire string (verified by grep: no `snprintf`/`command_processor.h`
        include in `planner.{h,cpp}`).

## SUC-005: MainLoop::tick() reads as named phases
Parent: N/A — cross-cutting; the mechanism underlying every UC's own
control-loop execution, not a use case in its own right.

- **Actor**: Firmware maintainer/debugger.
- **Preconditions**: Tickets 001–004 have landed (the COMMIT block has
  already shrunk to reflect their changes).
- **Main Flow**:
  1. `MainLoop::tick()`'s COMMIT block (the x[k]→x[k+1] snapshot copy) is
     extracted into a private `MainLoop::commit(bb, now)` method, mirroring
     the already-landed `serviceWatchdogs()` extraction (commit `0b2929c5`).
  2. `tick()` itself reads as a named sequence:
     `serviceWatchdogs → control → plan → commit → routeOutputs`.
- **Postconditions**: `MainLoop` (the composition root) still owns wiring
  and commit ordering; no `Blackboard::update(...)` API is introduced
  (explicitly rejected — see architecture-update.md Decision 5).
- **Acceptance Criteria**:
  - [ ] `tests/sim` is green, byte-identical COMMIT-step behavior.
  - [ ] `Blackboard` gains no new method; it remains a pure data struct.
