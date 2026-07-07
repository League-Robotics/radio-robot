---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 087 Use Cases

Parent issue: [`clasi/issues/plan-file-a-design-issue-blackboard-architecture-state-objects-command-queues.md`](../../issues/plan-file-a-design-issue-blackboard-architecture-state-objects-command-queues.md).

This is an internal firmware-architecture sprint: the "actor" in every use
case below is the firmware itself (its runtime behavior) or a firmware
developer/reviewer (the person who benefits from the restructuring), not a
robot operator. No wire-visible behavior changes — the `SET`/`DEV`/motion/
`SI`/`ZERO` verb surface, its replies, and its timing contract are unchanged;
this sprint changes how the firmware is wired internally, not what it does
from the host's point of view. Use cases are correspondingly thin, per the
sprint brief, and each maps directly to one of the design's locked decisions.

## SUC-001: Subsystem ticks are deterministic and order-independent

- **Actor**: The firmware runtime (and, downstream, anyone reasoning about or
  testing it).
- **Preconditions**: Two or more subsystems each depend, directly or
  transitively, on another subsystem's output (e.g. `Drivetrain` reads motor
  observations; `PoseEstimator` and `Planner` read motor observations and
  fused pose).
- **Main Flow**:
  1. The loop reads the committed snapshot `x[k]` (the blackboard's state
     plane) and passes each subsystem its own slice as explicit `tick()`
     arguments.
  2. Each subsystem computes its next state from `x[k]` alone and writes only
     its own cell — never a peer's, never the blackboard directly.
  3. After every subsystem has ticked, the loop bulk-copies each subsystem's
     cell into the blackboard, producing `x[k+1]` in one atomic step (the
     "clock edge").
- **Postconditions**: The result of a pass depends only on `x[k]` and the
  inputs consumed that pass — never on the order subsystems were ticked in.
- **Acceptance Criteria**:
  - [ ] Re-ordering the mandatory-tick call sequence in the loop body
        produces bit-identical `x[k+1]` for a fixed `x[k]` and fixed inputs.
  - [ ] No subsystem reads another subsystem's freshly-written cell within
        the same pass (verified by construction: `tick()` signatures take
        only `x[k]`-sourced arguments, never a peer reference).
  - [ ] The one sanctioned exception — the watchdog/emergency-stop neutral —
        still acts same-pass, not on the next edge.

## SUC-002: A subsystem is unit-testable fully in isolation

- **Actor**: A firmware developer writing or running a subsystem unit test.
- **Preconditions**: A subsystem (e.g. `Drivetrain`, `PoseEstimator`,
  `Planner`, `Hardware`) needs a test that exercises its `tick()` logic.
- **Main Flow**:
  1. The developer constructs the subsystem with no peers (default
     constructor takes no subsystem references).
  2. The developer hands `tick()` plain snapshot values and, where the
     subsystem consumes commands, a queue instance pre-loaded with a test
     command.
  3. The developer asserts on the subsystem's own `state()`/output-edge
     accessor.
- **Postconditions**: The test required no mock subsystem, no wiring
  harness, and no ordering assumption — every dependency the subsystem has is
  visible in its `tick()` signature.
- **Acceptance Criteria**:
  - [ ] Every control subsystem's entire dependency set is enumerable by
        reading its `tick()` signature alone.
  - [ ] No subsystem's header includes `blackboard.h` or another subsystem's
        header (only `messages/*.h` and the generic queue-template header).
  - [ ] `configure()`/`config()` exist on every subsystem (killing the
        `*Shadow` workaround), so a test can set up and read back config
        without touching the Configurator.

## SUC-003: All configuration writes apply through one authority

- **Actor**: The firmware runtime, processing a `SET`/`DEV M CFG`/
  `DEV DT CFG`/OTOS-config wire command.
- **Preconditions**: A config-writing command arrives on the wire.
- **Main Flow**:
  1. The `SET`-family handler reads the current-config state object, folds
     and validates the candidate synchronously, and replies `ERR` immediately
     on failure (nothing enqueued).
  2. On success it posts one target-tagged `ConfigDelta` onto the
     Configurator's single `WorkQueue` and replies `OK`.
  3. In the loop's best-effort slack, the Configurator folds the delta into
     its per-target desired config and calls that subsystem's plain
     `configure()` — a full replace, not a queued mutation the subsystem must
     itself interpret.
  4. The Configurator publishes the new current config to the blackboard's
     config state cells, where `GET`/telemetry read it.
- **Postconditions**: No command handler holds a subsystem pointer; the three
  scattered shadow-config sets (`motorConfigShadow[]`/`drivetrainConfigShadow`,
  `drivetrainShadow`/`motorShadow[]`/`plannerShadow`, `configShadow`) and the
  cross-family `sTimeoutWatchdog` reach-through no longer exist.
- **Acceptance Criteria**:
  - [ ] `SET tw=128 rotSlip=0` (and every other existing `SET`/`DEV *CFG` key)
        still validates synchronously and still replies `OK`/`ERR` on the
        same tick it was sent, even though application is deferred.
  - [ ] A burst of `SET`s followed by a motion command still executes the
        motion command on the very next mandatory pass (config application
        never blocks motion routing).
  - [ ] `GET`/telemetry read config values that reflect the most recently
        *applied* delta, not a stale shadow.

## SUC-004: State-reset commands (`SI`, `ZERO enc`) apply atomically at the clock edge

- **Actor**: The firmware runtime, processing `SI <x> <y> <h>` or `ZERO enc`.
- **Preconditions**: `SI`/`ZERO enc` arrives on the wire.
- **Main Flow**:
  1. The router fans the verb out into typed reset commands on each affected
     target's own reset queue (`PoseEstimator`'s pose-reset `WorkQueue` for
     `SI`; `PoseEstimator`'s baseline reset plus `Hardware`'s per-motor reset
     flags for `ZERO enc`) — reading any needed port binding from the
     snapshot, never from a `Drivetrain*`.
  2. `PoseEstimator` drains its own reset queue inside its own `tick()` (the
     phantom-jump coherence stays inside the subsystem that owns the
     entangled integration), using its existing pending-flag mechanism.
  3. All fanned-out effects are consumed on the same next edge and commit to
     `x[k+1]` together.
  4. The handler replies `OK` synchronously (accepted + routed); the effect
     lands next edge, identical in spirit to the `SET` pattern.
- **Postconditions**: `SI`'s re-anchor and `ZERO enc`'s re-baseline are each
  internally atomic (no partial-reanchor window), without a new locking or
  transaction mechanism — atomicity is a free consequence of the clock edge.
- **Acceptance Criteria**:
  - [ ] `SI <x> <y> <h>` re-anchors both `encoderPose()` and `fusedPose()`
        together, with the odometer's next fusion pass reading an already-
        agreeing sample (no one-tick phantom residual).
  - [ ] `ZERO enc` zeroes both motors' encoder accumulators and re-syncs
        `PoseEstimator`'s delta baseline on the same edge (no `-E` phantom
        jump on the next tick).
  - [ ] Neither reset path routes through the Configurator.

## SUC-005: The control cadence is protected from configuration and routing load

- **Actor**: The firmware runtime, under load from a mix of motion commands
  and configuration writes.
- **Preconditions**: The loop is running; commands of mixed kinds (motion,
  config, statements) arrive within one period.
- **Main Flow**:
  1. Each pass, the loop runs the mandatory control tick first
     (`Hardware`/`Drivetrain`/`PoseEstimator`/`Planner`, then commit).
  2. In the remaining time before the next period deadline (measured against
     the wall clock, not budgeted), the loop **yields to the CODAL cooperative
     scheduler** (`uBit.sleep(1)`) and then drains best-effort slack: ingest
     comms, then route a statement if one is pending, else apply one
     Configurator delta if one is pending.
  3. If the mandatory portion overruns the period, the pass simply proceeds
     with no slack that pass; control keeps running, config waits.
- **Postconditions**: A config-write burst never delays a motion command's
  execution; a motion command posted this pass's slack always executes on
  the very next mandatory pass; the radio transport keeps receiving datagrams
  (the yield gives CODAL's `MessageBus` a fiber slice to run `Radio::onData`
  on every pass, not just when the loop happens to idle).
- **Acceptance Criteria**:
  - [ ] A 25-`SET`-then-motion burst executes the motion command on the next
        mandatory pass regardless of how many `SET`s preceded it.
  - [ ] The target period (20 ms / 50 Hz) is treated as best-effort pacing —
        an overrun degrades gracefully (slower cadence) rather than
        violating a hard deadline or dropping the mandatory work.
  - [ ] **The hardware-bench gate round-trips a command over the radio relay
        specifically, not only serial** — a command sent over serial alone
        cannot catch a missing slack-loop yield, since serial RX is
        IRQ-driven and needs no scheduler slice while radio RX (a CODAL
        `MessageBus` event listener) does; this criterion is the concrete
        acceptance bar for whichever ticket rewrites the loop.

## SUC-006: Command handlers are pure translators with zero subsystem pointers

- **Actor**: A firmware developer reading or modifying a command-family
  handler (dev, telemetry, motion, config, pose, otos).
- **Preconditions**: A command-family handler needs to read state or post a
  command.
- **Main Flow**:
  1. The handler reads whatever state objects it needs (motor/drivetrain/pose/
     planner/config state cells) directly from the blackboard's state plane.
  2. To act, it posts a typed command onto the appropriate queue
     (`driveIn`, `motorIn[i]`, `configIn`, `poseResetIn`, the per-motor reset
     flags, or `otosSetPoseIn`).
  3. It never stores, receives, or dereferences a `Subsystems::*` pointer.
- **Postconditions**: The six `*State` structs (`DevLoopState`,
  `TelemetryState`, `MotionLoopState`, `ConfigCommandState`, `PoseCommandState`,
  `OtosCommandState`) plus the seventh holder (`DevLoop`) no longer exist in
  their pointer-holding form; command-family headers no longer `#include` the
  subsystem layer.
- **Acceptance Criteria**:
  - [ ] Grepping `source/commands/` for `Subsystems::` (outside comments)
        returns nothing.
  - [ ] Every command family compiles and links without including any
        `subsystems/*.h` header.
  - [ ] `tests/_infra/sim/sim_api.cpp` mirrors the same router/blackboard/
        Configurator wiring as `source/main.cpp` (both wiring sites change
        identically, per the design's own grounding).
