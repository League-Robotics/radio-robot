---
id: '002'
title: Rt::Blackboard state and command planes
status: exception
use-cases:
- SUC-001
- SUC-006
depends-on:
- '001'
github-issue: ''
issue: plan-file-a-design-issue-blackboard-architecture-state-objects-command-queues.md
completes_issue: true
exception:
  thrown_by: programmer
  thrown_at: '2026-07-07T06:39:13.381421+00:00'
  attempted: 'Read architecture-update.md''s full Reference code block for source/runtime/blackboard.h
    and ticket 002''s acceptance criteria. Confirmed every msg:: type name is real
    (MotorState, DrivetrainState, PoseEstimate, PlannerState, DrivetrainConfig, MotorConfig,
    PlannerConfig, OdometerConfig, DrivetrainCommand, MotorCommand, SetPose all exist
    verbatim in source/messages/*.h) and that Subsystems::Hardware::kPortCount==4
    is reachable via subsystems/hardware.h alone. Then traced the one remaining named
    type, Subsystems::CommunicatorToCommandProcessorStatement (statementsIn''s WorkQueue<T,16>
    payload): it is defined only in source/subsystems/communicator.h, which #includes
    MicroBit.h/com/radio.h/com/serial_port.h (real CODAL vendor headers, no HOST_BUILD
    guard anywhere in that file). Rt::WorkQueue<T,N> (ticket 001, already merged)
    stores T buf_[N] as a fixed array member, so T must be a complete type wherever
    Rt::Blackboard is defined -- there is no forward-declare escape available. Checked
    tests/_infra/sim/CMakeLists.txt, which explicitly documents ''subsystems/communicator.*
    -- serial+radio comms (MicroBit.h, CODAL-only)'' as excluded from the host-testable
    sim tier, and grepped tests/sim/unit/*.cpp confirming no existing harness includes
    subsystems/communicator.h. So building statementsIn exactly as specified would
    require blackboard.h to include subsystems/communicator.h, which (a) violates
    ticket 002''s own Acceptance Criterion 1 (''the header includes only messages/*.h,
    runtime/queue.h, and subsystems/hardware.h''), (b) drags MicroBit.h into the header,
    making the ticket''s own required test (''instantiate a Rt::Blackboard'' in a
    host-compiled harness mirroring runtime_queue_harness.cpp''s explicit no-MicroBit.h
    convention) impossible without an ARM toolchain, and (c) crosses the project''s
    established CODAL-only/host-build boundary that every other sim/unit harness respects.
    I did not modify source/subsystems/communicator.h to add a HOST_BUILD guard (the
    ticket''s Implementation Plan lists no files to modify besides creating blackboard.h),
    and did not silently substitute a different payload type for statementsIn, since
    either move overrides an upstream decision rather than following one.'
  conflict: 'architecture-update.md''s Reference code for source/runtime/blackboard.h
    (`WorkQueue<Subsystems::CommunicatorToCommandProcessorStatement, 16> statementsIn;`)
    is structurally incompatible with ticket 002''s own Acceptance Criterion 1 (''the
    header includes only messages/*.h, runtime/queue.h, and subsystems/hardware.h'')
    and with the project''s host/CODAL build boundary (tests/_infra/sim/CMakeLists.txt:
    ''subsystems/communicator.* -- serial+radio comms (MicroBit.h, CODAL-only)'',
    excluded from the sim tier every other tests/sim/unit harness respects). Resolving
    this requires an upstream decision: either (1) add subsystems/communicator.h (and
    therefore MicroBit.h) as a permitted include and add a HOST_BUILD guard to communicator.h
    so the required host harness can still compile (out of this ticket''s stated file-modify
    scope, and itself a small design decision about where that guard lives), accepting
    Rt::Blackboard now depends on a CODAL-coupled header; or (2) change statementsIn''s
    payload type in the architecture to a new host-safe Rt:: or msg:: statement type
    not yet named anywhere in the Reference code -- an architecture-level change,
    not a ticket-level implementation detail.'
  surface: internal
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Rt::Blackboard state and command planes

**Revised per `architecture-update-r1.md` Decision 10**, resolving this
ticket's own `INTERNAL` exception (see the `exception:` frontmatter block
above for the original report). Scope now includes extracting a host-safe
statement POD as a prerequisite — see the Description and Implementation
Plan below.

## Description

Implement `Rt::Blackboard` exactly as specified in
`architecture-update-r1.md`'s Reference code — the aggregate struct owning
every state-plane cell (`motor[kPortCount]`, `drivetrain`, `encoderPose`,
`fusedPose`, `planner`, `otos`/`otosValid`, and the four current-config
cells) and every command-plane queue instance (`statementsIn`, `driveIn`,
`motorIn[kPortCount]`, `configIn`, `poseResetIn`, `motorResetIn[kPortCount]`,
`otosSetPoseIn`). This is pure data — no method computes anything; it holds
**no subsystem pointer of any kind** (SUC-006). Also define the two payload
types the Blackboard's queues carry that don't already exist as `msg::`
types: `Rt::PoseResetCommand` and `Rt::ConfigDelta`.

**Prerequisite folded into this ticket (Decision 10).** `statementsIn`'s
payload, `Subsystems::CommunicatorToCommandProcessorStatement`, is today
defined inside `source/subsystems/communicator.h`, which pulls in
`MicroBit.h`/`com/radio.h`/`com/serial_port.h` with no `HOST_BUILD` guard —
incompatible with a host-instantiable `Rt::Blackboard` (`Rt::WorkQueue<T,N>`
stores `T` by value, so `T` must be a complete, host-compilable type
wherever `Blackboard` is defined). Before writing `blackboard.h`:

1. Create `source/subsystems/statement.h` — a new, CODAL-free header
   (`<cstdint>` only) containing the `Channel` enum and the
   `CommunicatorToCommandProcessorStatement` struct, moved out of
   `communicator.h` verbatim in name/shape, **except** `line` changes from
   an aliasing `const char* line` (into `Communicator`'s internal buffer) to
   an **owned** `char line[256]` — see Decision 10's rationale (a value
   stored in a 16-deep `WorkQueue` must not alias mutable state a later
   `Communicator::tick()` can overwrite out from under an unread queued
   entry).
2. Update `source/subsystems/communicator.h` to `#include
   "subsystems/statement.h"` instead of defining those two types inline; no
   other change to `Communicator`'s public faceplate.
3. Update `source/subsystems/communicator.cpp`'s `takeStatement()` to copy
   the held line into the returned struct's owned buffer (instead of
   returning an aliasing pointer).
4. Then build `blackboard.h`, including `subsystems/statement.h` (not
   `subsystems/communicator.h`) for `statementsIn`'s payload type.

See `architecture-update-r1.md`'s "The host-safe statement type" Reference
code subsection for the exact header content.

## Acceptance Criteria

- [ ] `source/subsystems/statement.h` exists, is CODAL-free (only
      `<cstdint>`), and defines `Subsystems::Channel` and
      `Subsystems::CommunicatorToCommandProcessorStatement` (with an owned
      `char line[256]`, not an aliasing pointer).
- [ ] `source/subsystems/communicator.h` includes `subsystems/statement.h`
      and no longer defines `Channel`/`CommunicatorToCommandProcessorStatement`
      inline; `communicator.cpp`'s `takeStatement()` copies into the owned
      buffer. `Communicator`'s public faceplate (`configure()`, `begin()`,
      `tick()`, `hasStatement()`, `takeStatement()`, `state()`,
      `capabilities()`, `sendSerial()`, `sendRadio()`) is unchanged in
      signature.
- [ ] `Rt::Blackboard` compiles and default-constructs with every cell
      zero/default-initialized; the header includes only `messages/*.h`,
      `runtime/queue.h`, `subsystems/hardware.h` (for the `kPortCount`
      constant only), and **`subsystems/statement.h`** (for `statementsIn`'s
      payload type) — per the Reference code in `architecture-update-r1.md`.
- [ ] Every state cell listed in `architecture-update-r1.md`'s Reference code
      is present with the exact `msg::` type named there.
- [ ] Every command-plane queue is present with the exact vehicle
      (`Mailbox` vs. `WorkQueue`) and capacity named there: `statementsIn`
      (`WorkQueue`, 16), `configIn` (`WorkQueue`, 16), `poseResetIn`
      (`WorkQueue`, 4), `driveIn`/`motorIn[i]`/`otosSetPoseIn` (`Mailbox`,
      capacity 1).
- [ ] `Rt::PoseResetCommand` (`kind` enum `{kSetPose, kResetBaseline}` +
      `msg::SetPose pose`) and `Rt::ConfigDelta` (`target` enum
      `{kDrivetrain, kMotor, kPlanner, kOdometer}` + `port` + a field-mask
      placeholder) are defined in `source/runtime/blackboard.h` exactly as
      specified.
- [ ] Grepping `source/runtime/blackboard.h` for any `Subsystems::` type
      used as a pointer/reference member (as opposed to the `kPortCount`
      constant reference and the `statementsIn` payload's type name) returns
      nothing — i.e., `Subsystems::CommunicatorToCommandProcessorStatement`
      appearing as a `WorkQueue` template argument is expected and fine; a
      `Subsystems::*` pointer or reference member would not be.
- [ ] `tests/sim/unit/runtime_blackboard_harness.cpp` (and any harness that
      includes `blackboard.h`) compiles and links with the plain host C++
      compiler, no ARM toolchain, no `MicroBit.h` transitively included —
      confirmed by the harness's own build command carrying no CODAL
      include paths.

## Implementation Plan

**Approach.** First extract the host-safe statement type (steps 1-3 in the
Description above), then write `source/runtime/blackboard.h`, namespace
`Rt`, built directly from `architecture-update-r1.md`'s Reference code
block. No `.cpp` for the Blackboard itself — pure aggregate, no logic.

**Files to create:**
- `source/subsystems/statement.h` (new — the extracted host-safe `Channel` +
  `CommunicatorToCommandProcessorStatement`)
- `source/runtime/blackboard.h`

**Files to modify:**
- `source/subsystems/communicator.h` (swap the inline `Channel`/
  `CommunicatorToCommandProcessorStatement` definitions for `#include
  "subsystems/statement.h"`)
- `source/subsystems/communicator.cpp` (`takeStatement()` copies into the
  owned buffer instead of returning an aliasing pointer)

**Testing plan:**
- New `tests/sim/unit/statement_harness.cpp` (or fold into the blackboard
  harness) confirming `source/subsystems/statement.h` compiles standalone
  with zero CODAL includes.
- New `tests/sim/unit/runtime_blackboard_harness.cpp` — instantiate a
  `Rt::Blackboard`, exercise a representative post/take round-trip on
  `driveIn`, `configIn`, `poseResetIn`, `motorIn[0]`, and `statementsIn`
  (post/take a `CommunicatorToCommandProcessorStatement` with a known `line`
  and `returnPath`, confirm it round-trips by value with no aliasing), and
  confirm the state cells default-construct to zero/default `msg::` values.
- Update/add any existing `Communicator` test (e.g. wherever
  `takeStatement()`'s current aliasing-pointer contract is exercised) to
  assert the new owned-copy behavior instead.
- New `tests/sim/unit/test_runtime_blackboard.py` driving the harness.
- **Verification command**: `uv run pytest tests/sim/unit/test_runtime_blackboard.py`

**Documentation updates:** none beyond `architecture-update-r1.md` (already
written).
