---
id: "006"
title: "Real Hal::Odometer (OTOS) leaf + NezhaHardware wiring"
status: open
use-cases: [SUC-005, SUC-006, SUC-007]
depends-on: ["005"]
github-issue: ""
issue: nezha-hardware-otos-driver-for-new-source-tree.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Real Hal::Odometer (OTOS) leaf + NezhaHardware wiring

## Description

Implement the real `Hal::Odometer` leaf for the SparkFun OTOS sensor and
wire it into `Subsystems::NezhaHardware`. Depends on ticket 005 (lever-arm
math + boot-config surface).

**Grounding (architecture-update.md facts 4/5) — read before starting:**
- `Subsystems::Hardware::odometer()` defaults to `nullptr`; `NezhaHardware`
  does not override it today. `source/dev_loop.cpp`'s pose-estimation step
  ALREADY calls `hardware.odometer()` and, generically, for any non-null
  result, calls its `tick(now)`/`pose()` every pass before
  `PoseEstimator::tick()` runs — unconditionally, with no per-owner special
  casing. `source/commands/otos_commands.cpp` ALREADY resolves
  `hardware.odometer()` live on every one of the seven OTOS verb dispatches
  and already replies `ERR nodev` gracefully when null. **Do not modify
  `dev_loop.cpp` or `otos_commands.{h,cpp}` — they need zero changes.**
- The new leaf's `tick()` is NOT part of `NezhaHardware`'s brick flip-flop
  motor scheduler (`REQUEST_DUE`/`COLLECT_DUE`, address `0x10`) — the OTOS
  chip is a different address (`0x17`) and is driven by `dev_loop.cpp`'s own
  separate per-pass call. Do not fold OTOS scheduling into the flip-flop
  phase state machine.
- Working directory: `source/hal/otos/` (a new top-level HAL device
  directory, parallel to `source/hal/nezha/`, `source/hal/sim/`,
  `source/hal/capability/` — NOT nested under `hal/nezha/`, since the OTOS
  sensor is not a Nezha-brand device; it just happens to be orchestrated by
  the same `NezhaHardware` owner in this single-hardware-owner tree).
- Reuse `I2CBus`'s existing per-device `preClear`/`postClear` lazy-clearance
  mechanism (already generic over any 7-bit address) for the leaf's own
  register writes/reads — do not invent a second bus-safety mechanism, and
  do not become a new source of bus contention (issue 3's non-goal applies
  here too, even though this is a different issue).
- Port the register map / read sequencing from `source_old/hal/real/
  OtosSensor.{h,cpp}` (product ID detect, `init()`, `resetTracking()`,
  position/velocity burst reads, linear/angular scalar registers) —
  conforming to this tree's naming/coding standards (CamelCase, no units in
  identifiers), not copied verbatim syntax.

## Acceptance Criteria

- [ ] A new leaf (working name `Hal::OtosOdometer`, `source/hal/otos/
      otos_odometer.{h,cpp}`) implements all five `Hal::Odometer` primitives
      (`init()`, `resetTracking()`, `setPose()`, `setLinearScalar()`,
      `setAngularScalar()`) plus `pose()`/`connected()`/`tick()`/`begin()`.
- [ ] `pose()` applies the lever-arm compensation (ticket 005's math) using
      the same-instant heading from the same read burst — not a lagged one.
- [ ] The leaf is constructed with ticket 005's boot-config values (offset,
      linear/angular scalar) — no new live `SET`/wire surface.
- [ ] `Subsystems::NezhaHardware` gains one new member (the leaf instance)
      and overrides `odometer()` to return its address — the flip-flop
      scheduler (`tick()`'s `REQUEST_DUE`/`COLLECT_DUE`) is untouched.
- [ ] `source/main.cpp` constructs the new leaf alongside existing hardware
      construction, wired with ticket 005's boot-config values.
- [ ] `source/dev_loop.cpp` and `source/commands/otos_commands.{h,cpp}` are
      confirmed UNCHANGED (diff shows zero lines touched in either file).
- [ ] Unit tests exercise the leaf's register sequencing against a scripted
      `I2CBus` fake (mirroring `NezhaMotor`'s own existing test precedent),
      without requiring real hardware.
- [ ] `Hal::Odometer`'s public interface is unchanged (no new virtual, no
      signature change) — the leaf conforms to the existing five-primitive
      contract.

## Implementation Plan

**Approach**: Read `source_old/hal/real/OtosSensor.{h,cpp}` for the register
map and read/write sequencing, and `source/hal/sim/sim_odometer.{h,cpp}`
(081-003) for this tree's `Hal::Odometer` leaf conventions (constructor
shape, how `pose()`/`tick()` are structured). Build the leaf host-testable
first (scripted `I2CBus` fake, same pattern `NezhaMotor`'s own tests use),
then wire it into `NezhaHardware`/`main.cpp`.

**Files to create/modify**:
- `source/hal/otos/otos_odometer.{h,cpp}` (new).
- `source/subsystems/nezha_hardware.{h,cpp}` — new member + `odometer()`
  override.
- `source/main.cpp` — construct the new leaf with ticket 005's boot-config
  values.

**Testing plan**:
- New unit test file exercising the leaf's register protocol against a
  scripted `I2CBus` (product-ID detect, init sequencing, position/velocity
  burst read + lever-arm-corrected `pose()`, linear/angular scalar
  set/read-back).
- Confirm `dev_loop.cpp`/`otos_commands.{h,cpp}` are untouched via `git
  diff` (this is an explicit acceptance criterion, not just an aspiration).
- Full existing `tests/sim/unit/` suite re-run to confirm no regression
  (this ticket adds a new leaf; it should not change any existing sim
  behavior since `Hal::SimOdometer` is a separate, untouched leaf).

**Documentation updates**: None required at the wire/protocol level (the
seven OTOS verbs already document this behavior in `docs/protocol-v2.md`
§11 — once this leaf is live, no prose there needs to change, since it
already describes the intended behavior, only previously unreachable on
real hardware).
