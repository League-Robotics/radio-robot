---
id: '004'
title: NezhaHal brick flip-flop and distribution; NezhaMotor split-phase wiring
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-008
- SUC-009
depends-on:
- '001'
- '003'
github-issue: ''
issue:
- i2c-bus-lazy-clearance-timers.md
- tick-model-command-flow-and-the-command-board-design-sketch.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# NezhaHal brick flip-flop and distribution; NezhaMotor split-phase wiring

## Description

Wire the brick flip-flop into `NezhaHal` and the split-phase encoder read
into `NezhaMotor`, and give `NezhaHal` its distribution role (decision 3).
This is the ticket that actually removes the ~8 ms/tick blocking spin and
implements in-use port tracking. Depends on ticket 001 (`I2CBus::clear()`/
lazy clearance) and ticket 003 (`Hal::CommandProcessorToHalCommand`/
`DrivetrainToHalCommand`, `Drivetrain::ports()` for the wiring these
`apply()` overloads consume).

**`NezhaMotor` split-phase wiring** (`source/hal/nezha/nezha_motor.{h,cpp}`):
- New public method `void requestSample();` — wraps the existing (ported,
  previously unwired) `requestEncoder()`. Not a `Hal::Motor` virtual — Nezha-
  specific, called only by `NezhaHal`.
- `requestEncoder()`'s 0x46 write already carries `postClear=4000` (ticket
  001) — no further change needed there.
- `collectEncoder()` gains the line it is missing today:
  `connected_ = pendingEncRequestOk_ && (readResult == MICROBIT_OK);`
  (confirmed via code read: it does not set `connected_` today — dead-code-
  safe only because nothing calls it yet).
- `tick()`'s step 2 (the 078 base-class 5-step contract's ONLY changed
  step) becomes:
  ```cpp
  int32_t raw = collectEncoder();
  float pos = (static_cast<float>(raw) / 10.0f)
            * config_.travel_calib * static_cast<float>(config_.fwd_sign);
  ```
  replacing `float pos = readEncoderSettle();`. Steps 1/3/4/5 (
  `processResetIfPending`, `updateWedgeDetector`, mode dispatch/
  `armoredWrite`, `updateRestTracking`) are **untouched, byte-for-byte** —
  do not reorder or modify them in this ticket.
- `readEncoderSettle()` is **deleted** (its only caller is gone).

**`NezhaHal` brick flip-flop + distribution** (`source/hal/nezha/
nezha_hal.{h,cpp}`):
- New: `enum class Phase : uint8_t { REQUEST_DUE, COLLECT_DUE };`,
  `uint32_t activePort_`, `Phase phase_`, `bool portInUse_[kPortCount]`,
  `I2CBus& bus_` (NezhaHal keeps its own reference now, alongside the four
  `NezhaMotor` members it already owns).
- `tick(uint32_t now)` becomes the flip-flop sequencer — implement exactly
  per `architecture-update.md`'s "The flip-flop and the 078 base-class
  contract" code block (idle-schedule check, defensive resync,
  REQUEST_DUE/COLLECT_DUE switch, `bus_.clear(kNezhaDeviceAddr)` gate).
  `kNezhaDeviceAddr` is a `namespace Hal` constant (`0x10`), promoted from
  `NezhaMotor::kAddr` (private) — shared by both classes.
- New private `NezhaMotor& motorAt(uint32_t port)` (internal scheduling/
  distribution use; the public `Motor& motor(uint32_t port)` accessor is
  unchanged for external read/query callers).
- New private `uint32_t nextPortInUse(uint32_t cur) const` and
  `bool anyPortInUse() const`.
- Two new `apply()` overloads:
  `void apply(const CommandProcessorToHalCommand&)` and
  `void apply(const DrivetrainToHalCommand&)` — implement exactly per the
  architecture doc's code block. Broadcast (`allPorts==true`) does **not**
  mark any port in-use (Design Rationale 5); addressed calls (both
  overloads) do.

## Acceptance Criteria

- [x] `NezhaMotor::requestSample()` exists (public, wraps `requestEncoder()`).
- [x] `collectEncoder()` sets `connected_` correctly (both halves ANDed).
- [x] `NezhaMotor::tick()`'s step 2 uses `collectEncoder()`, not
      `readEncoderSettle()`; `readEncoderSettle()` is deleted; steps 1/3/4/5
      are unchanged (diff review confirms no reordering).
- [x] `NezhaHal::tick()` implements the REQUEST_DUE/COLLECT_DUE flip-flop;
      only in-use ports are cycled; idle schedule (`anyPortInUse()==false`)
      performs zero bus actions.
- [x] `NezhaHal::apply(CommandProcessorToHalCommand)` and
      `apply(DrivetrainToHalCommand)` mark the correct port(s) in-use
      (addressed) or none (broadcast), and forward to the right motor(s).
- [x] `bus_.clear(kNezhaDeviceAddr)` is called with the **bare** `0x10`
      (7-bit), not `(0x10 << 1)` — host test asserts this explicitly
      (regression guard against the off-by-one-bit trap from ticket 001).
- [x] Host tests (against ticket 001's `HOST_BUILD` scripted `I2CBus` fake)
      cover: flip-flop sequencing (request → settle-not-elapsed pass →
      collect), the 40 ms write throttle interaction, dwell interaction
      (078's armor still holds through a scripted reversal), in-use
      tracking (an unaddressed port never gets a bus transaction scripted
      against it), and the `apply()` broadcast-vs-addressed distinction.
- [x] Both `ROBOT_DEV_BUILD` forks build (`just build`).
- [x] No stand pass required in this ticket (ticket 006 covers hardware).

## Implementation Notes (post-hoc)

- `main.cpp` also received a minimal loop adaptation beyond this ticket's
  own "Files to modify" list, per explicit team-lead direction at dispatch
  time: `hal.apply(drivetrain.takeCommand())` now drains the Drivetrain's
  held output (closing the gap ticket 079-003 left open), and the old
  explicit bound-pair re-tick is replaced by the sanctioned second
  `hal.tick(now)` call per pass (architecture-update.md decision 6). The
  full three-beat loop / `DevLoopState` outbox reshape remains ticket 005's
  job — comms dispatch position and DEV-handler wiring are untouched here.
- Host coverage lives in `tests/sim/unit/nezha_flipflop_harness.cpp` (7
  scenarios) + `tests/sim/unit/test_nezha_flipflop.py`, compiling the REAL
  `nezha_motor.cpp`/`nezha_hal.cpp` under `-DHOST_BUILD` against ticket
  001's scripted `I2CBus` fake. `nezha_motor.cpp` gained an `#ifndef
  HOST_BUILD` guard around its `MicroBit.h` include (mirroring
  `i2c_bus.h`'s own pattern) so it can be compiled host-side at all; the
  `HOST_BUILD` fork defines `MICROBIT_OK` and a `system_timer_current_time_us()`
  shim that delegates to `I2CBus::clock()`, the same fake clock the
  scripted bus itself runs against.

## Implementation Plan

**Approach**: `NezhaMotor`'s split-phase wiring first (smaller, mostly
mechanical given ticket 001 already did the `I2CBus` groundwork), then
`NezhaHal`'s flip-flop + distribution (the larger piece), verified together
against the ticket 001 scripted fake before touching anything upstream.

**Files to modify**:
- `source/hal/nezha/nezha_motor.h` — `requestSample()` declaration; promote
  `kAddr` to a shared `namespace Hal` constant (`kNezhaDeviceAddr`); delete
  `readEncoderSettle()`'s declaration.
- `source/hal/nezha/nezha_motor.cpp` — `requestSample()` body,
  `collectEncoder()` fix, `tick()` step 2, delete `readEncoderSettle()`.
- `source/hal/nezha/nezha_hal.h` — `Phase`, `activePort_`, `phase_`,
  `portInUse_[]`, `bus_`, `motorAt()`, `nextPortInUse()`,
  `anyPortInUse()`, two `apply()` overloads; `#include
  "hal/capability/hal_command.h"`.
- `source/hal/nezha/nezha_hal.cpp` — constructor (store `bus_`),
  `tick()` flip-flop body, `apply()` bodies.

**Testing plan**:
- Existing tests: `uv run python -m pytest`; `just build` both forks.
- New tests: host-level, against ticket 001's `HOST_BUILD` `I2CBus`
  scripted fake, exercising the **real** `NezhaMotor`/`NezhaHal` (per the
  design sketch's "subsystem is the unit of test" principle — this is the
  confined, sanctioned hardware-fake exception): script a request/settle/
  collect sequence and assert timing; script two in-use ports and confirm
  even cycling; script an idle HAL and confirm zero transactions; script a
  reversal mid-cycle and confirm 078's dwell still suppresses the write;
  assert the `clear(0x10)` vs `clear(0x20)` 7-bit-address distinction.
- No hardware in this ticket — ticket 006 is the stand gate.

**Documentation updates**: none required (architecture doc covers the
design); update `nezha_hal.h`'s existing class-comment ("no left/right
pairing... NezhaHal only knows about ports") if the new distribution role
makes any part of it stale — read it first and correct only what's now
inaccurate, don't rewrite wholesale.
