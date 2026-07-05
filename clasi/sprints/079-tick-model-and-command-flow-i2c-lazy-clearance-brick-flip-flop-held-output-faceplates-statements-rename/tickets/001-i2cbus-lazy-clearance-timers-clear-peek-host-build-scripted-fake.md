---
id: '001'
title: I2CBus lazy clearance timers, clear() peek, HOST_BUILD scripted fake
status: open
use-cases: [SUC-008, SUC-009]
depends-on: []
github-issue: ''
issue: i2c-bus-lazy-clearance-timers.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# I2CBus lazy clearance timers, clear() peek, HOST_BUILD scripted fake

## Description

Move the encoder-path busy-waits out of the callers and into `I2CBus` itself,
as lazy, per-device deadlines. This is the substrate ticket 004's flip-flop
scheduler builds on, and the first ticket to make a real, dependency-free
`HOST_BUILD` `I2CBus` exist (078 deferred this to 079 — see its Design
Rationale 9 / Open Question 4).

Per `architecture-update.md`'s "The I2CBus lazy-clearance mechanism — exact
contract" section:

- `DeviceSlot` (in `source/com/i2c_bus.h`) gains `uint64_t lastEnd`/`readyAt`
  `// [us]` fields.
- `write()`/`read()` gain two new, defaulted `// [us]` parameters:
  `preClear = 0`, `postClear = 0`. At entry — **before** the re-entrancy
  guard's `target_disable_irq()` critical section — spin until
  `max(slot.readyAt, slot.lastEnd + preClear)`. After the transaction
  completes, set `lastEnd = now()`, `readyAt = lastEnd + postClear`.
- A new non-spinning peek: `bool clear(uint16_t addr7) const` — takes the
  **7-bit** device address (matching `txnCount()`/`errCount()`/`lastErr()`'s
  existing convention), **not** the 8-bit wire address `write()`/`read()`
  take. This is an easy off-by-one-bit trap — the test plan below asserts it
  explicitly.
- Every existing call site (`writeMotorRun()`, `readVersion()`,
  `setGlobalSpeed()`, `resetHome()`, `timedMove()`) keeps its 4-argument
  call — defaults leave behavior byte-identical.
- Three `nezha_motor.cpp` call sites change to use the new parameters instead
  of hand-rolled busy-waits:
  - `requestEncoder()`'s 0x46 write passes `postClear = 4000`.
  - `readEncoderAtomicRaw()`'s write/read pair passes `preClear = 4000`/
    `postClear = 4000` on the write, replacing its two manual
    `deadline = now + 4000; while (now < deadline) {}` loops.
  - `writePositionMove()`'s write passes `postClear = 4000`, replacing its
    trailing manual spin.
  - `collectEncoder()`'s read needs **no** parameter change — ticket 004's
    HAL scheduler will have already confirmed `bus_.clear()` before calling
    it, so the entry spin fires immediately (free).
- A new `source/com/i2c_bus_host.cpp` (compiled only under `HOST_BUILD`,
  never linked alongside the real `i2c_bus.cpp`) implements the same public
  `I2CBus` surface against a scripted transaction queue instead of
  `MicroBitI2C`, backed by an injectable/steppable clock (not a real wall
  clock) so tests can assert clearance-timer behavior deterministically
  without real sleeps. The clock-injection mechanism (function pointer vs. a
  static test-settable counter) is an implementation choice — pick whichever
  is simplest to wire into a CMake `HOST_BUILD` target.

**Do not** touch `NezhaMotor::tick()`'s call sequence in this ticket — that
is ticket 004's job (wiring `requestSample()`/`collectEncoder()` into the
flip-flop). This ticket only changes `I2CBus` and the three call sites named
above (which keep calling `readEncoderAtomicRaw()`/`writePositionMove()`
exactly as before, just without their own manual spins).

## Acceptance Criteria

- [ ] `I2CBus::DeviceSlot` has `lastEnd`/`readyAt` (`uint64_t`, `// [us]`).
- [ ] `write()`/`read()` accept `preClear`/`postClear` defaulted to 0; every
      existing call site (grep confirms) is unchanged and still compiles.
- [ ] The entry-side clearance spin happens **before**
      `target_disable_irq()` (verify by reading the diff, not just tests —
      this is the IRQ-guard-window constraint from the source issue).
- [ ] `bool I2CBus::clear(uint16_t addr7) const` exists, is non-spinning, and
      takes the bare 7-bit address.
- [ ] `requestEncoder()` passes `postClear = 4000` on its 0x46 write.
- [ ] `readEncoderAtomicRaw()`'s two manual spin loops are deleted, replaced
      by `preClear = 4000`/`postClear = 4000` on its write call.
- [ ] `writePositionMove()`'s trailing manual spin is deleted, replaced by
      `postClear = 4000` on its write call.
- [ ] `source/com/i2c_bus_host.cpp` exists, builds under `HOST_BUILD`, and
      implements `write`/`read`/`clear`/`txnCount`/`errCount`/`lastErr` with
      a scripted queue + injectable clock.
- [ ] Both `ROBOT_DEV_BUILD` forks still build (`just build`).
- [ ] Host tests (new, see Testing) pass under `uv run python -m pytest`.

## Implementation Plan

**Approach**: extend `I2CBus` additively first (device build, `just build`
green), then add the `HOST_BUILD` host file and its test harness, then
migrate the three `nezha_motor.cpp` call sites last (smallest, most
mechanical step, easy to review against the "byte-identical for existing
call sites" claim).

**Files to modify**:
- `source/com/i2c_bus.h` — `DeviceSlot` fields, `write`/`read` signatures,
  `clear()` declaration.
- `source/com/i2c_bus.cpp` — entry-spin + `lastEnd`/`readyAt` bookkeeping,
  `clear()` implementation.
- `source/hal/nezha/nezha_motor.cpp` — the three call-site changes
  (`requestEncoder()`, `readEncoderAtomicRaw()`, `writePositionMove()`); do
  **not** touch `tick()`'s call sequence.

**Files to create**:
- `source/com/i2c_bus_host.cpp` — `HOST_BUILD` scripted fake.
- Build wiring so the host target compiles this instead of `i2c_bus.cpp`
  (check `justfile`/CMake for the existing `HOST_BUILD` pattern used by
  other host-buildable files before inventing a new one).
- `tests/sim/unit/` (or wherever ticket 078's `MockMotor` harness lives) —
  a new host test file exercising the scripted fake directly: script a
  write with `postClear=4000`, assert `clear()` is false immediately after
  and true once the injected clock advances past the deadline; assert
  `clear(0x10)` and `clear(0x20)` (the shifted, wrong-bit value) behave
  differently to guard the 7-bit-vs-8-bit convention explicitly.

**Testing plan**:
- Existing tests: `uv run python -m pytest` (host suite), `just build`
  (both `ROBOT_DEV_BUILD` forks).
- New tests: the scripted-fake clearance-timer test above; a defaults test
  confirming `write()`/`read()` called with no `preClear`/`postClear` behave
  identically to today (no spin, immediate return status).
- No stand verification in this ticket — ticket 006 covers the hardware
  A/B gate. This ticket is host-test-only.

**Documentation updates**: none required this ticket (the architecture
document already carries the full design); a one-line comment at each
touched call site pointing at `architecture-update.md`'s clearance-timer
section is good practice but not mandatory.
