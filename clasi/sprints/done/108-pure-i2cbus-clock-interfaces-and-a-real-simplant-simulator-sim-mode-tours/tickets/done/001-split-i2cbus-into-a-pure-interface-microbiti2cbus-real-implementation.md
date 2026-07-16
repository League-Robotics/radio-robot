---
id: '001'
title: Split I2CBus into a pure interface + MicroBitI2CBus real implementation
status: done
use-cases:
- SUC-038
depends-on: []
github-issue: ''
issue: plan-pure-i2cbus-clock-interfaces-a-real-simplant-simulator.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Split I2CBus into a pure interface + MicroBitI2CBus real implementation

## Description

Stage 1 of the master plan (`clasi/issues/plan-pure-i2cbus-clock-interfaces-
a-real-simplant-simulator.md`). This is a **behavior-preserving** firmware
refactor: `source/devices/i2c_bus.h` today is a concrete class with two
`#ifdef HOST_BUILD` forks (real MicroBitI2C machinery vs. a scripted-FIFO
test fake), plus private data members and a `HOST_BUILD` scripted-fake
surface baked into the production header.

Reduce `I2CBus` to a pure abstract interface and move the real machinery
into a new concrete class:

1. `source/devices/i2c_bus.h`: reduce to `class I2CBus` with virtual dtor +
   pure-virtual `write()`, `read()`, `clearanceSafetyNetCount()` — the
   exact 3 methods any command handler or device leaf calls today (grep-
   confirmed: no other public member of the current `I2CBus` — `txnCount`,
   `errCount`, `lastErr`, `clear`, `reentryViolations`, `dumpRecent`,
   `setLogging`, `setIrqGuard`, `irqGuard`, `resetStats` — is called from
   outside `i2c_bus.h`/`.cpp` themselves). Delete every `#ifdef` fork, the
   scripted surface (`scriptWrite`/`scriptRead`/`setClock`/`advanceClock`/
   `clock()`), the `Responder` seam, and all private members.
2. New `source/devices/microbit_i2c_bus.{h,cpp}`: `class MicroBitI2CBus :
   public Devices::I2CBus` holding the CURRENT real machinery verbatim —
   `MicroBitI2C&` member, re-entrancy guard, lazy preClear/postClear
   clearance timers, per-device stats (`txnCount`/`errCount`/`lastErr`/
   `clear`), the transaction ring log (`dumpRecent`/`setLogging`), the IRQ
   guard (`setIrqGuard`/`irqGuard`), `resetStats()`. Move `i2c_bus.cpp`'s
   body here verbatim (re-cased/re-namespaced only as needed to compile
   against the new header split — no behavior change). Delete
   `source/devices/i2c_bus.cpp` and `source/devices/i2c_bus_host.cpp`.
3. `source/main.cpp:99`: `static Devices::I2CBus bus(uBit.i2c);` becomes
   `static Devices::MicroBitI2CBus bus(uBit.i2c);` (the `I2CBus&` reference
   every device leaf already holds is unaffected — `MicroBitI2CBus` IS-A
   `I2CBus`).
4. `CMakeLists.txt:300`: drop the
   `list(FILTER SOURCE_FILES EXCLUDE REGEX ".*/devices/i2c_bus_host\\.cpp$")`
   line (the file it excludes no longer exists). Leave the `clock_host.cpp`
   exclude line for ticket 010.

This ticket does NOT touch `source/devices/clock.h` (ticket 010's job) and
does NOT create any new sim/host implementation (ticket 002's job) — it
only makes the ARM-side split clean. The 13 register-scripting sim-unit
tests (`tests/sim/unit/devices_i2c_bus_harness.cpp`,
`tests/sim/unit/test_devices_i2c_bus.py`, and any other test that used
`I2CBus::scriptWrite`/`scriptRead`/`setClock`) will go RED once the
scripted surface is deleted — this is EXPECTED (see sprint.md Test
Strategy and architecture-update.md's Migration Concerns); do not chase
them green this ticket. They come back green in ticket 009.

## Acceptance Criteria

- [x] `source/devices/i2c_bus.h` declares only `write()`, `read()`,
      `clearanceSafetyNetCount()` (pure virtual) + virtual dtor. No data
      members, no `#ifdef HOST_BUILD`/`#ifndef HOST_BUILD`.
- [x] `source/devices/microbit_i2c_bus.{h,cpp}` exist, hold the real
      machinery verbatim (same public surface `i2c_bus.h` used to expose:
      `txnCount`/`errCount`/`lastErr`/`clear`/`reentryViolations`/
      `reentryInFlightAddr`/`reentryNewAddr`/`resetStats`/`dumpRecent`/
      `setLogging`/`setIrqGuard`/`irqGuard`), and are CamelCase-conformant
      per `.claude/rules/naming-and-style.md` (lowerCamelCase methods,
      `UpperCamelCase` type name, trailing-underscore private members).
- [x] `source/devices/i2c_bus.cpp` and `source/devices/i2c_bus_host.cpp`
      are deleted.
- [x] `source/main.cpp` constructs a `MicroBitI2CBus` in the bus slot.
- [x] `CMakeLists.txt`'s `i2c_bus_host.cpp` FILTER-EXCLUDE line is removed.
- [x] `python build.py --fw-only` builds the ARM firmware successfully.
- [x] `grep -n "HOST_BUILD" source/devices/i2c_bus.h` returns nothing.
- [x] The 4 device leaves (`NezhaMotor`, `Otos`, `LineSensorLeaf`,
      `ColorSensorLeaf`) are unmodified — they already hold `I2CBus&`.

## Implementation Plan

**Approach**: Pure mechanical split, mirroring the existing
`App::Transport` pattern (`source/app/comms.h:34`) already in this
codebase — a plain virtual base with zero `#ifdef`, real ARM adapters as
separate files. Move code, don't rewrite logic.

**Files to create**:
- `source/devices/microbit_i2c_bus.h`
- `source/devices/microbit_i2c_bus.cpp`

**Files to modify**:
- `source/devices/i2c_bus.h` (reduce to pure interface)
- `source/main.cpp` (construct `MicroBitI2CBus`)
- `CMakeLists.txt` (drop one exclude line)

**Files to delete**:
- `source/devices/i2c_bus.cpp`
- `source/devices/i2c_bus_host.cpp`

**Testing plan**:
- `python build.py --fw-only` — ARM build green.
- Expect `tests/sim/unit/devices_i2c_bus_harness.cpp` /
  `test_devices_i2c_bus.py` (and any other test using the deleted scripted
  surface) to fail to build/collect — confirm the failure is exactly "no
  scripted surface" (a compile error referencing `scriptWrite`/
  `scriptRead`/`setClock`), not something else. Do not fix them here.
- Spot-check: no other test outside `tests/sim/unit/` references the
  deleted scripted surface (grep `scriptWrite\|scriptRead\|I2CBus::setClock`
  across `tests/`).

**Documentation updates**: none beyond code comments already carried over
from `i2c_bus.h`'s existing header (update its own file-header comment to
describe the new pure-interface shape instead of the old two-fork one).
