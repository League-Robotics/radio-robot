---
id: '003'
title: Resolve DebugCommandable I2CBus leak via IBusDiagnostics+IRawBusAccess; empty
  vendor baseline
status: done
use-cases:
- SUC-004
depends-on:
- 044-002
github-issue: ''
issue: migrate-radio-robot-c-to-the-frc-elite-architecture-c-codal-adaptation.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 044-003: Resolve DebugCommandable I2CBus leak via IBusDiagnostics+IRawBusAccess; empty vendor baseline

## Description

`source/app/DebugCommandable.h` is the LAST file above `source/io/` that references
a vendor type (`I2CBus`). It forward-declares `class I2CBus` and holds `I2CBus* bus`
in `DbgCtx`. `DebugCommandable.cpp` `#include "I2CBus.h"` inside `#ifndef HOST_BUILD`.
These four occurrences are the entire content of `tests/_infra/vendor_baseline.txt`.

This ticket seals the final leak using two narrow interfaces:
1. **Extended `IBusDiagnostics`** — adds the diagnostic methods actually needed by
   the DBG handlers (`txnCount`, `errCount`, `lastErr`, `resetStats`, `setLogging`,
   `dumpRecent`, `irqGuard`, `setIrqGuard`).
2. **New `IRawBusAccess`** — two-method interface (`write`, `read`) for raw I2C
   byte transfer needed by `I2CW`/`I2CR`.

After this ticket the vendor-confinement grep has zero hits above `source/io/` and
`vendor_baseline.txt` is empty.

**Depends on 044-002** because T2 changes many include paths; doing T3 after T2
avoids a conflict on `DebugCommandable.cpp` include ordering.

## Files to Create

**`source/io/capability/IRawBusAccess.h`:**
```cpp
#pragma once
#include <stdint.h>
// IRawBusAccess — raw I2C byte-level read/write access.
// Implemented by I2CBusRawAccess (io/real/) wrapping I2CBus.
// Used ONLY by DebugCommandable for I2CW / I2CR diagnostics.
class IRawBusAccess {
public:
    virtual ~IRawBusAccess() = default;
    // Write len bytes from data to addr8 (8-bit shifted address).
    // If repeated=true, issue a repeated-start instead of stop.
    // Returns 0 on success, non-zero on error.
    virtual int write(uint16_t addr8, const uint8_t* data, int len,
                      bool repeated = false) = 0;
    // Read len bytes from addr8 into buf.
    // Returns 0 on success, non-zero on error.
    virtual int read(uint16_t addr8, uint8_t* buf, int len) = 0;
};
```

**`source/io/real/I2CBusRawAccess.h` and `I2CBusRawAccess.cpp`:**
- `I2CBusRawAccess` implements `IRawBusAccess` by wrapping `I2CBus&`.
- `write` delegates to `_bus.write(addr8, data, len, repeated)`.
- `read` delegates to `_bus.read(addr8, buf, len)`.
- Compiled only in the firmware (`#ifndef HOST_BUILD`).

## Files to Modify

**`source/io/capability/IBusDiagnostics.h`** — extend with 8 new pure-virtual methods:
```cpp
// Added in Sprint 044 (Phase F) to seal DebugCommandable's I2CBus leak:
virtual uint32_t txnCount(uint8_t addr7) const = 0;
virtual uint32_t errCount(uint8_t addr7) const = 0;
virtual int8_t   lastErr(uint8_t addr7) const = 0;
virtual uint32_t reentryViolations() const = 0;
virtual void     resetStats() = 0;
virtual void     setLogging(bool on) = 0;
virtual void     dumpRecent(ReplyFn fn, void* ctx) const = 0;
virtual bool     irqGuard() const = 0;
virtual void     setIrqGuard(bool on) = 0;
```
(3 original methods are kept; `errorCount()`, `reentryViolations()`, `lastError()`
are either renamed or retained for backward compat — check what `MotorController`
currently calls via `_busDiag`; keep those names and add the new ones.)

**`source/io/real/MotorBusDiagnostics.h/.cpp`** — implement the 8 new methods by
forwarding to `_bus.txnCount(addr)`, etc. The existing `_bus` member (`I2CBus&`) already
exposes all these methods. Add `#include "ReplyFn.h"` or equivalent for the `dumpRecent`
signature if needed (ReplyFn is defined in `CommandTypes.h` — check the include path).

**`NezhaHAL.h/.cpp`** — add `IRawBusAccess& rawBusAccess()` accessor exposing
`_rawBusAccess` value member. Add `I2CBusRawAccess _rawBusAccess` value member
constructed with `_bus` at `NezhaHAL` construction time (before `DebugCommandable`
is constructed).

**`source/app/DebugCommandable.h`** — rewrite `DbgCtx`:
```cpp
#pragma once
#include "CommandTypes.h"
#include "io/capability/IBusDiagnostics.h"
#include "io/capability/IRawBusAccess.h"

class LoopScheduler;
struct Robot;

struct DbgCtx {
    LoopScheduler*   sched;
    IBusDiagnostics* busDiag;    // was: I2CBus* bus
    IRawBusAccess*   busAccess;  // new: for I2CW / I2CR raw transfers
    Robot*           robot;
};
class DebugCommandable : public Commandable { /* unchanged */ };
```
Remove `class I2CBus;` forward declaration.

**`source/app/DebugCommandable.cpp`** — remove `#include "I2CBus.h"` (inside or outside
`#ifndef HOST_BUILD`). Update all handler functions:
- `handleDbgI2clog`: `ctx.bus->resetStats()` → `ctx.busDiag->resetStats()`;
  `ctx.bus->setLogging(true)` → `ctx.busDiag->setLogging(true)`;
  `ctx.bus->dumpRecent(replyFn, replyCtx)` → `ctx.busDiag->dumpRecent(replyFn, replyCtx)`.
- `handleDbgI2c`: `ctx.bus->reentryViolations()` → `ctx.busDiag->reentryViolations()`;
  `ctx.bus->txnCount(0x10)` → `ctx.busDiag->txnCount(0x10)`; etc.;
  `ctx.bus->resetStats()` → `ctx.busDiag->resetStats()`.
- `handleDbgIrqguard`: `ctx.bus->setIrqGuard(...)` → `ctx.busDiag->setIrqGuard(...)`;
  `ctx.bus->irqGuard()` → `ctx.busDiag->irqGuard()`.
- `handleI2cw`: `ctx.bus->write(...)` → `ctx.busAccess->write(...)`.
- `handleI2cr`: `ctx.bus->write(..., true)` → `ctx.busAccess->write(..., true)`;
  `ctx.bus->read(...)` → `ctx.busAccess->read(...)`.
- Check for null: existing `if (ctx.bus == nullptr)` guards become
  `if (ctx.busDiag == nullptr)` (for diag handlers) and `if (ctx.busAccess == nullptr)`
  (for I2CW/I2CR). The null cases produce `ERR noimpl` — behavior unchanged.

**`source/robot/main.cpp`** — update `DbgCtx` construction to pass
`&hardware.busDiagnostics()` as `busDiag` and `&hardware.rawBusAccess()` as `busAccess`
(instead of `&hardware.bus()` as `bus`). The `bus()` accessor on `NezhaHAL` may be
kept for now (it's firmware-internal; not a leak); only its use in `DbgCtx` is replaced.

**`tests/_infra/vendor_baseline.txt`** — clear all four DebugCommandable entries
(the file becomes empty).

## Acceptance Criteria

- [x] `source/app/DebugCommandable.h` has no `class I2CBus` forward declaration.
- [x] `source/app/DebugCommandable.h` `DbgCtx` has `IBusDiagnostics* busDiag` and
      `IRawBusAccess* busAccess`; no `I2CBus*`.
- [x] `source/app/DebugCommandable.cpp` does not `#include "I2CBus.h"` anywhere.
- [x] `source/io/capability/IRawBusAccess.h` exists with `write` and `read` methods.
- [x] `source/io/real/I2CBusRawAccess.h/.cpp` exists implementing `IRawBusAccess`.
- [x] `source/io/capability/IBusDiagnostics.h` declares the extended method set.
      (Kept original `errorCount`/`reentryViolations`/`lastError` — read by
      MotorController — and added the 8 new methods. Used an inline `DumpFn`
      function-pointer typedef for `dumpRecent` so `io/capability/` takes no
      dependency on the command-dispatch layer, per the T3 note.)
- [x] `source/io/real/MotorBusDiagnostics.h/.cpp` implements all new methods
      (verbatim forwarders to the same-named `I2CBus` methods).
- [x] `NezhaHAL` exposes `rawBusAccess()` returning `IRawBusAccess&`, backed by a
      new `I2CBusRawAccess _rawBusAccess(_bus)` value member.
- [x] `tests/_infra/vendor_baseline.txt` is empty.
- [x] `test_vendor_confinement.py` reports zero violations, and a new
      `test_vendor_confinement_zero_hits_empty_baseline` hard-asserts ZERO hits
      above `source/io/` with an empty baseline (the FINAL criterion).
- [x] Host build green. NOTE: `DebugCommandable.cpp` is actually COMPILED in the
      host build (its CODAL-dependent handler bodies are `#ifndef HOST_BUILD`-
      guarded, not the whole file); it compiles cleanly with the new `DbgCtx`
      header, and `sim_api.cpp` constructs `DbgCtx{nullptr,nullptr,nullptr,&robot}`
      (busDiag/busAccess null in HOST_BUILD). The ticket wording "excluded from
      HOST_BUILD" is inaccurate; the intent (host compiles with null pointers) holds.
- [x] ARM firmware build green: `python3 build.py --fw-only` → 0 `error:`,
      `MICROBIT.hex` produced. `git checkout -- source/robot/DefaultConfig.cpp` done.
- [x] Full simulation tier green: `uv run --with pytest python -m pytest -q` → 2002 passed, 0 errors.
- [x] Golden-TLM canary passes byte-exact (`test_golden_tlm_unchanged`).

## Implementation Plan

1. Create `source/io/capability/IRawBusAccess.h` (new interface, two methods).

2. Extend `source/io/capability/IBusDiagnostics.h` with the 8 new pure-virtual methods.
   NOTE: `MotorController` calls `_busDiag->errorCount()`, `_busDiag->reentryViolations()`,
   `_busDiag->lastError()` — keep the original 3 method names. The new methods have
   different names (e.g., `txnCount(addr)` vs the original `errorCount()` which takes
   no addr). Check `MotorController.cpp` to confirm which methods it calls and ensure
   they remain in the interface.

3. Create `source/io/real/I2CBusRawAccess.h` and `I2CBusRawAccess.cpp`.

4. Extend `source/io/real/MotorBusDiagnostics.h/.cpp` to implement the 8 new methods.

5. Add `I2CBusRawAccess _rawBusAccess` to `NezhaHAL` and expose `rawBusAccess()`.
   Construction order: `_rawBusAccess(_bus)` — add after `_busDiag(_bus)` in the
   member initializer list (same pattern).

6. Update `source/app/DebugCommandable.h` — remove forward decl and `I2CBus*`;
   add `IBusDiagnostics*` + `IRawBusAccess*`.

7. Update `source/app/DebugCommandable.cpp` — remove `#include "I2CBus.h"`; update
   handler functions as described above.

8. Update `main.cpp` `DbgCtx` construction.

9. Update `tests/_infra/sim/CMakeLists.txt` if `I2CBusRawAccess.cpp` is in the sim
   source glob — it should NOT be (it's a real-hardware file; check glob exclusions).

10. Add `I2CBusRawAccess.cpp` and updated `MotorBusDiagnostics.cpp` to the firmware
    CMake source list (they're in `source/io/real/` which is globbed).

11. Compile: `uv run --with pytest python -m pytest tests/simulation/unit/test_golden_tlm.py -q`.

12. Clear `tests/_infra/vendor_baseline.txt`.

13. Run simulation tier + ARM build gate.

## Testing Plan

- **Vendor confinement:** `test_vendor_confinement.py` — zero violations.
- **Golden-TLM:** `test_golden_tlm.py` — byte-exact.
- **Full simulation tier:** `uv run --with pytest python -m pytest -q`.
- **ARM build:** `python3 build.py --fw-only`.
- **No new tests in this ticket** — tests are in T4. The vendor confinement test
  already runs as part of the simulation tier.

## Notes

- `ReplyFn` type (used in `dumpRecent`) is defined in `CommandTypes.h`. Since
  `IBusDiagnostics.h` is in `source/io/capability/` and `CommandTypes.h` is in
  `source/types/`, adding `#include "types/CommandTypes.h"` to `IBusDiagnostics.h`
  would create a dependency from `io/capability/` → `types/`. Check if this is
  acceptable per the layering rules. Alternative: use raw function pointer type
  `void (*fn)(const char*, void*)` matching `ReplyFn`'s definition directly in
  `IBusDiagnostics.h` (avoids the include). Programmer decides.
- `NezhaHAL::bus()` accessor can be kept as a protected or private method (it's
  used internally by `NezhaHAL` for device construction). It should NOT be called
  from `main.cpp` for `DbgCtx` after this ticket.
- `I2CBusRawAccess.cpp` must be excluded from the sim build (it wraps CODAL `I2CBus`;
  `SimHardware` doesn't have a real bus). Check the sim CMakeLists.txt glob rules for
  `source/io/real/` — it currently excludes `NezhaHAL.cpp` and real I2C files. Add
  `I2CBusRawAccess.cpp` to the exclusion list if needed.
