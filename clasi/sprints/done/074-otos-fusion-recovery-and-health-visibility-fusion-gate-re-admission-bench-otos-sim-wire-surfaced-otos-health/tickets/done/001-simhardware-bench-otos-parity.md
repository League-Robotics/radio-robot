---
id: '001'
title: SimHardware bench-OTOS parity
status: done
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: otos-not-used-frozen-pose-ekf-rejects-everything.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# SimHardware bench-OTOS parity

## Description

Firmware's `NezhaHAL`/`MecanumHAL` already implement a correct bench-OTOS
swap: `_otosActive` is a pointer that `setOtosBench(bool)` redirects between
the real `OtosSensor` and an owned `BenchOtosSensor`, and `otos()` returns
`*_otosActive` (`source/robot/NezhaHAL.h:55-62`, gated
`#ifdef BENCH_OTOS_ENABLED`). Host-sim's `SimHardware` has no such object to
swap to at all: `otos()` unconditionally returns `_odom` (a `SimOdometer`,
`source/hal/sim/SimHardware.h:44`), and `setOtosBench(bool on)` only records
a flag (`_benchMode`) that nothing reads (`SimHardware.h:60`). Independently,
`DebugCommands::handleDbgOtos()` (`source/commands/DebugCommands.cpp:421-465`)
hardcodes `ideal=0,0,0 otos=0,0,0` under `HOST_BUILD` because its only path to
a bench sensor is `#if !defined(HOST_BUILD) && defined(BENCH_OTOS_ENABLED)`
followed by a `static_cast<NezhaHAL*>(&ctx.robot->hal)` downcast that cannot
compile in `HOST_BUILD` (`SimHardware` is not a `NezhaHAL`). Together these
mean `DBG OTOS BENCH 1` is a structural no-op in every sim/TestGUI-sim
session — there is no moving object behind it.

`BenchOtosSensor` (`source/hal/real/BenchOtosSensor.{h,cpp}`) already
compiles under `HOST_BUILD` with no `#ifdef` gate inside the header, and is
already explicitly built into the host-sim shared library
(`tests/_infra/sim/CMakeLists.txt:104-107`, comment: "BenchOtosSensor lives
in source/hal/real/ ... so it is not globbed [by the hal/real/ exclusion] ...
explicitly added"). This ticket gives `Hardware`, and specifically
`SimHardware`, the same swap machinery `NezhaHAL` already has, reusing
`BenchOtosSensor` as-is — no new sensor-model code.

This is the first of two independent root causes behind "bench mode shows
the same frozen-OTOS signature" (the second, `Drive`'s boot-bound reference,
is ticket 002 — it needs THIS ticket's substrate to be sim-testable at all).
See `architecture-update.md` Step 3 "Module: Hardware OTOS bench-swap
substrate", Step 5 items 1-2, Design Rationale Decision 2 (why `SimHardware`
owns its own `BenchOtosSensor` rather than reusing the test-only
`SimHandle::benchOtos`); `usecases.md` SUC-001.

## Acceptance Criteria

- [x] `Hardware` (`source/hal/Hardware.h`) gains a forward declaration
      `class BenchOtosSensor;` and a new virtual accessor
      `virtual BenchOtosSensor* benchOtosPtr() { return nullptr; }`
      (default no-op, mirrors the existing `setOtosBench`/`isBenchMode`
      default pattern at lines 57-61). No existing virtual signature changes.
- [x] `NezhaHAL.h`/`MecanumHAL.h`'s EXISTING `BenchOtosSensor* benchOtosPtr()
      { return &_benchOtos; }` (inside their `#ifdef BENCH_OTOS_ENABLED`
      blocks, e.g. `NezhaHAL.h:97`) gains the `override` keyword. No other
      change to either file's bench-swap logic (`setOtosBench`/`isBenchMode`/
      `_otosActive`/`tick(now,cmds)`) — it already works correctly.
- [x] `SimHardware` (`source/hal/sim/SimHardware.h`) gains a
      `BenchOtosSensor _benchOtos;` member and an `IOdometer* _otosActive;`
      member (initialized to `&_odom` in the constructor, mirroring
      `NezhaHAL::_otosActive`'s init at `NezhaHAL.cpp:29`).
      `IOdometer& otos() override` changes from `{ return _odom; }` to
      `{ return *_otosActive; }`. `setOtosBench(bool on) override` performs a
      real swap: `_otosActive = on ? static_cast<IOdometer*>(&_benchOtos) :
      static_cast<IOdometer*>(&_odom);` (previously flag-only). `isBenchMode()
      const override` returns `_otosActive == static_cast<const
      IOdometer*>(&_benchOtos)` (previously returned the standalone
      `_benchMode` flag — remove `_benchMode`, its only reader/writer was
      these two methods). `benchOtosPtr() override` returns `&_benchOtos`.
      `#include "hal/real/BenchOtosSensor.h"` added.
- [x] `SimHardware::tick(uint32_t now_ms, const MotorCommands& cmds)` drives
      `_benchOtos.tick(...)` every call, using the SAME dt-baseline-
      maintained-every-tick discipline `NezhaHAL::tick(now, cmds)` uses
      (`NezhaHAL.cpp:88-101`: the dt baseline is updated unconditionally,
      BEFORE the `if (!isBenchMode()) return;` early-return, so the first
      tick after `DBG OTOS BENCH 1` does not see a large stale `dt` and
      integrate a spike). `SimHardware::advance()`
      (`source/hal/sim/SimHardware.cpp:38-70`) is the single call site to
      extend — add the same dt-baseline member (e.g. `_lastBenchTick`) and
      the bench-mode-gated `_benchOtos.tick(cmds.tgtSpeed[1], cmds.tgtSpeed[0],
      _trackwidth, dt)` call (array convention `[0]=R,[1]=L` matching
      `NezhaHAL.cpp:100`'s call and `OutputState.h`'s documented convention).
- [x] `DebugCommands::handleDbgOtosBench()`'s noise-setting branch
      (`DebugCommands.cpp:379-393`, currently `#if !defined(HOST_BUILD) &&
      defined(BENCH_OTOS_ENABLED)` guarding a `static_cast<NezhaHAL*>`
      downcast) is replaced by an unconditional, null-checked call through
      the new virtual: `if (enable && args.count >= 4) { auto* bench =
      ctx.robot->hal.benchOtosPtr(); if (bench != nullptr) { ...
      bench->setNoise(noiseXY, noiseH, drift); } }` — works identically in
      firmware (`BENCH_OTOS_ENABLED` on) and `HOST_BUILD` (SimHardware always
      has a real `BenchOtosSensor` now). The `#if`/downcast is removed
      entirely, not left as a second path.
- [x] `DebugCommands::handleDbgOtos()` (`DebugCommands.cpp:421-465`) is
      simplified to one path: `BenchOtosSensor* bench =
      ctx.robot->hal.benchOtosPtr(); if (bench != nullptr) { idealX =
      bench->idealX(); ... }` — the `#if !defined(HOST_BUILD) &&
      defined(BENCH_OTOS_ENABLED)` / hardcoded-zero `#else` branch
      (`DebugCommands.cpp:432-449`) is removed. In `HOST_BUILD` with
      `SimHardware`, `benchOtosPtr()` now always returns non-null (unlike
      firmware without `BENCH_OTOS_ENABLED`, where it returns `nullptr` via
      the `Hardware` base default), so `DBG OTOS`'s `ideal=`/`otos=` fields
      reflect the real accumulator in every `HOST_BUILD` session.
- [x] The now-unused `#include "NezhaHAL.h"` at `DebugCommands.cpp:44` is
      removed (drive-by cleanup — confirmed by grep this ticket's own
      change removes the last two `NezhaHAL`-typed references in this file;
      `#include "LoopScheduler.h"`/`#include "WedgeTest.h"` on the
      surrounding lines stay, both still used by other handlers in this
      file: `runWedgeTest`/`ctx.sched`).
- [x] A new sim test drives: `DBG OTOS BENCH 1`, then a driving command
      (`VW`/`T`), ticks forward, and asserts `DBG OTOS`'s `ideal=`/`otos=`
      reply (or the equivalent `sim_get_bench_otos_*`-style hook added for
      `SimHardware`'s new member — see Testing) is non-zero and tracks the
      commanded arc, matching the noiseless `BenchOtosSensor` accumulator
      math. Must FAIL against pre-fix `SimHardware` (which always reports
      `0,0,0` per the hardcoded `HOST_BUILD` branch).
- [x] `DBG OTOS BENCH 0` (disable) restores `otos()`/`benchOtosPtr()`
      callers to the real ground-truth `SimOdometer` — no behavior change
      for any session that never enables bench mode.
- [x] No change to `Drive`'s behavior in this ticket — `Drive::_otos` is
      still the boot-bound reference (fixed in ticket 002); this ticket only
      makes the SUBSTRATE swappable and observable via `DBG OTOS` /
      `benchOtosPtr()`, exactly as `usecases.md` SUC-001's acceptance
      criteria scope it (SUC-002's "live path switches" assertion belongs to
      ticket 002).
- [x] Full suite (`uv run python -m pytest`) passes at the 2672 baseline +
      this ticket's net new test count, zero unexplained failures. The
      `data/robots` drift noted in the sprint's hard contract is
      environmental — do not chase or touch it.

## Testing

- **Existing tests to run**: any existing `DBG OTOS`/`DBG OTOS BENCH`
  round-trip test (grep `tests/simulation` for `"DBG OTOS"` at ticket time);
  full suite.
- **New tests to write**: a new sim test file (or an addition to an existing
  OTOS-adjacent unit test file) that sends `DBG OTOS BENCH 1`, drives with
  `VW`/`T`, ticks, and reads back `DBG OTOS`'s reply line (parse
  `ideal=x,y,h otos=x,y,h`) confirming non-zero, arc-tracking values. If a
  raw ctypes accessor is more convenient than parsing the `DBG OTOS` text
  reply, add symmetrical `sim_get_sim_bench_otos_x/y/h`-style hooks in
  `tests/_infra/sim/sim_api.cpp` + `firmware.py` reading
  `hal.benchOtosPtr()`'s accumulators through `SimHardware` — name these
  distinctly from the EXISTING standalone `sim_get_bench_otos_*` hooks
  (`sim_api.cpp:1083-1101`, which read `SimHandle::benchOtos`, a different,
  still-present object per Design Rationale Decision 2's Consequences) to
  avoid confusing the two. Also add a narrow construction/round-trip test:
  `DBG OTOS BENCH 1` then `DBG OTOS BENCH 0` restores `isBenchMode()` to
  false and `otos()` to the `SimOdometer`.
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: Add the `benchOtosPtr()` virtual to `Hardware` first (additive,
compiles with zero behavior change everywhere). Add `override` to
`NezhaHAL`/`MecanumHAL`'s existing methods (trivial, no logic change — these
two files are correct today). Give `SimHardware` its own `BenchOtosSensor`
member and real pointer-swap logic, mirroring `NezhaHAL` field-for-field
(`_otosActive`, `setOtosBench`, `isBenchMode`, `benchOtosPtr`) and
`NezhaHAL::tick(now,cmds)`'s dt-baseline discipline in
`SimHardware::advance()`. Last, simplify `DebugCommands.cpp`'s two OTOS
handlers to the single `benchOtosPtr()`-based path and drop the now-dead
`#include`.

**Files to create/modify**:
- `source/hal/Hardware.h` — new virtual `benchOtosPtr()`, forward decl.
- `source/robot/NezhaHAL.h`, `source/robot/MecanumHAL.h` — add `override`
  to existing `benchOtosPtr()`.
- `source/hal/sim/SimHardware.h` — new `_benchOtos`/`_otosActive` members,
  `#include "hal/real/BenchOtosSensor.h"`, `otos()`/`setOtosBench()`/
  `isBenchMode()`/`benchOtosPtr()` overrides, remove `_benchMode`.
- `source/hal/sim/SimHardware.cpp` — constructor inits `_otosActive(&_odom)`;
  `advance()` gains the dt-baseline member and the bench-mode-gated
  `_benchOtos.tick(...)` call.
- `source/commands/DebugCommands.cpp` — simplify `handleDbgOtos()` and
  `handleDbgOtosBench()`'s noise branch; remove `#include "NezhaHAL.h"`.
- `tests/_infra/sim/sim_api.cpp`, `tests/_infra/sim/firmware.py` — new
  read-only hooks for `SimHardware`'s bench accumulators if needed (see
  Testing); wired into `tests/_infra/sim/CMakeLists.txt` only if a new
  `.cpp` is added (unlikely — `sim_api.cpp` already builds).
- New or extended test file under `tests/simulation/unit/` for the
  bench-tracks-motion assertion.

**Testing plan**: build the host-sim shared library first (confirms the new
member compiles against the already-included `BenchOtosSensor.cpp`), run the
new bench-tracking test in isolation, then any existing `DBG OTOS`-adjacent
test, then the full suite.

**Documentation updates**: `SimHardware.h`'s class-comment block
(`source/hal/sim/SimHardware.h:14-33`) gets a short note that `otos()` now
depends on bench-mode state, matching the class comment already present at
lines 56-61 (update in place rather than leaving the old "otos() always
returns the SimOdometer" claim stale — see Impact table entry in
`architecture-update.md`).
