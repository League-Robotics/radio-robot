---
id: '007'
title: "app/Preamble \u2014 boot-time device-detection driver"
status: done
use-cases:
- SUC-007
depends-on:
- '003'
github-issue: ''
issue: single-loop-firmware-p3-p7-continuation.md
completes_issue:
  single-loop-firmware-p3-p7-continuation.md: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# app/Preamble — boot-time device-detection driver

## Description

Build `source/app/preamble.{h,cpp}`: an app-level driver that sequences
each leaf's own already-existing boot-time detection state machine
(`NezhaMotor::begin()`, `Otos::begin()`, `ColorSensorLeaf::beginStep(nowUs)`,
`LineSensorLeaf::beginStep(nowUs)` — all unchanged, KEPT) to a `done()`
terminal signal, replacing `DeviceBus::runPreamble()` (deleted, ticket
003) with a flatter equivalent over the bare leaves.

Depends on ticket 003 (the leaves this drives are no longer reached
through `DeviceBus`).

## Acceptance Criteria

- [x] `Preamble::step()` advances each leaf's own detection entry point at
      most once per call (one bounded probe action per pass, matching the
      archived plan's boot-loop framing) — no leaf's own retry loop is
      reimplemented inside `Preamble`, only sequenced/called.
- [x] `Preamble::done()` returns true once every leaf has reached a
      terminal state — present-and-ready, OR confirmed-absent after
      exhausting its own retry budget. An absent sensor does not hang boot
      forever (a bounded worst case, matching the retired
      `DeviceBus::kMaxPreambleTicks`'s defensive-bound spirit — this
      ticket picks its own bound, documented, not copied verbatim from a
      deleted constant).
- [x] No I2C traffic is issued by any leaf before `Preamble` has begun
      probing it (confirms boot ordering is deterministic, not
      accidentally overlapping with steady-state reads).
- [x] A decision is made and documented on whether to keep an explicit
      boot power-settle wait (mirroring the retired
      `DeviceBus::kPowerSettleMs`) or rely on each leaf's own retry
      pacing — either is acceptable, but the choice must be stated, not
      left implicit.
- [x] A host-buildable test proves `Preamble::done()` is reachable with
      one or more leaves scripted absent (using each leaf's own
      `HOST_BUILD` scripted-fake I2C responses to simulate a NAK/no-chip
      condition).

## Implementation Plan

**Approach**: Read each leaf's actual `begin()`/`beginStep(nowUs)`
signature and terminal-state accessor (`connected()`/`present()`) directly
— `otos.h`, `color_sensor.h`, `line_sensor.h` — confirmed during this
sprint's planning to already expose exactly what a sequencer needs.
`NezhaMotor::begin()` is a single-shot call (not a `beginStep` state
machine like color/line) — `Preamble` calls it once per motor, not
repeatedly.

**Files to create/modify**:
- `source/app/preamble.h`, `source/app/preamble.cpp` (new)

**Testing plan**:
- Existing tests to run: none directly (new file); confirm each leaf's own
  existing `devices_*` detection-state-machine tests stay green
  (`Preamble` is a new caller, not a modifier, of that logic).
- New tests to write: the done()-reachable-with-absent-leaves test
  (Acceptance Criteria above); a test confirming `step()` never issues
  more than one probe action per leaf per call (assert scripted I2C
  transaction counts advance by at most the expected amount per `step()`
  call).
- Verification command: `uv run python -m pytest tests/sim/unit/ -k preamble`
  (once the test file exists).

**Documentation updates**: document the power-settle-wait decision (kept
or dropped, and why) directly in `preamble.h`'s header comment.

## Completion Notes

**Files created**: `source/app/preamble.h`, `source/app/preamble.cpp`,
`tests/sim/unit/app_preamble_harness.cpp`,
`tests/sim/unit/test_app_preamble.py`.

**API shape**: `App::Preamble` holds references to the same bare leaves
`main.cpp` (ticket 008) will construct — `Devices::NezhaMotor& left/right`,
`Devices::Otos& otos`, `Devices::ColorSensorLeaf& color`,
`Devices::LineSensorLeaf& line` — plus a `const Devices::Clock& clock`
(the same fiber-level time seam `Deadman` already takes; the boot-loop
sketch's bare `preamble.step();` call site, confirmed against
`usecases.md` SUC-008, has no `nowUs` argument, so `Preamble` reads
`clock_.nowMicros()` internally rather than taking it as a parameter — a
deliberate divergence from the illustrative issue sketch's bare
`Preamble preamble(motorL, motorR, otos, color, line);`, exactly mirroring
`Deadman`'s own already-shipped divergence for the identical reason).
Public surface: `step()` (no args), `done()`, and six per-device status
accessors (`leftConnected()`, `rightConnected()`, `otosPresent()`,
`otosConnected()`, `colorPresent()`, `linePresent()`) that forward to each
leaf's own existing accessor — `Preamble` holds no duplicate copy of
device-presence state. Wiring these accessors into
`App::Telemetry::setFrame()`'s `connLeft`/`connRight`/`otosConnected`
fields and `setEvent(kEventBootReady, ...)` on `done()`'s first-true
transition is ticket 008's job (`telemetry.h`'s own fault/event bit-layout
comment already names this hand-off).

**One-bounded-probe-action design**: `step()` maintains a round-robin
cursor over 5 slots (`Left`, `Right`, `Otos`, `Color`, `Line`); each call
visits at most one *unresolved and due* slot and calls that leaf's own
entry point exactly once, then returns — never touching a second leaf in
the same call. Motor slots are always "due" (one-shot terminal —
`NezhaMotor::begin()`'s internal `hardReset()` median-of-3 + retry is the
ticket's own documented multi-transaction exception to "one transaction,"
never to "one leaf"). Color/Line slots are always "due" from `Preamble`'s
point of view — their own `beginStep(nowUs)` already owns its internal
retry pacing (`kAltRetryPeriod`/`kRetryPeriod`) and no-ops internally when
not yet due, so `Preamble` never reimplements that pacing itself. OTOS is
the one exception requiring `Preamble`-owned pacing (`Otos::begin()` has no
retry of its own) — ported unchanged from the retired `DeviceBus`:
`kOtosBeginAttempts = 20`, `kOtosBeginRetryPeriod = 100000` `[us]` (was
`kOtosBeginRetryPacingMs = 100`).

**Ported constants table** (from `device_bus.h`, git history
`88e04f1b^:source/devices/device_bus.h`):

| Retired name | Retired value | `preamble.h` name | New value | Note |
|---|---|---|---|---|
| `kPowerSettleMs` | 50 `[ms]` | `kPowerSettleUs` | 50000 `[us]` | kept, see Decision below |
| `kOtosBeginAttempts` | 20 | `kOtosBeginAttempts` | 20 | unchanged |
| `kOtosBeginRetryPacingMs` | 100 `[ms]` | `kOtosBeginRetryPeriod` | 100000 `[us]` | unit only |
| `kMaxPreambleTicks` | 64 (ticks, ~50ms pacing each) | `kMaxPreambleUs` | 5000000 `[us]` | NOT copied verbatim — re-derived as a wall-clock bound (see below), per the ticket's own instruction |

`color_sensor.h`'s `kMaxAltAttempts`/`kAltRetryPeriod` and
`line_sensor.h`'s `kMaxAttempts`/`kRetryPeriod` are not re-ported — they
already live on the leaves themselves, unchanged.

**Decision: KEEP the boot power-settle wait** (documented in
`preamble.h`'s own header comment, per this ticket's own AC): kept,
unchanged in value. Rationale: it is free (no leaf touched, no bus traffic
while it elapses), it is a bench-tuned value already proven on real
hardware, and "rely on each leaf's own pacing instead" is not actually
available for the very first device probed (`NezhaMotor::begin()` has no
retry pacing of its own to lean on — it is a single `hardReset()` call).

**Defensive bound re-derivation** (`kMaxPreambleUs = 5000000` `[us]`, NOT
`kMaxPreambleTicks` copied verbatim, per the ticket's own instruction):
every slot already self-bounds without this constant (motor: one call;
OTOS: `Preamble`'s own `kOtosBeginAttempts` counter; color/line: each
leaf's own internal attempt bound), *provided* `step()` is called often
enough with real elapsed time between calls — the boot loop's job, not
this ticket's. `kMaxPreambleUs` is a pure wall-clock safety net (a future
leaf regression, e.g. a `detectDone()` that never returns true, forces
every remaining slot terminal via `forceResolveAll()`), sized generously
above the natural worst case (OTOS-bound: 20 × 100 ms = 2000 ms; color/line
≈ 1050 ms / 1000 ms; plus the 50 ms power-settle wait).

**Test scenarios** (`app_preamble_harness.cpp`, 4 scenarios, all real
`Devices::` leaves over a scripted `HOST_BUILD` `I2CBus`/`Clock`):
1. All-present happy path — every leaf detects on its first attempt;
   proves `done()` reachable in exactly 5 probe-carrying `step()` calls,
   zero I2C traffic before the first real probe, and `step()`'s
   at-most-one-address-per-call bound holds on every single call.
2. OTOS absent — every other leaf detects normally, OTOS's address is left
   completely unscripted (the project's established "absent device"
   convention: an unscripted read decodes to a mismatched product ID);
   `done()` is still reached, and OTOS was retried **exactly**
   `kOtosBeginAttempts` (20) times (40 transactions, 2 per attempt) — not
   fewer (premature latch), not more (unbounded retry). Required
   interleaving the script-queueing calls between `step()` calls (see the
   harness's own header note): `scriptWrite()`/`scriptRead()` are ONE
   shared FIFO per `I2CBus` instance across every address, so pre-queuing
   Color's/Line's scripts before OTOS's unscripted turn would let OTOS's
   own mismatched pop silently consume their entries out of order — caught
   during local scenario iteration before landing.
3. Transient NAK during `NezhaMotor::begin()` is not latched — two of the
   four internal `readEncoderAtomicRaw()` calls inside `hardReset()`'s
   first attempt carry a scripted transient I2C failure, but the last
   (readback) call succeeds; proves `Preamble::leftConnected()` reports
   `true` from the leaf's own freshest state after `Preamble`'s single
   `begin()` call, not latched false by an earlier hiccup within that same
   call — `Preamble` itself never retries the leaf.
4. Multiple leaves absent simultaneously (OTOS + color + line, motors
   present) — a stronger form of the "absent sensor does not hang boot
   forever" AC; `done()` still reached, bounded, motors unaffected.

**Verification evidence**:
- `c++ -std=c++20 -Wall -Wextra -DHOST_BUILD` compile of the harness +
  `preamble.cpp` + every leaf `.cpp` it drives: clean, zero warnings.
  Binary run: `OK: all App::Preamble scenarios passed`.
- `uv run python -m pytest tests/sim/ -q` — 339 passed (336 baseline from
  ticket 005's own completion notes + 2 from ticket 006, already merged on
  this branch, `test_app_drive.py`/`test_app_odometry.py` + this ticket's
  own 1 new `test_app_preamble.py::test_app_preamble_harness_compiles_and_
  passes`, which itself runs the harness's 4 scenarios internally — the
  harness's `beginScenario()` calls print progress but are not separate
  pytest collection items).
- `just build` — succeeds; `source/app/preamble.cpp` compiles clean for
  the real ARM target and links into `MICROBIT.hex` at v0.20260714.13.
  FLASH 27.84%, RAM 98.33% — unchanged from ticket 006's baseline (expected:
  `main.cpp` is still the sprint-102 stub, so nothing calls into
  `Preamble` yet; ticket 008 wires it in).
- `tests/unit/` was not touched and was not run against this ticket's new
  code (Decision 4 — pre-existing breakage there is out of scope, per
  sprint 103's own established precedent).

**Surprises**: the archived-plan boot-loop sketch's `preamble.step();`
call site has no `nowUs` argument, which only becomes apparent by reading
`usecases.md` SUC-008's main-flow text closely (the sprint's own
`architecture-update.md` never states this explicitly) — `Preamble`
therefore takes a `Clock&` constructor argument the issue's illustrative
5-argument constructor sketch omits, mirroring `Deadman`'s identical,
already-shipped divergence (ticket 004). The shared-FIFO
`scriptWrite()`/`scriptRead()` hazard (scenario 2 above) was caught and
fixed during this ticket's own test-writing pass, not left for a future
bug report.
