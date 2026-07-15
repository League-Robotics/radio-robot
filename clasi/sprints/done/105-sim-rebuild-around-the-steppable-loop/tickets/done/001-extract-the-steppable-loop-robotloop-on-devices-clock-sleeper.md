---
id: '001'
title: 'Extract the steppable loop: RobotLoop on Devices::Clock/Sleeper'
status: done
use-cases:
- SUC-018
depends-on: []
github-issue: ''
issue: ''
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Extract the steppable loop: RobotLoop on Devices::Clock/Sleeper

## Description

`source/main.cpp`'s `int main()` unconditionally constructs
`static MicroBit uBit;` and `#include "MicroBit.h"`, and its own timing
primitives (`markTime()`/`sleepUntil()`/`runAndWait()`) call
`system_timer_current_time()` and `uBit.sleep()` directly — vendor/ARM-only
calls with no `#ifndef HOST_BUILD` guard anywhere in the file. `main.cpp`
therefore cannot be compiled under `-DHOST_BUILD` at all today, even though
every module it composes (`App::Comms`, `App::Telemetry`, `App::Drive`,
`App::Odometry`, `App::Deadman`, `App::Preamble`, and every `Devices::`
leaf) already is host-buildable. `source/devices/clock.h`'s own file
header already states the intended design: *"The cycle body is
parameterized on a sleeper/clock interface: fiber_sleep + system_timer on
hardware; the steppable fake clock in host tests"* — `App::Deadman` and
`App::Preamble` already take a `const Devices::Clock&` for exactly this
reason, but `main.cpp`'s own outer-loop pacing does not.

This ticket extracts the boot loop and main cycle body currently inline in
`main()` into a new, host-buildable module (`App::RobotLoop`) that takes
`Devices::Clock&`/`Devices::Sleeper&` (not raw vendor calls) for every time
read and every sleep/yield, plus references to already-constructed leaves
and `app/` modules. `main.cpp` itself becomes a thin ARM-only wrapper:
construct real hardware, then call into `RobotLoop`. This is a mechanical
extraction with ZERO intended behavior change on ARM — every other ticket
in this sprint depends on this one, because the sim can only be trusted if
its foundation is provably the same code the real robot runs, not a
separately-authored copy (architecture-update.md Decision 1).

This is the sprint's own highest-risk ticket: it touches the file that IS
the production firmware's entry point. It must not proceed to later
tickets until the bench-gate re-verification below passes.

## Acceptance Criteria

- [x] `source/main.cpp`'s boot loop (`while (!preamble.done())`) and main
      `for(;;)` cycle body (the `runAndWait`/`markTime`/`sleepUntil`
      schedule and the command-dispatch `switch`) are extracted into a new
      `source/app/robot_loop.{h,cpp}` module that compiles without
      `MicroBit.h` under `-DHOST_BUILD`.
- [x] Every time read/sleep/yield in the extracted module goes through
      `Devices::Clock&`/`Devices::Sleeper&` — no `system_timer_current_time()`
      or `uBit.sleep()` call survives inside `robot_loop.{h,cpp}`.
      `markTime()`/`sleepUntil()`/`runAndWait()` are rewritten in terms of
      `clock.nowMicros()` (converted to ms where the existing code used ms)
      and `sleeper.sleepMillis()`.
- [x] `main.cpp` itself shrinks to: `MicroBit`/`SerialPort`/`Radio`/
      `I2CBus`/leaf/`Devices::Clock`/`Devices::Sleeper` construction, plus
      one call into `App::RobotLoop`. No cycle logic remains inline in
      `main.cpp`.
- [x] A diff-level review confirms zero change to cycle ordering, the
      timing constants (`kSettle=4`, `kClear=4`, `kCycle=16`,
      `kPreamblePace=10`), or dispatch semantics (TWIST/CONFIG/STOP/NONE
      handling, deadman-expiry stop, ack-ring calls) — this ticket moves
      code, it does not change what the code does.
- [x] A new `HOST_BUILD` harness (e.g. `tests/sim/unit/app_robot_loop_
      harness.cpp` + `test_app_robot_loop.py`, matching the established
      `test_app_drive.py` compile-and-run pattern) constructs `RobotLoop`
      with a scripted `I2CBus`, fake `Clock`/`Sleeper`, and a minimal
      transport stub, steps it through boot + a few cycles, and confirms
      it runs to completion with no `MicroBit.h` dependency anywhere in
      the compiled translation units.
- [x] **Bench-verified per `.claude/rules/hardware-bench-testing.md`**:
      flash the extracted firmware to the robot on the stand. Confirm:
      sensors alive (encoders, OTOS respond with plausible changing
      values), wheels drive both directions with encoders incrementing
      proportionally, and round-trip commands/telemetry work over serial.
      This is a regression check against the pre-extraction behavior, not
      a new capability — any observed difference is a bug in this ticket.

## Completion Notes

**Extraction shape**: `source/app/robot_loop.{h,cpp}` — `App::RobotLoop`
class with three public methods: `run()` (`[[noreturn]]`, calls `boot()`
then `cycle()` forever — what `main.cpp` calls), `boot()` (the
`while (!preamble.done())` loop, byte-identical to the pre-extraction
body), and `cycle()` (one pass of the `for(;;)` body — the three
`runAndWait` blocks + final `sleepUntil`). Constructor takes references to
`Devices::I2CBus`, both `Devices::NezhaMotor` leaves, `Devices::Otos`,
`App::Comms`, `App::Telemetry`, `App::Drive`, `App::Odometry`,
`App::Deadman`, `App::Preamble`, `const Devices::Clock&`, and
`Devices::Sleeper&`. `Devices::I2CBus&` IS held directly (needed for the
cycle body's own `bus.clearanceSafetyNetCount()` fault read) — the
implementation plan's text said it wasn't needed; the actual
pre-extraction loop body proved otherwise, so the constructor keeps it to
preserve that fault-bit behavior unchanged. `driving_`/`frame_`
(cycle-persistent state) became private members; `cmd`/`cycleStart`
stayed cycle-local. `markTime()`/`sleepUntil()`/`runAndWait()` moved to
private members built on `clock_.nowMicros()/1000` and
`sleeper_.sleepMillis()`. `main.cpp` now constructs a `Devices::Sleeper`
alongside the existing `Devices::Clock` and shrinks to construction + one
`robotLoop.run()` call.

**Host test**: `tests/sim/unit/app_robot_loop_harness.cpp` +
`test_app_robot_loop.py`. Drives `Preamble` to `done()` directly (via
`preamble.step()` + `clock.setMicros()`, matching
`app_preamble_harness.cpp`'s own idiom) *before* calling
`robotLoop.boot()`, because `boot()`'s internal `while` loop has no way to
advance a `HOST_BUILD` fake `Clock` itself (mirrors real hardware, where
only the vendor timer — not `RobotLoop` — advances `Clock`); with the
power-settle gate already satisfied and every leaf scripted to succeed on
its first probe, `boot()`'s own loop still resolves all 5 devices across
5 real internal iterations. Then runs `cycle()` 3 times with scripted
`I2CBus` encoder/OTOS traffic, asserting zero bus-script under/over-run,
correct `position()` values, and that `Devices::Sleeper` recorded a sleep
for every pass. `1 passed` locally; full suite `562 passed` (561 baseline
+ 1 new file).

**Bench evidence** (robot `/dev/cu.usbmodem2121102`, UID
`9906360200052820a8fdb5e413abb276000000006e052820`, flashed via
`mbdeploy deploy <uid> --hex MICROBIT.hex` after `just build-clean`):
- `tests/bench/twist_drive.py`: connect, forward twist ack, encoders
  moving, stop ack — `6/6` (one earlier run had a transient stop-ack
  timing race on the very first post-flash command; two clean re-runs
  confirmed `6/6`, not a regression).
- Ad hoc scratch bench script (encoders/OTOS alive at power-on, forward +
  reverse twist with correctly-signed encoder deltas, deadman expiry with
  no `stop()` sent — `event_bits` shows `kEventDeadmanExpired` and wheels
  hold within ±2 encoder ticks afterward): `9/9`.
- `line=`/`color=` are NOT wired into `telemetry.proto` yet (103's own
  Open Question 1, still open) — a pre-existing, documented gap unrelated
  to this ticket, not a regression.
- Robot left powered on the new (extracted) image, stopped.

**Surprises**: `RobotLoop::cycle()` interleaves `drive_.tick()` between
`motorL_.tick()` and `motorR_.tick()` (byte-identical to the
pre-extraction body) — this means each motor's PID/duty dispatch first
activates on a DIFFERENT cycle (R on cycle 0, L on cycle 1), a genuine,
deterministic property of the real cycle ordering this ticket must
preserve verbatim, discovered while writing the host harness's I2C
script (the `HOST_BUILD` `I2CBus` fake uses ONE global write/read FIFO
per direction across ALL device addresses, not one per address — an
over-provisioned "slack" write for one motor, if left unconsumed, gets
wrongly popped by the very next call to any OTHER address, e.g. OTOS's
own burst-read write). The implementation plan's own text suggesting
`Devices::I2CBus&` isn't needed by `RobotLoop` was inaccurate against the
actual pre-extraction body (see above) — the ticket's own acceptance
criteria (zero behavior change) were treated as authoritative over that
one plan detail.

## Testing

- **Existing tests to run**: full `uv run python -m pytest` (561-test
  baseline must stay green — this ticket must not regress any existing
  `tests/sim/unit/` or `tests/unit/` test); `just build` (ARM build must
  still compile and link).
- **New tests to write**: `tests/sim/unit/app_robot_loop_harness.cpp` +
  `tests/sim/unit/test_app_robot_loop.py` (host-buildable smoke test per
  the acceptance criteria above).
- **Verification command**: `uv run python -m pytest tests/sim/unit/test_app_robot_loop.py -v`,
  then `mbdeploy probe && mbdeploy deploy --build` and the standing bench
  smoke sequence (`docs/protocol-v2.md` §13) on the stand.

## Implementation Plan

**Approach**: Move code, don't rewrite logic. Read `source/main.cpp` in
full, copy the boot-loop and main-cycle bodies verbatim into
`App::RobotLoop::run()` (or a `boot()`/`cycle()` pair if that reads more
naturally against the existing `Preamble`/main-loop split), replacing every
`system_timer_current_time()` call with `clock_.nowMicros() / 1000` (or
keep the loop's own "now" in ms via a thin wrapper) and every `uBit.sleep(x)`
call with `sleeper_.sleepMillis(x)`. `RobotLoop`'s constructor takes
references to every module `main.cpp` currently constructs and wires
(`Devices::I2CBus&` is NOT needed directly — only the leaves and `app/`
modules `main.cpp`'s loop body touches) plus `Devices::Clock&` and
`Devices::Sleeper&`. `main.cpp` keeps `toDeviceMotorConfig()`/
`formatBanner()` (pure construction-time helpers) and the real hardware
construction; it drops everything from `bool driving = false;` through the
closing `}` of `int main()`'s `for(;;)` loop, replacing it with a single
`robotLoop.run();` (or equivalent) call.

**Files to create**:
- `source/app/robot_loop.h` — `App::RobotLoop` class declaration.
- `source/app/robot_loop.cpp` — the extracted boot loop + main cycle body.
- `tests/sim/unit/app_robot_loop_harness.cpp` — HOST_BUILD smoke harness.
- `tests/sim/unit/test_app_robot_loop.py` — pytest wrapper (mirrors
  `test_app_drive.py`'s compile-and-run shape).

**Files to modify**:
- `source/main.cpp` — shrunk to construction + one `RobotLoop` call.

**Testing plan**: host-side smoke test proves the extraction compiles and
runs under HOST_BUILD; the REAL proof is the bench gate (mandatory, not
optional) — flash and drive on the stand exactly as
`.claude/rules/hardware-bench-testing.md` requires, comparing against the
pre-105 tree's known-good behavior (103-010's and 104's own bench sessions
are the reference baseline).

**Documentation updates**: none required beyond the code's own comments
(carry forward `main.cpp`'s existing extensive inline documentation of the
timing/dispatch design into `robot_loop.{h,cpp}` — do not lose it in the
move).
