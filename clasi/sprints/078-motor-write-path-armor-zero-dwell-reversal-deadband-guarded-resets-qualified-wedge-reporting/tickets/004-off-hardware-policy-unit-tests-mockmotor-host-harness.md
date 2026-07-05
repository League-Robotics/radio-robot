---
id: '004'
title: Off-hardware policy unit tests (MockMotor host harness)
status: open
use-cases:
- SUC-005
- SUC-002
- SUC-003
- SUC-004
depends-on:
- '002'
github-issue: ''
issue: armor-motor-write-path-against-reversal-latch.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Off-hardware policy unit tests (MockMotor host harness)

## Description

Builds the sprint's off-hardware acceptance proof (SUC-005) for ticket
002's `Hal::Motor` armor policy. Per architecture-update.md Design
Rationale 9: this deliberately does **not** build the deferred new-tree
simulator harness or a scripted `I2CBus` `HOST_BUILD` fake (neither exists
in operable form today, and the fuller seam is recommended for sprint 079,
which needs it for flip-flop/throttle testing — see Open Question 4).
Instead: a small, dependency-free `MockMotor` test leaf that implements
only the four protected pure virtuals `Hal::Motor` now requires
(`writeRawDuty`, `hardReset`, `softRebaseline`, `configureDevice`),
recording calls instead of touching hardware — no I2C, no CODAL. This
tests `Hal::Motor`'s shared policy in complete isolation, which is exactly
the testability benefit of placing the armor in the base class (Decision
1).

**New files**:
- A standalone C++ test harness (e.g.
  `tests/sim/unit/motor_policy_harness.cpp`) that `#include`s only
  `source/hal/capability/motor.h` and `source/messages/*.h` (both already
  dependency-free — no `MicroBit.h`, no `I2CBus`). Defines `MockMotor :
  public Hal::Motor` recording every call to the four protected virtuals
  (call count, arguments, ordering) into simple member vectors/counters,
  plus stub `setDutyCycle()`/`position()`/`velocity()`/`appliedDuty()`/
  `connected()`/`tick()`/`capabilities()` etc. sufficient to drive
  scripted scenarios. Exercises, at minimum, the six scenarios from
  SUC-005's Main Flow:
  1. A commanded sign change triggers `writeRawDuty(0)` immediately, then
     suppresses further non-zero `writeRawDuty()` calls until
     `reversalDwell_` ms of simulated `now` have elapsed, then the new
     direction is written.
  2. A sub-`outputDeadband_` duty request writes 0, not a tiny signed
     value.
  3. A commanded stop (`duty == 0`) is written immediately even while a
     dwell is in progress (cancels it).
  4. `resetPosition()` while "moving" (simulate `lastRequestedDuty_ != 0`
     and/or `restTicks_` below `kRestTicksRequired`) dispatches
     `softRebaseline()`, never `hardReset()`.
  5. `resetPosition()` while genuinely at rest (`restTicks_ >=
     kRestTicksRequired`) dispatches `hardReset()`.
  6. A motor with a frozen position and zero applied duty reports
     `wedged() == true` but `wedgeSuspect() == false`; the same frozen
     position with applied duty above the deadband reports both `true`.
  - Exits nonzero on any assertion failure, prints a clear message per
    scenario (this is a plain C++ program with hand-rolled assertions —
    no test framework dependency needed for six scenarios).
- A thin pytest wrapper (e.g. `tests/sim/unit/test_motor_policy.py`) that:
  compiles the harness with the system C++ compiler (`c++`/`clang++`,
  `-std=c++17` or whatever standard `source/` targets — check
  `CMakeLists.txt` for the project's C++ standard flag) via `subprocess`,
  runs the resulting binary, and asserts exit code 0. Collected under
  `tests/sim/unit/` alongside the existing `test_placeholder.py` — no
  `pyproject.toml` `testpaths` change needed (`tests/sim` is already
  collected).

## Acceptance Criteria

- [ ] `MockMotor` implements only the four new protected virtuals plus the
      minimal existing public primitives needed to drive scenarios — no
      I2C, no CODAL, no dependency beyond `capability/motor.h` and
      `messages/*.h`.
- [ ] All six scenarios from the Description pass.
- [ ] The pytest wrapper is collected by `uv run python -m pytest` (runs
      as part of the existing `tests/sim` domain, no `pyproject.toml`
      change required) and completes in well under a second with no
      connected hardware and no ARM toolchain.
- [ ] The harness/test is self-contained: compiling and running it does
      not require `just build`, `mbdeploy`, or any CODAL/ARM toolchain
      component.
- [ ] `tests/sim/unit/test_placeholder.py` is left in place or removed
      only if this ticket's test makes `tests/sim/unit/` non-empty in a
      way that satisfies the placeholder's own stated purpose (its
      docstring says delete it "once `tests/sim/unit/` gains real tests")
      — reviewer's call, not required either way.

## Testing

- **Existing tests to run**: `uv run python -m pytest` (must stay green,
  including the new test).
- **New tests to write**: the harness + pytest wrapper described above —
  this ticket's entire deliverable *is* the new test.
- **Verification command**: `uv run python -m pytest tests/sim/unit/`
