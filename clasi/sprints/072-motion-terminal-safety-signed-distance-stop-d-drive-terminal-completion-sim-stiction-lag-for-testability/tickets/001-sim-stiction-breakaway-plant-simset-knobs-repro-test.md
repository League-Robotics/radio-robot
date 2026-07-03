---
id: '001'
title: Sim stiction/breakaway plant + SIMSET knobs + repro test
status: open
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: d-drive-terminal-instability-reversal-thrash.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sim stiction/breakaway plant + SIMSET knobs + repro test

## Description

`PhysicsWorld::update()`'s chassis integration is a purely algebraic,
memoryless function of commanded PWM (`velL = (_pwmL / 100.0f) *
_nominalMaxSpeed * _offsetFactorL;`, `source/hal/sim/PhysicsWorld.cpp:73-74`)
— any nonzero PWM produces a proportionally nonzero velocity, so the plant
structurally cannot "land short of target" the way real motor
stiction/breakaway does. This makes `d-drive-terminal-instability-reversal-thrash.md`'s
field failure (5 of 6 recorded `D` drives landing 1-3 mm short, stalling,
reversing, and thrashing before a violent lunge completes the move)
unreproducible in sim except via an artificial forced-encoder-cap harness
that bypasses the plant entirely — useful for diagnosis but not a
regression-testable fix vehicle for tickets 002/003.

This ticket adds a stateless PWM dead-zone stiction gate to `PhysicsWorld`
(`|pwm| < stictionPwm -> vel = 0`, default `stictionPwm = 0` so the gate is
never true and the change is a no-op for every existing test) and,
independently, an optional first-order response-lag filter (separately
defaulted off, `tau <= 0` skips the `expf()` call entirely). Both are
exposed through the existing `SIMSET`/`SIMGET` registry established in
sprint 069, following the exact `simsetters::` free-function-per-knob
pattern — this is an extension of an already-cohesive module family, not a
new module. See `architecture-update.md` Step 3 (`PhysicsWorld` (extended),
`simsetters::` (extended)), Step 4b's insertion-point diagram, and Decision
3 (why a stateless dead-zone gate, not a stateful two-threshold friction
model; why the lag filter is a separate, independently-toggleable knob).

This ticket is the test vehicle tickets 002-004 validate against: it must
ship a repro test that reproduces the D-drive land-short/stall signature
against the CURRENT (pre-002/003) firmware control code, which ticket 004
later flips to a must-now-complete-cleanly assertion once tickets 002/003
land.

Per architecture-update.md Open Question 8, the first-order lag filter is
in scope for this ticket but not load-bearing for the sprint's acceptance
criteria — defer it if it proves non-trivial (e.g. lag-state interaction
with the encoder-noise RNG streams' determinism) without blocking the
sprint; the stiction gate alone is sufficient to reproduce and validate the
fix for the reliability defect.

See `architecture-update.md` Step 3, Step 4b, Step 5 ("Ticket 001"),
Decision 3; `usecases.md` SUC-001.

## Acceptance Criteria

- [ ] `stictionPwmL`/`stictionPwmR` (per-wheel breakaway threshold, PWM
      units 0-100, default 0) are `SIMSET`/`SIMGET`-able.
- [ ] With `stictionPwmL/R` at default (0), every existing test that never
      configures the knob observes byte-identical `PhysicsWorld::update()`
      output (golden-TLM canary unaffected) — confirmed by running the
      existing zero-slip/zero-noise/offset-factor-1.0 golden-TLM test
      unmodified.
- [ ] A commanded `|pwm| < stictionPwmSide` produces exactly zero velocity
      for that wheel this tick, regardless of the wheel's velocity on the
      previous tick (stateless — no "was moving" memory); once
      `|pwm| >= stictionPwmSide`, velocity follows the existing unmodified
      algebraic formula.
- [ ] Boundary behavior is tested explicitly: `|pwm|` exactly at
      `stictionPwmSide` does NOT gate (formula applies); `|pwm|` one unit
      below does gate (`vel = 0`).
- [ ] (If implemented per Open Question 8) `motorLagMsL`/`motorLagMsR`
      (per-wheel first-order response time constant, ms, default 0 = no-op)
      are `SIMSET`/`SIMGET`-able; `tau <= 0` takes a no-`exp()`-call path
      producing `vel == velTarget` bit-for-bit; `tau > 0` converges the
      lag-filtered velocity toward `velTarget` correctly over successive
      ticks. Persistent lag-filter state (`_lagVelL/R`) is zeroed in
      `PhysicsWorld::reset()`.
- [ ] A new end-to-end scenario test, running the actual firmware control
      code (not an artificial encoder-cap harness), configures
      `stictionPwmL/R` above the terminal-decel PWM a `D` drive commands
      near its target and demonstrates a scripted `D` drive measurably
      lands short of the target distance in sim — this is the repro
      vehicle tickets 002/003 are validated against and ticket 004 later
      flips.
- [ ] The existing forced-encoder-cap sim harness (used to diagnose the
      issue against real firmware code, per the issue text) is not removed
      or broken — it remains available as an independent diagnostic tool.
- [ ] Full existing test suite (`uv run python -m pytest`) remains green at
      the confirmed pre-sprint baseline (2621 passed, 0 failed), plus this
      ticket's new tests.

## Testing

- **Existing tests to run**: the golden-TLM canary test(s) covering
  `PhysicsWorld::update()` (zero-slip/zero-noise/offset-factor-1.0
  fixture), the existing `SIMSET`/`SIMGET` registry tests (069/070/071
  precedent), full suite (`uv run python -m pytest`).
- **New tests to write**: isolated `PhysicsWorld` unit tests for the
  stiction gate (fires/does not fire at the threshold boundary; no-op at
  default; per-wheel independence) and, if implemented, the lag filter
  (converges correctly; no-op at `tau <= 0`); one end-to-end scenario test
  reproducing the D-drive land-short/stall signature against the current
  (pre-fix) control code, configured via `SIMSET`.
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: Add the stiction gate as sub-step A's first new stage
(Step 4b's diagram: `PWM -> algebraic target velocity (UNCHANGED formula)
-> NEW stiction gate -> NEW optional lag filter -> encoder accumulation`),
mirroring the existing per-wheel setter/getter shape
(`setOffsetFactor`/`offsetFactorL()`). Register the new knobs in
`SimSetters.h`/`SimCommands.cpp`'s existing registry pattern — new rows,
no changes to existing rows/keys. Add ctypes forwards in
`tests/_infra/sim/sim_api.cpp` only if a caller needs the knob outside
`SIMSET` (069-005 precedent). Write the repro test last, once the gate is
wired end-to-end, using a scripted `D` drive against the real firmware
control code (per the issue's own diagnostic methodology) rather than the
forced-encoder-cap harness.

**Files to create/modify**:
- `source/hal/sim/PhysicsWorld.h` — new per-wheel `_stictionPwmL/R` fields
  (default 0); new optional per-wheel `_motorLagL/R` time-constant fields
  (default 0) and persistent lag-filter state (`_lagVelL/R`); new
  setter/getter declarations.
- `source/hal/sim/PhysicsWorld.cpp` — stiction gate and optional lag filter
  inserted into sub-step A per Step 4b's diagram; `_lagVelL/R` zeroed in
  `reset()`.
- `source/commands/SimSetters.h` — new `simsetters::stiction*`/`motorLag*`
  free functions, one per knob.
- `source/commands/SimCommands.cpp` — new `kSimRegistry[]` rows
  (`stictionPwmL`, `stictionPwmR`, and if implemented `motorLagMsL`,
  `motorLagMsR`); existing rows/keys untouched.
- `tests/_infra/sim/sim_api.cpp` — ctypes forwards for the new knobs, if
  warranted.
- New test file(s) under `tests/simulation/unit/` (PhysicsWorld unit
  coverage) and a scenario test under `tests/simulation/system/` (or
  wherever the project's existing D-drive scenario tests live) for the
  end-to-end repro.

**Testing plan**: `--clean` sim rebuild (`tests/_infra/sim/build.py`)
before running any tests (stale incremental builds on `/Volumes` are a
known project gotcha). Run new `PhysicsWorld`/`SIMSET` unit tests in
isolation first, then the new repro scenario test, then the full suite.

**Documentation updates**: none required in this ticket beyond code
comments on the new fields (unit noted per `docs/coding-standards.md`'s
comment convention, not identifier suffixes, per 071).
