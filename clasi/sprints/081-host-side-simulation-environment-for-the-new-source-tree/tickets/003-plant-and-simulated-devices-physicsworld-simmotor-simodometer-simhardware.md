---
id: '003'
title: 'Plant and simulated devices: PhysicsWorld, SimMotor, SimOdometer, SimHardware'
status: done
use-cases:
- SUC-003
depends-on:
- '001'
- '002'
github-issue: ''
issue: host-side-simulation-environment-for-the-new-tree-design-write-up.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Plant and simulated devices: PhysicsWorld, SimMotor, SimOdometer, SimHardware

## Description

Port the errorless ground-truth plant and the two simulated devices
(motors, OTOS) from `source_old/hal/sim/` to the new tree, and introduce
`Subsystems::SimHardware` — the sim's owner/scheduler, a Subsystems-tier
peer of `Subsystems::NezhaHardware`, implementing ticket 002's
`Subsystems::Hardware` seam (see `architecture-update.md` Decision 2 for
why `SimHardware` is a `Subsystems::` class, not a `Hal::` leaf beside
`SimMotor`/`SimOdometer`).

Depends on ticket 001 (`SimMotor`'s VELOCITY mode calls the same
`Hal::MotorVelocityPid` `NezhaMotor` calls) and ticket 002 (`SimHardware`
derives from `Subsystems::Hardware`).

**This ticket's most important, non-obvious acceptance criterion is the
dt=0 re-entry guard** (`architecture-update.md` Decision 4): a repeated
`SimHardware::tick(now)` call at an **unchanged** `now` — which happens on
**every ordinary pass** of `devLoopTick` (its two-slice `hardware.tick(now)`
call), not only during the sim's synchronous-command replay trick — must
not re-invoke any `SimMotor`'s `MotorVelocityPid::compute()` a second time.
`NezhaMotor`'s existing `dt<=0 -> kNominalDt` fallback (ticket 001) is
otherwise inert for real hardware (the I2C bus's microsecond clearance
timer naturally prevents re-entry) but would silently double-integrate the
PID in sim if `SimHardware` forwarded both slices straight through to each
`SimMotor::tick()`.

## Acceptance Criteria

- [x] `Hal::PhysicsWorld` (`source/hal/sim/physics_world.{h,cpp}`) ports the
      motor/pose/encoder plant from `source_old/hal/sim/PhysicsWorld.{h,cpp}`
      — midpoint-arc integration, true pose + true wheel travel, plus the
      separate *reported* (errored) encoder accumulator (scale/slip/noise,
      stiction gate + first-order lag, body scrub + rotational slip). Aux
      line/color/port truth channels (`_lineRaw`/`_colorRGBC`/`_port` +
      their setters/getters) are **dropped**, per the design's resolved
      decision 2 — do not port them.
- [x] `Hal::SimMotor : public Hal::Motor` (`source/hal/sim/sim_motor.{h,cpp}`)
      implements the full `Hal::Motor` contract: DUTY mode stages duty
      straight to the plant (stiction gate + lag applied there); VELOCITY
      mode calls `Hal::MotorVelocityPid::compute(...)` (ticket 001's class,
      identical to `NezhaMotor`'s) each tick, feeding the resulting duty
      through the same stiction/lag path; `position()` reads the plant's
      reported (errored) encoder; `capabilities()` reports
      `duty_cycle=true, voltage=false, velocity=true, position=false,
      has_encoder=true` (no POSITION mode — `DEV M n POS` answers
      `ERR unsupported`, matching a Nezha that lacked the capability);
      `connected()=true`, `wedged()=false` in v1 (fault injection is
      out of scope — see `clasi/issues/later/sim-hardware-fault-injection.md`,
      already filed, referenced here rather than re-filed).
- [x] `Hal::SimOdometer : public Hal::Odometer` (`source/hal/sim/sim_odometer.{h,cpp}`)
      is the first concrete leaf of `Hal::Odometer`
      (`source/hal/capability/odometer.h`, currently declared only):
      `pose()` returns `msg::PoseEstimate` sampled from the plant's true
      pose each tick, differenced, with independent OTOS noise/scale/drift
      error applied in its own accumulator (never sharing state with the
      encoder error model).
- [x] `Subsystems::SimHardware : public Subsystems::Hardware`
      (`source/subsystems/sim_hardware.{h,cpp}`) owns one `PhysicsWorld` +
      four `SimMotor`s + the `SimOdometer`; binds port 1->plant LEFT,
      port 2->plant RIGHT by default (ports 3/4 unbound, trivial standalone
      integrators), rebindable to mirror `DEV DT PORTS`.
- [x] **The dt=0 re-entry guard**: `SimHardware::tick(now)` tracks its own
      `lastAdvancedNow_` and is a complete no-op — no `SimMotor::tick()`
      call, no `PhysicsWorld::update()` call — when `now == lastAdvancedNow_`.
      A standalone-compiled harness (ad hoc compile, matching the existing
      `tests/sim/unit/*_harness.cpp` convention — no CMake needed yet)
      proves this explicitly: call `SimHardware::tick(now)` twice with the
      same `now` while a `SimMotor` is in VELOCITY mode, and assert its
      duty/integral state is identical after the second call (not advanced
      a second time).
- [x] **Zero-error determinism gate**: with every error knob at its zero
      default, true encoder == reported encoder == OTOS accumulator,
      bit-for-bit, over a scripted sequence of ticks.
- [x] `source/hal/sim/sim_setters.h` adds one **`Hal::` free function per
      error knob** (motor scale/slip/noise, stiction/lag, OTOS noise/scale/
      drift, trackwidth, body scrub, plant port binding) — **not** a new
      `simsetters::` namespace (the design write-up's own sketch used a
      lowercase namespace, which would violate
      `.claude/rules/naming-and-style.md` rule 3; see
      `architecture-update.md` Decision 2). One canonical call site per
      knob, callable both by ticket 004's ctypes ABI and by this ticket's
      own tests.
- [x] No `SIMSET`/`SIMGET` wire command and no sim-specific `TLM` field is
      introduced anywhere in `source/commands/` or `docs/protocol-v2.md`.
- [x] `docs/protocol-v2.md` gains a short sim-notes addition documenting the
      `position=false` capability divergence (Open Question 4).
- [x] Existing `tests/sim/unit/*` harnesses still pass with no regression.

## Testing

- **Existing tests to run**: `uv run python -m pytest tests/sim tests/unit`;
  existing `tests/sim/unit/*_harness.cpp` harnesses.
- **New tests to write**: standalone-compiled harnesses (ad hoc compile,
  same convention as `motor_policy_harness.cpp`) for: (a) the dt=0 re-entry
  guard, (b) the zero-error determinism gate, (c) `SimMotor`'s VELOCITY-mode
  step response against the same `MotorVelocityPid` class ticket 001
  verified on the bench (a sim-side sanity check that the two motors'
  control loops genuinely match, not a substitute for ticket 001's bench
  gate).
- **Verification command**: `uv run python -m pytest tests/sim -q`. No
  hardware-bench-testing gate applies to this ticket itself (it touches no
  real hardware path), but it depends on tickets 001/002, both of which do.

## Implementation Plan

**Approach:**

1. Read `source_old/hal/sim/PhysicsWorld.{h,cpp}` in full; port the motor/
   pose/encoder plant to `source/hal/sim/physics_world.{h,cpp}`, dropping
   the aux truth channels; keep the internal `[-100,100]` actuator scale so
   stiction-knob semantics and old test expectations port unchanged (per
   the design write-up's own note).
2. Implement `Hal::SimMotor` against the `Hal::Motor` contract in
   `capability/motor.h` (the same base `NezhaMotor` implements) — `apply()`/
   `state()` come free from the base; only the primitive setters/getters
   and `tick()`/`capabilities()` need implementing.
3. Implement `Hal::SimOdometer` against `Hal::Odometer` in
   `capability/odometer.h`.
4. Implement `Subsystems::SimHardware` against ticket 002's
   `Subsystems::Hardware`, including the dt=0 guard.
5. Write `source/hal/sim/sim_setters.h`'s free functions, one per error
   knob, each with a doc comment naming the exact field/state it mutates
   (so a future ctypes wrapper — ticket 004 — has an unambiguous 1:1
   mapping).
6. Write the standalone harnesses proving the dt=0 guard and the
   zero-error determinism gate.
7. Add the `docs/protocol-v2.md` sim-notes line for the `position=false`
   divergence.

**Files to create:**
- `source/hal/sim/physics_world.h`, `source/hal/sim/physics_world.cpp`
- `source/hal/sim/sim_motor.h`, `source/hal/sim/sim_motor.cpp`
- `source/hal/sim/sim_odometer.h`, `source/hal/sim/sim_odometer.cpp`
- `source/hal/sim/sim_setters.h`
- `source/subsystems/sim_hardware.h`, `source/subsystems/sim_hardware.cpp`
- `tests/sim/unit/sim_hardware_harness.cpp` (dt=0 guard + determinism)

**Files to modify:**
- `docs/protocol-v2.md` (sim-notes addition, capability divergence)

**Testing plan:** see "Testing" section above.

**Documentation updates:** `docs/protocol-v2.md`'s sim-notes addition (see
acceptance criteria); no other doc changes required — this ticket
introduces no wire surface.
