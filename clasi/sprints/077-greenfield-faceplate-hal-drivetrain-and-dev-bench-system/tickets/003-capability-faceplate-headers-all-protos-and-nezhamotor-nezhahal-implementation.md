---
id: '003'
title: Capability faceplate headers (all protos) and NezhaMotor/NezhaHal implementation
status: open
use-cases:
- SUC-003
depends-on:
- '002'
github-issue: ''
issue: greenfield-rebuild-faceplate-hal-in-a-fresh-source-old-tree-parked.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Capability faceplate headers (all protos) and NezhaMotor/NezhaHal implementation

## Description

Write `source/hal/capability/*.h` — one faceplate interface header per proto
component (motor, gripper, line sensor, color sensor, ports, odometer), in
`namespace Hal`. Only the motor faceplate gets an implementation this
ticket: `source/hal/nezha/nezha_motor.{h,cpp}` (the concrete Nezha V2 leaf)
and `source/hal/nezha/nezha_hal.{h,cpp}` (I2CBus + up to four `NezhaMotor`s,
one per port). The other five faceplate headers are declarations only —
later tickets (out of this sprint) implement them.

**This is the highest-risk ticket in the sprint.** The split-phase 0x46
encoder request/collect sequencing in `source_old/hal/real/Motor.cpp` is the
direct fix for a recurring wedge-latch class of bug (see
`docs/knowledge/2026-07-01-encoder-wedge-boundary-latch-flavor.md` and
`docs/knowledge/2026-07-04-encoder-latch-reversal-write-train.md`). Port it
byte-for-byte, not by re-deriving it from the register-map comments alone.

## Acceptance Criteria

### Faceplate headers (`source/hal/capability/`, all in `namespace Hal`)

- [ ] `motor.h` — matches the issue's locked interface exactly: primitive
      setters (`setDutyCycle`, `setVoltage`, `setVelocity`, `setPosition`,
      `setNeutral`, `setFeedforward`, `resetPosition`), primitive getters
      (`position`, `velocity`, `appliedDuty`, `connected`, `wedged`),
      faceplate verbs (`configure(const msg::MotorConfig&)`,
      `tick(uint32_t now)`, `capabilities() const`), and the message-plane
      pair `apply(const msg::MotorCommand&)` / `state() const` implemented
      ONCE in this base class (non-virtual, calling the virtual
      setters/getters above) — not left to each leaf to reimplement.
- [ ] `gripper.h`, `line_sensor.h`, `color_sensor.h`, `ports.h`, `odometer.h`
      — one faceplate header each, following the same primitive-setters/
      -getters + `configure`/`tick`/`capabilities` + shared `apply`/`state`
      shape, sized to each proto's actual Command/State/Config/Capabilities
      messages (from ticket 2). These are declarations only this ticket —
      no `.cpp`, no concrete leaf. Do not implement `apply()`/`state()` for
      these five if there's no primitive surface to base them on yet;
      leaving the base-class method bodies to whichever ticket adds the
      first concrete leaf is acceptable, PROVIDED the header still compiles
      standalone (e.g., forward-declare or stub the shared verbs so nothing
      that includes the header requires it to be complete against a leaf
      that doesn't exist yet — programmer's call on the exact mechanism,
      documented in the header comment).
- [ ] Naming: `namespace Hal`, class names UpperCamelCase (`Motor`,
      `Gripper`, ...), methods lowerCamelCase, no unit suffixes on any
      identifier (units in `// [unit]` trailing comment tags per
      `.claude/rules/coding-standards.md`) — e.g. `setVelocity(float
      velocity); // [mm/s] signed`, never `setVelocity(float mmPerSec)`.

### `NezhaMotor` (`source/hal/nezha/nezha_motor.{h,cpp}`)

- [ ] Constructible per-port: `NezhaMotor(I2CBus&, const msg::MotorConfig&)`
      with `config.port` in 1..4 — no baked-in left/right pair, no hardcoded
      motor-id-to-role mapping anywhere in this class.
- [ ] Register map ported from `source_old/hal/real/Motor.cpp`, frames
      verified byte-identical against the existing file's documented wire
      bytes: 0x60 (run), 0x5F (stop — NOT used for coast; 0x60 with speed 0
      is the coast path, exactly as `source_old` does it and for the same
      documented reason: 0x5F wedges subsequent encoder reads), 0x46
      (encoder read), 0x47 (speed read), 0x5D (onboard absolute-angle
      position move), 0x1D (home reset), 0x77 (global servo speed), 0x88
      (version). Only the subset ticket 3 actually needs to wire into
      `tick()`/`apply()` must be exercised end-to-end; the rest (0x70
      timedMove, 0x77, 0x88, 0x1D) may be ported as private wrappers for
      completeness (matching `source_old`'s coverage) without being reachable
      from the public faceplate this sprint.
- [ ] **Split-phase encoder sequencing preserved exactly**: a
      `requestEncoder()` (phase 1 — issue the 0x46 write, non-repeated-start,
      return immediately, no busy-wait) / `collectEncoder()` (phase 2 — read
      the 4-byte response, no busy-wait) pair, called once per loop tick each
      (request this tick, collect next), matching
      `source_old/hal/real/Motor.cpp`'s `requestEncoder()`/`collectEncoder()`
      byte-for-byte. `tick()` uses the settle-read path
      (`readEncoderSettle`-equivalent: 0x46 write → 4 ms post-write busy-wait
      → 4-byte read, no pre-write idle) for the per-tick position sample,
      exactly as `source_old`'s `Motor::tick()` does via
      `readEncoderSettle()`. `NezhaHal::tick()` ticks the ports in the same
      relative order `source_old/robot/NezhaHAL.h` documents (right-equivalent
      before left-equivalent was the old convention keyed to bus-wedge
      history) — for a port-indexed HAL with no L/R baked in, preserve a
      **fixed, deterministic port order** across ticks (e.g., ascending port
      number) and document it, since the old ordering's purpose was
      determinism, not a specific L/R priority.
- [ ] Encoder offset/reset semantics ported: `resetEncoder()`-equivalent
      (median-of-3 atomic read + readback-verify + retry, matching
      `source_old`'s `kMaxRetries`/`kReadbackThreshold` constants) reachable
      via `MotorCommand.reset_position`. A soft/no-I2C rebaseline
      (`rebaselineSoft`-equivalent) may be ported too but is not required to
      be reachable from `apply()` this sprint if nothing calls it yet —
      note the decision either way.
- [ ] Slew limiting ported: the `MotorSlew::clampStep`-equivalent
      |ΔPWM|-per-write cap, driven by `MotorConfig.slew_rate` (defaulting to
      the existing `kMaxDeltaPwmPerWrite = 25` value — see architecture-
      update.md Design Rationale 2 for why this default, not the newer
      unvalidated zero-dwell fix, is what's ported this sprint). A stop
      (`pct == 0`) remains the explicit, unclamped, immediate-write
      exemption, matching `source_old`.
- [ ] Wedge detection surfaced in `MotorState.wedged` — port whatever
      signal `source_old` uses to populate its analogous field (do not
      invent a new detector; this sprint explicitly excludes any
      `DBG WEDGE`-equivalent diagnostic command, per the issue).
- [ ] `tick(uint32_t now)` executes the staged command per mode: DUTY → slew
      → register write; VELOCITY → embedded PID (encoder-derived filtered
      velocity vs. target; gains/anti-windup/min-duty from `MotorConfig`) →
      duty; POSITION → onboard 0x5D; NEUTRAL; `reset_position` rides beside
      any arm and zeroes the encoder that tick.
- [ ] `apply()` rejects any command mode not present in `capabilities()`
      before touching hardware — proven by `VOLT` on Nezha returning
      `ERR unsupported` at the command layer (ticket 5 wires this to `DEV`,
      but the rejection itself is `NezhaMotor`'s/the base faceplate's
      responsibility).
- [ ] Encoder plumbing and raw register verbs are **private** to
      `NezhaMotor`. Nothing outside this class calls `readEncoderAtomic`,
      `requestEncoder`, `collectEncoder`, or any 0x-register wrapper
      directly — the public surface is the `Hal::Motor` faceplate only.
- [ ] Google/CamelCase style: `namespace Hal`, class `NezhaMotor`
      (UpperCamelCase), methods lowerCamelCase, `.cpp` extension, snake_case
      filenames (`nezha_motor.h`/`nezha_motor.cpp`), trailing-underscore
      private members.

### `NezhaHal` (`source/hal/nezha/nezha_hal.{h,cpp}`)

- [ ] Owns `I2CBus` + up to four `NezhaMotor` instances (one per configured
      port), value members (no heap allocation).
- [ ] `tick(uint32_t now)` orchestrates the split-phase bus schedule across
      however many ports are configured, in the fixed deterministic order
      established above.
- [ ] The dev build (wired in ticket 5's `main.cpp`) instantiates a
      `NezhaMotor` on all four ports so the bench rig can address any of
      them, and so the coupled rig (ports 3+4) is reachable without any
      `NezhaHal`-level special-casing.

### Build

- [ ] `python build.py --clean` succeeds with `NezhaMotor`/`NezhaHal`
      compiled in (even if nothing in `main.cpp` calls them yet — that's
      ticket 5; this ticket may need a minimal smoke call in `main.cpp` or a
      throwaway compile-only reference to prove the translation units build,
      programmer's choice, removed/superseded once ticket 5 lands).

## Testing

- **Existing tests to run**: None in `tests/` yet (ticket 006 creates the
  new tree; this ticket predates it). Do not add tests under `tests_old/`.
- **New tests to write**: None required at this ticket if `tests/` does not
  exist yet when this ticket executes — defer register-map/slew-cap unit
  coverage to ticket 6/7 once `tests/unit/` exists, and note that deferral
  in the PR. If ticket 6 has already landed by the time this ticket
  executes (sprint tickets are meant to run in dependency order, so it
  should not have), add a `tests/unit/` test asserting
  `MotorSlew`-equivalent clamp behavior and the split-phase request/collect
  call sequence via a fake `I2CBus`.
- **Verification command**: `python build.py --clean`. Full behavioral
  verification (does the motor actually spin correctly, does the encoder
  read correctly) is a bench-only concern deferred to ticket 7 — this
  ticket's gate is "compiles, and the ported sequencing matches
  `source_old` byte-for-byte on inspection," not a bench pass.
