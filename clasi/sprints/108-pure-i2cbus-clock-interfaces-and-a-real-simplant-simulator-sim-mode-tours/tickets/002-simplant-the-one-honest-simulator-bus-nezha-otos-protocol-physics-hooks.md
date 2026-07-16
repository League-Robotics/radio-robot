---
id: "002"
title: "SimPlant: the one honest simulator bus (Nezha/OTOS protocol + physics + hooks)"
status: open
use-cases: ["SUC-040", "SUC-041"]
depends-on: ["001"]
github-issue: ""
issue: "plan-pure-i2cbus-clock-interfaces-a-real-simplant-simulator.md"
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# SimPlant: the one honest simulator bus (Nezha/OTOS protocol + physics + hooks)

## Description

Stage 2 (part a) of the master plan. Build `SimPlant`, a real `I2CBus`
implementation (depends on ticket 001's pure interface) that RESPONDS to
what firmware actually puts on the wire instead of predicting it from a
write count (the old `SimApi`/`DutyPredictor` desync bug this sprint
replaces).

Create `tests/_infra/sim/sim_plant.{h,cpp}`: `class SimPlant : public
Devices::I2CBus`.

- Owns two `TestSim::WheelPlant` + one `TestSim::OtosPlant` — REUSE the
  existing physics from `tests/sim/plant/{wheel,otos}_plant.{h,cpp}`
  verbatim (do not re-derive tuning). `SimPlant` owns the *protocol*, not
  the physics.
- `defaultWrite(addr, data, len)`: dispatch by wire address.
  - Motor `0x10` (8-byte Nezha frame `[0xFF,0xF9,port,dir,0x60,speed,0xF5,
    0x00]`): `0x60` → parse dir+speed into that port's duty in [-1,1]
    **off the wire** — no `appliedDuty()` back-channel of any kind; `0x46`
    → remember the selected port for the next read.
  - OTOS `0x17`: track the register pointer written; swallow init writes.
  - Color/line addresses: return a NAK-equivalent status (matches the real
    bus's behavior for an absent/uninitialized device — this feeds
    ticket 008's regression test).
- `defaultRead(addr, data, len)`: motor → the selected port's accumulated
  encoder as 4-byte LE int32 tenths-of-mm; OTOS → product-ID `0x5F` (reg
  `0x00`) or the 12-byte pos/vel burst (reg `0x20`: position LSB 0.305mm,
  heading LSB 0.00549deg) per the currently-tracked register pointer.
- `read()`/`write()` (the `I2CBus` overrides): hook-wrapper structure per
  the master plan's Target architecture —
  ```
  int SimPlant::read(...)  { return readHook_  ? readHook_(...)  : defaultRead(...); }
  int SimPlant::write(...) { return writeHook_ ? writeHook_(...) : defaultWrite(...); }
  ```
  `defaultRead`/`defaultWrite` NEVER re-enter the hook (no recursion).
- `tick(dt)`: step both `WheelPlant`s from their parsed duty, step the
  `OtosPlant`, accumulate encoder counts. Called once per cycle by the
  harness (ticket 003), not by `SimPlant` itself.
- Fault knobs as plain methods: reuse `WheelPlant::setDisconnected`/
  `freezePosition`/`setDropoutRate`; add OTOS noise/drift knobs on
  `OtosPlant` if not already present. These are plain C++ methods on
  `SimPlant`/its owned plants — NOT on `I2CBus`.
- `setReadHook(fn)`/`setWriteHook(fn)` (clear via `nullptr`): register the
  Python-facing hook. `clearanceSafetyNetCount()` (the one non-write/read
  `I2CBus` member) returns 0 always — `SimPlant` never trips the real
  bus's clearance safety net; there is no spinning wait to trip.

## Acceptance Criteria

- [ ] `tests/_infra/sim/sim_plant.{h,cpp}` exist; `SimPlant : public
      Devices::I2CBus`; compiles as a HOST_BUILD-style host binary (no ARM
      dependency).
- [ ] `defaultWrite()` correctly parses the Nezha `0x60` duty-write and
      `0x46` encoder-select frames and the OTOS register-pointer writes;
      `defaultRead()` returns live physics-integrated encoder/pose bytes,
      never a back-channel `appliedDuty()` read.
- [ ] `read()`/`write()` call the registered hook when present, the
      `default*` path otherwise; `defaultRead`/`defaultWrite` never call
      back into the hook.
- [ ] `tick(dt)` steps both `WheelPlant`s and the `OtosPlant` from
      wire-parsed duty only.
- [ ] Fault-injection knobs (disconnect, freeze, dropout, OTOS noise/drift)
      exist as plain methods on `SimPlant`/its owned plants, not on
      `I2CBus`.
- [ ] Naming conforms to `.claude/rules/naming-and-style.md` (CamelCase,
      no unit-suffixed identifiers — units in `// [unit]` comment tags).

## Implementation Plan

**Approach**: Build `SimPlant` as a standalone, harness-independent unit —
it should compile and be unit-testable in isolation (a `write()`/`read()`
pair against a known frame, asserting the right internal state changes)
before ticket 003 wires it into a full `App::RobotLoop` graph.

**Files to create**:
- `tests/_infra/sim/sim_plant.h`
- `tests/_infra/sim/sim_plant.cpp`

**Files to reuse (read, do not modify physics)**:
- `tests/sim/plant/wheel_plant.{h,cpp}`
- `tests/sim/plant/otos_plant.{h,cpp}`

**Testing plan**:
- New: a small standalone test/driver (can be a scratch `main()` or an
  early Python hook test stub, whichever ticket 005's ctypes ABI makes
  available first — this ticket does not need pytest wiring, that lands in
  ticket 005) exercising: write a `0x60` frame with known dir/speed, `tick`
  once, read back the encoder and confirm it moved the expected direction;
  write an OTOS reg-0x00 pointer then read and confirm `0x5F` comes back;
  register a write hook that swallows a duty write and confirm the wheel
  does NOT move.
- No `tests/_infra/sim/` CMake build exists yet (ticket 005 creates it) —
  this ticket's own verification can compile `sim_plant.cpp` + the reused
  plant `.cpp` files directly with the host C++ compiler as a throwaway
  smoke build; do not block on the full harness/ABI existing yet.

**Documentation updates**: file-header doc comment on `sim_plant.h`
explaining the hook-wrapper structure and the "protocol here, physics in
WheelPlant/OtosPlant" boundary, matching this sprint's architecture-
update.md Step 3 module description.
