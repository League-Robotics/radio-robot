---
id: "010"
title: "Clock/Sleeper purification: pure interfaces + MicroBit/Sim impls; end-to-end verification"
status: open
use-cases: ["SUC-039"]
depends-on: ["003", "009"]
github-issue: ""
issue: "plan-pure-i2cbus-clock-interfaces-a-real-simplant-simulator.md"
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Clock/Sleeper purification: pure interfaces + MicroBit/Sim impls; end-to-end verification

## Description

Stage 5 of the master plan — the same interface-split pattern ticket 001
applied to `I2CBus`, now applied to `Devices::Clock`/`Devices::Sleeper`
(`source/devices/clock.h`), the LAST `#ifdef HOST_BUILD` fork in
`source/devices/`. This ticket also carries the sprint's final end-to-end
verification pass (the master plan's own Verification section), since it
is the last ticket in dependency order.

1. `source/devices/clock.h` → pure `Clock`/`Sleeper` interfaces:
   `Clock::nowMicros() const = 0`; `Sleeper::sleepMillis(uint32_t) = 0`,
   `Sleeper::yield() = 0`; virtual dtors. Delete the `#ifdef HOST_BUILD`
   inspection surface (`setMicros`/`advanceMicros`/`sleepCount`/
   `lastSleepMillis`/`yieldCount`) from the interface — it moves to the
   sim concrete class.
2. New `source/devices/microbit_clock.{h,cpp}`: `MicroBitClock : Clock`
   wrapping `system_timer_current_time_us()`; `MicroBitSleeper : Sleeper`
   wrapping `fiber_sleep()`/`schedule()`. Delete `clock_real.cpp`/
   `clock_host.cpp`.
3. `TestSim::SimClock`/`TestSim::SimSleeper` under `tests/_infra/sim/`
   (colocated in `sim_harness.h` or split into their own `sim_clock.
   {h,cpp}` — see architecture-update.md Open Question 2, implementer's
   choice): steppable counter (`setMicros`/`advanceMicros`) + sleep/yield
   counters, mirroring the deleted `clock_host.cpp`'s own inspection
   surface so `sim_harness.h`'s tests keep the same assertion power they
   had before.
4. `main.cpp` updated to construct `MicroBitClock`/`MicroBitSleeper`;
   `sim_harness.h` (ticket 003) updated to construct `SimClock`/
   `SimSleeper` in the loop's `Clock&`/`Sleeper&` slots.
5. `CMakeLists.txt:301`'s `clock_host.cpp` FILTER-EXCLUDE line deleted.

**End-to-end verification** (master plan's own Verification section — run
ALL of these as this ticket's own closing acceptance, now that every prior
ticket has landed):
1. `python build.py --fw-only` — ARM firmware still builds.
2. `uv run python -m pytest tests/sim` — full gate green.
3. `grep -rn "HOST_BUILD" source/` — returns nothing under `devices/`.
4. Standalone: straight twist → heading stays ~0 (ticket 004's own check,
   re-run here as a final confirmation nothing regressed).
5. Headless Tour 1 via `sim_loop` → all legs run, closure finite/small
   (ticket 007's own check, re-confirmed).
6. `just testgui` → Connect (Sim) → **Tour 1** → trace draws on the canvas
   (ticket 007's own bench check, re-confirmed as the sprint's final
   acceptance).

## Acceptance Criteria

- [ ] `source/devices/clock.h` declares only pure-virtual `Clock`/
      `Sleeper` interfaces; no `#ifdef HOST_BUILD`.
- [ ] `source/devices/microbit_clock.{h,cpp}` exist with the real
      implementations; `clock_real.cpp`/`clock_host.cpp` deleted.
- [ ] `TestSim::SimClock`/`SimSleeper` exist under `tests/_infra/sim/` with
      the steppable/inspection surface the sim harness needs.
- [ ] `main.cpp` and `sim_harness.h` construct the correct concrete types.
- [ ] `CMakeLists.txt`'s `clock_host.cpp` FILTER-EXCLUDE line is removed.
- [ ] `grep -rn "HOST_BUILD" source/devices/` returns nothing at all.
- [ ] All 6 end-to-end verification items above pass, in order, and are
      recorded (command output or bench observation) in this ticket's
      completion notes.

## Implementation Plan

**Approach**: Identical mechanical pattern to ticket 001 (pure interface +
real ARM impl + sim impl under `tests/_infra/sim/`), applied to a smaller,
simpler seam. The bulk of this ticket's effort is the closing end-to-end
verification pass, not the Clock/Sleeper split itself.

**Files to create**:
- `source/devices/microbit_clock.h`, `source/devices/microbit_clock.cpp`
- `tests/_infra/sim/sim_clock.{h,cpp}` (or the equivalent surface inside
  `sim_harness.h` — implementer's choice per architecture-update.md Open
  Question 2)

**Files to modify**:
- `source/devices/clock.h` (reduce to pure interfaces)
- `source/main.cpp`
- `tests/_infra/sim/sim_harness.h`
- `CMakeLists.txt`

**Files to delete**:
- `source/devices/clock_real.cpp`, `source/devices/clock_host.cpp`

**Testing plan**:
- `python build.py --fw-only` and `python build.py` (full) both green.
- `uv run python -m pytest tests/sim` fully green.
- Re-run ticket 004's straight-twist check and ticket 007's headless
  Tour 1 + manual bench check as this ticket's own closing verification
  (not new tests — confirmation nothing this ticket touched regressed
  them).
- Verification command:
  `python build.py --fw-only && python build.py && uv run python -m pytest tests/sim`
  plus the manual `just testgui` bench step.

**Documentation updates**: `clock.h`'s file header updated to describe the
new pure-interface shape (mirrors ticket 001's own doc update for
`i2c_bus.h`); sprint.md's own Success Criteria section checked off against
this ticket's 6-item verification list.
