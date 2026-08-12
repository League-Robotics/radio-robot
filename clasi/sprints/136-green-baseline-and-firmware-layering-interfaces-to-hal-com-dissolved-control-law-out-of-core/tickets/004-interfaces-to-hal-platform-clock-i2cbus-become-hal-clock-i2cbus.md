---
id: '004'
title: Interfaces to hal/ -- Platform::Clock/I2CBus become Hal::Clock/I2CBus
status: open
use-cases: ["SUC-002"]
depends-on: ["003"]
github-issue: ''
issue: firmware-layering-cleanup-interfaces-to-hal-impls-to-platform-microbit.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Interfaces to hal/ -- Platform::Clock/I2CBus become Hal::Clock/I2CBus

## Description

Phase 2 of the layering-cleanup issue. `platform/clock.h` and
`platform/i2c_bus.h` are pure interfaces (abstract bases, no chip
knowledge, `namespace Platform`) sitting in the directory whose job is
per-target *implementations*. Move them into `hal/`, renaming
`Platform::Clock`/`Sleeper`/`I2CBus` to `Hal::Clock`/`Sleeper`/`I2CBus`.

**This is a move-and-rename of the interfaces only.** The per-target
implementations keep their own names and their own `Platform::`/
`TestSim::` namespaces — `Platform::MicroBitClock`/`MicroBitSleeper`/
`MicroBitI2CBus` (in `platform/microbit/`) and `TestSim::SimClock`/
`SimSleeper`/`SimPlant` (in `platform/host/`) simply re-base onto the
relocated `Hal::` interfaces instead of the old `Platform::` ones — their
class names and namespaces do not change.

This makes `platform/` an implementor of HAL interfaces for the first
time, so `test_layer_isolation.py`'s rule table must change in the same
ticket, not after — that test's own rule change is this phase's proof of
completion, per the source issue's "what each phase proves" note:

```python
_LAYERS = (
    ("platform", ("platform/", "hal/")),
    ("hal",      ("hal/",)),
    ("hardware", ("hardware/", "hal/", "platform/")),
)
```

## Acceptance Criteria

- [ ] `hal/clock.h` (moved from `platform/clock.h`) declares `Hal::Clock`,
      `Hal::Sleeper`.
- [ ] `hal/i2c_bus.h` (moved from `platform/i2c_bus.h`) declares
      `Hal::I2CBus`.
- [ ] `Platform::MicroBitClock`/`MicroBitSleeper`/`MicroBitI2CBus` (in
      `platform/microbit/`) now inherit from `Hal::Clock`/`Hal::Sleeper`/
      `Hal::I2CBus` — their own class names and `Platform::` namespace
      are unchanged; only their base interface's location/namespace
      changed.
- [ ] `TestSim::SimClock`/`SimSleeper`/`SimPlant` (in `platform/host/`)
      re-based the same way.
- [ ] All firm includes of the old `platform/clock.h`/`platform/i2c_bus.h`
      paths updated to `hal/clock.h`/`hal/i2c_bus.h` — confirmed by a
      repo-wide grep for the old paths returning zero hits, excluding
      `.claude/worktrees/rogo-revival/`.
- [ ] Both test harnesses referencing the old paths updated.
- [ ] `src/tests/sim/unit/test_layer_isolation.py`'s `_LAYERS` tuple
      updated so `platform` allows `("platform/", "hal/")`; `hal`/
      `hardware` rows unchanged.
- [ ] `test_layer_isolation.py` passes against the moved tree.
- [ ] `just build-sim` and the ARM build both clean.
- [ ] Any explicit (non-glob) build/test source list naming
      `platform/clock.h`/`platform/i2c_bus.h` by path found and updated
      (headers aren't normally in a `_SRC`-family list, but confirm
      rather than assume).

## Testing

- **Existing tests to run**: `just build-sim`, `uv run python -m pytest
  src/tests/sim/unit/test_layer_isolation.py`, ARM build
  (`uv run python3 build.py`).
- **New tests to write**: none — pure move + rename, existing coverage
  content is unaffected.
- **Verification command**: `just build-sim && uv run python -m pytest
  src/tests/sim/unit/test_layer_isolation.py`
