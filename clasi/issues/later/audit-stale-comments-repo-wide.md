---
status: pending
---

# Audit all source comments repo-wide and validate they are still true

## Description

Sprint 108's SimPlant rebuild exposed a stale-comment problem: the header
comment in `src/sim/plant/wheel_plant.h` (written in ticket 105-003) still
asserts the plant is "LEAF-GETTER-DRIVEN, not bus-byte-driven" and "never
intercepts a raw Devices::I2CBus write payload" — which sprint 108
explicitly reversed. `SimPlant::defaultWrite()` now parses the raw Nezha/
OTOS wire frames off the bus, and the `SimApi`/DutyPredictor leaf-getter
path the comment describes was deleted (108-003). The stale comment
actively misled the stakeholder into thinking the I2C-interception work
wasn't being used.

Do a systematic pass over **all comments in the live source trees** and
validate each substantive claim against the current code:

- Design/architecture claims ("per Decision N…", "this class never…",
  "X is the only caller of…", "see Y for where Z lives").
- Cross-references to files, classes, tickets, and sprint artifacts that
  have since been deleted, moved (e.g. the `src/` tree unification), or
  superseded.
- Behavioral claims ("this returns…", "callers must…", conventions cited
  from other files' comments).

For each false or superseded claim: fix the comment to describe the
current design (citing the superseding sprint/decision where useful), or
delete it if it no longer earns its keep. Do NOT weaken true, load-bearing
comments — the goal is accuracy, not comment reduction.

## Known instances to start from

- `src/sim/plant/wheel_plant.h` header (105-era "leaf-getter-driven /
  never intercepts a raw I2C write" claim — superseded by 108
  Decisions 1–3; SimPlant now owns wire-protocol parsing, WheelPlant is a
  pure physics model fed from parsed writes).
- Anything else citing sprint 105's Decision 2 or the deleted scripted-FIFO
  `I2CBus` fake / `SimApi` / DutyPredictor as if they still exist.
- Comments predating the `src/` tree unification that reference old
  `source/`/`tests/` paths.

## Scope

Live trees only: `src/`, `host/`, `tests/` (or their post-unification
equivalents). Parked trees (`source_old/`, `tests_old/`) are out of scope.
