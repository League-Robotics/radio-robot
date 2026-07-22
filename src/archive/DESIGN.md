# Archive (`src/archive`)

**Owner:** Eric Busboom · **Last reviewed:** 2026-07-21 · **Status:** deprecated (by design)

---

## 1. Purpose

`src/archive/` is parked history: entire pre-rebuild trees and one-off
artifacts kept for reference after the sprint 077 greenfield rebuild
(and later, sprint 108's device-bus bring-up and sprint 115's
motion-stack excision) replaced them. Nothing here has architecturally
significant *current* content — its value is purely historical (what
did the old design look like, what did an old build produce), not
design this project's live subsystems depend on or extend. It exists as
its own directory, excluded from every build and test collection path
(`norecursedirs`, per [`../tests/DESIGN.md`](../tests/DESIGN.md) §3), so
"parked" is enforced mechanically, not just by convention.

## 2. Orientation

- **`source_old/`** — the pre-077-rebuild C++ firmware tree (the
  subsystem/message-dispatch/`CommandProcessor` architecture the current
  single-loop `src/firm/` replaced).
- **`tests_old/`** — the matching pre-077 test tree (see its own
  `CLAUDE.md`).
- **`source_parked/`** — later-vintage parked subsystems (e.g. the `094`
  planner/subsystems code referenced from
  [`../firm/messages/DESIGN.md`](../firm/messages/DESIGN.md) §6 as one of
  `event.h`'s only remaining references).
- **`host_scripts/`** — old standalone calibration scripts
  (`calibrate_linear.py`, `calibrate_angular.py`, `calibrate_verify.py`)
  superseded by `src/host/robot_radio/calibration/`.
- **`wedgelab/`** — a standalone historical bring-up/debug rig (its own
  `build.py`, `CMakeLists.txt`, hex artifacts) predating the current
  build.
- **`hex/`** — archived built firmware images (`.hex`), kept as
  point-in-time flash artifacts, not source.

## 3. Constraints and Invariants

- **Never imported, built, or tested by anything live.** No file under
  `src/firm/`, `src/sim/`, `src/host/`, or `src/tests/` (outside this
  directory itself) may depend on `src/archive/`. If a live subsystem
  ever needs to reference or resurrect archived logic, that is a
  deliberate, reviewed act of un-parking it into a live directory — not
  a live import reaching back into archive.
- **Never hand-edited to "fix" it.** This is a historical snapshot;
  editing it to match current conventions would destroy its value as a
  record of what the pre-rebuild design actually was.

## 4. Design

No internal structure worth describing — this is a set of independent,
frozen historical snapshots, not a subsystem with its own architecture.

## 5. Interfaces

### Exposes

Nothing consumed by any live subsystem (see §3). It is read-only
reference material for a human (or an agent doing historical research,
e.g. tracing what a since-deleted type used to do).

### Consumes

Nothing — self-contained historical snapshots.

## 6. Open Questions / Known Limitations

None — this directory's role (parked history, no live dependents) is
stable by definition.
