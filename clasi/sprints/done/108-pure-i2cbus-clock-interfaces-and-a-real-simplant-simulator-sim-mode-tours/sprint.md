---
id: '108'
title: Pure I2CBus/Clock interfaces and a real SimPlant simulator (sim-mode tours)
status: done
branch: sprint/108-pure-i2cbus-clock-interfaces-and-a-real-simplant-simulator-sim-mode-tours
use-cases:
- SUC-038
- SUC-039
- SUC-040
- SUC-041
- SUC-042
- SUC-043
- SUC-044
issues:
- plan-pure-i2cbus-clock-interfaces-a-real-simplant-simulator.md
- sim-api-ctypes-abi-for-sim-mode-tours.md
- binary-bridge-segment-replace-arms-deleted.md
- color-sensor-apds-probe-success-on-failure.md
- sim-mode-tour-1-fault-baseline-exclusion-mismatch.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 108: Pure I2CBus/Clock interfaces and a real SimPlant simulator (sim-mode tours)

## Goals

Replace the firmware's `#ifdef HOST_BUILD`-forked `I2CBus`/`Clock`
concrete classes with pure interfaces (mirroring the existing
`App::Transport` pattern), and replace the host simulator's scripted-FIFO
fake bus + write-count predictor with one honest simulator (`SimPlant`)
that parses the real Nezha/OTOS wire protocol and integrates real physics.
End state: pressing **Tour 1** in the TestGUI (Sim) drives the real
compiled firmware against `SimPlant` and draws the trace on the canvas —
over an architecture with a clean interface, one honest simulator, and no
`#ifdef` soup.

## Problem

Three compounding problems, per
`clasi/issues/plan-pure-i2cbus-clock-interfaces-a-real-simplant-simulator.md`
(the binding master plan for this sprint):

1. `source/devices/i2c_bus.h` (and `clock.h`) are concrete classes with two
   `#ifdef HOST_BUILD` forks each — test-only scripting machinery lives
   inside the production header, and the ARM build depends on a CMake
   `FILTER EXCLUDE` hack to avoid linking the host fork.
2. The host simulator *predicts* firmware I2C behavior (from a per-cycle
   duty-write count, `SimApi`'s `DutyPredictor`) instead of *responding* to
   what firmware actually puts on the wire — under an arbitrary twist
   stream this desyncs (left encoder freezes, right runs away).
3. Register-level fault injection (13 scenarios) is scripted in bespoke
   C++ harnesses with no Python-callable path, and sim-mode tours have
   never worked (`SimTransport` targets a ctypes ABI over a subsystem graph
   that no longer exists — `sim_conn.py` has been dead code since commit
   `72d8be7e`).

This sprint also resolves three satellite issues discovered investigating
the above: `sim-api-ctypes-abi-for-sim-mode-tours.md` (no ctypes ABI has
ever existed over any current-tree sim harness), `binary-bridge-segment-
replace-arms-deleted.md` (`testgui/binary_bridge.py` targets a deleted
wire-envelope arm — scoped here to what unblocks the Sim path, not a full
rewrite), and `color-sensor-apds-probe-success-on-failure.md` (a status-
ignoring probe read latches "present" on a NAK'd bus).

## Solution

Staged per the master plan (see `architecture-update.md` for full module
boundaries, diagrams, and design rationale):

1. **Stage 1** — Split `I2CBus` into a pure interface + `MicroBitI2CBus`
   real implementation (behavior-preserving).
2. **Stage 2** — Build `SimPlant` (an honest `I2CBus` implementation
   parsing the real wire protocol, owning reused `WheelPlant`/`OtosPlant`
   physics) and `SimHarness` (composes the real `App::RobotLoop` against
   it); migrate the whole-robot scenario tests; prove the divergence bug
   is gone.
3. **Stage 3** — `sim_ctypes` C ABI (including the Python-scripting
   read/write hook) + `sim_loop.py` + `SimTransport` rewire; un-gate Sim
   tour buttons; verify Tour 1 draws in the GUI.
4. **Stage 4** — Migrate the 13 register-level C++ scripted-bus unit tests
   to Python `SimPlant` hook tests (deleting the C++ harnesses); pair with
   the color-sensor probe fix's own regression test.
5. **Stage 5** — Apply the same interface-split pattern to `Clock`/
   `Sleeper`; final end-to-end verification.

## Success Criteria

The master plan's own end-to-end Verification, run at ticket 010's close:

1. `python build.py --fw-only` — ARM firmware still builds.
2. `uv run python -m pytest tests/sim` — full gate green.
3. `grep -rn "HOST_BUILD" source/devices/` — returns nothing.
4. Standalone: a straight twist keeps heading ~0 for the full run
   (divergence bug fixed).
5. Headless: a Tour 1 run through `sim_loop` completes every leg with
   finite/small closure.
6. Manual/bench: `just testgui` → Connect (Sim) → **Tour 1** → the trace
   draws on the canvas.

## Scope

### In Scope

- `source/devices/i2c_bus.h` / `source/devices/clock.h` → pure interfaces;
  `MicroBitI2CBus`/`MicroBitClock`/`MicroBitSleeper` real implementations.
- `tests/_infra/sim/` rebuilt from scratch: `sim_plant.{h,cpp}`,
  `sim_harness.h`, `sim_ctypes.cpp`, `CMakeLists.txt`.
- Deletion of `tests/sim/support/sim_api.{h,cpp}` (+ `DutyPredictor`),
  `host/robot_radio/io/sim_conn.py`, the 13 register-scripting C++ test
  harnesses (migrated to Python).
- New `host/robot_radio/io/sim_loop.py`; `SimTransport` rewired onto it;
  Sim Tour buttons un-gated.
- `Devices::ColorSensorLeaf`'s APDS probe fix (status-returning read).
- `binary_bridge.py`: verify-only scope (GUI launches; `SimTransport`
  independence from the dead `segment`/`replace` builders) — no rewrite of
  its manual command-row translation.

### Out of Scope

- Rewiring `binary_bridge.py`'s manual `D`/`RT`/`R`/`TURN`/`G` GUI command
  rows or `_GotoRunner` onto the twist-based planner surface (deferred
  stakeholder call per `binary-bridge-segment-replace-arms-deleted.md`'s
  own "Recommended direction").
- `comms.h`'s smaller, separate `#ifndef HOST_BUILD` residue around
  `SerialTransport`/`RadioTransport` (the master plan's own Notes/risks —
  optional follow-up, not required for the tour).
- Extracting `TOUR_1`/`TOUR_2` geometry into a `data/tours/*.json` file
  (sprint 107's own Open Question 3 — unrelated to this sprint).

## Test Strategy

**Expected transient CI red window (not a regression):** the 13 register-
scripting sim-unit tests go RED the moment ticket 001 removes the scripted
fake (Stage 1) and come back GREEN as Python hook tests once ticket 009
lands (Stage 4). The ARM firmware build and every non-scripting test stay
green throughout every ticket boundary. If CI shows exactly this pattern
mid-sprint, it is expected and documented here — do not treat it as a
regression to chase early.

Testing mix: C++ standalone/smoke drivers for `SimPlant`/`SimHarness`
before the ctypes ABI exists (tickets 002-004); Python ctypes/pytest tests
once the ABI lands (tickets 005+); one manual/bench GUI check as the final
acceptance gate (ticket 007, re-confirmed at ticket 010's close). Every
ticket that changes `source/` also verifies `python build.py --fw-only`
stays green — Stages 1 and 5 are explicitly behavior-preserving on the ARM
side.

## Architecture Notes

See `architecture-update.md` for full module boundaries, Mermaid diagrams,
and design rationale (the hook lives on `SimPlant` not `I2CBus`; source
placement rule kills the CMake FILTER-EXCLUDE hack structurally; SimPlant
reuses WheelPlant/OtosPlant physics verbatim and owns only the wire
protocol; binary_bridge.py's remaining scope is deliberately deferred).

## GitHub Issues

None linked yet.

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [x] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Split I2CBus into a pure interface + MicroBitI2CBus real implementation | — |
| 002 | SimPlant: the one honest simulator bus (Nezha/OTOS protocol + physics + hooks) | 001 |
| 003 | SimHarness: compose RobotLoop + SimPlant; delete SimApi/DutyPredictor/live_sim | 002 |
| 004 | Migrate whole-robot scenario tests onto SimPlant/SimHarness; verify straight-twist stays straight | 003 |
| 005 | sim_ctypes C ABI over SimHarness/SimPlant + tests/_infra/sim build wiring | 004 |
| 006 | sim_loop.py: TwistTransport-shaped Python object over sim_ctypes; delete dead sim_conn.py | 005 |
| 007 | Rewire SimTransport onto sim_loop; un-gate Sim tour buttons; verify GUI launches and Tour 1 draws | 006 |
| 008 | Fix ColorSensorLeaf APDS probe success-on-NAK + Python hook regression test | 005 |
| 009 | Migrate the 13 register-level unit tests to Python SimPlant hook tests; delete C++ harnesses | 008 |
| 010 | Clock/Sleeper purification: pure interfaces + MicroBit/Sim impls; end-to-end verification | 003, 009 |
| 011 | Dither stopped-wheel plant reads to stop Tour 1 false wedge-latch faults | — |

Tickets execute serially in the order listed. Note: 008 branches off 005
in parallel with 006/007 (both depend only on 005), but is listed after
007 in execution order since 009 depends on it and the master sequence is
otherwise linear; 010 waits on both 003 (sim harness must exist to update)
and 009 (needs the full pytest gate green before its own final
verification pass). 011 is a standalone sim-fidelity fix discovered
during 007's own execution (see `clasi/issues/sim-mode-tour-1-fault-
baseline-exclusion-mismatch.md`) with no blocking dependency on any
prior ticket.
