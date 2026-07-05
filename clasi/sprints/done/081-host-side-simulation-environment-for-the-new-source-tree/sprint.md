---
id: 081
title: Host-side simulation environment for the new source/ tree
status: done
branch: sprint/081-host-side-simulation-environment-for-the-new-source-tree
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
- SUC-006
issues:
- host-side-simulation-environment-for-the-new-tree-design-write-up.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 081: Host-side simulation environment for the new source/ tree

## Goals

Stand up a real, host-compiled simulation environment for the new `source/`
tree: one shared library (`libfirmware_host`), loaded from Python via
ctypes, running the actual firmware C++ (`CommandProcessor`,
`Subsystems::Drivetrain`, the DEV command family) against two simulated
devices — motors and OTOS — behind an errorless ground-truth plant. This
replaces `tests/sim/`'s single placeholder test with a real regression
suite and gives the new tree the off-hardware, deterministic error-model
coverage the pre-rebuild `tests_old/simulation/` suite used to provide.

## Problem

Sprint 077's greenfield rebuild parked the old simulation environment
(`source_old/hal/sim/`, `tests_old/_infra/sim/`) and the new `source/` tree
has had no simulator since. Every test runs on real hardware (`tests/bench/`,
HITL) or is an ad hoc, CMake-free compiled harness
(`tests/sim/unit/*_harness.cpp`) introduced specifically to defer this work.
A stakeholder-reviewed design write-up
(`clasi/issues/host-side-simulation-environment-for-the-new-tree-design-write-up.md`)
lays out the model, but it predates a same-day rename
(`Hal::NezhaHal` -> `Subsystems::NezhaHardware`) and its own "fold the loop
extraction into sprint 079" resolution, both now stale — see this sprint's
`architecture-update.md` for the full reconciliation.

## Solution

Six sequenced tickets: extract the real velocity PID into a shared,
host-clean class (001); introduce the host-clean loop seams — an abstract
`Subsystems::Hardware` owner base, the extracted `devLoopTick`, and a clock
seam — proven against the existing `Subsystems::NezhaHardware` (002); port
the errorless plant and the two simulated devices behind a new
`Subsystems::SimHardware` (003); build the shared library and its C ABI
(004); wrap it for Python with pytest fixtures and write the first real
tests (005); and port the highest-value legacy error-injection suites (006).
See `architecture-update.md` for the full design, the naming decisions
(`Subsystems::Hardware`/`Subsystems::SimHardware`, not the design write-up's
now-stale `Hal::MotorHal`/`SimHal`), and the dt=0 re-entry-guard finding.

## Success Criteria

- `just build-sim` produces `tests/_infra/sim/build/libfirmware_host.{dylib,so}`.
- `uv run python -m pytest tests/sim` collects and passes a real suite
  (plant correctness, errored-observation split, velocity-PID response,
  protocol round-trips, determinism, watchdog policy, ported legacy
  error-model suites) — not the current placeholder.
- The velocity-PID extraction (001) and the loop-seam extraction (002) are
  each confirmed byte-identical on real hardware via the
  hardware-bench-testing gate before this sprint is considered done.
- No `SIMSET`/`SIMGET` wire command family and no sim-specific `TLM` field
  exists anywhere — every sim knob and ground-truth read is ctypes-only.
- Zero-error determinism: all error knobs at zero -> true encoder ==
  reported encoder == OTOS accumulator, bit-for-bit; an identical script
  run twice produces bit-identical logs.

## Scope

### In Scope

- `Hal::MotorVelocityPid` extraction from `NezhaMotor` (behavior-preserving,
  bench-verified).
- `Subsystems::Hardware` abstract owner base; `source/dev_loop.{h,cpp}`
  extraction; `source/types/clock.h` seam.
- `Hal::PhysicsWorld`, `Hal::SimMotor`, `Hal::SimOdometer`,
  `Subsystems::SimHardware`, `source/hal/sim/sim_setters.h`.
- `tests/_infra/sim/CMakeLists.txt` + `sim_api.cpp` (the C ABI).
- `tests/_infra/sim/firmware.py`, `tests/sim/conftest.py` fixtures,
  `host/robot_radio/io/sim_conn.py` fix-up, first real tests.
- Porting the encoder-error, OTOS-error, and stiction/lag suites from
  `tests_old/simulation/`.

### Out of Scope

- Any `SIMSET`/`SIMGET` wire command or sim-specific `TLM` field (a firm
  design decision, not deferred).
- Fault injection (`connected=false`, `wedged=true`, encoder dropout) —
  already filed as `clasi/issues/later/sim-hardware-fault-injection.md`;
  referenced, not re-filed or implemented, by ticket 003.
- Sensor simulation beyond motors/OTOS (line/color/port truth channels are
  explicitly dropped from the ported `PhysicsWorld`).
- SimTransport/TestGUI revival — the old protocol verbs (T/D/VW) above the
  transport layer don't exist in the new firmware; this is separate, later
  work.
- EKF/fusion-dependent tests — no firmware consumer of OTOS exists yet in
  the new tree.

## Test Strategy

Tickets 001-002 carry a **hardware-bench-testing gate**
(`.claude/rules/hardware-bench-testing.md`): 001's bench step-response
comparison (pre/post PID extraction) and 002's bench smoke pass (ARM build
behavior-identical) are both required, not optional, acceptance items.
Tickets 003 onward are pure host-side: a standalone-compiled harness
convention (matching the existing `tests/sim/unit/*_harness.cpp` pattern,
no CMake) proves ticket 003's dt=0 guard and zero-error determinism before
ticket 004's CMake build exists; tickets 004-006 are verified via
`just build-sim` and `uv run python -m pytest tests/sim`. The default
`uv run python -m pytest` invocation must stay green after every ticket.

## Architecture Notes

See `architecture-update.md` in full. Headline decisions:

- The abstract hardware-owner seam is `Subsystems::Hardware` (not the
  design write-up's `Hal::MotorHal`, a name that referred to symbols
  renamed the same day the design was reviewed) — Decision 1.
- The sim's owner/scheduler is `Subsystems::SimHardware`, a Subsystems-tier
  peer of `Subsystems::NezhaHardware`, not a `Hal::` leaf beside
  `SimMotor`/`SimOdometer` — Decision 2.
- `devLoopTick` takes a nullable `DevLoopStatement*` (not a held/taken
  pair) plus a loop-originated default reply sink for the watchdog `EVT` —
  Decision 3.
- The dt=0 re-entry guard (a repeated `hardware.tick(now)` call at an
  unchanged `now` — which happens on **every ordinary pass**, not only
  during command replay) lives in `SimHardware::tick()`, not inside
  `MotorVelocityPid` — Decision 4. This is a genuine correctness risk this
  document identifies beyond the design write-up's own risk list.
- Ticket order swaps the design write-up's "seams" and "plant + sim
  devices" sequencing, since `SimHardware` (folded into the design's plant
  ticket) needs the abstract base only the seams ticket introduces —
  Decision 5.

**Sizing flag**: tickets 001-003 need the hardware bench; tickets 004-006
are pure host-side work with no hardware dependency at all. If stand
availability makes it awkward to hold one sprint open across both halves,
splitting after ticket 003 lands is a reasonable, low-risk split point —
see `architecture-update.md`'s "Sizing recommendation" section. This
document keeps all six tickets in one sprint since their dependency chain
is already fully serial either way; the choice of whether to split is
deliberately left to the stakeholder, not decided here.

## GitHub Issues

None. This sprint tracks against
`clasi/issues/host-side-simulation-environment-for-the-new-tree-design-write-up.md`
(referenced by every ticket's `issue:` frontmatter field), not a GitHub
issue.

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [x] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Extract shared velocity PID into Hal::MotorVelocityPid | (none) |
| 002 | Host-clean loop seams: Subsystems::Hardware, dev_loop, clock | (none) |
| 003 | Plant and simulated devices: PhysicsWorld, SimMotor, SimOdometer, SimHardware | 001, 002 |
| 004 | Host-sim build and C ABI: CMakeLists and sim_api.cpp | 002, 003 |
| 005 | Python simulation wrapper, pytest fixtures, and first real sim tests | 004 |
| 006 | Port high-value legacy error-injection test suites | 005 |

Tickets execute serially in the order listed. See `architecture-update.md`'s
"Sizing recommendation" for the optional 003/004 split point.
