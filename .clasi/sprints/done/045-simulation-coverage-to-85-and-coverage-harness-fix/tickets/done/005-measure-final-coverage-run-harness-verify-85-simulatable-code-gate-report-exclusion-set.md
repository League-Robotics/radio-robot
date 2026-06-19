---
id: '005'
title: "Measure final coverage: run harness, verify \u226585% simulatable-code gate,\
  \ report exclusion set"
status: done
use-cases:
- SUC-006
depends-on:
- '001'
- '002'
- '003'
- '004'
github-issue: ''
issue: ''
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 045-005: Measure final coverage: run harness, verify ≥85% simulatable-code gate, report exclusion set

## Description

After T001–T004 are complete, run the fixed `coverage.sh` harness end-to-end to
measure the final coverage numbers. Record the result. If the 85% gate is not met,
identify the remaining largest uncovered reachable paths and add tests to close the
gap before declaring the sprint done.

This is the closing verification ticket. It does not write new test files ahead of
time — it runs the harness, reads the per-file table, and either declares success
or adds targeted fill tests to reach the gate.

## Acceptance Criteria

- [x] `bash tests/_infra/coverage.sh --fail-under 85` exits 0 — meaning either:
  - (a) Overall `source/` line coverage is ≥85%, OR
  - (b) Simulatable-code coverage is ≥85% AND a clear note in the coverage.sh output
    identifies the excluded CODAL-only files and their uncovered line counts.
- [x] The full test suite passes with the exact baseline count or higher (no tests deleted).
  `uv run --with pytest python -m pytest tests/simulation -q` exits 0.
- [x] Golden-TLM byte-exact: `uv run --with pytest python -m pytest tests/simulation/unit/test_golden_tlm.py -q` passes.
- [x] Field-pin gate: `uv run --with pytest python -m pytest tests/simulation/unit/test_default_config_pin.py -q` passes.
- [x] Vendor grep gate: `uv run --with pytest python -m pytest tests/simulation/unit/test_vendor_confinement.py -q` passes.
- [x] Final coverage numbers (overall % and simulatable %) are reported back to team-lead in the ticket closing comment.
- [x] CODAL-only exclusion set is finalized in `coverage.sh` (any additions from T002 RatioPidController audit incorporated).

## Implementation Plan

### Approach

1. Run `bash tests/_infra/coverage.sh` and capture output.
2. Read the per-file table. Identify the top-5 files still below 80% that are
   reachable (not CODAL-only).
3. If overall or simulatable-code coverage is already ≥85%: done; record numbers.
4. If below 85%: write fill tests targeting the specific uncovered lines in the
   top remaining files. Repeat until gate is met.
5. Run `bash tests/_infra/coverage.sh --fail-under 85` and confirm exit 0.
6. Run the four hard gates (full suite, golden-TLM, field-pin, vendor grep).
7. Document final numbers in the closing comment.

### Likely fill targets (if gap remains after T002–T004)

These are the candidates most likely to still have uncovered lines after T002–T004:

- `source/app/MotionCommandHandlers.cpp` — large file (144 unc at baseline); T003
  covers error paths but some parser branches may remain.
- `source/control/StopCondition.cpp` — depends on sensor injection; COLOR/LINE_ANY
  branches may need additional test cases if sim_api wrappers were not added.
- `source/app/SystemCommands.cpp` — many CODAL-only lines inflate the uncovered count;
  these are in the exclusion set.

For fill tests: write focused one-function tests in an existing coverage test file
(e.g., `test_motion_handlers_coverage.py`) rather than creating new files.

### Files potentially modified

- Existing sprint-045 test files (add fill tests as needed)
- `tests/_infra/coverage.sh` — finalize exclusion set comment if T002 adds entries

### Testing plan

Run the full sequence:
```bash
bash tests/_infra/coverage.sh --fail-under 85
uv run --with pytest python -m pytest tests/simulation -q
```

Both must exit 0.

### Documentation updates

- Record final coverage percentages and exclusion set in this ticket's closing comment.
- Update `coverage.sh` header comment with the final baseline numbers.

---

## Closing Comment (045-005, 2026-06-19)

### Final Coverage Numbers

| Metric | Lines Covered | Lines Total | % |
|---|---|---|---|
| Overall `source/` | 4239 | 5201 | **81.5%** |
| Simulatable-code | 4051 | 4702 | **86.2%** |

Gate: `bash tests/_infra/coverage.sh --fail-under 85`
Result: `PASS: simulatable-code coverage 86.2% meets --fail-under 85%`

### Gate Results

- `uv run --with pytest python -m pytest tests/simulation -q` — **2093 passed** (exit 0)
- `uv run --with pytest python -m pytest tests/simulation/unit/test_golden_tlm.py -q` — **1 passed** (exit 0)
- `uv run --with pytest python -m pytest tests/simulation/unit/test_default_config_pin.py -q` — **1 passed** (exit 0)
- `uv run --with pytest python -m pytest tests/simulation/unit/test_vendor_confinement.py -q` — **2 passed** (exit 0)

### Finalized CODAL-only + Dead-code Exclusion Set

Files excluded from the simulatable-code denominator (confirmed in `tests/_infra/coverage.sh`):

| File | Reason |
|---|---|
| `source/app/DebugCommandable.cpp` | HOST_BUILD stubs only; I2C handlers guarded by `#ifndef HOST_BUILD` |
| `source/control/PortController.cpp` | NezhaHAL hardware I/O, not sim-exercisable |
| `source/control/ServoController.cpp` | Hardware PWM output, not sim-exercisable |
| `source/io/real/*` | Real device drivers, absent from host lib |
| `source/app/WedgeTest.cpp` | CODAL-only diagnostic (`#ifndef HOST_BUILD`) |
| `source/control/LoopScheduler.cpp` | CODAL scheduler (MicroBit fiber APIs) |
| `source/main.cpp` | CODAL entry point, not in host lib |
| `source/io/real/BenchOtosSensor.cpp` | Bench-only, physical OTOS over I2C |
| `source/control/RatioPidController.cpp` | **Dead code** (045-002): removed from live control loop by N13/030-010; zero call sites in codebase |

### What Remains Uncovered (per-file summary)

The 13.8% gap in simulatable-code coverage is concentrated in these areas:

1. **`#ifndef HOST_BUILD` hardware paths** (throughout `source/`): Guarded CODAL
   device-access blocks that compile out under `HOST_BUILD`. These are structurally
   unreachable from simulation — correct behavior, not a gap.

2. **`source/control/MotorController.cpp`** (dead single-wheel ZOH and `startDrive*`
   entry points): The zero-order-hold single-wheel path and legacy `startDrive*`
   methods exist but are never called by the live `Superstructure`/`Drive` stack.

3. **`source/superstructure/MotionController.cpp`** (~78%): Complex multi-branch
   planner; some rare state transitions (deceleration corner cases, multi-segment
   handoffs) not fully exercised.

4. **`source/subsystems/drive/Drive.cpp`** (~56%): Several `startDrive*` overloads
   and ZOH single-wheel paths that are dead in the current architecture.

5. **`source/app/SystemCommands.cpp`**: Mixed file — RESET and `#ifndef HOST_BUILD`
   dispatch paths are unreachable; testable paths are included in the simulatable
   denominator. File-granularity exclusion cannot split them.

6. **`Kind::SENSOR` queue-drop branch** in the command queue: The branch that drops
   sensor-kind commands when the queue is full is structurally hard to hit under
   normal test load.

7. **`source/state/PhysicsWorld.cpp` — `PhysicsWorld::reset()`**: Never called from
   any test or from the sim's normal reset path; only reachable via a direct call.

All remaining uncovered lines are either hardware-gated (`#ifndef HOST_BUILD`),
dead code in the current architecture, or rare error paths. None are blocking for
the 85% simulatable gate.

### coverage.sh Header Updated

`tests/_infra/coverage.sh` header comment updated from `~74.6% overall` to
`81.5% overall / 86.2% simulatable-code (Sprint 045 final baseline)`.
