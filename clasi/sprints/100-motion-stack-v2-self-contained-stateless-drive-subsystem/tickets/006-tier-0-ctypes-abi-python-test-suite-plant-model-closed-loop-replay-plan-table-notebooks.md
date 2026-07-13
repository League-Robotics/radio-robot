---
id: '006'
title: Tier-0 ctypes ABI + Python test suite (plant model, closed loop, replay, plan-table notebooks)
status: done
use-cases: [SUC-001, SUC-002, SUC-004, SUC-005, SUC-006, SUC-007]
depends-on: ['005']
github-issue: ''
issue: motion-stack-v2-a-self-contained-stateless-motion-control-subsystem.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Tier-0 ctypes ABI + Python test suite (plant model, closed loop, replay, plan-table notebooks)

## Description

Build `tests/_infra/drive/drive_api.cpp` (ctypes C ABI over
`Drive::Drivetrain`/`Drive::MotionPlan`, mirroring `tests/_infra/sim/
sim_api.cpp`'s proven shape) and the tier-0 Python test suite: the plant
model of the level-2 velocity servo, purity/property tests, the replay
harness, and plan-table notebooks. This is the first point `source/
drive/` becomes independently observable from Python, and the primary
verification instrument for the rest of this sprint (tier 0 is cheaper
and more complete than any higher tier, per the issue's four-tier
ladder).

If tickets 004/005 stood up temporary, ticket-scoped test scaffolding
(per their own "if 006 has not landed yet" allowance), THIS ticket
reconciles those tests onto the real tier-0 infrastructure and removes
the temporary scaffolding — check both tickets' completion notes first.

## Acceptance Criteria

- [x] `tests/_infra/drive/drive_api.cpp` exposes (mirroring
      `sim_api.cpp`'s `extern "C"` pattern): create
      `Drivetrain(limits, trackwidth)`, destroy, `plan(request)` ->
      `PlanResult`, `admit(goal, tail)` -> `Verdict`, `referenceAt(plan,
      t)` -> `RefState` (table dump), `step(plan, input, state*)` ->
      `StepOutput` with `StepState` passed/returned as a ctypes struct,
      `replan(plan, measured, elapsed)` -> `PlanResult`, `planVelocity
      (...)` -> `PlanResult`. Builds into `libdrive_host` via a new
      `tests/_infra/drive/CMakeLists.txt` mirroring `tests/_infra/sim/
      CMakeLists.txt`.
- [x] `tests/_infra/drive/drive.py` (ctypes loader, mirrors `tests/
      _infra/sim/firmware.py`'s `Sim` class shape): a `Drive` class
      wrapping the ABI with Python-friendly types (dataclasses or
      equivalent for `Pose`/`Twist`/`Limits`/`StepState`/`StepOutput`/
      `TrackRecord`).
- [x] A Python plant model (location: `tests/_infra/drive/` or a new
      `tests/sim/drive/` — programmer's judgment, document choice)
      implements first-order lag 120-140ms, stiction, encoder staleness
      ~80ms, quantization, and slip as independently configurable knobs,
      conceptually aligned with the sim's own fault-knob philosophy
      (`motor_lag`/`enc_slip`/`stiction`/`trackwidth`/`scrub` on the
      `Sim` class) so tier-0 and tier-1 plant behavior stay comparable.
- [x] Closed-loop convergence tests: an arc and a pivot segment converge
      to `DONE_STOP` within the plant model's lag/stiction range, using
      the issue's gains (`k_θ`=6.0, `k_c`=1.5e-5, `k_s`=2.0, `k_d`=0).
- [x] Purity/property tests (SUC-002): determinism (same inputs ->
      identical output, across arc/pivot/velocity-mode plans);
      `StepState` round-trips through the ctypes struct boundary
      unchanged when nothing in the tick would alter it; a fuzz test
      over `>=1000` generated `StepInput`/config combinations asserts
      zero `NaN`/`Inf` in `StepOutput`.
- [x] Replay harness: given a recorded `TrackRecord.in` sequence (from
      ANY tier — sim, bench, field), replay it through `step()` and
      reproduce the recorded output bit-exact. This is the mechanism
      every higher-tier ticket (007/010/011/012) relies on to isolate a
      defect to `source/drive/` itself vs. the adapter/plant.
- [x] Plan-table notebook(s) under `tests/notebooks/`: at least one arc
      and one pivot plan table (`referenceAt()` dump) plotted, before any
      `step()` call — the "interpretability deliverable at its purest"
      (SUC-001).
- [x] A static/source check (or, if impractical, an explicit code-review
      note in completion notes) confirms no method in `source/drive/`
      mutates a `const MotionPlan&` or reads global/static mutable
      state.
- [x] `uv run python -m pytest` passes; the full tier-0 suite runs in
      well under a second per run per the issue's "milliseconds per run"
      characterization — record actual wall-clock time in completion
      notes.

## Testing

- **Existing tests to run**: `uv run python -m pytest`; every prior
  ticket's C++ unit harnesses (002-005) must stay passing.
- **New tests to write**: as listed in Acceptance Criteria — this
  ticket's own tests ARE the sprint's primary test infrastructure.
- **Verification command**: `uv run pytest`

## Implementation Plan

**Approach**: mirror `tests/_infra/sim/sim_api.cpp` + `tests/_infra/sim/
firmware.py`'s proven pattern as closely as possible — same build
mechanism (`CMakeLists.txt` -> a `.dylib`/`.so`), same ctypes
struct-passing conventions, same context-manager Python class shape. Do
not invent a new pattern where the existing one already solves the
problem.

**Files to create**:
- `tests/_infra/drive/drive_api.cpp`
- `tests/_infra/drive/CMakeLists.txt`
- `tests/_infra/drive/drive.py`
- A plant-model module (location per programmer's judgment, documented)
- Python test files for purity/property/closed-loop/replay (under
  `tests/sim/unit/` or a new `tests/sim/drive/`, matching the existing
  `tests/CLAUDE.md` domain split)
- `tests/notebooks/` plan-table notebook(s)

**Testing plan**: as listed in Acceptance Criteria.

**Documentation updates**: a doc-comment header on `drive_api.cpp`
explaining the tier-0 workflow (how to build `libdrive_host`, how to run
the suite), mirroring `sim_api.cpp`'s own extensive header-comment
convention — no separate `docs/` page required.
