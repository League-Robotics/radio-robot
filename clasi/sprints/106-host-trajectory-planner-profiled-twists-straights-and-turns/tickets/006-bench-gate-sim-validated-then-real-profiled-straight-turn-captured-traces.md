---
id: '006'
title: 'Bench gate: sim-validated then real profiled straight + turn, captured traces'
status: open
use-cases:
- SUC-030
depends-on:
- '001'
- '002'
- '003'
- '004'
- '005'
github-issue: ''
issue:
- host-planner-design-lessons-from-drive-v2-review.md
- heading-loop-output-clamp-and-velocity-resonance.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Bench gate: sim-validated then real profiled straight + turn, captured traces

## Description

This sprint's own Definition of Done. Depends on every prior ticket
(001-005): the cadence fix, the tamed inner loop, the sim decay-window fix,
the pure profiler, and the streaming executor + heading loop all exist and
are individually verified before this ticket runs them together end to end.

Phase 1 (sim-validated first, per sprint.md's own success criteria): new
`tests/sim/system/` scenario(s) exercise a profiled straight leg and a
profiled in-place turn against `SimApi` (ticket 003's generalized plant
scripting makes a full closed-loop settle observable), asserting completion,
the expected ramp shape, and no fault bits — using the REAL profile
generator/executor logic, not a reimplemented test-only model.

Phase 2 (real bench proof): the SAME profiled straight and profiled turn are
executed for real on the bench rig
(`.claude/rules/hardware-bench-testing.md`), capturing the full streamed
telemetry trace (commanded vs. measured velocity and heading over time) to
`tests/bench/out/`. A human reviews the captured trace and records a
pass/fail judgment on whether the acceleration/deceleration phases are
clean — no visible resonance ringing, matching ticket 002's own `<~10%`
overshoot bar — in this ticket's own Completion Notes. The actual chart
production is sprint 107's deliverable; this ticket produces the raw
material.

A profiled arc (simultaneous `v_x` + `omega`) leg is an explicit STRETCH goal
only (sprint.md's own Success Criteria: "if the ticket structure allows") —
not required for this ticket's own completion.

## Acceptance Criteria

- [ ] Sim scenario(s) for a profiled straight and a profiled turn pass under
      `uv run python -m pytest`, exercising the REAL `planner/profile.py` +
      `planner/executor.py` logic against `SimApi` (not a reimplemented
      test-only model of either).
- [ ] The same profiled straight and profiled turn are run for real on the
      bench stand; a captured telemetry trace (CSV/JSON under
      `tests/bench/out/`) records commanded vs. measured velocity and
      heading over the full run.
- [ ] A human reviewing the captured trace judges the acceleration and
      deceleration phases clean — no visible resonance ringing (matching
      ticket 002's `<~10%` overshoot bar) — recorded as an explicit
      pass/fail judgment in this ticket's own Completion Notes.
- [ ] Every device the run touches (motors, encoders, telemetry link) is
      confirmed alive per `.claude/rules/hardware-bench-testing.md`'s
      standing verification gate (sensors alive, wheels drive both
      directions with encoders incrementing proportionally, round-trip
      confirmed over the real link).
- [ ] Heading correction (ticket 005) holds the profiled straight/turn
      within a stated tolerance, recorded numerically in Completion Notes —
      not merely "looked fine."
- [ ] (Stretch, only if time/ticket sequencing allows) a profiled arc leg is
      also run and captured — explicitly optional, not required for this
      ticket or this sprint to be considered complete.
- [ ] Full project test suite green (`uv run python -m pytest`).

## Testing

- **Existing tests to run**: full `uv run python -m pytest`, in particular
  every ticket 001-005 test suite this ticket's own scenarios build on top
  of.
- **New tests to write**: `tests/sim/system/` profiled-straight and
  profiled-turn scenarios (Phase 1); the bench script itself (Phase 2) is
  manually run, not a pytest-automatable check, but its OUTPUT (the captured
  trace file) is an artifact this ticket's Completion Notes reference.
- **Verification command**: `uv run python -m pytest tests/sim/system/ -v`
  for Phase 1; the new bench script (manual, per
  `.claude/rules/hardware-bench-testing.md`) for Phase 2.

## Implementation Plan

**Approach**: Two phases, in order.

*Phase 1 — sim*: add scenario(s) under `tests/sim/system/` that construct a
`SimApi`, generate a straight-leg profile and a turn profile via
`planner/profile.py`, and drive them through — either the real
`planner/executor.py` against a Python-facing sim transport (if one exists
by this point; `architecture-update.md` (105) Decision 4 explicitly
deferred `io/sim_conn.py` to sprint 107, so this ticket likely instead
injects the SAME setpoint sequence `planner/profile.py` would generate
directly into `SimApi.injectTwist()` calls, exercising `SimApi`'s plant +
`RobotLoop` against a realistic profiled sequence without requiring a full
Python-to-sim transport this sprint) — asserting profile completion,
expected ramp shape (accel/cruise/decel visible in decoded telemetry), and
zero fault bits throughout. Exact wiring is this ticket's own implementation
call; document the choice in Completion Notes.

*Phase 2 — bench*: a new `tests/bench/` script (e.g.
`profiled_motion_verify.py`) constructs a `PlannerParams`, builds a
straight-leg profile and a turn profile via `planner/profile.py`, runs each
through `planner/executor.py` against the real robot (direct USB via
`SerialConnection`, following `rig_soak.py`'s own STOP-in-`finally` safety
convention — there is no `DEV`-watchdog-widen equivalent on the P4 wire to
mirror `bench_ruckig_motion_verify.py`'s older pattern), and writes the
captured commanded-vs-measured trace to `tests/bench/out/`. Run on the
bench stand per `.claude/rules/hardware-bench-testing.md`; review the trace
and record the pass/fail judgment.

**Files to create**:
- `tests/sim/system/` profiled-straight and profiled-turn scenario file(s).
- `tests/bench/profiled_motion_verify.py` (or similar name, implementer's
  call).

**Files to modify**: none beyond what tickets 001-005 already changed.

**Testing plan**: sim scenarios run under `uv run python -m pytest
tests/sim/system/ -v` as part of the standing suite; the bench script is run
manually per the hardware-bench-testing rule, with its captured trace
file(s) and the human pass/fail judgment recorded in this ticket's own
Completion Notes.

**Documentation updates**: Completion Notes record the captured trace file
path(s) (for sprint 107 to consume), the numeric heading-tolerance result,
the pass/fail resonance judgment, and whether the stretch-goal arc leg was
attempted. `tests/CLAUDE.md` updated if the bench script family gains a
new, permanent documented entry point (mirroring 105-006's own precedent of
updating that file when the sim/bench tier's shape changes).
