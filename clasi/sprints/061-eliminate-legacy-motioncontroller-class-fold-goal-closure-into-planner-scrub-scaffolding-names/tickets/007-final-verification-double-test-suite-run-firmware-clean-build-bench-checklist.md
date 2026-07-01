---
id: '007'
title: 'Final verification: double test-suite run, firmware clean build, bench checklist'
status: open
use-cases:
- SUC-007
depends-on:
- "006"
github-issue: ''
issue: internalize-legacy-motioncontroller-into-planner.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 007 — Final verification: double test-suite run, firmware clean build, bench checklist

## Description

This is the final ticket of the sprint. It performs no structural code changes —
its purpose is validation and documentation. After tickets 001-006 the
implementation is complete. This ticket:

1. Runs the full host test suite TWICE and confirms stability.
2. Produces a clean firmware binary via `build.py --clean`.
3. Verifies the `MICROBIT.hex` is a fresh build (not a stale incremental artifact).
4. Creates `tests/bench/061_bench_checklist.md` for the stakeholder's bench
   validation on the physical tovez robot.

The sprint branch is left open after this ticket. The stakeholder bench-tests
on the branch before any merge to master.

### Step-by-step

1. **Run host suite twice**:
   ```
   uv run python -m pytest 2>&1 | tee /tmp/061_run1.txt
   uv run python -m pytest 2>&1 | tee /tmp/061_run2.txt
   diff /tmp/061_run1.txt /tmp/061_run2.txt
   ```
   Both runs must pass with only the 2 known baseline failures
   (`test_tovez_validates_against_schema`, `test_default_robot_config_unchanged`).
   The diff must show no flapping tests.

2. **Firmware clean build**:
   ```
   cd firmware   # or wherever build.py lives
   python build.py --clean
   ```
   Must exit with code 0.

3. **Verify MICROBIT.hex**:
   Decode the build banner from `MICROBIT.hex` to confirm it is a fresh
   build and not a stale incremental artifact. Per the `stale-incremental-build`
   knowledge note: `build.py --clean` is required; the build banner in the hex
   is the verification. Check the hex metadata or use the project's hex-decode
   utility to confirm the sprint-branch version stamp.

4. **Create `tests/bench/061_bench_checklist.md`**:
   Document the manual bench-validation steps for the stakeholder on physical
   tovez. The checklist covers each motion mode changed in this sprint
   (all modes go through `Planner` now). Format: tabular, with command,
   expected EVT, and a checkbox for the stakeholder to mark.

   Commands to include:
   - `VW 200 0` — straight forward 200 mm/s; confirm `EVT` stream active, `mode=V`.
   - `VW 0 0` -> `X` — stop; confirm `mode=I`.
   - `TURN 9000` — turn to 90 degrees; confirm `EVT done TURN`, `mode=I`.
   - `D 500 500 300` — distance drive 300 mm; confirm `EVT done D`, `mode=I`.
   - `G <x> <y>` — go-to (robot-relative); confirm `EVT done G`, `mode=I`.
   - `RT 18000` — relative rotation 180 degrees; confirm `EVT done RT`.
   - `SAFE off` -> `VW 200 0` — safety one-shot test; confirm motion starts,
     safety re-armed on next begin, `EVT safety re-armed`.
   - TLM mode char: confirm `mode=V` during VW, `mode=D` during D,
     `mode=G` during G, `mode=I` at idle. (This confirms `planner.mode()`
     reports correctly through the TLM path.)

## Acceptance Criteria

- [ ] `uv run python -m pytest` run 1: passes except 2 baseline failures.
- [ ] `uv run python -m pytest` run 2: same result as run 1 (no flapping).
- [ ] `build.py --clean` exits zero.
- [ ] `MICROBIT.hex` build banner verified as fresh (not stale incremental).
- [ ] `tests/bench/061_bench_checklist.md` exists with all command sequences
      listed above, each with a checkbox for stakeholder sign-off.
- [ ] Sprint branch left open for stakeholder bench-test.

## Implementation Plan

### Approach

This ticket is mostly execution, not coding. Run the test commands, capture
output, create the checklist file.

### Files to create

- `tests/bench/061_bench_checklist.md`

### Testing plan

The ticket IS the test plan. Two sequential `uv run python -m pytest` runs.

### Documentation updates

`tests/bench/061_bench_checklist.md` is the primary deliverable. After the
stakeholder completes bench validation and signs off the checklist, the sprint
can be closed via the normal close-sprint process.
