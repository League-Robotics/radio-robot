---
id: "005"
title: "Measure final coverage: run harness, verify ≥85% simulatable-code gate, report exclusion set"
status: open
use-cases: ["SUC-006"]
depends-on: ["001", "002", "003", "004"]
github-issue: ""
issue: ""
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

- [ ] `bash tests/_infra/coverage.sh --fail-under 85` exits 0 — meaning either:
  - (a) Overall `source/` line coverage is ≥85%, OR
  - (b) Simulatable-code coverage is ≥85% AND a clear note in the coverage.sh output
    identifies the excluded CODAL-only files and their uncovered line counts.
- [ ] The full test suite passes with the exact baseline count or higher (no tests deleted).
  `uv run --with pytest python -m pytest tests/simulation -q` exits 0.
- [ ] Golden-TLM byte-exact: `uv run --with pytest python -m pytest tests/simulation/unit/test_golden_tlm.py -q` passes.
- [ ] Field-pin gate: `uv run --with pytest python -m pytest tests/simulation/unit/test_default_config_pin.py -q` passes.
- [ ] Vendor grep gate: `uv run --with pytest python -m pytest tests/simulation/unit/test_vendor_confinement.py -q` passes.
- [ ] Final coverage numbers (overall % and simulatable %) are reported back to team-lead in the ticket closing comment.
- [ ] CODAL-only exclusion set is finalized in `coverage.sh` (any additions from T002 RatioPidController audit incorporated).

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
