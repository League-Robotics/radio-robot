---
id: '006'
title: Golden-TLM re-baseline, validation, firmware clean build
status: in-progress
use-cases:
- SUC-005
depends-on:
- 053-003
- 053-004
- 053-005
issue: stop-conditions-as-a-first-class-system-primitive.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 053-006: Golden-TLM re-baseline, validation, firmware clean build

## Description

This is the final validation ticket for sprint 053. It re-baselines the
golden-TLM canary (which changes intentionally after Phase 2), runs the full
simulation test suite, and verifies the firmware ARM clean build. It also closes
the stop-conditions issue.

**The golden-TLM canary WILL change**. Tickets 003–005 move begin* calls from
the queue-drain hop to the verb handler hop (one tick earlier). TLM packets
that include drive mode or command state may shift by one tick index. This is
expected and must be reviewed — not blindly accepted.

**Do not run `uv run pytest`**. The canonical command is:
`uv run --with pytest python -m pytest tests/simulation -q`

**Do not run incremental firmware build**. Always use:
`python build.py --clean`

## Acceptance Criteria

- [ ] **Canary diff review**: Run the golden-TLM canary test before re-baselining.
  Record the diff in this ticket's commit message: what lines changed, whether
  changes are tick-shifted timestamps vs. behavioral differences (motion values,
  final positions, stop reasons).
- [ ] **Canary re-baseline**: Update the golden TLM capture file with the new
  baseline. The canary test passes after the update.
- [ ] **Canary diff documented**: A brief summary of the diff is in the commit
  message: "TLM canary re-baselined: N lines shifted by 1 tick; no change to
  motion values or stop reasons." If any behavioral difference is found (different
  final position, different stop reason, unexpected mode value), STOP and report
  as an exception — do not re-baseline over a behavioral regression.
- [ ] **Full sim test suite**: `uv run --with pytest python -m pytest
  tests/simulation -q` passes with exactly 2 known failures:
  - `tests/simulation/unit/test_default_config_pin.py::test_default_robot_config_unchanged`
  - `tests/simulation/unit/test_robot_config.py::TestSchemaValidation::test_tovez_validates_against_schema`
  No other failures.
- [ ] **New stop= on S tests pass** (from ticket 003): S with stop=d and stop=t
  fire correctly.
- [ ] **Wire-label preservation tests pass** (from tickets 003/004/005):
  EVT done S/T/D/R labels present in completion events.
- [ ] **Keepalive guard tests pass** (from ticket 002): RETARGETABLE updates;
  FIXED replies busy=.
- [ ] **Firmware clean build**: `python build.py --clean` exits 0. No compile
  errors or warnings from modified files. Verify by decoding or inspecting
  `MICROBIT.hex` timestamp to confirm the build banner is not stale.
- [ ] **Issue back-ref**: This ticket carries `issue:
  stop-conditions-as-a-first-class-system-primitive.md`.
- [ ] **Issue resolved**: The `stop-conditions-as-a-first-class-system-primitive.md`
  issue file is updated with `status: done` (or moved to `done/`) upon sprint
  close. This ticket carries `completes_issue: true`.

## Implementation Plan

### Approach

Validation-only ticket. No new code. Run tests, review the canary diff,
re-baseline, commit.

### Files to Modify

- Golden-TLM canary baseline file (locate via grep for the canary test fixture).
  Update with the new capture.
- No source files.

### Step-by-Step Plan

1. **Locate the canary**: Find the golden-TLM baseline file by reading
   `tests/simulation/` for tests whose name contains "canary" or "golden_tlm".
   The file is likely in `tests/simulation/unit/` or a fixtures subdirectory.

2. **Run canary before reset**: Run only the canary test first. Note which
   assertions fail and what the diff shows:
   ```
   uv run --with pytest python -m pytest tests/simulation -k "canary or golden" -v
   ```

3. **Review the diff**: For each failing assertion, determine:
   - Is it a tick-index shift? (Expected for this sprint.)
   - Is it a value change in motion outputs, stop reasons, or final state?
     (NOT expected — stop if found.)
   - Document findings briefly.

4. **Re-baseline**: Update the golden baseline file with the new capture
   (follow the canary test's update mechanism — there may be a `--update-baseline`
   flag or a manual file replacement).

5. **Full suite**: Run `uv run --with pytest python -m pytest tests/simulation -q`.
   Verify exactly 2 failures.

6. **Firmware build**:
   ```
   python build.py --clean
   ```
   Verify exit code 0. Check for any ARM-specific compile errors not caught by
   the host sim build (this has bitten the project before — sprint 051).

7. **Commit**: Include the canary diff summary in the commit message.

### Testing Plan

All tests run as part of this ticket. No new tests written here — all new test
files were added in tickets 002–005.

### Documentation Updates

None. The architecture update and use cases were written in the planning phase.
