---
id: '005'
title: 'Validation: sim tests, firmware clean build, end-to-end'
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-003
depends-on:
- 052-003
- 052-004
issue: stop-conditions-as-a-first-class-system-primitive.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Validation: sim tests, firmware clean build, end-to-end

## Description

Final validation pass for sprint 052. Run the full simulation test suite,
perform a firmware clean build, and write and run one end-to-end scenario test
that exercises `stop=` + `reason=` from the wire through to the Python host.

This ticket produces no new source changes — only tests and verification runs.
If any gate fails, the programmer must identify which ticket's work is defective
and file it as a bug fix before marking this ticket done.

## Implementation Plan

### Step 1: Full simulation test suite

Run: `uv run --with pytest python -m pytest tests/simulation -q`

Expected result: same 2 pre-existing failures (test_default_config_pin and
test_robot_config), zero new failures.

If any new failure is found, diagnose and fix in the appropriate prior ticket
(001-004) before returning here.

### Step 2: Firmware clean build

Run: `python build.py --clean`

Expected result: exit 0, no ARM cross-compiler errors.

The Python simulation build does not exercise ARM-specific code paths
(e.g. `__attribute__((packed))`, CMSIS register headers, linker script issues).
The clean build is the only gate that catches ARM-only compile errors.

If the clean build fails, diagnose which file introduced the error (typically
a C++ syntax issue or missing include in the firmware-only code path) and fix
before marking this ticket done.

### Step 3: End-to-end scenario test

Write one focused end-to-end scenario test in
`tests/simulation/system/test_052_stop_reason_e2e.py`:

**Scenario A — VW with distance stop, reason reported**:
1. Create a sim instance.
2. Send `VW 200 0 stop=d:300` via the sim command interface.
3. Tick the sim forward until either the command completes or 10 seconds
   elapse (whichever first).
4. Collect all output lines; find the `EVT done` line.
5. Assert the EVT line contains `reason=dist`.
6. Assert the robot approximately stopped after ~300 mm (or confirm the
   DISTANCE stop condition was actually evaluated by checking the sim state).

**Scenario B — T command, reason=time on implicit stop**:
1. Send `T 200 200 1000` (no explicit stop=).
2. Tick until completion.
3. Assert EVT line contains `reason=time`.

**Scenario C — back-compat: sensor= still works**:
1. Send `T 200 200 5000 sensor=line0:ge:512`.
2. Inject a high line0 sensor value in the sim.
3. Tick until completion.
4. Assert EVT line contains `reason=line0`.

**Scenario D — golden-TLM canary is unaffected**:
Confirm `test_golden_tlm.py` still passes (it is part of the full suite run,
but call this out explicitly: the EVT lines that T generates are NOT in the
golden capture, only TLM lines are — this test must pass without modification).

### Step 4: Verify use-case coverage

Review `usecases.md` acceptance criteria and confirm every criterion is met:
- SUC-001: all stop= kinds accepted on VW (from ticket 001 tests).
- SUC-002: all reason tokens emitted (from ticket 002 tests).
- SUC-003: Stop builder and wait_for_evt_done tuple (from ticket 003 tests).
- SUC-004: docs updated (from ticket 004 Markdown changes).

## Files to Create or Modify

- `tests/simulation/system/test_052_stop_reason_e2e.py` — new end-to-end
  scenario test (Scenarios A, B, C, D above).
- No source code changes.

## Acceptance Criteria

- [ ] `uv run --with pytest python -m pytest tests/simulation -q` passes with exactly
  2 failures (the pre-existing baseline), zero new failures.
- [ ] `python build.py --clean` exits 0 (ARM firmware clean build).
- [ ] Scenario A: `VW 200 0 stop=d:300` emits `reason=dist` in EVT line.
- [ ] Scenario B: `T 200 200 1000` emits `reason=time` in EVT line.
- [ ] Scenario C: `T 200 200 5000 sensor=line0:ge:512` emits `reason=line0` in EVT line.
- [ ] Scenario D: `test_golden_tlm.py` passes (TLM-only canary unaffected).
- [ ] All SUC-001 through SUC-004 acceptance criteria verified (by referencing
  passing tests from tickets 001-004).

## Testing

**Verification commands**:
1. `uv run --with pytest python -m pytest tests/simulation -q` (full sim suite)
2. `python build.py --clean` (ARM firmware)

**Pre-existing baseline**: 2 failures. No new failures acceptable.

This ticket is complete only when BOTH commands succeed and the end-to-end
scenario tests pass. The firmware clean build is non-negotiable — the Python
sim alone does not catch ARM-only errors (this bit us in sprint 051).
