---
status: pending
priority: medium
filed: 2026-08-11
filed_by: "programmer (sprint 136 ticket 002, full-suite baseline triage)"
tickets:
- 136-002
---

# `test_app_preamble.py` and `test_devices_otos.py` fail at RUNTIME (scenario assertions), not at compile time

## Description

Both `src/tests/sim/unit/test_app_preamble.py::
test_app_preamble_harness_compiles_and_passes` and
`src/tests/sim/unit/test_devices_otos.py::
test_devices_otos_harness_compiles_and_passes` fail on `src/tests/sim`'s
clean post-136-001 baseline. Both are long-documented pre-existing
failures -- sprint 135 ticket 008's own Completion Notes confirm byte-for-
byte identical failure output before and after that ticket's change (via
`git stash` comparison), and this sprint's own source issue
(`sprint-135-pre-existing-test-failures-need-triage.md`) names both as
already flagged pre-existing. **Nobody had filed a live tracked issue for
this pair before now** -- only sprint-ticket Completion Notes mention them,
each time as "out of scope, not re-diagnosed here."

This issue exists so the pair has a real target for a strict `xfail` mark
(sprint 136 ticket 002's own process fix -- issue A's item 2) and so a
future sprint doesn't re-discover the same "pre-existing, not our fault"
finding a third or fourth time.

## Confirmed: both are RUNTIME scenario failures, compile step succeeds

Verified directly (2026-08-11, `uv run python -m pytest
src/tests/sim/unit/test_app_preamble.py src/tests/sim/unit/test_devices_otos.py
-v --tb=long`) -- both harnesses compile cleanly (`compile_result.returncode
== 0`, no compile-step assertion fires); the failure is the SECOND
assertion, `run_result.returncode == 0`, after the compiled binary actually
ran and reported scenario failures itself. This distinction matters for
sprint 136's own Half B (moves ~10 source files across ~49 hardcoded
source-path lists): a COMPILE failure in a currently-red test could mask a
Half B path-list mistake landing invisibly on top of it. Neither of these
two is that risk -- both compile fine against the current tree.

### `test_app_preamble.py` -- `Core::Preamble` scenario failures (7 assertions)

```
--- Preamble: all-present happy path -- done() reached, every leaf present/connected
  FAIL: done() reached within 10 steps once every leaf detects on its first attempt -- expected true, got false
  FAIL: exactly 5 probe-carrying steps: Left, Right, Otos, Color, Line -- expected 5, got 10
  FAIL: linePresent() true -- expected true, got false
  FAIL: no script under-run: otos -- expected 0, got 1
  FAIL: no script under-run: color -- expected 0, got 1
  FAIL: no script under-run: line -- expected 0, got 1
--- Preamble: transient I2C NAK during NezhaMotor::begin() does not latch connected() false
  FAIL: done() reached -- the transient NAK cost no extra Preamble-level retry -- expected true, got false
```

Reads as `Core::Preamble` taking roughly double the expected number of
probe-carrying steps to reach `done()` even in the all-present happy path
(10 steps instead of 5), with the OTOS/color/line leaf scripts under-running
by exactly one step each -- consistent with an off-by-one or a doubled
retry somewhere in the boot-detection sequencing, not investigated further
here.

### `test_devices_otos.py` -- `Hal::Otos` scenario failures (5 assertions)

```
--- PRODUCT_ID detect gates all bus traffic
  FAIL: never begun: getOffset() returns zero -- expected 0, got 1
  FAIL: match: begin() issued exactly the expected probe+init+scalar+zero-pose transactions -- expected 8, got 9
--- secondary primitives: setOffset/getOffset, signal-cfg, imu-calib, resetTracking
  FAIL: getOffset() issues exactly one write + one 6-byte read -- expected 2, got 0
  FAIL: signalProcessConfig() returns the raw scripted byte unmodified -- expected 15, got 140
  FAIL: imuCalibrationSamplesRemaining() returns the raw scripted byte unmodified -- expected 37, got 15
```

Per sprint 135 ticket 008's own attribution: "concern `Devices::Otos::
begin()`'s write-transaction count and scripted calibration-register bytes" --
a transaction-count/byte-scripting mismatch in the harness's expectations
vs. `Hal::Otos`'s actual current I2C sequencing, unrelated to heading sign
or any sprint 135/136 work.

## What to do

Root-cause each independently (they are different modules, likely different
causes despite always being reported as a pair):

1. `Core::Preamble`: read the current boot-detection sequencing and compare
   against the scenario's expected 5-step happy path -- find where the extra
   5 steps and the OTOS/color/line script under-runs come from.
2. `Hal::Otos`: read `begin()`'s current transaction sequence against the
   harness's scripted expectations (8 vs 9 transactions; the secondary-
   primitives scenario's 0-vs-2 and byte-mismatch findings suggest the
   scripted read/write ordering itself may have drifted from what the
   harness scripts, not necessarily a real Otos defect).

## Verification

- Both harnesses' own scenario runners report zero `FAIL` lines.
- `test_app_preamble_harness_compiles_and_passes` and
  `test_devices_otos_harness_compiles_and_passes` pass.

## Related

- `clasi/sprints/done/135-go-to-navigator-in-the-motion-library/tickets/done/008-fix-sim-otos-heading-sign-divergence-option-a-simplant-packs-the-hardware-mounted-sign.md`
  -- confirms both pre-existing via `git stash` comparison, explicitly out
  of that ticket's scope.
- `clasi/sprints/136-.../issues/sprint-135-pre-existing-test-failures-need-triage.md`
  -- names both as already-flagged pre-existing.
