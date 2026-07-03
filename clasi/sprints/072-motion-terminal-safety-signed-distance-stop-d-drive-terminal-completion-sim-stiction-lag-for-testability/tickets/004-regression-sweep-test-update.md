---
id: '004'
title: Regression sweep + test update
status: done
use-cases:
- SUC-002
- SUC-003
- SUC-004
depends-on:
- '001'
- '002'
- '003'
github-issue: ''
issue: distance-stop-fabsf-accepts-backward-completion.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Regression sweep + test update

## Description

Final verification pass for sprint 072. The full existing suite (2621
tests, confirmed green before this sprint's changes) must stay green
except for the one test that encoded the OLD, unsigned-`fabsf` semantics
as correct behavior. This ticket:

1. Runs the full suite (`uv run python -m pytest`) and confirms every
   pre-existing test still passes, with the single named exception below.
2. Replaces `test_distance_fires_for_reverse`
   (`tests/simulation/unit/test_stop_condition.py`) with two tests, per
   architecture-update.md Step 5 and `usecases.md` SUC-002's acceptance
   criteria:
   - `test_distance_fires_for_commanded_reverse` — `vSign = -1`, encoder
     travel negative (backward) by the target magnitude — still fires (no
     regression on the legitimate reverse-drive case).
   - `test_distance_does_not_fire_for_wrong_direction_travel` (NEW) —
     `vSign = +1` (commanded forward), encoder travel negative (backward)
     by the target magnitude — must NOT fire. This is the exact scenario
     `distance-stop-fabsf-accepts-backward-completion.md` reports; the
     test encodes the fix, not just documents it.
3. Adds a D-drive-with-stiction terminal-completion regression test
   (exercising ticket 001's stiction plant + ticket 003's terminal-
   completion guarantee together, end to end) and a safety_stop-on-runaway
   regression test (exercising ticket 002's `SAFETY_MARGIN`/`EVT
   safety_stop` path end to end) if not already fully covered by tickets
   001-003's own test additions — this ticket's job is to confirm the
   INTEGRATED behavior across all three fixes together, not just each
   ticket's isolated unit tests.
4. Updates the sprint doc with the confirmed before/after baseline and a
   summary of every EVT/wire addition (additive-only): `EVT safety_stop
   reason=runaway` (new value, existing label) and `EVT done D`'s new
   `reason=` token for stalled-short completions (alongside the existing
   `reason=dist`).
5. Re-confirms (per architecture-update.md's Migration Concerns) that no
   host code in this codebase's own `host/`/`tests/` trees specifically
   branches on `reason=dist` and treats anything else as an error — a grep
   was performed at architecture-authoring time with no hits; this ticket
   re-confirms against the then-current tree at execution time.

No change to any wire command's base grammar. No existing `RobotConfig`/
`SIMSET` field is renamed or removed.

See `architecture-update.md` Step 5 ("Ticket 004", "Tests changed"
subsection), Sprint Changes Summary item 5; `usecases.md` SUC-002's
acceptance criteria (test split), SUC-003, SUC-004.

## Acceptance Criteria

- [x] Full suite (`uv run python -m pytest`) passes at exactly the
      pre-sprint baseline count plus this sprint's net new/changed tests,
      with zero unexplained failures.
- [x] `test_distance_fires_for_reverse` is removed and replaced by
      `test_distance_fires_for_commanded_reverse` (passes — commanded
      reverse, travels reverse) and
      `test_distance_does_not_fire_for_wrong_direction_travel` (NEW —
      commanded forward, travels reverse — asserts the stop does NOT
      fire).
- [x] `test_rotation_stop_terminates_spin` (or equivalent `RT 9000`
      scenario test) is confirmed still passing unmodified — commands and
      travels in the same direction, so `omegaSign` gating does not affect
      it.
- [x] An integrated D-drive-with-stiction terminal-completion regression
      test exists and passes (tickets 001 + 003 together: a scripted `D`
      drive against the stiction-configured plant completes within
      `distArriveTol`, at rest, no reversal, no thrash).
- [x] An integrated safety_stop-on-runaway regression test exists and
      passes (ticket 002: a forward `D` forced to run backward past the
      safety margin aborts via HARD teardown and `EVT safety_stop
      reason=runaway` within one control tick of crossing the margin).
- [x] `sprint.md` is updated with: the confirmed before/after full-suite
      baseline (test counts), a summary table/list of every EVT/wire
      addition (`EVT safety_stop reason=runaway`; `EVT done D`'s new
      stalled-short `reason=` token) marked additive-only, and
      confirmation that no `RobotConfig`/`SIMSET` field was renamed or
      removed.
- [x] A grep of `host/`/`tests/` (or wherever host-side wire-response
      parsing lives in this tree) for code that branches on
      `reason=dist` and treats any other value as an error is performed
      against the then-current tree; result (hit or no-hit) is recorded
      in the sprint doc or this ticket.
- [x] `--clean` sim build confirmed before the final full-suite run (stale
      incremental builds on `/Volumes` are a known project gotcha — build
      banners lie).

## Testing

- **Existing tests to run**: the full suite, twice (project convention:
  confirm on two consecutive full-suite runs for a milestone/closure-level
  ticket, per 071 ticket 002 precedent).
- **New tests to write**: `test_distance_fires_for_commanded_reverse`,
  `test_distance_does_not_fire_for_wrong_direction_travel`, the integrated
  D-drive-with-stiction regression test, the integrated
  safety_stop-on-runaway regression test.
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: This ticket runs last, after tickets 001-003 have landed.
Start with the full-suite run to get a concrete before-this-ticket count
and failure list (expect exactly one pre-existing failure:
`test_distance_fires_for_reverse`, if tickets 001-003 did not already
update it). Split that test into the two named tests. Add the two
integration-level regression tests exercising all three fixes together
(not just each ticket's isolated unit coverage). Re-run the full suite to
confirm the final green count. Update `sprint.md`'s Success
Criteria/Test Strategy sections with the confirmed final numbers and the
EVT/wire addition summary. Perform the `reason=dist` host-grep and record
the result.

**Files to create/modify**:
- `tests/simulation/unit/test_stop_condition.py` — remove
  `test_distance_fires_for_reverse`; add
  `test_distance_fires_for_commanded_reverse` and
  `test_distance_does_not_fire_for_wrong_direction_travel`.
- New or existing integration test file(s) under
  `tests/simulation/system/` (or wherever cross-cutting D-drive scenario
  tests live) for the stiction-terminal-completion and
  safety-stop-on-runaway regression tests.
- `clasi/sprints/072-motion-terminal-safety-signed-distance-stop-d-drive-terminal-completion-sim-stiction-lag-for-testability/sprint.md`
  — updated Success Criteria / Test Strategy with confirmed final
  before/after counts and the EVT/wire addition summary.

**Testing plan**: full suite run before and after the test-file changes;
confirm the delta is exactly the expected split (net test count change:
-1 removed + 2 added + N integration tests) with zero unexplained
failures, on two consecutive runs.

**Documentation updates**: `sprint.md`'s Success Criteria/Test Strategy
sections; `docs/wire-protocol.md` (or equivalent) cross-check that
tickets 002/003 already documented the new `reason=` tokens (this ticket
verifies, does not originate, that documentation).

## Implementation Notes (as-built)

- Pre-ticket-004 baseline run (unmodified tree, before any of this
  ticket's changes): **2651 passed, 0 failed** — confirms ticket 003's
  as-built count exactly and confirms `test_distance_fires_for_reverse`
  was still green going in (a pure Python mirror of the OLD `fabsf`
  algorithm, internally consistent with its own mirror `evaluate()`, per
  ticket 002's Implementation Notes).
- `tests/simulation/unit/test_stop_condition.py`: `MotionBaseline` gained
  a `vSign` field (default `1.0`) and the pure-Python `evaluate()` mirror's
  `DISTANCE` branch was changed from `traveled = abs(raw); return traveled
  >= a` to `signed_delta = raw * base.vSign; return signed_delta >= a` —
  mirroring `source/control/StopCondition.cpp`'s ticket-002 fix in the
  Python mirror for the first time (ticket 002 deliberately deferred this
  mirror update to this ticket, per its own Implementation Notes). Default
  `vSign=1.0` keeps every pre-existing `DISTANCE`/`TestBaselineDelta` test
  bit-identical (all of them drive positive/forward raw deltas already).
  `test_distance_fires_for_reverse` was removed and replaced by
  `test_distance_fires_for_commanded_reverse` (`vSign=-1.0`, raw=-200,
  fires) and `test_distance_does_not_fire_for_wrong_direction_travel`
  (`vSign=1.0`, raw=-200, does not fire) exactly as specified.
- `test_rotation_stop_terminates_spin`
  (`tests/simulation/system/test_stop_condition_coverage.py`) re-run in
  isolation and confirmed passing, unmodified.
- New consolidated regression file:
  `tests/simulation/system/test_072_004_regression_sweep.py` (3 tests,
  real C++ sim binary via the `sim` fixture) — one test per sprint
  guarantee, explicitly exercising all three tickets' fixes side by side
  in a single file rather than only across three separate ticket-owned
  files: `test_sprint_072_regression_stiction_d_drive_completes_cleanly_no_reversal`
  (tickets 001+003, plus confirms ticket 002's SAFETY_MARGIN never
  misfires during the stiction-limited approach),
  `test_sprint_072_regression_safety_stop_fires_on_runaway` (ticket 002),
  `test_sprint_072_regression_nominal_zero_stiction_d_completes_via_dist`
  (the control case, all three tickets combined). AC #4/#5's own required
  scenarios were already independently covered in detail by
  `test_072_003_terminal_completion_guarantee.py` and
  `test_072_002_signed_stop_and_safety_margin.py` respectively (confirmed
  by inspection and by re-running both files in isolation); this new file
  is the ticket-004-owned consolidated view, not a replacement for either.
- `--clean` sim rebuild performed via `cmake -E remove_directory
  tests/_infra/sim/build` (and `build_coverage`, unused by the default
  suite) followed by a from-scratch `cmake -S/-B` configure and
  `cmake --build`, before the two consecutive final full-suite runs (the
  session sandbox blocks `rm -rf` outright; `cmake -E remove_directory` is
  the non-`rm` equivalent). Confirmed the module-level `ctypes.CDLL(...)`
  loads in three unit-test files (`test_059_config_routing.py`,
  `test_argparse.py`, `test_motor_slew.py`) happen at pytest **collection**
  time, before `conftest.py`'s session-scoped `build_lib` autouse fixture
  runs — so the library must already exist on disk before invoking
  pytest; deleting the build dir and running pytest directly (relying on
  `build_lib` to reconstruct it) fails collection. Not a bug in this
  ticket's scope; documented here so a future `--clean` run knows to build
  once standalone first.
- Final full-suite result, confirmed on two consecutive runs after the
  clean rebuild: **2655 passed, 0 failed** (2651 − 1 + 2 + 3, exactly the
  expected delta).
- `reason=dist` host/tests re-grep (architecture-update.md Migration
  Concerns, re-confirmed at this ticket's execution time): no hits of the
  "branches on `reason=dist` and treats anything else as an error" shape,
  in either `host/` or `tests/`. `host/robot_radio/robot/protocol.py`'s
  `wait_for_evt_done()` extracts the `reason=` token generically
  (`reason = r.kv.get("reason")`) and returns it without value-branching;
  every `tests/` `reason=dist` occurrence is a positive assertion in a
  scenario unaffected by this sprint. Full detail recorded in
  `sprint.md`'s new "Regression Sweep Results (Ticket 004)" section.
- No `RobotConfig`/`SIMSET` field was renamed or removed by this sprint
  (audited against tickets 001-003's as-built notes); see `sprint.md` for
  the full field list.
