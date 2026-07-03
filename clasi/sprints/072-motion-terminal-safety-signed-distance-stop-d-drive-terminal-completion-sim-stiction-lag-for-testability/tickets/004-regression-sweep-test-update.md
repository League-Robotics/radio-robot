---
id: '004'
title: Regression sweep + test update
status: open
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

- [ ] Full suite (`uv run python -m pytest`) passes at exactly the
      pre-sprint baseline count plus this sprint's net new/changed tests,
      with zero unexplained failures.
- [ ] `test_distance_fires_for_reverse` is removed and replaced by
      `test_distance_fires_for_commanded_reverse` (passes — commanded
      reverse, travels reverse) and
      `test_distance_does_not_fire_for_wrong_direction_travel` (NEW —
      commanded forward, travels reverse — asserts the stop does NOT
      fire).
- [ ] `test_rotation_stop_terminates_spin` (or equivalent `RT 9000`
      scenario test) is confirmed still passing unmodified — commands and
      travels in the same direction, so `omegaSign` gating does not affect
      it.
- [ ] An integrated D-drive-with-stiction terminal-completion regression
      test exists and passes (tickets 001 + 003 together: a scripted `D`
      drive against the stiction-configured plant completes within
      `distArriveTol`, at rest, no reversal, no thrash).
- [ ] An integrated safety_stop-on-runaway regression test exists and
      passes (ticket 002: a forward `D` forced to run backward past the
      safety margin aborts via HARD teardown and `EVT safety_stop
      reason=runaway` within one control tick of crossing the margin).
- [ ] `sprint.md` is updated with: the confirmed before/after full-suite
      baseline (test counts), a summary table/list of every EVT/wire
      addition (`EVT safety_stop reason=runaway`; `EVT done D`'s new
      stalled-short `reason=` token) marked additive-only, and
      confirmation that no `RobotConfig`/`SIMSET` field was renamed or
      removed.
- [ ] A grep of `host/`/`tests/` (or wherever host-side wire-response
      parsing lives in this tree) for code that branches on
      `reason=dist` and treats any other value as an error is performed
      against the then-current tree; result (hit or no-hit) is recorded
      in the sprint doc or this ticket.
- [ ] `--clean` sim build confirmed before the final full-suite run (stale
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
