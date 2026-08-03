---
id: '016'
title: Host cleanup (extra='forbid', drift assertions)
status: open
use-cases:
- SUC-001
depends-on:
- '013'
github-issue: ''
issue: the-configuration-object.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Host cleanup (extra='forbid', drift assertions)

## Description

Set `extra='forbid'` on the generated pydantic model (or its thin
hand-written wrapper layer) so an unrecognized JSON key raises loudly
instead of silently ignoring it (today's default `extra='ignore'`,
confirmed this round — no `model_config`/`ConfigDict` override exists).
Add a test asserting the fields that were previously silently dropped
(`output_deadband`, `reversal_dwell_ms`, and the other 16 identified in
the issue's own audit) are now present on the generated model — this
should already be true structurally once the schema is generated (ticket
002), so this ticket's job is proving it, not fixing it by hand.

**Note**: this ticket runs BEFORE ticket 017's JSON reshape, so the robot
JSONs are still in their OLD 13-section shape at this point —
`extra='forbid'` will likely cause validation to fail against the
current files until the reshape lands (expected; do not treat this as a
bug to work around mid-sprint).

## Acceptance Criteria

- [ ] The generated pydantic model (or its thin wrapper) sets
      `extra='forbid'` (via `model_config = ConfigDict(extra='forbid')`
      or equivalent).
- [ ] A new test asserts the model has fields for every previously-dropped
      `control.*` key from this round's audit (`output_deadband`,
      `reversal_dwell_ms`, and the other 16 — pull the exact list from
      this ticket's own re-verification, not assumed from memory).
- [ ] The test explicitly documents that current robot JSONs are
      expected to FAIL `extra='forbid'` validation until ticket 017
      lands (a skip/xfail marker with a clear reason, not a
      silently-passing test that proves nothing).
- [ ] Compiles/imports cleanly.

## Testing

- **Existing tests to run**: any existing test asserting
  `extra='ignore'`-shaped behavior is updated to reflect the new
  `forbid` behavior.
- **New tests to write**: as in Acceptance Criteria.
- **Verification command**: `uv run python -m pytest <robot_config test
  path> -q`.

## Implementation Plan

**Approach**: A small, targeted pydantic config change plus one new test
file/test.

**Files to modify**: `src/host/robot_radio/config/robot_config.py` (or
wherever the thin hand-written wrapper from ticket 002 lives).

**Testing plan**: as above.

**Documentation updates**: none beyond the new test's own docstring
explaining the expected-to-fail-until-017 status.
