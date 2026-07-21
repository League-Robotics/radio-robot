---
id: '002'
title: Fix kCycle/kPrimaryPeriod doc-comment and mislabeling mismatch
status: open
use-cases:
- SUC-115-006
depends-on: []
github-issue: ''
issue: kcycle-kprimaryperiod-mismatch.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Fix kCycle/kPrimaryPeriod doc-comment and mislabeling mismatch

## Description

`src/firm/app/robot_loop.cpp`'s own doc comment (lines ~17-21) claims
`kCycle` (20ms) matches `Devices::Telemetry::kPrimaryPeriod` (`telemetry.h`,
40ms) "by construction" — false at a 2:1 ratio. Both constants also
independently mislabel their own rate ("~25Hz" on both: 20ms is actually
~50Hz, 40ms is actually ~25Hz). Filed by 111-002
(`clasi/issues/kcycle-kprimaryperiod-mismatch.md`) while retargeting
`sim_api_harness.cpp`'s own duplicated timing constants; explicitly scoped
out of that ticket.

## Implementation Plan

- **Approach**: decide whether `kCycle`/`kPrimaryPeriod` are meant to be
  1:1 (restore `kPrimaryPeriod` to 20 to match `kCycle`'s current value,
  preserving the doc comment's "matching by construction" claim as true)
  or a deliberate 2:1 relationship (keep both values, rewrite the doc
  comment to describe the 2:1 relationship honestly instead of claiming
  parity). Either resolution is acceptable — the requirement is that code
  and comment agree afterward. Absent a specific reason to change runtime
  behavior, prefer describing the existing 2:1 relationship accurately
  over changing `kPrimaryPeriod`'s value (a primary-frame-rate change is a
  behavior change this ticket does not need to make just to fix a doc
  comment) — but this is the implementer's call to make and record, not
  pre-decided here.
- **Files to modify**: `src/firm/app/robot_loop.cpp` (the `kCycle`
  doc-comment block and `kCycle`'s own trailing "~25Hz" mislabel),
  `src/firm/app/telemetry.h` (`kPrimaryPeriod`'s own trailing "~25Hz"
  comment — already numerically correct at 40ms, just needs the label
  double-checked against whatever `kCycle` decision is made).
- **Do not touch**: `src/tests/sim/system/sim_api_harness.cpp` — 111-002
  already retargeted its own duplicated constants to the tree's current
  values; this ticket only touches the two source-of-truth constants and
  their comments, not test-side duplicates (there are none left to fix
  per the issue's own account).

## Acceptance Criteria

- [ ] A decision (1:1 vs. deliberate 2:1) is recorded in this ticket's
      completion notes.
- [ ] `robot_loop.cpp`'s doc comment accurately describes the actual
      shipped relationship between `kCycle` and `kPrimaryPeriod` — no
      false "matching by construction" claim.
- [ ] `kCycle`'s own trailing comment states its real rate (not "~25Hz"
      for a 20ms period).
- [ ] `telemetry.h`'s `kPrimaryPeriod` trailing comment states its real
      rate correctly.
- [ ] If the 1:1 resolution is chosen and `kPrimaryPeriod`'s VALUE
      changes, that behavior change is called out explicitly in this
      ticket's completion notes (not silently folded into a "doc fix").
- [ ] No other timing behavior changes.

## Testing

- **Existing tests to run**: full `uv run python -m pytest` sim suite
  (confirms no behavior regression if `kPrimaryPeriod`'s value changes);
  `just build-clean`.
- **New tests to write**: none required — this is a doc/constant
  correctness fix, not new behavior, unless the 1:1 resolution is chosen,
  in which case confirm existing primary-frame-rate-sensitive tests (if
  any) still pass at the new period.
- **Verification command**: `uv run pytest`
