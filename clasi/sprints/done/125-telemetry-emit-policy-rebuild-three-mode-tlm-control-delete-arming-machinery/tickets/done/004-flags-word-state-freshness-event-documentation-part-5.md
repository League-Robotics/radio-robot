---
id: '004'
title: Flags-word State/Freshness/Event documentation (Part 5)
status: done
use-cases: []
depends-on:
- '002'
github-issue: ''
issue: telemetry-emit-policy-rebuild-spec.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Flags-word State/Freshness/Event documentation (Part 5)

## Description

Documentation-only ticket (no wire change) that closes off the category
error that caused this sprint's root defect: a Freshness bit
(`kFlagLinePresent`/`kFlagColorPresent`) was treated as stability
evidence by the deleted report-on-change arm. Rewrite the flags layout
comment in `telemetry.h` so every bit is classified into exactly one of
three declared classes, under labeled section headers, per the issue's
Part 5:

- **State bits** (level, meaningful across frames): 1, 2 (`kFlagActive`),
  3, 4, 6, 7, 8, 9, 12, 16, 17, 18.
- **Freshness bits** (valid-THIS-frame qualifiers for a payload field,
  toggle by design, carry no cross-frame information): 0
  (`kFlagOtosPresent`), 13, 14. These may NEVER participate in any
  change-detection or stability logic.
- **Event bits** (transition-cycle-only): 10 (`kFlagEventDeadmanExpired`),
  15 (`kFlagFaultMoveTimeout` — rides the completing frame). Bit 11 is
  deleted (ticket 002); bit 5 stays RESERVED.

No wire change: bit positions and meanings on the wire are untouched
(bit 11 simply never sets any more). This ticket depends on ticket 002
because it documents the POST-deletion bit layout (bit 11 removed) —
writing this before 002 lands would describe a layout that does not yet
exist in code.

## Acceptance Criteria

- [x] `telemetry.h`'s flags layout comment has three labeled section
      headers (State / Freshness / Event) and every live bit (0-4, 6-10,
      12-18) is listed under exactly one of them; bit 5 and bit 11 are
      both marked RESERVED.
- [x] The Freshness section's header text explicitly states the "may
      NEVER participate in change-detection or stability logic" rule, so
      a future reader sees the constraint, not just the classification.
- [x] No bit position or wire meaning changes — this ticket touches
      comments only; confirm via `git diff` that no `constexpr uint32_t
      kFlag*` value changed.
- [x] The per-bit prose already present (e.g. bit 6's long
      `kFlagFaultI2CSafetyNet` explanation) is preserved under its new
      section header, not deleted — this ticket reorganizes, it does not
      remove existing hard-won documentation.

## Testing

- **Existing tests to run**: none functionally required (doc-only
  change), but run the full sim build to confirm no accidental code
  edit was introduced while touching the comment block.
- **New tests to write**: none — this is documentation. If desired, a
  cheap grep-based test could assert the three section headers exist in
  `telemetry.h`, matching the spirit of ticket 006's criterion-12
  grep-enforceable check, but this is optional, not required.
- **Verification command**: `uv run pytest` (sanity only — this ticket
  should produce zero test deltas).

## Completion Notes

- Ticket 002's own telemetry.h edit had NOT yet added the three labeled
  section headers this ticket requires (it built `TlmMode`/`setMode()`/
  the emit predicate, and updated bit 11's per-bit prose to RESERVED, but
  left the flags-layout comment as one flat per-bit list) — this ticket
  did the actual State/Freshness/Event reorganization.
- Classification used exactly the issue's Part 5 lists: State = 1, 2, 3,
  4, 6, 7, 8, 9, 12, 16, 17, 18; Freshness = 0, 13, 14; Event = 10, 15;
  Reserved = 5, 11. Called out explicitly in the new header note: bit 12
  (`kFlagEventConfigApplied`) is named "Event" but classified STATE —
  the ticket's own point that class is a property of meaning, not name.
- `git diff -- src/firm/app/telemetry.h` shows no `constexpr uint32_t
  kFlag*` line changed — confirmed no wire-value edits, comment-only
  diff.
- Verified with a syntax-only compile of `telemetry.cpp` (`-fsyntax-only
  -DHOST_BUILD`) and the full `just build-clean` + `uv run python -m
  pytest src/tests/sim/ -q` run done for ticket 003 (same commit) — 418
  passed, 1 skipped, 1 xfailed, 0 failed, zero test deltas as expected
  for a comment-only change.
