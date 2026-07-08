---
id: '001'
title: Rename Rt::MainLoop::emergencyNeutralize() to estop()
status: done
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: rename-emergencyneutralize-to-estop.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Rename Rt::MainLoop::emergencyNeutralize() to estop()

## Description

`Rt::MainLoop::emergencyNeutralize()` (declared `source/runtime/main_loop.h`,
defined `source/runtime/main_loop.cpp`) carries a stakeholder `// FIXME
rename to "estop"` comment on its declaration, deferred out of sprint 088
to keep that sprint's tree clean. Pure rename, no behavior change: the
method's body, call sites, and bypass semantics (direct `hardware_.apply()`/
`drivetrain_.apply()`, no `bb.driveIn`/`bb.motorIn`/`bb.hardwareBroadcastIn`
queue) are untouched.

This ticket goes first in the sprint because ticket 003 (the watchdog
motors-running gate) edits the SAME call site inside `serviceWatchdogs()`
again — better to land under the final name once than touch the old name
a second time.

- [x] `grep -rn emergencyNeutralize` across `source/`, `tests/`, and `docs/`
      returns nothing.
- [x] `Rt::MainLoop::estop()` exists with the exact same signature, body,
      and doc comment content as the old `emergencyNeutralize()` (only the
      name changes; update the doc comment's own self-reference too).
- [x] The one call site inside `serviceWatchdogs()`'s watchdog-fire branch
      (`main_loop.cpp`) calls `estop()`.
- [x] `tests/sim/unit/test_watchdog_policy.py` and
      `tests/sim/unit/test_protocol_roundtrips.py` (both reference the old
      name in comments/docstrings only, not in asserted strings) are
      updated to say `estop()` instead of `emergencyNeutralize()`.
- [x] `uv run python -m pytest tests/sim` stays green at the existing
      baseline (309 passed / 2 xfailed) — this ticket changes no test
      assertions, only comments/identifiers.

## Completion Notes

Pure rename, no behavior change. Sites changed (4 files, 8 total edits):

- `source/runtime/main_loop.h`: file-header cross-reference comment (line
  60, "See emergencyNeutralize()'s doc comment" → "See estop()'s doc
  comment"), the method's own doc comment self-reference (line 140,
  "emergencyNeutralize --" → "estop --"), and the declaration itself (line
  146: `void emergencyNeutralize(); // FIXME rename to "estop"` → `void
  estop();` — FIXME marker removed as instructed).
- `source/runtime/main_loop.cpp`: the definition (`MainLoop::
  emergencyNeutralize()` → `MainLoop::estop()`) and the one call site
  inside `serviceWatchdogs()`'s watchdog-fire branch (`emergencyNeutralize();`
  → `estop();`).
- `tests/sim/unit/test_watchdog_policy.py`: docstring reference
  (`` ``Rt::MainLoop::emergencyNeutralize()`` `` → `` ``Rt::MainLoop::estop()`` ``)
  in `test_watchdog_neutralizes_within_the_same_pass_it_fires_in`.
- `tests/sim/unit/test_protocol_roundtrips.py`: docstring reference
  ("the watchdog's own emergencyNeutralize() bypass" → "the watchdog's own
  estop() bypass").

`grep -rn emergencyNeutralize source/ tests/` returns zero hits (confirmed
via exit code 1 / empty output after the edits).

`uv run python -m pytest tests/sim` → `309 passed, 2 xfailed in 98.43s` —
byte-identical to the ticket's stated baseline; no test assertions changed.

## Implementation Plan

### Approach

Straight rename, in this order:

1. `source/runtime/main_loop.h`: rename the `emergencyNeutralize()` method
   declaration to `estop()`; update its doc comment (currently references
   itself by the old name and still carries the `// FIXME rename to
   "estop"` marker — remove the FIXME, keep the substantive bypass
   explanation) and any other doc comment in the same file that names the
   method (the file header's own "See emergencyNeutralize()'s doc comment
   for the bypass mechanism itself" cross-reference).
2. `source/runtime/main_loop.cpp`: rename the definition
   (`MainLoop::emergencyNeutralize()` → `MainLoop::estop()`); update its
   doc comment's self-reference; update the one call site in
   `serviceWatchdogs()`.
3. Grep the whole tree for `emergencyNeutralize` (source, tests, docs,
   clasi/) and fix every remaining reference — expected hits beyond the
   two files above are comment/docstring mentions in
   `tests/sim/unit/test_watchdog_policy.py` and
   `tests/sim/unit/test_protocol_roundtrips.py` (confirmed by a repo-wide
   grep during sprint planning — no other files reference the name).

### Files to Modify

- `source/runtime/main_loop.h`
- `source/runtime/main_loop.cpp`
- `tests/sim/unit/test_watchdog_policy.py` (comment/docstring only)
- `tests/sim/unit/test_protocol_roundtrips.py` (comment/docstring only)

### Testing Plan

- Run `uv run python -m pytest tests/sim` before and after; the set of
  passing/xfailed tests must be byte-identical (this ticket changes no
  assertions).
- `grep -rn emergencyNeutralize .` (repo root) must return zero hits after
  the change.

### Documentation Updates

- None beyond the doc-comment edits already covered above (no
  `docs/protocol-v2.md` change — this is an internal C++ identifier, not a
  wire-visible name).
