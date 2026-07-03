---
id: '001'
title: 'Transport primitives: rename unit-suffixed parameters in serial and ctypes-sim
  transport'
status: open
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: remove-units-from-identifier-names-host-python.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Transport primitives: rename unit-suffixed parameters in serial and ctypes-sim transport

## Description

`host/robot_radio/io/serial_conn.py`'s `SerialConnection` and
`host/robot_radio/io/sim_conn.py`'s ctypes-wrapper carry unit-suffixed
parameter names. This is the **lowest layer** of `host/robot_radio/` — zero
intra-package dependencies (confirmed by import grep, `architecture-update.md`
Step 2) — so it is the sprint's root ticket: every other ticket's file set
imports one or both of these modules, directly or transitively.

This ticket is also the **origin point** for the sprint's single most
pervasive rename: `read_ms` → `read_timeout` (the exact worked example
already given in `docs/coding-standards.md`'s Python convention section),
defined here in `SerialConnection.send` and consumed by **216** keyword call
sites across 34 files sprint-wide (`architecture-update.md` Step 1 census,
Decision 2). Every later ticket must converge on this exact name for its own
`read_ms=` call sites — do not invent an alternative spelling.

`io/preview.py` has zero unit-suffix hits (Step 1 census) and needs no edit,
but is in this ticket's review scope to confirm it stays clean.

Total scope: 48 rename-eligible occurrences (Step 3).

## Hard Contract (applies to this and every sprint 076 ticket)

- **Pure rename — no behavioral change.** No algorithm, timing, buffering,
  or output-value change; only identifier spelling and comment placement.
- **Every renamed declaration carries a `# [unit]` comment** at its
  declaration, per `docs/coding-standards.md`'s Python convention (leading
  bracketed tag, first token of the trailing comment).
- **Wire keys/tokens and pydantic attributes are STABLE** — see
  `architecture-update.md`'s Wire-Compatibility Exclusion Table. This
  ticket's own surface has no wire-key literals to worry about, but do not
  touch the `extern "C"` ctypes ABI function names `io/sim_conn.py` calls —
  already unit-free (071), not part of this rename.
- **Full suite green throughout**: `uv run python -m pytest -q` must remain
  **2682 passed, 0 failed**.
- **`read_ms` → `read_timeout` is decided here, once** (Decision 2) — every
  later ticket inherits this name; do not let a different spelling land in
  this file.
- **Ignore environmental `data/robots` drift** — any pre-existing
  modification to `data/robots/*.json` in `git status` is unrelated to this
  sprint and must not be touched or attributed to this ticket.

## Acceptance Criteria

- [ ] `io/serial_conn.py`'s `SerialConnection.send` (and any sibling method
      carrying the same parameter) renames `read_ms` → `read_timeout`, with
      a `# [ms]` comment at the declaration.
- [ ] `io/sim_conn.py`'s ctypes-wrapper Python-side parameter/local names are
      renamed to match `io/serial_conn.py`'s renamed names (the two modules
      present the same transport contract to their callers).
- [ ] `io/preview.py` is reviewed and confirmed to have zero unit-suffixed
      identifiers before and after this ticket's edits (no edit expected).
- [ ] No `extern "C"` ctypes ABI function name is renamed — only
      `io/sim_conn.py`'s own Python-side parameter/local names change.
- [ ] Every `read_ms=` (or positional `read_ms`) call site **inside this
      ticket's three files** (`serial_conn.py`, `sim_conn.py`, `preview.py`)
      is updated to `read_timeout` in this same ticket.
- [ ] `grep -rn "read_ms\b" host/robot_radio/io/serial_conn.py
      host/robot_radio/io/sim_conn.py host/robot_radio/io/preview.py`
      returns zero results after this ticket (excluding this ticket's own
      historical commit messages).
- [ ] Hard Contract above holds.

## Testing

- **Existing tests to run**: any `tests/simulation/unit/` test exercising
  `SerialConnection` or the ctypes sim transport (grep for
  `SerialConnection`/`sim_conn` imports to enumerate — `test_serial_conn_reader.py`
  is confirmed relevant per `usecases.md` SUC-002's acceptance criteria).
- **New tests to write**: none required — pure rename, no new behavior.
- **Verification command**: `uv run python -m pytest -q` (confirm 2682
  passed, 0 failed).

## Implementation Plan

**Approach**: Rename in `io/serial_conn.py` first (the true origin of
`read_ms`), then mirror the same names into `io/sim_conn.py`'s ctypes
wrapper, then confirm `io/preview.py` needs no change.

1. In `host/robot_radio/io/serial_conn.py`, rename `read_ms` →
   `read_timeout` on `SerialConnection.send` and any sibling method/helper
   sharing the parameter; add `# [ms]` at each declaration site (following
   `docs/coding-standards.md`'s exact worked example). Update any
   `read_ms=` call site within this file itself.
2. In `host/robot_radio/io/sim_conn.py`, rename the ctypes-wrapper's
   corresponding Python-side parameter/local names to match step 1's
   choices; do **not** touch the `extern "C"` function names themselves.
   Update any `read_ms=` call site within this file.
3. Read `host/robot_radio/io/preview.py` in full; confirm it has zero
   unit-suffixed identifiers (per Step 1's census) and needs no edit.
4. Run `grep -rn "read_ms\b"` scoped to these three files to confirm zero
   residual hits.
5. Run the transport-related unit tests, then the full suite.

**Files to create/modify**:
- `host/robot_radio/io/serial_conn.py` — modified (rename + `# [unit]`
  comments).
- `host/robot_radio/io/sim_conn.py` — modified (rename to match).
- `host/robot_radio/io/preview.py` — reviewed only, no edit expected.

**Testing plan**: Run the transport-related tests under
`tests/simulation/unit/` individually first, then
`uv run python -m pytest -q` and confirm the 2682-passed / 0-failed
baseline holds.

**Documentation updates**: None in this ticket — `docs/coding-standards.md`'s
"not yet applied" status line is closed out by ticket 011, once every
ticket has landed.
