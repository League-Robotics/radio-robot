---
id: '010'
title: 'Calibration CLI and MCP surface: rename unit-suffixed identifiers in the calibration
  wizard and agent-facing tools'
status: open
use-cases:
- SUC-007
depends-on:
- '003'
- '005'
- '007'
github-issue: ''
issue: remove-units-from-identifier-names-host-python.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Calibration CLI and MCP surface: rename unit-suffixed identifiers in the calibration wizard and agent-facing tools

## Description

`host/robot_radio/io/calibrate.py` (interactive calibration wizard),
`io/robot_mcp.py` (the agent-facing MCP tool surface), and `media/movie.py`
(companion media helper) sit at the same top-of-dependency-graph level as
`io/cli.py` (ticket 009), consuming the `robot/` object model (ticket 003),
`calibration/` (ticket 005), and `testgui/` (ticket 007, via shared
transport/connection concepts) — hence this ticket depends on those three,
not on ticket 009 itself (the two top-level surfaces are siblings, not
sequential).

Renames (per Step 5): `io/calibrate.py`, `io/robot_mcp.py` — unit-suffixed
parameters/locals; `media/movie.py` (`min_interval_ms` → `min_interval
# [ms]`).

**Security/API-surface note (from the Architecture Self-Review's Risks
section)**: `io/robot_mcp.py`'s `@tool`-decorated function **names**
(as distinct from their parameter names) are the MCP tool surface exposed
to agent sessions. This planning pass's own reading found none of those
tool names are unit-suffixed, so no tool name should need to change — but
this ticket's implementer must explicitly re-verify that before renaming
anything in this file, since renaming an exposed tool name (unlike a
parameter name) would be a breaking change for any agent session with a
cached tool schema. Only internal parameter/local names are in scope.

Total scope: 165 occurrences (126 in `io/calibrate.py`, 31 in
`io/robot_mcp.py`, 8 in `media/movie.py`).

## Hard Contract (applies to this and every sprint 076 ticket)

- **Pure rename — no behavioral change.** Every `rogo calibrate` wizard
  step and every MCP tool call behaves identically to pre-076.
- **Every renamed declaration carries a `# [unit]` comment.**
- **`io/robot_mcp.py`'s `@tool`-decorated function names are STABLE** —
  re-verify none are unit-suffixed before editing; if one is found
  unit-suffixed, do not rename it in this ticket without first raising it
  to the team-lead (a tool-name change is a breaking API change beyond
  this sprint's pure-rename scope, distinct from a parameter rename).
- **Wire keys and pydantic attributes are STABLE** per the sprint-wide
  Exclusion Table — any `SET`/`SIMSET` key or `RobotConfig` attribute this
  file reads or builds stays untouched.
- **Full suite green throughout**: `uv run python -m pytest -q` remains
  **2682 passed, 0 failed**.
- **Cross-cutting kwargs**: `read_ms` → `read_timeout` and every other
  sprint-decided rename must be used consistently.
- **Ignore environmental `data/robots` drift.**

## Acceptance Criteria

- [ ] `io/calibrate.py`: all unit-suffixed identifiers renamed with
      `# [unit]` comments, converging on names already decided by
      tickets 001–007 for any shared cross-cutting parameter.
- [ ] `io/robot_mcp.py`: all unit-suffixed **parameter/local** identifiers
      renamed with `# [unit]` comments; every `@tool`-decorated function's
      own **name** is confirmed unchanged (explicit check, documented in
      the ticket's completion notes).
- [ ] `media/movie.py`: `min_interval_ms` → `min_interval` with `# [ms]`.
- [ ] No `SET`/`SIMSET` wire-key string or `RobotConfig` pydantic attribute
      name is touched anywhere in this ticket's three files.
- [ ] Hard Contract above holds.

## Testing

- **Existing tests to run**: any `tests/simulation/unit/` test exercising
  `io/calibrate.py`'s wizard logic or `io/robot_mcp.py`'s tool functions
  (grep for `calibrate`/`robot_mcp` imports to enumerate); `media/movie.py`
  test coverage if any exists.
- **New tests to write**: none required — pure rename.
- **Verification command**: `uv run python -m pytest -q` (confirm 2682
  passed, 0 failed).

## Implementation Plan

**Approach**: Handle `io/robot_mcp.py` most carefully, since it is the
only file in this sprint with an externally-cached-schema risk distinct
from every other pure-Python rename.

1. Read `io/robot_mcp.py` in full; list every `@tool`-decorated function
   name and confirm none is unit-suffixed (expected result per this
   pass's own planning-time reading — re-verify, don't assume).
2. Rename `io/robot_mcp.py`'s internal parameter/local names only, leaving
   every tool function's own name untouched.
3. Rename `io/calibrate.py`'s unit-suffixed identifiers, converging on
   any cross-cutting name already decided upstream.
4. Rename `media/movie.py`'s `min_interval_ms` → `min_interval`.
5. Grep this ticket's three files for every renamed identifier's old name
   to confirm no call site was missed, and specifically confirm no
   `@tool`-decorated function name changed.
6. Run relevant unit tests, then the full suite.

**Files to create/modify**:
- `host/robot_radio/io/calibrate.py`
- `host/robot_radio/io/robot_mcp.py`
- `host/robot_radio/media/movie.py`

**Testing plan**: Run any calibration-wizard/MCP-specific unit tests
individually, then `uv run python -m pytest -q` and confirm the 2682
baseline holds.

**Documentation updates**: None in this ticket.
