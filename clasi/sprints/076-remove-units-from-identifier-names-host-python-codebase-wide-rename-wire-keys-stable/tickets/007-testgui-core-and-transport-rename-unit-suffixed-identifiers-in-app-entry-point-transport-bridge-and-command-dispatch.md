---
id: '007'
title: 'TestGUI core and transport: rename unit-suffixed identifiers in app entry
  point, transport bridge, and command dispatch'
status: open
use-cases:
- SUC-006
depends-on:
- '002'
github-issue: ''
issue: remove-units-from-identifier-names-host-python.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# TestGUI core and transport: rename unit-suffixed identifiers in app entry point, transport bridge, and command dispatch

## Description

This is planned ticket **007a** in `architecture-update.md`'s Step 3 table
(filed here as sprint ticket 007). It renames unit-suffixed identifiers in
`testgui/__main__.py`, `testgui/transport.py`, `testgui/commands.py`, and
`testgui/drive.py` — the TestGUI's app entry point, connection/transport
bridge, and command dispatch. This subsystem depends on `io/serial_conn.py`
(ticket 001) and `robot/protocol.py` (ticket 002, already renamed) only —
not on the `robot/` object model (ticket 003), `sensors/` (004),
`calibration/` (005), or `nav/` (006) layers.

Renames (per Step 5): `testgui/transport.py` (`read_ms` → `read_timeout`,
`encoder_noise_mm` → `encoder_noise  # [mm]`); `testgui/__main__.py`,
`testgui/commands.py`, `testgui/drive.py` — remaining unit-suffixed
identifiers. Matching `tests/testgui/*.py` files are updated **in this same
ticket**.

**`tests/testgui/*.py` file assignment (Open Question 3, resolved here)**:
per Step 3's own grouping, this ticket owns the **transport / connection /
command / mode / relay / telemetry / smoke** test files (e.g. files
exercising `testgui/transport.py`'s connect/disconnect flow, mode
indicator, relay discovery, and command dispatch). Ticket 008 owns the
**sim-errors / traces / canvas / camera / tour / recorder** test files. If
a specific file's ownership is ambiguous at implementation time, grep the
file's own imports (`from robot_radio.testgui.transport import ...` vs.
`from robot_radio.testgui.sim_prefs import ...`, etc.) to resolve it —
`architecture-update.md` explicitly left the exact file-by-file split to
the ticketing/implementation pass.

**Test-tier note**: `tests/testgui/` (24 files, 579 tests) is **not**
collected by the default `uv run python -m pytest` invocation
(`pyproject.toml`'s `testpaths = ["tests/simulation"]` excludes it
entirely — not merely `norecursedirs`-skipped). This ticket's acceptance
criteria must include an explicit, separate testgui-tier test run.

Total scope: 96 occurrences in `host/robot_radio/testgui/` files touched by
this ticket, plus this ticket's share of `tests/testgui/`'s 169
occurrences.

## Hard Contract (applies to this and every sprint 076 ticket)

- **Pure rename — no behavioral change.** The TestGUI must look and behave
  identically; only Python identifiers change.
- **Every renamed declaration carries a `# [unit]` comment.**
- **No wire-key surface in this ticket's files** — `testgui/transport.py`
  is a thin bridge over `io/serial_conn.py`/`robot/protocol.py`, both
  already renamed; nothing here builds a raw wire string.
- **Full suite green throughout**:
  - `uv run python -m pytest -q` remains **2682 passed, 0 failed**
    (`tests/testgui/conftest.py` inserts `host/` onto `sys.path`, so this
    baseline must still be checked even though this ticket's primary files
    live outside `testpaths`).
  - `QT_QPA_PLATFORM=offscreen uv run python -m pytest tests/testgui/ -q`
    remains **579 passed, 2 xfailed**.
- **Cross-cutting kwargs**: `read_ms` → `read_timeout` convergence is
  required in every call site inside this ticket's files (`transport.py`
  is itself one of the 34 files in the 216-site census).
- **Ignore environmental `data/robots` drift.**

## Acceptance Criteria

- [ ] `testgui/transport.py`: `read_ms` → `read_timeout` with `# [ms]`;
      `encoder_noise_mm` → `encoder_noise` with `# [mm]`.
- [ ] `testgui/__main__.py`, `testgui/commands.py`, `testgui/drive.py`:
      remaining unit-suffixed identifiers renamed with `# [unit]` comments.
- [ ] Every widget's displayed value, slider range, and issued command
      computed from a renamed field/local produces unchanged numeric
      behavior.
- [ ] Matching `tests/testgui/*.py` files (transport/connection/command/
      mode/relay/telemetry/smoke tier, per the file-assignment guidance
      above) are updated in this same ticket — every `read_ms=` or other
      renamed-parameter call site in these test files converges on the
      ticket 001/002-decided names.
- [ ] `uv run python -m pytest -q` remains 2682 passed, 0 failed.
- [ ] `QT_QPA_PLATFORM=offscreen uv run python -m pytest tests/testgui/ -q`
      remains 579 passed, 2 xfailed.
- [ ] Hard Contract above holds.

## Testing

- **Existing tests to run**: the transport/connection/command/mode/relay/
  telemetry/smoke subset of `tests/testgui/*.py` (grep each file's imports
  to confirm it belongs here vs. ticket 008), run individually first, then
  the full `tests/testgui/` tier, then the default suite.
- **New tests to write**: none required — pure rename.
- **Verification commands**:
  - `QT_QPA_PLATFORM=offscreen uv run python -m pytest tests/testgui/ -q`
    (confirm 579 passed, 2 xfailed).
  - `uv run python -m pytest -q` (confirm 2682 passed, 0 failed).

## Implementation Plan

**Approach**: Rename `testgui/transport.py` first (it re-exposes
`read_ms`, the cross-cutting name), then `__main__.py`/`commands.py`/
`drive.py`, then their matching test files.

1. `testgui/transport.py` — rename `read_ms` → `read_timeout`,
   `encoder_noise_mm` → `encoder_noise`; add `# [unit]` comments.
2. `testgui/__main__.py`, `testgui/commands.py`, `testgui/drive.py` —
   rename remaining unit-suffixed identifiers.
3. Identify the transport/connection/command/mode/relay/telemetry/smoke
   subset of `tests/testgui/*.py` by grepping each file's imports against
   this ticket's four host files; update every renamed identifier and
   keyword-argument call site in those test files.
4. Grep this ticket's full file set for every renamed identifier's old
   name to confirm no call site was missed.
5. Run the identified `tests/testgui/*.py` subset individually, then the
   full `tests/testgui/` tier with `QT_QPA_PLATFORM=offscreen`, then the
   default suite.

**Files to create/modify**:
- `host/robot_radio/testgui/__main__.py`
- `host/robot_radio/testgui/transport.py`
- `host/robot_radio/testgui/commands.py`
- `host/robot_radio/testgui/drive.py`
- The transport/connection/command/mode/relay/telemetry/smoke subset of
  `tests/testgui/*.py` (exact file list determined by import grep at
  implementation time).

**Testing plan**: Run the identified `tests/testgui/*.py` subset
individually, then `QT_QPA_PLATFORM=offscreen uv run python -m pytest
tests/testgui/ -q` (confirm 579 passed, 2 xfailed), then
`uv run python -m pytest -q` (confirm 2682 passed, 0 failed). Manually
launch the TestGUI to visually confirm connect/disconnect, mode indicator,
and command dispatch are unaffected.

**Documentation updates**: None in this ticket.
