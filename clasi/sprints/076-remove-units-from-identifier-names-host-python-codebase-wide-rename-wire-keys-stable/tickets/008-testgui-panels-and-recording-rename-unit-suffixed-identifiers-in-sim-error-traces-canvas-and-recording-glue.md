---
id: '008'
title: 'TestGUI panels and recording: rename unit-suffixed identifiers in sim-error,
  traces, canvas, and recording glue'
status: open
use-cases:
- SUC-006
depends-on:
- '007'
github-issue: ''
issue: remove-units-from-identifier-names-host-python.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# TestGUI panels and recording: rename unit-suffixed identifiers in sim-error, traces, canvas, and recording glue

## Description

This is planned ticket **007b** in `architecture-update.md`'s Step 3 table
(filed here as sprint ticket 008). It renames unit-suffixed identifiers in
`testgui/sim_prefs.py`, `traces.py`, `operations.py`, `canvas.py`,
`camera_prefs.py`, `live_view.py`, and `recorder.py` — the sim-error/
traces/canvas panels and recording/camera-preview glue. `transport.py`
(ticket 007) is imported by nothing in this ticket's files, but both live
in the same PySide6 application and share incidental state via
`testgui/__main__.py`; sequencing 008 immediately after 007 reduces review
context-switching (Step 5 "Why").

**`sim_prefs.py`'s two-layer key pattern (handle carefully)**:
`DEFAULT_PROFILE["trackwidth_mm"]` is a host-side profile dict keyed by a
unit-suffixed string, mapped through an explicit table
(`{"trackwidth_mm": "trackwidthMm", ...}`) to the real `SIMSET` wire key.
Per the Wire-Compatibility Exclusion Table:
- **RENAME** the host-internal key (`"trackwidth_mm"` → e.g.
  `"trackwidth"`) — this is a convenience string local to `sim_prefs.py`
  and its own tests, not itself wire-visible.
- **EXCLUDE** the mapping table's *value* (`"trackwidthMm"`) — this is the
  real `SIMSET` wire key and must stay byte-identical.
- The file's own docstrings/comments reference both spellings by name —
  read them before editing to avoid conflating the two.

Renames (per Step 5): `testgui/sim_prefs.py` (`trackwidth_mm` host-internal
key → `trackwidth`, mapping-table value `"trackwidthMm"` untouched);
`testgui/traces.py`; `testgui/operations.py` (`rotation_deg` → `rotation
# [deg]`); `testgui/canvas.py`. Matching `tests/testgui/*.py` files are
updated **in this same ticket**.

**`tests/testgui/*.py` file assignment (Open Question 3, resolved here)**:
this ticket owns the **sim-errors / traces / canvas / camera / tour /
recorder** test files (files exercising `sim_prefs.py`, `traces.py`,
`canvas.py`, `camera_prefs.py`, `live_view.py`, `recorder.py`) — the
complement of ticket 007's transport/connection/command/mode/relay/
telemetry/smoke set. Resolve any ambiguous file by grepping its imports.

Total scope: 49 occurrences in `host/robot_radio/testgui/` files touched by
this ticket, plus this ticket's share of `tests/testgui/`'s 169
occurrences.

## Hard Contract (applies to this and every sprint 076 ticket)

- **Pure rename — no behavioral change.** The TestGUI must look and behave
  identically; only Python identifiers change.
- **`sim_prefs.py`'s SIMSET mapping-table VALUES are STABLE** — see the
  two-layer pattern above; only the host-internal key is renamable.
- **Every renamed declaration carries a `# [unit]` comment.**
- **Full suite green throughout**:
  - `uv run python -m pytest -q` remains **2682 passed, 0 failed**.
  - `QT_QPA_PLATFORM=offscreen uv run python -m pytest tests/testgui/ -q`
    remains **579 passed, 2 xfailed**.
- **Cross-cutting kwargs**: any call into `testgui/transport.py` (ticket
  007, already renamed) using a renamed keyword argument must already use
  the converged name; fix any stale one found here.
- **Ignore environmental `data/robots` drift.**

## Acceptance Criteria

- [ ] `testgui/sim_prefs.py`: `DEFAULT_PROFILE`'s `"trackwidth_mm"`
      host-internal key is renamed (e.g. to `"trackwidth"`); the mapping
      table's `"trackwidthMm"` SIMSET-key *value* is byte-identical to
      pre-076 (diff-confirm).
- [ ] `testgui/operations.py`: `rotation_deg` → `rotation` with `# [deg]`.
- [ ] `testgui/traces.py`, `testgui/canvas.py`, `testgui/camera_prefs.py`,
      `testgui/live_view.py`, `testgui/recorder.py`: remaining
      unit-suffixed identifiers renamed with `# [unit]` comments.
- [ ] Traces/recordings capture the same data under renamed field names —
      no change to recorded values, only the field name.
- [ ] Matching `tests/testgui/*.py` files (sim-errors/traces/canvas/camera/
      tour/recorder tier) are updated in this same ticket.
- [ ] `sim_prefs.py`'s SIMSET wire-key mapping-table values are
      byte-identical to pre-076 (explicit diff check, per `usecases.md`
      SUC-006's acceptance criteria).
- [ ] `uv run python -m pytest -q` remains 2682 passed, 0 failed.
- [ ] `QT_QPA_PLATFORM=offscreen uv run python -m pytest tests/testgui/ -q`
      remains 579 passed, 2 xfailed.
- [ ] Hard Contract above holds.

## Testing

- **Existing tests to run**: the sim-errors/traces/canvas/camera/tour/
  recorder subset of `tests/testgui/*.py` (grep each file's imports to
  confirm it belongs here vs. ticket 007), run individually first, then
  the full `tests/testgui/` tier, then the default suite.
- **New tests to write**: none required — pure rename.
- **Verification commands**:
  - `QT_QPA_PLATFORM=offscreen uv run python -m pytest tests/testgui/ -q`
    (confirm 579 passed, 2 xfailed).
  - `uv run python -m pytest -q` (confirm 2682 passed, 0 failed).

## Implementation Plan

**Approach**: Handle `sim_prefs.py`'s two-layer key pattern first and most
carefully, since it is the one file in this ticket with a real wire-key
adjacency; then rename the remaining panel files.

1. Read `testgui/sim_prefs.py` in full, including its docstrings, before
   editing — confirm exactly which strings are host-internal keys
   (renamable) vs. mapping-table values (wire keys, excluded).
2. Rename `DEFAULT_PROFILE`'s `"trackwidth_mm"` key and any Python
   identifier referencing it; leave the mapping table's `"trackwidthMm"`
   value untouched.
3. `testgui/operations.py` — rename `rotation_deg` → `rotation`.
4. `testgui/traces.py`, `testgui/canvas.py`, `testgui/camera_prefs.py`,
   `testgui/live_view.py`, `testgui/recorder.py` — rename remaining
   unit-suffixed identifiers.
5. Identify the sim-errors/traces/canvas/camera/tour/recorder subset of
   `tests/testgui/*.py` by grepping each file's imports against this
   ticket's host files; update every renamed identifier and keyword
   call site.
6. Grep this ticket's full file set for every renamed identifier's old
   name, and specifically re-verify `"trackwidthMm"` (the wire-key value)
   is unchanged.
7. Run the identified `tests/testgui/*.py` subset individually, then the
   full `tests/testgui/` tier with `QT_QPA_PLATFORM=offscreen`, then the
   default suite.

**Files to create/modify**:
- `host/robot_radio/testgui/sim_prefs.py`
- `host/robot_radio/testgui/traces.py`
- `host/robot_radio/testgui/operations.py`
- `host/robot_radio/testgui/canvas.py`
- `host/robot_radio/testgui/camera_prefs.py`
- `host/robot_radio/testgui/live_view.py`
- `host/robot_radio/testgui/recorder.py`
- The sim-errors/traces/canvas/camera/tour/recorder subset of
  `tests/testgui/*.py` (exact file list determined by import grep at
  implementation time).

**Testing plan**: Run the identified `tests/testgui/*.py` subset
individually, then `QT_QPA_PLATFORM=offscreen uv run python -m pytest
tests/testgui/ -q` (confirm 579 passed, 2 xfailed), then
`uv run python -m pytest -q` (confirm 2682 passed, 0 failed). Manually
launch the TestGUI and confirm the Sim Errors panel's sliders still push
the correct `SIMSET trackwidthMm=...` wire command.

**Documentation updates**: None in this ticket.
