---
id: '006'
title: Schema-driven command-entry rows and Send buttons
status: done
use-cases:
- SUC-005
depends-on:
- '004'
issue: plan-robot-test-gui-pyside6.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 006 — Schema-driven command-entry rows and Send buttons

## Description

Build the command-entry panel in `app.py`: six rows, one per motion command
(S, T, D, R, TURN, G), each constructed from a data table so adding a new
command only requires a new table entry. Each row has a label, one labeled
`QLineEdit`/`QSpinBox` per parameter, and a **Send** button. Clicking Send
assembles the wire string from the row's current field values and calls
`transport.command(line)`. The reply and the sent line appear in the log pane.

This ticket can run in parallel with ticket 005 (SimTransport) since both
depend only on ticket 004 (Transport ABC). Commands will work immediately with
Serial/Relay transports.

Corresponds to item 4 in the approved design's ticket breakdown.

## Acceptance Criteria

- [x] Six command rows are present and labeled: S, T, D, R, TURN, G.
- [x] Each row assembles the correct wire string on Send:
  - S: `S <left_mms> <right_mms>`
  - T: `T <left_mms> <right_mms> <ms>`
  - D: `D <left_mms> <right_mms> <mm>`
  - R: `R <speed> <radius_mm>`
  - TURN: `TURN <heading_cdeg>` (eps field optional; if non-zero: `TURN <h> eps=<e>`)
  - G: `G <x_mm> <y_mm> <speed>`
- [x] TURN row: heading and eps fields take centidegrees directly (wire values). The
  default for heading is 9000 (= 90°) and eps default is 0 (omitted). This matches
  the firmware's native centidegree convention.
- [x] Send button is disabled when no transport is connected.
- [x] Sent string and firmware reply appear timestamped in the log pane.
- [x] Rows are built from a data structure (list of dicts or dataclass), not six
  separate hardcoded blocks. Adding a new command requires only adding a table entry.
- [x] `uv run python -m pytest tests/simulation` passes.

## Implementation Plan

### Approach

Define a `COMMANDS` list at the top of `app.py` (or a separate `commands.py`):
```python
COMMANDS = [
    {"label": "S",    "params": [("left_mms", int), ("right_mms", int)],             "fmt": "S {0} {1}"},
    {"label": "T",    "params": [("left_mms", int), ("right_mms", int), ("ms", int)],"fmt": "T {0} {1} {2}"},
    {"label": "D",    "params": [("left_mms", int), ("right_mms", int), ("mm", int)],"fmt": "D {0} {1} {2}"},
    {"label": "R",    "params": [("speed", int), ("radius_mm", int)],                 "fmt": "R {0} {1}"},
    {"label": "TURN", "params": [("heading_deg", int), ("eps_deg", int, 0)],          "fmt": ...},  # cdeg conversion
    {"label": "G",    "params": [("x_mm", int), ("y_mm", int), ("speed", int)],       "fmt": "G {0} {1} {2}"},
]
```

A `build_command_row(spec)` helper creates one `QHBoxLayout` per entry. The
TURN row's Send handler applies `* 100` to degree values before formatting.

### Files to modify

- `host/robot_radio/testgui/app.py` (or `__main__.py` if not yet extracted) —
  add COMMANDS table and `build_command_row` helper; replace the command-row
  placeholder with real rows.

### Testing plan

Manual: connect via Sim or Serial. Click Send on each row with sample values.
Confirm wire strings in log match the spec. Confirm Send is disabled before
connecting. The headless smoke test (ticket 010) will formally validate wire
strings programmatically. Run simulation gate.

### Documentation updates

None yet. README is written in ticket 010.
