---
id: '003'
title: Add gui dependency group and testgui package skeleton
status: done
use-cases:
- SUC-001
depends-on: []
issue: plan-robot-test-gui-pyside6.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 003 — Add gui dependency group and testgui package skeleton

## Description

Create the `host/robot_radio/testgui/` package and register it with the build
system. This is the foundation ticket — all subsequent GUI tickets depend on it.
It establishes the import path, the entry point, and the empty `QMainWindow`
with a transport selector `QComboBox`. No transport logic is wired yet.

Corresponds to item 1 in the approved design's ticket breakdown.

## Acceptance Criteria

- [x] `host/pyproject.toml` has a `[dependency-groups] gui` entry containing
  `PySide6>=6.0`.
- [x] `host/robot_radio/testgui/__init__.py` exists (package marker).
- [x] `host/robot_radio/testgui/__main__.py` exists; running
  `python -m robot_radio.testgui` opens a `QMainWindow` without error.
- [x] The window contains at minimum: a transport selector `QComboBox`
  (`Sim / Serial / Relay`), placeholder areas for command rows and operations
  panel, a placeholder `QGraphicsView` for the canvas, and a placeholder
  `QPlainTextEdit` for the log pane.
- [x] The window closes cleanly (no dangling threads — none started yet).
- [x] `uv run python -m pytest tests/simulation` passes (no regressions).
- [x] `uv sync` (without `--group gui`) does NOT pull in PySide6.
- [x] `uv sync --group gui` succeeds and `python -m robot_radio.testgui` opens
  the window.

## Implementation Plan

### Approach

1. Edit `host/pyproject.toml`: add `[dependency-groups] gui = ["PySide6>=6.0"]`.
2. Create `host/robot_radio/testgui/` directory with:
   - `__init__.py` — minimal package marker
   - `__main__.py` — skeleton `QMainWindow` with title "Robot Test GUI" and the
     layout placeholders (QSplitter or QHBoxLayout with stubbed panels)

### Files to create

- `host/robot_radio/testgui/__init__.py`
- `host/robot_radio/testgui/__main__.py`

### Files to modify

- `host/pyproject.toml` — add `[dependency-groups] gui`

### Testing plan

Manual: `cd host && uv sync --group gui && uv run python -m robot_radio.testgui`.
Confirm the window opens and closes cleanly. Run `uv run python -m pytest
tests/simulation` to confirm no regressions from the skeleton addition.

### Documentation updates

None yet. README is written in ticket 010.
