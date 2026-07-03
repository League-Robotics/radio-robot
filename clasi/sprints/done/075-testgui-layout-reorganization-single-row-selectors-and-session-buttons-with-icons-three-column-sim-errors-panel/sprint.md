---
id: '075'
title: 'TestGUI layout reorganization: single-row selectors and session buttons with
  icons, three-column sim-errors panel'
status: done
branch: sprint/075-testgui-layout-reorganization-single-row-selectors-and-session-buttons-with-icons-three-column-sim-errors-panel
use-cases: []
issues:
- testgui-layout-space-reorganization.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 075: TestGUI layout reorganization: single-row selectors and session buttons with icons, three-column sim-errors panel

## Goals

Reorganize the TestGUI's left-panel controls and Sim Errors panel to use
vertical space more efficiently, and make the five session buttons
distinguishable by icon. Layout and iconography only — no functional
changes to any handler, wire command, or persisted data.

## Problem

The TestGUI's control area and Sim Errors panel waste vertical space and
are hard to scan: `transport_combo`/`robot_combo` each occupy their own
label+combo row (and `camera_combo` lives in a different panel entirely),
Connect/Disconnect and Record/Pause/Stop are split across two separate
rows, and the Sim Errors panel stacks all ~15 numeric knobs in one tall
column.

## Solution

- Combine `transport_combo`, `robot_combo`, and `camera_combo` (the only
  three combo boxes in the TestGUI) onto one row in the left panel;
  `camera_combo` relocates from the right panel into this row.
- Combine Connect, Disconnect, Record, Pause, and Stop onto one row, each
  with a distinguishing icon from `QStyle.StandardPixmap` (no external
  asset files).
- Rework the Sim Errors panel from one stacked column into three
  side-by-side columns with narrower spin-box widths; OTOS Error and
  Geometry & Actuation share the left column.

See `architecture-update.md` for the full widget/layout tree (before and
after), icon-choice rationale, and three flagged Open Questions
(camera_combo-as-"import" mapping, icon substitutions, left-column
row-count imbalance).

## Success Criteria

- Transport/robot/camera selectors share one row.
- Connect/Disconnect/Record/Pause/Stop share one row, each with a
  distinct icon.
- Sim Errors panel renders as three columns with compact numeric fields;
  OTOS Error and Geometry & Actuation are both in the left column.
- Full test suite stays green with no behavior changes: baseline
  confirmed at 2682 passed / 0 failed (`uv run python -m pytest -q`)
  before this sprint's tickets execute.

## Scope

### In Scope

- Widget re-parenting in `host/robot_radio/testgui/__main__.py`'s
  `_build_main_window()`: selector row, session-button row, Sim Errors
  panel's 3-column layout.
- Icon assignment on the five session buttons via built-in
  `QStyle.StandardPixmap` values.
- Label/spin-box width tuning inside the Sim Errors panel to fit three
  columns.
- Any headless test updates needed if a test happens to assert on
  something layout-sensitive (none currently found — see
  `architecture-update.md`'s Codebase Alignment review).

### Out of Scope

- Any change to handler/click logic, enable/disable state machines, wire
  protocol, or persisted config/profile schemas.
- The `port_edit` row (not one of "the three combo boxes"; stays as-is).
- Renaming any widget's `objectName`.
- Custom-drawn (non-standard-library) icons.

## Test Strategy

No new test scenarios — this is a layout/iconography-only sprint. All
existing TestGUI headless tests locate widgets via
`window.findChild(WidgetType, "object_name")`, which is parent-layout
agnostic; confirmed by grep that no current test asserts widget
parenting, row membership, or label text for any widget this sprint
touches. The bar for done is: full suite stays green
(`uv run python -m pytest -q`, not `uv run pytest`) at the confirmed
baseline of 2682 passed, plus a manual/visual check that icons render and
rows/columns look as designed.

## Architecture Notes

See `architecture-update.md`. Two independent layout modules: the
Session-Initiation Controls Strip (selector row + button row with icons)
and the Sim Errors Panel Layout (3-column reflow). No module dependency
or data-model changes. Self-review verdict: APPROVE WITH CHANGES (three
non-blocking Open Questions carried to ticketing).

## GitHub Issues

(GitHub issues linked to this sprint's tickets. Format: `owner/repo#N`.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Session-initiation controls strip: single-row selectors and iconed session buttons | none |
| 002 | Sim Errors panel three-column reflow | 001 |

Tickets execute serially in the order listed.
