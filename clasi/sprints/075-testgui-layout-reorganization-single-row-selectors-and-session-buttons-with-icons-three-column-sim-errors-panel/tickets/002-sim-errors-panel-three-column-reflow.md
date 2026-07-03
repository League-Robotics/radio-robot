---
id: '002'
title: Sim Errors panel three-column reflow
status: open
use-cases:
- SUC-003
depends-on:
- '001'
github-issue: ''
issue: testgui-layout-space-reorganization.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sim Errors panel three-column reflow

## Description

`sim_errors_group`'s internal layout (`host/robot_radio/testgui/__main__.py`,
lines 656-876) is a single `QVBoxLayout` stacking four labeled sections in
this code order: Encoder Report Error (3 spins, 701-713), Body-Truth
Scrub (3 spins, 716-728), Geometry & Actuation (3 spins, 731-748), OTOS
Error (6 spins, 751-775) — 15 spin rows plus two section-less rows
(section labels), ~19 rows tall in one column. Every row is built by
`_make_sim_err_spin()` (664-684), which hardcodes both the destination
layout (`sim_errors_layout`, line 683) and the fixed pixel widths (label
120px at 673, spin 90px at 680); section headers are built by
`_add_sim_err_section_label()` (686-698), which also hardcodes
`sim_errors_layout` as its destination (698).

This ticket implements the Sim Errors Panel Layout module from
`architecture-update.md` (Step 3, Step 4b "After" diagram, Step 5 "What
Changed" item 3, Design Rationale Decision 2): reflow the panel into a
3-column `QHBoxLayout` of column `QVBoxLayout`s — LEFT = OTOS Error + 
Geometry & Actuation, MIDDLE = Encoder Report Error, RIGHT = Body-Truth
Scrub — with narrower per-row widths so three columns fit in roughly the
panel's current overall width, and the Apply/From-Calibration button row
(777-787) placed below all three columns rather than inside any single
column. Depends on ticket 001 because both tickets edit
`_build_main_window()` in the same file — 001 lands the selector/button
row consolidation first so this ticket applies cleanly on top rather than
via a parallel edit to the same function.

Every `sim_err_*` spin box keeps its `objectName`, range, decimals, and
default value; `_on_sim_errors_apply`/`_on_sim_errors_from_cal` and
`sim_prefs.py` are untouched — this is a pure layout reflow. See
`usecases.md` SUC-003 for the full before/after behavioral contract
(identical — only column grouping and field width change).

## Acceptance Criteria

- [ ] `sim_errors_group`'s content is organized into exactly 3 side-by-side
      column `QVBoxLayout`s (e.g. via a `columns_row` `QWidget`/`QHBoxLayout`
      added as the first child of `sim_errors_layout`, each column a
      `QWidget`/`QVBoxLayout` added to `columns_row`).
- [ ] LEFT column contains, in order: the "OTOS Error" section label and
      its 6 spin rows (`sim_err_otos_linear`, `sim_err_otos_yaw`,
      `sim_err_otos_lin_scale`, `sim_err_otos_ang_scale`,
      `sim_err_otos_lin_drift`, `sim_err_otos_yaw_drift`), followed by the
      "Geometry & Actuation" section label and its 3 spin rows
      (`sim_err_motor_offset_l`, `sim_err_motor_offset_r`,
      `sim_err_trackwidth`) — 9 spin rows total in this column, per
      `architecture-update.md` Decision 2.
- [ ] MIDDLE column contains the "Encoder Report Error" section label and
      its 3 spin rows (`sim_err_encoder_mm`, `sim_err_enc_scale_l`,
      `sim_err_enc_scale_r`).
- [ ] RIGHT column contains the "Body-Truth Scrub" section label and its 3
      spin rows (`sim_err_slip_turn`, `sim_err_body_rot_scrub`,
      `sim_err_body_lin_scrub`).
- [ ] All 15 `sim_err_*` spin boxes keep their existing `objectName`,
      `setRange`, `setDecimals`, and default `setValue` arguments — no
      change to any of the 15 `_make_sim_err_spin(...)` call sites' value
      arguments (only which column layout each row lands in changes).
- [ ] `_make_sim_err_spin()`'s per-row label/spin fixed widths (120px/90px
      at lines 673/680) are narrowed so three columns fit without
      excessive panel widening (exact values are this ticket's
      implementation call — e.g. ~90-100px label, ~65-70px spin).
- [ ] `sim_errors_apply_btn` and `sim_errors_from_cal_btn` (777-787) keep
      their `objectName`s and click-handler wiring (`_on_sim_errors_apply`,
      `_on_sim_errors_from_cal`, connected at 874-875) unchanged, and their
      row renders below all three columns — not split across or nested
      inside any single column.
- [ ] `sim_errors_group`'s own `objectName` and its placement in
      `left_layout` (`left_layout.addWidget(sim_errors_group)` at line
      876) are unchanged.
- [ ] No change to `_on_sim_errors_apply` (789-822),
      `_on_sim_errors_from_cal` (824-872), `sim_prefs.py`, or
      `SimTransport.apply_error_profile` — none of this ticket's changes
      touch how a value moves from a spin box to the wire.
- [ ] Headless tests referencing any Sim Errors widget pass unmodified:
      `tests/testgui/test_sim_errors_panel.py`,
      `tests/testgui/test_sim_errors_from_cal_button.py`,
      `tests/testgui/test_sim_errors_from_calibration.py`,
      `tests/testgui/test_calibration_push_on_connect.py`.
- [ ] Full suite (`uv run python -m pytest -q`) passes at the 2682
      baseline (plus ticket 001's changes, which are layout-only and add
      no tests), zero unexplained failures. Ignore any `data/robots`
      drift — noted as environmental in the sprint's hard contract.

## Testing

- **Existing tests to run**: `tests/testgui/test_sim_errors_panel.py`,
  `tests/testgui/test_sim_errors_from_cal_button.py`,
  `tests/testgui/test_sim_errors_from_calibration.py`,
  `tests/testgui/test_calibration_push_on_connect.py` individually first
  (grep-confirmed references to `sim_err_*`/`sim_errors_group`/
  `sim_errors_apply_btn`/`sim_errors_from_cal_btn`), then the full suite.
- **New tests to write**: None required — existing tests locate every
  spin box and button via `window.findChild(WidgetType, "object_name")`,
  which is column/parent-layout agnostic (confirmed in
  `architecture-update.md`'s Codebase Alignment review). No test updates
  are expected.
- **Verification command**: `uv run python -m pytest -q`

## Implementation Plan

**Approach**: Work entirely inside `_build_main_window()`'s Sim Errors
panel block (656-876) in `host/robot_radio/testgui/__main__.py`, building
on ticket 001's already-landed selector/button-row changes.

1. Parameterize the two section-building helpers so callers control the
   destination layout instead of always targeting `sim_errors_layout`:
   change `_make_sim_err_spin(object_name, label, value, lo, hi, decimals)`
   to `_make_sim_err_spin(target_layout, object_name, label, value, lo, hi,
   decimals)`, replacing `sim_errors_layout.addWidget(row)` (683) with
   `target_layout.addWidget(row)`; likewise change
   `_add_sim_err_section_label(title)` to
   `_add_sim_err_section_label(target_layout, title)`, replacing
   `sim_errors_layout.addWidget(lbl)` (698) with
   `target_layout.addWidget(lbl)`. Narrow the fixed widths at lines 673
   (`lbl.setFixedWidth(120)`) and 680 (`spin.setFixedWidth(90)`) to fit
   three columns.
2. Build `columns_row = QWidget()`,
   `columns_layout = QHBoxLayout(columns_row)` with tight margins; build
   three column widgets `col_left`, `col_mid`, `col_right`, each with its
   own `QVBoxLayout` (`col_left_layout`, `col_mid_layout`,
   `col_right_layout`), and add all three to `columns_layout` in that
   order.
3. Update each existing section's calls to pass the correct column
   layout — no need to physically reorder the Python statements, just
   change which layout variable each block's calls target: the "Encoder
   Report Error" block (701-713) targets `col_mid_layout`; the
   "Body-Truth Scrub" block (716-728) targets `col_right_layout`; the
   "Geometry & Actuation" block (731-748) and the "OTOS Error" block
   (751-775) both target `col_left_layout` (OTOS section first, per the
   existing code's OTOS-last ordering — reorder these two blocks in the
   left column so OTOS Error appears above Geometry & Actuation, matching
   `architecture-update.md`'s stated column content order, or keep
   Geometry-then-OTOS if that reads better; SUC-003 only requires both
   sections be reachable in the left column, not a specific sub-order).
4. Replace the current interleaved `sim_errors_layout.addWidget(row)`
   calls (one per section, previously issued automatically by
   `_make_sim_err_spin`/`_add_sim_err_section_label`) with two top-level
   additions to `sim_errors_layout`: `sim_errors_layout.addWidget(columns_row)`
   first, then `sim_errors_layout.addWidget(sim_errors_btn_row)` (787)
   second — the button row stays exactly as constructed today (777-787),
   only its position relative to the (now columnar) spin rows changes.
5. Leave `_on_sim_errors_apply`/`_on_sim_errors_from_cal` (789-872) and
   their `clicked.connect` wiring (874-875) untouched — they reference
   spin box variables directly, not layout structure.

**Files to create/modify**:
- `host/robot_radio/testgui/__main__.py` — the only file touched (same
  function as ticket 001; per `architecture-update.md`'s Anti-Pattern
  Detection, no other file needs a change for this module either).

**Testing plan**: Run the four Sim-Errors-specific test files listed
above individually first, then the full suite (`uv run python -m pytest
-q`), confirming the 2682-passed / 0-failed baseline holds (net of ticket
001's changes, which add no tests). Manually launch the TestGUI with
"Sim" selected as the transport and visually confirm: three side-by-side
columns (OTOS Error above Geometry & Actuation on the left, Encoder
Report Error in the middle, Body-Truth Scrub on the right), narrower spin
boxes, and the Apply/From Calibration buttons on one row below all three
columns; click Apply and From Calibration to confirm both still behave
identically to before.

**Documentation updates**: None required — no README, user guide, or
wire-protocol doc describes the Sim Errors panel's exact column/row
layout; `architecture-update.md` already documents the target layout for
this sprint.
