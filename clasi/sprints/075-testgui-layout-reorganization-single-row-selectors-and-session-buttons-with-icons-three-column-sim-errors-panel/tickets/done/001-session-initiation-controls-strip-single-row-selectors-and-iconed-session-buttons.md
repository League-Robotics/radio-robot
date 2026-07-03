---
id: '001'
title: 'Session-initiation controls strip: single-row selectors and iconed session
  buttons'
status: done
use-cases:
- SUC-001
- SUC-002
depends-on: []
github-issue: ''
issue: testgui-layout-space-reorganization.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Session-initiation controls strip: single-row selectors and iconed session buttons

## Description

`host/robot_radio/testgui/__main__.py`'s `_build_main_window()` currently
spreads the TestGUI's three combo boxes and five session buttons across
six separate rows in two different panels:

- `transport_combo` (own label+combo row, lines 383-389) and `robot_combo`
  (own label+combo row, lines 400-414) each get a full row in the left
  panel.
- `camera_combo` (lines 900-918) lives in the RIGHT panel's `camera_row`,
  above the canvas — a different panel entirely from the other two combos.
- `connect_btn`/`disconnect_btn` share `btn_row` (430-443); `record_btn`/
  `pause_btn`/`stop_btn` share a separate `rec_row` (445-461) — two rows
  for what SUC-002 treats as one five-action group.

This ticket implements the Session-Initiation Controls Strip module from
`architecture-update.md` (Step 3, Step 4b "After" diagram, Step 5 "What
Changed" items 1-2 and 4): collapse the three combos into one
`selector_row` `QHBoxLayout` in the left panel (moving `camera_combo` out
of the right panel), and collapse the five buttons into one
`session_btn_row` `QHBoxLayout`, each button gaining a
`QStyle.StandardPixmap` icon per Design Rationale Decision 3:
`connect_btn`→`SP_DialogYesButton`, `disconnect_btn`→`SP_DialogNoButton`,
`record_btn`→`SP_MediaPlay`, `pause_btn`→`SP_MediaPause`,
`stop_btn`→`SP_MediaStop`. `port_label`/`port_edit` (416-428) are
explicitly OUT of scope — they stay a separate row, unchanged (Decision 1:
the issue's "three combo boxes" are transport/robot/camera; the port field
is a `QLineEdit`, not a combo).

This is a pure widget-tree rearrangement: every `objectName`, tooltip,
population routine, and click/change handler is reused unchanged. See
`usecases.md` SUC-001, SUC-002 for the full before/after behavioral
contract (identical — only visual grouping changes).

## Acceptance Criteria

- [x] `transport_combo`, `robot_combo`, and `camera_combo` are children of
      one new `QHBoxLayout` (`selector_row`) in the left panel, in that
      order, each preceded by its existing label
      (`transport_label`/`robot_label`/`camera_combo_label`).
- [x] `camera_combo`'s construction (currently lines 900-918, inside the
      right-panel block) is relocated so it can be added to `selector_row`
      before the left panel is finalized — `camera_combo`'s `objectName`,
      tooltip text, and the fact that it is populated later by
      `_populate_camera_combo()` are all unchanged.
- [x] The old `camera_row` widget/wrapper is removed entirely from the
      right panel; `right_layout` goes directly from `mode_label` (890-898)
      to `right_splitter` (920+), with no gap widget left in its place.
- [x] `port_label`/`port_edit` (416-428) keep their own row, unchanged,
      still enabled/disabled the same way on transport change; they are
      NOT merged into `selector_row`.
- [x] `connect_btn`, `disconnect_btn`, `record_btn`, `pause_btn`, and
      `stop_btn` are children of one new `QHBoxLayout` (`session_btn_row`)
      replacing `btn_row` (430-443) and `rec_row` (445-461), in that exact
      order (Connect, Disconnect, Record, Pause, Stop — per SUC-002's Main
      Flow step 2).
- [x] Each button has a non-null icon set via
      `style().standardIcon(QStyle.StandardPixmap.SP_...)` (no external
      asset files): `connect_btn`→`SP_DialogYesButton`,
      `disconnect_btn`→`SP_DialogNoButton`, `record_btn`→`SP_MediaPlay`,
      `pause_btn`→`SP_MediaPause`, `stop_btn`→`SP_MediaStop`. No two
      buttons share the same `QStyle.StandardPixmap` value.
- [x] No change to any of the eight widgets' `objectName`, to
      `disconnect_btn`/`pause_btn`/`stop_btn`'s initial
      `setEnabled(False)`, or to any click/change handler
      (`_on_transport_changed`, `_on_robot_changed`,
      `_on_camera_combo_changed`, connect/disconnect click wiring,
      record/pause/stop click wiring) — only parent-layout membership and
      icon assignment change.
- [x] `left_layout.insertWidget(left_layout.count() - 1, ops_panel)`
      (~line 1844) still inserts `ops_panel` immediately before the
      trailing `addStretch()`, unaffected by the row consolidation above
      it (verify by inspection/manual run — `ops_panel` should render in
      the same relative position, just with a shorter block of rows above
      it).
- [x] `splitter.setSizes([360, 840])` (line 938) is widened modestly for
      the left pane to comfortably fit the wider `selector_row` (exact
      pixel values are this ticket's implementation call; no test asserts
      a specific size).
- [x] Headless tests that reference any of the eight touched widgets pass
      unmodified: `tests/testgui/test_camera_combo.py`,
      `tests/testgui/test_transport.py`,
      `tests/testgui/test_mode_indicator.py`,
      `tests/testgui/test_relay_discovery.py`,
      `tests/testgui/test_recorder.py`,
      `tests/testgui/test_operations.py`,
      `tests/testgui/test_calibration_push_on_connect.py`.
- [x] Full suite (`uv run python -m pytest -q`) passes at the 2682
      baseline confirmed in `architecture-update.md`, zero unexplained
      failures. Ignore any `data/robots` drift — noted as environmental in
      the sprint's hard contract.

## Testing

- **Existing tests to run**: `tests/testgui/test_camera_combo.py`,
  `tests/testgui/test_transport.py`, `tests/testgui/test_mode_indicator.py`,
  `tests/testgui/test_relay_discovery.py`, `tests/testgui/test_recorder.py`,
  `tests/testgui/test_operations.py`,
  `tests/testgui/test_calibration_push_on_connect.py` individually first
  (these grep-confirmed reference the eight widgets this ticket touches),
  then the full suite.
- **New tests to write**: None required — this is a layout/iconography-only
  change and every existing test locates widgets via
  `window.findChild(WidgetType, "object_name")`, which is parent-layout
  agnostic (confirmed in `architecture-update.md`'s Codebase Alignment
  review). No test updates are expected.
- **Verification command**: `uv run python -m pytest -q`

## Implementation Plan

**Approach**: Work entirely inside `_build_main_window()` in
`host/robot_radio/testgui/__main__.py`.

1. Relocate `camera_combo`/`camera_combo_label`'s construction (currently
   lines 900-918, in the right-panel block built after the left panel) up
   into the left-panel construction section, since it must exist before
   `selector_row` (part of `left_layout`, finalized before the right panel
   starts building) can include it. Its `setObjectName`, tooltip text, and
   the fact that population (`_populate_camera_combo`) and the
   change-handler connection happen later (once `ops_ctrl` is in scope)
   are unaffected by moving the constructor call earlier — only the
   variable's parent-widget assignment changes.
2. Build `selector_row = QWidget()`, `selector_layout = QHBoxLayout(selector_row)`
   with tight margins (matching the existing `btn_row`/`camera_row`
   pattern: `setContentsMargins(0, 0, 0, 0)`); add
   `transport_label`/`transport_combo`, `robot_label`/`robot_combo`,
   `camera_combo_label`/`camera_combo` to it in that order; call
   `left_layout.addWidget(selector_row)` once, replacing the four
   individual `left_layout.addWidget(...)` calls at 384/389/401/414.
3. Leave `port_label`/`port_edit`'s row (416-428) exactly as-is,
   positioned immediately after `selector_row`.
4. Build `session_btn_row = QWidget()`,
   `session_btn_layout = QHBoxLayout(session_btn_row)` with tight margins;
   construct the five buttons exactly as today (object names, initial
   `setEnabled(False)` for `disconnect_btn`/`pause_btn`/`stop_btn`
   unchanged), add each to `session_btn_layout` in Connect/Disconnect/
   Record/Pause/Stop order, then `left_layout.addWidget(session_btn_row)`
   once — replacing the `btn_row`/`rec_row` construction and their two
   `left_layout.addWidget` calls (443, 461).
5. Assign each button's icon right after construction:
   `connect_btn.setIcon(window.style().standardIcon(QStyle.StandardPixmap.SP_DialogYesButton))`
   (and the corresponding pixmap for each of the other four, per the
   mapping in Acceptance Criteria). Check whether `QStyle` is already
   imported in the file's Qt-widget import block; add the import if not.
6. Remove the old `camera_row` block entirely from the right-panel section
   (900-918) — `right_layout` should go straight from `mode_label`
   (890-898) to `right_splitter` (920+).
7. Widen `splitter.setSizes([360, 840])` (938) modestly, e.g.
   `[420, 780]` — pick a value that keeps the wider selector row and
   five-icon button row from feeling cramped; this is not test-covered,
   judge visually.
8. Sanity-check `left_layout.insertWidget(left_layout.count() - 1, ops_panel)`
   (~1844) still lands `ops_panel` right before the trailing
   `addStretch()` — it addresses `left_layout` by position relative to the
   end, so it should be unaffected by consolidating earlier rows, but
   confirm by running the GUI or reading the surrounding code after the
   edit.

**Files to create/modify**:
- `host/robot_radio/testgui/__main__.py` — the only file touched (per
  `architecture-update.md`'s Anti-Pattern Detection: "every change is
  confined to `_build_main_window()`"; no other file needs a change).

**Testing plan**: Run the seven widget-specific test files listed above
individually first (fast signal if a rename/reparent broke a `findChild`
lookup), then run the full suite (`uv run python -m pytest -q`) and
confirm the 2682-passed / 0-failed baseline holds. Manually launch the
TestGUI (`uv run python -m robot_radio.testgui`, or the project's usual
launch command) and visually confirm: one row with Transport/Robot/Camera
combos, one row below it with five icon-bearing buttons in
Connect/Disconnect/Record/Pause/Stop order, and that Connect/Disconnect
still enables/disables correctly and Record/Pause/Stop still follow the
existing state machine.

**Documentation updates**: None required — no README, user guide, or
wire-protocol doc describes the exact row/panel placement of these
widgets; `architecture-update.md` already documents the target layout for
this sprint.
