---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 075 Use Cases

These SUCs describe TestGUI operator ergonomics (a dev/test tool), not
robot behavior, so none narrows an existing entry in `docs/usecases.md`
(UC-001..019 are all wire-protocol/robot-behavior use cases). No parent UC
is cited, matching the precedent set by sprint 063's TestGUI SUCs (mode
indicator, relay auto-discovery, live camera view), which also cite no
parent. This is a layout-and-iconography-only sprint: no wire protocol,
handler, or persisted-data behavior changes. Every acceptance criterion
below is a visual/structural assertion (widget position, icon presence),
not a behavioral one.

## SUC-001: Select transport, robot, and camera from a single row

- **Actor**: Developer or robot operator opening the Test GUI
- **Preconditions**: Test GUI is open.
- **Main Flow**:
  1. User looks at the top of the left panel.
  2. The Transport selector, Robot selector, and Camera selector (the
     three `QComboBox` widgets in the window — transport/robot/camera are
     the only three that exist) appear side by side on one row, each with
     its own short label.
  3. User changes any one of the three selections; existing behavior is
     unchanged (transport switch shows/hides the port field and Sim
     Errors panel; robot switch reloads that robot's config; camera
     switch triggers a playfield refresh).
- **Postconditions**: All three selectors are visible and usable without
  scrolling; their existing change-handlers still fire exactly as before.
- **Acceptance Criteria**:
  - [ ] `transport_combo`, `robot_combo`, and `camera_combo` are children
        of the same row widget/layout (single `QHBoxLayout`).
  - [ ] Each selector still carries its existing `objectName`.
  - [ ] No change to `_on_transport_changed`, `_on_robot_changed`, or
        `_on_camera_combo_changed` — only their widgets' parent layout
        changes.
  - [ ] Headless tests that locate these combos via `findChild` still
        pass unmodified (they search the whole window, not a specific
        parent).

## SUC-002: Start, pause, and stop a session from one button row with icons

- **Actor**: Developer or robot operator running a connect/record session
- **Preconditions**: Test GUI is open.
- **Main Flow**:
  1. User looks below the selector row.
  2. Connect, Disconnect, Record, Pause, and Stop appear together on one
     row, in that order.
  3. Each button carries a distinct icon in addition to its text label,
     so the five actions are visually distinguishable at a glance without
     reading text (e.g. skimming the row while watching the canvas).
  4. User clicks any button; existing enable/disable rules and click
     handlers are unchanged (Connect/Disconnect toggle on
     connection state; Record/Pause/Stop follow the existing
     Idle/Recording/Paused state machine).
- **Postconditions**: All five controls are visible on one row with
  icons; button behavior is bit-for-bit identical to before the sprint.
- **Acceptance Criteria**:
  - [ ] `connect_btn`, `disconnect_btn`, `record_btn`, `pause_btn`, and
        `stop_btn` are children of the same row widget/layout.
  - [ ] Each of the five buttons has a non-null `icon()` set via a
        built-in `QStyle.StandardPixmap` (no external asset files).
  - [ ] No two of the five buttons share the same icon.
  - [ ] No change to any button's `objectName`, enable/disable wiring, or
        click handler.
  - [ ] Headless tests exercising Connect/Disconnect/Record/Pause/Stop
        behavior pass unmodified.

## SUC-003: Read and edit sim-error knobs in a compact three-column panel

- **Actor**: Developer tuning the simulator's plant/sensor error model
- **Preconditions**: Test GUI is open with "Sim" selected as the
  transport (Sim Errors panel visible).
- **Main Flow**:
  1. User looks at the Sim Errors group box.
  2. Its ~15 numeric knobs are laid out in three side-by-side columns
     instead of one tall stacked list, with visibly narrower spin boxes.
  3. The OTOS Error section and the Geometry & Actuation section are both
     in the left column; Encoder Report Error and Body-Truth Scrub occupy
     the other two columns.
  4. User edits a value and clicks Apply (or From Calibration); both
     buttons remain reachable below the three columns and behave exactly
     as before.
- **Postconditions**: The panel is visibly shorter than before (three
  columns instead of one) while every knob remains present, editable, and
  wired to the same save/apply logic.
- **Acceptance Criteria**:
  - [ ] All 15 existing `sim_err_*` spin boxes are present with their
        existing `objectName`s and unchanged min/max/decimals/default
        values.
  - [ ] The OTOS Error section's 6 spin boxes and the Geometry &
        Actuation section's 3 spin boxes are both reachable within the
        panel's left-most column.
  - [ ] The panel's overall layout uses 3 side-by-side columns, each
        narrower than today's single-column row width.
  - [ ] `sim_errors_apply_btn` and `sim_errors_from_cal_btn` keep their
        `objectName`s and click handlers; their row is not split across
        columns.
  - [ ] Headless tests that read/write `sim_err_*` spin boxes via
        `findChild` and exercise Apply/From-Calibration pass unmodified.
