---
id: '008'
title: TraceModel and playfield QGraphicsView canvas with robot marker
status: open
use-cases:
- SUC-007
- SUC-011
depends-on:
- '005'
- '006'
- '007'
issue: plan-robot-test-gui-pyside6.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 008 — TraceModel and playfield QGraphicsView canvas with robot marker

## Description

Implement `testgui/traces.py` (the `TraceModel`) and the right-side
`QGraphicsView` playfield canvas in `app.py`. The `TraceModel` accumulates four
world-cm polylines from incoming `TLMFrame` deltas, using the `tw()` body-to-
world transform pattern from `tests/bench/ccw_square_50.py`. The canvas renders:
the playfield `QPixmap` background, four `QPainterPath` trace paths in their
assigned colors, a robot marker rectangle at the current fused pose (red front
half / blue back half), and per-trace toggle checkboxes.

World-cm coordinates are mapped to pixel coordinates using the playfield
calibration (`playfield_calibration.json`: field 134 cm × 89.3 cm).

This ticket also resolves OQ-2 from the architecture: where to source the
playfield image. The default is the checked-in
`tests/old/playfield_tour/playfield.jpg` + `tests/old/playfield_tour/playfield_calibration.json`.
The programmer should use the project root relative to the installed package to
locate them (e.g., `pathlib.Path(__file__).parents[4] / "tests/old/..."`) or
make the path configurable. Document the resolution in the commit.

Corresponds to item 6 in the approved design's ticket breakdown.

## Acceptance Criteria

- [ ] `host/robot_radio/testgui/traces.py` defines `TraceModel`:
  - Holds four lists of `(x_cm, y_cm)` world points: `camera`, `encoder`,
    `otos`, `fused`.
  - `anchor(x_cm, y_cm, yaw_rad)` sets the initial world pose for the
    body-to-world transform.
  - `feed(frame: TLMFrame)` appends a new point to each trace using the delta
    fields (`frame.enc`, `frame.otos`, `frame.pose`) and the `tw()` transform.
  - `feed_truth(x_cm, y_cm, yaw_rad)` appends a point to the camera trace.
  - `clear()` resets all four lists and the anchor.
  - Each trace has an `enabled` boolean flag.
- [ ] Canvas `QGraphicsView`:
  - Background: playfield `QPixmap` loaded from `playfield.jpg`.
  - Four `QPainterPath` items in colors: green (camera), orange (encoder),
    cyan (OTOS), magenta (fused).
  - Robot marker: a rectangle of fixed pixel size at the fused pose, front half
    filled red, back half filled blue.
  - Per-trace checkboxes in a column beside the canvas; toggling shows/hides
    the corresponding path item.
  - Canvas updates whenever `TraceModel` changes (Qt signal or explicit
    `scene.update()` call).
- [ ] World-cm to pixel mapping uses calibration from `playfield_calibration.json`
  (134 cm × 89.3 cm → canvas pixel dimensions).
- [ ] `TraceModel.clear()` is wired to the Clear Traces button from ticket 007.
- [ ] The trace model and canvas are connected to the transport's `telemetry`
  and `truth` callbacks: each incoming `TLMFrame` calls `traces.feed(frame)`;
  each truth pose calls `traces.feed_truth(...)`.
- [ ] OQ-2 (playfield image path) is resolved and documented in the commit
  message.
- [ ] `uv run python -m pytest tests/simulation` passes.

## Implementation Plan

### Approach

Read `tests/bench/ccw_square_50.py` for the `tw()` pattern (body-to-world
transform using heading radians and mm deltas). Adapt it for `TraceModel.feed()`.

Read `playfield_calibration.json` to understand the coordinate system and
pixel/cm ratio. The world-to-pixel transform is a simple linear scale + offset
(no perspective; the playfield image is top-down).

For the robot marker: draw a `QGraphicsPolygonItem` or `QGraphicsRectItem` split
into two halves via two `QGraphicsRectItem`s. The marker rotates with the fused
heading using `setRotation(heading_deg)`.

For the `tw()` transform: TLM frames deliver mm deltas (`enc.dL`, `enc.dR`,
`otos.dx`, `otos.dy`, etc.) anchored at the start of the command. Accumulate
an absolute world position by integrating deltas from the anchor pose.

### Files to create

- `host/robot_radio/testgui/traces.py` — `TraceModel`

### Files to modify

- `host/robot_radio/testgui/app.py` — replace canvas placeholder with real
  `QGraphicsView`; wire `TraceModel` to telemetry/truth callbacks; wire Clear
  Traces button.

### Reuse

- `tests/bench/ccw_square_50.py` — `tw()` body→world transform pattern (read
  and adapt; do not import from bench scripts directly)
- `tests/old/playfield_tour/playfield.jpg` — default background image
- `tests/old/playfield_tour/playfield_calibration.json` — field dimensions
- `host/robot_radio/robot/protocol.py` — `TLMFrame` field names

### Testing plan

Manual sim run: connect to Sim, send `D 200 200 500`, observe four traces
advancing and diverging. Confirm camera/truth trace (green) tracks
`sim_get_true_pose_*` while encoder trace diverges. Toggle checkboxes. Click
Clear Traces. The headless smoke test (ticket 010) formally validates trace
growth and marker position. Run simulation gate.

### Documentation updates

None yet. README is written in ticket 010.
