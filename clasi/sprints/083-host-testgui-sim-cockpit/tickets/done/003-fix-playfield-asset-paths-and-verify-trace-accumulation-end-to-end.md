---
id: '003'
title: Fix playfield asset paths and verify trace accumulation end-to-end
status: done
use-cases:
- SUC-003
depends-on:
- '001'
github-issue: ''
issue: host-testgui-sim-cockpit.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Fix playfield asset paths and verify trace accumulation end-to-end

## Description

`host/robot_radio/testgui/canvas.py` resolves the bundled playfield
image/calibration at `tests/old/playfield_tour/playfield.jpg` and
`.../playfield_calibration.json`. The greenfield rebuild (sprint 077) parked
the pre-rebuild tree at `tests_old/`, not `tests/old/` — the files actually
live at `tests_old/old/playfield_tour/playfield.jpg` and
`tests_old/old/playfield_tour/playfield_calibration.json` (confirmed by
`find`). Today this silently falls back to the grey placeholder background
(`canvas.py`'s existing graceful-degradation path), which works but hides the
real playfield image the operator expects to see.

`protocol.py`'s `TLMFrame` dataclass and `traces.py`'s `TraceModel.feed()`
already match the sprint-082 `TLM` format (`encpose=`/`otos=`/`pose=` as
absolute `(x_mm, y_mm, heading_cdeg)` triples) — no logic changes are needed
there. This ticket is the asset-path fix plus the end-to-end verification
that traces actually accumulate correctly against a working `SimTransport`
(ticket 001), closing the loop the architecture doc's Step 1 investigation
opened but did not itself fix.

## Acceptance Criteria

- [x] `canvas.py`'s three asset-path constants (`_PLAYFIELD_IMAGE`,
      `_PLAYFIELD_DESKEWED`, `_PLAYFIELD_CALIB`) are corrected to resolve
      under `tests_old/old/playfield_tour/` instead of
      `tests/old/playfield_tour/`.
- [x] `_load_calibration()` and `_build_playfield_calibration()` successfully
      read the real calibration JSON (no longer silently falling back to
      `_FIELD_WIDTH_CM_DEFAULT`/`_FIELD_HEIGHT_CM_DEFAULT`) — verified by a
      test asserting the loaded field dimensions match the JSON's actual
      `width`/`height` values, not the hardcoded defaults.
  - Note: the startup/live-view behavior (grey placeholder until an
    `aprilcam` daemon grab, or `TESTGUI_LOAD_STATIC_PLAYFIELD=1` to force the
    static image) is unchanged — this ticket only fixes what path the
    debug-override / calibration-dimension code resolves to.
- [x] An integration test drives a connected `SimTransport` (ticket 001)
      directly (bypassing `KeyboardDriver`, e.g. via `transport.send("DEV DT
      PORTS 1 2")` + `transport.send("DEV DT VW 200 0 0")`), ticks it forward,
      feeds the resulting `TLMFrame`s into a `TraceModel`, and asserts the
      `encoder`, `otos`, and `fused` trace lists all grow with plausible
      forward-motion points.
- [x] A second scenario in the same test (or a follow-on) feeds ground-truth
      poses via `feed_truth()` (sourced from `SimTransport`'s `on_truth`
      callback / `conn.get_true_pose()`) and asserts the `camera` trace grows
      in step with the others during a short straight drive.
- [x] No changes to `traces.py`'s transform math (`_tw`/`_rw`/baseline
      handling) — if the existing logic is found to be wrong during this
      verification, that is a separate bug to report, not silently patched
      here without updating the architecture doc first.

## Testing

- **Existing tests to run**: none yet in `tests/` (ported in ticket 004,
  including whatever of `tests_old/testgui/test_canvas.py`/
  `test_traces.py` still applies).
- **New tests to write**: a headless (`QT_QPA_PLATFORM=offscreen`, no
  `QApplication` needed for the `TraceModel`-only assertions) test asserting
  the corrected asset paths exist and calibration loads real dimensions; an
  integration test per the two acceptance-criteria scenarios above, using
  `SimTransport` from ticket 001.
- **Verification command**: `QT_QPA_PLATFORM=offscreen uv run pytest tests/testgui -k "traces or canvas"` (once ticket 004 creates the directory; until then, run the new test files directly with `uv run pytest <path>`).
