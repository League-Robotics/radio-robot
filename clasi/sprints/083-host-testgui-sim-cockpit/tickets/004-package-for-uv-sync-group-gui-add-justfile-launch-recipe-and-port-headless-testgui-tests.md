---
id: '004'
title: Package for uv sync --group gui, add justfile launch recipe, and port headless
  TestGUI tests
status: open
use-cases: [SUC-006]
depends-on: ['001', '002', '003']
github-issue: ''
issue: host-testgui-sim-cockpit.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Package for uv sync --group gui, add justfile launch recipe, and port headless TestGUI tests

## Description

The GUI is currently only runnable/testable ad hoc: there is no `justfile`
launch recipe, and its non-visual logic (`transport.py`, `drive.py`,
`traces.py`, `sim_prefs.py`, `canvas.py`'s Qt-free helpers, `commands.py`,
`operations.py`'s pure helpers) has no coverage in the rebuilt `tests/` tree
— the old suite lives in `tests_old/testgui/` (23 test files, excluded from
collection by `pyproject.toml`'s `norecursedirs`). This ticket makes the
sprint's cockpit runnable and test-verifiable, and is this sprint's
end-to-end acceptance gate: it is the ticket that proves tickets 001-003
actually deliver "launch GUI -> Sim connect -> arrow-key drive moves avatar +
traces -> error profile separates encoder trace from truth" together, not
just individually.

## Acceptance Criteria

- [ ] `uv sync --group gui` installs cleanly and
      `QT_QPA_PLATFORM=offscreen uv run python -c "import robot_radio.testgui"`
      succeeds; `robot_radio.testgui` and every submodule remain importable
      **without** PySide6 installed (verified by a plain `uv run python -c
      "import robot_radio.testgui.drive, robot_radio.testgui.transport,
      robot_radio.testgui.traces, robot_radio.testgui.sim_prefs,
      robot_radio.testgui.commands"` in the base environment, no `gui` group)
      — the lazy-import discipline documented in every module's docstring is
      preserved, not broken by any ticket 001-003 change.
- [ ] A `justfile` recipe (e.g. `testgui`) runs
      `uv run python -m robot_radio.testgui`, gated on `build-sim` having
      been run (matching the existing `build-sim` recipe's doc comment
      style); document the one-time `uv sync --group gui` prerequisite in the
      recipe's comment.
- [ ] `tests/testgui/` is created (new package, `__init__.py` +
      `conftest.py` as needed) and populated by porting the still-applicable
      files from `tests_old/testgui/`: at minimum `test_transport.py`,
      `test_drive.py`, `test_traces.py`, `test_sim_prefs.py`,
      `test_canvas.py`, `test_commands.py`, `test_smoke.py`. Each ported
      file is updated in place for the sprint-083 API changes (tickets
      001-003) — not copied verbatim if it asserts the old `VW`/`SIMSET`/
      `sim.get_true_pose()` surface.
  - Files that only exercise out-of-scope features (tours, GOTO, Sync Pose,
    Set Origin, calibration-push, live camera view — e.g.
    `test_calibration_push_on_connect.py`, `test_tour_stop.py`,
    `test_tour_idle_detection.py`, `test_tour1_geometry.py`,
    `test_set_origin.py`, `test_live_view.py`, `test_live_frame_bridge.py`,
    `test_camera_combo.py`, `test_camera_prefs.py`,
    `test_sim_errors_from_cal_button.py`,
    `test_sim_errors_from_calibration.py`, `test_mode_indicator.py`,
    `test_recorder.py`, `test_relay_discovery.py`,
    `test_telemetry_gating.py`, `test_sim_errors_panel.py`) are left
    un-ported this sprint — note which files were intentionally skipped and
    why in the PR/ticket close-out, so the gap is traceable rather than
    silently dropped.
- [ ] `pyproject.toml`'s `testpaths` gains `"tests/testgui"` alongside
      `"tests/sim"`/`"tests/unit"`.
- [ ] The full ported suite passes:
      `QT_QPA_PLATFORM=offscreen uv run pytest tests/testgui`.
- [ ] A new scripted end-to-end test (may live in `tests/testgui/` or
      `tests/bench/` per house convention — match whichever existing
      integration tests in this repo already combine `SimTransport` +
      `TraceModel`) drives the reconciled stack fully: connects
      `SimTransport`, binds `DEV DT PORTS`, sends `DEV DT VW` (mirroring
      `KeyboardDriver`'s wire strings), applies a nonzero
      `encoder_noise`/`enc_scale_err_l` error profile, ticks forward, and
      asserts the `encoder` trace has measurably diverged from the `camera`
      (ground-truth) trace by an amount consistent with the injected error —
      this is the sprint's stated Success Criteria
      ("injecting a slip/encoder-error profile visibly separates the encoder
      trace from truth"), made concrete and automated.
- [ ] `sprint.md`'s `## Tickets` table is consistent with the four tickets
      actually created (cross-check, no code change required for this bullet
      itself).

## Testing

- **Existing tests to run**: N/A (this ticket creates the test tree).
- **New tests to write**: the ported `tests/testgui/*` files (see Acceptance
  Criteria) plus the new end-to-end error-divergence scripted test.
- **Verification command**: `QT_QPA_PLATFORM=offscreen uv run pytest tests/testgui` and, separately, `uv run python -c "import robot_radio.testgui"` in a `gui`-group-free environment to confirm lazy-import discipline.
