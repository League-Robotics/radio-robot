---
id: 009
title: Doc-rot and minor sweep from the 2026-07-30 craftsmanship review
status: done
use-cases:
- SUC-009
depends-on:
- '006'
- '007'
- 008
github-issue: ''
issue: doc-rot-and-minor-sweep-from-2026-07-30-review.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Doc-rot and minor sweep from the 2026-07-30 craftsmanship review

**Worktree note**: implement in the git worktree checked out for
`sprint/128-complexity-reduction-delete-dead-code-unburden-the-robot-loop-fix-bit-rot`.
All paths below are relative to that worktree.

**Sequencing note**: depends on tickets 006-008 landing first so this
ticket's `DESIGN.md` row updates (`sensors/`, `planner/`, `path/`)
describe the POST-deletion state in one pass, rather than needing a
second pass once those deletions land.

**Scope note**: one mechanical-sweep ticket, per the project's
established mechanical-sprint batch pattern — each item below is
independent and small; do them all in one ticket rather than one ticket
per checkbox.

## Description

The MINOR/NOTE findings from the 2026-07-30 craftsmanship review not
absorbed by a larger issue in this sprint. See the source issue's own
checklist for the full itemized list (stale DESIGN.md rows, a wrong
docstring number, unit-suffixed parameters surviving the rename sprints,
a fail-open config default, duplicated helper logic, a cross-package
private import, landmine annotations, and a few one-line stakeholder
decisions).

## Acceptance Criteria

- [x] `src/host/robot_radio/DESIGN.md`'s `io/` row re-audited: it
      currently says `repl`/`stop` are live with no caveat and that
      `turnto`/`goto` route "through the dead half of nezha.py" (they
      actually route through `nav/camera_goto.py`, which has no row at
      all — add one, noting ticket 002's loud-gate this sprint). DONE:
      `io/` row corrected; the pre-existing `nav/` row updated to record
      128-002's `NotImplementedError` gate.
- [x] `io/sim_loop.py:1073-1078`'s `step()` docstring ("50ms sim-time
      each") fixed to 40 ms (matching `_CYCLE_DURATION_S = 0.040`). DONE.
- [x] `src/firm/app/robot_loop.cpp:502-503`'s trailing whitespace removed.
      DONE.
- [x] Unit-suffixed parameters fixed per `.claude/rules/coding-standards.md`:
      `robot.py:20,24`, `nezha.py:231,244`, `cutebot.py:106,111` (`ms`/`mm`
      params renamed to the quantity, unit in a `# [ms]`/`# [mm]` tag);
      `nezha_kinematic.py:66-126` (`_trackwidth_m`, `_encoder_offset_m`,
      `left_total_m`, `left_delta_m`, etc.) — **skip this file if it was
      already deleted by a Nezha-facade rebuild** (out of this sprint's
      scope; check before editing, don't resurrect a file this sprint
      didn't touch). DONE: file still exists (no Nezha-facade rebuild
      touched it this sprint); all renamed to `duration`/`distance`
      (params) and `_trackwidth`/`_encoder_offset`/`left_total`/
      `right_total`/`left_delta`/`right_delta` (nezha_kinematic.py),
      unit tags added, no call-site breakage (all positional).
- [x] `config/robot_config.py:135`: `laser_port: Optional[int] = 4`
      changed to `None` (togov already opts out explicitly; this was the
      one non-`None` default in the model). DONE.
- [x] `field/geofence.py`: `checkPlayfieldLights()` and
      `_playfieldLightsOn()`'s duplicated Shelly status fetch shared via
      one helper. DONE: shared `_fetchPlayfieldLightsStatus()` helper,
      original per-caller warning messages preserved exactly.
- [x] `testgui/canvas.py:214`'s cross-package import of
      `media.movie._deskew_frame` fixed by promoting `_deskew_frame` to
      public. DONE: renamed to `deskew_frame` in both files.
- [x] Landmine annotations added: `kinematics/differential_drive.py`
      gets a WARNING block (CW-positive vs. the project's CCW
      convention) at its top — **or is deleted instead**, if its one
      caller (`nezha_kinematic.py`) was itself removed by a Nezha-facade
      rebuild (check before choosing); `odometry.h`/`estimation.h` get a
      one-line float32 unwrapped-heading precision-bound note;
      `robot/clock_sync.py`'s `record_ping()` gets a docstring note that
      it never updates `_last_sync_s` and samples span reboots. DONE:
      `differential_drive.py` kept (caller not removed) + WARNING block
      added; `odometry.h`/`state_estimator.h` (post-122 names, under
      `src/motion/`) got the precision-bound note on `theta_`/
      `BodyPeer.heading`; `clock_sync.py`'s `record_ping()` docstring
      updated.
- [x] Small decisions recorded (one line each, in this ticket or the
      touched file): `testgui/turn_control.py`'s zero-caller TCP control
      socket — either add a small bench script that exercises it, or
      remove it (programmer's call, record the reason); `telemetry_panel.py`
      gets a "no frames in N s" staleness banner. DONE: `turn_control.py`
      KEPT (reason recorded in its own module docstring — deliberate
      external automation entry point, and its underlying wire primitive
      is already exercised by `test_gui_button_acceptance.py`);
      `telemetry_panel.py` got a staleness banner (`_STALE_AFTER_S = 3.0`),
      tested in `test_telemetry_panel.py`.
- [x] **The line-sensor dead-parity-tick item is explicitly deferred, NOT
      fixed here**: `cycleCount_` never incrementing (`robot_loop.h:171-181`)
      is a firmware behavior fix, not a doc-rot item, despite living in
      the source issue's checklist. File it as a fresh
      `clasi/issues/` entry (new issue, not a TODO buried in this
      ticket) rather than fixing or silently dropping it. DONE: filed as
      `clasi/issues/line-sensor-dead-parity-tick-cycle-count-never-increments.md`.
- [x] Each checkbox in `doc-rot-and-minor-sweep-from-2026-07-30-review.md`
      itself is marked done or struck with a stated reason. DONE — all
      items marked done with per-item notes; issue moved to
      `issues/done/` via `move_issue_to_done`.

## Testing

- **Existing tests to run**: `uv run python -m pytest` (full suite —
  several of these are renames touching call sites; confirm nothing
  regresses).
- **New tests to write**: none required; this is a documentation and
  small-fix sweep, not new behavior.
- **Verification command**: `uv run python -m pytest -q`, plus a
  targeted `grep` per item per the source issue's own acceptance section.

## Implementation Notes

- **Approach**: work through the source issue's checklist top to bottom;
  strike (with a one-line reason) any item that no longer applies
  because a prior ticket in this sprint already touched that file
  (e.g. if ticket 007 already updated the `planner/` DESIGN.md row,
  don't re-edit it here — note that it was covered elsewhere).
- **Files to modify**: see each checklist item above; primarily
  `src/host/robot_radio/DESIGN.md`, `io/sim_loop.py`,
  `src/firm/app/robot_loop.cpp`, `robot.py`, `nezha.py`, `cutebot.py`,
  `nezha_kinematic.py` (conditionally), `config/robot_config.py`,
  `field/geofence.py`, `testgui/canvas.py`, `media/movie.py`,
  `kinematics/differential_drive.py` (conditionally),
  `src/firm/motion/odometry.h`, `src/firm/app/estimation.h` (or their
  current post-122 locations — confirm against `src/motion/` first),
  `robot/clock_sync.py`, `testgui/turn_control.py`, `telemetry_panel.py`.
- **Documentation updates**: this ticket IS primarily documentation
  updates; also produces one new `clasi/issues/` file for the deferred
  line-sensor item.
