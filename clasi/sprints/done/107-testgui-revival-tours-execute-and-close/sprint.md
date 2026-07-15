---
id: '107'
title: 'TestGUI revival: tours execute and close'
status: done
branch: sprint/107-testgui-revival-tours-execute-and-close
use-cases: []
issues:
- executor-fault-check-needs-baseline-exclusion.md
- heading-loop-default-gains-overshoot-on-bench-rig.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 107: TestGUI revival: tours execute and close

## Goals

ROADMAP-STAGE ENTRY (not yet detailed — no architecture-update.md content,
no tickets). This is the LAST sprint in the arc that started with sprint
103's single-loop rebuild — it delivers the stakeholder's stated end goal
directly: **"demonstrate that the tours that we have in our test GUI
actually execute... I want those tours to be closed. I want to see charts
in Jupyter Notebooks that show nice acceleration and deceleration on
straights and turns."**

**What "tours" are** (confirmed by direct investigation of the current
tree, 2026-07-14): `Tour 1` and `Tour 2` are defined in
`host/robot_radio/testgui/commands.py` (`TOUR_1`, `TOUR_2`, collected in
`TOURS: dict[str, list[str]]`) as ordered lists of legacy TEXT-plane
verbs — `D <left> <right> <mm>` (distance drive) and `RT <rel_cdeg>`
(relative in-place turn) — 7 legs each, closing back toward the start
(Tour 1: symmetric out-and-back distances with four 90°-equivalent turns;
Tour 2: a longer irregular polygon with mixed turn angles, also closing).
`host/robot_radio/testgui/__main__.py`'s `_TourRunner` (a `QObject`
worker) sends each step in sequence over whatever transport is connected,
polling `SNAP`'s `mode=I` (idle) to detect step completion before
dispatching the next leg, and narrates progress via `[TOUR]`-prefixed log
lines. `tests/testgui/test_tour1_geometry.py`/`test_tour_stop.py` are the
existing (085-era) test coverage — real against ctypes-sim firmware, not
current hardware.

**Both `D` and `RT` are retired verbs** — sprint 102/103's single-loop
rebuild deleted the entire pre-102 text-plane command surface (segment/
drive/turn) along with the on-robot trajectory planner that executed
them. Under the current (post-103/104) architecture there is no wire verb
that means "drive this far" or "turn this many degrees" — the robot only
accepts a `twist` (velocity + duration) and reports raw telemetry. A tour
CANNOT run against current firmware without a substitute for `D`/`RT`,
and that substitute is exactly sprint 106's host trajectory planner
(profiled twist sequences with heading feedback). This sprint is
therefore sequenced strictly after 106, not just after 104/105.

## Problem

The TestGUI's tour buttons are dead against current firmware — they speak
a wire surface that no longer exists, and even if they did, the on-robot
trajectory execution and completion-detection (`SNAP mode=I`) they relied
on has been deleted along with the rest of the pre-102 architecture. The
stakeholder's stated acceptance for the ENTIRE single-loop rebuild arc is
that these specific tours run again, close (return to origin within
tolerance), and produce the accel/decel chart evidence sprint 106 makes
possible. No sprint before this one delivers that visible, GUI-driven
proof.

## Solution

Design deferred to this sprint's own detail-mode planning pass. Steer:

1. **TestGUI transport realignment** onto the new wire surface — sim via
   sprint 105's `sim_api` (so tours can be exercised headless/in CI), real
   hardware via sprint 104's fully-realigned host tooling. The GUI's
   existing transport abstraction (`SimTransport` vs. real
   `SerialConnection`, per `tests/testgui/test_tour1_geometry.py`'s own
   framing) is the seam to update, not necessarily rebuild.
2. **Tour definitions re-expressed as host-planner calls.** `TOUR_1`/
   `TOUR_2`'s existing leg lists (distances + turn angles) are a
   REUSABLE geometric spec — the same 7-leg sequences, just re-driven
   through sprint 106's profiled-twist planner instead of raw `D`/`RT`
   text commands. Completion detection moves from `SNAP mode=I` polling
   to whatever telemetry-based "leg complete" signal 106's planner
   exposes.
3. **Tour closure measurement** — pose returns to origin within a stated
   tolerance. On the bench rig: encoder/telemetry-based closure (the
   planner's own reported pose at tour end vs. start). Optionally,
   playfield + camera-verified closure if this sprint runs on the
   playfield rather than the bench rig — per
   `.claude/rules/`-adjacent project convention, NEVER blind-drive the
   playfield (vision + geofence + hop first, per the project's standing
   safety rule) if that path is chosen; the bench-rig closure check does
   not carry this constraint (wheels off the ground, per
   `.claude/rules/hardware-bench-testing.md`).
4. **The deliverable notebook** — a Jupyter notebook (in `tests/notebooks/`,
   alongside the existing `motion_control.ipynb`/`wheel_motion_trace.ipynb`
   precedent) charting: commanded-vs-measured velocity, acceleration/
   deceleration envelopes on a straight leg and a turn leg (sourced from
   106's clean, resonance-tamed telemetry), and tour path closure (a
   plotted path showing start/end proximity). This notebook IS the
   stated end-goal deliverable — the sprint is not complete without it.

## Success Criteria

- Both `Tour 1` and `Tour 2` (or their host-planner-driven equivalents,
  if detail-mode planning finds the exact 7-leg geometry needs adjustment
  for the new planner's own admission/preemption rules — a ticket-time
  call) execute end-to-end from the TestGUI, against real hardware on the
  bench rig, with no step timing out.
- Tour closure is measured and reported: final pose vs. start pose, within
  a stated tolerance, using telemetry/encoder evidence (not assumed from
  "it looked right").
- A Jupyter notebook exists, runs, and displays: commanded-vs-measured
  velocity/acceleration on a straight, commanded-vs-measured on a turn,
  and a tour closure plot — this is the literal artifact the stakeholder
  asked to see.
- `tests/testgui/test_tour1_geometry.py`/`test_tour_stop.py` (or their
  rewritten equivalents) pass against the CURRENT architecture (sim via
  105, not the pre-rebuild ctypes sim they currently target).

## Scope

### In Scope

- TestGUI transport realignment (sim + real) onto the new wire surface.
- Tour definitions re-driven through sprint 106's host planner.
- Tour closure measurement (encoder/telemetry; camera-verified optional,
  playfield-safety-gated if used).
- The deliverable Jupyter notebook (accel/decel + closure charts).
- Rewriting `test_tour1_geometry.py`/`test_tour_stop.py` against the
  current architecture.

### Out of Scope

- New tour geometries beyond Tour 1/Tour 2 (unless 106's planner requires
  a geometry adjustment to close correctly, per Success Criteria above —
  a fix to the existing tours, not a new feature).
- Any further host-side sensor fusion beyond what 106 already built.

## Test Strategy

- **Unit** (`tests/unit/`, no hardware/sim): `planner/executor.py`'s
  baseline-relative fault handling (001), `planner/tour.py`'s geometry
  parser + closure computation against a `FakeTransport` double (002).
- **Headless GUI** (`tests/testgui/`, re-added to `pyproject.toml`'s
  `testpaths` by ticket 004): tour-button control flow, Stop Tour
  reactivation, against a `FakeTransport`-backed harness — no dependency
  on the deleted `tests/_infra/sim` ctypes library.
- **Bench** (`.claude/rules/hardware-bench-testing.md`, real hardware on
  the stand): both tours run end-to-end via the TestGUI (003) and via a
  dedicated bench script capturing per-leg traces and tour closure (005),
  2-3 repeat runs per tour to characterize run-to-run variance before a
  closure tolerance is finalized.
- **Notebook** (006): executes end-to-end, reviewed against the `dataviz`
  skill's own guidance for chart quality.
- Sim-mode tour coverage is explicitly OUT of scope this sprint (see
  Architecture Notes / architecture-update.md Decision 1) — a follow-up
  issue is filed, not built here.

## Architecture Notes

Depends on sprint 104 (host tooling), sprint 105 (sim, for headless tour
CI coverage), and sprint 106 (the trajectory planner tours actually drive
through) all being complete. This is the capstone sprint of the arc that
began with sprint 102 — closing it closes the stakeholder's stated end
goal for the current planning horizon. Full architecture-update.md is
written when this sprint is detailed.

## GitHub Issues

(GitHub issues linked to this sprint's tickets. Format: `owner/repo#N`.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [x] Stakeholder has approved the sprint plan (auto-approved under the
      stakeholder's standing 2026-07-14 power-through directive)

## Tickets

| # | Title | Use Cases | Depends On |
|---|-------|-----------|------------|
| 001 | Planner production hardening: fault-check baseline exclusion + heading-gain retune | SUC-031, SUC-032 | — |
| 002 | Tour driver: planner/tour.py owns tour geometry, chains legs through the executor, closure bookkeeping | SUC-033 | 001 |
| 003 | TestGUI rewire: transport accessor + _TourRunner on the live twist surface, real-hardware-only scope | SUC-034 | 002 |
| 004 | Tour test suite rewrite: FakeTransport-backed, re-added to testpaths | SUC-035 | 002, 003 |
| 005 | Bench tour runs + trace capture | SUC-036 | 003 |
| 006 | Deliverable notebook: accel/decel and tour-closure charts | SUC-037 | 005 |

Tickets execute serially in the order listed.

## Follow-up Issues Filed

Out of this sprint's own scope, filed for a future sprint:

- `clasi/issues/sim-api-ctypes-abi-for-sim-mode-tours.md` — a new ctypes C
  ABI over `tests/sim/support/sim_api.h`'s `SimApi` is needed before
  TestGUI tours (or `SimTransport` generally) can run in Sim mode; the
  backing library this sprint would have used was deleted wholesale at
  sprint 102 ticket 005.
- `clasi/issues/binary-bridge-segment-replace-arms-deleted.md` —
  `testgui/binary_bridge.py`'s `R`/`TURN`/`G` translation (and the manual
  `D`/`RT` GUI command rows, as opposed to the tour buttons this sprint
  fixed) still targets `segment`/`replace` envelope arms that no longer
  exist in `protos/envelope.proto` at all.
