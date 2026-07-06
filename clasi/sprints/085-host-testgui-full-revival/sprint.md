---
id: 085
title: Host TestGUI full revival
status: roadmap
branch: sprint/085-host-testgui-full-revival
use-cases: []
issues:
- host-testgui-full-revival.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 085: Host TestGUI full revival

## Goals

Wire TestGUI's remaining features — tours, camera GOTO, the Operations panel
(Sync-Pose/Zero-Encoders/Set-Origin/STREAM toggle), connect-time calibration
push, and live camera view — onto sprint 084's new motion + config verbs,
and port the remaining GUI test suite. This completes the TestGUI-revival
program epic
(`clasi/issues/plan-revive-testgui-against-the-new-tree-simulator.md`).

**Delivers:** the fully restored TestGUI cockpit — tours, camera GOTO,
calibration, live view — completing the TestGUI-revival program.

**Dependency:** depends on sprint 083 (sim cockpit) + sprint 084 (firmware
motion/config verbs). Cannot start until both are done.

## Problem

Sprint 083 delivers a drive-and-observe cockpit; sprint 084 delivers the
firmware verbs. Neither alone restores TestGUI's full pre-greenfield feature
set — tours, camera-guided GOTO, pose anchoring, and calibration push all
still need to be wired onto the new surface.

## Solution

Point TestGUI's command rows, tour runner, GOTO runner, Operations panel,
and calibration-push helper at 084's new top-level verbs; restore the live
camera view; port the remaining `tests_old/testgui/` suite to `tests/testgui/`.

## Success Criteria

Full cockpit against the sim (and, where applicable, the bench/relay): a
tour runs to completion and returns near origin; camera GOTO drives to a
world point; Sync-Pose/Set-Origin anchor pose; calibration pushes on
connect; the full GUI test suite is green.

## Scope

### In Scope

- Command rows (`testgui/commands.py`): point `S/T/D/R/TURN/RT/G` at the new
  top-level motion verbs; keep centidegree/wrap conventions.
- Tours (`_TourRunner`): run `D`/`RT` sequences with `SNAP`-polled `mode=I`
  completion detection.
- Camera GOTO (`_GotoRunner`): host-side pure-pursuit loop — camera truth →
  `SI` → `G` — restored on the new verbs.
- Operations panel (`testgui/operations.py`): Sync-Pose (`SI`),
  Zero-Encoders (`ZERO enc`), Set-Origin (`STOP`+plant-teleport+`ZERO`+
  `OZ`+`SI`), STREAM toggle.
- Connect-time calibration push (`calibration.push.calibration_commands`)
  via the new `SET`/OTOS verbs.
- Live camera view (Relay/PLAYFIELD mode) and the remaining GUI test port
  (`tests_old/testgui/` → `tests/testgui/`), added to `pyproject.toml`
  `testpaths`.

### Out of Scope

Nothing further deferred within the program — this is the final sprint.
(Program-level follow-ons, e.g. the real-hardware OTOS driver, remain
separate deferred issues outside this program's scope.)

## Test Strategy

Full GUI test suite (headless + where applicable sim/bench-driven),
including the ported `tests_old/testgui/` tests. Detail-planning phase sizes
out specific test files.

## Architecture Notes

No architecture changes finalized yet — this roadmap entry precedes
detail-planning. The architecture-update.md (written at detail-planning
time) will cover how the GUI's runners/panels bind to 084's verb surface.

## GitHub Issues

(GitHub issues linked to this sprint's tickets. Format: `owner/repo#N`.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [ ] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [ ] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|

Tickets execute serially in the order listed.
