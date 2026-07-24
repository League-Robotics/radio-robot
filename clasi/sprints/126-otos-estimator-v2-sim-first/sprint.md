---
id: '126'
title: Dead-legacy cleanup
status: roadmap
branch: sprint/126-dead-legacy-cleanup
worktree: false
use-cases: []
issues:
- stale-ruckig-cmake-comment-and-dead-dev-family-docs.md
- testgui-dbg-otos-bench-verb-dead-on-serial-connect.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 126: Dead-legacy cleanup

> Roadmap-level plan (Phase 1). Architecture, use cases, and tickets are
> filled in at detail-planning time.

## Goals

Retire three pockets of dead/stale legacy that mislead readers and log noise on
every connect. All low-risk docs/host hygiene; no behavior change to any live
motion or wire path.

## Problem

1. **Stale Ruckig CMake comment.** `CMakeLists.txt:183-184` claims Ruckig is
   "restored ... load-bearing again" (sprint 109); sprint 115-002 re-deleted it
   permanently. Fix the comment and check whether any Ruckig build machinery it
   describes is also dead weight.
2. **Dead DEV-command-family docs.** `src/tests/DESIGN.md`,
   `src/tests/CLAUDE.md`, and several `src/tests/bench/*.py` scripts describe a
   `DEV` command family (docs/protocol-v2.md sec 16) that no longer exists in
   `src/firm/` (grep-verified 2026-07-23). The bench scripts that depend on DEV
   verbs are dead against current firmware. Refresh or archive them; update the
   two docs.
3. **Dead `DBG OTOS BENCH 1` push on Serial connect.** TestGUI `__main__.py`'s
   `_on_connect()` sends `DBG OTOS BENCH 1` on every Serial connect, but `DBG`
   has no binary-wire arm and falls through the (permanently-`False`) legacy
   translation guard, so every Serial connect logs
   `ERR unavailable legacy verb translation removed ...` and the bench-OTOS
   swap never happens. Remove the dead push (or replace it with something real
   — see the note below).

## Solution (candidate — confirm at detail time)

- Fix the CMake comment; delete any confirmed-dead Ruckig build machinery.
- Refresh the still-useful bench scripts to the current MOVE/protocol-v4
  surface (the phase-B bench session will show which ones matter); archive the
  rest; correct `src/tests/DESIGN.md`/`src/tests/CLAUDE.md`.
- Remove the dead `DBG OTOS BENCH 1` connect-time push and its `[BENCH]`
  log line. NOTE: this is distinct from actually PROVIDING a bench-OTOS
  substitution — sprint 120-002's build-selectable `FAKE_OTOS` already gives a
  bench build a meaningful `frame.otos`, so the runtime `DBG` verb this push
  targeted is genuinely obsolete, not merely unrouted. If a runtime toggle is
  ever wanted, that is its own future feature, not this cleanup.

## Success Criteria

- No source/doc claims Ruckig is live; no dead Ruckig build machinery remains.
- `src/tests` docs and bench scripts no longer reference the removed `DEV`
  family; kept scripts run against current firmware, archived ones are clearly
  parked.
- A Serial connect produces no `ERR ... legacy verb translation removed` log
  line; the dead `DBG OTOS BENCH` push is gone.
- No behavior change to any live motion, telemetry, or wire path.

## Scope

### In Scope

- `CMakeLists.txt` comment + dead Ruckig build machinery.
- `src/tests/DESIGN.md`, `src/tests/CLAUDE.md`, dead `src/tests/bench/*.py` DEV
  scripts (refresh/archive).
- TestGUI `__main__.py` dead `DBG OTOS BENCH 1` connect push
  (`src/host/robot_radio/testgui/`).

### Out of Scope

- Building any NEW runtime OTOS-substitution feature (120-002's `FAKE_OTOS`
  build seam already covers the bench need).
- Any change to live motion/telemetry/wire behavior.

## Dependencies / Sequencing

- **Independent** of all other 121-127 sprints. Can run any time. (Only soft
  ordering: the phase-B bench session informs which `src/tests/bench` scripts
  are worth refreshing vs. archiving.)

## Architecture

Deferred to detail planning. Expected tier: trivial/small — docs + dead-code
removal, no architectural impact, architecture review expected `skipped`.

## Use Cases

Deferred to detail planning (likely "N/A — trivial").

## Tickets

Deferred to detail planning.
