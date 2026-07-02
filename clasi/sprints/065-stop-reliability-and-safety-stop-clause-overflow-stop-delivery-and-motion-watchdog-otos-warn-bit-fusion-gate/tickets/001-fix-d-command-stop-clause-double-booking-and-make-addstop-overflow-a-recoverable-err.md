---
id: '001'
title: Fix D-command stop-clause double-booking and make addStop overflow a recoverable
  ERR
status: open
use-cases: [SUC-001]
depends-on: []
github-issue: ''
issue: stop-clause-overflow-aborts-process.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Fix D-command stop-clause double-booking and make addStop overflow a recoverable ERR

## Description

CR-01 (critical). `Superstructure::requestGoal`'s `Goal::DISTANCE` case
re-adds every entry of `gr.stops[]` onto the `MotionCommand` that
`distanceDrive()` → `Planner::beginDistance()` already populated with its own
`DISTANCE` + `TIME` stops. `MotionCommands.cpp`'s `handleD` (line ~759)
additionally pre-populates `gr.stops[0]` with a *duplicate* `makeDistanceStop`
before that re-add loop runs. Net effect: a plain `D` installs 3 stops (1
wasted duplicate); `D ... stop=... sensor=...` (2 wire clauses) installs 5
against `MotionCommand::kMaxStopConds == 4`, and `MotionCommand::addStop()`'s
`assert(false && "addStop overflow")` fires. The sim build sets no `NDEBUG`,
so this **aborts the whole Python process** hosting the sim (pytest run or
the TestGUI's `SimTransport` tick-thread); real firmware panics via the CODAL
assert path mid-drive.

`Goal::VELOCITY` (`T`/`S`/`VW`/`R`, via `beginVelocity()`) has **no**
analogous bug — `beginVelocity()` deliberately installs zero stops
internally, so `requestGoal`'s re-add loop is the only place stops are
installed for those verbs. The double-booking is specific to
`Goal::DISTANCE`.

See `architecture-update.md` Step 4-5 item 1 and Design Rationale Decision 1
& 2 for the full analysis and the two-layer fix rationale.

## Acceptance Criteria

- [ ] `handleD` (`source/commands/MotionCommands.cpp`) no longer
      pre-populates `gr.stops[0]` with `makeDistanceStop(mm)`; `gr.stops[]`
      carries only wire-supplied `stop=`/`sensor=` clauses, starting at
      index 0.
- [ ] A plain `D <l> <r> <mm>` (no extra clauses) installs exactly 2 stop
      conditions (`DISTANCE` + the internal `TIME` safety net) — no
      duplicate.
- [ ] `MotionCommand::addStop()` (`source/commands/MotionCommand.cpp`) no
      longer calls `assert(false ...)` on overflow; it returns `false` and
      leaves `_nStops` unchanged (matches its existing documented contract).
- [ ] `Superstructure::requestGoal`'s `Goal::DISTANCE` and `Goal::VELOCITY`
      cases (`source/superstructure/Superstructure.cpp`) check each
      `addStop()` return value in their `gr.stops[]` re-add loop; on the
      first `false`, the just-started command is cancelled
      (`activeCmd().cancel(MotionCommand::StopStyle::HARD)`) and the host
      receives a wire-visible `ERR stopoverflow` (via `gr.corrId`/
      `gr.replyFn`/`gr.replyCtx`) instead of continuing with incomplete stop
      coverage.
- [ ] Regression test: `D 150 150 300 stop=time:9000 sensor=line0>500` runs
      to completion in sim without process abort and stops on whichever
      clause fires first (exactly 4 stops installed: internal DISTANCE+TIME,
      wire TIME(9000)+SENSOR(line0>500) — no overflow).
- [ ] New test: a `D`/`T` with enough wire clauses to still overflow (e.g. 3+
      `stop=` clauses on `D`) is cancelled cleanly and replies `ERR
      stopoverflow` — never crashes, never asserts.
- [ ] Full default sim suite green (`uv run --with pytest python -m pytest
      -q`), including existing stop-clause/`MotionCommand` tests
      (`tests/simulation/unit/test_motion_command.py`,
      `tests/simulation/unit/test_n4_n5_cancel_on_begin_stream_timed_distance.py`).

## Implementation Plan

**Approach**: Two independent, additive changes applied together (per
architecture-update.md Design Rationale Decision 1 & 2): (a) stop the
duplicate at its source in `handleD`; (b) make overflow recoverable at
`MotionCommand::addStop()` + `Superstructure::requestGoal`, as defense in
depth for any remaining clause combination.

**Files to modify**:
- `source/commands/MotionCommand.cpp` — remove the `assert(false ...)` line
  in `addStop()`; keep the `return false;` overflow path.
- `source/commands/MotionCommands.cpp` — `handleD`: delete the
  `gr.stops[gr.nStops++] = makeDistanceStop((float)mm);` line; the
  wire-clause-packing loop already starts at `gr.nStops` (now 0) so no other
  change is needed there.
- `source/superstructure/Superstructure.cpp` — `requestGoal`'s `DISTANCE`
  and `VELOCITY` cases: wrap the `for (uint8_t i = 0; i < gr.nStops; ++i)
  _planner.activeCmd().addStop(gr.stops[i]);` loop to check the return
  value; on `false`, call `_planner.activeCmd().cancel(HARD)`, reply `ERR
  stopoverflow` via `CommandProcessor::replyErr` (header already included),
  and `break`.

**Testing plan**:
- Extend `tests/simulation/unit/test_motion_command.py` (or a sibling file)
  with: (1) plain `D` stop-count assertion (exactly 2, no duplicate); (2) the
  sprint's exact regression command
  (`D 150 150 300 stop=time:9000 sensor=line0>500`) — must not crash, must
  honor the earliest clause; (3) an overflow-forcing case (3+ wire clauses)
  — must reply `ERR stopoverflow` and leave the robot stopped, not crash.
- Run the full default sim suite to confirm no regression in existing
  `D`/`T`/stop-clause coverage.

**Documentation updates**: None beyond this ticket and the sprint's
architecture-update.md (already written). No wire-protocol doc changes
beyond the additive `ERR stopoverflow` code, which is self-describing per
the existing `ERR <code> <detail> [#id]` taxonomy.
