---
id: '002'
title: Fix Drive's _drvCfg shadow-cache for trackwidth and OTOS lag
status: done
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: set-config-not-propagated-to-planner.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Fix Drive's _drvCfg shadow-cache for trackwidth and OTOS lag

## Description

`Drive` has a second, narrower instance of the same disease it doesn't even
inflict on itself consistently. `Drive::_drvCfg` (a `msg::DrivetrainConfig`
snapshot, refreshed only when a `"drive"`-annotated key is SET) shadows the
live `_robCfg` fallback for `trackwidthMm` and `lagOtosMs` in
`Drive::tickUpdate()`'s EKF-predict step (~`Drive.cpp:127-138`):

```
_drvCfg.get_trackwidth() > 0.0f ? _drvCfg.get_trackwidth() : _robCfg.trackwidthMm
_drvCfg.get_lag_otos() > 0      ? _drvCfg.get_lag_otos()      : _robCfg.lagOtosMs
```

The fallback's `> 0.0f`/`> 0` guard condition can only ever be true *before*
`_drvCfg` is first populated — i.e. never, after boot's initial
`configure()` call sets a positive value once. Since neither `tw` nor
`lag.otos` is `"drive"`-annotated, `_drvCfg` never refreshes again, so
`SET tw=<x>` and `SET lag.otos=<x>` alone never reach this EKF-predict step
— even though the very next line in the same function already reads
`rotationalSlip` directly from live `_robCfg` correctly.

This is the same shadow-cache disease as the Planner bug (Ticket 001), one
layer down, discovered by auditing every `_drvCfg` read site rather than
just the keys the original issue named. See `architecture-update.md` Step
4-5 item 2 and Design Rationale Decision 2 for the full analysis
(annotating `tw`/`lag.otos` `"drive"` was considered and rejected — it
would reintroduce the exact "propagates only when this specific key is
annotated" fragility Ticket 001 just eliminated for Planner, one layer
down).

## Acceptance Criteria

- [x] `source/subsystems/drive/Drive.cpp`, `tickUpdate()`: the trackwidth
      ternary (`_drvCfg.get_trackwidth() > 0.0f ? ... : _robCfg.trackwidthMm`)
      is replaced with a direct `_robCfg.trackwidthMm` read.
- [x] Same function: the OTOS-lag ternary
      (`_drvCfg.get_lag_otos() > 0 ? ... : _robCfg.lagOtosMs`) is replaced
      with a direct `_robCfg.lagOtosMs` read.
- [x] `_drvCfg` itself is NOT removed as a type or member — it remains in
      use for other fields it legitimately carries (e.g.
      `drivetrain_type`'s capability-reporting use, other
      `msg::DrivetrainConfig` accessor consumers). Only these two read
      sites stop consulting it.
- [x] No signature change to `tickUpdate()` or any other `Drive` method.
- [x] `SET tw=<x>` alone (not bundled with any `"drive"`-annotated key in
      the same `SET` line) changes the trackwidth `Drive`'s EKF-predict
      step uses on the very next tick.
- [x] `SET lag.otos=<x>` alone changes the OTOS-lag compensation `Drive`'s
      EKF-predict step uses on the very next tick.
- [x] Full default sim/unit test suite green.

## Testing

- **Existing tests to run**: any existing `Drive`/EKF-predict sim tests
  that exercise trackwidth or OTOS-lag behavior; full default suite via
  `uv run python -m pytest`.
- **New tests to write**: a focused sim test (or an addition to Ticket
  004's sweep, whichever lands first — coordinate to avoid duplication)
  that SETs `tw` alone and confirms the EKF-predict trackwidth used
  changes, and similarly for `lag.otos`. If Ticket 004 is expected to
  cover this, a minimal smoke assertion here is still useful for isolated
  verification during implementation.
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: `Drive.cpp` already reads `rotationalSlip` directly from live
`_robCfg` one line below the two broken ternaries — make `trackwidthMm` and
`lagOtosMs` consistent with their own neighbor instead of introducing a
second mechanism. This is a two-line, single-function change with no
cross-file impact.

**Files to modify**:
- `source/subsystems/drive/Drive.cpp` — `tickUpdate()`, the two ternary
  read sites (~lines 127-138).

**Testing plan**:
- Isolated measurement: `SET tw=<x>` (no other `"drive"`-annotated key in
  the same command), then drive a motion command that exercises the
  EKF-predict step, and confirm the trackwidth used reflects `<x>` rather
  than the boot default or the last `"drive"`-annotated-SET value.
- Same pattern for `SET lag.otos=<x>`.
- Run the full default suite (`uv run python -m pytest`) and confirm no
  regressions — in particular, confirm no existing test relies on
  `_drvCfg`'s stale trackwidth/lag-otos values remaining frozen (search
  `tests/simulation/` for `get_trackwidth`/`get_lag_otos`-adjacent
  assertions before changing the read sites).

**Documentation updates**: none — `architecture-update.md` already
documents this change in full (Step 4-5 item 2, Design Rationale
Decision 2). No wire-protocol change, no `RobotConfig`/`DrivetrainConfig`
schema change.
