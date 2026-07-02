---
id: '006'
title: Restore outlier-filter reject-streak rebaseline and idle refresh in Drive
status: open
use-cases: [SUC-007]
depends-on: []
github-issue: ''
issue: encoder-integrity-i2c-failures-and-outlier-filter-recovery.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Restore outlier-filter reject-streak rebaseline and idle refresh in Drive

## Description

CR-02 (`clasi/issues/encoder-integrity-i2c-failures-and-outlier-filter-recovery.md`):
`Drive::_runOutlierFilter()` (`source/subsystems/drive/Drive.cpp:394-444`)
increments `_filterRejectStreakL/R` on each rejected delta and resets it on
an accepted one, but never consumes it — `kFilterRejectStreakThreshold = 3`
(`Drive.h:148`) is declared and unused. A large, persistent divergence (e.g.
the wheel was hand-rolled while idle) is therefore rejected forever: every
fresh read differs from the same stale baseline by the same large delta, so
the filter holds `_hw.encMm[]` frozen indefinitely, and `Odometry::
predict()` sees zero deltas while commanded motion continues — the
"freakout" failure scenario in the filed issue.

The filter's whole block is also gated `if (driving)`; while idle,
`_hw.encMm[]` is never refreshed at all, so this failure mode is guaranteed
to occur on the very first tick of the next command if the robot was
touched while parked.

## Acceptance Criteria

- [ ] Reject-streak rebaseline: when a rejection's retries are exhausted,
      increment `_filterRejectStreakW` as today; once it reaches
      `kFilterRejectStreakThreshold` (3), accept the already-computed fresh
      reading (`_motorR.positionMm()`/`_motorL.positionMm()` — the value
      already read this tick, no extra I2C) as the new `_hw.encMm[]` value
      and reset the streak to 0. Apply to both the L and R blocks.
- [ ] Idle refresh: the `else` branch (not driving) additionally copies
      `_hw.encMm[0] = _motorR.positionMm(); _hw.encMm[1] =
      _motorL.positionMm();` every tick, unconditionally (no outlier gate —
      see architecture-update.md Design Rationale 5 for why no gate is
      needed at rest).
- [ ] The `if (driving)` block's existing retry-then-hold behavior for a
      *transient* outlier (one bad read that recovers within `kRetries`
      attempts) is unchanged — only the *persistent* (3+ consecutive
      rejection) case gets the new rebaseline.
- [ ] `uv run --with pytest python -m pytest -q` is green (2 known-baseline
      failures allowed, no new failures).

## Testing

- **Existing tests to run**: full default suite; `test_drive_subsystem.py`,
  `test_estimator_isolation.py`, `test_odom_tracker.py` in particular.
- **New tests to write** (the issue's own stated acceptance criteria):
  - Sim test: jump the plant encoders while idle
    (`sim_set_true_wheel_travel`/`sim_set_enc_l/r`, hand-reposition
    analogue), then command a `TURN` — odometry must track the turn (the
    idle refresh means the filter's baseline already absorbed the jump
    before the command started, so no rejected-forever freeze occurs).
  - Sim test: during an active command, inject 3+ consecutive large deltas
    (e.g. via `sim_set_motor_offset` or a sudden `sim_set_enc_l/r` jump
    beyond `kMaxDeltaMm`) and assert the filter rebaselines (accepts the
    fresh reading) after the third consecutive rejection instead of
    freezing `_hw.encMm[]` forever.
  - Regression guard: a single transient bad read (1-2 consecutive
    rejections that then recover within `kRetries`) must NOT trigger a
    rebaseline — assert `_filterRejectStreakL/R`-driven behavior only
    engages at exactly the threshold, not before.
- **Verification command**: `uv run --with pytest python -m pytest -q`
