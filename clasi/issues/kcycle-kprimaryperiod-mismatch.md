---
title: "kCycle=20 vs Telemetry::kPrimaryPeriod=40 mismatch -- robot_loop.cpp's own doc comment is now false"
filed: 2026-07-19
filed_by: programmer (111-002, discovered while fixing sim_api_harness.cpp's stale timing expectations)
status: pending
related:
  - motion-control-terminal-blips-reconciled-fix-plan.md
---

# kCycle/kPrimaryPeriod mismatch

## What's going on

`src/firm/app/robot_loop.cpp`'s own doc comment (lines 17-21, in the
anonymous-namespace timing-constants block) reads:

> kCycle is the STATED TOTAL for the whole schedule (all four pacing
> blocks, not just the trailing one) -- ~25 Hz/~40ms, matching
> Devices::Telemetry's own kPrimaryPeriod=40ms (telemetry.h) so the
> primary-frame throttle and the loop's own pace agree by construction.

The actual value two lines below (`robot_loop.cpp:25`) is:

```cpp
constexpr uint32_t kCycle = 20;  // [ms] whole-schedule pace target (~25 Hz)
```

`src/firm/app/telemetry.h:94` still has:

```cpp
constexpr uint32_t kPrimaryPeriod = 40;  // [ms] ~25 Hz
```

`20 != 40` -- the comment's own claim ("matching ... so the primary-frame
throttle and the loop's own pace agree by construction") is currently
**false**. Both constants also independently mislabel themselves "~25 Hz"
in their own trailing comments (20ms is actually ~50Hz; 40ms is ~25Hz) --
a second, smaller inconsistency in the same neighborhood, worth fixing in
the same pass as the main mismatch.

## Where this was found

111-002 (baseline sim-suite triage) was fixing
`src/tests/sim/system/sim_api_harness.cpp`'s
`scenarioVirtualCycleTimingDiagnostic()`, which had hardcoded 106-001-era
expectations (`kSettle=kClear=4`, `kCycle=40`, `kPace=28`) that no longer
matched the tree's current values (`kSettle=kClear=0`, `kCycle=20`,
`kPace=20`). Retargeting that harness's own duplicated constants to match
`robot_loop.cpp`'s current values surfaced this doc-comment/code
divergence. 111-002's own ticket explicitly scoped this mismatch OUT --
"no ticket here touches `robot_loop.cpp`'s timing constants or
`telemetry.h`" -- so it is filed here instead, per that ticket's AC #5.

## Why it might matter

`kPrimaryPeriod` throttles how often `Devices::Telemetry` emits a primary
frame; `kCycle` is the loop's own per-cycle pace target. If they are
meant to agree "by construction" (per the comment), a primary frame is
now emitted only once every ~2 loop cycles, not every cycle -- possibly
intentional (nothing forces a 1:1 relationship), possibly a genuine drift
nobody has revisited since `kCycle` was retargeted from 40ms to 20ms
(likely as part of the pid-debugging cycle-order experiment era, given
the reconciled fix plan's own §5 correction: "Cycle time is 20ms, not
50ms... `robot_loop.cpp:25` now has `kCycle = 20`").

## Suggested next step

A future sprint should either (a) update `kPrimaryPeriod` to 20 to
restore the "matching by construction" invariant the comment describes,
or (b) update the comment to describe the current, deliberate 2:1
relationship if one was intended, and fix both constants' own "~25 Hz"
labels to match their actual values (20ms ~= 50Hz, 40ms ~= 25Hz). Not
resolved by 111-002 -- out of that ticket's scope by explicit
instruction.
