---
status: pending
---

# Line sensor is 100% dead: decide increment-or-delete, and give perception pacing an owner

**Merged 2026-08-02** with `sensors-subsystem-owns-line-color-tick-flipflop.md`
(closed): they are one decision, not two. Re-verified in the tree the same day
(2026-08-02 review, Part 1) — `cycleCount_` declared `robot_loop.h:201`, tested
`robot_loop.cpp:544`, incremented nowhere, since `8a691651` (07-26). The
preamble still spends boot I2C probing a sensor that will never be read.

## The merged half — perception pacing has no owner

`robot_loop.cpp` alternates line/color inline in the loop body:

```cpp
const bool tickedLine = (cycleCount_ % 2) == 1;  // first cycle ticks line
if (tickedLine) line_.tick(nowUs); else color_.tick(nowUs);
```

The flip-flop, `packLine()`/`packColor()`, and publication all live inline in
`RobotLoop` — bus-budget policy leaking into the loop. A `Sensors` subsystem
should own the cursor and the pacing budget, exposing a single `tick(nowUs)`;
the loop then has no knowledge of which sensor runs on which cycle, and the
alternation has one owner if the schedule ever changes.

**Sequence the two halves together**: whether the line sensor exists at all
(increment-or-delete) determines what the Sensors subsystem is pacing. Deleting
the sensor makes the flip-flop moot; keeping it makes the subsystem the right
home for the added I2C transaction's budget.



**Source:** code review 2026-07-30, `doc-rot-and-minor-sweep-from-2026-07-30-review.md`
§"Small decisions to record" (01 MINOR §5) — deferred out of 128-009 (a
doc-rot/minor-sweep ticket) because this is a firmware BEHAVIOR fix, not a
documentation fix.
**Priority:** P2 — the line sensor has apparently been fully inert since the
parity counter was introduced; nothing downstream currently depends on it,
but any future consumer of `kFlagLinePresent`/line-sensor telemetry will
silently get nothing.

## What is wrong

`src/firm/app/robot_loop.h:171-181`, `cycleCount_`:

```cpp
// Parity picks line vs color in the pace block.
//
// KNOWN DEFECT, deliberately left alone by the command-ingestion rework:
// nothing increments this. It has been stuck at 0 since the counter was
// introduced, so `(cycleCount_ % 2) == 1` is permanently false and the
// LINE sensor is never ticked -- only the color sensor is. The one-line
// fix is real but it adds an I2C transaction to every other pace block,
// which shifts the loop period the motion tuning is calibrated against;
// that belongs to its own change with its own bench measurement, not to
// a command-plane rework whose gate is a square tour.
uint32_t cycleCount_ = 0;
```

The pace block alternates between polling the line sensor and the color
sensor by checking `cycleCount_ % 2`, but nothing ever increments
`cycleCount_` — it is permanently `0`, so the `== 1` branch (line sensor)
never fires. The color sensor is polled every pace-block cycle; the line
sensor is polled never. This is self-documented as a known defect in the
comment above, not a fresh discovery — 128-009's craftsmanship-review sweep
found it while re-auditing doc rot and is filing it as its own issue per
that ticket's explicit instruction not to fix or silently drop it inline.

## Why this is not a trivial one-line fix

Per the comment's own reasoning: incrementing `cycleCount_` (or otherwise
fixing the parity check) makes the line sensor start actually being polled
on alternating cycles, which adds an I2C transaction to every OTHER pace
block that did not have one before. That changes the loop's cycle timing
(`cycleBusy`/`cyclePeriod`), and the motion tuning (PID, velocity shaping)
is calibrated against the CURRENT loop period. This needs its own bench
measurement of the timing delta before/after, not a drive-by increment.

## What to do

1. Fix the parity tick (increment `cycleCount_` once per pace-block cycle,
   or replace the parity scheme with something that doesn't need a counter
   at all — e.g. alternate off `state_.time.cycleStart` parity, if that
   avoids adding new state).
2. Bench-measure `cycleBusy`/`cyclePeriod` before and after on real
   hardware — confirm the added I2C transaction does not regress the loop
   period enough to matter, or if it does, re-tune/re-document the motion
   PID against the new period.
3. Confirm on the stand that the line sensor (4 channels) actually reports
   plausible, changing values once ticked, per
   `.claude/rules/hardware-bench-testing.md`'s standing verification gate.
4. Update `kFlagLinePresent`'s telemetry.h comment/DESIGN.md docs once the
   sensor is live, since today its presence bit can never actually go true
   for a real reason.

## Acceptance

- `cycleCount_` (or its replacement) actually alternates, and the line
  sensor is polled every other pace-block cycle, confirmed by a firmware
  test (sim harness) that the line-sensor read path fires at the expected
  cadence.
- Bench: `enc`/`otos`/line sensor all report plausible values on the stand;
  loop timing measured and, if it shifted, either re-tuned or explicitly
  accepted with a stated margin.
- No regression in the square-tour system-test gate
  (`clasi/issues/square-tour-is-the-one-system-test-sim-bench-playfield.md`).
