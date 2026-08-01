---
status: pending
---

# Sensors subsystem should own the line/color tick flip-flop

## Description

`robot_loop.cpp` currently alternates ticking the line and color sensors
inline in the loop body:

```cpp
const bool tickedLine = (cycleCount_ % 2) == 1;  // first cycle ticks line
if (tickedLine) line_.tick(nowUs); else color_.tick(nowUs);
```

This scheduling detail (the even/odd cycle flip-flop) is bus-budget policy
leaking into the robot loop. It should be moved into a Sensors subsystem
that:

- exposes a single `tick(nowUs)` entry point to the loop, and
- keeps the line/color alternation (the flip-flop state) internal to
  itself.

The robot loop then just calls `sensors_.tick(nowUs)` each cycle, with no
knowledge of which sensor runs on which cycle. This also gives the
alternation policy one owner if the schedule ever changes (e.g. adding a
third sensor or reweighting the cadence).
