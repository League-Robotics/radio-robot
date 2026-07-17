---
status: done
sprint: '110'
tickets:
- 110-003
---

# TestGUI: speed-up factor is herky-jerky / broken at higher multipliers

## Description

The speed-up factor selector at the very top of the TestGUI does not scale
cleanly:

- **2×** — works OK.
- **5×** — seems to work OK (roughly).
- Other multipliers between — herky-jerky; it doesn't actually run any
  faster, just stutters.
- **20×** — broken; doesn't work at all.

Expected: increasing the speed-up factor should smoothly increase the
simulation/replay rate proportionally, without stuttering, up through the
highest available multiplier.

## Notes / where to look

- Likely the sim/replay loop steps at a fixed wall-clock tick and multiplies
  per-tick advance, so at high factors it either hits a step/rate ceiling or
  the timer granularity makes it stutter instead of taking bigger/more steps.
- Check the sim loop timing (`host/robot_radio/testgui/` sim driver /
  `sim_loop.py`) and how the speed-up factor is applied to the tick interval
  vs. the per-tick dt.
