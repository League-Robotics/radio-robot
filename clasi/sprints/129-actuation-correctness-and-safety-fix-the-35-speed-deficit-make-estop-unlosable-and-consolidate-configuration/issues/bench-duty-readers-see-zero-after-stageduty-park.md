---
status: in-progress
sprint: '129'
tickets:
- 129-006
---

# Bench duty readers see zero after stageDuty() park (128-015 residual)

Sprint 128 ticket 015 parked `Planner::stageDuty()` out of the live tick
(stakeholder-approved PARK decision). Residual found at implementation
time: two explicitly-experimental bench tools read the duty stage's
output via ctypes and will now always see 0:

- `src/tests/bench/hil_drive.py --duty`
- `src/tests/bench/square_tour_sim.py` (its `plannerDuty()` read)

Neither is covered by automated tests and neither is a default path.
When the future duty-sink cutover happens (owner named in
`src/motion/DESIGN.md` §6), these tools come back to life; until then
they silently report zeros.

## Options for a future sprint

- Have both tools call the now-public `stageDuty()` explicitly before
  reading (keeps them honest without unpausing the live tick), or
- Print a "duty stage parked (128-015)" warning when `--duty`/duty reads
  are used, or
- Drop the duty-read modes from both tools until the cutover.
