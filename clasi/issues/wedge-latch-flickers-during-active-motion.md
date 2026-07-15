---
status: pending
---

# `kFaultWedgeLatch` observed flickering DURING active motion, not just idle

Found during ticket 107-001's own HITL bench verification
(`clasi/sprints/107-testgui-revival-tours-execute-and-close/tickets/001-planner-production-hardening-fault-check-baseline-exclusion-heading-gain-retune.md`)
of `StreamingExecutor`'s new baseline-relative fault check, against the real
robot (`/dev/cu.usbmodem2121102`, "tovez", bench rig).

## Problem

`.clasi/knowledge/encoder-wedge-boundary-latch.md` and
`heading-loop-default-gains-overshoot-on-bench-rig.md`/
`executor-fault-check-needs-baseline-exclusion.md` (107-001's own consumed
issues) both characterize `kFaultWedgeLatch` (bit `0x2`) as a "boundary
latch" that appears during an IDLE gap between runs (several consecutive
identical encoder reads while genuinely stationary).

A standalone diagnostic run during 107-001's own bench session (issuing a
forward `twist()` nudge and reading raw, per-frame `fault_bits` — not just
the last frame of a drain batch) observed the bit toggling
`3 (0x1|0x2) → 3 → 1 → 1 → ...` WHILE the robot was actively, continuously
moving forward — not idle. Separately, two full `profiled_motion_verify.py`
runs saw the straight leg's `StreamingExecutor` run correctly fault-stop
mid-motion on a genuinely NEW `0x2` bit (baseline-relative exclusion working
exactly as designed — this is not a bug in 107-001's own fix), interrupting
an otherwise-clean profiled straight leg.

This is a real, previously-uncharacterized flavor of the wedge-latch family:
it is not confined to idle/stationary periods. It reduces the real-world
completion rate of `StreamingExecutor` runs (baseline exclusion only
absorbs what is present AT `begin()`; a bit that flickers on-and-off within
milliseconds during active motion will still occasionally land as "new"
relative to that baseline and fault-stop a run that was otherwise healthy).

## Evidence

- 107-001's own Completion Notes, "Surprises" #2.
- Standalone diagnostic transcript (this session, not committed): raw
  per-frame `(fault_bits, event_bits)` during a forward `twist()` nudge
  showed `(3, 2), (3, 2), (1, 2), (1, 2), ...` — the bit dropping from set
  to clear WHILE `v_x=80mm/s` was still being commanded.
- `tests/bench/out/profiled_straight_20260715T161012Z.json` and
  `..._161035Z.json` — both `outcome=fault`, `new_fault_bits=0x2`,
  interrupting an in-progress straight leg (not an idle gap).

## Recommended direction

A dedicated investigation (mirrors `.clasi/knowledge/encoder-wedge-boundary-latch.md`'s
own root-causing) into why `kFaultWedgeLatch` flickers during active,
continuous single-direction motion, not just at idle/direction-reversal
boundaries. Candidate directions: check whether the firmware's own
wedge-detection heuristic (consecutive-identical-encoder-reads) can
false-trigger on a genuinely slow/near-stall wheel speed, not just a truly
stopped one; consider whether `StreamingExecutor` needs a short temporal
debounce (e.g. "new bit must persist across 2 consecutive drained frames")
on top of the baseline exclusion 107-001 already added, rather than
fault-stopping on a single transient frame.
