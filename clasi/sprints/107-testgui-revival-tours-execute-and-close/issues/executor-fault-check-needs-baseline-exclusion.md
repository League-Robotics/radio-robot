---
status: in-progress
sprint: '107'
tickets:
- 106-006
- 107-001
---

# planner/executor.py's StreamingExecutor fault check has no baseline exclusion

## Problem

`StreamingExecutor.tick()` (`host/robot_radio/planner/executor.py`) stops the
run the instant ANY drained frame's `fault_bits` is nonzero:

```python
fault = any(f.fault_bits for f in frames if f.fault_bits is not None)
```

There is no baseline-relative exclusion for the boot-time one-shot
`kFaultI2CSafetyNet` bit, unlike every other bench script in this tree
(`tests/bench/rig_soak.py`'s own "only a bit that turns on DURING the run...
counts as a NEW fault" convention). On real hardware `kFaultI2CSafetyNet` is
latched from boot and essentially always present — confirmed during sprint
106 ticket 006's bench session: `fault_bits=1` on the very first real
drained frame of every leg tried. As written, `StreamingExecutor.run()`/the
`tick()` loop **never completes a real run** — it fault-stops on tick 2
every single time, 100% reproducible.

`tests/unit/test_planner_executor.py::test_fault_bit_mid_run_stops_and_logs`
(ticket 106-005's own AC 10 coverage) never caught this because its
`FakeTransport` double always starts implicitly at a zero baseline — the
"a bit is ALREADY set before the run even begins" real-hardware case was
never exercised.

## Workaround in place

`tests/bench/profiled_motion_verify.py` (ticket 106-006) wraps
`NezhaProtocol` in a `BaselineFaultMaskingTransport`: it masks out whatever
`fault_bits` are present in the first frame drained after each
`rebaseline()` call before handing the frame to `StreamingExecutor`.
`rebaseline()` is called once per leg (not just once globally) because a
benign `kFaultWedgeLatch` "boundary latch"
(`.clasi/knowledge/encoder-wedge-boundary-latch.md` — several consecutive
identical encoder reads while genuinely stationary, not a real mechanical
wedge) was observed appearing during an intentional idle gap between legs;
a single global baseline would have incorrectly poisoned the fault gate for
the rest of the session. This workaround is scoped to the bench script only
— `executor.py` itself was left completely unmodified (ticket 006's own
plan: "Files to modify: none beyond what tickets 001-005 already changed"),
since `TwistTransport` is a `Protocol` precisely so this kind of adapter
needs no source change.

## Recommended fix

Move baseline-relative fault-bit handling into `executor.py` itself
(mirroring `rig_soak.py`'s own convention: capture whatever `fault_bits` the
first drained frame at `begin()` carries as that run's baseline, and only
treat a bit as "new" — triggering `RunOutcome.FAULT` — if it turns on
relative to that baseline) so every real caller of `StreamingExecutor`
benefits, not just `profiled_motion_verify.py`. This is a production-code
change to a module delivered by ticket 106-005, not something ticket 106-006
was scoped to make.

## Evidence

- `clasi/sprints/106-host-trajectory-planner-profiled-twists-straights-and-turns/tickets/006-bench-gate-sim-validated-then-real-profiled-straight-turn-captured-traces.md`'s
  own Completion Notes (finding 1).
- Captured bench traces showing `fault_bits=1` from the very first real
  frame: `tests/bench/out/profiled_{straight,turn}_20260715T*.json` sidecars'
  `raw_fault_bits_ever_observed` field.
