---
id: "001"
title: "Planner production hardening: fault-check baseline exclusion + heading-gain retune"
status: open
use-cases: [SUC-031, SUC-032]
depends-on: []
github-issue: ""
issue:
- executor-fault-check-needs-baseline-exclusion.md
- heading-loop-default-gains-overshoot-on-bench-rig.md
# completes_issue: Controls whether linked issues are archived when this ticket
# is moved to done. Default: true (archive when all referencing tickets are done).
# Set to false (scalar) to suppress archival for ALL linked issues on this ticket.
# Set to a mapping {filename.md: false} to suppress archival per issue filename.
# Use false for tickets that partially address a multi-sprint umbrella issue.
completes_issue: true
# exception: Written by a lower agent when it cannot proceed (see architecture §exception-protocol).
# exception:
#   thrown_by: "programmer"          # "programmer" | "sprint-planner"
#   thrown_at: "2026-05-07T14:23:00Z"
#   attempted: |
#     Description of what was attempted before giving up.
#   conflict: "architecture-update.md §3 — reason the agent is blocked"
#   surface: "internal"              # "user-visible" | "internal"
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Planner production hardening: fault-check baseline exclusion + heading-gain retune

## Description

Sprint 106 shipped `planner/executor.py`'s `StreamingExecutor` and
`planner/model.py`'s `PlannerParams` with two real gaps, both found during
106-006's own bench session and worked around only in that one bench
script (`tests/bench/profiled_motion_verify.py`), never promoted into
production `planner/` code:

1. **Fault-check has no baseline exclusion**
   (`executor-fault-check-needs-baseline-exclusion.md`). `StreamingExecutor.
   tick()` stops the run the instant ANY drained frame's `fault_bits` is
   nonzero. On real hardware `kFaultI2CSafetyNet` is latched from boot and
   essentially always present, so `tick()` as shipped fault-stops on tick 2
   of EVERY real run, 100% reproducible. 106-006 worked around this with a
   bench-script-local `BaselineFaultMaskingTransport` wrapper; the fix
   belongs in `executor.py` itself so every caller benefits — this
   sprint's own tour driver (ticket 002) chains 7+ legs through the SAME
   executor and would inherit the identical footgun without this fix
   landing first.
2. **Heading-loop default gains overshoot on this bench rig**
   (`heading-loop-default-gains-overshoot-on-bench-rig.md`).
   `PlannerParams`' shipped defaults (`heading_kp=2.0`,
   `heading_omega_clamp=0.5`) saturate the correction trim on the rig's
   high-inertia proxy load — a 60° turn landed at ~79° (+19°, ~+32%
   overshoot) during 106-006's bench session. `heading_kp=0.4`,
   `heading_omega_clamp=0.2` (exposed there only as CLI overrides) measured
   much better across 4 runs: -4.09°, -1.18°, +2.10°, +15.75° (one
   outlier). This sprint promotes those bench-proven values to
   `PlannerParams`' own field defaults — NOT a full gain sweep to a tight
   (`±3°`) tolerance, which both issues' own "Recommended follow-up"
   sections explicitly defer to a later, dedicated tuning session.

This is architecture-update.md's Step 6 Decision 2 and serves SUC-031/032.
It is the sprint's foundation ticket — everything else in this sprint
(the tour driver, the TestGUI rewire, the bench runs) chains multiple
`StreamingExecutor` runs and needs both fixes in place first, or every
downstream ticket would either fault-stop immediately or overshoot turns
unpredictably.

## Acceptance Criteria

- [ ] `StreamingExecutor.begin()` captures whichever `fault_bits` the first
      drained frame carries as that run's own baseline (mirroring
      `rig_soak.py`'s established "only a bit that turns on DURING the run
      counts as new" convention, and `profiled_motion_verify.py`'s own
      `BaselineFaultMaskingTransport.rebaseline()` per-run — not
      per-process — re-baselining rationale, since a benign
      `kFaultWedgeLatch` boundary latch can appear during an idle gap
      between runs).
- [ ] `tick()`'s fault check only raises `RunOutcome.FAULT` for a bit that
      is set NEW relative to that run's own baseline — a bit already
      present in the baseline frame never trips it.
- [ ] A bit that turns on freshly DURING a run (not present in the
      baseline frame) still stops the run with `RunOutcome.FAULT` —
      regression-protected, not weakened.
- [ ] `PlannerParams.heading_kp` defaults to `0.4`; `heading_omega_clamp`
      defaults to `0.2`. No new fields; `load()`'s JSON/env override
      plumbing is unchanged.
- [ ] `tests/unit/test_planner_executor.py` covers: (a) a fake transport
      whose first frame carries a nonzero `fault_bits` — the run must NOT
      fault-stop on it; (b) a bit that turns on mid-run after a
      zero-baseline first frame — the run MUST still fault-stop.
- [ ] `tests/unit/test_planner_model.py` (or equivalent) asserts the new
      `heading_kp`/`heading_omega_clamp` defaults.
- [ ] Full suite (`uv run python -m pytest`) stays green.
- [ ] The known `+15.75°` outlier risk (heading-loop issue's own bench
      finding) is restated in this ticket's own Completion Notes, not
      silently treated as solved — it is a documented, carried-forward risk
      for ticket 005's own closure-tolerance choice.

## Implementation Plan

### Approach

`StreamingExecutor.begin()` (currently drains one frame and sets
`self._baseline` to that frame's PROGRESS value, e.g. mean encoder
position) additionally captures that same first-drained frame's
`fault_bits` (or `0` if the frame is `None`/carries no `fault_bits`) into a
new `self._fault_baseline: int` field. `tick()`'s existing:

```python
fault = any(f.fault_bits for f in frames if f.fault_bits is not None)
```

becomes baseline-relative:

```python
fault = any((f.fault_bits & ~self._fault_baseline) for f in frames
            if f.fault_bits is not None)
```

`preempt()` already calls `begin()` fresh (binding requirement #4), so a
preempted run automatically gets a fresh fault baseline too — no separate
handling needed there.

`PlannerParams`' two field defaults are a one-line-each change; no other
field, no `load()` behavior change.

### Files to Modify

- `host/robot_radio/planner/executor.py` — `StreamingExecutor.begin()`
  captures `self._fault_baseline`; `tick()`'s fault check becomes
  baseline-relative.
- `host/robot_radio/planner/model.py` — `PlannerParams.heading_kp`
  `2.0 → 0.4`; `heading_omega_clamp` `0.5 → 0.2`. Update the inline
  docstring/comment on both fields to state the new bench-proven defaults
  and cite this ticket, replacing the "starting point... ticket 006's
  bench session would need to measure" language now that a real
  measurement backs the new defaults.

### Testing Plan

- Extend `tests/unit/test_planner_executor.py` with the two cases in
  Acceptance Criteria (nonzero-baseline-does-not-trip,
  new-bit-during-run-does-trip) using the file's own established
  `FakeTransport` double convention.
- Extend/add a `tests/unit/test_planner_model.py` (create if it doesn't
  exist) asserting the new field defaults directly (`PlannerParams().
  heading_kp == 0.4`, etc.) — a cheap regression guard against a future
  accidental revert.
- Run the full suite: `uv run python -m pytest`.
- No hardware/bench session is required for THIS ticket specifically (the
  fixes are unit-testable in isolation) — the bench-level confirmation that
  tours now run without fault-stopping happens naturally in ticket 005.

### Documentation Updates

- Update `executor-fault-check-needs-baseline-exclusion.md` and
  `heading-loop-default-gains-overshoot-on-bench-rig.md`'s own status (both
  are linked via this ticket's `issue:` frontmatter and will auto-archive
  to `done/` when this ticket completes — `completes_issue: true`).
- `profiled_motion_verify.py` (106-006) may optionally drop its own
  `BaselineFaultMaskingTransport` wrapper and CLI gain-override defaults
  now that production `executor.py`/`model.py` do the same thing — NOT
  required by this ticket's acceptance criteria (that script is outside
  this sprint's own Modified list), but leave a one-line comment there if
  touched incidentally.
