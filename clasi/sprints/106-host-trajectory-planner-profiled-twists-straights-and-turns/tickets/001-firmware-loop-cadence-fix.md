---
id: '001'
title: Firmware loop-cadence fix
status: open
use-cases:
- SUC-024
depends-on: []
github-issue: ''
issue:
- host-planner-design-lessons-from-drive-v2-review.md
- heading-loop-output-clamp-and-velocity-resonance.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Firmware loop-cadence fix

## Description

`source/app/robot_loop.cpp`'s `cycle()` already writes `sleepUntil(cycleStart,
kCycle)` — syntactically "anchored" — but ticket 105-004's virtual-cycle-timing
diagnostic proved this is a real scheduling defect, not merely environmental
slowdown: with a fake `Devices::Clock` that never advances mid-`cycle()`, the
SAME code deterministically requests `4+4+4+16=28ms` of virtual sleep per
cycle (the three hardware-mandated `kSettle`×2/`kClear`×1 windows plus the
final pace block) against a stated `kCycle=16ms` target — 12ms over, because
the three windows are additive to the final pace block, not absorbed into it.
Sprint 104 bench-measured ~36ms/cycle real; `ack-ring-intermittent-delivery-
gap.md` separately cites ~13.87 Hz (~72ms) — a further, currently
unreconciled gap.

Telemetry emits once per cycle and is the ONLY feedback channel this sprint's
host planner (heading loop, profile executor) will have — this ticket fixes
the schedule's own internal consistency and retargets it to an honest ~25 Hz
(~40ms) design point (not the original, never-achievable 16ms), before any
other sprint 106 ticket depends on a stable cadence. See
`architecture-update.md` Step 1 finding 1 and Decision 1 for the full
reasoning, and Step 7 Open Question 3 for the reconciliation this ticket must
close.

## Acceptance Criteria

- [ ] `kSettle + kClear + kSettle` is `<=` the new `kCycle` target, and the
      final `sleepUntil` call's accounting is proven — via the sim's
      zero-real-time-cost virtual clock — to pad to that target rather than
      add a fresh, unabsorbed increment on top of it.
- [ ] `tests/sim/support/sim_api.h`'s virtual-cycle-timing diagnostic
      (`CycleTimingReport`/`measureOneCycle()`, from 105-004) is promoted
      from an observational report to a hard pytest assertion on the new
      schedule shape — a regression in this schedule fails
      `uv run python -m pytest`, not just a future bench session.
- [ ] Deployed to the bench rig
      (`.claude/rules/hardware-bench-testing.md`) and the real TLM cadence is
      measured (e.g. `relay_telemetry_rate.py` or an equivalent seq-gap
      capture) and recorded in this ticket's own Completion Notes,
      explicitly reconciling the new number against BOTH prior figures
      (105-004's ~36ms, the ack-ring issue's ~72ms/13.87Hz) rather than
      reporting a bare new number in isolation.
- [ ] No change to command-dispatch or telemetry frame CONTENT — only
      pacing constants and/or `sleepUntil`'s own accounting change; every
      existing `TWIST`/`STOP`/`CONFIG` behavior and every telemetry field
      is byte-identical to before this ticket.
- [ ] Full project test suite green (`uv run python -m pytest`).
- [ ] Bench-verified per `.claude/rules/hardware-bench-testing.md`: sensors
      alive, wheels drive both directions with encoders incrementing in
      proportion to commanded speed, round-trip confirmed over the real
      serial link.

## Testing

- **Existing tests to run**: full `uv run python -m pytest` (baseline 569
  passed at sprint start); `tests/sim/support/`/`tests/sim/system/`'s
  existing 105-001..006 suites in particular, since this ticket touches the
  same `RobotLoop::cycle()` those tests exercise.
- **New tests to write**: a hard pytest assertion (in `tests/sim/system/` or
  wherever 105-004's own harness test lives) on the new virtual per-cycle
  schedule total, replacing the current diagnostic-only report.
- **Verification command**: `uv run python -m pytest`, plus the bench
  cadence measurement (not a pytest-automatable step — real hardware).

## Implementation Plan

**Approach**: Recompute `source/app/robot_loop.cpp`'s `kSettle`/`kClear`/
`kCycle` constants (and/or `sleepUntil`'s own elapsed-time accounting) so the
three hardware-mandated windows are explicitly counted INSIDE the stated
cycle budget rather than stacked on top of it, retargeting the total to a new,
honest figure (~40ms/~25Hz, not the original aspirational 16ms — 105-004's
completion notes show 12ms of that gap is pure schedule arithmetic, ~8ms is
irreducible real I2C overhead the schedule cannot remove). Iterate the exact
constant values against `tests/sim/support/sim_api.h`'s existing
`CycleTimingReport`/`measureOneCycle()` (already reports `sleepCount`/
`lastSleepMillis`/derived `virtualCycleMillis`) until the sim's own
deterministic zero-noise virtual clock confirms the intended total, then
promote that check from a report to a hard assertion. Deploy to the bench rig
per `.claude/rules/hardware-bench-testing.md` and re-measure real cadence
with an existing bench capture tool (`relay_telemetry_rate.py` or equivalent
seq-gap accounting over direct USB), explicitly stating in Completion Notes
how the new figure relates to the two prior, currently-conflicting
measurements.

**Files to modify**:
- `source/app/robot_loop.cpp` — `kSettle`/`kClear`/`kCycle` constants and/or
  `sleepUntil`'s accounting.
- `tests/sim/support/sim_api.h`/`.cpp` and/or the `tests/sim/system/` harness
  test that currently reports (but does not assert on) the timing
  diagnostic — promote to a hard assertion.

**Files to create**: none expected; reuse an existing bench cadence-capture
tool if one already fits the P4 binary telemetry stream (check
`relay_telemetry_rate.py` first) rather than writing a new one.

**Testing plan**: full `uv run python -m pytest` must stay green; the
promoted timing-diagnostic assertion is this ticket's own regression guard
going forward. Bench re-measurement is manual, per
`.claude/rules/hardware-bench-testing.md`, with the result recorded in
Completion Notes (not itself a pytest-automatable check).

**Documentation updates**: this ticket's own Completion Notes record the
final constants, the sim-asserted schedule total, the bench-measured real
cadence, and the reconciliation against the 105-004 (~36ms) and ack-ring
issue (~72ms) figures — the artifact `architecture-update.md` Step 7 Open
Question 3 asks for. No other external doc requires an update.
