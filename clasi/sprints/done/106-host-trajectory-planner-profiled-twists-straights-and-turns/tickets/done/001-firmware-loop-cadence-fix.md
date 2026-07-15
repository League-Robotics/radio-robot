---
id: '001'
title: Firmware loop-cadence fix
status: done
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

- [x] `kSettle + kClear + kSettle` is `<=` the new `kCycle` target, and the
      final `sleepUntil` call's accounting is proven — via the sim's
      zero-real-time-cost virtual clock — to pad to that target rather than
      add a fresh, unabsorbed increment on top of it.
- [x] `tests/sim/support/sim_api.h`'s virtual-cycle-timing diagnostic
      (`CycleTimingReport`/`measureOneCycle()`, from 105-004) is promoted
      from an observational report to a hard pytest assertion on the new
      schedule shape — a regression in this schedule fails
      `uv run python -m pytest`, not just a future bench session.
- [x] Deployed to the bench rig
      (`.claude/rules/hardware-bench-testing.md`) and the real TLM cadence is
      measured (e.g. `relay_telemetry_rate.py` or an equivalent seq-gap
      capture) and recorded in this ticket's own Completion Notes,
      explicitly reconciling the new number against BOTH prior figures
      (105-004's ~36ms, the ack-ring issue's ~72ms/13.87Hz) rather than
      reporting a bare new number in isolation.
- [x] No change to command-dispatch or telemetry frame CONTENT — only
      pacing constants and/or `sleepUntil`'s own accounting change; every
      existing `TWIST`/`STOP`/`CONFIG` behavior and every telemetry field
      is byte-identical to before this ticket.
- [x] Full project test suite green (`uv run python -m pytest`).
- [x] Bench-verified per `.claude/rules/hardware-bench-testing.md`: sensors
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

## Completion Notes

**Final constants** (`source/app/robot_loop.cpp`, anonymous namespace):

| constant | value | meaning |
|---|---|---|
| `kSettle` | 4ms | unchanged — encoder-settle window, both motors |
| `kClear` | 4ms | unchanged — post-duty-write clearance window |
| `kCycle` | **40ms** (was 16ms) | whole-SCHEDULE total (all 4 pacing blocks), not just the trailing one |
| `kWindows` | 12ms (`2*kSettle+kClear`) | derived; what the 3 settle/clear blocks consume |
| `kPace` | **28ms** (`kCycle-kWindows`) | derived; the final block's own gap — NEW |

**The fix**: 105-004 proved the old code's trailing `sleepUntil(cycleStart,
kCycle)` requested a fresh, unabsorbed `kCycle` on top of the 12ms the three
settle/clear blocks already consumed (virtual total 4+4+4+16=28ms against a
16ms target). The trailing block is now a 4th `runAndWait`, using the SAME
"own mark, own gap" shape as the other three (previously the one exception —
it referenced `cycleStart` instead of taking its own fresh mark), with its
gap set to `kPace` (`kCycle` minus the three windows) instead of `kCycle`.
This makes the schedule sum to `kCycle` **by construction**, provable under
the sim's frozen virtual clock, not dependent on real elapsed time happening
to absorb it. `static_assert(kWindows <= kCycle, ...)` guards the invariant
at compile time. The OTOS-read + odometry-integrate body that used to sit
unwrapped after the old trailing `sleepUntil` now lives inside this 4th
block's body (mechanical move, same statements, same order — no behavior
change; this block is documented as the one exception to "the body never
touches the bus," since it's the schedule's pace block, not a
settle/clearance window).

**Sim assertion** (`tests/sim/system/sim_api_harness.cpp`,
`scenarioVirtualCycleTimingDiagnostic`): promoted from a diagnostic-only
report to a hard `checkTrue` regression assertion (already propagated to
pytest failure via `run_result.returncode`, `test_sim_api.py` — this was
mechanically already "hard" at the C++ level, but was previously asserting
the OLD, buggy 28ms/16ms-target numbers as the expected baseline; it now
asserts the FIXED 40ms/28ms-pace numbers, and its own comments were
rewritten to state it is a regression guard, not merely a diagnostic).
Verified: `report.sleepCount==4`, `report.lastSleepMillis==28`,
`report.yieldCount==0`, `report.virtualCycleMillis==40`. Full
`tests/sim/support/sim_api.cpp` `measureOneCycle()` derivation comment
updated to match (4+4+4+28=40==kCycle, an equality now, not an inequality).

**Full test suite**: `uv run python -m pytest` — 569 passed (same count as
the 569-passed sprint-start baseline; no test added/removed, only two
existing hard-coded expected numbers changed to match the new schedule).

**Bench deployment**: `just build-clean` (flash 132112B/364KB=35.44%,
RAM 98.33% — normal for this codebase, not a regression), flashed via
`mbdeploy deploy --hex MICROBIT.hex 9906360200052820a8fdb5e413abb276...`
(confirmed target by UID against `mbdeploy list`'s ROLE column — enum 2,
NEZHA2/robot, port `/dev/cu.usbmodem2121102`; the relay at 2121302 was never
touched). Robot left on this new image at session end.

**Bench-measured real cadence — the reconciled timing story**:

| capture | transport | period | rate | seq gaps |
|---|---|---|---|---|
| idle | direct USB | **52ms, rock-solid (stdev 0.00ms)** | 19.27 Hz | 0/288 |
| idle | relay | **52ms, rock-solid (stdev 0.00ms)** | 19.20 Hz | 0/287 |
| loaded (driving fwd+rev) | direct USB | median 52ms (69/92 samples exactly 52ms; the rest reflect the test script's own inter-phase pauses, not firmware jitter) | ~19 Hz | n/a |

Measured with a small scratch capture script (`NezhaProtocol.
read_binary_tlm_frames()`, always-on primary telemetry, no arming step
needed) reading each frame's own robot-clock `t` field — not committed to
the tree (ticket's own plan: reuse existing tooling first; a purpose-built
seq-gap script equivalent to `relay_telemetry_rate.py`'s own analysis was
simpler here since `relay_telemetry_rate.py` targets the separate, opt-in
armed-STREAM feature, not the P4 design's always-on primary push).

**Reconciliation of all THREE cadence figures** (the ticket's own required
deliverable):

1. **105-004's ~36ms** was an ESTIMATE (28ms virtual schedule + ~8ms
   estimated irreducible real I2C overhead), for the OLD, buggy schedule
   (16ms target, 28ms actually requested).
2. **ack-ring issue's ~72ms/13.87Hz** (104-007, a real sustained-soak
   measurement) is now fully explained: the OLD real loop period (~36ms,
   matching estimate 1) was BELOW `kPrimaryPeriod`=40ms, so `primaryDue()`
   needed TWO loop cycles to clear the 40ms gate — 2×36ms=72ms observed
   primary-emit period, i.e. the doubling defect Decision 1 set out to
   eliminate.
3. **This ticket's fresh measurement: 52ms/19.27Hz**, both transports,
   confirmed rock-solid. Reconciles as `kCycle`(40ms, now sim-provably
   exact) + ~12ms real, irreducible I2C/bus overhead not modeled by the
   HOST_BUILD virtual clock (the two `requestSample()`+`tick()` bus round
   trips and the OTOS read, none of which are wrapped in any accounted
   `runAndWait` window) = 52ms. This 12ms figure is in the same range as
   105-004's own ~8ms *estimate* for the same irreducible overhead category
   (not identical — the earlier figure was an estimate against a different,
   buggy schedule, not a controlled A/B of the overhead alone). Critically,
   52ms > `kPrimaryPeriod`=40ms on every cycle now, so `primaryDue()` is
   true every single call — the doubling defect from figure 2 is GONE
   (1:1 loop-cycle-to-primary-emit correlation, confirmed by the 0-gap,
   locked-52ms seq capture). Net cadence improvement: 72ms→52ms period,
   13.87Hz→19.27Hz (~39% faster) — real, measured, but short of the
   architecture doc's own "~25 Hz" aspiration, because real per-cycle I2C
   overhead (~12ms) is larger than the ~8ms Decision 1's alternatives
   analysis assumed.

**Bench gate** (`.claude/rules/hardware-bench-testing.md`): sensors alive —
OTOS confirmed live/plausible (`otos=(47,-3,1)`-style readings, changing
frame to frame); encoders confirmed live and changing. Line/color sensor
telemetry fields are NOT currently on the wire at all (a pre-existing,
documented gap — architecture-update.md (103) Step 7 Open Question 1: "no
`line=`/`color=` fields yet" — unrelated to and untouched by this ticket;
not claimed as verified here). Wheels drive both directions with encoders
incrementing/decrementing correctly: forward twist `enc (1,1)→(372,359)`
(both increasing), reverse twist `enc (399,386)→(36,44)` (both decreasing).
Round-trip over the real serial link confirmed: `twist()`/`stop()` acks
observed via the ack ring (`corr_id=1 ok=True`, `corr_id=3 ok=True`).

**Surprise — a genuine regression found and NOT fixed in this ticket
(out of its own stated scope, "only pacing constants ... change"):**
secondary telemetry (`TelemetrySecondary` — `cmd_vel`/`acc`/`glitch`/`ts`)
is now **starved to 0 Hz**, confirmed with two independent draining methods
over a 3s window, direct USB. Root cause: `Telemetry::emit()`'s own
pre-existing (103-009), documented contract — "primary checked first,
unconditionally sent when due — secondary can never delay it" — combined
with the new real loop period (52ms) sitting ABOVE `kPrimaryPeriod` (40ms)
on every cycle, means `primaryDue()` is true every single call and
`secondaryDue()` is never even reached. `telemetry.h`'s own comment already
named this exact failure mode ("a caller that invokes emit() at EXACTLY
the primary period would starve the secondary frame ... Not a defect this
ticket resolves — flagged for ticket 008's own loop-cadence choice" — 103-008
being the ticket that set the original 16ms `kCycle`; 106-001, this ticket,
is the one that actually changed it). Confirmed working BEFORE this ticket
(104-007's own soak measurement: 4.676 Hz secondary, close to its 5 Hz
target) and confirmed NOT working after. Filed as a new issue,
`clasi/issues/secondary-telemetry-starved-by-106-001-cadence-retarget.md`,
with candidate follow-up directions (round-robin priority, shrinking
`kCycle` — rejected, reintroduces the primary-doubling defect — moving
fields to the primary frame, or accepting the loss) — resolving the
trade-off is an `App::Telemetry` scheduling-contract decision, not a
pacing-constant tweak, so it is out of this ticket's own scope and is left
for a follow-up ticket rather than silently absorbed here.

Also observed (not a regression, not investigated further — pre-existing,
documented, level-set mechanisms unrelated to cadence): `fault_bits`
nonzero during bench sessions (`kFaultI2CSafetyNet`'s own documented
boot-time one-shot; `kFaultWedgeLatch` transiently during the bench
verification script's own rapid forward/reverse reversal pattern, a known
trigger per `.clasi/knowledge/encoder-wedge-boundary-latch.md`) — neither
is new behavior introduced by this ticket's pacing change.
