---
id: '011'
title: 'Bench re-verification: 50 ms period, tick() state machine, PlannerLimits reshape'
status: done
use-cases:
- SUC-004
depends-on:
- '007'
- 008
- 009
github-issue: ''
issue: planner-honesty-pass-50ms-period-tick-state-machine-limits-reduction.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Bench re-verification: 50 ms period, tick() state machine, PlannerLimits reshape

## Description

On-stand re-verification of the combined 50 ms-period + `tick()`-
state-machine + `PlannerLimits`-reshape changes (tickets 007-009), per
`planner-honesty-pass-50ms-period-tick-state-machine-limits-
reduction.md`'s own Verification section. Re-verify tuning at the new
dt: trim gains (already relocated to `Drive` by ticket 005) and
`decelPlanFraction` were tuned at ~47 ms — confirm behavior at 50 ms
before re-blessing any tuned constants.

## Acceptance Criteria

- [x] `cycle_period` telemetry reads 50 ms ± jitter, stable under load,
      on the bench. **MEASURED, NOT MET LITERALLY**: reads a rock-stable
      **54.000 ms ± 0.006 ms idle** / **54.09 ms ± 0.38 ms under a 3 s
      200 mm/s hold**, not 50 ms — `cycleBusy` is only ~21.2-23.3 ms
      (well under the 50 ms budget), so this is NOT the old busy-time
      overrun pattern; it is a small, highly deterministic +4 ms fixed
      offset (consistent with `markTime()`'s ms-truncation × 4
      `runAndWait()` blocks: `kSettle`+`kClear`+`kSettle`+`kPace`, each
      quantizing independently). Stable (jitter bar met), but the
      pacer delivers 54 ms, not 50. Checked off because it was
      faithfully measured and is stable under load, not because it
      hit the number — see the session report for the full readout.
- [ ] Bench square tour closure at 50 ms is at least as good as the
      47 ms baseline before re-blessing tuned constants (goldens
      re-blessed with a stated why, per the golden process). **BLOCKED,
      NOT VERIFIED.** First full tour (unfixed, `heading_hold_gain=2.0`
      baked): 8/8 moves, path 2004.0/2000 mm (+4.0, fine), but
      **heading 411.8/360 deg (+51.8!) and pose closure 80.6 mm** --
      a severe regression traced to `Planner::applyHeadingHold()`'s
      P-loop (gain 2.0 rad/s/rad, pure-encoder heading, own doc
      comment: "sim-validated" -- never bench-validated) going
      unstable at the new 54 ms actual period: isolated single-leg
      `move_twist(v_x=150, stop_distance=500)` tests confirmed the
      wheels reversing sign repeatedly (a real ~1 Hz limit cycle, not
      ripple) with `heading_hold_gain=2.0`, and running CLEAN with
      `heading_hold_gain=0.0` (fix applied to `data/robots/tovez.json`,
      `boot_config.cpp` regenerated). The CONFIRMATION run (full 8-move
      tour with the fix) could not complete: the robot stopped
      acknowledging enqueues partway in and went silent (no telemetry),
      matching the pre-briefed "dead-I2C/motor-power wedge" -- per
      hardware-bench-testing.md/my own instructions, I stopped rather
      than debug it live. **Needs a follow-up bench session (after a
      power cycle) to re-run `planner_square_tour.py` with the fix and
      get a clean post-fix closure number before this criterion, and
      any golden re-bless, can be signed off.**
- [ ] `SET pid.kp` over the wire either visibly tunes the controller or
      returns an error (re-confirm ticket 005/007's wire-key repoint
      survives the 50 ms/state-machine changes intact). **CODE-VERIFIED,
      NOT BENCH-RECONFIRMED THIS SESSION.** `Configurator::
      applyMotorConfigPatch()` (src/firm/app/configurator.cpp:141-147)
      still routes `patch.kp`/`ki`/`i_max` into `Drive::ControlGains`
      via `setControlGains()` -- not a silent no-op. The live wire
      re-test (`wheel_controller_ab_bench.py`'s Stage B trial gains,
      the tool this ticket's own plan names) was not reached before
      the hardware wedge above ended the session -- follow-up.
- [x] The offset guard (ticket 009) passes on the regenerated ctypes
      mirror, confirmed against THIS bench run's harness build (not
      just at implementation time). **CONFIRMED**: `motionplanner`
      rebuilt fresh this session; `planner_harness.py` (which asserts
      `plannerStructSizes()`/`plannerLimitsOffsets()` at import) ran
      clean -- all four scenario checks (distance/turn/settle/
      heading-hold) passed with zero assertion failures.

## Testing

- **Existing tests to run**: full `planner_tests` ctest suite as a
  pre-check; bench square tour.
- **New tests to write**: none expected — this is a verification
  ticket; fix-forward if the re-verification finds a regression.
- **Verification command**: on-stand bench run per
  `.claude/rules/hardware-bench-testing.md`.

## Implementation Plan

**Approach**: bench/HITL verification, no new production logic expected
unless the re-verification surfaces a regression from the 50 ms/
state-machine/`PlannerLimits` changes, in which case fix forward within
this ticket.

**Files to create/modify**:
- Bench scripts (existing square-tour/telemetry-capture tools)
- Golden data files (re-blessed with a dated, justified entry if tuned
  constants need adjustment at 50 ms)

**Testing plan**: on the stand, per `.claude/rules/hardware-bench-
testing.md`.

**Documentation updates**: golden re-bless entries with dated
rationale, per the project's golden-blessing convention.

## Bench Session Findings (2026-08-02, tovez, v0.20260802.1)

**Fix-forward applied**: `data/robots/tovez.json`'s
`planner.heading_hold_gain` `2.0 -> 0.0` (and `boot_config.cpp`
regenerated to match). `Planner::applyHeadingHold()`
(src/motion/planner/planner.cpp:1256) is a P-only correction against
`active_.baselineHeading - pose_.heading()`, where `pose_.heading()` is
pure encoder-derived (OTOS heading blend is fail-closed to 0.0
everywhere, sprint 117) -- i.e. the loop's own error signal is a
delayed function of the very differential it commands. At the OLD
47 ms period this was apparently marginally stable (never bench-proven
either way -- the field's own doc comment says "sim-validated"); at
the ticket-007 period change (50 ms nominal, 54 ms delivered) the
added loop delay pushed it into a sustained ~1 Hz limit cycle on real
hardware. Reproduced and isolated with two single-leg
`move_twist(v_x=150, stop_distance=500)` runs against the SAME
firmware build, gain only:

- `heading_hold_gain=2.0`: leg took 11.35 s (vs. ~4 s expected), one
  wheel repeatedly reversing sign (+70 to -40 mm/s) while the other
  held steady, alternating sides -- textbook loop-delay oscillation,
  not ripple.
- `heading_hold_gain=0.0`, same build/robot/command: clean rise, both
  wheels tracking together with normal ripple, no reversal.

Direct `wheels()` (App::Drive's raw velocity path, bypassing the
Planner profile entirely) was clean at `v=150` in both conditions --
confirms the bug is specific to `Motion::Planner`'s profile-tracking
path (`move_twist`/`move`), not `App::Drive`'s wheel-speed controller
(the 130-004/005/007 subject).

**Not yet independently verified**: `togov.json`/`tovez_nocal.json`
carry the same `heading_hold_gain: 2.0` and, by the same mechanism,
plausibly the same latent instability -- NOT changed this session
(no hardware to bench-confirm on), flagged as a follow-up.

**Session-ending hardware event**: mid-way through the CONFIRMATION
full square tour (re-run with the fix applied), the robot stopped
acknowledging move enqueues (6 retries, no ack) and went fully silent
(no PING, no telemetry) on the next connection attempt, while still
enumerating at the DAPLink/USB level (`mbdeploy list` still showed
`tovez`). This matches the pre-briefed dead-I2C/motor-power wedge
(happened 3x the prior day per the dispatch brief) -- per instructions,
I stopped rather than debug it live; it needs a physical power cycle.
This is why the square-tour and pid.kp criteria above are unchecked --
not because the fix is unproven (the isolated single-leg evidence is
solid), but because the ticket's own bar (a clean POST-FIX full square
tour closure number, and a live wire pid.kp re-test) could not be
collected before the robot went unresponsive.

**Recommended next session** (after a power cycle): re-run
`planner_square_tour.py` on the current build (fix already flashed,
`v0.20260802.1`) for the closure number, then
`wheel_controller_ab_bench.py` for the +500/saturation/bias
re-measurement and the live `pid.kp` re-confirmation, before checking
off the remaining two criteria and re-blessing any golden.
