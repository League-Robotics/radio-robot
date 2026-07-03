---
id: '002'
title: Signed/direction-aware DISTANCE + ROTATION stop + EVT safety_stop
status: open
use-cases:
- SUC-002
- SUC-003
depends-on:
- '001'
github-issue: ''
issue: distance-stop-fabsf-accepts-backward-completion.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Signed/direction-aware DISTANCE + ROTATION stop + EVT safety_stop

## Description

`StopCondition::evaluate()`, `Kind::DISTANCE`
(`source/control/StopCondition.cpp:97-106`) gates on
`fabsf(enc_avg - base.enc0) >= a` — a magnitude comparison with no record
anywhere in `MotionBaseline` of which direction the drive was actually
commanded to move. A `D 200 200 500` (forward) that instead runs away
backward (controller windup, a stuck wheel, etc.) satisfies
`traveled >= 500` once it has gone 500 mm the WRONG way and reports
`EVT done D reason=dist` — a success-indistinguishable terminal event for a
robot that just drove a meter off the far edge of a table. The same
`fabsf` pattern exists in `Kind::ROTATION` (`StopCondition.cpp:171-174`)
and in the Planner D-mode decel hook's own `d_traveled = fabsf(enc_avg -
_dEnc0)` (`Planner.cpp:224`). This is not hypothetical: a forced-stall sim
experiment against the real firmware control code reproduced exactly this
— the controller wound up, flipped negative, and drove over a meter
backward before reporting `EVT done D reason=dist`.

This ticket makes `DISTANCE` and `ROTATION` direction-aware and adds a
faster wire-visible safety net:

1. **Signed stop conditions.** `MotionBaseline` gains `vSign`/`omegaSign`
   (float, ±1.0 or 0.0), captured at `MotionCommand::start()` from the
   command's commanded `v`/`omega`. `Kind::DISTANCE`/`Kind::ROTATION`
   compute `raw * base.vSign` / `raw * base.omegaSign` instead of
   `fabsf(raw)` and gate on `signedDelta >= target`. The common case
   (travel matches commanded direction) is bit-identical in outcome
   (`signedDelta == |raw|`); a reverse-commanded drive (`D -200 -200 500`)
   still completes normally on backward travel. The Planner D-mode decel
   hook's own `d_traveled` computation drops its `fabsf` for the same
   signed convention, so the profile and the stop condition agree about
   what "remaining" means.
2. **New `Kind::SAFETY_MARGIN` -> `EVT safety_stop`.** Item 1 stops FALSE
   completions but does not by itself cut power faster than the existing
   generous TIME net (2x nominal + 2 s) when a robot is actively running
   away. A new stop kind fires when signed travel crosses a configurable
   negative margin during a directed `D`; `MotionCommand` recognizes this
   kind as safety-class: forced HARD teardown (regardless of the command's
   configured SOFT style) and the EVT label forced to `EVT safety_stop`
   (reusing the exact label the keepalive watchdog already emits, with an
   additive `reason=runaway` token) instead of the command's configured
   `EVT done D`.

Scoped to `D` only (not `G`/PURSUE/`RT`) per architecture-update.md Open
Question 2. See `architecture-update.md` Step 3 (`MotionBaseline`,
`StopCondition`, `MotionCommand` — all extended), Step 4a's pipeline
diagram, Decision 1 (why `vSign`/`omegaSign` live on `MotionBaseline`, not
a `StopCondition` param or a call-site pre-negation trick), and Decision 2
(why `SAFETY_MARGIN` is a new `StopCondition::Kind` dispatched via a
`MotionCommand`-level special case, not a bespoke Planner check or an
overloaded `DISTANCE` index).

See `architecture-update.md` Step 3, Step 4a, Step 5 ("Ticket 002"),
Decisions 1 and 2; `usecases.md` SUC-002, SUC-003.

## Acceptance Criteria

- [ ] `MotionBaseline` gains `vSign`/`omegaSign` fields, computed at
      `MotionCommand::start()` from the command's commanded `v`/`omega`
      (±1.0, or 0.0 if commanded velocity is exactly zero).
- [ ] `Kind::DISTANCE` fires on `signedDelta >= target` where
      `signedDelta = raw * base.vSign`, not `fabsf(raw) >= target`.
- [ ] `Kind::ROTATION` fires on the equivalent signed comparison using
      `omegaSign`.
- [ ] A forward `D` (`D 200 200 500`) whose encoders instead accumulate
      500 mm of BACKWARD travel does NOT fire the DISTANCE stop from that
      backward travel (does not emit `EVT done D reason=dist`).
- [ ] A reverse `D` (`D -200 -200 500`) that travels 500 mm backward DOES
      fire the DISTANCE stop and completes normally — no regression on the
      legitimate reverse-drive case.
- [ ] `RT <cdeg>` in each direction (positive and negative) still
      terminates on its own commanded-direction arc; a wrong-direction
      encoder differential does not satisfy the ROTATION stop.
- [ ] The Planner D-mode decel hook's `d_traveled` computation
      (`Planner.cpp:224`) uses the same signed convention (drops its
      `fabsf`), so `v_cap`'s shaping and the stop condition agree about
      "remaining" throughout a drive.
- [ ] New `StopCondition::Kind::SAFETY_MARGIN` fires when signed traveled
      distance crosses more than a configurable margin NEGATIVE relative
      to the commanded direction during a directed `D`.
- [ ] `beginDistance()` installs `SAFETY_MARGIN` as a third stop condition
      alongside the existing DISTANCE/TIME pair (`kMaxStopConds` stays at
      4; `D` now uses 3 of 4 slots).
- [ ] `MotionCommand::tick()` special-cases a fired `SAFETY_MARGIN`: forces
      HARD teardown (zero PWM immediately, not a SOFT ramp) regardless of
      the command's configured `_stopStyle`, and forces the emitted EVT
      label to `EVT safety_stop` with an additive `reason=runaway` token,
      bypassing the command's configured `_doneEvtLabel`.
- [ ] The safety-margin abort fires within one control tick of crossing the
      margin — not the multi-second TIME net.
- [ ] `EVT safety_stop`'s wire shape remains compatible with existing hosts
      that already recognize it from the keepalive-watchdog path (additive
      `reason=` token only; no change to the base label).
- [ ] The safety margin is a new `RobotConfig`/`SET`-able field (not a
      hardcoded constant), consistent with 067's live-SET propagation
      rule.
- [ ] `docs/wire-protocol.md` (or equivalent) documents `EVT
      safety_stop`'s new `reason=runaway` token as additive.
- [ ] `test_distance_fires_for_reverse`
      (`tests/simulation/unit/test_stop_condition.py`) is split per
      architecture-update.md Step 5 into a still-passes commanded-reverse
      case and a new must-not-fire commanded-forward-travels-backward case
      (full split finalized in ticket 004; this ticket may stage the split
      if convenient, but ticket 004 is the authoritative verification
      pass).
- [ ] Full existing test suite remains green except the one test named
      above, which ticket 004 finalizes.

## Testing

- **Existing tests to run**: `tests/simulation/unit/test_stop_condition.py`,
  `tests/simulation/system/test_stop_condition_coverage.py`,
  `test_rotation_stop_terminates_spin` (confirms unaffected — commands and
  travels in the same direction), full suite.
- **New tests to write**: forward-D-runs-backward does not fire DISTANCE;
  reverse-D behaves unchanged; SAFETY_MARGIN fires and forces HARD +
  `EVT safety_stop`; ROTATION direction-awareness in both spin directions;
  a test confirming `EVT safety_stop`'s `reason=runaway` token is additive
  and does not break existing keepalive-path assertions.
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: Add `vSign`/`omegaSign` to `MotionBaseline`
(`source/control/StopCondition.h`) and compute them in
`MotionCommand::start()` from the command's commanded `v`/`omega` at the
moment `start()` is called. Update `StopCondition.cpp`'s `DISTANCE`/
`ROTATION` branches to use the signed delta. Add `Kind::SAFETY_MARGIN` as
a new enumerator with its own `a` param (margin, mm) and evaluate()
branch: fires when `raw * base.vSign <= -a`. In
`MotionCommand::tick()`'s existing stop-fired branch (which already
switches on `_stopStyle`/`_firedKind` to build the `reason=` token and
choose SOFT/HARD teardown), add one more condition:
`_firedKind == Kind::SAFETY_MARGIN` forces HARD + overrides the EVT label
— the same mechanism the existing `_stopStyle == HARD` check already uses,
not a new one. In `Planner.cpp`'s `beginDistance()`, install the new
`SAFETY_MARGIN` stop condition alongside DISTANCE/TIME; in the D-mode
decel hook, drop the `fabsf` on `d_traveled` in favor of the signed
convention.

**Files to create/modify**:
- `source/control/StopCondition.h` — `MotionBaseline` gains `vSign`/
  `omegaSign`; new `Kind::SAFETY_MARGIN` enumerator and its `a` param.
- `source/control/StopCondition.cpp` — `DISTANCE`/`ROTATION` signed-delta
  computation; new `SAFETY_MARGIN` branch.
- `source/commands/MotionCommand.h`/`.cpp` — `start()` computes `vSign`/
  `omegaSign`; `tick()`'s stop-fired branch special-cases `SAFETY_MARGIN`.
- `source/superstructure/Planner.cpp` — `beginDistance()` installs the
  third stop condition; the D-mode decel hook's `d_traveled` drops
  `fabsf`.
- New `RobotConfig` field for the safety margin: `source/types/Config.h`,
  `source/robot/DefaultConfig.cpp`, `source/robot/ConfigRegistry.cpp`,
  `data/robots/robot_config.schema.json` (same four-file coordinated-edit
  pattern as 071's field additions; unit noted in a comment, not the
  identifier, per 071's naming convention).
- `docs/wire-protocol.md` (or equivalent) — document `EVT safety_stop
  reason=runaway` as additive.
- `tests/simulation/unit/test_stop_condition.py` — new direction-aware
  tests; `test_distance_fires_for_reverse` split (finalized in ticket 004).

**Testing plan**: run `test_stop_condition.py` and
`test_stop_condition_coverage.py` in isolation first, then the full suite.
Confirm `test_rotation_stop_terminates_spin`'s `RT 9000` scenario is
unaffected (commands and travels in the same direction).

**Documentation updates**: `docs/wire-protocol.md` (or equivalent) for the
new `EVT safety_stop reason=runaway` token and the new `SET`-able safety
margin field.
