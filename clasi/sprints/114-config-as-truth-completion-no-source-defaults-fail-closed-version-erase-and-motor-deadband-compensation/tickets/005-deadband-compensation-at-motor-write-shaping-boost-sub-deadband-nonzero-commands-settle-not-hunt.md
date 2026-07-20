---
id: '005'
title: 'Deadband compensation at motor write-shaping: boost sub-deadband nonzero commands,
  settle not hunt'
status: open
use-cases: [SUC-005]
depends-on: ['003', '007']
github-issue: ''
issue: deadband-compensation-small-commands-must-produce-real-motion.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Deadband compensation at motor write-shaping: boost sub-deadband nonzero commands, settle not hunt

## Description

Fix the motor write-shaping dead zone at its source: `NezhaMotor::
writeShapedDuty()` boosts a genuine nonzero sub-deadband duty up to the
deadband floor instead of zeroing it, while an exact-zero command stays an
immediate, unclamped hard stop. Depends on ticket 003 because the fix boosts
*to* `outputDeadband_`, which must be a real, config-sourced value (not a
private implementation constant) for "boost to the configured deadband" to
be a meaningful, correct statement.

**Depends on ticket 007 too (Revision 2)**: ticket 007 fixes a pre-existing
`TestSim::WheelPlant`/`SimPlant` gap (no per-port motor mount-orientation
model) that makes the sim's ground-truth pose spin instead of translate
under `tovez_nocal.json`'s real asymmetric `fwd_sign`. This ticket's own
production change (`writeShapedDuty()`) is a pure scalar-duty function with
no pose/kinematics involvement — provably orthogonal to that bug. But this
ticket's own **sim system test** (the ~11 mm/s-terminal-correction scenario
and the settle-not-hunt sweep) runs in the same sim; sequencing ticket 007
first removes any need to reason about whether that test's own scenario
construction happens to route through OTOS ground truth (a heading-hold
move's terminal correction, for instance, would) — it simply cannot be
contaminated by a bug that no longer exists by the time this ticket's tests
run. Write this ticket's test scenario however is clearest/most direct
(a full heading-hold move terminal correction is fine now); do not add
special-case scaffolding to avoid ground truth — that scaffolding is no
longer necessary once ticket 007 has landed.

## Context

Current code (`src/firm/devices/nezha_motor.cpp`, `writeShapedDuty()`):

```cpp
if (duty == 0.0f || fabsf(duty) < outputDeadband_) {
    // Stop always wins: immediate, unclamped, cancels any dwell.
    dwelling_ = false;
    lastRequestedDuty_ = 0.0f;
    writeRawDuty(0.0f);
    return;
}
```

This single branch treats "exactly zero" and "nonzero but too small to move
the plant" identically — both get written as 0. The deadband-compensation
issue's own diagnosis: a ~11 mm/s terminal heading correction (a *genuine*,
wanted, nonzero command) falls inside the ~15 mm/s (`outputDeadband_ ≈ 0.03`
duty) dead zone, gets zeroed, the wheel never moves, the PD's error never
shrinks, the command holds flat for ~8s until an arrive-timeout gives up.

**Why this does not reproduce sprint 112-004's deleted min-speed-floor
regression** (read sprint.md's Design Rationale Decision 4 in full before
implementing — this is the load-bearing design argument for *why* this fix
is safe where the old one wasn't): the old floor lived in `App::Pilot`'s PD
*output* (the twist reference), a layer with no velocity feedback of its
own — boosting the reference there could push it past what the PD actually
wanted, and the next PD sample only ever compared against its own prior
reference, never the plant's actual response, so it could sustain an
oscillation. This fix lives instead inside `NezhaMotor`'s own velocity-PID
closed loop: `writeShapedDuty()` is downstream of the PID's `compute()`
call, which reads real *measured* velocity every tick. A boosted write is
never invisible to the loop — next tick's measurement already reflects the
boosted motion, and the PID's own proportional term shrinks in response. It
is also strictly one-sided (only ever lifts a nonzero command toward the
threshold, never floors a genuine zero), so it cannot manufacture a new
zero-crossing the way a symmetric minimum-speed clamp can.

**Mind the failure mode the issue explicitly warns about**: "because the
motor cannot produce velocities between zero and its minimum, naive
compensation can hunt around the target." This fix's own safety argument
(above) is structural, not a substitute for empirical confirmation — the
acceptance criteria below require an actual sim sweep, not just the
algebraic argument.

## Approach

1. **`src/firm/devices/nezha_motor.cpp`**, `writeShapedDuty()`: replace the
   single combined condition with two explicit cases:

   ```cpp
   if (duty == 0.0f) {
       // Exact zero -- a genuine "stop"/"on target" command (STOP, Mode::Neutral,
       // or App::Pilot's own exact-zero twist on completion). Immediate, unclamped,
       // cancels any dwell. NOT boosted -- boosting an intentional zero would make
       // the robot buzz at rest.
       dwelling_ = false;
       lastRequestedDuty_ = 0.0f;
       writeRawDuty(0.0f);
       return;
   }
   if (fabsf(duty) < outputDeadband_) {
       // Genuine nonzero command, but smaller than the plant can actually produce.
       // Boost to the deadband floor (sign-preserving) instead of zeroing it, so a
       // real, wanted correction still moves the wheel. See nezha_motor.h's own
       // file header / sprint 114 Design Rationale Decision 4 for why this does not
       // reproduce the deleted App::Pilot min-speed-floor regression: this sits
       // INSIDE the velocity PID's own closed loop (real measured velocity feeds
       // back every tick), one-sided (never floors a genuine zero).
       duty = std::copysign(outputDeadband_, duty);
   }
   ```

   then fall through into the existing reversal-dwell / same-sign logic
   below, unchanged (the boosted `duty` value now participates in the
   existing dwell/sign-change logic exactly as any other nonzero duty
   would — no special-casing needed there).

2. **Do not touch** `App::Pilot`, `Motion::Executor`, `App::HeadingSource`,
   or any planner-layer code — the fix is entirely inside `NezhaMotor`. If
   you find yourself wanting to touch one of those files, stop — that means
   the fix has drifted from its intended locus; re-read Design Rationale
   Decision 4.

3. **Update `nezha_motor.h`'s file-header comment and `writeShapedDuty()`'s
   own doc comment** to describe the new two-case behavior (they currently
   describe the single combined case being replaced).

## Files to Touch

- `src/firm/devices/nezha_motor.cpp` (`writeShapedDuty()`)
- `src/firm/devices/nezha_motor.h` (doc comments only)

## Acceptance Criteria

- [ ] Exact `duty == 0.0f` still writes 0 immediately, unclamped, cancelling
      any dwell — byte-for-byte unchanged from today.
- [ ] `0 < |duty| < outputDeadband_` writes `std::copysign(outputDeadband_,
      duty)` (i.e. the wheel actually moves) instead of 0.
- [ ] `|duty| >= outputDeadband_` is unaffected (passes through to the
      existing logic unchanged).
- [ ] Reversal-dwell interaction is unchanged for both the exact-zero and
      the boosted-nonzero case (a boosted duty that also happens to be a
      sign reversal still triggers the dwell exactly as an unboosted
      same-magnitude duty would).
- [ ] **Sim system test**: injecting an ~11 mm/s-equivalent terminal heading
      correction (against the shipped ~15 mm/s deadband) produces nonzero
      *measured* wheel velocity within one tick, and the move completes
      inside a bounded time window — not the ~8s arrive-timeout.
- [ ] **Settle-not-hunt sweep** (empirical, not just algebraic): sweep a
      range of small residual errors near the dwell-tolerance boundary
      (e.g. across the model-reference feedback's current gain) and assert
      monotonic convergence into the dwell tolerance with a bounded
      overshoot count (e.g. at most one sign reversal before settling) — a
      sustained oscillation anywhere in the sweep is a failing result, not a
      tuning note.
- [ ] The stakeholder's motion-shape bar (clean trapezoid, no oscillation,
      no end bumps) holds on at least the straight and turn scenarios
      already exercised by the existing tour-closure/behavior-lock suites,
      run against this fix.

## Testing

- **Existing tests to run**: `test_tour_closure_gate.py`,
  `behavior_lock_harness.cpp`, `test_turn_error_characterization.py` —
  re-run against this fix (not yet against the re-baselined `vel_kp=0.002`
  config; that's ticket 006).
- **New tests to write**: targeted `writeShapedDuty()` unit cases
  (exact-zero, sub-deadband boost, above-deadband passthrough, dwell
  interaction with a boosted duty); a new sim system test for the ~11
  mm/s-terminal-correction scenario; the settle-not-hunt sweep test
  (parametrized over a small range of residual errors/gains).
- **Verification command**: `uv run python -m pytest src/tests/sim -v -s`
  (targeted), then full suite `uv run python -m pytest`.
