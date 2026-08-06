---
status: done
sprint: '135'
tickets:
- 135-008
---

# Sim OTOS heading sign diverges from hardware — ANGLE Moves never stop in sim

## Description

**Regression live on master right now** (merge `27dda1ab`, from
`19354c2e` "motion: close the turn loop on the OTOS", 2026-08-05).
`src/tests/sim/system/test_move_protocol.py::test_move_protocol_scenarios_pass`
fails with 2 assertions on an otherwise untouched tree:

```
FAIL [SUC-050 ANGLE: a TWIST MOVE with an ANGLE stop condition turns within
      tolerance of the commanded heading change]:
  kFlagFaultMoveTimeout is NOT set -- ended via ANGLE, not timeout
      -- expected true, got false
  turned within tolerance of the commanded angle (high bound)
      -- expected <= 1.25, got 5.157
```

The scenario commands `omega = 1.0 rad/s` with an ANGLE stop at `1.0 rad` and
a 5000 ms timeout. Measured final heading is **5.157 rad** — i.e. the robot
turned for the entire 5 s timeout at 1 rad/s and the ANGLE stop condition
never fired at all. This is not tolerance drift; the stop condition is
completely inoperative in sim.

Baseline as of 2026-08-05: `src/tests/sim` = 1 failed, 26 passed, 1 xfailed,
1 xpassed. The `planner_tests` ctest suite is unaffected (8/8 green) because
it uses `PerfectPlant`, not the OTOS-bearing `SimPlant`.

## Cause

The firmware negates the OTOS heading before the planner consumes it
(`src/motion/planner/planner.cpp:513`):

```cpp
pose_.applyOtosHeading(-state.otos.heading, state.otos.present, state.otos.connected);
```

Its own comment (planner.cpp:499-513) documents why, and — precisely —
predicts this failure mode:

> NEGATED, and that is not a fudge -- it reconciles a real, pre-existing sign
> inversion. Measured 2026-08-05 on a single commanded rotation: the OTOS
> turned +84.58deg while the encoder-derived heading reported -82.45deg. Same
> magnitude, opposite sign. [...] feeding unnegated optical truth into a
> planner built on the inverted convention makes the Angle stop condition
> count the wrong way and **spin forever**.

The negation is correct for the hardware, where the physical chip's mounted
orientation really is inverted relative to the encoder-derived heading. It is
wrong for the sim, because **the sim's OTOS is not an independently-oriented
sensor — it is computed from the wheels**. `TestSim::OtosPlant`
(`src/tests/sim/plant/otos_plant.h:5-18`) accumulates heading with "the SAME
midpoint-arc update `Odometry::integrate()` already performs — literally the
same three lines (`BodyKinematics::forward()` + cosf/sinf midpoint-arc
accumulation)", and `TestSim::SimPlant` packs that centre-frame `(x, y,
heading)` **directly** into the OTOS chip's raw registers with no transform
(otos_plant.h:25-32, "identity-mounting assumption").

So the sim's OTOS heading carries the *same* sign as the encoder heading by
construction, the hardware's carries the *opposite* sign, and one negation
constant cannot be right for both. In sim the negation makes the planner's
heading run backwards against the commanded turn, the ANGLE threshold is
never reached, and the Move runs to its timeout — exactly what the comment
warned about.

**The wider hazard, which is the real reason this matters:** sim and hardware
now disagree about the sign of optical heading. Any sim verification of
heading-dependent behavior is currently invalid — it validates against a
robot whose OTOS is oriented the opposite way from the real one. That
silently undermines the sim-first verification strategy for anything using
optical heading, including the go-to Navigator
([[replaceable-go-to-moves-in-the-motion-library]], sprint 135), whose pivot
path issues ANGLE Moves and whose world-frame targets are solved against
`state.otos`.

## Proposed fix

Two options. They are not equivalent in cost or blast radius.

### Option A — model the inversion in the sim (recommended, small)

Have `TestSim::SimPlant` pack the OTOS heading register with the hardware's
actual mounted sign (negated relative to its internal centre-frame
accumulator), so the simulated chip reports what the real chip reports. The
firmware's negation then reconciles sim and hardware identically, SUC-050
passes, and sim verification of heading behavior becomes trustworthy again.

- Touches sim only; no wire change, no calibration invalidation.
- Must be a single named constant with a comment tying it to
  planner.cpp:513, so the two flip together when Option B eventually lands.
- Verify: `uv run python -m pytest src/tests/sim` returns fully green
  (27 passed), and a sim ANGLE Move of 1.0 rad ends via the ANGLE stop with
  `kFlagFaultMoveTimeout` clear.

### Option B — fix the body-kinematics omega sign (the right fix, deferred)

planner.cpp:509-512 names this itself:

> The RIGHT fix is the body kinematics' omega sign, so commanded omega and
> world CCW finally agree. That is deliberately not done here: it changes the
> meaning of omega on the wire and invalidates both stored per-direction
> rotation calibrations, so it needs its own change with the bench free.

This would delete the negation, delete every host-side `YAW_SIGN = -1`, and
require re-measuring the per-direction rotation calibrations on hardware.
It is a wire-semantics change and deserves its own sprint. **Stakeholder
decision required** — note the bench IS currently free (tovez on the stand,
direct serial, `getez` relay attached), so the stated precondition is
satisfied whenever it is wanted.

Recommendation: take Option A now to unblock sim verification, and schedule
Option B as its own sprint rather than letting a second sign constant
accumulate.

## Verification

- `uv run python -m pytest src/tests/sim` — fully green (currently 1 failed,
  26 passed, 1 xfailed, 1 xpassed).
- SUC-050 specifically: a 1.0 rad ANGLE Move ends via the ANGLE stop
  condition with `kFlagFaultMoveTimeout` clear and a final heading inside
  1.0 ± 0.25 rad.
- `planner_tests` ctest suite stays 8/8 green (it should be untouched).
- Hardware cross-check on `tovez` (by UID, direct serial): a commanded
  1.0 rad turn lands within the same tolerance the sim now asserts, so the
  two tiers are demonstrably measuring the same convention.

## Related

- [[replaceable-go-to-moves-in-the-motion-library]] — sprint 135; its
  Navigator sim-tests ANGLE pivots and solves world targets against
  `state.otos`, so this blocks that sprint's verification tier being
  meaningful. Its landmine list already flags the omega-sign hazard
  ("keep one constant, one comment, one flip").
- `src/motion/planner/planner.cpp:490-513` — the negation and its rationale.
- `src/tests/sim/plant/otos_plant.h:5-32` — the sim's wheel-derived,
  identity-mounted OTOS.
- `src/tests/sim/system/move_protocol_harness.cpp:365-438` — the failing
  scenario.
- Commit `19354c2e` (merged as `27dda1ab`) — where the negation landed.
